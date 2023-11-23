/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * The velocity pass outputs motion vectors to use for either
 * temporal re-projection or motion blur.
 *
 * It is the module that tracks the objects between frames updates.
 */

#include "BKE_duplilist.h"
#include "BKE_object.hh"
#include "BLI_map.hh"
#include "DEG_depsgraph_query.hh"
#include "DNA_modifier_types.h"
#include "DNA_particle_types.h"
#include "DNA_rigidbody_types.h"

#include "draw_cache_impl.hh"

#include "eevee_instance.hh"
// #include "eevee_renderpasses.hh"
#include "eevee_shader.hh"
#include "eevee_shader_shared.hh"
#include "eevee_velocity.hh"

#include "draw_common.hh"

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name VelocityModule
 *
 * \{ */

void VelocityModule::init()
{
  if (!inst_.is_viewport() && (inst_.film.enabled_passes_get() & EEVEE_RENDER_PASS_VECTOR) &&
      !inst_.motion_blur.postfx_enabled())
  {
    /* No motion blur and the vector pass was requested. Do the steps sync here. */
    const Scene *scene = inst_.scene;
    float initial_time = scene->r.cfra + scene->r.subframe;
    step_sync(STEP_PREVIOUS, initial_time - 1.0f);
    step_sync(STEP_NEXT, initial_time + 1.0f);
    /* Let the main sync loop handle the current step. */
    inst_.set_time(initial_time);
    step_ = STEP_CURRENT;
  }

  /* For viewport, only previous motion is supported.
   * Still bind previous step to avoid undefined behavior. */
  next_step_ = inst_.is_viewport() ? STEP_PREVIOUS : STEP_NEXT;
}

/* Similar to Instance::object_sync, but only syncs velocity. */
static void step_object_sync_render(void *instance,
                                    Object *ob,
                                    RenderEngine * /*engine*/,
                                    Depsgraph * /*depsgraph*/)
{
  Instance &inst = *reinterpret_cast<Instance *>(instance);

  const bool is_velocity_type = ELEM(
      ob->type, OB_CURVES, OB_GPENCIL_LEGACY, OB_MESH, OB_POINTCLOUD);
  const int ob_visibility = DRW_object_visibility_in_active_context(ob);
  const bool partsys_is_visible = (ob_visibility & OB_VISIBLE_PARTICLES) != 0 &&
                                  (ob->type == OB_MESH);
  const bool object_is_visible = DRW_object_is_renderable(ob) &&
                                 (ob_visibility & OB_VISIBLE_SELF) != 0;

  if (!is_velocity_type || (!partsys_is_visible && !object_is_visible)) {
    return;
  }

  /* NOTE: Dummy resource handle since this won't be used for drawing. */
  ResourceHandle resource_handle(0);
  ObjectHandle &ob_handle = inst.sync.sync_object(ob);

  if (partsys_is_visible) {
    auto sync_hair =
        [&](ObjectHandle hair_handle, ModifierData &md, ParticleSystem &particle_sys) {
          inst.velocity.step_object_sync(
              ob, hair_handle.object_key, resource_handle, hair_handle.recalc, &md, &particle_sys);
        };
    foreach_hair_particle_handle(ob, ob_handle, sync_hair);
  };

  if (object_is_visible) {
    inst.velocity.step_object_sync(ob, ob_handle.object_key, resource_handle, ob_handle.recalc);
  }

  ob_handle.reset_recalc_flag();
}

void VelocityModule::step_sync(eVelocityStep step, float time)
{
  inst_.set_time(time);
  step_ = step;
  object_steps_usage[step_] = 0;
  step_camera_sync();

  draw::hair_init();
  draw::curves_init();

  DRW_render_object_iter(&inst_, inst_.render, inst_.depsgraph, step_object_sync_render);

  draw::hair_update(*inst_.manager);
  draw::curves_update(*inst_.manager);
  draw::hair_free();
  draw::curves_free();

  geometry_steps_fill();
}

void VelocityModule::step_camera_sync()
{
  inst_.camera.sync();
  *camera_steps[step_] = inst_.camera.data_get();
  step_time[step_] = inst_.scene->r.cfra + inst_.scene->r.subframe;
  /* Fix undefined camera steps when rendering is starting. */
  if ((step_ == STEP_CURRENT) && (camera_steps[STEP_PREVIOUS]->initialized == false)) {
    *camera_steps[STEP_PREVIOUS] = *static_cast<CameraData *>(camera_steps[step_]);
    camera_steps[STEP_PREVIOUS]->initialized = true;
    step_time[STEP_PREVIOUS] = step_time[step_];
  }
}

bool VelocityModule::step_object_sync(Object *ob,
                                      ObjectKey &object_key,
                                      ResourceHandle resource_handle,
                                      int /*IDRecalcFlag*/ recalc,
                                      ModifierData *modifier_data /*=nullptr*/,
                                      ParticleSystem *particle_sys /*=nullptr*/)
{
  bool has_motion = object_has_velocity(ob) || (recalc & ID_RECALC_TRANSFORM);
  /* NOTE: Fragile. This will only work with 1 frame of lag since we can't record every geometry
   * just in case there might be an update the next frame. */
  bool has_deform = object_is_deform(ob) || (recalc & ID_RECALC_GEOMETRY);

  if (!has_motion && !has_deform) {
    return false;
  }

  /* Object motion. */
  /* FIXME(fclem) As we are using original objects pointers, there is a chance the previous
   * object key matches a totally different object if the scene was changed by user or python
   * callback. In this case, we cannot correctly match objects between updates.
   * What this means is that there will be incorrect motion vectors for these objects.
   * We live with that until we have a correct way of identifying new objects. */
  VelocityObjectData &vel = velocity_map.lookup_or_add_default(object_key);
  vel.obj.ofs[step_] = object_steps_usage[step_]++;
  vel.obj.resource_id = resource_handle.resource_index();
  vel.id = object_key.hash();
  object_steps[step_]->get_or_resize(vel.obj.ofs[step_]) = float4x4_view(ob->object_to_world);
  if (step_ == STEP_CURRENT) {
    /* Replace invalid steps. Can happen if object was hidden in one of those steps. */
    if (vel.obj.ofs[STEP_PREVIOUS] == -1) {
      vel.obj.ofs[STEP_PREVIOUS] = object_steps_usage[STEP_PREVIOUS]++;
      object_steps[STEP_PREVIOUS]->get_or_resize(vel.obj.ofs[STEP_PREVIOUS]) = float4x4_view(
          ob->object_to_world);
    }
    if (vel.obj.ofs[STEP_NEXT] == -1) {
      vel.obj.ofs[STEP_NEXT] = object_steps_usage[STEP_NEXT]++;
      object_steps[STEP_NEXT]->get_or_resize(vel.obj.ofs[STEP_NEXT]) = float4x4_view(
          ob->object_to_world);
    }
  }

  /* Geometry motion. */
  if (has_deform) {
    auto add_cb = [&]() {
      VelocityGeometryData data;
      if (particle_sys) {
        if (inst_.is_viewport()) {
          data.pos_buf = DRW_hair_pos_buffer_get(ob, particle_sys, modifier_data);
        }
        else {
          data.pos_buf = draw::hair_pos_buffer_get(inst_.scene, ob, particle_sys, modifier_data);
        }
        return data;
      }
      switch (ob->type) {
        case OB_CURVES:
          if (inst_.is_viewport()) {
            data.pos_buf = DRW_curves_pos_buffer_get(ob);
          }
          else {
            data.pos_buf = draw::curves_pos_buffer_get(inst_.scene, ob);
          }
          break;
        case OB_POINTCLOUD:
          data.pos_buf = DRW_pointcloud_position_and_radius_buffer_get(ob);
          break;
        default:
          data.pos_buf = DRW_cache_object_pos_vertbuf_get(ob);
          break;
      }
      return data;
    };

    const VelocityGeometryData &data = geometry_map.lookup_or_add_cb(vel.id, add_cb);

    if (data.pos_buf == nullptr) {
      has_deform = false;
    }
  }

  /* Avoid drawing object that has no motions but were tagged as such. */
  if (step_ == STEP_CURRENT && has_motion == true && has_deform == false) {
    float4x4 &obmat_curr = (*object_steps[STEP_CURRENT])[vel.obj.ofs[STEP_CURRENT]];
    float4x4 &obmat_prev = (*object_steps[STEP_PREVIOUS])[vel.obj.ofs[STEP_PREVIOUS]];
    float4x4 &obmat_next = (*object_steps[STEP_NEXT])[vel.obj.ofs[STEP_NEXT]];
    if (inst_.is_viewport()) {
      has_motion = (obmat_curr != obmat_prev);
    }
    else {
      has_motion = (obmat_curr != obmat_prev || obmat_curr != obmat_next);
    }
  }

#if 0
  if (!has_motion && !has_deform) {
    std::cout << "Detected no motion on " << ob->id.name << std::endl;
  }
  if (has_deform) {
    std::cout << "Geometry Motion on " << ob->id.name << std::endl;
  }
  if (has_motion) {
    std::cout << "Object Motion on " << ob->id.name << std::endl;
  }
#endif

  if (!has_motion && !has_deform) {
    return false;
  }

  /* TODO(@fclem): Reset sampling here? Should ultimately be covered by depsgraph update tags. */
  /* NOTE(Miguel Pozo): Disable, since is_deform is always true for objects with particle
   * modifiers, and this causes the renderer to get stuck at sample 1. */
  // inst_.sampling.reset();

  return true;
}

void VelocityModule::geometry_steps_fill()
{
  uint dst_ofs = 0;
  for (VelocityGeometryData &geom : geometry_map.values()) {
    uint src_len = GPU_vertbuf_get_vertex_len(geom.pos_buf);
    geom.len = src_len;
    geom.ofs = dst_ofs;
    dst_ofs += src_len;
  }
  /* TODO(@fclem): Fail gracefully (disable motion blur + warning print) if
   * `tot_len * sizeof(float4)` is greater than max SSBO size. */
  geometry_steps[step_]->resize(max_ii(16, dst_ofs));

  for (VelocityGeometryData &geom : geometry_map.values()) {
    GPU_storagebuf_copy_sub_from_vertbuf(*geometry_steps[step_],
                                         geom.pos_buf,
                                         geom.ofs * sizeof(float4),
                                         0,
                                         geom.len * sizeof(float4));
  }
  /* Copy back the #VelocityGeometryIndex into #VelocityObjectData which are
   * indexed using persistent keys (unlike geometries which are indexed by volatile ID). */
  for (VelocityObjectData &vel : velocity_map.values()) {
    const VelocityGeometryData &geom = geometry_map.lookup_default(vel.id, VelocityGeometryData());
    vel.geo.len[step_] = geom.len;
    vel.geo.ofs[step_] = geom.ofs;
    /* Avoid reuse. */
    vel.id = 0;
  }

  geometry_map.clear();
}

/**
 * In Render, moves the next frame data to previous frame data. Nullify next frame data.
 * In Viewport, the current frame data will be used as previous frame data in the next frame.
 */
void VelocityModule::step_swap()
{
  auto swap_steps = [&](eVelocityStep step_a, eVelocityStep step_b) {
    std::swap(object_steps[step_a], object_steps[step_b]);
    std::swap(geometry_steps[step_a], geometry_steps[step_b]);
    std::swap(camera_steps[step_a], camera_steps[step_b]);
    std::swap(step_time[step_a], step_time[step_b]);

    for (VelocityObjectData &vel : velocity_map.values()) {
      vel.obj.ofs[step_a] = vel.obj.ofs[step_b];
      vel.obj.ofs[step_b] = uint(-1);
      vel.geo.ofs[step_a] = vel.geo.ofs[step_b];
      vel.geo.len[step_a] = vel.geo.len[step_b];
      vel.geo.ofs[step_b] = uint(-1);
      vel.geo.len[step_b] = uint(-1);
    }
  };

  if (inst_.is_viewport()) {
    geometry_steps_fill();
    /* For viewport we only use the last rendered redraw as previous frame.
     * We swap current with previous step at the end of a redraw.
     * We do not support motion blur as it is rendered to avoid conflicting motions
     * for temporal reprojection. */
    swap_steps(eVelocityStep::STEP_PREVIOUS, eVelocityStep::STEP_CURRENT);
  }
  else {
    /* Render case: The STEP_CURRENT is left untouched. */
    swap_steps(eVelocityStep::STEP_PREVIOUS, eVelocityStep::STEP_NEXT);
  }
}

void VelocityModule::begin_sync()
{
  step_ = STEP_CURRENT;
  step_camera_sync();
  object_steps_usage[step_] = 0;
}

/* This is the end of the current frame sync. Not the step_sync. */
void VelocityModule::end_sync()
{
  Vector<ObjectKey, 0> deleted_obj;

  uint32_t max_resource_id_ = 0u;

  for (MapItem<ObjectKey, VelocityObjectData> item : velocity_map.items()) {
    if (item.value.obj.resource_id == uint32_t(-1)) {
      deleted_obj.append(item.key);
    }
    else {
      max_resource_id_ = max_uu(max_resource_id_, item.value.obj.resource_id);
    }
  }

  if (deleted_obj.size() > 0) {
    inst_.sampling.reset();
  }

  if (inst_.is_viewport() && camera_has_motion()) {
    inst_.sampling.reset();
  }

  for (auto &key : deleted_obj) {
    velocity_map.remove(key);
  }

  indirection_buf.resize(ceil_to_multiple_u(max_resource_id_ + 1, 128));

  /* Avoid uploading more data to the GPU as well as an extra level of
   * indirection on the GPU by copying back offsets the to VelocityIndex. */
  for (VelocityObjectData &vel : velocity_map.values()) {
    /* Disable deform if vertex count mismatch. */
    if (inst_.is_viewport()) {
      /* Current geometry step will be copied at the end of the frame.
       * Thus vel.geo.len[STEP_CURRENT] is not yet valid and the current length is manually
       * retrieved. */
      GPUVertBuf *pos_buf = geometry_map.lookup_default(vel.id, VelocityGeometryData()).pos_buf;
      vel.geo.do_deform = pos_buf != nullptr &&
                          (vel.geo.len[STEP_PREVIOUS] == GPU_vertbuf_get_vertex_len(pos_buf));
    }
    else {
      vel.geo.do_deform = (vel.geo.len[STEP_CURRENT] != 0) &&
                          (vel.geo.len[STEP_CURRENT] == vel.geo.len[STEP_PREVIOUS]) &&
                          (vel.geo.len[STEP_CURRENT] == vel.geo.len[STEP_NEXT]);
    }
    indirection_buf[vel.obj.resource_id] = vel;
    /* Reset for next sync. */
    vel.obj.resource_id = uint(-1);
  }

  object_steps[STEP_PREVIOUS]->push_update();
  object_steps[STEP_NEXT]->push_update();
  camera_steps[STEP_PREVIOUS]->push_update();
  camera_steps[STEP_CURRENT]->push_update();
  camera_steps[STEP_NEXT]->push_update();
  indirection_buf.push_update();
}

bool VelocityModule::object_has_velocity(const Object *ob)
{
#if 0
  RigidBodyOb *rbo = ob->rigidbody_object;
  /* Active rigidbody objects only, as only those are affected by sim. */
  const bool has_rigidbody = (rbo && (rbo->type == RBO_TYPE_ACTIVE));
  /* For now we assume dupli objects are moving. */
  const bool is_dupli = (ob->base_flag & BASE_FROM_DUPLI) != 0;
  const bool object_moves = is_dupli || has_rigidbody || BKE_object_moves_in_time(ob, true);
#else
  UNUSED_VARS(ob);
  /* BKE_object_moves_in_time does not work in some cases.
   * Better detect non moving object after evaluation. */
  const bool object_moves = true;
#endif
  return object_moves;
}

bool VelocityModule::object_is_deform(const Object *ob)
{
  RigidBodyOb *rbo = ob->rigidbody_object;
  /* Active rigidbody objects only, as only those are affected by sim. */
  const bool has_rigidbody = (rbo && (rbo->type == RBO_TYPE_ACTIVE));
  const bool is_deform = BKE_object_is_deform_modified(inst_.scene, (Object *)ob) ||
                         (has_rigidbody && (rbo->flag & RBO_FLAG_USE_DEFORM) != 0);

  return is_deform;
}

void VelocityModule::bind_resources(DRWShadingGroup *grp)
{
  /* For viewport, only previous motion is supported.
   * Still bind previous step to avoid undefined behavior. */
  eVelocityStep next = inst_.is_viewport() ? STEP_PREVIOUS : STEP_NEXT;
  DRW_shgroup_storage_block_ref(grp, "velocity_obj_prev_buf", &(*object_steps[STEP_PREVIOUS]));
  DRW_shgroup_storage_block_ref(grp, "velocity_obj_next_buf", &(*object_steps[next]));
  DRW_shgroup_storage_block_ref(grp, "velocity_geo_prev_buf", &(*geometry_steps[STEP_PREVIOUS]));
  DRW_shgroup_storage_block_ref(grp, "velocity_geo_next_buf", &(*geometry_steps[next]));
  DRW_shgroup_uniform_block_ref(grp, "camera_prev", &(*camera_steps[STEP_PREVIOUS]));
  DRW_shgroup_uniform_block_ref(grp, "camera_curr", &(*camera_steps[STEP_CURRENT]));
  DRW_shgroup_uniform_block_ref(grp, "camera_next", &(*camera_steps[next]));
  DRW_shgroup_storage_block_ref(grp, "velocity_indirection_buf", &indirection_buf);
}

bool VelocityModule::camera_has_motion() const
{
  /* Only valid after sync. */
  if (inst_.is_viewport()) {
    /* Viewport has no next step. */
    return *camera_steps[STEP_PREVIOUS] != *camera_steps[STEP_CURRENT];
  }
  return *camera_steps[STEP_PREVIOUS] != *camera_steps[STEP_CURRENT] &&
         *camera_steps[STEP_NEXT] != *camera_steps[STEP_CURRENT];
}

bool VelocityModule::camera_changed_projection() const
{
  /* Only valid after sync. */
  if (inst_.is_viewport()) {
    return camera_steps[STEP_PREVIOUS]->type != camera_steps[STEP_CURRENT]->type;
  }
  /* Cannot happen in render mode since we set the type during the init phase. */
  return false;
}

float VelocityModule::step_time_delta_get(eVelocityStep start, eVelocityStep end) const
{
  return step_time[end] - step_time[start];
}

/** \} */

}  // namespace blender::eevee
