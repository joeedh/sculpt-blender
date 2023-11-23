/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2009 by Nicholas Bishop. All rights reserved. */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BKE_attribute.h"
#include "BKE_paint.hh"
#include "BKE_pbvh.hh"

#include "BLI_compiler_compat.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include <type_traits>

struct Object;

/*
 * Stroke ID API.  This API is used to detect if
 * an element has already been processed for some task
 * inside a given stroke.
 */

struct StrokeID {
  short id;
  short userflag;
};

enum StrokeIDUser {
  STROKEID_USER_AUTOMASKING = 1 << 0,
  STROKEID_USER_BOUNDARY = 1 << 1,
  STROKEID_USER_SCULPTVERT = 1 << 2,
  STROKEID_USER_PREV_COLOR = 1 << 3,
  STROKEID_USER_SMOOTH = 1 << 4,
  STROKEID_USER_OCCLUSION = 1 << 5,
  STROKEID_USER_LAYER_BRUSH = 1 << 6,
  STROKEID_USER_ORIGINAL = 1 << 7,
};
ENUM_OPERATORS(StrokeIDUser, STROKEID_USER_LAYER_BRUSH);

void BKE_sculpt_reproject_cdata(SculptSession *ss,
                                PBVHVertRef vertex,
                                float startco[3],
                                float startno[3],
                                eAttrCorrectMode undistort_mode);

namespace blender::bke::sculpt {
void sculpt_vert_boundary_ensure(Object *ob);

BLI_INLINE bool stroke_id_clear(SculptSession *ss, PBVHVertRef vertex, StrokeIDUser user)
{
  StrokeID *id = blender::bke::paint::vertex_attr_ptr<StrokeID>(vertex, ss->attrs.stroke_id);

  bool retval = id->userflag & user;
  id->userflag &= ~user;

  return retval;
}

BLI_INLINE bool stroke_id_test(SculptSession *ss, PBVHVertRef vertex, StrokeIDUser user)
{
  StrokeID *id = blender::bke::paint::vertex_attr_ptr<StrokeID>(vertex, ss->attrs.stroke_id);
  bool ret;

  if (id->id != ss->stroke_id) {
    id->id = ss->stroke_id;
    id->userflag = 0;
    ret = true;
  }
  else {
    ret = !(id->userflag & (int)user);
  }

  id->userflag |= (int)user;

  return ret;
}

BLI_INLINE bool stroke_id_test_no_update(SculptSession *ss, PBVHVertRef vertex, StrokeIDUser user)
{
  StrokeID *id = blender::bke::paint::vertex_attr_ptr<StrokeID>(vertex, ss->attrs.stroke_id);

  if (id->id != ss->stroke_id) {
    return true;
  }

  return !(id->userflag & (int)user);
}

BLI_INLINE void add_sculpt_flag(SculptSession *ss, PBVHVertRef vertex, uint8_t flag)
{
  *blender::bke::paint::vertex_attr_ptr<uint8_t>(vertex, ss->attrs.flags) |= flag;
}
BLI_INLINE void clear_sculpt_flag(SculptSession *ss, PBVHVertRef vertex, uint8_t flag)
{
  *blender::bke::paint::vertex_attr_ptr<uint8_t>(vertex, ss->attrs.flags) &= ~flag;
}
BLI_INLINE bool test_sculpt_flag(SculptSession *ss, PBVHVertRef vertex, uint8_t flag)
{
  return blender::bke::paint::vertex_attr_get<uint8_t>(vertex, ss->attrs.flags) & flag;
}

/* Interpolates loops surrounding a vertex, splitting any UV map by
 * island as appropriate and enforcing proper boundary conditions.
 */
void interp_face_corners(PBVH *pbvh,
                         PBVHVertRef vertex,
                         Span<BMLoop *> loops,
                         Span<float> ws,
                         float factor,
                         int cd_vert_boundary);
float calc_uv_snap_limit(BMLoop *l, int cd_uv);
bool loop_is_corner(BMLoop *l, int cd_uv, float limit = 0.01, const CustomData *ldata = nullptr);

/* Finds sets of loops with the same vertex data
 * prior to an operation, then re-snaps them afterwards.
 */
struct VertLoopSnapper {
  Vector<Vector<short>, 16> snap_sets;
  Span<CustomDataLayer *> layers;
  Span<BMLoop *> ls;
  Vector<int, 16> max_indices;
  float limit = 0.001;

  VertLoopSnapper(Span<BMLoop *> ls_, Span<CustomDataLayer *> layers_) : layers(layers_), ls(ls_)
  {
    if (ls.size() == 0) {
      return;
    }

    snap_sets.resize(ls.size());
    for (auto &snap_set : snap_sets) {
      for (int i = 0; i < layers.size(); i++) {
        snap_set.append(0);
      }
    }

    for (int i : layers.index_range()) {
      switch (layers[i]->type) {
        case CD_PROP_FLOAT:
          begin<VecBase<float, 1>>(i);
          break;
        case CD_PROP_FLOAT2:
          begin<float2>(i);
          break;
        case CD_PROP_FLOAT3:
          begin<float3>(i);
          break;
        case CD_PROP_COLOR:
          begin<float4>(i);
          break;
      }
    }
  }

  void snap()
  {
    for (int i : layers.index_range()) {
      switch (layers[i]->type) {
        case CD_PROP_FLOAT:
          do_snap<VecBase<float, 1>>(i);
          break;
        case CD_PROP_FLOAT2:
          do_snap<float2>(i);
          break;
        case CD_PROP_FLOAT3:
          do_snap<float3>(i);
          break;
        case CD_PROP_COLOR:
          do_snap<float4>(i);
          break;
      }
    }
  }

 private:
  template<typename T> void begin(int layer_i)
  {
    CustomDataLayer *layer = layers[layer_i];
    int idx_base = 1;

    limit = 0.001f;

    if constexpr (std::is_same_v<T, float2>) {
      /* Set UV snap limit as 1/10th the average uv edge length. */
      limit = 0.1f;
      float len = 0.0f;

      for (BMLoop *l : ls) {
        T value1 = *BM_ELEM_CD_PTR<T *>(l, layer->offset);
        T value2 = *BM_ELEM_CD_PTR<T *>(l->next, layer->offset);

        len += fabsf(value1[0] - value2[0]) * 0.5f;
        len += fabsf(value1[1] - value2[1]) * 0.5f;
      }

      len /= ls.size();
      limit *= len;
    }

    for (int i : ls.index_range()) {
      if (snap_sets[i][layer_i] != 0) {
        continue;
      }

      T a = *BM_ELEM_CD_PTR<T *>(ls[i], layer->offset);
      int set = snap_sets[i][layer_i] = idx_base++;

      for (int j : ls.index_range()) {
        if (snap_sets[j][layer_i] != 0) {
          continue;
        }

        T b = *BM_ELEM_CD_PTR<T *>(ls[j], layer->offset);
        if (math::distance_squared(a, b) <= limit * limit) {
          snap_sets[j][layer_i] = set;
        }
      }
    }

    max_indices.append(idx_base);
  }

  template<typename T> void do_snap(int layer_i)
  {
    const int cd_offset = layers[layer_i]->offset;

    for (int set_i : IndexRange(max_indices[layer_i])) {
      T sum = {};
      float tot = 0.0f;

      for (int i : ls.index_range()) {
        if (snap_sets[i][layer_i] == set_i) {
          sum += *BM_ELEM_CD_PTR<T *>(ls[i], cd_offset);
          tot += 1.0f;
        }
      }

      if (tot == 0.0f) {
        continue;
      }

      sum /= tot;

      for (int i : ls.index_range()) {
        if (snap_sets[i][layer_i] == set_i) {
          *BM_ELEM_CD_PTR<T *>(ls[i], cd_offset) = sum;
        }
      }
    }
  }
};
}  // namespace blender::bke::sculpt

/* Uncomment to enable PBVH NaN debugging. */
//#define PBVH_CHECK_NANS

#ifdef PBVH_CHECK_NANS
#  include "atomic_ops.h"
#  include <float.h>
#  include <math.h>

/* Why is atomic_ops defining near & far macros? */
#  ifdef near
#    undef near
#  endif
#  ifdef far
#    undef far
#  endif

// static global to limit the number of reports per source file
static int _bke_pbvh_report_count = 0;

#  define PBVH_NAN_REPORT_LIMIT 16

// for debugging NaNs that don't appear on developer's machines
static ATTR_NO_OPT bool _pbvh_nan_check1(const float f,
                                         const char *func,
                                         const char *file,
                                         int line)
{
  bool bad = false;

  if (_bke_pbvh_report_count > PBVH_NAN_REPORT_LIMIT) {
    return false;
  }

  if (isnan(f) || !isfinite(f)) {
    const char *type = !isfinite(f) ? "infinity" : "nan";
    printf("float corruption (value was %s): %s:%d\n\t%s\n", type, func, line, file);
    bad = true;
  }

  if (bad) {
    atomic_add_and_fetch_int32(&_bke_pbvh_report_count, 1);
  }

  return bad;
}

static ATTR_NO_OPT bool _pbvh_nan_check3(const float co[3],
                                         const char *func,
                                         const char *file,
                                         int line)
{
  if (!co) {
    return false;
  }

  bool ret = false;

  for (int i = 0; i < 3; i++) {
    ret |= _pbvh_nan_check1(co[i], func, file, line);
  }

  return ret;
}

static ATTR_NO_OPT bool _pbvh_nan_check4(const float co[4],
                                         const char *func,
                                         const char *file,
                                         int line)
{
  if (!co) {
    return false;
  }

  bool ret = false;

  for (int i = 0; i < 4; i++) {
    ret |= _pbvh_nan_check1(co[i], func, file, line);
  }

  return ret;
}

#  define PBVH_CHECK_NAN(co) _pbvh_nan_check3(co, __func__, __FILE__, __LINE__)
#  define PBVH_CHECK_NAN1(f) _pbvh_nan_check1(f, __func__, __FILE__, __LINE__)
#  define PBVH_CHECK_NAN4(co) _pbvh_nan_check4(co, __func__, __FILE__, __LINE__)
#else
#  define PBVH_CHECK_NAN(co)
#  define PBVH_CHECK_NAN1(f)
#  define PBVH_CHECK_NAN4(co)
#endif
