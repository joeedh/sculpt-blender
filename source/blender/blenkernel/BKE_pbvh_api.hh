/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief A BVH for high poly meshes.
 */

#include "BKE_dyntopo_set.hh"

#include "BLI_bitmap.h"
#include "BLI_compiler_compat.h"
#include "BLI_ghash.h"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_vector.hh"

#include "DNA_brush_enums.h" /* for eAttrCorrectMode */
#include "DNA_customdata_types.h"

/* For embedding CCGKey in iterator. */
#include "BKE_attribute.h"
#include "BKE_ccg.h"
#include "BKE_pbvh.h"

#include "bmesh.h"
#include "bmesh_log.h"

#include <stdint.h>

struct BMesh;
struct BMVert;
struct BMEdge;
struct BMFace;
struct BMIdMap;
struct Scene;
struct CCGElem;
struct CCGKey;
struct CustomData;
struct DMFlagMat;
struct IsectRayPrecalc;
struct MLoopTri;
struct Mesh;
struct PBVH;
struct PBVHNode;
struct PBVHBatches;
struct PBVH_GPU_Args;
struct SculptSession;
struct SubdivCCG;
struct TaskParallelSettings;
struct Image;
struct ImageUser;

struct PBVHTri {
  int v[3];      /* References into PBVHTriBuf->verts. */
  int eflag;     /* Bitmask of which edges in the tri are real edges in the mesh. */
  intptr_t l[3]; /* Loops, currently just BMLoop pointers for now. */
  PBVHFaceRef f;
  float no[3];
};

struct PBVHTriBuf {
  blender::Vector<PBVHTri> tris;
  blender::Vector<PBVHVertRef> verts;
  blender::Vector<int> edges;
  blender::Vector<uintptr_t> loops;

  int mat_nr = 0;

  float min[3], max[3];
};

/*
 * These structs represent logical verts/edges/faces.
 * for PBVH_GRIDS and PBVH_FACES they store integer
 * offsets, PBVH_BMESH stores pointers.
 *
 * The idea is to enforce stronger type checking by encapsulating
 * intptr_t's in structs.
 */

/* A generic PBVH vertex.
 *
 * NOTE: in PBVH_GRIDS we consider the final grid points
 * to be vertices.  This is not true of edges or faces which are pulled from
 * the base mesh.
 */

struct PBVHProxyNode {
  float (*co)[3];
};

struct PBVHColorBufferNode {
  float (*color)[4];
  int size;
};

struct PBVHPixels {
  /**
   * Storage for texture painting on PBVH level.
   *
   * Contains #blender::bke::pbvh::pixels::PBVHData
   */
  void *data;
};

struct PBVHPixelsNode {
  /**
   * Contains triangle/pixel data used during texture painting.
   *
   * Contains #blender::bke::pbvh::pixels::NodeData.
   */
  void *node_data;
};

struct PBVHAttrReq {
  char name[MAX_CUSTOMDATA_LAYER_NAME];
  eAttrDomain domain;
  eCustomDataType type;
};

struct PBVHFrustumPlanes {
  float (*planes)[4];
  int num_planes;
};

BLI_INLINE BMesh *BKE_pbvh_get_bmesh(PBVH *pbvh)
{
  return ((struct PBVHPublic *)pbvh)->bm;
}

BLI_INLINE PBVHVertRef BKE_pbvh_make_vref(intptr_t i)
{
  PBVHVertRef ret = {i};
  return ret;
}

BLI_INLINE PBVHEdgeRef BKE_pbvh_make_eref(intptr_t i)
{
  PBVHEdgeRef ret = {i};
  return ret;
}

BLI_INLINE PBVHFaceRef BKE_pbvh_make_fref(intptr_t i)
{
  PBVHFaceRef ret = {i};
  return ret;
}

BLI_INLINE int BKE_pbvh_vertex_to_index(PBVH *pbvh, PBVHVertRef v)
{
  return (BKE_pbvh_type(pbvh) == PBVH_BMESH && v.i != PBVH_REF_NONE ?
              BM_elem_index_get((BMVert *)(v.i)) :
              (v.i));
}

BLI_INLINE PBVHVertRef BKE_pbvh_index_to_vertex(PBVH *pbvh, int index)
{
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES:
    case PBVH_GRIDS:
      return BKE_pbvh_make_vref(index);
    case PBVH_BMESH:
      return BKE_pbvh_make_vref((intptr_t)BKE_pbvh_get_bmesh(pbvh)->vtable[index]);
  }

  return BKE_pbvh_make_vref(PBVH_REF_NONE);
}

BLI_INLINE int BKE_pbvh_edge_to_index(PBVH *pbvh, PBVHEdgeRef e)
{
  return (BKE_pbvh_type(pbvh) == PBVH_BMESH && e.i != PBVH_REF_NONE ?
              BM_elem_index_get((BMEdge *)(e.i)) :
              (e.i));
}

BLI_INLINE PBVHEdgeRef BKE_pbvh_index_to_edge(PBVH *pbvh, int index)
{
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES:
    case PBVH_GRIDS:
      return BKE_pbvh_make_eref(index);
    case PBVH_BMESH:
      return BKE_pbvh_make_eref((intptr_t)BKE_pbvh_get_bmesh(pbvh)->etable[index]);
  }

  return BKE_pbvh_make_eref(PBVH_REF_NONE);
}

BLI_INLINE int BKE_pbvh_face_to_index(PBVH *pbvh, PBVHFaceRef f)
{
  return (BKE_pbvh_type(pbvh) == PBVH_BMESH && f.i != PBVH_REF_NONE ?
              BM_elem_index_get((BMFace *)(f.i)) :
              (f.i));
}

BLI_INLINE PBVHFaceRef BKE_pbvh_index_to_face(PBVH *pbvh, int index)
{
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES:
    case PBVH_GRIDS:
      return BKE_pbvh_make_fref(index);
    case PBVH_BMESH:
      return BKE_pbvh_make_fref((intptr_t)BKE_pbvh_get_bmesh(pbvh)->ftable[index]);
  }

  return BKE_pbvh_make_fref(PBVH_REF_NONE);
}

/* Callbacks */

/**
 * Returns true if the search should continue from this node, false otherwise.
 */
typedef bool (*BKE_pbvh_SearchCallback)(PBVHNode *node, void *data);

typedef void (*BKE_pbvh_HitCallback)(PBVHNode *node, void *data);
typedef void (*BKE_pbvh_HitOccludedCallback)(PBVHNode *node, void *data, float *tmin);

typedef void (*BKE_pbvh_SearchNearestCallback)(PBVHNode *node, void *data, float *tmin);

PBVHNode *BKE_pbvh_get_node(PBVH *pbvh, int node);

/* Building */

PBVH *BKE_pbvh_new(PBVHType type);

/**
 * Do a full rebuild with on Mesh data structure.
 */
void BKE_pbvh_build_mesh(PBVH *pbvh, Mesh *mesh);
void BKE_pbvh_update_mesh_pointers(PBVH *pbvh, Mesh *mesh);

/**
 * Do a full rebuild with on Grids data structure.
 */
void BKE_pbvh_build_grids(PBVH *pbvh,
                          CCGElem **grids,
                          int totgrid,
                          CCGKey *key,
                          void **gridfaces,
                          DMFlagMat *flagmats,
                          unsigned int **grid_hidden,
                          float *face_areas,
                          Mesh *me,
                          SubdivCCG *subdiv_ccg);
/**
 * Build a PBVH from a BMesh.
 */
void BKE_pbvh_build_bmesh(PBVH *pbvh,
                          Mesh *me,
                          BMesh *bm,
                          BMLog *log,
                          BMIdMap *idmap,
                          const int cd_vert_node_offset,
                          const int cd_face_node_offset,
                          const int cd_face_areas,
                          const int cd_boundary_flag,
                          const int cd_edge_boundary,
                          const int cd_flag,
                          const int cd_valence,
                          const int cd_origco,
                          const int cd_origno);

void BKE_pbvh_set_idmap(PBVH *pbvh, BMIdMap *idmap);

void BKE_pbvh_update_offsets(PBVH *pbvh,
                             const int cd_vert_node_offset,
                             const int cd_face_node_offset,
                             const int cd_face_areas,
                             const int cd_boudnary_flags,
                             const int cd_edge_boundary,
                             const int cd_flag,
                             const int cd_valence,
                             const int cd_origco,
                             const int cd_origno,
                             const int cd_curvature_dir);

void BKE_pbvh_update_bmesh_offsets(PBVH *pbvh, int cd_vert_node_offset, int cd_face_node_offset);

void BKE_pbvh_build_pixels(PBVH *pbvh, Mesh *mesh, Image *image, ImageUser *image_user);
void BKE_pbvh_free(PBVH *pbvh);

void BKE_pbvh_set_bm_log(PBVH *pbvh, BMLog *log);
BMLog *BKE_pbvh_get_bm_log(PBVH *pbvh);

/**
checks if original data needs to be updated for v, and if so updates it.  Stroke_id
is provided by the sculpt code and is used to detect updates.  The reason we do it
inside the verts and not in the nodes is to allow splitting of the pbvh during the stroke.
*/
bool BKE_pbvh_bmesh_check_origdata(SculptSession *ss, BMVert *v, int stroke_id);

/** used so pbvh can differentiate between different strokes,
    see BKE_pbvh_bmesh_check_origdata */
void BKE_pbvh_set_stroke_id(PBVH *pbvh, int stroke_id);

/* Hierarchical Search in the BVH, two methods:
 * - For each hit calling a callback.
 * - Gather nodes in an array (easy to multi-thread) see blender::bke::pbvh::search_gather.
 */

void BKE_pbvh_search_callback(PBVH *pbvh,
                              BKE_pbvh_SearchCallback scb,
                              void *search_data,
                              BKE_pbvh_HitCallback hcb,
                              void *hit_data);

/* Ray-cast
 * the hit callback is called for all leaf nodes intersecting the ray;
 * it's up to the callback to find the primitive within the leaves that is
 * hit first */

void BKE_pbvh_raycast(PBVH *pbvh,
                      BKE_pbvh_HitOccludedCallback cb,
                      void *data,
                      const float ray_start[3],
                      const float ray_normal[3],
                      bool original,
                      int stroke_id);

bool BKE_pbvh_node_raycast(SculptSession *ss,
                           PBVH *pbvh,
                           PBVHNode *node,
                           float (*origco)[3],
                           bool use_origco,
                           const float ray_start[3],
                           const float ray_normal[3],
                           IsectRayPrecalc *isect_precalc,
                           int *hit_count,
                           float *depth,
                           float *back_depth,
                           PBVHVertRef *active_vertex_index,
                           PBVHFaceRef *active_face_grid_index,
                           float *face_normal,
                           int stroke_id);

bool BKE_pbvh_bmesh_node_raycast_detail(PBVH *pbvh,
                                        PBVHNode *node,
                                        const float ray_start[3],
                                        IsectRayPrecalc *isect_precalc,
                                        float *depth,
                                        float *r_edge_length);

/**
 * For orthographic cameras, project the far away ray segment points to the root node so
 * we can have better precision.
 *
 * Note: the interval is not guaranteed to lie between ray_start and ray_end; this is
 * not necessary for orthographic views and is impossible anyhow due to the necessity of
 * projecting the far clipping plane into the local object space.  This works out to
 * dividing view3d->clip_end by the object scale, which for small object and large
 * clip_end's can easily lead to floating-point overflows.
 *
 */
void BKE_pbvh_clip_ray_ortho(
    PBVH *pbvh, bool original, float ray_start[3], float ray_end[3], float ray_normal[3]);

void BKE_pbvh_find_nearest_to_ray(PBVH *pbvh,
                                  BKE_pbvh_HitOccludedCallback cb,
                                  void *data,
                                  const float ray_start[3],
                                  const float ray_normal[3],
                                  bool original);

bool BKE_pbvh_node_find_nearest_to_ray(SculptSession *ss,
                                       PBVH *pbvh,
                                       PBVHNode *node,
                                       float (*origco)[3],
                                       bool use_origco,
                                       const float ray_start[3],
                                       const float ray_normal[3],
                                       float *depth,
                                       float *dist_sq,
                                       int stroke_id);

/* Drawing */
void BKE_pbvh_set_frustum_planes(PBVH *pbvh, PBVHFrustumPlanes *planes);
void BKE_pbvh_get_frustum_planes(PBVH *pbvh, PBVHFrustumPlanes *planes);

void BKE_pbvh_draw_cb(PBVH *pbvh,
                      Mesh *me,
                      bool update_only_visible,
                      PBVHFrustumPlanes *update_frustum,
                      PBVHFrustumPlanes *draw_frustum,
                      void (*draw_fn)(void *user_data, PBVHBatches *batches, PBVH_GPU_Args *args),
                      void *user_data,
                      bool full_render,
                      PBVHAttrReq *attrs,
                      int attrs_num);

/* PBVH Access */

bool BKE_pbvh_has_faces(const PBVH *pbvh);

/**
 * Get the PBVH root's bounding box.
 */
void BKE_pbvh_bounding_box(const PBVH *pbvh, float min[3], float max[3]);

/**
 * Multi-res hidden data, only valid for type == PBVH_GRIDS.
 */
unsigned int **BKE_pbvh_grid_hidden(const PBVH *pbvh);

void BKE_pbvh_sync_visibility_from_verts(PBVH *pbvh, Mesh *me);

/**
 * Returns the number of visible quads in the nodes' grids.
 */
int BKE_pbvh_count_grid_quads(BLI_bitmap **grid_hidden,
                              const int *grid_indices,
                              int totgrid,
                              int gridsize,
                              int display_gridsize);

/**
 * Multi-res level, only valid for type == #PBVH_GRIDS.
 */
const CCGKey *BKE_pbvh_get_grid_key(const PBVH *pbvh);

CCGElem **BKE_pbvh_get_grids(const PBVH *pbvh);
BLI_bitmap **BKE_pbvh_get_grid_visibility(const PBVH *pbvh);
int BKE_pbvh_get_grid_num_verts(const PBVH *pbvh);
int BKE_pbvh_get_grid_num_faces(const PBVH *pbvh);

/* Node Access */

void BKE_pbvh_check_tri_areas(PBVH *pbvh, PBVHNode *node);
void BKE_pbvh_face_areas_begin(PBVH *pbvh);

bool BKE_pbvh_bmesh_check_valence(PBVH *pbvh, PBVHVertRef vertex);
void BKE_pbvh_bmesh_update_valence(PBVH *pbvh, PBVHVertRef vertex);
void BKE_pbvh_bmesh_update_all_valence(PBVH *pbvh);
bool BKE_pbvh_bmesh_mark_update_valence(PBVH *pbvh, PBVHVertRef vertex);

/* if pbvh uses a split index buffer, will call BKE_pbvh_vert_tag_update_normal_triangulation;
   otherwise does nothing.  returns true if BKE_pbvh_vert_tag_update_normal_triangulation was
   called.*/
void BKE_pbvh_vert_tag_update_normal_triangulation(PBVHNode *node);
void BKE_pbvh_node_mark_original_update(PBVHNode *node);
void BKE_pbvh_vert_tag_update_normal_tri_area(PBVHNode *node);
void BKE_pbvh_update_all_tri_areas(PBVH *pbvh);
void BKE_pbvh_node_mark_update(PBVHNode *node);
void BKE_pbvh_node_mark_update_mask(PBVHNode *node);
void BKE_pbvh_node_mark_update_defrag(PBVHNode *node);
void BKE_pbvh_node_mark_update_color(PBVHNode *node);
void BKE_pbvh_vert_tag_update_normal_visibility(PBVHNode *node);
void BKE_pbvh_node_mark_update_face_sets(PBVHNode *node);
void BKE_pbvh_node_mark_update_visibility(PBVHNode *node);
void BKE_pbvh_node_mark_rebuild_draw(PBVHNode *node);
void BKE_pbvh_node_mark_redraw(PBVHNode *node);
void BKE_pbvh_node_mark_normals_update(PBVHNode *node);
void BKE_pbvh_node_mark_topology_update(PBVHNode *node);
void BKE_pbvh_node_fully_hidden_set(PBVHNode *node, int fully_hidden);
bool BKE_pbvh_node_fully_hidden_get(PBVHNode *node);
void BKE_pbvh_node_fully_masked_set(PBVHNode *node, int fully_masked);
bool BKE_pbvh_node_fully_masked_get(PBVHNode *node);
void BKE_pbvh_node_fully_unmasked_set(PBVHNode *node, int fully_masked);
bool BKE_pbvh_node_fully_unmasked_get(PBVHNode *node);
void BKE_pbvh_node_mark_curvature_update(PBVHNode *node);

void BKE_pbvh_mark_rebuild_pixels(PBVH *pbvh);
void BKE_pbvh_vert_tag_update_normal(PBVH *pbvh, PBVHVertRef vertex);

void BKE_pbvh_node_get_grids(PBVH *pbvh,
                             PBVHNode *node,
                             int **grid_indices,
                             int *totgrid,
                             int *maxgrid,
                             int *gridsize,
                             CCGElem ***r_griddata);
void BKE_pbvh_node_num_verts(PBVH *pbvh, PBVHNode *node, int *r_uniquevert, int *r_totvert);
const int *BKE_pbvh_node_get_vert_indices(PBVHNode *node);
void BKE_pbvh_node_get_loops(PBVH *pbvh,
                             PBVHNode *node,
                             const int **r_loop_indices,
                             const int **r_corner_verts);

/* Get number of faces in the mesh; for PBVH_GRIDS the
 * number of base mesh faces is returned.
 */
int BKE_pbvh_num_faces(const PBVH *pbvh);

void BKE_pbvh_node_get_BB(PBVHNode *node, float bb_min[3], float bb_max[3]);
void BKE_pbvh_node_get_original_BB(PBVHNode *node, float bb_min[3], float bb_max[3]);

float BKE_pbvh_node_get_tmin(PBVHNode *node);

/**
 * Test if AABB is at least partially inside the #PBVHFrustumPlanes volume.
 */
bool BKE_pbvh_node_frustum_contain_AABB(PBVHNode *node, void *frustum);
/**
 * Test if AABB is at least partially outside the #PBVHFrustumPlanes volume.
 */
bool BKE_pbvh_node_frustum_exclude_AABB(PBVHNode *node, void *frustum);

blender::bke::dyntopo::DyntopoSet<BMVert> *BKE_pbvh_bmesh_node_unique_verts(PBVHNode *node);
blender::bke::dyntopo::DyntopoSet<BMVert> *BKE_pbvh_bmesh_node_other_verts(PBVHNode *node);
blender::bke::dyntopo::DyntopoSet<BMFace> *BKE_pbvh_bmesh_node_faces(PBVHNode *node);

void BKE_pbvh_bmesh_regen_node_verts(PBVH *pbvh, bool report);
void BKE_pbvh_bmesh_mark_node_regen(PBVH *pbvh, PBVHNode *node);

/* Update Bounding Box/Redraw and clear flags. */

void BKE_pbvh_update_bounds(PBVH *pbvh, int flags);
void BKE_pbvh_update_vertex_data(PBVH *pbvh, int flags);
void BKE_pbvh_update_visibility(PBVH *pbvh);
void BKE_pbvh_update_normals(PBVH *pbvh, SubdivCCG *subdiv_ccg);
void BKE_pbvh_redraw_BB(PBVH *pbvh, float bb_min[3], float bb_max[3]);
void BKE_pbvh_get_grid_updates(PBVH *pbvh, bool clear, void ***r_gridfaces, int *r_totface);
void BKE_pbvh_grids_update(PBVH *pbvh,
                           CCGElem **grids,
                           void **gridfaces,
                           DMFlagMat *flagmats,
                           unsigned int **grid_hidden,
                           CCGKey *key);
void BKE_pbvh_subdiv_ccg_set(PBVH *pbvh, SubdivCCG *subdiv_ccg);
void BKE_pbvh_face_sets_set(PBVH *pbvh, int *face_sets);

/**
 * If an operation causes the hide status stored in the mesh to change, this must be called
 * to update the references to those attributes, since they are only added when necessary.
 */
void BKE_pbvh_update_hide_attributes_from_mesh(PBVH *pbvh);

void BKE_pbvh_face_sets_color_set(PBVH *pbvh, int seed, int color_default);

/* Vertex Deformer. */

float (*BKE_pbvh_vert_coords_alloc(PBVH *pbvh))[3];
void BKE_pbvh_vert_coords_apply(PBVH *pbvh, const float (*vertCos)[3], int totvert);
bool BKE_pbvh_is_deformed(PBVH *pbvh);

/* Vertex Iterator. */

/* This iterator has quite a lot of code, but it's designed to:
 * - allow the compiler to eliminate dead code and variables
 * - spend most of the time in the relatively simple inner loop */

/* NOTE: PBVH_ITER_ALL does not skip hidden vertices,
 * PBVH_ITER_UNIQUE does */
#define PBVH_ITER_ALL 0
#define PBVH_ITER_UNIQUE 1

struct PBVHVertexIter {
  /* iteration */
  int g;
  int width;
  int height;
  int gx;
  int gy;
  int i;
  int index;
  PBVHVertRef vertex;

  /* grid */
  CCGKey key;
  CCGElem **grids;
  CCGElem *grid;
  BLI_bitmap **grid_hidden, *gh;
  int *grid_indices;
  int totgrid;
  int gridsize;

  /* mesh */
  blender::MutableSpan<blender::float3> vert_positions;
  blender::MutableSpan<blender::float3> vert_normals;
  const bool *hide_vert;
  int totvert;
  const int *vert_indices;
  float *vmask;
  bool is_mesh;

  /* bmesh */
  int bi;
  int bm_cur_set;
  blender::bke::dyntopo::DyntopoSet<BMVert> *bm_unique_verts, *bm_other_verts;
  blender::bke::dyntopo::DyntopoSet<BMVert>::iterator bm_iter, bm_iter_end;

  CustomData *bm_vdata;
  int cd_vert_mask_offset;
  int cd_vcol_offset;

  /* result: these are all computed in the macro, but we assume
   * that compiler optimization's will skip the ones we don't use */
  BMVert *bm_vert;
  float *co;
  float *no;
  float *fno;
  float *mask;
  bool visible;
};

void pbvh_vertex_iter_init(PBVH *pbvh, PBVHNode *node, PBVHVertexIter *vi, int mode);

#define BKE_pbvh_vertex_iter_begin(pbvh, node, vi, mode) \
  pbvh_vertex_iter_init(pbvh, node, &vi, mode); \
\
  for (vi.i = 0, vi.g = 0; vi.g < vi.totgrid; vi.g++) { \
    if (vi.grids) { \
      vi.width = vi.gridsize; \
      vi.height = vi.gridsize; \
      vi.index = vi.vertex.i = vi.grid_indices[vi.g] * vi.key.grid_area - 1; \
      vi.grid = vi.grids[vi.grid_indices[vi.g]]; \
      if (mode == PBVH_ITER_UNIQUE) { \
        vi.gh = vi.grid_hidden[vi.grid_indices[vi.g]]; \
      } \
    } \
    else { \
      vi.width = vi.totvert; \
      vi.height = 1; \
    } \
\
    for (vi.gy = 0; vi.gy < vi.height; vi.gy++) { \
      for (vi.gx = 0; vi.gx < vi.width; vi.gx++, vi.i++) { \
        if (vi.grid) { \
          vi.co = CCG_elem_co(&vi.key, vi.grid); \
          vi.fno = CCG_elem_no(&vi.key, vi.grid); \
          vi.mask = vi.key.has_mask ? CCG_elem_mask(&vi.key, vi.grid) : NULL; \
          vi.grid = CCG_elem_next(&vi.key, vi.grid); \
          vi.index++; \
          vi.vertex.i++; \
          vi.visible = true; \
          if (vi.gh) { \
            if (BLI_BITMAP_TEST(vi.gh, vi.gy * vi.gridsize + vi.gx)) { \
              continue; \
            } \
          } \
        } \
        else if (!vi.vert_positions.is_empty()) { \
          vi.visible = !(vi.hide_vert && vi.hide_vert[vi.vert_indices[vi.gx]]); \
          if (mode == PBVH_ITER_UNIQUE && !vi.visible) { \
            continue; \
          } \
          vi.co = vi.vert_positions[vi.vert_indices[vi.gx]]; \
          vi.no = vi.vert_normals[vi.vert_indices[vi.gx]]; \
          vi.index = vi.vertex.i = vi.vert_indices[vi.i]; \
          if (vi.vmask) { \
            vi.mask = &vi.vmask[vi.index]; \
          } \
        } \
        else { \
          if (vi.bm_iter == vi.bm_iter_end) { \
            if (vi.bm_cur_set == 0 && mode == PBVH_ITER_ALL) { \
              vi.bm_cur_set = 1; \
              vi.bm_iter = vi.bm_other_verts->begin(); \
              vi.bm_iter_end = vi.bm_other_verts->end(); \
              if (vi.bm_iter == vi.bm_iter_end) { \
                continue; \
              } \
            } \
            else { \
              continue; \
            } \
          } \
          BMVert *bv = *vi.bm_iter; \
          ++vi.bm_iter; \
          vi.bm_vert = bv; \
          vi.vertex.i = (intptr_t)bv; \
          vi.index = BM_elem_index_get(vi.bm_vert); \
          vi.visible = !BM_elem_flag_test_bool(vi.bm_vert, BM_ELEM_HIDDEN); \
          if (mode == PBVH_ITER_UNIQUE && !vi.visible) { \
            continue; \
          } \
          vi.co = vi.bm_vert->co; \
          vi.fno = vi.bm_vert->no; \
          vi.mask = vi.cd_vert_mask_offset != -1 ? \
                        (float *)BM_ELEM_CD_GET_VOID_P(vi.bm_vert, vi.cd_vert_mask_offset) : \
                        nullptr; \
        }

#define BKE_pbvh_vertex_iter_end \
  } \
  } \
  } \
  ((void)0)

#define PBVH_FACE_ITER_VERTS_RESERVED 8

struct PBVHFaceIter {
  PBVHFaceRef face;
  int index;
  bool *hide;
  int *face_set;
  int i;

  PBVHVertRef *verts;
  int verts_num;

  PBVHVertRef verts_reserved_[PBVH_FACE_ITER_VERTS_RESERVED];
  const PBVHNode *node_;
  PBVHType pbvh_type_;
  int verts_size_;
  blender::bke::dyntopo::DyntopoSet<BMFace>::iterator bm_iter_, bm_iter_end_;
  int cd_face_set_;
  bool *hide_poly_;
  int *face_sets_;
  const int *poly_offsets_;
  const int *looptri_polys_;
  const int *corner_verts_;
  int prim_index_;
  const SubdivCCG *subdiv_ccg_;
  const BMesh *bm;
  CCGKey subdiv_key_;

  int last_poly_index_;
};

void BKE_pbvh_face_iter_init(PBVH *pbvh, PBVHNode *node, PBVHFaceIter *fd);
void BKE_pbvh_face_iter_step(PBVHFaceIter *fd);
bool BKE_pbvh_face_iter_done(PBVHFaceIter *fd);
void BKE_pbvh_face_iter_finish(PBVHFaceIter *fd);

/**
 * Iterate over faces inside a #PBVHNode. These are either base mesh faces
 * (for PBVH_FACES and PBVH_GRIDS) or BMesh faces (for PBVH_BMESH).
 */
#define BKE_pbvh_face_iter_begin(pbvh, node, fd) \
  BKE_pbvh_face_iter_init(pbvh, node, &fd); \
  for (; !BKE_pbvh_face_iter_done(&fd); BKE_pbvh_face_iter_step(&fd)) {

#define BKE_pbvh_face_iter_end(fd) \
  } \
  BKE_pbvh_face_iter_finish(&fd)

void BKE_pbvh_node_get_proxies(PBVHNode *node, PBVHProxyNode **proxies, int *proxy_count);
void BKE_pbvh_node_free_proxies(PBVHNode *node);
PBVHProxyNode *BKE_pbvh_node_add_proxy(PBVH *pbvh, PBVHNode *node);

/**
 * \note doing a full search on all vertices here seems expensive,
 * however this is important to avoid having to recalculate bound-box & sync the buffers to the
 * GPU (which is far more expensive!) See: #47232.
 */
bool BKE_pbvh_node_has_vert_with_normal_update_tag(PBVH *pbvh, PBVHNode *node);

// void BKE_pbvh_node_BB_reset(PBVHNode *node);
// void BKE_pbvh_node_BB_expand(PBVHNode *node, float co[3]);

bool pbvh_has_mask(const PBVH *pbvh);

bool pbvh_has_face_sets(PBVH *pbvh);

void pbvh_show_mask_set(PBVH *pbvh, bool show_mask);
void pbvh_show_face_sets_set(PBVH *pbvh, bool show_face_sets);

/* Parallelization. */

void BKE_pbvh_parallel_range_settings(TaskParallelSettings *settings,
                                      bool use_threading,
                                      int totnode);

float (*BKE_pbvh_get_vert_positions(const PBVH *pbvh))[3];
const float (*BKE_pbvh_get_vert_normals(const PBVH *pbvh))[3];
const bool *BKE_pbvh_get_vert_hide(const PBVH *pbvh);
bool *BKE_pbvh_get_vert_hide_for_write(PBVH *pbvh);

const bool *BKE_pbvh_get_poly_hide(const PBVH *pbvh);

/* Get active color attribute; if pbvh is non-null
 * and is of type PBVH_BMESH the layer inside of
 * pbvh->header.bm will be returned, otherwise the
 * layer will be looked up inside of me.
 */
bool BKE_pbvh_get_color_layer(PBVH *pbvh,
                              Mesh *me,
                              CustomDataLayer **r_layer,
                              eAttrDomain *r_domain);

/* Swaps colors at each element in indices (of domain pbvh->vcol_domain)
 * with values in colors. PBVH_FACES only.*/
void BKE_pbvh_swap_colors(PBVH *pbvh,
                          const int *indices,
                          const int indices_num,
                          float (*colors)[4]);

/* Stores colors from the elements in indices (of domain pbvh->vcol_domain)
 * into colors. PBVH_FACES only.*/
void BKE_pbvh_store_colors(PBVH *pbvh,
                           const int *indices,
                           const int indices_num,
                           float (*colors)[4]);

/* Like BKE_pbvh_store_colors but handles loop->vert conversion. PBVH_FACES only. */
void BKE_pbvh_store_colors_vertex(PBVH *pbvh,
                                  const int *indices,
                                  const int indices_num,
                                  float (*colors)[4]);

bool BKE_pbvh_is_drawing(const PBVH *pbvh);

/* Do not call in PBVH_GRIDS mode */
void BKE_pbvh_node_num_loops(PBVH *pbvh, PBVHNode *node, int *r_totloop);

void BKE_pbvh_update_active_vcol(PBVH *pbvh, Mesh *mesh);

void BKE_pbvh_vertex_color_set(PBVH *pbvh, PBVHVertRef vertex, const float color[4]);
void BKE_pbvh_vertex_color_get(const PBVH *pbvh, PBVHVertRef vertex, float r_color[4]);

void BKE_pbvh_ensure_node_loops(PBVH *pbvh);
bool BKE_pbvh_draw_cache_invalid(const PBVH *pbvh);
int BKE_pbvh_debug_draw_gen_get(PBVHNode *node);

int BKE_pbvh_get_node_index(PBVH *pbvh, PBVHNode *node);
int BKE_pbvh_get_node_id(PBVH *pbvh, PBVHNode *node);

void BKE_pbvh_curvature_update_set(PBVHNode *node, bool state);
bool BKE_pbvh_curvature_update_get(PBVHNode *node);

int BKE_pbvh_get_totnodes(PBVH *pbvh);

bool BKE_pbvh_bmesh_check_tris(PBVH *pbvh, PBVHNode *node);
PBVHTriBuf *BKE_pbvh_bmesh_get_tris(PBVH *pbvh, PBVHNode *node);
void BKE_pbvh_bmesh_free_tris(PBVH *pbvh, PBVHNode *node);

/*recalculates boundary flags for *all* vertices.  used by
  symmetrize.*/
void BKE_pbvh_recalc_bmesh_boundary(PBVH *pbvh);
void BKE_pbvh_set_boundary_flags(PBVH *pbvh, int *boundary_flags);

/* saves all bmesh references to internal indices, to be restored later */
void BKE_pbvh_bmesh_save_indices(PBVH *pbvh);

/* restore bmesh references from previously indices saved by BKE_pbvh_bmesh_save_indices */
void BKE_pbvh_bmesh_from_saved_indices(PBVH *pbvh);

/* wraps calls to BM_mesh_toolflags_set in BKE_pbvh_bmesh_save_indices and
 * BKE_pbvh_bmesh_from_saved_indices */
void BKE_pbvh_bmesh_set_toolflags(PBVH *pbvh, bool use_toolflags);

void BKE_pbvh_bmesh_remove_face(PBVH *pbvh, BMFace *f, bool log_face);
void BKE_pbvh_bmesh_remove_edge(PBVH *pbvh, BMEdge *e, bool log_edge);
void BKE_pbvh_bmesh_remove_vertex(PBVH *pbvh, BMVert *v, bool log_vert);
void BKE_pbvh_bmesh_add_face(PBVH *pbvh, BMFace *f, bool log_face, bool force_tree_walk);

/* e_tri and f_example are allowed to be nullptr. */
BMFace *BKE_pbvh_face_create_bmesh(PBVH *pbvh,
                                   BMVert *v_tri[3],
                                   BMEdge *e_tri[3],
                                   const BMFace *f_example);

/* If node is nullptr then one will be found in the pbvh. */
BMVert *BKE_pbvh_vert_create_bmesh(
    PBVH *pbvh, float co[3], float no[3], PBVHNode *node, BMVert *v_example);
PBVHNode *BKE_pbvh_node_from_face_bmesh(PBVH *pbvh, BMFace *f);
PBVHNode *BKE_pbvh_node_from_index(PBVH *pbvh, int node_i);

PBVHNode *BKE_pbvh_get_node_leaf_safe(PBVH *pbvh, int i);

void BKE_pbvh_get_vert_face_areas(PBVH *pbvh, PBVHVertRef vertex, float *r_areas, int valence);
void BKE_pbvh_set_symmetry(PBVH *pbvh, int symmetry, int boundary_symmetry);

int BKE_pbvh_do_fset_symmetry(int fset, const int symflag, const float *co);

/*
Uses pmap to build an array of edge indices surrounding vertex
r_edges, r_edges_size, heap_alloc define an existing array to put data in.

final array is similarly put in these pointers.  note that calling code
may pass a stack allocated array (*heap_alloc should be false), and must
check if heap_alloc is true afterwards and free *r_edges.

r_polys is an array of integer pairs and must be same logical size as r_edges
*/
void BKE_pbvh_pmap_to_edges(PBVH *pbvh,
                            PBVHVertRef vertex,
                            int **r_edges,
                            int *r_edges_size,
                            bool *heap_alloc,
                            int **r_polys);

void BKE_pbvh_distort_correction_set(PBVH *pbvh, eAttrCorrectMode value);

void BKE_pbvh_set_face_areas(PBVH *pbvh, float *face_areas);
void BKE_pbvh_set_bmesh(PBVH *pbvh, BMesh *bm);
void BKE_pbvh_free_bmesh(PBVH *pbvh, BMesh *bm);

void BKE_pbvh_show_orig_set(PBVH *pbvh, bool show_orig);
bool BKE_pbvh_show_orig_get(PBVH *pbvh);

void BKE_pbvh_flush_tri_areas(PBVH *pbvh);
void BKE_pbvh_bmesh_check_nodes(PBVH *pbvh);

#include "BLI_math_vector.hh"

namespace blender::bke::pbvh {

void set_flags_valence(PBVH *pbvh, uint8_t *flags, int *valence);
void set_original(PBVH *pbvh, Span<float3> origco, Span<float3> origno);
void update_vert_boundary_bmesh(int cd_faceset_offset,
                                int cd_vert_node_offset,
                                int cd_face_node_offset,
                                int cd_vcol,
                                int cd_boundary_flag,
                                const int cd_flag,
                                const int cd_valence,
                                BMVert *v,
                                const CustomData *ldata,
                                float sharp_angle_limit);
void update_sharp_vertex_bmesh(BMVert *v, int cd_boundary_flag, const float sharp_angle_limit);

void update_vert_boundary_faces(int *boundary_flags,
                                const int *face_sets,
                                const bool *hide_poly,
                                const blender::int2 *medge,
                                const int *corner_verts,
                                const int *corner_edges,
                                blender::OffsetIndices<int> polys,
                                const blender::GroupedSpan<int> &pmap,
                                PBVHVertRef vertex,
                                const bool *sharp_edges,
                                const bool *seam_edges,
                                uint8_t *flags,
                                int *valence);
void update_edge_boundary_bmesh(BMEdge *e,
                                int cd_faceset_offset,
                                int cd_edge_boundary,
                                const int cd_flag,
                                const int cd_valence,
                                const CustomData *ldata,
                                float sharp_angle_limit);
void update_edge_boundary_faces(int edge,
                                Span<float3> vertex_positions,
                                Span<float3> vertex_normals,
                                Span<blender::int2> edges,
                                OffsetIndices<int> polys,
                                Span<float3> poly_normals,
                                int *edge_boundary_flags,
                                const int *vert_boundary_flags,
                                const int *face_sets,
                                const bool *sharp_edge,
                                const bool *seam_edge,
                                const GroupedSpan<int> &pmap,
                                const GroupedSpan<int> &epmap,
                                const CustomData *ldata,
                                float sharp_angle_limit,
                                blender::Span<int> corner_verts,
                                blender::Span<int> corner_edges);
void update_edge_boundary_grids(int edge,
                                Span<blender::int2> edges,
                                OffsetIndices<int> polys,
                                int *edge_boundary_flags,
                                const int *vert_boundary_flags,
                                const int *face_sets,
                                const bool *sharp_edge,
                                const bool *seam_edge,
                                const GroupedSpan<int> &pmap,
                                const GroupedSpan<int> &epmap,
                                const CustomData *ldata,
                                SubdivCCG *subdiv_ccg,
                                const CCGKey *key,
                                float sharp_angle_limit,
                                blender::Span<int> corner_verts,
                                blender::Span<int> corner_edges);
void update_vert_boundary_grids(PBVH *pbvh, int vertex);

bool check_vert_boundary(PBVH *pbvh, PBVHVertRef vertex);
bool check_edge_boundary(PBVH *pbvh, PBVHEdgeRef edge);

Vector<PBVHNode *> search_gather(PBVH *pbvh,
                                 BKE_pbvh_SearchCallback scb,
                                 void *search_data,
                                 PBVHNodeFlags leaf_flag = PBVH_Leaf);
Vector<PBVHNode *> gather_proxies(PBVH *pbvh);
Vector<PBVHNode *> get_flagged_nodes(PBVH *pbvh, int flag);
void set_pmap(PBVH *pbvh, GroupedSpan<int> pmap);
void set_vemap(PBVH *pbvh, GroupedSpan<int> vemap);
GroupedSpan<int> get_pmap(PBVH *pbvh);

void sharp_limit_set(PBVH *pbvh, float limit);
float test_sharp_faces_bmesh(BMFace *f1, BMFace *f2, float limit);
float test_sharp_faces_mesh(int f1,
                            int f2,
                            float limit,
                            blender::Span<blender::float3> positions,
                            blender::OffsetIndices<int> &polys,
                            blender::Span<blender::float3> poly_normals,
                            blender::Span<int> corner_verts);

blender::Span<blender::float3> get_poly_normals(const PBVH *pbvh);
void set_vert_boundary_map(PBVH *pbvh, BLI_bitmap *vert_boundary_map);
void on_stroke_start(PBVH *pbvh);
void defragment_pbvh(PBVH *pbvh, bool partial);
void fragment_node(PBVH *pbvh, PBVHNode *node);
void assign_hives(PBVH *pbvh);

}  // namespace blender::bke::pbvh
