/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file
 * \ingroup bli
 */

#include "MEM_guardedalloc.h"

#include "BLI_array.h"
#include "BLI_buffer.h"
#include "BLI_ghash.h"
#include "BLI_heap_simple.h"
#include "BLI_math.h"
#include "BLI_memarena.h"
#include "BLI_task.h"
#include "BLI_utildefines.h"

#include "BLI_rand.h"
#include "PIL_time.h"

#include "BLI_threadsafe_mempool.h"
#include "trimesh.h"

#include "BKE_DerivedMesh.h"
#include "BKE_ccg.h"
#include "BKE_pbvh.h"

#include "GPU_buffers.h"

#include "bmesh.h"
#include "pbvh_intern.h"

#include <assert.h>

#define DYNTOPO_CD_INTERP

#define DYNTOPO_TIME_LIMIT 0.015
#define DYNTOPO_RUN_INTERVAL 0.01

struct EdgeQueueContext;
static bool pbvh_trimesh_subdivide_long_edges(struct EdgeQueueContext *eq_ctx, PBVH *bvh);

/* Avoid skinny faces */
#define USE_EDGEQUEUE_EVEN_SUBDIV
#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
#  include "BKE_global.h"
#endif

/* Support for only operating on front-faces */
#define USE_EDGEQUEUE_FRONTFACE

/* don't add edges into the queue multiple times */
#define USE_EDGEQUEUE_TAG

/**
 * Ensure we don't have dirty tags for the edge queue, and that they are left cleared.
 * (slow, even for debug mode, so leave disabled for now).
 */
#if defined(USE_EDGEQUEUE_TAG) && 0
#  if !defined(NDEBUG)
#    define USE_EDGEQUEUE_TAG_VERIFY
#  endif
#endif

// #define USE_VERIFY

#ifdef USE_VERIFY
static void pbvh_trimesh_verify(PBVH *bvh);
#endif

/** \} */

TableGSet *BLI_table_gset_new()
{
  TableGSet *ts = MEM_callocN(sizeof(TableGSet), "TableGSet");

  ts->ptr_to_idx = BLI_ghash_ptr_new("tm elem set ptr to idx");

  return ts;
}

void BLI_table_gset_free(TableGSet *ts)
{
  if (!ts->ptr_to_idx) {
    printf("double call to BLI_table_gset_free!\n");
    return;
  }

  if (ts->elems) {
    MEM_freeN(ts->elems);
  }

  BLI_ghash_free(ts->ptr_to_idx, NULL, NULL);
  ts->ptr_to_idx = NULL;

  MEM_freeN(ts);
}

bool BLI_table_gset_add(TableGSet *ts, void *elem)
{
  if (BLI_table_gset_has(ts, elem)) {
    return true;
  }

  BLI_table_gset_insert(ts, elem);
  return false;
}

void BLI_table_gset_insert(TableGSet *ts, void *elem)
{
  if (ts->cur >= ts->size) {
    int newsize = (ts->cur + 1);
    newsize = (newsize << 1) - (newsize >> 1);
    newsize = MAX2(newsize, 8);

    if (!ts->elems) {
      ts->elems = (void *)MEM_mallocN(sizeof(void *) * newsize, "ts->elems");
    }
    else {
      ts->elems = (void *)MEM_reallocN(ts->elems, newsize * sizeof(void *));
    }

    BLI_ghash_clear(ts->ptr_to_idx, NULL, NULL);

    // compact
    int i = 0, j = 0;
    for (i = 0; i < ts->cur; i++) {
      void *elem2 = ts->elems[i];

      if (elem2) {
        BLI_ghash_insert(ts->ptr_to_idx, elem2, (void *)j);
        ts->elems[j++] = elem2;
      }
    }

    ts->size = newsize;
    ts->cur = ts->length;
  }

  BLI_ghash_insert(ts->ptr_to_idx, elem, (void *)ts->cur);
  ts->elems[ts->cur++] = elem;
  ts->length++;
}

void BLI_table_gset_remove(TableGSet *ts, void *elem, bool ignoreExist)
{
  int idx = (int)BLI_ghash_lookup(ts->ptr_to_idx, elem);

  BLI_ghash_remove(ts->ptr_to_idx, elem, NULL, NULL);

  if (!ts->elems || ts->elems[idx] != elem) {
    if (!ignoreExist) {
      printf("eek! %d %p %p\n", idx, elem, ts->elems[idx]);
    }
    return;
  }

  ts->length--;
  ts->elems[idx] = NULL;
}

bool BLI_table_gset_has(TableGSet *ts, void *elem)
{
  return BLI_ghash_haskey(ts->ptr_to_idx, elem);
}

/****************************** Building ******************************/

#define _TRITEST(a, b, c) tri->v1 == a && tri->v2 == b && tri->v3 == c

void BKE_pbvh_trimesh_detail_size_set(PBVH *pbvh, float detail_size)
{
  pbvh->bm_max_edge_len = detail_size;
  pbvh->bm_min_edge_len = pbvh->bm_max_edge_len * 0.4f;
}

static TMFace *trimesh_tri_exists(TMEdge *e, TMVert *opposite)
{
  for (int i = 0; i < e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    if (_TRITEST(e->v1, e->v2, opposite))
      return tri;
    if (_TRITEST(e->v1, opposite, e->v2))
      return tri;
    if (_TRITEST(e->v2, e->v1, opposite))
      return tri;
    if (_TRITEST(e->v2, opposite, e->v1))
      return tri;
    if (_TRITEST(opposite, e->v1, e->v2))
      return tri;
    if (_TRITEST(opposite, e->v2, e->v1))
      return tri;
  }

  return NULL;
}
#undef _TRITEST

/**
 * Uses a map of vertices to lookup the final target.
 * References can't point to previous items (would cause infinite loop).
 */
static TMVert *tm_vert_hash_lookup_chain(GHash *deleted_verts, TMVert *v)
{
  while (true) {
    TMVert **v_next_p = (TMVert **)BLI_ghash_lookup_p(deleted_verts, v);

    if (v_next_p == NULL) {
      /* not remapped*/
      return v;
    }
    else if (*v_next_p == NULL) {
      /* removed and not remapped */
      return NULL;
    }
    else {
      /* remapped */
      v = *v_next_p;
    }
  }
}

static void tm_edges_from_tri(
    TM_TriMesh *tm, TMVert *vs[3], TMEdge *es[3], int threadnr, bool skipcd)
{
  es[0] = TM_get_edge(tm, vs[0], vs[1], skipcd);
  es[1] = TM_get_edge(tm, vs[1], vs[2], skipcd);
  es[2] = TM_get_edge(tm, vs[2], vs[0], skipcd);
}

/* Update node data after splitting */
static void pbvh_trimesh_node_finalize(PBVH *bvh,
                                       const int node_index,
                                       const int cd_vert_node_offset,
                                       const int cd_face_node_offset)
{
  GSetIterator gs_iter;
  PBVHNode *n = &bvh->nodes[node_index];
  bool has_visible = false;

  /* Create vert hash sets */
  n->tm_unique_verts = BLI_table_gset_new();  // BLI_gset_ptr_new("trimesh_unique_verts");
  n->tm_other_verts = BLI_table_gset_new();   // BLI_gset_ptr_new("trimesh_other_verts");

  BB_reset(&n->vb);

  GSET_ITER (gs_iter, n->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

    /* Update ownership of faces */
    TM_ELEM_CD_SET_INT(f, cd_face_node_offset, node_index);

    /* Update vertices */
    for (int i = 0; i < 3; i++) {
      TMVert *v = TM_GET_TRI_VERT(f, i);
      TMEdge *e = TM_GET_TRI_EDGE(f, i);
      TMLoopData *l = TM_GET_TRI_LOOP(f, i);

      has_visible |= !(v->flag & TM_ELEM_HIDDEN);

      if (TM_ELEM_CD_GET_INT(v, cd_vert_node_offset) != DYNTOPO_NODE_NONE) {
        BLI_table_gset_add(n->tm_other_verts, v);
      }
      else {
        BLI_table_gset_insert(n->tm_unique_verts, v);
        TM_ELEM_CD_SET_INT(v, cd_vert_node_offset, node_index);
      }

      /* Update node bounding box */
      BB_expand(&n->vb, v->co);
    }
  }

  BLI_assert(n->vb.bmin[0] <= n->vb.bmax[0] && n->vb.bmin[1] <= n->vb.bmax[1] &&
             n->vb.bmin[2] <= n->vb.bmax[2]);
  n->orig_vb = n->vb;

  /* Build GPU buffers for new node and update vertex normals */
  BKE_pbvh_node_mark_rebuild_draw(n);

  BKE_pbvh_node_fully_hidden_set(n, !has_visible);
  n->flag |= PBVH_UpdateNormals;
}

/* Recursively split the node if it exceeds the leaf_limit */
static void pbvh_trimesh_node_split(PBVH *bvh, const BBC *bbc_array, int node_index)
{
  const int cd_vert_node_offset = bvh->cd_vert_node_offset;
  const int cd_face_node_offset = bvh->cd_face_node_offset;
  PBVHNode *n = &bvh->nodes[node_index];

  if (BLI_gset_len(n->tm_faces) <= bvh->leaf_limit) {
    /* Node limit not exceeded */
    pbvh_trimesh_node_finalize(bvh, node_index, cd_vert_node_offset, cd_face_node_offset);
    return;
  }

  /* Calculate bounding box around primitive centroids */
  BB cb;
  BB_reset(&cb);
  GSetIterator gs_iter;
  GSET_ITER (gs_iter, n->tm_faces) {
    const TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
    const BBC *bbc = &bbc_array[f->index];

    BB_expand(&cb, bbc->bcentroid);
  }

  /* Find widest axis and its midpoint */
  const int axis = BB_widest_axis(&cb);
  const float mid = (cb.bmax[axis] + cb.bmin[axis]) * 0.5f;

  /* Add two new child nodes */
  const int children = bvh->totnode;
  n->children_offset = children;
  pbvh_grow_nodes(bvh, bvh->totnode + 2);

  /* Array reallocated, update current node pointer */
  n = &bvh->nodes[node_index];

  /* Initialize children */
  PBVHNode *c1 = &bvh->nodes[children], *c2 = &bvh->nodes[children + 1];
  c1->flag |= PBVH_Leaf;
  c2->flag |= PBVH_Leaf;
  c1->tm_faces = BLI_gset_ptr_new_ex("tm_faces", BLI_gset_len(n->tm_faces) / 2);
  c2->tm_faces = BLI_gset_ptr_new_ex("tm_faces", BLI_gset_len(n->tm_faces) / 2);

  /* Partition the parent node's faces between the two children */
  GSET_ITER (gs_iter, n->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
    const BBC *bbc = &bbc_array[f->index];

    if (bbc->bcentroid[axis] < mid) {
      BLI_gset_insert(c1->tm_faces, f);
    }
    else {
      BLI_gset_insert(c2->tm_faces, f);
    }
  }

  /* Enforce at least one primitive in each node */
  GSet *empty = NULL, *other = NULL;
  if (BLI_gset_len(c1->tm_faces) == 0) {
    empty = c1->tm_faces;
    other = c2->tm_faces;
  }
  else if (BLI_gset_len(c2->tm_faces) == 0) {
    empty = c2->tm_faces;
    other = c1->tm_faces;
  }

  if (empty && other) {
    GSET_ITER (gs_iter, other) {
      void *key = BLI_gsetIterator_getKey(&gs_iter);
      BLI_gset_insert(empty, key);
      BLI_gset_remove(other, key, NULL);
      break;
    }
  }

  /* Clear this node */

  /* Mark this node's unique verts as unclaimed */
  if (n->tm_unique_verts) {
    TMVert *v;

    TMS_ITER (v, n->tm_unique_verts) {
      TM_ELEM_CD_SET_INT(v, cd_vert_node_offset, DYNTOPO_NODE_NONE);
    }
    TMS_ITER_END

    BLI_table_gset_free(n->tm_unique_verts);
  }

  /* Unclaim faces */
  GSET_ITER (gs_iter, n->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
    if (f) {
      TM_ELEM_CD_SET_INT(f, cd_face_node_offset, DYNTOPO_NODE_NONE);
    }
  }
  BLI_gset_free(n->tm_faces, NULL);

  if (n->tm_other_verts) {
    BLI_table_gset_free(n->tm_other_verts);
  }

  if (n->layer_disp) {
    MEM_freeN(n->layer_disp);
  }

  n->tm_faces = NULL;
  n->tm_unique_verts = NULL;
  n->tm_other_verts = NULL;
  n->layer_disp = NULL;

  if (n->draw_buffers) {
    GPU_pbvh_buffers_free(n->draw_buffers);
    n->draw_buffers = NULL;
  }
  n->flag &= ~PBVH_Leaf;

  /* Recurse */
  pbvh_trimesh_node_split(bvh, bbc_array, children);
  pbvh_trimesh_node_split(bvh, bbc_array, children + 1);

  /* Array maybe reallocated, update current node pointer */
  n = &bvh->nodes[node_index];

  /* Update bounding box */
  BB_reset(&n->vb);
  BB_expand_with_bb(&n->vb, &bvh->nodes[n->children_offset].vb);
  BB_expand_with_bb(&n->vb, &bvh->nodes[n->children_offset + 1].vb);
  n->orig_vb = n->vb;
}

/* Recursively split the node if it exceeds the leaf_limit */
static bool pbvh_trimesh_node_limit_ensure(PBVH *bvh, int node_index)
{
  GSet *tm_faces = bvh->nodes[node_index].tm_faces;
  const int tm_faces_size = BLI_gset_len(tm_faces);
  if (tm_faces_size <= bvh->leaf_limit) {
    /* Node limit not exceeded */
    return false;
  }

  /* For each TMFace, store the AABB and AABB centroid */
  BBC *bbc_array = MEM_mallocN(sizeof(BBC) * tm_faces_size, "BBC");

  GSetIterator gs_iter;
  int i;
  GSET_ITER_INDEX (gs_iter, tm_faces, i) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
    BBC *bbc = &bbc_array[i];

    BB_reset((BB *)bbc);
    BB_expand((BB *)bbc, f->v1->co);
    BB_expand((BB *)bbc, f->v2->co);
    BB_expand((BB *)bbc, f->v3->co);

    BBC_update_centroid(bbc);

    /* so we can do direct lookups on 'bbc_array' */
    f->index = i; /* set_dirty! */
  }
  /* Likely this is already dirty. */
  bvh->tm->elem_index_dirty |= TM_TRI;

  pbvh_trimesh_node_split(bvh, bbc_array, node_index);

  MEM_freeN(bbc_array);

  return true;
}

/**********************************************************************/

#if 0
static int pbvh_trimesh_node_offset_from_elem(PBVH *bvh, BMElem *ele)
{
  switch (ele->head.htype) {
  case BM_VERT:
    return bvh->cd_vert_node_offset;
  default:
    BLI_assert(ele->head.htype == BM_FACE);
    return bvh->cd_face_node_offset;
  }
}

static int pbvh_trimesh_node_index_from_elem(PBVH *bvh, void *key)
{
  const int cd_node_offset = pbvh_trimesh_node_offset_from_elem(bvh, key);
  const int node_index = TRIMESH_ELEM_CD_GET_INT((BMElem *)key, cd_node_offset);

  BLI_assert(node_index != DYNTOPO_NODE_NONE);
  BLI_assert(node_index < bvh->totnode);
  (void)bvh;

  return node_index;
}

static PBVHNode *pbvh_trimesh_node_from_elem(PBVH *bvh, void *key)
{
  return &bvh->nodes[pbvh_trimesh_node_index_from_elem(bvh, key)];
}

/* typecheck */
#  define pbvh_trimesh_node_index_from_elem(bvh, key) \
    (CHECK_TYPE_ANY(key, TMFace *, TMVert *), pbvh_trimesh_node_index_from_elem(bvh, key))
#  define pbvh_trimesh_node_from_elem(bvh, key) \
    (CHECK_TYPE_ANY(key, TMFace *, TMVert *), pbvh_trimesh_node_from_elem(bvh, key))
#endif

BLI_INLINE int pbvh_trimesh_node_index_from_vert(PBVH *bvh, const TMVert *key)
{
  const int node_index = TM_ELEM_CD_GET_INT((const TMElement *)key, bvh->cd_vert_node_offset);
  BLI_assert(node_index != DYNTOPO_NODE_NONE);
  BLI_assert(node_index < bvh->totnode);
  return node_index;
}

BLI_INLINE int pbvh_trimesh_node_index_from_face(PBVH *bvh, const TMFace *key)
{
  const int node_index = TM_ELEM_CD_GET_INT((const TMElement *)key, bvh->cd_face_node_offset);
  BLI_assert(node_index != DYNTOPO_NODE_NONE);
  BLI_assert(node_index < bvh->totnode);
  return node_index;
}

BLI_INLINE PBVHNode *pbvh_trimesh_node_from_vert(PBVH *bvh, const TMVert *key)
{
  int idx = pbvh_trimesh_node_index_from_vert(bvh, key);
  return idx == DYNTOPO_NODE_NONE ? NULL : bvh->nodes + idx;
}

BLI_INLINE PBVHNode *pbvh_trimesh_node_from_face(PBVH *bvh, const TMFace *key)
{
  int ni = pbvh_trimesh_node_index_from_face(bvh, key);

  if (ni < 0) {
    return NULL;
  }

  return &bvh->nodes[ni];
}

static TMVert *pbvh_trimesh_vert_create(
    PBVH *bvh, int node_index, const float co[3], const float no[3], const int cd_vert_mask_offset)
{
  PBVHNode *node = &bvh->nodes[node_index];
  TMVert *v = TM_make_vert(bvh->tm, co, no, false);

  if (node_index < 0 || node_index >= bvh->totnode) {
    printf("eek!\n");
    return v;
  }

  v->index = bvh->tm->totvert - 1;
  TM_ELEM_CD_SET_INT(v, bvh->cd_vert_node_offset, node_index);

  BLI_table_gset_insert(node->tm_unique_verts, v);

  bvh->tm->elem_table_dirty |= TM_VERTEX;
  bvh->tm->elem_index_dirty |= TM_VERTEX;

  node->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateBB;

  TM_log_vert_add(bvh->tm_log, v, cd_vert_mask_offset, false);

  return v;
#if 0
  PBVHNode *node = &bvh->nodes[node_index];

  BLI_assert((bvh->totnode == 1 || node_index) && node_index <= bvh->totnode);


  /* avoid initializing customdata because its quite involved */
  TMVert *v = TM_make_vert(bvh->tm, co, no, 0, false);
  v->index = bvh->tm->totvert-1;
  //CustomData_bmesh_set_default(&bvh->tm->vdata, &v->customdata);

  //BLI_gset_insert(node->tm_unique_verts, v);
  TM_ELEM_CD_SET_INT(v, bvh->cd_vert_node_offset, node_index);

  bvh->tm->elem_table_dirty |= TM_VERTEX|TM_EDGE|TM_TRI;
  bvh->tm->elem_index_dirty |= TM_VERTEX|TM_EDGE|TM_TRI;

  node->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateBB;

  /* Log the new vertex */
  TM_log_vert_add(bvh->tm_log, v, cd_vert_mask_offset, false);

  return v;
#endif
}

/**
 * \note Callers are responsible for checking if the face exists before adding.
 */
static TMFace *pbvh_trimesh_face_create(
    PBVH *bvh, int node_index, TMVert *v_tri[3], TMEdge *e_tri[3], const TMFace *f_example)
{
  PBVHNode *node = &bvh->nodes[node_index];

  /* ensure we never add existing face */
  // BLI_assert(!BM_face_exists(v_tri, 3));

  TMFace *f = TM_make_tri(bvh->tm, v_tri[0], v_tri[1], v_tri[2], false);
  // f->head.hflag = f_example->head.hflag;
  if (f_example) {
    f->flag = f_example->flag;
  }

  BLI_gset_insert(node->tm_faces, f);
  TM_ELEM_CD_SET_INT(f, bvh->cd_face_node_offset, node_index);

  /* mark node for update */
  node->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateNormals;
  node->flag &= ~PBVH_FullyHidden;

  /* Log the new face */
  // BM_log_face_added(bvh->bm_log, f);
  BLI_trimesh_log_tri(bvh->tm_log, f, false);

  return f;
}

/* Return the number of faces in 'node' that use vertex 'v' */
#if 0
static int pbvh_trimesh_node_vert_use_count(PBVH *bvh, PBVHNode *node, TMVert *v)
{
  TMFace *f;
  int count = 0;

  TRIMESH_ITER_VERT_TRIS (v, f) {
    PBVHNode *f_node = pbvh_trimesh_node_from_face(bvh, f);
    if (f_node == node) {
      count++;
    }
  }
  TRIMESH_ITER_VERT_TRIS_END;

  return count;
}
#endif

#define pbvh_trimesh_node_vert_use_count_is_equal(bvh, node, v, n) \
  (pbvh_trimesh_node_vert_use_count_at_most(bvh, node, v, (n) + 1) == n)

static int pbvh_trimesh_node_vert_use_count_at_most(PBVH *bvh,
                                                    PBVHNode *node,
                                                    TMVert *v,
                                                    const int count_max)
{
  int count = 0;
  TMFace *f;

  TM_ITER_VERT_TRIS (v, f) {
    PBVHNode *f_node = pbvh_trimesh_node_from_face(bvh, f);
    if (f_node == node) {
      count++;
      if (count == count_max) {
        return count;
      }
    }
  }
  TM_ITER_VERT_TRIS_END;

  return count;
}

/* Return a node that uses vertex 'v' other than its current owner */
static PBVHNode *pbvh_trimesh_vert_other_node_find(PBVH *bvh, TMVert *v)
{
  PBVHNode *current_node = pbvh_trimesh_node_from_vert(bvh, v);
  TMFace *f;

  TM_ITER_VERT_TRIS (v, f) {
    PBVHNode *f_node = pbvh_trimesh_node_from_face(bvh, f);

    if (f_node != current_node) {
      return f_node;
    }
  }
  TM_ITER_VERT_TRIS_END;

  return NULL;
}

static void pbvh_trimesh_vert_ownership_transfer(PBVH *bvh, PBVHNode *new_owner, TMVert *v)
{
  PBVHNode *current_owner = pbvh_trimesh_node_from_vert(bvh, v);
  /* mark node for update */
  current_owner->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateBB;

  BLI_assert(current_owner != new_owner);

  /* Remove current ownership */
  BLI_table_gset_remove(current_owner->tm_unique_verts, v, false);

  /* Set new ownership */
  TM_ELEM_CD_SET_INT(v, bvh->cd_vert_node_offset, (int)(new_owner - bvh->nodes));
  BLI_table_gset_insert(new_owner->tm_unique_verts, v);
  BLI_table_gset_remove(new_owner->tm_other_verts, v, true);
  BLI_assert(!BLI_table_gset_has(new_owner->tm_other_verts, v));

  /* mark node for update */
  new_owner->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateBB;
}

static void pbvh_trimesh_vert_remove(PBVH *bvh, TMVert *v)
{
  /* never match for first time */
  int f_node_index_prev = DYNTOPO_NODE_NONE;

  PBVHNode *v_node = pbvh_trimesh_node_from_vert(bvh, v);
  if (!v_node) {
    printf("error: vertex not in any nodes!\n");
    return;
  }

  BLI_table_gset_remove(v_node->tm_unique_verts, v, false);
  TM_ELEM_CD_SET_INT(v, bvh->cd_vert_node_offset, DYNTOPO_NODE_NONE);

  /* Have to check each neighboring face's node */
  TMFace *f;
  TM_ITER_VERT_TRIS (v, f) {
    for (int i = 0; i < 3; i++) {
      TMEdge *e = TM_GET_TRI_EDGE(f, i);
      for (int j = 0; j < e->tris.length; j++) {
        TMFace *f = e->tris.items[j];

        const int f_node_index = pbvh_trimesh_node_index_from_face(bvh, f);

        /* faces often share the same node,
         * quick check to avoid redundant #BLI_gset_remove calls */
        if (f_node_index_prev != f_node_index) {
          f_node_index_prev = f_node_index;

          PBVHNode *f_node = &bvh->nodes[f_node_index];
          f_node->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateBB;

          /* Remove current ownership */
          BLI_table_gset_remove(f_node->tm_other_verts, v, true);

          BLI_assert(!BLI_table_gset_has(f_node->tm_unique_verts, v));
          BLI_assert(!BLI_table_gset_has(f_node->tm_other_verts, v));
        }
      }
    }
  }
  TM_ITER_VERT_TRIS_END;
}

static void pbvh_trimesh_face_remove(PBVH *bvh, TMFace *f)
{
  PBVHNode *f_node = pbvh_trimesh_node_from_face(bvh, f);

  /* Check if any of this face's vertices need to be removed
   * from the node */

  for (int i = 0; f_node && i < 3; i++) {
    TMVert *v = TM_GET_TRI_VERT(f, i);

    if (pbvh_trimesh_node_vert_use_count_is_equal(bvh, f_node, v, 1)) {
      if (BLI_table_gset_has(f_node->tm_unique_verts, v)) {
        /* Find a different node that uses 'v' */
        PBVHNode *new_node;

        new_node = pbvh_trimesh_vert_other_node_find(bvh, v);
        BLI_assert(new_node);  // || BM_vert_face_count_is_equal(v, 1));

        if (new_node) {
          pbvh_trimesh_vert_ownership_transfer(bvh, new_node, v);
        }
      }
      else {
        /* Remove from other verts */
        BLI_table_gset_remove(f_node->tm_other_verts, v, true);
      }
    }
  }

  /* Remove face from node and top level */
  if (f_node && TM_ELEM_CD_GET_INT(f, bvh->cd_face_node_offset) != DYNTOPO_NODE_NONE) {
    BLI_gset_remove(f_node->tm_faces, f, NULL);
    TM_ELEM_CD_SET_INT(f, bvh->cd_face_node_offset, DYNTOPO_NODE_NONE);
  }

  /* Log removed face */
  BLI_trimesh_log_tri_kill(bvh->tm_log, f, false, false);

  /* mark node for update */
  if (f_node) {
    f_node->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateNormals;
  }
}

static void pbvh_trimesh_node_drop_orig(PBVHNode *node)
{
  if (node->tm_orco) {
    MEM_freeN(node->tm_orco);
  }
  if (node->tm_ortri) {
    MEM_freeN(node->tm_ortri);
  }
  node->tm_orco = NULL;
  node->tm_ortri = NULL;
  node->tm_tot_ortri = 0;
}

/****************************** EdgeQueue *****************************/

struct EdgeQueue;

typedef struct EdgeQueue {
  HeapSimple *heap;
  void **elems;
  int totelems;

  const float *center;
  float center_proj[3]; /* for when we use projected coords. */
  float radius_squared;
  float limit_len_squared;
#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
  float limit_len;
#endif

  bool (*edge_queue_tri_in_range)(const struct EdgeQueue *q, TMFace *f);

  const float *view_normal;
#ifdef USE_EDGEQUEUE_FRONTFACE
  unsigned int use_view_normal : 1;
#endif
} EdgeQueue;

typedef struct EdgeQueueContext {
  EdgeQueue *q;
  BLI_mempool *pool;
  TM_TriMesh *tm;
  int cd_vert_mask_offset;
  int cd_vert_node_offset;
  int cd_face_node_offset;
} EdgeQueueContext;

/* only tag'd edges are in the queue */
#ifdef USE_EDGEQUEUE_TAG
#  define EDGE_QUEUE_TEST(e) \
    (TM_elem_flag_test((CHECK_TYPE_INLINE(e, TMEdge *), e), TRIMESH_TEMP_TAG2))
#  define EDGE_QUEUE_ENABLE(e) \
    TM_elem_flag_enable((CHECK_TYPE_INLINE(e, TMEdge *), e), TRIMESH_TEMP_TAG2)
#  define EDGE_QUEUE_DISABLE(e) \
    TM_elem_flag_disable((CHECK_TYPE_INLINE(e, TMEdge *), e), TRIMESH_TEMP_TAG2)
#endif

#ifdef USE_EDGEQUEUE_TAG_VERIFY
/* simply check no edges are tagged
 * (it's a requirement that edges enter and leave a clean tag state) */
static void pbvh_trimesh_edge_tag_verify(PBVH *bvh)
{
  for (int n = 0; n < bvh->totnode; n++) {
    PBVHNode *node = &bvh->nodes[n];
    if (node->tm_faces) {
      GSetIterator gs_iter;
      GSET_ITER (gs_iter, node->tm_faces) {
        TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
        TMEdge *e_tri[3];
        BMLoop *l_iter;

        BLI_assert(f->len == 3);
        l_iter = BM_FACE_FIRST_LOOP(f);
        e_tri[0] = l_iter->e;
        l_iter = l_iter->next;
        e_tri[1] = l_iter->e;
        l_iter = l_iter->next;
        e_tri[2] = l_iter->e;

        BLI_assert((EDGE_QUEUE_TEST(e_tri[0]) == false) && (EDGE_QUEUE_TEST(e_tri[1]) == false) &&
                   (EDGE_QUEUE_TEST(e_tri[2]) == false));
      }
    }
  }
}
#endif

static bool edge_queue_tri_in_sphere(const EdgeQueue *q, TMFace *f)
{
  float c[3];

  /* Get closest point in triangle to sphere center */
  // slow
  closest_on_tri_to_point_v3(c, q->center, f->v1->co, f->v2->co, f->v3->co);
  /* Check if triangle intersects the sphere */

  return len_squared_v3v3(c, q->center) < q->radius_squared;
  /*
  float l1 = len_squared_v3v3(f->v1->co, q->center);
  float l2 = len_squared_v3v3(f->v2->co, q->center);
  float l3 = len_squared_v3v3(f->v3->co, q->center);
  float l = MIN3(l1, l2, l3);

  bool ret = false;
  ret = ret || l < q->radius_squared;
  ret = ret || dist_squared_to_line_segment_v3(q->center, f->v1->co, f->v2->co) <
  q->radius_squared; ret = ret || dist_squared_to_line_segment_v3(q->center, f->v2->co, f->v3->co)
  < q->radius_squared; ret = ret || dist_squared_to_line_segment_v3(q->center, f->v3->co,
  f->v1->co) < q->radius_squared;

  return ret;
  */
}

static bool edge_queue_tri_in_circle(const EdgeQueue *q, TMFace *f)
{
  float c[3];
  float tri_proj[3][3];

  /* Get closest point in triangle to sphere center */

  project_plane_normalized_v3_v3v3(tri_proj[0], f->v1->co, q->view_normal);
  project_plane_normalized_v3_v3v3(tri_proj[1], f->v2->co, q->view_normal);
  project_plane_normalized_v3_v3v3(tri_proj[2], f->v3->co, q->view_normal);

  closest_on_tri_to_point_v3(c, q->center_proj, tri_proj[0], tri_proj[1], tri_proj[2]);

  /* Check if triangle intersects the sphere */
  return len_squared_v3v3(q->center_proj, c) <= q->radius_squared;
}

/* Return true if the vertex mask is less than 1.0, false otherwise */
static bool check_mask(EdgeQueueContext *eq_ctx, TMVert *v)
{
  return TM_ELEM_CD_GET_FLOAT(v, eq_ctx->cd_vert_mask_offset) < 1.0f;
}

static void edge_queue_insert(EdgeQueueContext *eq_ctx, TMEdge *e, float priority)
{
  void **elems = eq_ctx->q->elems;
  BLI_array_declare(elems);
  BLI_array_len_set(elems, eq_ctx->q->totelems);

  /* Don't let topology update affect fully masked vertices. This used to
   * have a 50% mask cutoff, with the reasoning that you can't do a 50%
   * topology update. But this gives an ugly border in the mesh. The mask
   * should already make the brush move the vertices only 50%, which means
   * that topology updates will also happen less frequent, that should be
   * enough. */
  if (((eq_ctx->cd_vert_mask_offset == -1) ||
       (check_mask(eq_ctx, e->v1) || check_mask(eq_ctx, e->v2))) &&
      !(TM_elem_flag_test_bool(e->v1, TM_ELEM_HIDDEN) ||
        TM_elem_flag_test_bool(e->v2, TM_ELEM_HIDDEN))) {
    TMVert **pair = BLI_mempool_alloc(eq_ctx->pool);
    pair[0] = e->v1;
    pair[1] = e->v2;

    BLI_array_append(elems, pair);
    eq_ctx->q->elems = elems;
    eq_ctx->q->totelems = BLI_array_len(elems);

    BLI_heapsimple_insert(eq_ctx->q->heap, priority, pair);
#ifdef USE_EDGEQUEUE_TAG
    BLI_assert(EDGE_QUEUE_TEST(e) == false);
    EDGE_QUEUE_ENABLE(e);
#endif
  }
}

static float calc_weighted_edge(EdgeQueueContext *eq_ctx, TMEdge *e)
{
#if 0
  float avg = 0.0;
  float tot = 0.0;

  for (int i = 0; i < 2; i++) {
    TMVert *v = i ? e->v2 : e->v1;
    for (int j = 0; j < v->edges.length; j++) {
      TMEdge *e = v->edges.items[j];
      TMVert *v2 = TM_other_vert(e, v);

      avg += len_squared_v3v3(v->co, v2->co);
      tot += 1.0f;
    }
  }

  avg /= tot;
  if (tot == 0.0f) {
    //return l;
  }
  //float ratio = avg / eq_ctx->q->limit_len_squared;
  float ratio = eq_ctx->q->limit_len / avg;
  ratio = powf(ratio, 10.0);
#endif
  float l = TM_edge_calc_length_squared(e);
  float n = ((float)e->v1->edges.length + (float)e->v2->edges.length) * 0.5f;
  n = MAX2(n - 5.0f, 1.0f);

  return l * powf(n, 5.0f);
}

static void long_edge_queue_edge_add(EdgeQueueContext *eq_ctx, TMEdge *e)
{
#ifdef USE_EDGEQUEUE_TAG
  if (EDGE_QUEUE_TEST(e) == false)
#endif
  {
    float len_sq = TM_edge_calc_length_squared(e);

    if (len_sq > eq_ctx->q->limit_len_squared) {
      edge_queue_insert(eq_ctx, e, -len_sq);
    }
  }
}

#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
static void long_edge_queue_edge_add_recursive(EdgeQueueContext *eq_ctx,
                                               TMEdge *l_edge,
                                               TMEdge *l_end,
                                               TMFace *t_edge,
                                               TMFace *t_end,
                                               const float len_sq,
                                               float limit_len)
{
  BLI_assert(len_sq > square_f(limit_len));

  if (l_edge->tris.length == 0 || l_end->tris.length == 0) {
    return;
  }

#  ifdef USE_EDGEQUEUE_FRONTFACE
  if (eq_ctx->q->use_view_normal) {
    if (dot_v3v3(t_edge->no, eq_ctx->q->view_normal) < 0.0f) {
      return;
    }
  }
#  endif

#  ifdef USE_EDGEQUEUE_TAG
  if (EDGE_QUEUE_TEST(l_edge) == false)
#  endif
  {
    edge_queue_insert(eq_ctx, l_edge, -len_sq);
  }

  /* temp support previous behavior! */
  if (UNLIKELY(G.debug_value == 1234)) {
    return;
  }

  if (l_edge->tris.length > 1) {
    /* How much longer we need to be to consider for subdividing
     * (avoids subdividing faces which are only *slightly* skinny) */
#  define EVEN_EDGELEN_THRESHOLD 1.2f
    /* How much the limit increases per recursion
     * (avoids performing subdivisions too far away). */
#  define EVEN_GENERATION_SCALE 1.6f

    const float len_sq_cmp = len_sq * EVEN_EDGELEN_THRESHOLD;

    limit_len *= EVEN_GENERATION_SCALE;
    const float limit_len_sq = square_f(limit_len);

    for (int i = 0; i < l_edge->tris.length; i++) {
      TMFace *tri = l_edge->tris.items[i];
      TMEdge *l_adjacent[2] = {TM_nextEdgeInTri(tri, l_edge), TM_prevEdgeInTri(tri, l_edge)};

      for (int j = 0; j < 2; j++) {
        float len_sq_other = TM_edge_calc_length_squared(l_adjacent[j]);
        if (len_sq_other > max_ff(len_sq_cmp, limit_len_sq)) {
          //                  edge_queue_insert(eq_ctx, l_adjacent[i]->e, -len_sq_other);
          long_edge_queue_edge_add_recursive(eq_ctx,
                                             l_adjacent[j],
                                             l_adjacent[j],
                                             TM_nextTriInEdge(l_adjacent[j], tri),
                                             tri,
                                             len_sq_other,
                                             limit_len);
        }
      }
    }

    /*
    BMLoop *l_iter = l_edge;
    do {
      BMLoop *l_adjacent[2] = {l_iter->next, l_iter->prev};
      for (int i = 0; i < ARRAY_SIZE(l_adjacent); i++) {
        float len_sq_other = TRIMESH_edge_calc_length_squared(l_adjacent[i]->e);
        if (len_sq_other > max_ff(len_sq_cmp, limit_len_sq)) {
          //                  edge_queue_insert(eq_ctx, l_adjacent[i]->e, -len_sq_other);
          long_edge_queue_edge_add_recursive(
            eq_ctx, l_adjacent[i]->radial_next, l_adjacent[i], len_sq_other, limit_len);
        }
      }
    } while ((l_iter = l_iter->radial_next) != l_end);
    */
#  undef EVEN_EDGELEN_THRESHOLD
#  undef EVEN_GENERATION_SCALE
  }
}
#endif /* USE_EDGEQUEUE_EVEN_SUBDIV */

static void short_edge_queue_edge_add(EdgeQueueContext *eq_ctx, TMEdge *e)
{
#ifdef USE_EDGEQUEUE_TAG
  if (EDGE_QUEUE_TEST(e) == false)
#endif
  {
    const float len_sq = calc_weighted_edge(eq_ctx, e);
    if (len_sq < eq_ctx->q->limit_len_squared) {
      edge_queue_insert(eq_ctx, e, len_sq);
    }
  }
}

static void long_edge_queue_face_add(EdgeQueueContext *eq_ctx, TMFace *f)
{
#ifdef USE_EDGEQUEUE_FRONTFACE
  if (eq_ctx->q->use_view_normal) {
    if (dot_v3v3(f->no, eq_ctx->q->view_normal) < 0.0f) {
      return;
    }
  }
#endif

  if (eq_ctx->q->edge_queue_tri_in_range(eq_ctx->q, f)) {
    /* Check each edge of the face */
#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
    for (int i = 0; i < 3; i++) {
      TMEdge *e = TM_GET_TRI_EDGE(f, i);

      const float len_sq = TM_edge_calc_length_squared(e);

      if (len_sq > eq_ctx->q->limit_len_squared) {
        if (e->v1->edges.length > 32 || e->v2->edges.length > 32) {
          // printf("eek! %.4f %d %d\n", len_sq, e->v1->edges.length, e->v2->edges.length);
          continue;
        }

        long_edge_queue_edge_add_recursive(
            eq_ctx, e, e, TM_nextTriInEdge(e, f), f, len_sq, eq_ctx->q->limit_len);
      }
#else
    long_edge_queue_edge_add(eq_ctx, l_iter->e);
#endif
    }
  }
}

static void short_edge_queue_face_add(EdgeQueueContext *eq_ctx, TMFace *f)
{
#ifdef USE_EDGEQUEUE_FRONTFACE
  if (eq_ctx->q->use_view_normal) {
    if (dot_v3v3(f->no, eq_ctx->q->view_normal) < 0.0f) {
      return;
    }
  }
#endif

  if (eq_ctx->q->edge_queue_tri_in_range(eq_ctx->q, f)) {
    for (int i = 0; i < 3; i++) {
      TMEdge *e = TM_GET_TRI_EDGE(f, i);
      short_edge_queue_edge_add(eq_ctx, e);
    }
  }
}

/* Create a priority queue containing vertex pairs connected by a long
 * edge as defined by PBVH.bm_max_edge_len.
 *
 * Only nodes marked for topology update are checked, and in those
 * nodes only edges used by a face intersecting the (center, radius)
 * sphere are checked.
 *
 * The highest priority (lowest number) is given to the longest edge.
 */
static void long_edge_queue_create(EdgeQueueContext *eq_ctx,
                                   PBVH *bvh,
                                   const float center[3],
                                   const float view_normal[3],
                                   float radius,
                                   const bool use_frontface,
                                   const bool use_projected)
{
  eq_ctx->q->elems = eq_ctx->q->totelems = 0;
  eq_ctx->q->heap = BLI_heapsimple_new();
  eq_ctx->q->center = center;
  eq_ctx->q->radius_squared = radius * radius;
  eq_ctx->q->limit_len_squared = bvh->bm_max_edge_len * bvh->bm_max_edge_len;
#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
  eq_ctx->q->limit_len = bvh->bm_max_edge_len;
#endif

  eq_ctx->q->view_normal = view_normal;

#ifdef USE_EDGEQUEUE_FRONTFACE
  eq_ctx->q->use_view_normal = use_frontface;
#else
  UNUSED_VARS(use_frontface);
#endif

  if (use_projected) {
    eq_ctx->q->edge_queue_tri_in_range = edge_queue_tri_in_circle;
    project_plane_normalized_v3_v3v3(eq_ctx->q->center_proj, center, view_normal);
  }
  else {
    eq_ctx->q->edge_queue_tri_in_range = edge_queue_tri_in_sphere;
  }

#ifdef USE_EDGEQUEUE_TAG_VERIFY
  pbvh_trimesh_edge_tag_verify(bvh);
#endif

  for (int n = 0; n < bvh->totnode; n++) {
    PBVHNode *node = &bvh->nodes[n];

    /* Check leaf nodes marked for topology update */
    if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology) &&
        !(node->flag & PBVH_FullyHidden)) {
      GSetIterator gs_iter;

      /* Check each face */
      GSET_ITER (gs_iter, node->tm_faces) {
        TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

        long_edge_queue_face_add(eq_ctx, f);
      }
    }
  }
}

static void long_edge_queue_create3(EdgeQueueContext *eq_ctx,
                                    PBVH *bvh,
                                    TMFace **tris,
                                    int tottri,
                                    const float center[3],
                                    const float view_normal[3],
                                    float radius,
                                    const bool use_frontface,
                                    const bool use_projected)
{

  eq_ctx->q->elems = eq_ctx->q->totelems = 0;
  eq_ctx->q->heap = BLI_heapsimple_new();
  eq_ctx->q->center = center;
  eq_ctx->q->radius_squared = radius * radius;
  eq_ctx->q->limit_len_squared = bvh->bm_max_edge_len * bvh->bm_max_edge_len;
#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
  eq_ctx->q->limit_len = bvh->bm_max_edge_len;
#endif

  eq_ctx->q->view_normal = view_normal;

#ifdef USE_EDGEQUEUE_FRONTFACE
  eq_ctx->q->use_view_normal = use_frontface;
#else
  UNUSED_VARS(use_frontface);
#endif

  if (use_projected) {
    eq_ctx->q->edge_queue_tri_in_range = edge_queue_tri_in_circle;
    project_plane_normalized_v3_v3v3(eq_ctx->q->center_proj, center, view_normal);
  }
  else {
    eq_ctx->q->edge_queue_tri_in_range = edge_queue_tri_in_sphere;
  }

#ifdef USE_EDGEQUEUE_TAG_VERIFY
  pbvh_trimesh_edge_tag_verify(bvh);
#endif

  for (int i = 0; i < tottri; i++) {
    long_edge_queue_face_add(eq_ctx, tris[i]);
  }
}

typedef struct ThreadSplitData {
  int flag;
  BLI_ThreadSafePool *pool;
  int cd_face_node_offset;
  int cd_vert_node_offset;
  TMTriIsland *islands;
  int cd_vert_mask_offset;
  PBVH *pbvh;
  float center[3], view_normal[3], radius;
  bool use_frontface, use_projected;
} ThreadSplitData;

void longqueue_job(void *__restrict userdata, int n, const TaskParallelTLS *__restrict tls)
{
  ThreadSplitData *data = userdata;
  TMTriIsland *island = data->islands + n;
  TMFace **tris = island->tris;
  int tottri = island->tottri;
  EdgeQueue q;
  EdgeQueueContext eq_ctx;

  memset(&q, 0, sizeof(q));

  eq_ctx.cd_face_node_offset = data->cd_face_node_offset;
  eq_ctx.cd_vert_mask_offset = data->cd_vert_mask_offset;
  eq_ctx.cd_vert_node_offset = data->cd_vert_node_offset;
  eq_ctx.pool = BLI_mempool_create(sizeof(void *) * 2, 0, 512, 0);
  eq_ctx.q = &q;
  eq_ctx.tm = data->pbvh->tm;

  long_edge_queue_create3(&eq_ctx,
                          data->pbvh,
                          tris,
                          tottri,
                          data->center,
                          data->view_normal,
                          data->radius,
                          data->use_frontface,
                          data->use_projected);
  pbvh_trimesh_subdivide_long_edges(&eq_ctx, data->pbvh);

  BLI_mempool_destroy(eq_ctx.pool);

  if (q.heap) {
    BLI_heapsimple_free(q.heap, NULL);
  }
  if (q.elems) {
    MEM_freeN(q.elems);
  }

  /*
static void long_edge_queue_create3(EdgeQueueContext * eq_ctx,
                                      PBVH * bvh,
                                      TMFace * *tris,
                                      int tottri,
                                      const float center[3],
                                      const float view_normal[3],
                                      float radius,
                                      const bool use_frontface,
                                      const bool use_projected)
                                      */
}

static bool face_on_node_boundary(PBVH *pbvh, TMFace *f, int n, int maxdepth)
{
  for (int i = 0; i < 3; i++) {
    TMVert *v = TM_GET_TRI_VERT(f, i);
    for (int j = 0; j < v->edges.length; j++) {
      TMEdge *e = v->edges.items[j];

      for (int k = 0; k < e->tris.length; k++) {
        TMFace *f2 = e->tris.items[k];

        int ni = TM_ELEM_CD_GET_INT(f2, pbvh->cd_face_node_offset);
        if (ni != n) {
          return true;
        }

        if (maxdepth > 0) {
          bool ret = face_on_node_boundary(pbvh, f2, n, maxdepth - 1);
          if (ret) {
            return true;
          }
        }
      }
    }
  }

  return false;
}

/* Create a priority queue containing vertex pairs connected by a long
 * edge as defined by PBVH.bm_max_edge_len.
 *
 * Only nodes marked for topology update are checked, and in those
 * nodes only edges used by a face intersecting the (center, radius)
 * sphere are checked.
 *
 * The highest priority (lowest number) is given to the longest edge.
 */
static bool long_edge_queue_create2(PBVH *pbvh,
                                    const float center[3],
                                    const float view_normal[3],
                                    float radius,
                                    const bool use_frontface,
                                    const bool use_projected)
{
  TM_TriMesh *tm = pbvh->tm;
  TMTriIsland *islands = NULL;
  TMFace **boundary = NULL;
  BLI_array_declare(islands);
  BLI_array_declare(boundary);

  for (int n = 0; n < pbvh->totnode; n++) {
    PBVHNode *node = &pbvh->nodes[n];

    /* Check leaf nodes marked for topology update */
    if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology) &&
        !(node->flag & PBVH_FullyHidden)) {
      GSetIterator gs_iter;

      GSET_ITER (gs_iter, node->tm_faces) {
        TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

        f->threadtag = face_on_node_boundary(pbvh, f, n, 0) ? 1 : 0;
      }
    }
  }

  for (int step = 0; step < 3; step++) {
    for (int n = 0; n < pbvh->totnode; n++) {
      PBVHNode *node = &pbvh->nodes[n];

      /* Check leaf nodes marked for topology update */
      if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology) &&
          !(node->flag & PBVH_FullyHidden)) {
        GSetIterator gs_iter;

        GSET_ITER (gs_iter, node->tm_faces) {
          TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

          if (f->threadtag == step + 1) {
            for (int i = 0; i < 3; i++) {
              TMEdge *e = TM_GET_TRI_EDGE(f, i);

              for (int j = 0; j < e->tris.length; j++) {
                TMFace *f2 = e->tris.items[j];

                f2->threadtag = !f2->threadtag ? step + 2 : f2->threadtag;
              }
            }
          }
        }
      }
    }
  }
  for (int n = 0; n < pbvh->totnode; n++) {
    PBVHNode *node = &pbvh->nodes[n];

    /* Check leaf nodes marked for topology update */
    if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology) &&
        !(node->flag & PBVH_FullyHidden)) {
      GSetIterator gs_iter;

      TMFace **tris = NULL;
      BLI_array_declare(tris);

      /* Check each face */
      GSET_ITER (gs_iter, node->tm_faces) {
        TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
        if (f->threadtag) {
          BLI_array_append(boundary, f);
        }
        else {
          BLI_array_append(tris, f);
        }
      }

      if (BLI_array_len(tris) > 0) {
        TMTriIsland island;
        island.tottri = BLI_array_len(tris);
        island.tris = tris;

        BLI_array_append(islands, island);
      }
    }
  }

  ThreadSplitData tdata = {.pool = BLI_safepool_create(sizeof(TMVert *) * 2, 512, tm->maxthread)};
  tdata.pbvh = pbvh;
  tdata.cd_face_node_offset = pbvh->cd_face_node_offset;
  tdata.cd_vert_node_offset = pbvh->cd_vert_node_offset;
  tdata.use_frontface = use_frontface;
  tdata.use_projected = use_projected;
  copy_v3_v3(tdata.center, center);
  copy_v3_v3(tdata.view_normal, view_normal);
  tdata.radius = radius;

  if (CustomData_has_layer(&tm->vdata, CD_PAINT_MASK)) {
    tdata.cd_vert_mask_offset = CustomData_get_offset(&tm->vdata, CD_PAINT_MASK);
  }
  else {
    tdata.cd_vert_mask_offset = -1;
  }
  tdata.islands = islands;

  if (BLI_array_len(islands) > 0) {
    TaskParallelSettings settings;
    BLI_parallel_range_settings_defaults(&settings);
    settings.use_threading = false;

    // printf("Total islands: %d\n", BLI_array_len(islands));
    BLI_task_parallel_range(0, BLI_array_len(islands), &tdata, longqueue_job, &settings);
  }

  if (boundary) {
    TMTriIsland island;
    island.tris = boundary;
    island.tottri = BLI_array_len(boundary);

    BLI_array_append(islands, island);

    tdata.islands = islands;

    longqueue_job(&tdata, BLI_array_len(islands) - 1, NULL);
  }

  for (int i = 0; i < BLI_array_len(islands) - 1; i++) {
    MEM_freeN(islands[i].tris);
  }

  BLI_array_free(islands);
  BLI_array_free(boundary);

  BLI_safepool_destroy(tdata.pool);

  return true;
}

/* Create a priority queue containing vertex pairs connected by a
 * short edge as defined by PBVH.bm_min_edge_len.
 *
 * Only nodes marked for topology update are checked, and in those
 * nodes only edges used by a face intersecting the (center, radius)
 * sphere are checked.
 *
 * The highest priority (lowest number) is given to the shortest edge.
 */
static void short_edge_queue_create(EdgeQueueContext *eq_ctx,
                                    PBVH *bvh,
                                    const float center[3],
                                    const float view_normal[3],
                                    float radius,
                                    const bool use_frontface,
                                    const bool use_projected)
{
  eq_ctx->q->elems = eq_ctx->q->totelems = 0;
  eq_ctx->q->heap = BLI_heapsimple_new();
  eq_ctx->q->center = center;
  eq_ctx->q->radius_squared = radius * radius;
  eq_ctx->q->limit_len_squared = bvh->bm_min_edge_len * bvh->bm_min_edge_len;
#ifdef USE_EDGEQUEUE_EVEN_SUBDIV
  eq_ctx->q->limit_len = bvh->bm_min_edge_len;
#endif

  eq_ctx->q->view_normal = view_normal;

#ifdef USE_EDGEQUEUE_FRONTFACE
  eq_ctx->q->use_view_normal = use_frontface;
#else
  UNUSED_VARS(use_frontface);
#endif

  if (use_projected) {
    eq_ctx->q->edge_queue_tri_in_range = edge_queue_tri_in_circle;
    project_plane_normalized_v3_v3v3(eq_ctx->q->center_proj, center, view_normal);
  }
  else {
    eq_ctx->q->edge_queue_tri_in_range = edge_queue_tri_in_sphere;
  }

  for (int n = 0; n < bvh->totnode; n++) {
    PBVHNode *node = &bvh->nodes[n];

    /* Check leaf nodes marked for topology update */
    if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology) &&
        !(node->flag & PBVH_FullyHidden)) {
      GSetIterator gs_iter;

      /* Check each face */
      GSET_ITER (gs_iter, node->tm_faces) {
        TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

        short_edge_queue_face_add(eq_ctx, f);
      }
    }
  }
}

/*************************** Topology update **************************/

static void pbvh_trimesh_split_edge(EdgeQueueContext *eq_ctx, PBVH *bvh, TMEdge *e)
{

#if 1

  float co_mid[3], no_mid[3];

  /* Get all faces adjacent to the edge */
  // pbvh_trimesh_edge_loops(edge_loops, e);

  /* Create a new vertex in current node at the edge's midpoint */
  mid_v3_v3v3(co_mid, e->v1->co, e->v2->co);
  mid_v3_v3v3(no_mid, e->v1->no, e->v2->no);
  normalize_v3(no_mid);

  int node_index = TM_ELEM_CD_GET_INT(e->v1, eq_ctx->cd_vert_node_offset);

  TMVert *v_new = pbvh_trimesh_vert_create(
      bvh, node_index, co_mid, no_mid, eq_ctx->cd_vert_mask_offset);

#  ifdef DYNTOPO_CD_INTERP
  void *vsrcs[2] = {e->v1->customdata, e->v2->customdata};
  float vws[2] = {0.5f, 0.5f};
  CustomData_bmesh_interp(&bvh->tm->vdata, vsrcs, vws, NULL, 2, v_new->customdata);
#  endif

  // return;

  /* update paint mask */
  if (eq_ctx->cd_vert_mask_offset != -1) {
    float mask_v1 = TM_ELEM_CD_GET_FLOAT(e->v1, eq_ctx->cd_vert_mask_offset);
    float mask_v2 = TM_ELEM_CD_GET_FLOAT(e->v2, eq_ctx->cd_vert_mask_offset);
    float mask_v_new = 0.5f * (mask_v1 + mask_v2);

    TM_ELEM_CD_SET_FLOAT(v_new, eq_ctx->cd_vert_mask_offset, mask_v_new);
  }

  bool first = true;

  TMEdge *e1 = TM_get_edge(bvh->tm, e->v1, v_new, false);
  TMEdge *e2 = TM_get_edge(bvh->tm, v_new, e->v2, false);
  int eflag = e->flag & ~TM_ELEM_HIDDEN;
  int vflag = (e->v1->flag | e->v2->flag) & ~TM_ELEM_HIDDEN;

  e1->flag = e2->flag = eflag;
  v_new->flag = vflag;

  /* For each face, add two new triangles and delete the original */
  for (int i = 0; i < e->tris.length;) {
    TMFace *f_adj = e->tris.items[i];
    TMFace *f_new;
    TMVert *v_opp, *v1, *v2;
    TMVert *v_tri[3];
    TMEdge *e_tri[3];

    BLI_assert(f_adj->len == 3);
    int ni = TM_ELEM_CD_GET_INT(f_adj, eq_ctx->cd_face_node_offset);

    if (ni < 0) {
      i++;
      continue;
    }
    /* Find the vertex not in the edge */
    v_opp = TM_getAdjVert(e, f_adj);

    /* Get e->v1 and e->v2 in the order they appear in the
     * existing face so that the new faces' winding orders
     * match */

    TM_orderEdgeInTri(f_adj, e, &v1, &v2);

    // v1 = ;
    // v2 = l_adj->next->v;

    if (ni != node_index && first) {
      pbvh_trimesh_vert_ownership_transfer(bvh, &bvh->nodes[ni], v_new);
    }
    first = false;

    /**
     * The 2 new faces created and assigned to ``f_new`` have their
     * verts & edges shuffled around.
     *
     * - faces wind anticlockwise in this example.
     * - original edge is ``(v1, v2)``
     * - original face is ``(v1, v2, v3)``
     *
     * <pre>
     *         + v3(v_opp)
     *        /|\
     *       / | \
     *      /  |  \
     *   e4/   |   \ e3
     *    /    |e5  \
     *   /     |     \
     *  /  e1  |  e2  \
     * +-------+-------+
     * v1      v4(v_new) v2
     *  (first) (second)
     * </pre>
     *
     * - f_new (first):  ``v_tri=(v1, v4, v3), e_tri=(e1, e5, e4)``
     * - f_new (second): ``v_tri=(v4, v2, v3), e_tri=(e2, e3, e5)``
     */

    /* Create two new faces */
    v_tri[0] = v1;
    v_tri[1] = v_new;
    v_tri[2] = v_opp;

    tm_edges_from_tri(bvh->tm, v_tri, e_tri, 0, false);
    f_new = pbvh_trimesh_face_create(bvh, ni, v_tri, e_tri, f_adj);

#  ifdef DYNTOPO_CD_INTERP
    TMLoopData *l1 = TM_GET_TRI_LOOP_VERTEX(f_adj, v1);
    TMLoopData *l2 = TM_GET_TRI_LOOP_VERTEX(f_adj, v2);
    TMLoopData *l3 = TM_GET_TRI_LOOP_VERTEX(f_adj, v_opp);

    void *lsrcs[2] = {l1->customdata, l2->customdata};
    float lws[2] = {0.5f, 0.5f};

    CustomData_bmesh_interp(&bvh->tm->ldata, lsrcs, lws, lws, 2, f_new->l2->customdata);

    lsrcs[0] = l1->customdata;
    lws[0] = 1.0f;

    CustomData_bmesh_interp(&bvh->tm->ldata, lsrcs, lws, lws, 1, f_new->l1->customdata);

    lsrcs[0] = l3->customdata;
    lws[0] = 1.0f;

    CustomData_bmesh_interp(&bvh->tm->ldata, lsrcs, lws, lws, 1, f_new->l3->customdata);

#  endif

    // long_edge_queue_face_add(eq_ctx, f_new);

    v_tri[0] = v_new;
    v_tri[1] = v2;
    /* v_tri[2] = v_opp; */ /* unchanged */
    e_tri[0] = TM_get_edge(bvh->tm, v_tri[0], v_tri[1], false);
    e_tri[2] = e_tri[1]; /* switched */
    e_tri[1] = TM_get_edge(bvh->tm, v_tri[1], v_tri[2], false);
    f_new = pbvh_trimesh_face_create(bvh, ni, v_tri, e_tri, f_adj);

#  ifdef DYNTOPO_CD_INTERP
    lsrcs[0] = l1->customdata;
    lsrcs[1] = l2->customdata;
    lws[0] = lws[1] = 0.5f;

    CustomData_bmesh_interp(&bvh->tm->ldata, lsrcs, lws, lws, 2, f_new->l1->customdata);

    lsrcs[0] = l2->customdata;
    lws[0] = 1.0f;

    CustomData_bmesh_interp(&bvh->tm->ldata, lsrcs, lws, lws, 1, f_new->l2->customdata);

    lsrcs[0] = l3->customdata;
    lws[0] = 1.0f;

    CustomData_bmesh_interp(&bvh->tm->ldata, lsrcs, lws, lws, 1, f_new->l3->customdata);

#  endif

    // long_edge_queue_face_add(eq_ctx, f_new);

    /* Delete original */
    pbvh_trimesh_face_remove(bvh, f_adj);
    TM_kill_tri(bvh->tm, f_adj, false, false);

    /* Ensure new vertex is in the node */
    if (!BLI_table_gset_has(bvh->nodes[ni].tm_unique_verts, v_new)) {
      BLI_table_gset_add(bvh->nodes[ni].tm_other_verts, v_new);
    }

    if (v_opp->edges.length > 8) {
      // prevent degenerate cases that make lots of geometry
      int len = MIN2(v_opp->edges.length, 512);

      for (int i = 0; i < len; i++) {
        TMEdge *e2 = v_opp->edges.items[i];

        if (e2 != e) {
          // long_edge_queue_edge_add(eq_ctx, e2);
        }
        else {
          printf("eek! %s %d\n", __FILE__, __LINE__);
          continue;
        }
      }
    }
  }

  TM_kill_edge(bvh->tm, e, false);
#endif
}

static bool pbvh_trimesh_subdivide_long_edges(EdgeQueueContext *eq_ctx, PBVH *bvh)
{
  bool any_subdivided = false;
  double time = PIL_check_seconds_timer();

  while (!BLI_heapsimple_is_empty(eq_ctx->q->heap)) {
    TMVert **pair = BLI_heapsimple_pop_min(eq_ctx->q->heap);
    TMVert *v1 = pair[0], *v2 = pair[1];
    TMEdge *e;

    if (PIL_check_seconds_timer() - time > DYNTOPO_TIME_LIMIT) {
      break;
    }

    BLI_mempool_free(eq_ctx->pool, pair);
    pair = NULL;

    /* Check that the edge still exists */
    if (!(e = TM_edge_exists(v1, v2))) {
      continue;
    }
#ifdef USE_EDGEQUEUE_TAG
    EDGE_QUEUE_DISABLE(e);
#endif

    /* At the moment edges never get shorter (subdiv will make new edges)
     * unlike collapse where edges can become longer. */
#if 0
    if (len_squared_v3v3(v1->co, v2->co) <= eq_ctx->q->limit_len_squared) {
      continue;
    }
#else
    BLI_assert(len_squared_v3v3(v1->co, v2->co) > eq_ctx->q->limit_len_squared);
#endif

    /* Check that the edge's vertices are still in the PBVH. It's
     * possible that an edge collapse has deleted adjacent faces
     * and the node has been split, thus leaving wire edges and
     * associated vertices. */
    if ((TM_ELEM_CD_GET_INT(e->v1, eq_ctx->cd_vert_node_offset) == DYNTOPO_NODE_NONE) ||
        (TM_ELEM_CD_GET_INT(e->v2, eq_ctx->cd_vert_node_offset) == DYNTOPO_NODE_NONE)) {
      continue;
    }

    any_subdivided = true;

    pbvh_trimesh_split_edge(eq_ctx, bvh, e);
  }

#ifdef USE_EDGEQUEUE_TAG_VERIFY
  pbvh_trimesh_edge_tag_verify(bvh);
#endif

  return any_subdivided;
}

static void pbvh_trimesh_collapse_edge(PBVH *bvh,
                                       TMEdge *e,
                                       TMVert *v1,
                                       TMVert *v2,
                                       GHash *deleted_verts,
                                       BLI_Buffer *deleted_faces,
                                       EdgeQueueContext *eq_ctx)
{
  TMVert *v_del, *v_conn;

#ifdef DYNTOPO_CD_INTERP
  if (e->flag & TRIMESH_SEAM) {
    //return;
    // only collapse edge if there are two seams in its neighborhood
    for (int step = 0; step < 2; step++) {
      int count = 0;
      TMVert *v = step ? v2 : v1;

      for (int i = 0; i < v->edges.length; i++) {
        TMEdge *e2 = v->edges.items[i];

        if (e2->flag & TRIMESH_SEAM) {
          count++;
        }
      }

      if (count < 2) {
        return;
      }
    }
  }
#endif

  /* one of the two vertices may be masked, select the correct one for deletion */
  if (TM_ELEM_CD_GET_FLOAT(v1, eq_ctx->cd_vert_mask_offset) <
      TM_ELEM_CD_GET_FLOAT(v2, eq_ctx->cd_vert_mask_offset)) {
    v_del = v1;
    v_conn = v2;
  }
  else {
    v_del = v2;
    v_conn = v1;
  }

  /* Remove the merge vertex from the PBVH */
  pbvh_trimesh_vert_remove(bvh, v_del);

  /* Remove all faces adjacent to the edge */
  while (e->tris.length) {
    int lastlen = e->tris.length;
    TMFace *f_adj = e->tris.items[0];

#ifdef DYNTOPO_CD_INTERP
    int eflag = 0;

    // propegate flags to merged edges
    for (int i = 0; i < 3; i++) {
      TMEdge *e2 = TM_GET_TRI_EDGE(f_adj, i);

      if (e2 != e) {
        eflag |= e2->flag & ~TM_ELEM_HIDDEN;
      }
    }

    for (int i = 0; i < 3; i++) {
      TMEdge *e2 = TM_GET_TRI_EDGE(f_adj, i);
      e2->flag |= eflag;
    }
#endif
    pbvh_trimesh_face_remove(bvh, f_adj);

    // XXX check threadnr argument!
    TM_kill_tri(bvh->tm, f_adj, false, false);

    if (lastlen == e->tris.length) {  // paranoia check
      printf("eek! %s %d\n", __FILE__, __LINE__);
      break;
    }
  }

  /* Kill the edge */
  BLI_assert(TM_edge_is_wire(e));

  TM_kill_edge(bvh->tm, e, false);

  /* For all remaining faces of v_del, create a new face that is the
   * same except it uses v_conn instead of v_del */
  /* Note: this could be done with BM_vert_splice(), but that
   * requires handling other issues like duplicate edges, so doesn't
   * really buy anything. */
  BLI_buffer_clear(deleted_faces);

  TM_ITER_VERT_TRIEDGES (v_del, tri, e) {
    tri->flag &= ~TRIMESH_TEMP_TAG;

    TMEdge *e2 = TM_nextEdgeInTri(tri, e);
    for (int i = 0; i < e2->tris.length; i++) {
      TMFace *tri2 = e2->tris.items[i];

      tri2->flag &= ~TRIMESH_TEMP_TAG;

      for (int j = 0; j < 3; j++) {
        TMEdge *e3 = TM_GET_TRI_EDGE(tri2, j);
        for (int k = 0; k < e3->tris.length; k++) {
          TMFace *tri3 = e3->tris.items[k];

          tri3->flag &= ~TRIMESH_TEMP_TAG;
        }
      }
    }
  }
  TM_ITER_VERT_TRIEDGES_END

  TM_ITER_VERT_TRIEDGES (v_del, tri, e) {
    TMEdge *e2 = TM_nextEdgeInTri(tri, e);
    TMFace *existing_face = NULL;
    TMVert *v2 = TM_nextVertInTri(tri, v_del);

    if (v2 == TM_other_vert(e, v_del)) {
      v2 = TM_prevVertInTri(tri, v_del);
    }

    if (UNLIKELY(existing_face = TM_tri_exists(v_conn, v2, v_del))) {
      if (!(existing_face->flag & TRIMESH_TEMP_TAG)) {
        BLI_buffer_append(deleted_faces, TMFace *, existing_face);
        existing_face->flag |= TRIMESH_TEMP_TAG;
      }
      else {
        printf("eek! %s %d\n", __FILE__, __LINE__);
      }
    }
    else if (1) {
      TMVert *v_tri[3] = {v_conn, TM_nextVertInTri(tri, v_del), TM_prevVertInTri(tri, v_del)};
      TMVert *v_triold[3] = {v_del, TM_nextVertInTri(tri, v_del), TM_prevVertInTri(tri, v_del)};

      // BLI_assert(!BM_face_exists(v_tri, 3));
      TMEdge *e_tri[3], *e_triold[3];
      PBVHNode *n = pbvh_trimesh_node_from_face(bvh, tri);
      int ni = (int)(n - bvh->nodes);

      tm_edges_from_tri(bvh->tm, v_tri, e_tri, 0, true);
      tm_edges_from_tri(bvh->tm, v_triold, e_triold, 0, true);


      e_tri[0]->flag |= e_triold[0]->flag;
      e_tri[1]->flag |= e_triold[1]->flag;
      e_tri[2]->flag |= e_triold[2]->flag;

      pbvh_trimesh_face_create(bvh, ni, v_tri, e_tri, tri);

      /* Ensure that v_conn is in the new face's node */
      if (!BLI_table_gset_has(n->tm_unique_verts, v_conn)) {
        BLI_table_gset_has(n->tm_other_verts, v_conn);
      }
    }

    if (!(tri->flag & TRIMESH_TEMP_TAG)) {
      tri->flag |= TRIMESH_TEMP_TAG;
      BLI_buffer_append(deleted_faces, TMFace *, tri);
    }
  }
  TM_ITER_VERT_TRIEDGES_END

  /* Delete the tagged faces */
  for (int i = 0; i < deleted_faces->count; i++) {
    TMFace *f_del = BLI_buffer_at(deleted_faces, TMFace *, i);

    /* Get vertices and edges of face */
    BLI_assert(f_del->len == 3);
    TMVert *v_tri[3];
    TMEdge *e_tri[3];

    v_tri[0] = f_del->v1;
    v_tri[1] = f_del->v2;
    v_tri[2] = f_del->v3;

    e_tri[0] = f_del->e1;
    e_tri[1] = f_del->e2;
    e_tri[2] = f_del->e3;

    /* Remove the face */
    pbvh_trimesh_face_remove(bvh, f_del);

    TM_kill_tri(bvh->tm, f_del, false, false);

    /* Check if any of the face's edges are now unused by any
     * face, if so delete them */
    for (int j = 0; j < 3; j++) {
      if (TM_edge_is_wire(e_tri[j])) {
        TM_kill_edge(bvh->tm, e_tri[j], false);
      }
    }

    /* Check if any of the face's vertices are now unused, if so
     * remove them from the PBVH */
    for (int j = 0; j < 3; j++) {
      if ((v_tri[j] != v_del) && (v_tri[j]->edges.length == 0)) {
        pbvh_trimesh_vert_remove(bvh, v_tri[j]);

        BLI_trimesh_log_vert_kill(bvh->tm_log, v_tri[j]);

        if (v_tri[j] == v_conn) {
          v_conn = NULL;
        }

        BLI_ghash_insert(deleted_verts, v_tri[j], NULL);
        TM_kill_vert(bvh->tm, v_tri[j]);  // XXX check threadnr
      }
    }
  }

  for (int i = 0; i < v_del->edges.length; i++) {
    TMEdge *e2 = v_del->edges.items[i];

    if (e2 != e && TM_edge_is_wire(e2)) {
      TMVert *v2 = TM_other_vert(e2, v_del);

      TM_kill_edge(bvh->tm, e2, false);

      if (v2 != v_conn && v2->edges.length == 0) {
        pbvh_trimesh_vert_remove(bvh, v2);
        BLI_trimesh_log_vert_kill(bvh->tm_log, v2);
        BLI_ghash_insert(deleted_verts, v2, NULL);
        TM_kill_vert(bvh->tm, v2);  // XXX check threadnr
      }
      i--;
      continue;
    }
  }

  /* Move v_conn to the midpoint of v_conn and v_del (if v_conn still exists, it
   * may have been deleted above) */
  if (v_conn != NULL) {
    BLI_trimesh_log_vert_state(bvh->tm_log, v_conn);

    mid_v3_v3v3(v_conn->co, v_conn->co, v_del->co);
    add_v3_v3(v_conn->no, v_del->no);
    normalize_v3(v_conn->no);

    /* update boundboxes attached to the connected vertex
     * note that we can often get-away without this but causes T48779 */
    for (int i = 0; i < v_conn->edges.length; i++) {
      TMEdge *e = v_conn->edges.items[i];

      for (int j = 0; j < e->tris.length; j++) {
        TMFace *f = e->tris.items[j];

        PBVHNode *f_node = pbvh_trimesh_node_from_face(bvh, f);
        f_node->flag |= PBVH_UpdateDrawBuffers | PBVH_UpdateNormals | PBVH_UpdateBB;
      }
    }
  }

  /* Delete v_del */
  BLI_assert(!TM_vert_face_check(v_del));
  if (TM_vert_face_check(v_del)) {
    printf("eek! %s %d\n", __FILE__, __LINE__);
  }

  BLI_trimesh_log_vert_kill(bvh->tm_log, v_del);

  /* v_conn == NULL is OK */
  BLI_ghash_insert(deleted_verts, v_del, v_conn);
  TM_kill_vert(bvh->tm, v_del);  // XXX threadnr
}

static bool pbvh_trimesh_collapse_short_edges(EdgeQueueContext *eq_ctx,
                                              PBVH *bvh,
                                              BLI_Buffer *deleted_faces)
{
  const float min_len_squared = bvh->bm_min_edge_len * bvh->bm_min_edge_len;
  bool any_collapsed = false;
  /* deleted verts point to vertices they were merged into, or NULL when removed. */
  GHash *deleted_verts = BLI_ghash_ptr_new("deleted_verts");

  double time = PIL_check_seconds_timer();

  struct RNG *rng = BLI_rng_new(PIL_check_seconds_timer_i());

  //}
  int step = 0;
#if 1
  while (!BLI_heapsimple_is_empty(eq_ctx->q->heap)) {
    step++;
    // if (step++ > 100) {
    //  break;
    //}

    if (PIL_check_seconds_timer() - time > DYNTOPO_TIME_LIMIT) {
      break;
    }
    TMVert **pair = BLI_heapsimple_pop_min(eq_ctx->q->heap);

#else
  int ilen = eq_ctx->q->totelems;

  for (int i = 0; i < ilen; i++) {
    if (i >= eq_ctx->q->totelems) {
      break;
    }

    int ri = i;
    // ri = BLI_rng_get_int(rng) % eq_ctx->q->totelems;
    TMVert **pair = eq_ctx->q->elems[ri];

    // eq_ctx->q->elems[ri] = eq_ctx->q->elems[eq_ctx->q->totelems-1];
    // eq_ctx->q->totelems--;
#endif
    TMVert *v1 = pair[0], *v2 = pair[1];

    BLI_mempool_free(eq_ctx->pool, pair);
    pair = NULL;

    /* Check the verts still exist */
    if (!(v1 = tm_vert_hash_lookup_chain(deleted_verts, v1)) ||
        !(v2 = tm_vert_hash_lookup_chain(deleted_verts, v2)) || (v1 == v2)) {
      continue;
    }

    /* Check that the edge still exists */
    TMEdge *e;
    if (!(e = TM_edge_exists(v1, v2))) {
      continue;
    }

#if 0
    for (int i = 0; i < 2; i++) {
      float co[3], tot = 0.0f;
      TMVert *v = i ? v2 : v1;
      co[0] = co[1] = co[2] = 0.0f;

      for (int j = 0; j < v->edges.length; j++) {
        TMEdge *e2 = v->edges.items[j];
        TMVert *v3 = TM_other_vert(e2, v);

        add_v3_v3(co, v3->co);
        tot += 1.0;
      }

      if (tot == 0.0f)
        continue;

      mul_v3_fl(co, 1.0f / tot);
      interp_v3_v3v3(v->co, v->co, co, 0.75);
    }
#endif

#ifdef USE_EDGEQUEUE_TAG
    EDGE_QUEUE_DISABLE(e);
#endif

    if (calc_weighted_edge(eq_ctx, e) >= min_len_squared) {
      continue;
    }

    /* Check that the edge's vertices are still in the PBVH. It's
     * possible that an edge collapse has deleted adjacent faces
     * and the node has been split, thus leaving wire edges and
     * associated vertices. */
    if ((TM_ELEM_CD_GET_INT(e->v1, eq_ctx->cd_vert_node_offset) == DYNTOPO_NODE_NONE) ||
        (TM_ELEM_CD_GET_INT(e->v2, eq_ctx->cd_vert_node_offset) == DYNTOPO_NODE_NONE)) {
      continue;
    }

    any_collapsed = true;

    pbvh_trimesh_collapse_edge(bvh, e, v1, v2, deleted_verts, deleted_faces, eq_ctx);
  }

  printf("collapse steps: %d\n", step);

  BLI_ghash_free(deleted_verts, NULL, NULL);
  BLI_rng_free(rng);

  return any_collapsed;
}

/************************* Called from pbvh.c *************************/

bool pbvh_trimesh_node_raycast(PBVHNode *node,
                               const float ray_start[3],
                               const float ray_normal[3],
                               struct IsectRayPrecalc *isect_precalc,
                               float *depth,
                               bool use_original,
                               SculptIdx *r_active_vertex_index,
                               float *r_face_normal)
{
  bool hit = false;
  float nearest_vertex_co[3] = {0.0f};

  if (use_original && node->tm_tot_ortri) {
    for (int i = 0; i < node->tm_tot_ortri; i++) {
      const int *t = node->tm_ortri[i];
      hit |= ray_face_intersection_tri(ray_start,
                                       isect_precalc,
                                       node->tm_orco[t[0]],
                                       node->tm_orco[t[1]],
                                       node->tm_orco[t[2]],
                                       depth);
    }
  }
  else {
    GSetIterator gs_iter;

    GSET_ITER (gs_iter, node->tm_faces) {
      TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

      BLI_assert(f->len == 3);
      if (!TM_elem_flag_test(f, TM_ELEM_HIDDEN)) {
        TMVert **v_tri = &f->v1;

        if (ray_face_intersection_tri(
                ray_start, isect_precalc, v_tri[0]->co, v_tri[1]->co, v_tri[2]->co, depth)) {
          hit = true;

          if (r_face_normal) {
            normal_tri_v3(r_face_normal, v_tri[0]->co, v_tri[1]->co, v_tri[2]->co);
          }

          if (r_active_vertex_index) {
            float location[3] = {0.0f};
            madd_v3_v3v3fl(location, ray_start, ray_normal, *depth);
            for (int j = 0; j < 3; j++) {
              if (len_squared_v3v3(location, v_tri[j]->co) <
                  len_squared_v3v3(location, nearest_vertex_co)) {
                copy_v3_v3(nearest_vertex_co, v_tri[j]->co);
                *r_active_vertex_index = (SculptIdx)v_tri[j];
              }
            }
          }
        }
      }
    }
  }

  return hit;
}

bool BKE_pbvh_trimesh_node_raycast_detail(PBVHNode *node,
                                          const float ray_start[3],
                                          struct IsectRayPrecalc *isect_precalc,
                                          float *depth,
                                          float *r_edge_length)
{
  if (node->flag & PBVH_FullyHidden) {
    return 0;
  }

  GSetIterator gs_iter;
  bool hit = false;
  TMFace *f_hit = NULL;

  GSET_ITER (gs_iter, node->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

    BLI_assert(f->len == 3);
    if (!TM_elem_flag_test(f, TM_ELEM_HIDDEN)) {
      TMVert **v_tri = &f->v1;
      bool hit_local;
      hit_local = ray_face_intersection_tri(
          ray_start, isect_precalc, v_tri[0]->co, v_tri[1]->co, v_tri[2]->co, depth);

      if (hit_local) {
        f_hit = f;
        hit = true;
      }
    }
  }

  if (hit) {
    TMVert **v_tri = &f_hit->v1;

    float len1 = len_squared_v3v3(v_tri[0]->co, v_tri[1]->co);
    float len2 = len_squared_v3v3(v_tri[1]->co, v_tri[2]->co);
    float len3 = len_squared_v3v3(v_tri[2]->co, v_tri[0]->co);

    /* detail returned will be set to the maximum allowed size, so take max here */
    *r_edge_length = sqrtf(max_fff(len1, len2, len3));
  }

  return hit;
}

bool pbvh_trimesh_node_nearest_to_ray(PBVHNode *node,
                                      const float ray_start[3],
                                      const float ray_normal[3],
                                      float *depth,
                                      float *dist_sq,
                                      bool use_original)
{
  bool hit = false;

  if (use_original && node->tm_tot_ortri) {
    for (int i = 0; i < node->tm_tot_ortri; i++) {
      const int *t = node->tm_ortri[i];
      hit |= ray_face_nearest_tri(ray_start,
                                  ray_normal,
                                  node->tm_orco[t[0]],
                                  node->tm_orco[t[1]],
                                  node->tm_orco[t[2]],
                                  depth,
                                  dist_sq);
    }
  }
  else {
    GSetIterator gs_iter;

    GSET_ITER (gs_iter, node->tm_faces) {
      TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

      if (!TM_elem_flag_test(f, TM_ELEM_HIDDEN)) {
        hit |= ray_face_nearest_tri(
            ray_start, ray_normal, f->v1->co, f->v2->co, f->v3->co, depth, dist_sq);
      }
    }
  }

  return hit;
}

void pbvh_trimesh_normals_update(PBVHNode **nodes, int totnode)
{
  for (int n = 0; n < totnode; n++) {
    PBVHNode *node = nodes[n];

    if (node->flag & PBVH_UpdateNormals) {
      TMVert *v;
      GSetIterator gs_iter;

      GSET_ITER (gs_iter, node->tm_faces) {
        TM_calc_tri_normal(BLI_gsetIterator_getKey(&gs_iter));
      }

      TMS_ITER (v, node->tm_unique_verts) {
        TM_calc_vert_normal(v, false);
      }
      TMS_ITER_END

      /* This should be unneeded normally */
      TMS_ITER (v, node->tm_other_verts) {
        TM_calc_vert_normal(v, false);
      }
      TMS_ITER_END

      node->flag &= ~PBVH_UpdateNormals;
    }
  }
}

struct FastNodeBuildInfo {
  int totface; /* number of faces */
  int start;   /* start of faces in array */
  struct FastNodeBuildInfo *child1;
  struct FastNodeBuildInfo *child2;
};

/**
 * Recursively split the node if it exceeds the leaf_limit.
 * This function is multi-thread-able since each invocation applies
 * to a sub part of the arrays.
 */
static void pbvh_trimesh_node_limit_ensure_fast(
    PBVH *bvh, TMFace **nodeinfo, BBC *bbc_array, struct FastNodeBuildInfo *node, MemArena *arena)
{
  struct FastNodeBuildInfo *child1, *child2;

  if (node->totface <= bvh->leaf_limit) {
    return;
  }

  /* Calculate bounding box around primitive centroids */
  BB cb;
  BB_reset(&cb);
  for (int i = 0; i < node->totface; i++) {
    TMFace *f = nodeinfo[i + node->start];
    BBC *bbc = &bbc_array[f->index];

    BB_expand(&cb, bbc->bcentroid);
  }

  /* initialize the children */

  /* Find widest axis and its midpoint */
  const int axis = BB_widest_axis(&cb);
  const float mid = (cb.bmax[axis] + cb.bmin[axis]) * 0.5f;

  int num_child1 = 0, num_child2 = 0;

  /* split vertices along the middle line */
  const int end = node->start + node->totface;
  for (int i = node->start; i < end - num_child2; i++) {
    TMFace *f = nodeinfo[i];
    BBC *bbc = &bbc_array[f->index];

    if (bbc->bcentroid[axis] > mid) {
      int i_iter = end - num_child2 - 1;
      int candidate = -1;
      /* found a face that should be part of another node, look for a face to substitute with */

      for (; i_iter > i; i_iter--) {
        TMFace *f_iter = nodeinfo[i_iter];
        const BBC *bbc_iter = &bbc_array[f_iter->index];
        if (bbc_iter->bcentroid[axis] <= mid) {
          candidate = i_iter;
          break;
        }
        else {
          num_child2++;
        }
      }

      if (candidate != -1) {
        TMFace *tmp = nodeinfo[i];
        nodeinfo[i] = nodeinfo[candidate];
        nodeinfo[candidate] = tmp;
        /* increase both counts */
        num_child1++;
        num_child2++;
      }
      else {
        /* not finding candidate means second half of array part is full of
         * second node parts, just increase the number of child nodes for it */
        num_child2++;
      }
    }
    else {
      num_child1++;
    }
  }

  /* ensure at least one child in each node */
  if (num_child2 == 0) {
    num_child2++;
    num_child1--;
  }
  else if (num_child1 == 0) {
    num_child1++;
    num_child2--;
  }

  /* at this point, faces should have been split along the array range sequentially,
   * each sequential part belonging to one node only */
  BLI_assert((num_child1 + num_child2) == node->totface);

  node->child1 = child1 = BLI_memarena_alloc(arena, sizeof(struct FastNodeBuildInfo));
  node->child2 = child2 = BLI_memarena_alloc(arena, sizeof(struct FastNodeBuildInfo));

  child1->totface = num_child1;
  child1->start = node->start;
  child2->totface = num_child2;
  child2->start = node->start + num_child1;
  child1->child1 = child1->child2 = child2->child1 = child2->child2 = NULL;

  pbvh_trimesh_node_limit_ensure_fast(bvh, nodeinfo, bbc_array, child1, arena);
  pbvh_trimesh_node_limit_ensure_fast(bvh, nodeinfo, bbc_array, child2, arena);
}

static void pbvh_trimesh_create_nodes_fast_recursive(
    PBVH *bvh, TMFace **nodeinfo, BBC *bbc_array, struct FastNodeBuildInfo *node, int node_index)
{
  PBVHNode *n = bvh->nodes + node_index;
  /* two cases, node does not have children or does have children */
  if (node->child1) {
    int children_offset = bvh->totnode;

    n->children_offset = children_offset;
    pbvh_grow_nodes(bvh, bvh->totnode + 2);
    pbvh_trimesh_create_nodes_fast_recursive(
        bvh, nodeinfo, bbc_array, node->child1, children_offset);
    pbvh_trimesh_create_nodes_fast_recursive(
        bvh, nodeinfo, bbc_array, node->child2, children_offset + 1);

    n = &bvh->nodes[node_index];

    /* Update bounding box */
    BB_reset(&n->vb);
    BB_expand_with_bb(&n->vb, &bvh->nodes[n->children_offset].vb);
    BB_expand_with_bb(&n->vb, &bvh->nodes[n->children_offset + 1].vb);
    n->orig_vb = n->vb;
  }
  else {
    /* node does not have children so it's a leaf node, populate with faces and tag accordingly
     * this is an expensive part but it's not so easily thread-able due to vertex node indices */
    const int cd_vert_node_offset = bvh->cd_vert_node_offset;
    const int cd_face_node_offset = bvh->cd_face_node_offset;

    bool has_visible = false;

    n->flag = PBVH_Leaf;
    n->tm_faces = BLI_gset_ptr_new_ex("tm_faces", node->totface);

    /* Create vert hash sets */
    n->tm_unique_verts = BLI_table_gset_new();
    n->tm_other_verts = BLI_table_gset_new();

    BB_reset(&n->vb);

    const int end = node->start + node->totface;

    for (int i = node->start; i < end; i++) {
      TMFace *f = nodeinfo[i];
      BBC *bbc = &bbc_array[f->index];

      /* Update ownership of faces */
      BLI_gset_insert(n->tm_faces, f);
      TM_ELEM_CD_SET_INT(f, cd_face_node_offset, node_index);

      /* Update vertices */
      for (int j = 0; j < 3; j++) {
        TMVert *v = TM_GET_TRI_VERT(f, j);

        if (!BLI_table_gset_has(n->tm_unique_verts, v)) {
          if (TM_ELEM_CD_GET_INT(v, cd_vert_node_offset) != DYNTOPO_NODE_NONE) {
            BLI_table_gset_add(n->tm_other_verts, v);
          }
          else {
            BLI_table_gset_insert(n->tm_unique_verts, v);
            TM_ELEM_CD_SET_INT(v, cd_vert_node_offset, node_index);
          }
        }
        /* Update node bounding box */
      }

      if (!TM_elem_flag_test(f, TM_ELEM_HIDDEN)) {
        has_visible = true;
      }

      BB_expand_with_bb(&n->vb, (BB *)bbc);
    }

    BLI_assert(n->vb.bmin[0] <= n->vb.bmax[0] && n->vb.bmin[1] <= n->vb.bmax[1] &&
               n->vb.bmin[2] <= n->vb.bmax[2]);

    n->orig_vb = n->vb;

    /* Build GPU buffers for new node and update vertex normals */
    BKE_pbvh_node_mark_rebuild_draw(n);

    BKE_pbvh_node_fully_hidden_set(n, !has_visible);
    n->flag |= PBVH_UpdateNormals;
  }
}

/***************************** Public API *****************************/

/* Build a PBVH from a BMesh */
void BKE_pbvh_build_trimesh(PBVH *bvh,
                            TM_TriMesh *tm,
                            bool smooth_shading,
                            TriMeshLog *log,
                            const int cd_vert_node_offset,
                            const int cd_face_node_offset)
{
  bvh->cd_vert_node_offset = cd_vert_node_offset;
  bvh->cd_face_node_offset = cd_face_node_offset;
  bvh->tm = tm;

  BKE_pbvh_bmesh_detail_size_set(bvh, 0.75);

  bvh->type = PBVH_TRIMESH;
  bvh->tm_log = log;

  /* TODO: choose leaf limit better */
  bvh->leaf_limit = 2000;

  if (smooth_shading) {
    bvh->flags |= PBVH_DYNTOPO_SMOOTH_SHADING;
  }

  /* bounding box array of all faces, no need to recalculate every time */
  BBC *bbc_array = MEM_mallocN(sizeof(BBC) * tm->tottri, "BBC");
  TMFace **nodeinfo = MEM_mallocN(sizeof(*nodeinfo) * tm->tottri, "nodeinfo");
  MemArena *arena = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, "fast PBVH node storage");

  TM_TriMeshIter iter;
  TMFace *f;
  int i;

  TM_tri_iternew(tm, &iter);
  f = TM_iterstep(&iter);
  for (i = 0; f; f = TM_iterstep(&iter), i++) {
    BBC *bbc = &bbc_array[i];

    BB_reset((BB *)bbc);
    BB_expand((BB *)bbc, f->v1->co);
    BB_expand((BB *)bbc, f->v2->co);
    BB_expand((BB *)bbc, f->v3->co);
    BBC_update_centroid(bbc);

    /* so we can do direct lookups on 'bbc_array' */
    f->index = i; /* set_dirty! */
    nodeinfo[i] = f;
    TM_ELEM_CD_SET_INT(f, cd_face_node_offset, DYNTOPO_NODE_NONE);
  }
  /* Likely this is already dirty. */
  tm->elem_index_dirty |= TM_TRI;

  TM_vert_iternew(tm, &iter);
  TMVert *v = TM_iterstep(&iter);
  for (; v; v = TM_iterstep(&iter)) {
    TM_ELEM_CD_SET_INT(v, cd_vert_node_offset, DYNTOPO_NODE_NONE);
  }

  /* setup root node */
  struct FastNodeBuildInfo rootnode = {0};
  rootnode.totface = tm->tottri;

  /* start recursion, assign faces to nodes accordingly */
  pbvh_trimesh_node_limit_ensure_fast(bvh, nodeinfo, bbc_array, &rootnode, arena);

  /* We now have all faces assigned to a node,
   * next we need to assign those to the gsets of the nodes. */

  /* Start with all faces in the root node */
  bvh->nodes = MEM_callocN(sizeof(PBVHNode), "PBVHNode");
  bvh->totnode = 1;

  /* take root node and visit and populate children recursively */
  pbvh_trimesh_create_nodes_fast_recursive(bvh, nodeinfo, bbc_array, &rootnode, 0);

  BLI_memarena_free(arena);
  MEM_freeN(bbc_array);
  MEM_freeN(nodeinfo);
}

static double last_update_time[128] = {
    0,
};

/* Collapse short edges, subdivide long edges */
bool BKE_pbvh_trimesh_update_topology(PBVH *bvh,
                                      PBVHTopologyUpdateMode mode,
                                      const float center[3],
                                      const float view_normal[3],
                                      float radius,
                                      const bool use_frontface,
                                      const bool use_projected,
                                      int sym_axis)
{

  if (PIL_check_seconds_timer() - last_update_time[sym_axis] < DYNTOPO_RUN_INTERVAL) {
    return false;
    // return false;
  }

  last_update_time[sym_axis] = PIL_check_seconds_timer();

  // return false;
  /* 2 is enough for edge faces - manifold edge */
  BLI_buffer_declare_static(TMFace *, deleted_faces, BLI_BUFFER_NOP, 32);

  const int cd_vert_mask_offset = CustomData_get_offset(&bvh->tm->vdata, CD_PAINT_MASK);
  const int cd_vert_node_offset = bvh->cd_vert_node_offset;
  const int cd_face_node_offset = bvh->cd_face_node_offset;

  bool modified = false;

  if (view_normal) {
    BLI_assert(len_squared_v3(view_normal) != 0.0f);
  }

  if (1 && (mode & PBVH_Collapse)) {
    EdgeQueue q;
    BLI_mempool *queue_pool = BLI_mempool_create(sizeof(TMVert *) * 2, 0, 128, BLI_MEMPOOL_NOP);
    EdgeQueueContext eq_ctx = {
        &q,
        queue_pool,
        bvh->tm,
        cd_vert_mask_offset,
        cd_vert_node_offset,
        cd_face_node_offset,
    };

    short_edge_queue_create(
        &eq_ctx, bvh, center, view_normal, radius, use_frontface, use_projected);

    modified |= pbvh_trimesh_collapse_short_edges(&eq_ctx, bvh, &deleted_faces);

    BLI_heapsimple_free(q.heap, NULL);
    if (q.elems) {
      MEM_freeN(q.elems);
    }
    BLI_mempool_destroy(queue_pool);
  }

  if (mode & PBVH_Subdivide) {
    EdgeQueue q;
    /*
    BLI_mempool *queue_pool = BLI_mempool_create(sizeof(TMVert *) * 2, 0, 128, BLI_MEMPOOL_NOP);
    EdgeQueueContext eq_ctx = {
        &q,
        queue_pool,
        bvh->tm,
        cd_vert_mask_offset,
        cd_vert_node_offset,
        cd_face_node_offset,
    };*/

    modified |= long_edge_queue_create2(
        bvh, center, view_normal, radius, use_frontface, use_projected);

    // modified |= pbvh_trimesh_subdivide_long_edges(&eq_ctx, bvh);

    // BLI_heapsimple_free(q.heap, NULL);
    // BLI_mempool_destroy(queue_pool);
    // if (q.elems) {
    //  MEM_freeN(q.elems);
    //}
  }

  BLI_buffer_free(&deleted_faces);

#ifdef USE_VERIFY
  pbvh_trimesh_verify(bvh);
#endif

  if (0 && modified) {
    bvh->tm->elem_table_dirty |= TM_VERTEX | TM_EDGE | TM_TRI;
    bvh->tm->elem_index_dirty |= TM_VERTEX | TM_EDGE | TM_TRI;

    TM_mesh_elem_index_ensure(bvh->tm, TM_VERTEX | TM_TRI);
    TM_mesh_elem_table_ensure(bvh->tm, TM_VERTEX | TM_TRI);
  }

  if (modified) {
    for (int i = 0; i < bvh->totnode; i++) {
      PBVHNode *node = bvh->nodes + i;

      if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology) &&
          !(node->flag & PBVH_FullyHidden)) {
        /* Recursively split nodes that have gotten too many
         * elements */
        pbvh_trimesh_node_limit_ensure(bvh, i);
      }

      if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology)) {
        node->flag &= ~PBVH_UpdateTopology;
      }
    }
  }
  else {  // still unmark nodes
    for (int i = 0; i < bvh->totnode; i++) {
      PBVHNode *node = bvh->nodes + i;

      if ((node->flag & PBVH_Leaf) && (node->flag & PBVH_UpdateTopology)) {
        node->flag &= ~PBVH_UpdateTopology;
      }
    }
  }

  return modified;
}

static void tm_face_as_array_index_tri(TMFace *f, int *out)
{
  out[0] = f->v1->index;
  out[1] = f->v2->index;
  out[2] = f->v3->index;
}

/* In order to perform operations on the original node coordinates
 * (currently just raycast), store the node's triangles and vertices.
 *
 * Skips triangles that are hidden. */
void BKE_pbvh_trimesh_node_save_orig(TM_TriMesh *tm, PBVHNode *node)
{
  /* Skip if original coords/triangles are already saved */
  if (node->tm_orco) {
    return;
  }

  const int totvert = node->tm_unique_verts->length + node->tm_other_verts->length;
  const int tottri = BLI_gset_len(node->tm_faces);

  if (node->tm_orco && node->tm_tot_orco == totvert) {
    return;
  }
  else if (node->tm_orco) {
    MEM_freeN(node->tm_orco);
    MEM_freeN(node->tm_ortri);
  }

  node->tm_tot_orco = totvert;

  // float (*orco)[3] = NULL;
  float *orco = NULL;
  BLI_array_declare(orco);

  // node->tm_orco = MEM_mallocN(sizeof(*node->tm_orco) * totvert, __func__);
  node->tm_ortri = MEM_mallocN(sizeof(*node->tm_ortri) * tottri, __func__);

  /* Copy out the vertices and assign a temporary index */
  GSetIterator gs_iter;

  GSET_ITER (gs_iter, node->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

    if (TM_elem_flag_test(f, TM_ELEM_HIDDEN)) {
      continue;
    }

    f->v1->index = f->v2->index = f->v3->index = -1;
  }

  int idx = 0, fidx = 0;
  GSET_ITER (gs_iter, node->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gs_iter);

    if (TM_elem_flag_test(f, TM_ELEM_HIDDEN)) {
      continue;
    }

    for (int i = 0; i < 3; i++) {
      TMVert *v = TM_GET_TRI_VERT(f, i);
      if (v->index == -1) {
        v->index = idx++;

        BLI_array_append(orco, v->co[0]);
        BLI_array_append(orco, v->co[1]);
        BLI_array_append(orco, v->co[2]);
      }

      node->tm_ortri[fidx][i] = v->index;
    }

    fidx++;
  }

  node->tm_orco = orco;

  /* Likely this is already dirty. */
  tm->elem_index_dirty |= TM_VERTEX;

  node->tm_tot_ortri = fidx;
}

static int pbvh_count_subtree_verts(PBVH *pbvh, PBVHNode *n)
{
  if (n->flag & PBVH_Leaf) {
    n->tm_subtree_tottri = BLI_gset_len(
        n->tm_faces);  // n->tm_unique_verts->length + n->tm_other_verts->length;
    return n->tm_subtree_tottri;
  }

  int ni = n->children_offset;

  int ret = pbvh_count_subtree_verts(pbvh, pbvh->nodes + ni);
  ret += pbvh_count_subtree_verts(pbvh, pbvh->nodes + ni + 1);

  n->tm_subtree_tottri = ret;

  return ret;
}

static void pbvh_trimesh_join_subnodes(PBVH *pbvh, PBVHNode *node, PBVHNode *parent)
{
  if (!(node->flag & PBVH_Leaf)) {
    int ni = node->children_offset;

    pbvh_trimesh_join_subnodes(pbvh, pbvh->nodes + ni, parent);
    pbvh_trimesh_join_subnodes(pbvh, pbvh->nodes + ni + 1, parent);

    if (node != parent) {
      node->flag = PBVH_Delete;  // mark for deletion
    }

    return;
  }

  node->flag = PBVH_Delete;  // mark for deletion

  TMVert *v;
  GSetIterator gsiter;

  TMS_ITER (v, node->tm_unique_verts) {
    BLI_table_gset_add(parent->tm_unique_verts, v);

    TM_ELEM_CD_SET_INT(v, pbvh->cd_vert_node_offset, (int)(parent - pbvh->nodes));
  }
  TMS_ITER_END

  TMS_ITER (v, node->tm_other_verts) {
    BLI_table_gset_add(parent->tm_other_verts, v);
  }
  TMS_ITER_END

  printf("  subtotface: %d\n", BLI_gset_len(node->tm_faces));
  GSET_ITER (gsiter, node->tm_faces) {
    TMFace *f = BLI_gsetIterator_getKey(&gsiter);

    BLI_gset_add(parent->tm_faces, f);
    TM_ELEM_CD_SET_INT(f, pbvh->cd_face_node_offset, (int)(parent - pbvh->nodes));
  }
}

static void BKE_pbvh_trimesh_corect_tree(PBVH *pbvh, PBVHNode *node, PBVHNode *parent)
{
  const int size_lower = pbvh->leaf_limit - (pbvh->leaf_limit >> 1);
  const int size_higher = pbvh->leaf_limit + (pbvh->leaf_limit >> 1);

  int totlower = 0;

  if (node->flag & PBVH_Leaf) {
    // pbvh_trimesh_node_limit_ensure(pbvh, (int)(node - pbvh->nodes));
    return;

    // join nodes if subtree lacks verts, unless node is root
  }
  else if (node->tm_subtree_tottri < size_lower && node != pbvh->nodes) {
    node->tm_unique_verts = BLI_table_gset_new();
    node->tm_other_verts = BLI_table_gset_new();
    node->tm_faces = BLI_gset_ptr_new("tm_faces");

    pbvh_trimesh_join_subnodes(pbvh, node, node);

    node->flag = PBVH_Leaf | PBVH_UpdateRedraw | PBVH_UpdateBB | PBVH_UpdateDrawBuffers |
                 PBVH_RebuildDrawBuffers | PBVH_UpdateOriginalBB | PBVH_UpdateMask |
                 PBVH_UpdateVisibility | PBVH_UpdateColor | PBVH_UpdateTopology |
                 PBVH_UpdateNormals;

    TableGSet *other = BLI_table_gset_new();
    TMVert *v;

    node->children_offset = 0;
    node->draw_buffers = NULL;

    // make sure other list isn't overlapping with unique
    TMS_ITER (v, node->tm_other_verts) {
      if (!BLI_table_gset_has(node->tm_unique_verts, v)) {
        BLI_table_gset_insert(other, v);
      }
    }
    TMS_ITER_END

    BLI_table_gset_free(node->tm_other_verts);
    node->tm_other_verts = other;

    BB_reset(&node->vb);
    TMS_ITER (v, node->tm_unique_verts) {
      BB_expand(&node->vb, v->co);
    }
    TMS_ITER_END

    TMS_ITER (v, node->tm_other_verts) {
      BB_expand(&node->vb, v->co);
    }
    TMS_ITER_END

    printf("totface: %d\n", BLI_gset_len(node->tm_faces));
    node->orig_vb = node->vb;

    return;
  }

  int ni = node->children_offset;

  for (int i = 0; i < 2; i++, ni++) {
    PBVHNode *child = pbvh->nodes + ni;
    BKE_pbvh_trimesh_corect_tree(pbvh, child, node);
  }

  if (totlower == 2) {
    printf("join nodes!");
  }
}

void BKE_pbvh_trimesh_after_stroke(PBVH *bvh)
{
  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = bvh->nodes + i;

    /* Free orco/ortri data */
    pbvh_trimesh_node_drop_orig(n);
    n->tm_subtree_tottri = 0;
  }

#if 0

  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = bvh->nodes + i;

    if (n->flag & PBVH_Leaf) {
      /* Recursively split nodes that have gotten too many
       * elements */
      pbvh_trimesh_node_limit_ensure(bvh, i);
    }
  }
#else
  pbvh_count_subtree_verts(bvh, bvh->nodes);

  BKE_pbvh_trimesh_corect_tree(bvh, bvh->nodes, NULL);
  // compact nodes
  int totnode = 0;
  for (int i = 0; i < bvh->totnode; i++) {
    if (bvh->nodes[i].flag != PBVH_Delete) {
      totnode++;
    }
  }

  int *map = MEM_callocN(sizeof(int) * bvh->totnode, "trimesh map temp");

  // build idx map for child offsets
  int j = 0;
  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = bvh->nodes + i;

    if (n->flag != PBVH_Delete) {
      map[i] = j++;
    }
    else {
      if (n->draw_buffers) {
        GPU_pbvh_buffers_free(n->draw_buffers);
      }
      if (n->vert_indices) {
        MEM_freeN((void *)n->vert_indices);
      }
      if (n->face_vert_indices) {
        MEM_freeN((void *)n->face_vert_indices);
      }

      if (n->tm_orco) {
        MEM_freeN(n->tm_orco);
      }

      if (n->tm_ortri) {
        MEM_freeN(n->tm_ortri);
      }

      if (n->tm_unique_verts) {
        BLI_table_gset_free(n->tm_unique_verts);
      }

      if (n->tm_other_verts) {
        BLI_table_gset_free(n->tm_other_verts);
      }

      if (n->tm_faces) {
        BLI_gset_free(n->tm_faces, NULL);
      }
    }
  }

  // compact
  j = 0;
  for (int i = 0; i < bvh->totnode; i++) {
    if (bvh->nodes[i].flag != PBVH_Delete) {
      int i1 = map[bvh->nodes[i].children_offset];
      int i2 = map[bvh->nodes[i].children_offset + 1];

      if (bvh->nodes[i].children_offset >= bvh->totnode) {
        printf("bad child node reference %d->%d\n", i, bvh->nodes[i].children_offset);
        continue;
      }

      if (bvh->nodes[i].children_offset && i2 != i1 + 1) {
        printf("      EVIL %d %d\n", i1, i2);
      }

      bvh->nodes[j] = bvh->nodes[i];
      bvh->nodes[j].children_offset = map[bvh->nodes[j].children_offset];

      j++;
    }
  }

  if (j != totnode) {
    printf("eek!");
  }
  bvh->totnode = j;

  GSetIterator gsiter;
  TMVert *v;

  // set vert/face node indices again
  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = bvh->nodes + i;

    if (!(n->flag & PBVH_Leaf)) {
      continue;
    }

    TMS_ITER (v, n->tm_unique_verts) {
      TM_ELEM_CD_SET_INT(v, bvh->cd_vert_node_offset, i);
    }
    TMS_ITER_END

    GSET_ITER (gsiter, n->tm_faces) {
      TMFace *f = BLI_gsetIterator_getKey(&gsiter);
      TM_ELEM_CD_SET_INT(f, bvh->cd_face_node_offset, i);
    }
  }
  MEM_freeN(map);

  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = bvh->nodes + i;

    if (n->flag & PBVH_Leaf) {
      /* Recursively split nodes that have gotten too many
       * elements */
      pbvh_trimesh_node_limit_ensure(bvh, i);
    }
  }

#endif
  // BKE_pbvh_update_vertex_data(bvh, PBVH_UpdateVisibility|PBVH_UpdateMask|PBVH_UpdateColor);
#if 0
  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = &bvh->nodes[i];
    if (n->flag & PBVH_Leaf) {
      /* Free orco/ortri data */
      pbvh_trimesh_node_drop_orig(n);

      /* Recursively split nodes that have gotten too many
      * elements */
      pbvh_trimesh_node_limit_ensure(bvh, i);
    }
  }
#endif
}

TableGSet *BKE_pbvh_trimesh_node_unique_verts(PBVHNode *node)
{
  return node->tm_unique_verts;
}

TableGSet *BKE_pbvh_trimesh_node_other_verts(PBVHNode *node)
{
  return node->tm_other_verts;
}

struct GSet *BKE_pbvh_trimesh_node_faces(PBVHNode *node)
{
  return node->tm_faces;
}

/****************************** Debugging *****************************/

#if 0

static void pbvh_trimesh_print(PBVH *bvh)
{
  fprintf(stderr, "\npbvh=%p\n", bvh);
  fprintf(stderr, "bm_face_to_node:\n");

  BMIter iter;
  TMFace *f;
  BM_ITER_MESH (f, &iter, bvh->bm, tm_faces_OF_MESH) {
    fprintf(stderr, "  %d -> %d\n", BM_elem_index_get(f), pbvh_trimesh_node_index_from_face(bvh, f));
  }

  fprintf(stderr, "bm_vert_to_node:\n");
  TMVert *v;
  BM_ITER_MESH (v, &iter, bvh->bm, tm_faces_OF_MESH) {
    fprintf(stderr, "  %d -> %d\n", BM_elem_index_get(v), pbvh_trimesh_node_index_from_vert(bvh, v));
  }

  for (int n = 0; n < bvh->totnode; n++) {
    PBVHNode *node = &bvh->nodes[n];
    if (!(node->flag & PBVH_Leaf)) {
      continue;
    }

    GSetIterator gs_iter;
    fprintf(stderr, "node %d\n  faces:\n", n);
    GSET_ITER (gs_iter, node->tm_faces)
      fprintf(stderr, "    %d\n", BM_elem_index_get((TMFace *)BLI_gsetIterator_getKey(&gs_iter)));
    fprintf(stderr, "  unique verts:\n");
    GSET_ITER (gs_iter, node->tm_unique_verts)
      fprintf(stderr, "    %d\n", BM_elem_index_get((TMVert *)BLI_gsetIterator_getKey(&gs_iter)));
    fprintf(stderr, "  other verts:\n");
    GSET_ITER (gs_iter, node->tm_other_verts)
      fprintf(stderr, "    %d\n", BM_elem_index_get((TMVert *)BLI_gsetIterator_getKey(&gs_iter)));
  }
}

static void print_flag_factors(int flag)
{
  printf("flag=0x%x:\n", flag);
  for (int i = 0; i < 32; i++) {
    if (flag & (1 << i)) {
      printf("  %d (1 << %d)\n", 1 << i, i);
    }
  }
}
#endif

#ifdef USE_VERIFY

static void pbvh_trimesh_verify(PBVH *bvh)
{
  /* build list of faces & verts to lookup */
  GSet *faces_all = BLI_gset_ptr_new_ex(__func__, bvh->bm->totface);
  BMIter iter;

  {
    TMFace *f;
    BM_ITER_MESH (f, &iter, bvh->bm, tm_faces_OF_MESH) {
      BLI_assert(TRIMESH_ELEM_CD_GET_INT(f, bvh->cd_face_node_offset) != DYNTOPO_NODE_NONE);
      BLI_gset_insert(faces_all, f);
    }
  }

  GSet *verts_all = BLI_gset_ptr_new_ex(__func__, bvh->bm->totvert);
  {
    TMVert *v;
    BM_ITER_MESH (v, &iter, bvh->bm, BM_VERTS_OF_MESH) {
      if (TRIMESH_ELEM_CD_GET_INT(v, bvh->cd_vert_node_offset) != DYNTOPO_NODE_NONE) {
        BLI_gset_insert(verts_all, v);
      }
    }
  }

  /* Check vert/face counts */
  {
    int totface = 0, totvert = 0;
    for (int i = 0; i < bvh->totnode; i++) {
      PBVHNode *n = &bvh->nodes[i];
      totface += n->tm_faces ? BLI_gset_len(n->tm_faces) : 0;
      totvert += n->tm_unique_verts ? BLI_gset_len(n->tm_unique_verts) : 0;
    }

    BLI_assert(totface == BLI_gset_len(faces_all));
    BLI_assert(totvert == BLI_gset_len(verts_all));
  }

  {
    TMFace *f;
    BM_ITER_MESH (f, &iter, bvh->bm, tm_faces_OF_MESH) {
      BMIter bm_iter;
      TMVert *v;
      PBVHNode *n = pbvh_trimesh_node_lookup(bvh, f);

      /* Check that the face's node is a leaf */
      BLI_assert(n->flag & PBVH_Leaf);

      /* Check that the face's node knows it owns the face */
      BLI_assert(BLI_gset_haskey(n->tm_faces, f));

      /* Check the face's vertices... */
      BM_ITER_ELEM (v, &bm_iter, f, BM_VERTS_OF_FACE) {
        PBVHNode *nv;

        /* Check that the vertex is in the node */
        BLI_assert(BLI_gset_haskey(n->tm_unique_verts, v) ^ BLI_gset_haskey(n->tm_other_verts, v));

        /* Check that the vertex has a node owner */
        nv = pbvh_trimesh_node_lookup(bvh, v);

        /* Check that the vertex's node knows it owns the vert */
        BLI_assert(BLI_gset_haskey(nv->tm_unique_verts, v));

        /* Check that the vertex isn't duplicated as an 'other' vert */
        BLI_assert(!BLI_gset_haskey(nv->tm_other_verts, v));
      }
    }
  }

  /* Check verts */
  {
    TMVert *v;
    BM_ITER_MESH (v, &iter, bvh->bm, BM_VERTS_OF_MESH) {
      /* vertex isn't tracked */
      if (TRIMESH_ELEM_CD_GET_INT(v, bvh->cd_vert_node_offset) == DYNTOPO_NODE_NONE) {
        continue;
      }

      PBVHNode *n = pbvh_trimesh_node_lookup(bvh, v);

      /* Check that the vert's node is a leaf */
      BLI_assert(n->flag & PBVH_Leaf);

      /* Check that the vert's node knows it owns the vert */
      BLI_assert(BLI_gset_haskey(n->tm_unique_verts, v));

      /* Check that the vertex isn't duplicated as an 'other' vert */
      BLI_assert(!BLI_gset_haskey(n->tm_other_verts, v));

      /* Check that the vert's node also contains one of the vert's
       * adjacent faces */
      bool found = false;
      BMIter bm_iter;
      TMFace *f = NULL;
      BM_ITER_ELEM (f, &bm_iter, v, tm_faces_OF_VERT) {
        if (pbvh_trimesh_node_lookup(bvh, f) == n) {
          found = true;
          break;
        }
      }
      BLI_assert(found || f == NULL);

#  if 1
      /* total freak stuff, check if node exists somewhere else */
      /* Slow */
      for (int i = 0; i < bvh->totnode; i++) {
        PBVHNode *n_other = &bvh->nodes[i];
        if ((n != n_other) && (n_other->tm_unique_verts)) {
          BLI_assert(!BLI_gset_haskey(n_other->tm_unique_verts, v));
        }
      }
#  endif
    }
  }

#  if 0
  /* check that every vert belongs somewhere */
  /* Slow */
  BM_ITER_MESH (vi, &iter, bvh->bm, BM_VERTS_OF_MESH) {
    bool has_unique = false;
    for (int i = 0; i < bvh->totnode; i++) {
      PBVHNode *n = &bvh->nodes[i];
      if ((n->tm_unique_verts != NULL) && BLI_gset_haskey(n->tm_unique_verts, vi)) {
        has_unique = true;
      }
    }
    BLI_assert(has_unique);
    vert_count++;
  }

  /* if totvert differs from number of verts inside the hash. hash-totvert is checked above  */
  BLI_assert(vert_count == bvh->bm->totvert);
#  endif

  /* Check that node elements are recorded in the top level */
  for (int i = 0; i < bvh->totnode; i++) {
    PBVHNode *n = &bvh->nodes[i];
    if (n->flag & PBVH_Leaf) {
      GSetIterator gs_iter;

      GSET_ITER (gs_iter, n->tm_faces) {
        TMFace *f = BLI_gsetIterator_getKey(&gs_iter);
        PBVHNode *n_other = pbvh_trimesh_node_lookup(bvh, f);
        BLI_assert(n == n_other);
        BLI_assert(BLI_gset_haskey(faces_all, f));
      }

      GSET_ITER (gs_iter, n->tm_unique_verts) {
        TMVert *v = BLI_gsetIterator_getKey(&gs_iter);
        PBVHNode *n_other = pbvh_trimesh_node_lookup(bvh, v);
        BLI_assert(!BLI_gset_haskey(n->tm_other_verts, v));
        BLI_assert(n == n_other);
        BLI_assert(BLI_gset_haskey(verts_all, v));
      }

      GSET_ITER (gs_iter, n->tm_other_verts) {
        TMVert *v = BLI_gsetIterator_getKey(&gs_iter);
        /* this happens sometimes and seems harmless */
        // BLI_assert(!BM_vert_face_check(v));
        BLI_assert(BLI_gset_haskey(verts_all, v));
      }
    }
  }

  BLI_gset_free(faces_all, NULL);
  BLI_gset_free(verts_all, NULL);
}

#endif
