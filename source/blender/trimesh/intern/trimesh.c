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
 *
 * The Original Code is Copyright (C) 2008 by Blender Foundation.
 * All rights reserved.
 */

/** \file
 * \ingroup trimesh
 *
 * optimized thread-safe triangle mesh library with topological info
 *
 */

#include <stdlib.h>
#include <string.h>

#include "BLI_listbase.h"

#include "BLI_array.h"
#include "BLI_hashmap.h"
#include "BLI_math.h"
#include "BLI_threadsafe_mempool.h"
#include "trimesh.h"

#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BLI_utildefines.h"
#include "atomic_ops.h"

#include "BKE_customdata.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_multires.h"

#include "BKE_key.h"
#include "BKE_main.h"

#include "MEM_guardedalloc.h"
#ifdef WITH_MEM_VALGRIND
#  include "valgrind/memcheck.h"
#endif

#include "trimesh_private.h"

// keep up to sync with MAX_TRIMESH_POOLS in BLI_trimesh.h
static int poolsizes[] = {sizeof(TMVert),
                          sizeof(TMEdge),
                          sizeof(TMFace),
                          sizeof(void *) * V_ELIST_ESIZE,
                          sizeof(void *) * E_TLIST_ESIZE,
                          sizeof(TMLoopData)};

void trimesh_element_init(void *elem, CustomData *customdata, bool skipcd)
{

  TMElement *e = elem;
  e->customdata = NULL;
  e->flag = e->index = e->threadtag = 0;

  if (skipcd) {
    if (customdata->totsize) {
      e->customdata = BLI_safepool_alloc(customdata->tpool);
      memset(e->customdata, 0, customdata->totsize);
    }
  }
  else {
    CustomData_bmesh_set_default(customdata, &e->customdata);
  }
}

void trimesh_element_destroy(void *elem, CustomData *customdata)
{
  TMElement *e = elem;

  CustomData_bmesh_free_block(customdata, &e->customdata);
}

static TMLoopData *trimesh_make_loop(TM_TriMesh *tm, bool skipcd)
{
  TMLoopData *loop = BLI_safepool_alloc(tm->pools[POOL_LOOP]);

  trimesh_element_init(loop, &tm->ldata, skipcd);

  return loop;
}

static void trimesh_kill_loop(TM_TriMesh *tm, TMLoopData *l)
{
  trimesh_element_destroy(l, &tm->ldata);
}

void TMesh_free(TM_TriMesh *tm)
{
  TMElement *elem;
  TM_TriMeshIter iter;

  const int types1[] = {TM_VERTEX, TM_EDGE, TM_LOOP, TM_TRI};
  const int types2[] = {TM_VERTS_OF_MESH, TM_EDGES_OF_MESH, TM_LOOPS_OF_MESH, TM_TRIS_OF_MESH};
  TMVert *v;
  TMEdge *e;
  int totmemd = 0;

  TM_ITER_MESH (v, &iter, tm, TM_VERTS_OF_MESH) {
    // trimesh_simplelist_free(tm, &v->edges, POOL_ELIST, 0);
    if (!v->edges.is_pool_allocd) {
      MEM_freeN(v->edges.items);
      // printf("len: %d\n", v->edges.length);
      totmemd++;
    }
  }

  TM_ITER_MESH (e, &iter, tm, TM_EDGES_OF_MESH) {
    // trimesh_simplelist_free(tm, &e->tris, POOL_TLIST, 0);
    if (!e->tris.is_pool_allocd) {
      MEM_freeN(e->tris.items);
      // printf("len: %d\n", e->tris.length);
      totmemd++;
    }
  }

  printf("totmemd: %d\n", totmemd);

  for (int i = 0; i < 4; i++) {
    CustomData *cdata = trimesh_get_customdata(tm, types1[i]);

    if (!cdata->tpool) {
      continue;
    }

    TM_ITER_MESH (elem, &iter, tm, types2[i]) {
      CustomData_bmesh_free_block(cdata, &elem->customdata);
    }

    BLI_safepool_destroy(cdata->tpool);
    cdata->tpool = NULL;
  }

  for (int i = 0; i < MAX_TRIMESH_POOLS; i++) {
    BLI_safepool_destroy(tm->pools[i]);
  }

  if (tm->vtable.table)
    MEM_freeN(tm->vtable.table);
  if (tm->etable.table)
    MEM_freeN(tm->etable.table);
  if (tm->ttable.table)
    MEM_freeN(tm->ttable.table);

  MEM_freeN(tm);
}

TM_TriMesh *TMesh_new(int maxthread)
{
  TM_TriMesh *tm = MEM_callocN(sizeof(*tm), "OptTriMesh");
  int i;

  maxthread = MAX2(maxthread, 1);
  tm->maxthread = maxthread;

  for (i = 0; i < MAX_TRIMESH_POOLS; i++) {
    tm->pools[i] = BLI_safepool_create(poolsizes[i], 512, maxthread);
  }

  return tm;
}

#define OTHER_VERT(e, v) ((v) == (e)->v1 ? (e)->v2 : (e)->v1)

static TMEdge *ensure_edge(TM_TriMesh *tm, TMVert *v1, TMVert *v2, bool skipcd)
{
  for (int i = 0; i < v1->edges.length; i++) {
    TMEdge *e = v1->edges.items[i];

    if (OTHER_VERT(e, v1) == v2) {
      return e;
    }
  }

  TMEdge *e = BLI_safepool_alloc(tm->pools[POOL_EDGE]);
  memset(e, 0, sizeof(*e));

  e->v1 = v1;
  e->v2 = v2;

  trilist_simplelist_init(tm, &e->tris, E_TLIST_ESIZE, POOL_TLIST);

  trilist_simplelist_append(tm, &e->v1->edges, e, POOL_ELIST);
  trilist_simplelist_append(tm, &e->v2->edges, e, POOL_ELIST);

  trimesh_element_init(e, &tm->edata, skipcd);
  tm->totedge++;

  return e;
}

void TM_vert_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter)
{
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_VERTEX;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter->iter);
}

void TM_edge_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter)
{
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_EDGE;

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter->iter);
}

void TM_loop_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter)
{
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_LOOP;

  BLI_safepool_iternew(tm->pools[POOL_LOOP], &iter->iter);
}

void TM_tri_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter)
{
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_TRI;

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter->iter);
}

static TMEdge *edge_add_tri(
    TM_TriMesh *tm, TMVert *v1, TMVert *v2, TMFace *tri, bool skipcd)
{
  TMEdge *e = ensure_edge(tm, v1, v2, skipcd);
  trilist_simplelist_append(tm, &e->tris, tri, POOL_TLIST);

  return e;
}

TMVert *TM_make_vert(TM_TriMesh *tm, float co[3], float no[3], bool skipcd)
{
  TMVert *v = BLI_safepool_alloc(tm->pools[POOL_VERTEX]);

  trimesh_element_init(v, &tm->vdata, skipcd);

  trilist_simplelist_init(tm, &v->edges, V_ELIST_ESIZE, POOL_ELIST);

  v->pad = 12345;
  v->threadtag = 12345;

  if (co) {
    copy_v3_v3(v->co, co);
  }
  else {
    zero_v3(v->co);
  }

  if (no) {
    copy_v3_v3(v->no, no);
  }
  else {
    zero_v3(v->no);
  }
  tm->totvert++;

  return v;
}

TMEdge *TM_get_edge(TM_TriMesh *tm, TMVert *v1, TMVert *v2, bool skipcd)
{
  return ensure_edge(tm, v1, v2, skipcd);
}

TMFace *TM_make_tri(TM_TriMesh *tm, TMVert *v1, TMVert *v2, TMVert *v3, bool skipcd)
{
  TMFace *tri = BLI_safepool_alloc(tm->pools[POOL_TRI]);

  memset(tri, 0, sizeof(*tri));

  trimesh_element_init(tri, &tm->tdata, skipcd);

  tri->l1 = trimesh_make_loop(tm, skipcd);
  tri->l2 = trimesh_make_loop(tm, skipcd);
  tri->l3 = trimesh_make_loop(tm, skipcd);

  tri->v1 = v1;
  tri->v2 = v2;
  tri->v3 = v3;

  tri->e1 = edge_add_tri(tm, v1, v2, tri, skipcd);
  tri->e2 = edge_add_tri(tm, v2, v3, tri, skipcd);
  tri->e3 = edge_add_tri(tm, v3, v1, tri, skipcd);

  if (!tri->e3) {
    printf("evil! %p\n", tri->e3);
  }
  tm->tottri++;

  return tri;
}

void TM_add(TM_TriMesh *tm,
            float *vertCos,
            float *vertNos,
            int totvert,
            int *triIndices,
            int tottri,
            int threadnr,
            bool skipcd)
{
  float *vco, *vno;
  int i;

  // abuse the normals array to store pointers?
  // or just allocate one?

  TMVert **vmap = MEM_mallocN(sizeof(*vmap) * totvert, "BLI_trimesh_add:vmap temporary");
  // OptTriVertex** vmap = (OptTriVertex**)vertNos;

  vco = vertCos;
  vno = vertNos;

  for (i = 0; i < totvert; i++, vco += 3, vno += 3) {
    TMVert *v = TM_make_vert(tm, vco, vno, skipcd);
    vmap[i] = v;
  }

  for (i = 0; i < tottri; i++, triIndices += 3) {
    TMVert *v1 = vmap[triIndices[0]];
    TMVert *v2 = vmap[triIndices[1]];
    TMVert *v3 = vmap[triIndices[2]];

    TM_make_tri(tm, v1, v2, v3, skipcd);
  }

  MEM_freeN(vmap);
}

CustomData *trimesh_get_customdata(TM_TriMesh *tm, int type)
{
  switch (type) {
    case TM_VERTEX:
      return &tm->vdata;
    case TM_EDGE:
      return &tm->edata;
    case TM_LOOP:
      return &tm->ldata;
    case TM_TRI:
      return &tm->tdata;
  }

  return NULL;
}

void TM_kill_vert(TM_TriMesh *tm, TMVert *v)
{
  tm->elem_index_dirty |= TM_VERTEX;
  tm->elem_table_dirty |= TM_VERTEX;

  /// clear debug tag, 12345
  v->pad = v->threadtag = 0;

  while (v->edges.length > 0) {
    TM_kill_edge(tm, v->edges.items[0], false);
  }

  trimesh_element_destroy(v, &tm->vdata);

  tm->totvert--;
  trimesh_simplelist_free(tm, &v->edges, POOL_ELIST);
  BLI_safepool_free(tm->pools[POOL_VERTEX], v);
}

// if kill_verts is true verts with no edges will be deleted
void TM_kill_edge(TM_TriMesh *tm, TMEdge *e, bool kill_verts)
{
  tm->elem_index_dirty |= TM_EDGE;
  tm->elem_table_dirty |= TM_EDGE;

  while (e->tris.length > 0) {
    TMFace *tri = e->tris.items[0];
    TM_kill_tri(tm, tri, false, false);
  }

  trimesh_simplelist_remove(tm, &e->v1->edges, e, POOL_ELIST);
  trimesh_simplelist_remove(tm, &e->v2->edges, e, POOL_ELIST);

  if (kill_verts) {
    if (e->v1->edges.length == 0) {
      TM_kill_vert(tm, e->v1);
    }

    if (e->v2->edges.length == 0) {
      TM_kill_vert(tm, e->v2);
    }
  }

  tm->totedge--;
  trimesh_element_destroy(e, &tm->edata);

  trimesh_simplelist_free(tm, &e->tris, POOL_TLIST);
  BLI_safepool_free(tm->pools[POOL_EDGE], e);
}

// kill_edges/verts is whether to automatically kill verts/edges that belong to no triangles
void TM_kill_tri(TM_TriMesh *tm, TMFace *tri, bool kill_edges, bool kill_verts)
{
  // static void simplelist_remove(OptTriMesh *tm, optmesh_simplelist *list, void *item, int pool,
  // int threadnr) {

  tm->elem_index_dirty |= TM_TRI;
  tm->elem_table_dirty |= TM_TRI;

  trimesh_simplelist_remove(tm, &tri->e1->tris, tri, POOL_TLIST);
  trimesh_simplelist_remove(tm, &tri->e2->tris, tri, POOL_TLIST);
  trimesh_simplelist_remove(tm, &tri->e3->tris, tri, POOL_TLIST);

  trimesh_kill_loop(tm, tri->l1);
  trimesh_kill_loop(tm, tri->l2);
  trimesh_kill_loop(tm, tri->l3);

  if (kill_edges) {
    if (tri->e1->tris.length == 0) {
      TM_kill_edge(tm, tri->e1, kill_verts);
    }
    if (tri->e2->tris.length == 0) {
      TM_kill_edge(tm, tri->e2, kill_verts);
    }
    if (tri->e3->tris.length == 0) {
      TM_kill_edge(tm, tri->e3, kill_verts);
    }
  }

  tm->tottri--;
  trimesh_element_destroy(tri, &tm->tdata);

  BLI_safepool_free(tm->pools[POOL_TRI], tri);
}

static void weld_verts(TM_TriMesh *tm, TMVert *v1, TMVert *v2)
{
  for (int i = 0; i < v2->edges.length; i++) {
    TMEdge *e = v2->edges.items[i];
    TMVert *vb = OTHER_VERT(e, v1);

    if (vb == v1) {
      // not sure this will ever happen; force deletion of edge?
      fprintf(stdout, "auto deleting edge in BLI_trimesh code\n");
      fflush(stdout);

      TM_kill_edge(tm, e, false);

      continue;
    }

    // swap out vertex references for triangles
    for (int j = 0; j < e->tris.length; j++) {
      TMFace *tri = e->tris.items[j];
      int tot = 0;

      // make sure we delete triangles that end up with duplicate verts
      if (tri->v1 == v2) {
        tri->v1 = v1;
        tot++;
      }

      if (tri->v2 == v2) {
        tri->v2 = v1;
        tot++;
      }

      if (tri->v3 == v2) {
        tri->v3 = v1;
        tot++;
      }

      if (tot > 1) {
        TM_kill_tri(tm, tri, false, false);
      }
    }

    // swap out vertex references for edge
    if (e->v1 == v2) {
      e->v1 = v1;
    }
    else {
      e->v2 = v1;
    }

    trilist_simplelist_append(tm, &v1->edges, e, POOL_ELIST);
  }

  v2->edges.length = 0;
  TM_kill_vert(tm, v2);
}

bool TM_elem_is_dead(void *elem)
{
  return BLI_safepool_elem_is_dead(elem);
}

void TM_collapse_edge(TM_TriMesh *tm, TMEdge *e)
{
  TMVert *v1 = e->v1, *v2 = e->v2;

  TM_kill_edge(tm, e, false);
  weld_verts(tm, v1, v2);
}

TMVert *TM_split_edge(TM_TriMesh *tm, TMEdge *e, float fac, bool skipcd)
{
  float co[3];
  float no[3];

  interp_v3_v3v3(co, e->v1->co, e->v2->co, fac);
  add_v3_v3v3(no, e->v1->no, e->v2->no);
  normalize_v3(no);

  TMVert *vc = TM_make_vert(tm, co, no, skipcd);

  TMEdge *e2 = ensure_edge(tm, e->v1, vc, skipcd);
  TMEdge *e3 = ensure_edge(tm, vc, e->v2, skipcd);

  int eflag = e->flag;

  e2->flag |= eflag;
  e3->flag |= eflag;

  if (!skipcd) {
    float src_weights[2] = {0.5f, 0.5f};
    void *src_blocks[2] = {e->v1->customdata, e->v2->customdata};

    CustomData_bmesh_interp(&tm->vdata, (void **)src_blocks, src_weights, NULL, 2, vc->customdata);
    CustomData_bmesh_copy_data(&tm->edata, &tm->edata, &e->customdata, &e2->customdata);
    CustomData_bmesh_copy_data(&tm->edata, &tm->edata, &e->customdata, &e3->customdata);
  }

  while (e->tris.length) {
    TMFace *tri = e->tris.items[0];

    TMVert *tv1, *tv2, *tv3;
    int vi = 0;

    for (int j = 0; j < 3; j++) {
      if (TRIEDGE(tri, j) == e) {
        vi = j;
        break;
      }
    }

    tv1 = TRIVERT(tri, vi);
    tv2 = TRIVERT(tri, (vi + 1) % 3);
    tv3 = TRIVERT(tri, (vi + 2) % 3);

    TMFace *t1 = TM_make_tri(tm, tv1, vc, tv3, skipcd);
    TMFace *t2 = TM_make_tri(tm, vc, tv2, tv3, skipcd);

    if (!skipcd) {
      TMLoopData *l1 = TRILOOP(tri, vi);
      TMLoopData *l2 = TRILOOP(tri, (vi + 1) % 3);
      TMLoopData *l3 = TRILOOP(tri, (vi + 2) % 3);

      float src_weights[2] = {0.5f, 0.5f};
      void *src_blocks[2] = {l1->customdata, l3->customdata};

      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l1->customdata, t1->l1->customdata);
      CustomData_bmesh_interp(&tm->ldata, src_blocks, src_weights, NULL, 2, t1->l2->customdata);
      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l3->customdata, t1->l3->customdata);

      src_blocks[0] = l2->customdata;
      src_blocks[1] = l3->customdata;

      CustomData_bmesh_interp(&tm->ldata, src_blocks, src_weights, NULL, 2, t2->l1->customdata);
      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l1->customdata, t1->l2->customdata);
      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l3->customdata, t1->l3->customdata);
    }
    TM_kill_tri(tm, tri, false, false);
  }

  TM_kill_edge(tm, e, false);
  TM_calc_vert_normal(vc, true);

  return vc;
}

void TM_index_update(TM_TriMesh *tm)
{
  ThreadSafePoolIter iter;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter);
  TMVert *v = BLI_safepool_iterstep(&iter);
  for (int i = 0; v; v = BLI_safepool_iterstep(&iter), i++) {
    v->index = i;
  }

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter);
  TMEdge *e = BLI_safepool_iterstep(&iter);
  for (int i = 0; e; e = BLI_safepool_iterstep(&iter), i++) {
    e->index = i;
  }

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter);
  TMFace *t = BLI_safepool_iterstep(&iter);
  for (int i = 0; t; t = BLI_safepool_iterstep(&iter), i++) {
    t->index = i;
  }
}

static void resize_table(TM_ElemTable *table, int newsize)
{
  if (!table || newsize > table->size) {
    int size2 = (newsize << 1) - (newsize >> 1);

    if (!table->table) {
      table->table = MEM_mallocN(sizeof(void *) * size2, "trimesh eleme table");
    }
    else {
      table->table = MEM_reallocN(table->table, sizeof(void *) * size2);
    }

    table->size = size2;
    table->used = newsize;
  }
}

static int TM_get_elem_count(TM_TriMesh *tm, int type)
{
  switch (type) {
    case TM_VERTEX:
      return tm->totvert;
    case TM_EDGE:
      return tm->totedge;
    //*
    case TM_LOOP:
      return tm->tottri * 3;
    case TM_TRI:
      return tm->tottri;
  }

  return -1;
}
static bool is_table_dirty(TM_TriMesh *tm, TM_ElemTable *table, int type)
{
  if (!type) {
    return false;
  }

  bool ret = !table->table;

  ret = ret || table->used != TM_get_elem_count(tm, type);
  ret = ret || (tm->elem_table_dirty & type);

  return ret;
}

TM_ElemTable *TM_getElemTable(TM_TriMesh *tm, int type)
{
  switch (type) {
    case TM_VERTEX:
      return &tm->vtable;
    case TM_EDGE:
      return &tm->etable;
    case TM_TRI:
      return &tm->ttable;
    default:
      return NULL;
  }
}

void TM_mesh_elem_table_ensure(TM_TriMesh *tm, int typemask)
{
  const int types[] = {TM_VERTEX, TM_EDGE, TM_TRI};
  const int iters[] = {TM_VERTS_OF_MESH, TM_EDGES_OF_MESH, TM_TRIS_OF_MESH};

  for (int ti = 0; ti < 3; ti++) {
    TM_ElemTable *table = TM_getElemTable(tm, types[ti]);

    if (is_table_dirty(tm, table, typemask & types[ti])) {
      TM_TriMeshIter iter;
      TMElement *e;
      int i = 0;

      resize_table(table, TM_get_elem_count(tm, types[ti]));

      TM_ITER_MESH (e, &iter, tm, iters[ti]) {
        table->table[i] = e;
        i++;
      }

      table->used = i;

      if (i != TM_get_elem_count(tm, types[ti])) {
        printf("eek %d %d\n", i, TM_get_elem_count(tm, types[ti]));
      }
    }
  }
  tm->elem_table_dirty &= ~typemask;
}

void TM_mesh_elem_index_ensure(TM_TriMesh *tm, int typemask)
{
  if ((typemask & tm->elem_index_dirty) & TM_VERTEX) {
    TM_TriMeshIter iter;
    TMVert *e;
    int i = 0;

    TM_ITER_MESH (e, &iter, tm, TM_VERTS_OF_MESH) {
      e->index = i++;
    }
  }

  if ((typemask & tm->elem_index_dirty) & TM_EDGE) {
    TM_TriMeshIter iter;
    TMEdge *e;
    int i = 0;

    TM_ITER_MESH (e, &iter, tm, TM_EDGES_OF_MESH) {
      e->index = i++;
    }
  }

  if ((typemask & tm->elem_index_dirty) & TM_TRI) {
    TM_TriMeshIter iter;
    TMFace *e;
    int i = 0;

    TM_ITER_MESH (e, &iter, tm, TM_TRIS_OF_MESH) {
      e->index = i++;
    }
  }

  tm->elem_index_dirty &= ~typemask;
}

void TM_mesh_normals_update(TM_TriMesh *tm)
{
  TM_TriMeshIter iter;
  TMVert *v;
  TMFace *f;
  return;

  TM_ITER_MESH (f, &iter, tm, TM_TRIS_OF_MESH) {
    TM_calc_tri_normal(f);
  }

  TM_ITER_MESH (v, &iter, tm, TM_VERTS_OF_MESH) {
    TM_calc_vert_normal(v, false);
  }
}
