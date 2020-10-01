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

#include "BLI_math.h"
#include "trimesh.h"
#include "BLI_threadsafe_mempool.h"
#include "BLI_array.h"

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


//keep up to sync with MAX_TRIMESH_POOLS in BLI_trimesh.h
static int poolsizes[] = {
  sizeof(TMVert),
  sizeof(TMEdge),
  sizeof(TMFace),
  sizeof(void*)*V_ELIST_ESIZE,
  sizeof(void*)*E_TLIST_ESIZE,
  sizeof(TMLoopData)
};

void trimesh_element_init(void *elem, CustomData *customdata) {

  TMElement *e = elem;
  e->customdata = NULL;
  e->flag = e->index = e->threadtag = 0;
  
  CustomData_bmesh_set_default(customdata, &e->customdata);
}

void trimesh_element_destroy(void *elem, int threadnr, CustomData *customdata) {
  TMElement *e = elem;

  CustomData_bmesh_free_block(customdata, &e->customdata);
}

static TMLoopData *trimesh_make_loop(TM_TriMesh *tm, int threadnr, bool skipcd) {
  TMLoopData *loop = BLI_safepool_alloc(tm->pools[POOL_LOOP]);

  trimesh_element_init(loop, &tm->ldata);

  return loop;
}

static void trimesh_kill_loop(TM_TriMesh *tm, TMLoopData *l, int threadnr) {
  trimesh_element_destroy(l, threadnr, &tm->ldata);
}

void TMesh_free(TM_TriMesh *tm) {
  TMElement *elem;
  TM_TriMeshIter iter;

  const int types1[] = {TM_VERTEX, TM_EDGE, TM_LOOP, TM_TRI};
  const int types2[] = {TM_VERTS_OF_MESH, TM_EDGES_OF_MESH, TM_LOOPS_OF_MESH, TM_TRIS_OF_MESH};

  for (int i=0; i<4; i++) {
    CustomData *cdata = trimesh_get_customdata(tm, types1[i]);

    if (!cdata->tpool) {
      continue;
    }

    TM_ITER_MESH(elem, &iter, tm, types2[i]) {
      CustomData_bmesh_free_block(cdata, &elem->customdata);
    }

    BLI_safepool_destroy(cdata->tpool);
  }

  for (int i=0; i<MAX_TRIMESH_POOLS; i++) {
    BLI_safepool_destroy(tm->pools[i]);
  }

  if (tm->vtable)
    MEM_freeN(tm->vtable);
  if (tm->etable)
    MEM_freeN(tm->etable);
  if (tm->ttable)
    MEM_freeN(tm->ttable);

  MEM_freeN(tm);
}

TM_TriMesh* TMesh_new(int maxthread) {
  TM_TriMesh* tm = MEM_callocN(sizeof(*tm), "OptTriMesh");
  int i;

  maxthread = MAX2(maxthread, 1);
  tm->maxthread = maxthread;

  for (i = 0; i < MAX_TRIMESH_POOLS; i++) {
    tm->pools[i] = BLI_safepool_create(poolsizes[i], 512, maxthread);
  }

  return tm;
}

#define OTHER_VERT(e, v) ((v) == (e)->v1 ? (e)->v2 : (e)->v1)

static TMEdge *ensure_edge(TM_TriMesh* tm, TMVert* v1, TMVert* v2, int threadnr, bool skipcd) {
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

  trilist_simplelist_append(tm, &e->v1->edges, e, POOL_ELIST, threadnr);
  trilist_simplelist_append(tm, &e->v2->edges, e, POOL_ELIST, threadnr);

  memset(&e->tris, 0, sizeof(e->tris));
  trimesh_element_init(e, &tm->edata);
  tm->totedge++;

  return e;
}

void TM_vert_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_VERTEX;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter->iter);
}

void TM_edge_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_EDGE;

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter->iter);
}

void TM_loop_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_LOOP;

  BLI_safepool_iternew(tm->pools[POOL_LOOP], &iter->iter);
}

void TM_tri_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_TRI;

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter->iter);
}

void *TM_iterstep(TM_TriMeshIter *iter) {
  return BLI_safepool_iterstep(&iter->iter);
}

static TMEdge *edge_add_tri(TM_TriMesh* tm, TMVert* v1, TMVert* v2, TMFace* tri, int threadnr, bool skipcd) {
  TMEdge *e = ensure_edge(tm, v1, v2, threadnr, skipcd);
  trilist_simplelist_append(tm, &e->tris, tri, POOL_TLIST, threadnr);

  return e;
}

TMVert *TM_make_vert(TM_TriMesh *tm, float co[3], float no[3], int threadnr, bool skipcd) {
  TMVert *v = BLI_safepool_alloc(tm->pools[POOL_VERTEX]);

  trimesh_element_init(v, &tm->vdata);

  trilist_simplelist_init(tm, &v->edges, V_ELIST_ESIZE, POOL_ELIST);

  if (co) {
    copy_v3_v3(v->co, co);
  } else {
    zero_v3(v->co);
  }

  if (no) {
    copy_v3_v3(v->no, no);
  } else {
      zero_v3(v->no);
  }
  tm->totvert++;

  return v;
}

TMEdge *TM_get_edge(TM_TriMesh *tm, TMVert *v1, TMVert *v2, int threadnr, bool skipcd) {
  return ensure_edge(tm, v1, v2, threadnr, skipcd);
}

TMFace *TM_make_tri(TM_TriMesh *tm, TMVert *v1, TMVert *v2, TMVert *v3, int threadnr, bool skipcd) {
  TMFace *tri = BLI_safepool_alloc(tm->pools[POOL_TRI]);

  memset(tri, 0, sizeof(*tri));

  trimesh_element_init(tri, &tm->tdata);

  tri->l1 = trimesh_make_loop(tm, threadnr, skipcd);
  tri->l2 = trimesh_make_loop(tm, threadnr, skipcd);
  tri->l3 = trimesh_make_loop(tm, threadnr, skipcd);

  tri->v1 = v1;
  tri->v2 = v2;
  tri->v3 = v3;

  tri->e1 = edge_add_tri(tm, v1, v2, tri, threadnr, skipcd);
  tri->e2 = edge_add_tri(tm, v2, v3, tri, threadnr, skipcd);
  tri->e3 = edge_add_tri(tm, v3, v1, tri, threadnr, skipcd);

  tm->tottri++;

  return tri;
}

void TM_add(TM_TriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr, bool skipcd) {
  float* vco, * vno;
  int i;

  //abuse the normals array to store pointers?
  //or just allocate one?

  TMVert** vmap = MEM_mallocN(sizeof(*vmap) * totvert, "BLI_trimesh_add:vmap temporary");
  //OptTriVertex** vmap = (OptTriVertex**)vertNos;

  vco = vertCos;
  vno = vertNos;

  for (i = 0; i < totvert; i++, vco += 3, vno += 3) {
    TMVert *v = TM_make_vert(tm, vco, vno, threadnr, skipcd);
    vmap[i] = v;
  }

  for (i = 0; i < tottri; triIndices += 3) {
    TMVert *v1 = vmap[triIndices[0]];
    TMVert *v2 = vmap[triIndices[1]];
    TMVert *v3 = vmap[triIndices[2]];

    TM_make_tri(tm, v1, v2, v3, threadnr, skipcd);
  }

  MEM_freeN(vmap);
}



CustomData *trimesh_get_customdata(TM_TriMesh *tm, int type) {
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

//kill_edges/verts is whether to automatically kill verts/edges that belong to no triangles
//note that threadnr doesn't refer to whichever thread created tri, but the calling thread
void TM_kill_tri(TM_TriMesh *tm, TMFace *tri, int threadnr, bool kill_edges, bool kill_verts) {
  //static void simplelist_remove(OptTriMesh *tm, optmesh_simplelist *list, void *item, int pool, int threadnr) {

  trimesh_simplelist_remove(tm, &tri->e1->tris, tri, POOL_TLIST, threadnr);
  trimesh_simplelist_remove(tm, &tri->e2->tris, tri, POOL_TLIST, threadnr);
  trimesh_simplelist_remove(tm, &tri->e3->tris, tri, POOL_TLIST, threadnr);

  trimesh_kill_loop(tm, tri->l1, threadnr);
  trimesh_kill_loop(tm, tri->l2, threadnr);
  trimesh_kill_loop(tm, tri->l3, threadnr);

  if (tri->e1->tris.length == 0) {
    TM_kill_edge(tm, tri->e1, threadnr, kill_verts);
  }
  if (tri->e2->tris.length == 0) {
    TM_kill_edge(tm, tri->e2, threadnr, kill_verts);
  }
  if (tri->e3->tris.length == 0) {
    TM_kill_edge(tm, tri->e3, threadnr, kill_verts);
  }


  tm->tottri--;
  trimesh_element_destroy(tri, threadnr, &tm->tdata);

  BLI_safepool_free(tm->pools[POOL_TRI], tri);
}

static void weld_verts(TM_TriMesh *tm, TMVert *v1, TMVert *v2, int threadnr) {
  for (int i=0; i<v2->edges.length; i++) {
    TMEdge *e = v2->edges.items[i];
    TMVert *vb = OTHER_VERT(e, v1);

    if (vb == v1) {
      //not sure this will ever happen; force deletion of edge?
      fprintf(stdout, "auto deleting edge in BLI_trimesh code\n");
      fflush(stdout);

      TM_kill_edge(tm, e, threadnr, false);

      continue;
    }

    //swap out vertex references for triangles
    for (int j=0; j<e->tris.length; j++) {
      TMFace *tri = e->tris.items[j];
      int tot = 0;

      //make sure we delete triangles that end up with duplicate verts
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
        TM_kill_tri(tm, tri, threadnr, false, false);
      }
    }

    //swap out vertex references for edge
    if (e->v1 == v2) {
      e->v1 = v1;
    } else {
      e->v2 = v1;
    }

    trilist_simplelist_append(tm, &v1->edges, e, POOL_ELIST, threadnr);
  }

  v2->edges.length = 0;
  TM_kill_vert(tm, v2, threadnr);
}

bool BLI_trimesh_elem_is_dead(void *elem) {
  return BLI_safepool_elem_is_dead(elem);
}

void TM_collapse_edge(TM_TriMesh *tm, TMEdge *e, int threadnr) {
  TMVert *v1 = e->v1, *v2 = e->v2;

  TM_kill_edge(tm, e, threadnr, false);
  weld_verts(tm, v1, v2, threadnr);
}


TMVert *TM_split_edge(TM_TriMesh *tm, TMEdge *e, int threadnr, float fac, bool skipcd) {
  float co[3];
  float no[3];

  interp_v3_v3v3(co, e->v1->co, e->v2->co, fac);
  add_v3_v3v3(no, e->v1->no, e->v2->no);
  normalize_v3(no);

  TMVert *vc = TM_make_vert(tm, co, no, threadnr, skipcd);

  TMEdge *e2 = ensure_edge(tm, e->v1, vc, threadnr, skipcd);
  TMEdge *e3 = ensure_edge(tm, vc, e->v2, threadnr, skipcd);

  if (!skipcd) {
    float src_weights[2] = {0.5f, 0.5f};
    void *src_blocks[2] = {e->v1->customdata, e->v2->customdata};
  
    CustomData_bmesh_interp(&tm->vdata, src_blocks, src_weights, NULL, 2, vc->customdata);
    CustomData_bmesh_copy_data(&tm->edata, &tm->edata, &e->customdata, &e2->customdata);
    CustomData_bmesh_copy_data(&tm->edata, &tm->edata, &e->customdata, &e3->customdata);
  }

  for (int i=0; i<e->tris.length; i++) {
    TMFace *tri = e->tris.items[0];

    TMVert *tv1, *tv2, *tv3;
    int vi = 0;

    for (int j=0; j<3; j++) {
      if (TRIEDGE(tri, j) == e) {
        vi = j;
        break;
      }
    }

    tv1 = TRIVERT(tri, vi);
    tv2 = TRIVERT(tri, (vi+1)%3);
    tv3 = TRIVERT(tri, (vi+2)%3);

    TMFace *t1 = TM_make_tri(tm, tv1, vc, tv3, threadnr, skipcd);
    TMFace *t2 = TM_make_tri(tm, vc, tv2, tv3, threadnr, skipcd);

    if (!skipcd) {
      TMLoopData *l1 = TRILOOP(tri, vi);
      TMLoopData *l2 = TRILOOP(tri, (vi+1)%3);
      TMLoopData *l3 = TRILOOP(tri, (vi+2)%3);

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
    TM_kill_tri(tm, tri, threadnr, false, false);
  }

  TM_kill_edge(tm, e, threadnr, false);
  TM_calc_vert_normal(vc, true);

  return vc;
}

void TM_index_update(TM_TriMesh *tm) {
  ThreadSafePoolIter iter;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter);
  TMVert *v = BLI_safepool_iterstep(&iter);
  for (int i=0; v; v = BLI_safepool_iterstep(&iter), i++) {
    v->index = i;
  }

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter);
  TMEdge *e = BLI_safepool_iterstep(&iter);
  for (int i=0; e; e = BLI_safepool_iterstep(&iter), i++) {
    e->index = i;
  }

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter);
  TMFace *t = BLI_safepool_iterstep(&iter);
  for (int i=0; t; t = BLI_safepool_iterstep(&iter), i++) {
    t->index = i;
  }
}


char BLI_trimesh_mesh_cd_flag_from_bmesh(TM_TriMesh *tm)
{
  char cd_flag = 0;
  if (CustomData_has_layer(&tm->vdata, CD_BWEIGHT)) {
    cd_flag |= ME_CDFLAG_VERT_BWEIGHT;
  }
  if (CustomData_has_layer(&tm->edata, CD_BWEIGHT)) {
    cd_flag |= ME_CDFLAG_EDGE_BWEIGHT;
  }
  if (CustomData_has_layer(&tm->edata, CD_CREASE)) {
    cd_flag |= ME_CDFLAG_EDGE_CREASE;
  }
  return cd_flag;
}

BLI_INLINE void tmesh_quick_edgedraw_flag(MEdge *med, TMEdge *e)
{
  /* This is a cheap way to set the edge draw, its not precise and will
  * pick the first 2 faces an edge uses.
  * The dot comparison is a little arbitrary, but set so that a 5 subd
  * IcoSphere won't vanish but subd 6 will (as with pre-bmesh Blender). */

  if (e->tris.length > 1) {
    TMFace *t1 = e->tris.items[0];
    TMFace *t2 = e->tris.items[1];

    if (dot_v3v3(t1->no, t2->no) > 0.9995f) {
      med->flag &= ~ME_EDGEDRAW;
    } else {
      med->flag |= ME_EDGEDRAW;
    }
  }
}


/**
* \brief BMesh -> Mesh
*/
static TMVert **tm_to_mesh_vertex_map(TM_TriMesh *bm, int ototvert)
{
  const int cd_shape_keyindex_offset = CustomData_get_offset(&bm->vdata, CD_SHAPE_KEYINDEX);
  TMVert **vertMap = NULL;
  TMVert *eve;
  int i = 0;
  TM_TriMeshIter iter;

  /* Caller needs to ensure this. */
  BLI_assert(ototvert > 0);

  vertMap = MEM_callocN(sizeof(*vertMap) * ototvert, "vertMap");
  if (cd_shape_keyindex_offset != -1) {
    TM_vert_iternew(bm, &iter);
    eve = TM_iterstep(&iter);

    for (; eve; eve = TM_iterstep(&iter), i++) {
      const int keyi = TM_ELEM_CD_GET_INT(eve, cd_shape_keyindex_offset);
      if ((keyi != ORIGINDEX_NONE) && (keyi < ototvert) &&
        /* Not fool-proof, but chances are if we have many verts with the same index,
        * we will want to use the first one,
        * since the second is more likely to be a duplicate. */
        (vertMap[keyi] == NULL)) {
        vertMap[keyi] = eve;
      }
    }
  }
  else {
    TM_vert_iternew(bm, &iter);
    eve = TM_iterstep(&iter);

    for (; eve; eve = TM_iterstep(&iter), i++) {
      if (i < ototvert) {
        vertMap[i] = eve;
      }
      else {
        break;
      }
    }
  }

  return vertMap;
}

void TM_mesh_bm_to_me(struct Main *bmain,
  TM_TriMesh *tm,
  struct Mesh *me,
  const struct TMeshToMeshParams *params)
{
  TMVert *v;
  TMEdge *e;
  TMFace *f;

  const int cd_vert_bweight_offset = CustomData_get_offset(&tm->vdata, CD_BWEIGHT);
  const int cd_edge_bweight_offset = CustomData_get_offset(&tm->edata, CD_BWEIGHT);
  const int cd_edge_crease_offset = CustomData_get_offset(&tm->edata, CD_CREASE);
  const int cd_shape_keyindex_offset = CustomData_get_offset(&tm->vdata, CD_SHAPE_KEYINDEX);

  MVert *oldverts = NULL;
  const int ototvert = me->totvert;

  if (me->key && (cd_shape_keyindex_offset != -1)) {
    /* Keep the old verts in case we are working on* a key, which is done at the end. */

    /* Use the array in-place instead of duplicating the array. */
#if 0
    oldverts = MEM_dupallocN(me->mvert);
#else
    oldverts = me->mvert;
    me->mvert = NULL;
    CustomData_update_typemap(&me->vdata);
    CustomData_set_layer(&me->vdata, CD_MVERT, NULL);
#endif
  }

  /* Free custom data. */
  CustomData_free(&me->vdata, me->totvert);
  CustomData_free(&me->edata, me->totedge);
  CustomData_free(&me->fdata, me->totface);
  CustomData_free(&me->ldata, me->totloop);
  CustomData_free(&me->pdata, me->totpoly);

  /* Add new custom data. */
  me->totvert = tm->totvert;
  me->totedge = tm->totedge;
  me->totloop = tm->tottri*3;
  me->totpoly = tm->tottri;
  me->totface = tm->tottri;
  me->act_face = -1;

  {
    CustomData_MeshMasks mask = CD_MASK_MESH;
    CustomData_MeshMasks_update(&mask, &params->cd_mask_extra);
    CustomData_copy(&tm->vdata, &me->vdata, mask.vmask, CD_CALLOC, me->totvert);
    CustomData_copy(&tm->edata, &me->edata, mask.emask, CD_CALLOC, me->totedge);
    CustomData_copy(&tm->ldata, &me->ldata, mask.lmask, CD_CALLOC, me->totloop);
    CustomData_copy(&tm->tdata, &me->pdata, mask.pmask, CD_CALLOC, me->totpoly);
  }

  MVert *mvert = me->totvert ? MEM_callocN(sizeof(MVert) * me->totvert, "tm_to_me.vert") : NULL;
  MEdge *medge = me->totedge ? MEM_callocN(sizeof(MEdge) * me->totedge, "tm_to_me.edge") : NULL;
  MLoop *mloop = me->totloop ? MEM_callocN(sizeof(MLoop) * me->totloop, "tm_to_me.loop") : NULL;
  MPoly *mpoly = me->totface ? MEM_callocN(sizeof(MPoly) * me->totpoly, "tm_to_me.poly") : NULL;
  MFace *mface = me->totface ? MEM_callocN(sizeof(MFace) * me->totface, "tm_to_me.face") : NULL;

  CustomData_add_layer(&me->vdata, CD_MVERT, CD_ASSIGN, mvert, me->totvert);
  CustomData_add_layer(&me->edata, CD_MEDGE, CD_ASSIGN, medge, me->totedge);
  CustomData_add_layer(&me->ldata, CD_MLOOP, CD_ASSIGN, mloop, me->totloop);
  CustomData_add_layer(&me->pdata, CD_MPOLY, CD_ASSIGN, mpoly, me->totpoly);

  me->cd_flag = BLI_trimesh_mesh_cd_flag_from_bmesh(tm);

  /* This is called again, 'dotess' arg is used there. */
  BKE_mesh_update_customdata_pointers(me, 0);

  TM_TriMeshIter iter;
  TM_vert_iternew(tm, &iter);
  v = TM_iterstep(&iter);
  int i = 0;

  for (; v; v = TM_iterstep(&iter), mvert++, i++) {
    copy_v3_v3(mvert->co, v->co);
    normal_float_to_short_v3(mvert->no, v->no);
    mvert->flag = BLI_trimesh_vert_flag_to_mflag(v->flag);
    v->index = i;

    /* Copy over custom-data. */
    CustomData_from_bmesh_block(&tm->vdata, &me->vdata, v->customdata, i);

    if (cd_vert_bweight_offset != -1) {
      mvert->bweight = TM_ELEM_CD_GET_FLOAT_AS_UCHAR(v, cd_vert_bweight_offset);
    }
  }

  TM_edge_iternew(tm, &iter);
  e = TM_iterstep(&iter);
  i = 0;

  MEdge *med = medge;
  for (; e; e = TM_iterstep(&iter), i++, med++) {
    med->v1 = e->v1->index;
    med->v2 = e->v2->index;
    e->index = i;

    med->flag = BLI_trimesh_edge_flag_to_mflag(e->flag);

    /* Copy over custom-data. */
    CustomData_from_bmesh_block(&tm->edata, &me->edata, e->customdata, i);

    tmesh_quick_edgedraw_flag(med, e);

    if (cd_edge_crease_offset != -1) {
      med->crease = TM_ELEM_CD_GET_FLOAT_AS_UCHAR(e, cd_edge_crease_offset);
    }
    if (cd_edge_bweight_offset != -1) {
      med->bweight = TM_ELEM_CD_GET_FLOAT_AS_UCHAR(e, cd_edge_bweight_offset);
    }
  }

  TM_tri_iternew(tm, &iter);
  f = TM_iterstep(&iter);
  i = 0;

  int j = 0;
  for (; f; f = TM_iterstep(&iter), i++, mpoly++) {
    mpoly->loopstart = j;
    mpoly->totloop = 3;
    mpoly->mat_nr = f->mat_nr;

    for (int k=0; k<3; k++, j++, mloop++) {
      TMLoopData *ld = TM_GET_TRI_LOOP(f, k);

      mloop->e = TM_GET_TRI_EDGE(f, k)->index;
      mloop->e = TM_GET_TRI_VERT(f, k)->index;

      /* Copy over custom-data. */
      CustomData_from_bmesh_block(&tm->ldata, &me->ldata, ld->customdata, j);
    }

    /* Copy over custom-data. */
    CustomData_from_bmesh_block(&tm->tdata, &me->pdata, f->customdata, i);
  }

  /* Patch hook indices and vertex parents. */
  if (params->calc_object_remap && (ototvert > 0)) {
    BLI_assert(bmain != NULL);
    Object *ob;
    ModifierData *md;
    TMVert **vertMap = NULL;
    TMVert *eve = NULL;

    for (ob = bmain->objects.first; ob; ob = ob->id.next) {
      if ((ob->parent) && (ob->parent->data == me) && ELEM(ob->partype, PARVERT1, PARVERT3)) {

        if (vertMap == NULL) {
          vertMap = tm_to_mesh_vertex_map(tm, ototvert);
        }

        if (ob->par1 < ototvert) {
          eve = vertMap[ob->par1];
          if (eve) {
            ob->par1 = eve->index;
          }
        }
        if (ob->par2 < ototvert) {
          eve = vertMap[ob->par2];
          if (eve) {
            ob->par2 = eve->index;
          }
        }
        if (ob->par3 < ototvert) {
          eve = vertMap[ob->par3];
          if (eve) {
            ob->par3 = eve->index;
          }
        }
      }
      if (ob->data == me) {
        for (md = ob->modifiers.first; md; md = md->next) {
          if (md->type == eModifierType_Hook) {
            HookModifierData *hmd = (HookModifierData *)md;

            if (vertMap == NULL) {
              vertMap = tm_to_mesh_vertex_map(tm, ototvert);
            }

            for (i = j = 0; i < hmd->totindex; i++) {
              if (hmd->indexar[i] < ototvert) {
                eve = vertMap[hmd->indexar[i]];

                if (eve) {
                  hmd->indexar[j++] = eve->index;
                }
              }
              else {
                j++;
              }
            }

            hmd->totindex = j;
          }
        }
      }
    }

    if (vertMap) {
      MEM_freeN(vertMap);
    }
  }

  BKE_mesh_update_customdata_pointers(me, false);
}

static void resize_table(void ***table, int *size, int newsize) {
  if (!*table || newsize > *size) {
    *size = newsize*2;

    if (*table) {
      *table = MEM_reallocN(*table, newsize*sizeof(void*));
    } else {
      *table = MEM_mallocN(sizeof(void*)*newsize, "trimesh table");
    }
  }
}

static int TM_get_elem_count(TM_TriMesh *tm, int type) {
  switch (type) {
  case TM_VERTEX:
    return tm->totvert;
  case TM_EDGE:
    return tm->totedge;
  /*
  case TM_LOOP:
    return tm->totloop;*/
  case TM_TRI:
    return tm->tottri;
  }

  return -1;
}
static bool is_table_dirty(TM_TriMesh *tm, void **table, int tot, int type) {
  if (!type) {
     return false;
  }

  bool ret = !table;

  ret = ret || tot < TM_get_elem_count(tm, type);
  ret = ret || (tm->elem_table_dirty & type);

  return ret;
}

void TM_mesh_elem_table_ensure(TM_TriMesh *tm, int typemask) {
  if (is_table_dirty(tm, tm->vtable, tm->vtable_tot, typemask & TM_VERTEX)) {
    TM_TriMeshIter iter;
    TMVert *v;

    resize_table(&tm->vtable, &tm->vtable_tot, tm->totvert);

    TM_vert_iternew(tm, &iter);
    v=TM_iterstep(&iter);

    for (int i=0; v; v=TM_iterstep(&iter), i++) {
      tm->vtable[i] = v;
    }
  }
  if (is_table_dirty(tm, tm->etable, tm->etable_tot, typemask & TM_EDGE)) {
    TM_TriMeshIter iter;
    TMEdge *v;

    resize_table(&tm->etable, &tm->etable_tot, tm->totedge);

    TM_vert_iternew(tm, &iter);
    v=TM_iterstep(&iter);

    for (int i=0; v; v=TM_iterstep(&iter), i++) {
      tm->etable[i] = v;
    }
  }
  if (is_table_dirty(tm, tm->ttable, tm->ttable_tot, typemask & TM_TRI)) {
    TM_TriMeshIter iter;
    TMFace *v;

    resize_table(&tm->ttable, &tm->ttable_tot, tm->tottri);

    TM_tri_iternew(tm, &iter);
    v=TM_iterstep(&iter);

    for (int i=0; v; v=TM_iterstep(&iter), i++) {
      tm->ttable[i] = v;
    }
  }

  tm->elem_table_dirty &= ~typemask;
}

void TM_mesh_elem_index_ensure(TM_TriMesh *tm, int typemask) {
  if ((typemask & tm->elem_index_dirty) & TM_VERTEX) {
    TM_TriMeshIter iter;
    TMVert *v;

    TM_vert_iternew(tm, &iter);
    v=TM_iterstep(&iter);

    for (int i=0; v; v=TM_iterstep(&iter), i++) {
      v->index = i;
    }
  }
  if ((typemask & tm->elem_index_dirty) & TM_EDGE) {
    TM_TriMeshIter iter;
    TMEdge *v;

    TM_vert_iternew(tm, &iter);
    v=TM_iterstep(&iter);

    for (int i=0; v; v=TM_iterstep(&iter), i++) {
      v->index = i;
    }
  }
  if ((typemask & tm->elem_index_dirty) & TM_TRI) {
    TM_TriMeshIter iter;
    TMFace *v;

    TM_tri_iternew(tm, &iter);
    v=TM_iterstep(&iter);

    for (int i=0; v; v=TM_iterstep(&iter), i++) {
      v->index = i;
    }
  }

  tm->elem_index_dirty &= ~typemask;
}

void TM_mesh_normals_update(TM_TriMesh *tm) {
  TM_TriMeshIter iter;
  TMVert *v;
  TMFace *f;

  TM_ITER_MESH(f, &iter, tm, TM_TRIS_OF_MESH) {
    TM_calc_tri_normal(f);
  }

  TM_ITER_MESH(v, &iter, tm, TM_VERTS_OF_MESH) {
    TM_calc_vert_normal(v, false);
  }
}

