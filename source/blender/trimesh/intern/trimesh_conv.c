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

#include "BLI_alloca.h"
#include "BLI_array.h"
#include "BLI_math.h"
#include "BLI_threadsafe_mempool.h"

#include "DNA_customdata_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "atomic_ops.h"

#include "BLI_ghash.h"
#include "BLI_utildefines.h"

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

#include "bmesh.h"
#include "trimesh.h"
#include "trimesh_private.h"

/* ME -> BM */
short TM_vert_flag_from_mflag(const char meflag)
{
  return (((meflag & SELECT) ? SELECT : 0) | ((meflag & ME_HIDE) ? TM_ELEM_HIDDEN : 0));
}
short TM_edge_flag_from_mflag(const short meflag)
{
  return (((meflag & SELECT) ? SELECT : 0) | ((meflag & ME_SEAM) ? TRIMESH_SEAM : 0) |
          ((meflag & ME_EDGEDRAW) ? TRIMESH_EDGEDRAW : 0) |
          ((meflag & ME_SHARP) ? TRIMESH_SHARP : 0) | ((meflag & ME_HIDE) ? TM_ELEM_HIDDEN : 0));
}
short TM_face_flag_from_mflag(const char meflag)
{
  return (((meflag & ME_FACE_SEL) ? SELECT : 0) | ((meflag & ME_SMOOTH) ? TRIMESH_SMOOTH : 0) |
          ((meflag & ME_HIDE) ? TM_ELEM_HIDDEN : 0));
}

static void update_data_blocks(TM_TriMesh *tm, CustomData *olddata, CustomData *data)
{
  TM_TriMeshIter iter;
  BLI_ThreadSafePool *oldpool = olddata->tpool;
  void *block;

  if (data == &tm->vdata) {
    TMVert *eve;

    CustomData_trimesh_init_pool(tm, data, tm->totvert, TM_VERTEX);

    TM_ITER_MESH (eve, &iter, tm, TM_VERTS_OF_MESH) {
      block = NULL;
      CustomData_bmesh_set_default(data, &block);
      CustomData_bmesh_copy_data(olddata, data, eve->customdata, &block);
      CustomData_bmesh_free_block(olddata, &eve->customdata);
      eve->customdata = block;
    }
  }
  else if (data == &tm->edata) {
    TMEdge *eed;

    CustomData_trimesh_init_pool(tm, data, tm->totedge, TM_EDGE);

    TM_ITER_MESH (eed, &iter, tm, TM_EDGES_OF_MESH) {
      block = NULL;
      CustomData_bmesh_set_default(data, &block);
      CustomData_bmesh_copy_data(olddata, data, eed->customdata, &block);
      CustomData_bmesh_free_block(olddata, &eed->customdata);
      eed->customdata = block;
    }
  }
  else if (data == &tm->ldata) {
    TMFace *efa;

    CustomData_trimesh_init_pool(tm, data, tm->tottri * 3, TM_LOOP);
    TM_ITER_MESH (efa, &iter, tm, TM_TRIS_OF_MESH) {
      for (int i = 0; i < 3; i++) {
        TMLoopData *l = TM_GET_TRI_LOOP(efa, i);
        block = NULL;
        CustomData_bmesh_set_default(data, &block);
        CustomData_bmesh_copy_data(olddata, data, l->customdata, &block);
        CustomData_bmesh_free_block(olddata, &l->customdata);
        l->customdata = block;
      }
    }
  }
  else if (data == &tm->tdata) {
    TMFace *efa;

    CustomData_trimesh_init_pool(tm, data, tm->tottri, TM_TRI);

    TM_ITER_MESH (efa, &iter, tm, TM_TRIS_OF_MESH) {
      block = NULL;
      CustomData_bmesh_set_default(data, &block);
      CustomData_bmesh_copy_data(olddata, data, efa->customdata, &block);
      CustomData_bmesh_free_block(olddata, &efa->customdata);
      efa->customdata = block;
    }
  }
  else {
    /* should never reach this! */
    BLI_assert(0);
  }

  if (oldpool) {
    /* this should never happen but can when dissolve fails - [#28960] */
    BLI_assert(data->tpool != oldpool);

    BLI_safepool_destroy(oldpool);
  }
}

void TM_data_layer_add(TM_TriMesh *tm, CustomData *data, int type)
{
  CustomData olddata;

  olddata = *data;
  olddata.layers = (olddata.layers) ? MEM_dupallocN(olddata.layers) : NULL;

  /* the pool is now owned by olddata and must not be shared */
  data->tpool = NULL;

  CustomData_add_layer(data, type, CD_DEFAULT, NULL, 0);

  update_data_blocks(tm, &olddata, data);
  if (olddata.layers) {
    MEM_freeN(olddata.layers);
  }
}

void TM_data_layer_add_named(TM_TriMesh *bm, CustomData *data, int type, const char *name)
{
  CustomData olddata;

  olddata = *data;
  olddata.layers = (olddata.layers) ? MEM_dupallocN(olddata.layers) : NULL;

  /* the pool is now owned by olddata and must not be shared */
  data->tpool = NULL;

  CustomData_add_layer_named(data, type, CD_DEFAULT, NULL, 0, name);

  update_data_blocks(bm, &olddata, data);
  if (olddata.layers) {
    MEM_freeN(olddata.layers);
  }
}

void TM_data_layer_free(TM_TriMesh *tm, CustomData *data, int type)
{
  CustomData olddata;
  bool has_layer;

  olddata = *data;
  olddata.layers = (olddata.layers) ? MEM_dupallocN(olddata.layers) : NULL;

  /* the pool is now owned by olddata and must not be shared */
  data->tpool = NULL;

  has_layer = CustomData_free_layer_active(data, type, 0);
  /* assert because its expensive to realloc - better not do if layer isnt present */
  BLI_assert(has_layer != false);
  UNUSED_VARS_NDEBUG(has_layer);

  update_data_blocks(tm, &olddata, data);
  if (olddata.layers) {
    MEM_freeN(olddata.layers);
  }
}

void TM_mesh_cd_flag_apply(TM_TriMesh *bm, const char cd_flag)
{
  /* CustomData_bmesh_init_pool() must run first */
  BLI_assert(bm->vdata.totlayer == 0 || bm->vdata.tpool != NULL);
  BLI_assert(bm->edata.totlayer == 0 || bm->edata.tpool != NULL);
  BLI_assert(bm->tdata.totlayer == 0 || bm->tdata.tpool != NULL);

  if (cd_flag & ME_CDFLAG_VERT_BWEIGHT) {
    if (!CustomData_has_layer(&bm->vdata, CD_BWEIGHT)) {
      TM_data_layer_add(bm, &bm->vdata, CD_BWEIGHT);
    }
  }
  else {
    if (CustomData_has_layer(&bm->vdata, CD_BWEIGHT)) {
      TM_data_layer_free(bm, &bm->vdata, CD_BWEIGHT);
    }
  }

  if (cd_flag & ME_CDFLAG_EDGE_BWEIGHT) {
    if (!CustomData_has_layer(&bm->edata, CD_BWEIGHT)) {
      TM_data_layer_add(bm, &bm->edata, CD_BWEIGHT);
    }
  }
  else {
    if (CustomData_has_layer(&bm->edata, CD_BWEIGHT)) {
      TM_data_layer_free(bm, &bm->edata, CD_BWEIGHT);
    }
  }

  if (cd_flag & ME_CDFLAG_EDGE_CREASE) {
    if (!CustomData_has_layer(&bm->edata, CD_CREASE)) {
      TM_data_layer_add(bm, &bm->edata, CD_CREASE);
    }
  }
  else {
    if (CustomData_has_layer(&bm->edata, CD_CREASE)) {
      TM_data_layer_free(bm, &bm->edata, CD_CREASE);
    }
  }
}

void TM_mesh_tm_from_me(TM_TriMesh *bm, const Mesh *me, const struct TriMeshFromMeshParams *params)
{
  const bool is_new = !(bm->totvert || (bm->vdata.totlayer || bm->edata.totlayer ||
                                        bm->tdata.totlayer || bm->ldata.totlayer));
  MVert *mvert;
  MEdge *medge;
  MLoop *mloop;
  MPoly *mp;
  KeyBlock *actkey, *block;
  TMVert *v, **vtable = NULL;
  TMEdge *e, **etable = NULL;
  TMFace *f, **ftable = NULL;
  float(*keyco)[3] = NULL;
  int totloops, i;
  CustomData_MeshMasks mask = CD_MASK_BMESH;
  CustomData_MeshMasks_update(&mask, &params->cd_mask_extra);

  if (!me || !me->totvert) {
    if (me && is_new) { /* No verts? still copy custom-data layout. */
      CustomData_copy(&me->vdata, &bm->vdata, mask.vmask, CD_ASSIGN, 0);
      CustomData_copy(&me->edata, &bm->edata, mask.emask, CD_ASSIGN, 0);
      CustomData_copy(&me->ldata, &bm->ldata, mask.lmask, CD_ASSIGN, 0);
      CustomData_copy(&me->pdata, &bm->tdata, mask.pmask, CD_ASSIGN, 0);

      CustomData_trimesh_init_pool(bm, &bm->vdata, 0, TM_VERTEX);
      CustomData_trimesh_init_pool(bm, &bm->edata, 0, TM_EDGE);
      CustomData_trimesh_init_pool(bm, &bm->ldata, 0, TM_LOOP);
      CustomData_trimesh_init_pool(bm, &bm->tdata, 0, TM_TRI);
    }
    return; /* Sanity check. */
  }

  if (is_new) {
    CustomData_copy(&me->vdata, &bm->vdata, mask.vmask, CD_CALLOC, 0);
    CustomData_copy(&me->edata, &bm->edata, mask.emask, CD_CALLOC, 0);
    CustomData_copy(&me->ldata, &bm->ldata, mask.lmask, CD_CALLOC, 0);
    CustomData_copy(&me->pdata, &bm->tdata, mask.pmask, CD_CALLOC, 0);
  }
  else {
    CustomData_trimesh_merge(&me->vdata, &bm->vdata, mask.vmask, CD_CALLOC, bm, TM_VERTEX);
    CustomData_trimesh_merge(&me->edata, &bm->edata, mask.emask, CD_CALLOC, bm, TM_EDGE);
    CustomData_trimesh_merge(&me->ldata, &bm->ldata, mask.lmask, CD_CALLOC, bm, TM_LOOP);
    CustomData_trimesh_merge(&me->pdata, &bm->tdata, mask.pmask, CD_CALLOC, bm, TM_TRI);
  }

  /* -------------------------------------------------------------------- */
  /* Shape Key */
  int tot_shape_keys = me->key ? BLI_listbase_count(&me->key->block) : 0;
  if (is_new == false) {
    tot_shape_keys = min_ii(tot_shape_keys, CustomData_number_of_layers(&bm->vdata, CD_SHAPEKEY));
  }
  const float(**shape_key_table)[3] = tot_shape_keys ?
                                          BLI_array_alloca(shape_key_table, tot_shape_keys) :
                                          NULL;

  if ((params->active_shapekey != 0) && (me->key != NULL)) {
    actkey = BLI_findlink(&me->key->block, params->active_shapekey - 1);
  }
  else {
    actkey = NULL;
  }

  if (is_new) {
    if (tot_shape_keys || params->add_key_index) {
      CustomData_add_layer(&bm->vdata, CD_SHAPE_KEYINDEX, CD_ASSIGN, NULL, 0);
    }
  }

  if (tot_shape_keys) {
    if (is_new) {
      /* Check if we need to generate unique ids for the shape-keys.
       * This also exists in the file reading code, but is here for a sanity check. */
      if (!me->key->uidgen) {
        fprintf(stderr,
                "%s had to generate shape key uid's in a situation we shouldn't need to! "
                "(bmesh internal error)\n",
                __func__);

        me->key->uidgen = 1;
        for (block = me->key->block.first; block; block = block->next) {
          block->uid = me->key->uidgen++;
        }
      }
    }

    if (actkey && actkey->totelem == me->totvert) {
      keyco = params->use_shapekey ? actkey->data : NULL;
      if (is_new) {
        bm->shapenr = params->active_shapekey;
      }
    }

    for (i = 0, block = me->key->block.first; i < tot_shape_keys; block = block->next, i++) {
      if (is_new) {
        CustomData_add_layer_named(&bm->vdata, CD_SHAPEKEY, CD_ASSIGN, NULL, 0, block->name);
        int j = CustomData_get_layer_index_n(&bm->vdata, CD_SHAPEKEY, i);
        bm->vdata.layers[j].uid = block->uid;
      }
      shape_key_table[i] = (const float(*)[3])block->data;
    }
  }

  if (is_new) {
    CustomData_trimesh_init_pool(bm, &bm->vdata, me->totvert, TM_VERTEX);
    CustomData_trimesh_init_pool(bm, &bm->edata, me->totedge, TM_EDGE);
    CustomData_trimesh_init_pool(bm, &bm->ldata, me->totloop, TM_LOOP);
    CustomData_trimesh_init_pool(bm, &bm->tdata, me->totpoly, TM_TRI);

    TM_mesh_cd_flag_apply(bm, me->cd_flag);
  }

  const int cd_vert_bweight_offset = CustomData_get_offset(&bm->vdata, CD_BWEIGHT);
  const int cd_edge_bweight_offset = CustomData_get_offset(&bm->edata, CD_BWEIGHT);
  const int cd_edge_crease_offset = CustomData_get_offset(&bm->edata, CD_CREASE);
  const int cd_shape_key_offset = me->key ? CustomData_get_offset(&bm->vdata, CD_SHAPEKEY) : -1;
  const int cd_shape_keyindex_offset = is_new && (tot_shape_keys || params->add_key_index) ?
                                           CustomData_get_offset(&bm->vdata, CD_SHAPE_KEYINDEX) :
                                           -1;

  vtable = MEM_mallocN(sizeof(TMVert **) * me->totvert, __func__);

  for (i = 0, mvert = me->mvert; i < me->totvert; i++, mvert++) {
    v = vtable[i] = TM_make_vert(bm, keyco ? keyco[i] : mvert->co, NULL, true);
    v->index = i;

    /* Transfer flag. */
    v->flag = TM_vert_flag_from_mflag(mvert->flag);

    normal_short_to_float_v3(v->no, mvert->no);

    /* Copy Custom Data */
    CustomData_to_bmesh_block(&me->vdata, &bm->vdata, i, &v->customdata, true);

    if (cd_vert_bweight_offset != -1) {
      TM_ELEM_CD_SET_FLOAT(v, cd_vert_bweight_offset, (float)mvert->bweight / 255.0f);
    }

    /* Set shape key original index. */
    if (cd_shape_keyindex_offset != -1) {
      TM_ELEM_CD_SET_INT(v, cd_shape_keyindex_offset, i);
    }

    /* Set shape-key data. */
    if (tot_shape_keys) {
      float(*co_dst)[3] = TM_ELEM_CD_GET_VOID_P(v, cd_shape_key_offset);
      for (int j = 0; j < tot_shape_keys; j++, co_dst++) {
        copy_v3_v3(*co_dst, shape_key_table[j][i]);
      }
    }
  }
  if (is_new) {
    bm->elem_index_dirty &= ~TM_VERTEX; /* Added in order, clear dirty flag. */
  }

  etable = MEM_mallocN(sizeof(TMEdge **) * me->totedge, __func__);

  medge = me->medge;
  for (i = 0; i < me->totedge; i++, medge++) {
    e = etable[i] = TM_get_edge(bm, vtable[medge->v1], vtable[medge->v2], true);
    e->index = i;

    /* Transfer flags. */
    e->flag = TM_edge_flag_from_mflag(medge->flag);

    /* Copy Custom Data */
    CustomData_to_bmesh_block(&me->edata, &bm->edata, i, &e->customdata, true);

    if (cd_edge_bweight_offset != -1) {
      TM_ELEM_CD_SET_FLOAT(e, cd_edge_bweight_offset, (float)medge->bweight / 255.0f);
    }
    if (cd_edge_crease_offset != -1) {
      TM_ELEM_CD_SET_FLOAT(e, cd_edge_crease_offset, (float)medge->crease / 255.0f);
    }
  }
  if (is_new) {
    bm->elem_index_dirty &= ~TM_EDGE; /* Added in order, clear dirty flag. */
  }

  mp = me->mpoly;
  mloop = me->mloop;
  for (i = 0; i < me->totpoly; i++, mp++) {
    MLoop *ml = mloop + mp->loopstart;

    for (int j = 1; j < mp->totloop - 1; j++) {
      int i1 = 0, i2 = j, i3 = j + 1;

      TMVert *v1 = vtable[ml[i1].v];
      TMVert *v2 = vtable[ml[i2].v];
      TMVert *v3 = vtable[ml[i3].v];

      TMFace *tri = TM_make_tri(bm, v1, v2, v3, true);
      tri->index = bm->tottri - 1;

      tri->flag = TM_face_flag_from_mflag(mp->flag);
      tri->mat_nr = mp->mat_nr;

      if (params->calc_face_normal) {
        TM_calc_tri_normal(tri);
      }

      /* Copy Custom Data */
      CustomData_to_bmesh_block(&me->pdata, &bm->tdata, i, &tri->customdata, true);

      i1 += mp->loopstart;
      i2 += mp->loopstart;
      i3 += mp->loopstart;

      CustomData_to_bmesh_block(&me->ldata, &bm->ldata, i1, &tri->l1->customdata, true);
      CustomData_to_bmesh_block(&me->ldata, &bm->ldata, i2, &tri->l2->customdata, true);
      CustomData_to_bmesh_block(&me->ldata, &bm->ldata, i3, &tri->l3->customdata, true);
    }
  }

  if (is_new) {
    bm->elem_index_dirty &= ~(TM_TRI | TM_LOOP); /* Added in order, clear dirty flag. */
  }

  MEM_freeN(vtable);
  MEM_freeN(etable);
  if (ftable) {
    MEM_freeN(ftable);
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
    }
    else {
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
  me->totloop = tm->tottri * 3;
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

    mpoly->flag = BLI_trimesh_tri_flag_to_mflag(f->flag);

    for (int k = 0; k < 3; k++, j++, mloop++) {
      TMLoopData *ld = TM_GET_TRI_LOOP(f, k);

      mloop->e = TM_GET_TRI_EDGE(f, k)->index;
      mloop->v = TM_GET_TRI_VERT(f, k)->index;

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
