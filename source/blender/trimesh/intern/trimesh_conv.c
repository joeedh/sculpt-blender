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
#include "BLI_threadsafe_mempool.h"
#include "BLI_array.h"
#include "BLI_alloca.h"

#include "DNA_key_types.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "atomic_ops.h"

#include "BLI_utildefines.h"
#include "BLI_ghash.h"

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
#include "trimesh.h"
#include "bmesh.h"

/* ME -> BM */
char TM_vert_flag_from_mflag(const char meflag)
{
  return (((meflag & SELECT) ? SELECT : 0) | ((meflag & ME_HIDE) ? TM_ELEM_HIDDEN : 0));
}
char TM_edge_flag_from_mflag(const short meflag)
{
  return (((meflag & SELECT) ? SELECT : 0) | ((meflag & ME_SEAM) ? TRIMESH_SEAM : 0) |
    ((meflag & ME_EDGEDRAW) ? TRIMESH_EDGEDRAW : 0) |
    ((meflag & ME_SHARP) == 0 ? TRIMESH_SHARP : 0) | /* invert */
    ((meflag & ME_HIDE) ? TM_ELEM_HIDDEN : 0));
}
char TM_face_flag_from_mflag(const char meflag)
{
  return (((meflag & ME_FACE_SEL) ? SELECT : 0) |
    ((meflag & ME_SMOOTH) ? TRIMESH_SMOOTH : 0) | ((meflag & ME_HIDE) ? TM_ELEM_HIDDEN : 0));
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

    CustomData_trimesh_init_pool(tm, data, tm->tottri*3, TM_LOOP);
    TM_ITER_MESH (efa, &iter, tm, TM_TRIS_OF_MESH) {
      for (int i=0; i<3; i++) {
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

void TM_mesh_tm_from_me(TM_TriMesh *bm, const Mesh *me, const struct TriMeshFromMeshParams *params) {
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
    v = vtable[i] = TM_make_vert(bm, keyco ? keyco[i] : mvert->co, NULL, 0, true);
    v->index = i;

    /* Transfer flag. */
    v->flag = mvert->flag & SELECT;
    if (mvert->flag & ME_HIDE) {
      v->flag |= TM_ELEM_HIDDEN;
    }

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
    e = etable[i] = TM_get_edge(
      bm, vtable[medge->v1], vtable[medge->v2], 0, true);
    e->index = i;

    /* Transfer flags. */
    e->flag = TM_edge_flag_from_mflag(medge->flag & ~SELECT);

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
  for (i=0; i<me->totpoly; i++, mp++) {
    MLoop *ml = mloop + mp->loopstart;

    for (int j=1; j<mp->totloop-1; j++) {
      int i1 = 0, i2 = j, i3 = j+1;

      TMVert *v1 = vtable[ml[i1].v];
      TMVert *v2 = vtable[ml[i2].v];
      TMVert *v3 = vtable[ml[i3].v];

      TMFace *tri = TM_make_tri(bm, v1, v2, v3, 0, true);
      tri->index = bm->tottri-1;

      tri->flag = TM_face_flag_from_mflag(mp->flag & ~ME_FACE_SEL);
      tri->mat_nr = mp->mat_nr;

      if (params->calc_face_normal) {
        TM_calc_tri_normal(tri);
      }

      /* Copy Custom Data */
      CustomData_to_bmesh_block(&me->pdata, &bm->tdata, i, &tri->customdata, true);

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
