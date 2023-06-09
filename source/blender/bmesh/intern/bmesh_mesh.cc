/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bmesh
 *
 * BM mesh level functions.
 */

#include "MEM_guardedalloc.h"

#include "DNA_listBase.h"
#include "DNA_scene_types.h"

#include "BLI_alloca.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_rand.h"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "BKE_customdata.h"
#include "BKE_mesh.hh"
#include "bmesh_hive_alloc_intern.hh"

#include "DNA_meshdata_types.h"

#include "bmesh.h"
#include "bmesh_private.h"
#include "range_tree.h"

using blender::Vector;

const BMAllocTemplate bm_mesh_allocsize_default = {512, 1024, 2048, 512};
const BMAllocTemplate bm_mesh_chunksize_default = {512, 1024, 2048, 512};

static void bm_alloc_toolflags(BMesh *bm);

static void bm_mempool_init(BMesh *bm,
                            const BMAllocTemplate * /*allocsize*/,
                            const bool /*use_toolflags*/)
{
  bm->vhive = make_vert_hive(bm);
  bm->ehive = make_edge_hive(bm);
  bm->lhive = make_loop_hive(bm);
  bm->fhive = make_face_hive(bm);
}

void BM_mesh_elem_toolflags_ensure(BMesh *bm)
{
  BLI_assert(bm->use_toolflags);

  if (!CustomData_has_layer(&bm->vdata, CD_TOOLFLAGS)) {
    if (bm->vtoolflagpool) {
      printf("%s: Error: toolflags were deallocated improperly\n", __func__);

      BM_mesh_elem_toolflags_clear(bm);

      bm_alloc_toolflags_cdlayers(bm, true);
    }
  }

  if (bm->vtoolflagpool && bm->etoolflagpool && bm->ftoolflagpool) {
    return;
  }

  bm->vtoolflagpool = BLI_mempool_create(sizeof(BMFlagLayer), bm->totvert, 512, BLI_MEMPOOL_NOP);
  bm->etoolflagpool = BLI_mempool_create(sizeof(BMFlagLayer), bm->totedge, 512, BLI_MEMPOOL_NOP);
  bm->ftoolflagpool = BLI_mempool_create(sizeof(BMFlagLayer), bm->totface, 512, BLI_MEMPOOL_NOP);

  bm_alloc_toolflags(bm);

  bm->totflags = 1;
}

void BM_mesh_elem_toolflags_clear(BMesh *bm)
{
  bool haveflags = bm->vtoolflagpool || bm->etoolflagpool || bm->ftoolflagpool;

  if (bm->vtoolflagpool) {
    BLI_mempool_destroy(bm->vtoolflagpool);
    bm->vtoolflagpool = nullptr;
  }
  if (bm->etoolflagpool) {
    BLI_mempool_destroy(bm->etoolflagpool);
    bm->etoolflagpool = nullptr;
  }
  if (bm->ftoolflagpool) {
    BLI_mempool_destroy(bm->ftoolflagpool);
    bm->ftoolflagpool = nullptr;
  }

  if (haveflags) {
    BM_data_layer_free(bm, &bm->vdata, CD_TOOLFLAGS);
    BM_data_layer_free(bm, &bm->edata, CD_TOOLFLAGS);
    BM_data_layer_free(bm, &bm->pdata, CD_TOOLFLAGS);
  }
}

BMesh *BM_mesh_create(const BMAllocTemplate *allocsize, const BMeshCreateParams *params)
{
  /* allocate the structure */
  BMesh *bm = static_cast<BMesh *>(MEM_callocN(sizeof(BMesh), __func__));

  /* allocate the memory pools for the mesh elements */
  bm_mempool_init(bm, allocsize, params->use_toolflags);

  /* allocate one flag pool that we don't get rid of. */
  bm->use_toolflags = params->use_toolflags;
  bm->toolflag_index = 0;
  bm->totflags = 0;

  CustomData_reset(&bm->vdata);
  CustomData_reset(&bm->edata);
  CustomData_reset(&bm->ldata);
  CustomData_reset(&bm->pdata);

  bool init_cdata_pools = false;

  if (bm->use_toolflags) {
    init_cdata_pools = true;
    bm_alloc_toolflags_cdlayers(bm, false);
  }

  if (init_cdata_pools) {
    if (bm->vdata.totlayer) {
      CustomData_bmesh_init_pool(&bm->vdata, 0, BM_VERT);
    }
    if (bm->edata.totlayer) {
      CustomData_bmesh_init_pool(&bm->edata, 0, BM_EDGE);
    }
    if (bm->ldata.totlayer) {
      CustomData_bmesh_init_pool(&bm->ldata, 0, BM_LOOP);
    }
    if (bm->pdata.totlayer) {
      CustomData_bmesh_init_pool(&bm->pdata, 0, BM_FACE);
    }
  }

  if (bm->use_toolflags) {
    BM_mesh_elem_toolflags_ensure(bm);
  }

  return bm;
}

static void customdata_pool_destroy(BMesh * /*bm*/, CustomData *cdata)
{
  if (cdata->hive) {
    customdata_hive_destroy(cdata->hive);
    cdata->hive = nullptr;
  }
}

void BM_mesh_data_free(BMesh *bm)
{
  BMVert *v;
  BMEdge *e;
  BMLoop *l;
  BMFace *f;

  BMIter iter;
  BMIter itersub;

  const bool is_ldata_free = CustomData_bmesh_has_free(&bm->ldata);
  const bool is_pdata_free = CustomData_bmesh_has_free(&bm->pdata);

  /* Check if we have to call free, if not we can avoid a lot of looping */
  if (CustomData_bmesh_has_free(&(bm->vdata))) {
    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      CustomData_bmesh_free_block(&(bm->vdata), &(v->head.data));
    }
  }
  if (CustomData_bmesh_has_free(&(bm->edata))) {
    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      CustomData_bmesh_free_block(&(bm->edata), &(e->head.data));
    }
  }

  if (is_ldata_free || is_pdata_free) {
    BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
      if (is_pdata_free) {
        CustomData_bmesh_free_block(&(bm->pdata), &(f->head.data));
      }
      if (is_ldata_free) {
        BM_ITER_ELEM (l, &itersub, f, BM_LOOPS_OF_FACE) {
          CustomData_bmesh_free_block(&(bm->ldata), &(l->head.data));
        }
      }
    }
  }

  /* Free custom data pools, This should probably go in CustomData_free? */
  if (bm->vdata.totlayer) {
    customdata_pool_destroy(bm, &bm->vdata);
  }
  if (bm->edata.totlayer) {
    customdata_pool_destroy(bm, &bm->edata);
  }
  if (bm->ldata.totlayer) {
    customdata_pool_destroy(bm, &bm->ldata);
  }
  if (bm->pdata.totlayer) {
    customdata_pool_destroy(bm, &bm->pdata);
  }

  /* free custom data */
  CustomData_free(&bm->vdata, 0);
  CustomData_free(&bm->edata, 0);
  CustomData_free(&bm->ldata, 0);
  CustomData_free(&bm->pdata, 0);

  /* destroy element pools */
  free_vert_hive(bm->vhive);
  free_edge_hive(bm->ehive);
  free_loop_hive(bm->lhive);
  free_face_hive(bm->fhive);

  if (bm->vtable) {
    MEM_freeN(bm->vtable);
  }
  if (bm->etable) {
    MEM_freeN(bm->etable);
  }
  if (bm->ftable) {
    MEM_freeN(bm->ftable);
  }

  /* destroy flag pools */

  if (bm->vtoolflagpool) {
    BLI_mempool_destroy(bm->vtoolflagpool);
    bm->vtoolflagpool = nullptr;
  }
  if (bm->etoolflagpool) {
    BLI_mempool_destroy(bm->etoolflagpool);
    bm->etoolflagpool = nullptr;
  }
  if (bm->ftoolflagpool) {
    BLI_mempool_destroy(bm->ftoolflagpool);
    bm->ftoolflagpool = nullptr;
  }

#ifdef USE_BMESH_HOLES
  BLI_mempool_destroy(bm->looplistpool);
#endif

  BLI_freelistN(&bm->selected);

  if (bm->lnor_spacearr) {
    BKE_lnor_spacearr_free(bm->lnor_spacearr);
    MEM_freeN(bm->lnor_spacearr);
  }

  BMO_error_clear(bm);
}

void BM_mesh_clear(BMesh *bm)
{
  const bool use_toolflags = bm->use_toolflags;

  /* free old mesh */
  BM_mesh_data_free(bm);
  memset(bm, 0, sizeof(BMesh));

  /* allocate the memory pools for the mesh elements */
  bm_mempool_init(bm, &bm_mesh_allocsize_default, use_toolflags);

  bm->use_toolflags = use_toolflags;
  bm->toolflag_index = 0;
  bm->totflags = 0;

  CustomData_reset(&bm->vdata);
  CustomData_reset(&bm->edata);
  CustomData_reset(&bm->ldata);
  CustomData_reset(&bm->pdata);
}

void BM_mesh_free(BMesh *bm)
{
  BM_mesh_data_free(bm);

  if (bm->py_handle) {
    /* keep this out of 'BM_mesh_data_free' because we want python
     * to be able to clear the mesh and maintain access. */
    bpy_bm_generic_invalidate(static_cast<BPy_BMGeneric *>(bm->py_handle));
    bm->py_handle = nullptr;
  }

  MEM_freeN(bm);
}
void bmesh_edit_begin(BMesh * /*bm*/, BMOpTypeFlag /*type_flag*/)
{
  /* Most operators seem to be using BMO_OPTYPE_FLAG_UNTAN_MULTIRES to change the MDisps to
   * absolute space during mesh edits. With this enabled, changes to the topology
   * (loop cuts, edge subdivides, etc) are not reflected in the higher levels of
   * the mesh at all, which doesn't seem right. Turning off completely for now,
   * until this is shown to be better for certain types of mesh edits. */
#ifdef BMOP_UNTAN_MULTIRES_ENABLED
  /* switch multires data out of tangent space */
  if ((type_flag & BMO_OPTYPE_FLAG_UNTAN_MULTIRES) && CustomData_has_layer(&bm->ldata, CD_MDISPS))
  {
    bmesh_mdisps_space_set(bm, MULTIRES_SPACE_TANGENT, MULTIRES_SPACE_ABSOLUTE);

    /* ensure correct normals, if possible */
    bmesh_rationalize_normals(bm, 0);
    BM_mesh_normals_update(bm);
  }
#endif
}

void bmesh_edit_end(BMesh *bm, BMOpTypeFlag type_flag)
{
  ListBase select_history;

  /* BMO_OPTYPE_FLAG_UNTAN_MULTIRES disabled for now, see comment above in bmesh_edit_begin. */
#ifdef BMOP_UNTAN_MULTIRES_ENABLED
  /* switch multires data into tangent space */
  if ((flag & BMO_OPTYPE_FLAG_UNTAN_MULTIRES) && CustomData_has_layer(&bm->ldata, CD_MDISPS)) {
    /* set normals to their previous winding */
    bmesh_rationalize_normals(bm, 1);
    bmesh_mdisps_space_set(bm, MULTIRES_SPACE_ABSOLUTE, MULTIRES_SPACE_TANGENT);
  }
  else if (flag & BMO_OP_FLAG_RATIONALIZE_NORMALS) {
    bmesh_rationalize_normals(bm, 1);
  }
#endif

  /* compute normals, clear temp flags and flush selections */
  if (type_flag & BMO_OPTYPE_FLAG_NORMALS_CALC) {
    bm->spacearr_dirty |= BM_SPACEARR_DIRTY_ALL;
    BM_mesh_normals_update(bm);
  }

  if ((type_flag & BMO_OPTYPE_FLAG_SELECT_VALIDATE) == 0) {
    select_history = bm->selected;
    BLI_listbase_clear(&bm->selected);
  }

  if (type_flag & BMO_OPTYPE_FLAG_SELECT_FLUSH) {
    BM_mesh_select_mode_flush(bm);
  }

  if ((type_flag & BMO_OPTYPE_FLAG_SELECT_VALIDATE) == 0) {
    bm->selected = select_history;
  }
  if (type_flag & BMO_OPTYPE_FLAG_INVALIDATE_CLNOR_ALL) {
    bm->spacearr_dirty |= BM_SPACEARR_DIRTY_ALL;
  }
}

void BM_mesh_elem_index_ensure_ex(BMesh *bm, const char htype, int elem_offset[4])
{

#ifdef DEBUG
  BM_ELEM_INDEX_VALIDATE(bm, "Should Never Fail!", __func__);
#endif

  if (elem_offset == nullptr) {
    /* Simple case. */
    const char htype_needed = bm->elem_index_dirty & htype;
    if (htype_needed == 0) {
      goto finally;
    }
  }

  if (htype & BM_VERT) {
    if ((bm->elem_index_dirty & BM_VERT) || (elem_offset && elem_offset[0])) {
      BMIter iter;
      BMElem *ele;

      int index = elem_offset ? elem_offset[0] : 0;
      BM_ITER_MESH (ele, &iter, bm, BM_VERTS_OF_MESH) {
        BM_elem_index_set(ele, index++); /* set_ok */
      }
      BLI_assert(elem_offset || index == bm->totvert);
    }
    else {
      // printf("%s: skipping vert index calc!\n", __func__);
    }
  }

  if (htype & BM_EDGE) {
    if ((bm->elem_index_dirty & BM_EDGE) || (elem_offset && elem_offset[1])) {
      BMIter iter;
      BMElem *ele;

      int index = elem_offset ? elem_offset[1] : 0;
      BM_ITER_MESH (ele, &iter, bm, BM_EDGES_OF_MESH) {
        BM_elem_index_set(ele, index++); /* set_ok */
      }
      BLI_assert(elem_offset || index == bm->totedge);
    }
    else {
      // printf("%s: skipping edge index calc!\n", __func__);
    }
  }

  if (htype & (BM_FACE | BM_LOOP)) {
    if ((bm->elem_index_dirty & (BM_FACE | BM_LOOP)) ||
        (elem_offset && (elem_offset[2] || elem_offset[3])))
    {
      BMIter iter;
      BMElem *ele;

      const bool update_face = (htype & BM_FACE) && (bm->elem_index_dirty & BM_FACE);
      const bool update_loop = (htype & BM_LOOP) && (bm->elem_index_dirty & BM_LOOP);

      int index_loop = elem_offset ? elem_offset[2] : 0;
      int index = elem_offset ? elem_offset[3] : 0;

      BM_ITER_MESH (ele, &iter, bm, BM_FACES_OF_MESH) {
        if (update_face) {
          BM_elem_index_set(ele, index++); /* set_ok */
        }

        if (update_loop) {
          BMLoop *l_iter, *l_first;

          l_iter = l_first = BM_FACE_FIRST_LOOP((BMFace *)ele);
          do {
            BM_elem_index_set(l_iter, index_loop++); /* set_ok */
          } while ((l_iter = l_iter->next) != l_first);
        }
      }

      BLI_assert(elem_offset || !update_face || index == bm->totface);
      if (update_loop) {
        BLI_assert(elem_offset || !update_loop || index_loop == bm->totloop);
      }
    }
    else {
      // printf("%s: skipping face/loop index calc!\n", __func__);
    }
  }

finally:
  bm->elem_index_dirty &= ~htype;
  if (elem_offset) {
    if (htype & BM_VERT) {
      elem_offset[0] += bm->totvert;
      if (elem_offset[0] != bm->totvert) {
        bm->elem_index_dirty |= BM_VERT;
      }
    }
    if (htype & BM_EDGE) {
      elem_offset[1] += bm->totedge;
      if (elem_offset[1] != bm->totedge) {
        bm->elem_index_dirty |= BM_EDGE;
      }
    }
    if (htype & BM_LOOP) {
      elem_offset[2] += bm->totloop;
      if (elem_offset[2] != bm->totloop) {
        bm->elem_index_dirty |= BM_LOOP;
      }
    }
    if (htype & BM_FACE) {
      elem_offset[3] += bm->totface;
      if (elem_offset[3] != bm->totface) {
        bm->elem_index_dirty |= BM_FACE;
      }
    }
  }
}

void BM_mesh_elem_index_ensure(BMesh *bm, const char htype)
{
  BM_mesh_elem_index_ensure_ex(bm, htype, nullptr);
}

void BM_mesh_elem_index_validate(
    BMesh *bm, const char *location, const char *func, const char *msg_a, const char *msg_b)
{
  const char iter_types[3] = {BM_VERTS_OF_MESH, BM_EDGES_OF_MESH, BM_FACES_OF_MESH};

  const char flag_types[3] = {BM_VERT, BM_EDGE, BM_FACE};
  const char *type_names[3] = {"vert", "edge", "face"};

  BMIter iter;
  BMElem *ele;
  int i;
  bool is_any_error = false;

  for (i = 0; i < 3; i++) {
    const bool is_dirty = (flag_types[i] & bm->elem_index_dirty) != 0;
    int index = 0;
    bool is_error = false;
    int err_val = 0;
    int err_idx = 0;

    BM_ITER_MESH (ele, &iter, bm, iter_types[i]) {
      if (!is_dirty) {
        if (BM_elem_index_get(ele) != index) {
          err_val = BM_elem_index_get(ele);
          err_idx = index;
          is_error = true;
          break;
        }
      }
      index++;
    }

    if ((is_error == true) && (is_dirty == false)) {
      is_any_error = true;
      fprintf(stderr,
              "Invalid Index: at %s, %s, %s[%d] invalid index %d, '%s', '%s'\n",
              location,
              func,
              type_names[i],
              err_idx,
              err_val,
              msg_a,
              msg_b);
    }
    else if ((is_error == false) && (is_dirty == true)) {

#if 0 /* mostly annoying */

      /* dirty may have been incorrectly set */
      fprintf(stderr,
              "Invalid Dirty: at %s, %s (%s), dirty flag was set but all index values are "
              "correct, '%s', '%s'\n",
              location,
              func,
              type_names[i],
              msg_a,
              msg_b);
#endif
    }
  }

#if 0 /* mostly annoying, even in debug mode */
#  ifdef DEBUG
  if (is_any_error == 0) {
    fprintf(stderr, "Valid Index Success: at %s, %s, '%s', '%s'\n", location, func, msg_a, msg_b);
  }
#  endif
#endif
  (void)is_any_error; /* shut up the compiler */
}

/* debug check only - no need to optimize */
#ifndef NDEBUG
bool BM_mesh_elem_table_check(BMesh *bm)
{
  BMIter iter;
  BMElem *ele;
  int i;

  if (bm->vtable && ((bm->elem_table_dirty & BM_VERT) == 0)) {
    BM_ITER_MESH_INDEX (ele, &iter, bm, BM_VERTS_OF_MESH, i) {
      if (ele != (BMElem *)bm->vtable[i]) {
        return false;
      }
    }
  }

  if (bm->etable && ((bm->elem_table_dirty & BM_EDGE) == 0)) {
    BM_ITER_MESH_INDEX (ele, &iter, bm, BM_EDGES_OF_MESH, i) {
      if (ele != (BMElem *)bm->etable[i]) {
        return false;
      }
    }
  }

  if (bm->ftable && ((bm->elem_table_dirty & BM_FACE) == 0)) {
    BM_ITER_MESH_INDEX (ele, &iter, bm, BM_FACES_OF_MESH, i) {
      if (ele != (BMElem *)bm->ftable[i]) {
        return false;
      }
    }
  }

  return true;
}
#endif

void BM_mesh_elem_table_ensure(BMesh *bm, const char htype)
{
  /* assume if the array is non-null then its valid and no need to recalc */
  const char htype_needed =
      (((bm->vtable && ((bm->elem_table_dirty & BM_VERT) == 0)) ? 0 : BM_VERT) |
       ((bm->etable && ((bm->elem_table_dirty & BM_EDGE) == 0)) ? 0 : BM_EDGE) |
       ((bm->ftable && ((bm->elem_table_dirty & BM_FACE) == 0)) ? 0 : BM_FACE)) &
      htype;

  BLI_assert((htype & ~BM_ALL_NOLOOP) == 0);

  /* in debug mode double check we didn't need to recalculate */
  BLI_assert(BM_mesh_elem_table_check(bm) == true);

  if (htype_needed == 0) {
    goto finally;
  }

  if (htype_needed & BM_VERT) {
    if (bm->vtable && bm->totvert <= bm->vtable_tot && bm->totvert * 2 >= bm->vtable_tot) {
      /* pass (re-use the array) */
    }
    else {
      if (bm->vtable) {
        MEM_freeN(bm->vtable);
      }
      bm->vtable = static_cast<BMVert **>(
          MEM_mallocN(sizeof(void **) * bm->totvert, "bm->vtable"));
      bm->vtable_tot = bm->totvert;
    }
    BM_iter_as_array(bm, BM_VERTS_OF_MESH, nullptr, (void **)bm->vtable, bm->totvert);
  }
  if (htype_needed & BM_EDGE) {
    if (bm->etable && bm->totedge <= bm->etable_tot && bm->totedge * 2 >= bm->etable_tot) {
      /* pass (re-use the array) */
    }
    else {
      if (bm->etable) {
        MEM_freeN(bm->etable);
      }
      bm->etable = static_cast<BMEdge **>(
          MEM_mallocN(sizeof(void **) * bm->totedge, "bm->etable"));
      bm->etable_tot = bm->totedge;
    }
    BM_iter_as_array(bm, BM_EDGES_OF_MESH, nullptr, (void **)bm->etable, bm->totedge);
  }
  if (htype_needed & BM_FACE) {
    if (bm->ftable && bm->totface <= bm->ftable_tot && bm->totface * 2 >= bm->ftable_tot) {
      /* pass (re-use the array) */
    }
    else {
      if (bm->ftable) {
        MEM_freeN(bm->ftable);
      }
      bm->ftable = static_cast<BMFace **>(
          MEM_mallocN(sizeof(void **) * bm->totface, "bm->ftable"));
      bm->ftable_tot = bm->totface;
    }
    BM_iter_as_array(bm, BM_FACES_OF_MESH, nullptr, (void **)bm->ftable, bm->totface);
  }

finally:
  /* Only clear dirty flags when all the pointers and data are actually valid.
   * This prevents possible threading issues when dirty flag check failed but
   * data wasn't ready still.
   */
  bm->elem_table_dirty &= ~htype_needed;
}

void BM_mesh_elem_table_init(BMesh *bm, const char htype)
{
  BLI_assert((htype & ~BM_ALL_NOLOOP) == 0);

  /* force recalc */
  BM_mesh_elem_table_free(bm, BM_ALL_NOLOOP);
  BM_mesh_elem_table_ensure(bm, htype);
}

void BM_mesh_elem_table_free(BMesh *bm, const char htype)
{
  if (htype & BM_VERT) {
    MEM_SAFE_FREE(bm->vtable);
  }

  if (htype & BM_EDGE) {
    MEM_SAFE_FREE(bm->etable);
  }

  if (htype & BM_FACE) {
    MEM_SAFE_FREE(bm->ftable);
  }
}

BMVert *BM_vert_at_index_find(BMesh *bm, const int index)
{
  return static_cast<VertHive *>(bm->vhive)->at_index(index);
}

BMEdge *BM_edge_at_index_find(BMesh *bm, const int index)
{
  return static_cast<EdgeHive *>(bm->ehive)->at_index(index);
}

BMFace *BM_face_at_index_find(BMesh *bm, const int index)
{
  return static_cast<FaceHive *>(bm->fhive)->at_index(index);
}

BMLoop *BM_loop_at_index_find(BMesh *bm, const int index)
{
  BMIter iter;
  BMFace *f;
  int i = index;
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (i < f->len) {
      BMLoop *l_first, *l_iter;
      l_iter = l_first = BM_FACE_FIRST_LOOP(f);
      do {
        if (i == 0) {
          return l_iter;
        }
        i -= 1;
      } while ((l_iter = l_iter->next) != l_first);
    }
    i -= f->len;
  }
  return nullptr;
}

BMVert *BM_vert_at_index_find_or_table(BMesh *bm, const int index)
{
  if ((bm->elem_table_dirty & BM_VERT) == 0) {
    return (index < bm->totvert) ? bm->vtable[index] : nullptr;
  }
  return BM_vert_at_index_find(bm, index);
}

BMEdge *BM_edge_at_index_find_or_table(BMesh *bm, const int index)
{
  if ((bm->elem_table_dirty & BM_EDGE) == 0) {
    return (index < bm->totedge) ? bm->etable[index] : nullptr;
  }
  return BM_edge_at_index_find(bm, index);
}

BMFace *BM_face_at_index_find_or_table(BMesh *bm, const int index)
{
  if ((bm->elem_table_dirty & BM_FACE) == 0) {
    return (index < bm->totface) ? bm->ftable[index] : nullptr;
  }
  return BM_face_at_index_find(bm, index);
}

int BM_mesh_elem_count(BMesh *bm, const char htype)
{
  BLI_assert((htype & ~BM_ALL_NOLOOP) == 0);

  switch (htype) {
    case BM_VERT:
      return bm->totvert;
    case BM_EDGE:
      return bm->totedge;
    case BM_FACE:
      return bm->totface;
    default: {
      BLI_assert(0);
      return 0;
    }
  }
}

void BM_mesh_remap(BMesh *bm, const uint *vert_idx, const uint *edge_idx, const uint *face_idx)
{
  /* Mapping old to new pointers. */
  GHash *vptr_map = nullptr, *eptr_map = nullptr, *fptr_map = nullptr;
  BMIter iter, iterl;
  BMVert *ve;
  BMEdge *ed;
  BMFace *fa;
  BMLoop *lo;

  if (!(vert_idx || edge_idx || face_idx)) {
    return;
  }

  BM_mesh_elem_table_ensure(
      bm, (vert_idx ? BM_VERT : 0) | (edge_idx ? BM_EDGE : 0) | (face_idx ? BM_FACE : 0));

  /* Remap Verts */
  if (vert_idx) {
    BMVert **verts_pool, *verts_copy, **vep;
    int i, totvert = bm->totvert;
    const uint *new_idx;
    /* Special case: Python uses custom data layers to hold PyObject references.
     * These have to be kept in place, else the PyObjects we point to, won't point back to us. */
    const int cd_vert_pyptr = CustomData_get_offset(&bm->vdata, CD_BM_ELEM_PYPTR);

    /* Init the old-to-new vert pointers mapping */
    vptr_map = BLI_ghash_ptr_new_ex("BM_mesh_remap vert pointers mapping", bm->totvert);

    /* Make a copy of all vertices. */
    verts_pool = bm->vtable;
    verts_copy = static_cast<BMVert *>(
        MEM_mallocN(sizeof(BMVert) * totvert, "BM_mesh_remap verts copy"));
    void **pyptrs = (cd_vert_pyptr != -1) ?
                        static_cast<void **>(MEM_mallocN(sizeof(void *) * totvert, __func__)) :
                        nullptr;
    for (i = totvert, ve = verts_copy + totvert - 1, vep = verts_pool + totvert - 1; i--;
         ve--, vep--) {
      *ve = **vep;
      // printf("*vep: %p, verts_pool[%d]: %p\n", *vep, i, verts_pool[i]);
      if (cd_vert_pyptr != -1) {
        void **pyptr = static_cast<void **>(BM_ELEM_CD_GET_VOID_P(((BMElem *)ve), cd_vert_pyptr));
        pyptrs[i] = *pyptr;
      }
    }

    /* Copy back verts to their new place, and update old2new pointers mapping. */
    new_idx = vert_idx + totvert - 1;
    ve = verts_copy + totvert - 1;
    vep = verts_pool + totvert - 1; /* old, org pointer */
    for (i = totvert; i--; new_idx--, ve--, vep--) {
      BMVert *new_vep = verts_pool[*new_idx];
      *new_vep = *ve;
#if 0
      printf(
          "mapping vert from %d to %d (%p/%p to %p)\n", i, *new_idx, *vep, verts_pool[i], new_vep);
#endif
      BLI_ghash_insert(vptr_map, *vep, new_vep);
      if (cd_vert_pyptr != -1) {
        void **pyptr = static_cast<void **>(
            BM_ELEM_CD_GET_VOID_P(((BMElem *)new_vep), cd_vert_pyptr));
        *pyptr = pyptrs[*new_idx];
      }
    }
    bm->elem_index_dirty |= BM_VERT;
    bm->elem_table_dirty |= BM_VERT;

    MEM_freeN(verts_copy);
    if (pyptrs) {
      MEM_freeN(pyptrs);
    }
  }

  /* Remap Edges */
  if (edge_idx) {
    BMEdge **edges_pool, *edges_copy, **edp;
    int i, totedge = bm->totedge;
    const uint *new_idx;
    /* Special case: Python uses custom data layers to hold PyObject references.
     * These have to be kept in place, else the PyObjects we point to, won't point back to us. */
    const int cd_edge_pyptr = CustomData_get_offset(&bm->edata, CD_BM_ELEM_PYPTR);

    /* Init the old-to-new vert pointers mapping */
    eptr_map = BLI_ghash_ptr_new_ex("BM_mesh_remap edge pointers mapping", bm->totedge);

    /* Make a copy of all vertices. */
    edges_pool = bm->etable;
    edges_copy = static_cast<BMEdge *>(
        MEM_mallocN(sizeof(BMEdge) * totedge, "BM_mesh_remap edges copy"));
    void **pyptrs = (cd_edge_pyptr != -1) ?
                        static_cast<void **>(MEM_mallocN(sizeof(void *) * totedge, __func__)) :
                        nullptr;
    for (i = totedge, ed = edges_copy + totedge - 1, edp = edges_pool + totedge - 1; i--;
         ed--, edp--) {
      *ed = **edp;
      if (cd_edge_pyptr != -1) {
        void **pyptr = static_cast<void **>(BM_ELEM_CD_GET_VOID_P(((BMElem *)ed), cd_edge_pyptr));
        pyptrs[i] = *pyptr;
      }
    }

    /* Copy back verts to their new place, and update old2new pointers mapping. */
    new_idx = edge_idx + totedge - 1;
    ed = edges_copy + totedge - 1;
    edp = edges_pool + totedge - 1; /* old, org pointer */
    for (i = totedge; i--; new_idx--, ed--, edp--) {
      BMEdge *new_edp = edges_pool[*new_idx];
      *new_edp = *ed;
      BLI_ghash_insert(eptr_map, *edp, new_edp);
#if 0
      printf(
          "mapping edge from %d to %d (%p/%p to %p)\n", i, *new_idx, *edp, edges_pool[i], new_edp);
#endif
      if (cd_edge_pyptr != -1) {
        void **pyptr = static_cast<void **>(
            BM_ELEM_CD_GET_VOID_P(((BMElem *)new_edp), cd_edge_pyptr));
        *pyptr = pyptrs[*new_idx];
      }
    }
    bm->elem_index_dirty |= BM_EDGE;
    bm->elem_table_dirty |= BM_EDGE;

    MEM_freeN(edges_copy);
    if (pyptrs) {
      MEM_freeN(pyptrs);
    }
  }

  /* Remap Faces */
  if (face_idx) {
    BMFace **faces_pool, *faces_copy, **fap;
    int i, totface = bm->totface;
    const uint *new_idx;
    /* Special case: Python uses custom data layers to hold PyObject references.
     * These have to be kept in place, else the PyObjects we point to, won't point back to us. */
    const int cd_poly_pyptr = CustomData_get_offset(&bm->pdata, CD_BM_ELEM_PYPTR);

    /* Init the old-to-new vert pointers mapping */
    fptr_map = BLI_ghash_ptr_new_ex("BM_mesh_remap face pointers mapping", bm->totface);

    /* Make a copy of all vertices. */
    faces_pool = bm->ftable;
    faces_copy = static_cast<BMFace *>(
        MEM_mallocN(sizeof(BMFace) * totface, "BM_mesh_remap faces copy"));
    void **pyptrs = (cd_poly_pyptr != -1) ?
                        static_cast<void **>(MEM_mallocN(sizeof(void *) * totface, __func__)) :
                        nullptr;
    for (i = totface, fa = faces_copy + totface - 1, fap = faces_pool + totface - 1; i--;
         fa--, fap--) {
      *fa = **fap;
      if (cd_poly_pyptr != -1) {
        void **pyptr = static_cast<void **>(BM_ELEM_CD_GET_VOID_P(((BMElem *)fa), cd_poly_pyptr));
        pyptrs[i] = *pyptr;
      }
    }

    /* Copy back verts to their new place, and update old2new pointers mapping. */
    new_idx = face_idx + totface - 1;
    fa = faces_copy + totface - 1;
    fap = faces_pool + totface - 1; /* old, org pointer */
    for (i = totface; i--; new_idx--, fa--, fap--) {
      BMFace *new_fap = faces_pool[*new_idx];
      *new_fap = *fa;
      BLI_ghash_insert(fptr_map, *fap, new_fap);
      if (cd_poly_pyptr != -1) {
        void **pyptr = static_cast<void **>(
            BM_ELEM_CD_GET_VOID_P(((BMElem *)new_fap), cd_poly_pyptr));
        *pyptr = pyptrs[*new_idx];
      }
    }

    bm->elem_index_dirty |= BM_FACE | BM_LOOP;
    bm->elem_table_dirty |= BM_FACE;

    MEM_freeN(faces_copy);
    if (pyptrs) {
      MEM_freeN(pyptrs);
    }
  }

  /* And now, fix all vertices/edges/faces/loops pointers! */
  /* Verts' pointers, only edge pointers... */
  if (eptr_map) {
    BM_ITER_MESH (ve, &iter, bm, BM_VERTS_OF_MESH) {
      // printf("Vert e: %p -> %p\n", ve->e, BLI_ghash_lookup(eptr_map, ve->e));
      if (ve->e) {
        ve->e = static_cast<BMEdge *>(BLI_ghash_lookup(eptr_map, ve->e));
        BLI_assert(ve->e);
      }
    }
  }

  /* Edges' pointers, only vert pointers (as we don't mess with loops!),
   * and - ack! - edge pointers,
   * as we have to handle disk-links. */
  if (vptr_map || eptr_map) {
    BM_ITER_MESH (ed, &iter, bm, BM_EDGES_OF_MESH) {
      if (vptr_map) {
#if 0
        printf("Edge v1: %p -> %p\n", ed->v1, BLI_ghash_lookup(vptr_map, ed->v1));
        printf("Edge v2: %p -> %p\n", ed->v2, BLI_ghash_lookup(vptr_map, ed->v2));
#endif
        ed->v1 = static_cast<BMVert *>(BLI_ghash_lookup(vptr_map, ed->v1));
        ed->v2 = static_cast<BMVert *>(BLI_ghash_lookup(vptr_map, ed->v2));
        BLI_assert(ed->v1);
        BLI_assert(ed->v2);
      }
      if (eptr_map) {
#if 0
        printf("Edge v1_disk_link prev: %p -> %p\n",
               ed->v1_disk_link.prev,
               BLI_ghash_lookup(eptr_map, ed->v1_disk_link.prev));
        printf("Edge v1_disk_link next: %p -> %p\n",
               ed->v1_disk_link.next,
               BLI_ghash_lookup(eptr_map, ed->v1_disk_link.next));
        printf("Edge v2_disk_link prev: %p -> %p\n",
               ed->v2_disk_link.prev,
               BLI_ghash_lookup(eptr_map, ed->v2_disk_link.prev));
        printf("Edge v2_disk_link next: %p -> %p\n",
               ed->v2_disk_link.next,
               BLI_ghash_lookup(eptr_map, ed->v2_disk_link.next));
#endif
        ed->v1_disk_link.prev = static_cast<BMEdge *>(
            BLI_ghash_lookup(eptr_map, ed->v1_disk_link.prev));
        ed->v1_disk_link.next = static_cast<BMEdge *>(
            BLI_ghash_lookup(eptr_map, ed->v1_disk_link.next));
        ed->v2_disk_link.prev = static_cast<BMEdge *>(
            BLI_ghash_lookup(eptr_map, ed->v2_disk_link.prev));
        ed->v2_disk_link.next = static_cast<BMEdge *>(
            BLI_ghash_lookup(eptr_map, ed->v2_disk_link.next));
        BLI_assert(ed->v1_disk_link.prev);
        BLI_assert(ed->v1_disk_link.next);
        BLI_assert(ed->v2_disk_link.prev);
        BLI_assert(ed->v2_disk_link.next);
      }
    }
  }

  /* Faces' pointers (loops, in fact), always needed... */
  BM_ITER_MESH (fa, &iter, bm, BM_FACES_OF_MESH) {
    BM_ITER_ELEM (lo, &iterl, fa, BM_LOOPS_OF_FACE) {
      if (vptr_map) {
        // printf("Loop v: %p -> %p\n", lo->v, BLI_ghash_lookup(vptr_map, lo->v));
        lo->v = static_cast<BMVert *>(BLI_ghash_lookup(vptr_map, lo->v));
        BLI_assert(lo->v);
      }
      if (eptr_map) {
        // printf("Loop e: %p -> %p\n", lo->e, BLI_ghash_lookup(eptr_map, lo->e));
        lo->e = static_cast<BMEdge *>(BLI_ghash_lookup(eptr_map, lo->e));
        BLI_assert(lo->e);
      }
      if (fptr_map) {
        // printf("Loop f: %p -> %p\n", lo->f, BLI_ghash_lookup(fptr_map, lo->f));
        lo->f = static_cast<BMFace *>(BLI_ghash_lookup(fptr_map, lo->f));
        BLI_assert(lo->f);
      }
    }
  }

  /* Selection history */
  {
    BMEditSelection *ese;
    for (ese = static_cast<BMEditSelection *>(bm->selected.first); ese; ese = ese->next) {
      switch (ese->htype) {
        case BM_VERT:
          if (vptr_map) {
            ese->ele = static_cast<BMElem *>(BLI_ghash_lookup(vptr_map, ese->ele));
            BLI_assert(ese->ele);
          }
          break;
        case BM_EDGE:
          if (eptr_map) {
            ese->ele = static_cast<BMElem *>(BLI_ghash_lookup(eptr_map, ese->ele));
            BLI_assert(ese->ele);
          }
          break;
        case BM_FACE:
          if (fptr_map) {
            ese->ele = static_cast<BMElem *>(BLI_ghash_lookup(fptr_map, ese->ele));
            BLI_assert(ese->ele);
          }
          break;
      }
    }
  }

  if (fptr_map) {
    if (bm->act_face) {
      bm->act_face = static_cast<BMFace *>(BLI_ghash_lookup(fptr_map, bm->act_face));
      BLI_assert(bm->act_face);
    }
  }

  if (vptr_map) {
    BLI_ghash_free(vptr_map, nullptr, nullptr);
  }
  if (eptr_map) {
    BLI_ghash_free(eptr_map, nullptr, nullptr);
  }
  if (fptr_map) {
    BLI_ghash_free(fptr_map, nullptr, nullptr);
  }
}

void bm_alloc_toolflags_cdlayers(BMesh *bm, bool set_elems)
{
  CustomData *cdatas[3] = {&bm->vdata, &bm->edata, &bm->pdata};
  int iters[3] = {BM_VERTS_OF_MESH, BM_EDGES_OF_MESH, BM_FACES_OF_MESH};

  for (int i = 0; i < 3; i++) {
    CustomData *cdata = cdatas[i];
    int cd_tflags = CustomData_get_offset(cdata, CD_TOOLFLAGS);

    if (cd_tflags == -1) {
      if (set_elems) {
        BM_data_layer_add(bm, cdata, CD_TOOLFLAGS);
      }
      else {
        CustomData_add_layer(cdata, CD_TOOLFLAGS, CD_SET_DEFAULT, 0);
      }

      int idx = CustomData_get_layer_index(cdata, CD_TOOLFLAGS);

      cdata->layers[idx].flag |= CD_FLAG_TEMPORARY | CD_FLAG_NOCOPY | CD_FLAG_ELEM_NOCOPY;
      cd_tflags = cdata->layers[idx].offset;

      if (set_elems) {
        BMIter iter;
        BMElem *elem;

        BM_ITER_MESH (elem, &iter, bm, iters[i]) {
          MToolFlags *flags = (MToolFlags *)BM_ELEM_CD_GET_VOID_P(elem, cd_tflags);

          flags->flag = nullptr;
        }
      }
    }
  }
}

static void bm_alloc_toolflags(BMesh *bm)
{
  bm_alloc_toolflags_cdlayers(bm, true);

  CustomData *cdatas[3] = {&bm->vdata, &bm->edata, &bm->pdata};
  BLI_mempool *flagpools[3] = {bm->vtoolflagpool, bm->etoolflagpool, bm->ftoolflagpool};
  void *elempools[3] = {bm->vhive, bm->ehive, bm->fhive};

  for (int i = 0; i < 3; i++) {
    CustomData *cdata = cdatas[i];
    int cd_tflags = CustomData_get_offset(cdata, CD_TOOLFLAGS);
    int htype = i < 2 ? (1 << i) : BM_FACE;

    HiveIter iter;
    BM_hive_iternew(elempools[i], &iter, htype);
    BMElem *elem = (BMElem *)BM_hive_iterstep(&iter);

    for (; elem; elem = (BMElem *)BM_hive_iterstep(&iter)) {
      MToolFlags *flags = (MToolFlags *)BM_ELEM_CD_GET_VOID_P(elem, cd_tflags);

      flags->flag = (short *)BLI_mempool_calloc(flagpools[i]);
    }
  }
}

void BM_mesh_toolflags_set(BMesh *bm, bool use_toolflags)
{
  if (bm->use_toolflags == use_toolflags) {
    return;
  }

  if (use_toolflags == false) {
    BLI_mempool_destroy(bm->vtoolflagpool);
    BLI_mempool_destroy(bm->etoolflagpool);
    BLI_mempool_destroy(bm->ftoolflagpool);

    bm->vtoolflagpool = nullptr;
    bm->etoolflagpool = nullptr;
    bm->ftoolflagpool = nullptr;

    BM_data_layer_free(bm, &bm->vdata, CD_TOOLFLAGS);
    BM_data_layer_free(bm, &bm->edata, CD_TOOLFLAGS);
    BM_data_layer_free(bm, &bm->pdata, CD_TOOLFLAGS);
  }
  else {
    bm_alloc_toolflags_cdlayers(bm, true);
  }

  bm->use_toolflags = use_toolflags;

  if (use_toolflags) {
    BM_mesh_elem_toolflags_ensure(bm);
  }
}

/* -------------------------------------------------------------------- */
/** \name BMesh Coordinate Access
 * \{ */

void BM_mesh_vert_coords_get(BMesh *bm, float (*vert_coords)[3])
{
  BMIter iter;
  BMVert *v;
  int i;
  BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, i) {
    copy_v3_v3(vert_coords[i], v->co);
  }
}

float (*BM_mesh_vert_coords_alloc(BMesh *bm, int *r_vert_len))[3]
{
  float(*vert_coords)[3] = static_cast<float(*)[3]>(
      MEM_mallocN(bm->totvert * sizeof(*vert_coords), __func__));
  BM_mesh_vert_coords_get(bm, vert_coords);
  *r_vert_len = bm->totvert;
  return vert_coords;
}

void BM_mesh_vert_coords_apply(BMesh *bm, const float (*vert_coords)[3])
{
  BMIter iter;
  BMVert *v;
  int i;
  BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, i) {
    copy_v3_v3(v->co, vert_coords[i]);
  }
}

void BM_mesh_vert_coords_apply_with_mat4(BMesh *bm,
                                         const float (*vert_coords)[3],
                                         const float mat[4][4])
{
  BMIter iter;
  BMVert *v;
  int i;
  BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, i) {
    mul_v3_m4v3(v->co, mat, vert_coords[i]);
  }
}

/** \} */
