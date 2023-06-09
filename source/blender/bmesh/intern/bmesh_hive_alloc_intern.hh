#include "BLI_compiler_compat.h"
#include "BLI_hive_alloc.hh"

#include "DNA_customdata_types.h"

#include "bmesh.h"
#include "bmesh_hive_alloc.h"
#include "bmesh_structure.h"

//#define AUTO_MOVE_CD_HIVES

struct NullCallbacks {
  static void move_elem(int *eold, int *enew, int totsize, int)
  {
    memcpy(static_cast<void *>(enew), static_cast<void *>(eold), totsize);
  }
};

struct CDHiveSizeof {
  static size_t size(int totsize)
  {
    return size_t(totsize);
  }
};
/* TODO: move somewhere in blenkernel. */
using CustomDataHive = blender::HiveAllocator<int, NullCallbacks, int, 4098, CDHiveSizeof>;

struct VertCallbacks {
  static void move_elem(BMVert *vold, BMVert *vnew, BMesh *bm, int hive)
  {
    bm->elem_index_dirty |= BM_VERT;
    bm->elem_table_dirty |= BM_VERT;

#ifdef AUTO_MOVE_CD_HIVES
    if (bm->vdata.hive) {
      CustomDataHive *cd_hive = static_cast<CustomDataHive *>(bm->vdata.hive);
      cd_hive->ensure_hives(hive + 1);
      vnew->head.data = cd_hive->move(static_cast<int *>(vnew->head.data), hive);
    }
#endif

    if (vold->e) {
      BMEdge *e = vold->e;
      BMEdge *next;
      do {
        if (e->v1 == vold) {
          e->v1 = vnew;
          next = e->v1_disk_link.next;
        }
        else {
          e->v2 = vnew;
          next = e->v2_disk_link.next;
        }

        if (e->l) {
          BMLoop *l = e->l;

          do {
            BMLoop *l2 = l;
            do {
              if (l2->v == vold) {
                l2->v = vnew;
              }
            } while ((l2 = l2->next) != l);
          } while ((l = l->radial_next) != e->l);
        }

        e = next;
      } while (next != vold->e);
    }

    if (bm->vhive_move_cb) {
      bm->vhive_move_cb(vold, vnew, bm->vhive_userdata, hive);
    }
  }
};

struct EdgeCallbacks {
  static void move_elem(BMEdge *eold, BMEdge *enew, BMesh *bm, int hive)
  {
    bm->elem_index_dirty |= BM_EDGE;
    bm->elem_table_dirty |= BM_EDGE;

#if 0  // def AUTO_MOVE_CD_HIVES
    if (bm->edata.hive) {
      CustomDataHive *cd_hive = static_cast<CustomDataHive *>(bm->edata.hive);
      cd_hive->ensure_hives(hive + 1);
      enew->head.data = cd_hive->move(static_cast<int *>(enew->head.data), hive);
    }
#endif

    bmesh_disk_edge_remove(eold, eold->v1);
    bmesh_disk_edge_remove(eold, eold->v2);

    bmesh_disk_edge_append(enew, eold->v1);
    bmesh_disk_edge_append(enew, eold->v2);

    if (eold->l) {
      BMLoop *l = eold->l;
      do {
        l->e = enew;
      } while ((l = l->radial_next) != eold->l);
    }

    if (bm->ehive_move_cb) {
      bm->ehive_move_cb(eold, enew, bm->ehive_userdata, hive);
    }
  }
};

struct LoopCallbacks {
  static void move_elem(BMLoop *lold, BMLoop *lnew, BMesh *bm, int hive)
  {
#if 0  // def AUTO_MOVE_CD_HIVES
    if (bm->ldata.hive) {
      CustomDataHive *cd_hive = static_cast<CustomDataHive *>(bm->ldata.hive);
      cd_hive->ensure_hives(hive + 1);
      lnew->head.data = cd_hive->move(static_cast<int *>(lnew->head.data), hive);
    }
#endif

    if (lold->prev) {
      lold->prev->next = lnew;
    }
    if (lold->next) {
      lold->next->prev = lnew;
    }

    if (lold->radial_next) {
      lold->radial_next->radial_prev = lnew;
    }

    if (lold->radial_prev) {
      lold->radial_prev->radial_next = lnew;
    }

    if (lold->e && lold == lold->e->l) {
      lold->e->l = lnew;
    }

    if (lold->f && lold == lold->f->l_first) {
      lold->f->l_first = lnew;
    }

    if (bm->lhive_move_cb) {
      bm->lhive_move_cb(lold, lnew, bm->lhive_userdata, hive);
    }
  }
};

struct FaceCallbacks {
  static void move_elem(BMFace *fold, BMFace *fnew, BMesh *bm, int hive)
  {
    bm->elem_index_dirty |= BM_FACE;
    bm->elem_table_dirty |= BM_FACE;

#if 0  // def AUTO_MOVE_CD_HIVES
    if (bm->pdata.hive) {
      CustomDataHive *cd_hive = static_cast<CustomDataHive *>(bm->pdata.hive);
      cd_hive->ensure_hives(hive + 1);
      fnew->head.data = cd_hive->move(static_cast<int *>(fnew->head.data), hive);
    }
#endif

    BMLoop *l = fnew->l_first;
    do {
      l->f = fnew;
    } while ((l = l->next) != fnew->l_first);

    if (bm->fhive_move_cb) {
      bm->fhive_move_cb(fold, fnew, bm->fhive_userdata, hive);
    }
  }
};

using VertHive = blender::HiveAllocator<BMVert, VertCallbacks, BMesh *>;
using EdgeHive = blender::HiveAllocator<BMEdge, EdgeCallbacks, BMesh *>;
using LoopHive = blender::HiveAllocator<BMLoop, LoopCallbacks, BMesh *>;
using FaceHive = blender::HiveAllocator<BMFace, FaceCallbacks, BMesh *>;
