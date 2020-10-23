#ifndef _TRIMESH_H
#define _TRIMESH_H

// optimized thread-safe triangle mesh library with topological info

// XXX get from user settings as appropriate
#define TRIMESH_THREADS 4

#include "BLI_assert.h"
#include "BLI_threads.h"
#include "BLI_threadsafe_mempool.h"

#include "DNA_customdata_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_compiler_compat.h"
#include "BLI_hashmap.h"

#include <stdint.h>

struct Main;
struct TMVert;
struct TMEdge;
struct TMTri;
struct Mesh;

enum { TM_VERTEX = 1, TM_EDGE = 2, TM_LOOP = 4, TM_TRI = 8 };

enum {
  TRIMESH_SELECT = 1 << 1,
  TM_ELEM_HIDDEN = 1 << 2,
  TRIMESH_SMOOTH = 1 << 3,
  TRIMESH_SEAM = 1 << 4,
  TRIMESH_SHARP = 1 << 5,
  TRIMESH_EDGEDRAW = 1 << 6,
  TRIMESH_VT_ITERFLAG = 1 << 9,
  TRIMESH_TEMP_TAG = 1 << 10,
  TRIMESH_TEMP_TAG2 = 1 << 10
};

BLI_INLINE uint8_t BLI_trimesh_vert_flag_to_mflag(int16_t f)
{
  uint8_t r = 0;

  r = f & TRIMESH_SELECT;

  if (f & TM_ELEM_HIDDEN) {
    r |= ME_HIDE;
  }

  return r;
}

BLI_INLINE uint8_t BLI_trimesh_edge_flag_to_mflag(int16_t f)
{
  uint8_t r = f & TRIMESH_SELECT;

  if (f & TM_ELEM_HIDDEN) {
    r |= ME_HIDE;
  }

  if (f & TRIMESH_SEAM) {
    r |= ME_SEAM;
  }
  if (f & TRIMESH_EDGEDRAW) {
    r |= ME_EDGEDRAW;
  }
  if (f & TRIMESH_SHARP) {
    r |= ME_SHARP;
  }

  return r;
}

BLI_INLINE uint8_t BLI_trimesh_tri_flag_to_mflag(int16_t f)
{
  uint8_t r = 0;

  if (f & TRIMESH_SELECT) {
    r |= ME_FACE_SEL;
  }
  if (f & TRIMESH_SMOOTH) {
    r |= ME_SMOOTH;
  }
  if (f & TM_ELEM_HIDDEN) {
    r |= ME_HIDE;
  }

  return r;
}

#define WITH_TRIMESH_CUSTOMDATA

#ifdef WITH_TRIMESH_CUSTOMDATA
#  define OPTELEM_HEAD(type) \
    int16_t threadtag; \
    int16_t flag; \
    int index; \
    void *customdata;
#else
#  define OPTELEM_HEAD(type) \
    int16_t threadtag; \
    flag; \
    int index;
#endif

typedef struct TMElement {
  OPTELEM_HEAD(TMElement)
} TMElement;

typedef struct optmesh_simplelist {
  void **items;
  int _size, length;
  bool is_pool_allocd;
} optmesh_simplelist;

typedef struct TMVert {
  OPTELEM_HEAD(TMVert)
  float co[3];
  float no[3];
  float oco[3];
  short ono[3], pad;

  optmesh_simplelist edges;
} TMVert;

typedef struct TMEdge {
  OPTELEM_HEAD(TMEdge)
  TMVert *v1, *v2;
  optmesh_simplelist tris;
} TMEdge;

#ifdef WITH_TRIMESH_CUSTOMDATA
typedef struct TMLoopData {
  OPTELEM_HEAD(TMLoopData)
} TMLoopData;
#endif

typedef struct TMFace {
  OPTELEM_HEAD(TMFace)

  TMVert *v1, *v2, *v3;
  TMEdge *e1, *e2, *e3;

#ifdef WITH_TRIMESH_CUSTOMDATA
  TMLoopData *l1, *l2, *l3;
#endif

  float no[3];
  int mat_nr;
} TMFace;

typedef struct TMTriIsland {
  TMFace **tris;
  int tottri;
  TMVert **verts;
  int totvert;
  int tag;
} TMTriIsland;

struct BLI_ThreadSafePool;

#ifdef WITH_TRIMESH_CUSTOMDATA
#  define MAX_TRIMESH_POOLS 6
#else
#  define MAX_TRIMESH_POOLS 5
#endif

typedef struct TM_ElemTable {
  void **table;
  int size, used;
} TM_ElemTable;

typedef struct TM_TriMesh {
  struct BLI_ThreadSafePool *pools[MAX_TRIMESH_POOLS];

  int totvert, tottri, totedge;
  int maxthread;
  int elem_table_dirty, elem_index_dirty;

  int shapenr;

  TM_ElemTable vtable, etable, ttable;

  SpinLock global_lock;

#ifdef WITH_TRIMESH_CUSTOMDATA
  CustomData vdata, edata, ldata, tdata;
#endif
} TM_TriMesh;

typedef struct TM_TriMeshIter {
  int pool;
  ThreadSafePoolIter iter;
} TM_TriMeshIter;

#define TRIMESH_NEED_TAG -1
#define TRIMESH_BOUNDARY -2
#define TRIMESH_BOUNDARY_TEMP -3
#define TRIMESH_TAG_CLEAR -4

#define TRIMESH_VERT 1
#define TRIMESH_EDGE 2
#define TRIMESH_TRI 4

void TM_index_update(TM_TriMesh *tm);

enum {  // same as TM_VERTEX/EDGE/LOOP/TRI
  TM_VERTS_OF_MESH = 1,
  TM_EDGES_OF_MESH = 2,
  TM_LOOPS_OF_MESH = 4,
  TM_TRIS_OF_MESH = 8
};

TM_TriMesh *TMesh_new(int maxthread);
void TMesh_free(TM_TriMesh *tm);
void TM_vert_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter);
void TM_edge_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter);
void TM_loop_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter);
void TM_tri_iternew(TM_TriMesh *tm, TM_TriMeshIter *iter);
void TM_add(TM_TriMesh *tm,
            float *vertCos,
            float *vertNos,
            int totvert,
            int *triIndices,
            int tottri,
            int threadnr,
            bool skipcd);

BLI_INLINE void *TM_iterstep(TM_TriMeshIter *iter)
{
  return BLI_safepool_iterstep(&iter->iter);
}

BLI_INLINE void TM_iterate(TM_TriMesh *tm, TM_TriMeshIter *iter, int type)
{
  switch (type) {
    case TM_VERTS_OF_MESH:
      TM_vert_iternew(tm, iter);
      break;
    case TM_EDGES_OF_MESH:
      TM_edge_iternew(tm, iter);
      break;
    case TM_LOOPS_OF_MESH:
      TM_loop_iternew(tm, iter);
      break;
    case TM_TRIS_OF_MESH:
      TM_tri_iternew(tm, iter);
      break;
    default:
      BLI_assert(0);
      memset(iter, 0, sizeof(*iter));
      break;
  }
}

#define TM_ITER_MESH(elem, iter, tm, type) \
  TM_iterate(tm, iter, type); \
  elem = TM_iterstep(iter); \
  for (; elem; elem = TM_iterstep(iter))

TMVert *TM_make_vert(TM_TriMesh *tm, float co[3], float no[3], bool skipcd);

BLI_INLINE TMEdge *TM_edge_exists(TMVert *v1, TMVert *v2)
{
  for (int i = 0; i < v1->edges.length; i++) {
    TMEdge *e = v1->edges.items[i];

    if (e->v1 == v2 || e->v2 == v2) {
      return e;
    }
  }

  return NULL;
}

#define TM_edge_is_wire(e) ((e)->tris.length == 0)

// only creates an edge if one doesn't already exist between v1 and v2
TMEdge *TM_get_edge(TM_TriMesh *tm, TMVert *v1, TMVert *v2, bool skipcd);
TMFace *TM_make_tri(TM_TriMesh *tm, TMVert *v1, TMVert *v2, TMVert *v3, bool skipcd);

void TM_kill_edge(TM_TriMesh *tm, TMEdge *e, bool kill_verts);
void TM_kill_vert(TM_TriMesh *tm, TMVert *v);
void TM_kill_tri(TM_TriMesh *tm, TMFace *tri, bool kill_edges, bool kill_verts);

// primary interface to run threaded jobs

typedef void (*OptTriMeshJob)(
    TM_TriMesh *tm, void **elements, int totelem, int threadnr, void *userdata);
void TM_foreach_tris(
    TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

// yes, the input is an array of triangles, even though the jobs are fed vertices.
void TM_foreach_verts(
    TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

// low-level api functions used by BLI_trimesh_foreach_XXX
// if tottris is -1 then all triangles will be tagged
void TM_thread_tag(TM_TriMesh *tm, TMFace **tris, int tottri);
void TM_clear_threadtags(TM_TriMesh *tm);
void TM_tag_thread_boundaries(TM_TriMesh *tm, TMFace **tris, int tottri);
void TM_tag_thread_boundaries_once(TM_TriMesh *tm, TMFace **tris, int tottri);

// called after BLI_trimesh_thread_tag
// last island is always boundary triangles
void TM_build_islands(
    TM_TriMesh *tm, TMFace **tris, int tottri, TMTriIsland **r_islands, int *r_totisland);
void TM_free_islands(TMTriIsland *islands, int totisland, bool free_islands);

bool TM_elem_is_dead(void *elem);

#define TM_ELEM_CD_SET_INT(ele, offset, f) (*((int *)((char *)(ele)->customdata + (offset))) = (f))
#define TM_ELEM_CD_GET_INT(ele, offset) (*((int *)((char *)(ele)->customdata + (offset))))
#define TM_ELEM_CD_SET_FLOAT(ele, offset, f) \
  (*((float *)((char *)(ele)->customdata + (offset))) = (f))
#define TM_ELEM_CD_GET_FLOAT(ele, offset) (*((float *)((char *)(ele)->customdata + (offset))))

#define TM_ELEM_CD_GET_FLOAT_AS_UCHAR(ele, offset) \
  (BLI_assert(offset != -1), (uchar)(TM_ELEM_CD_GET_FLOAT(ele, offset) * 255.0f))

#define TM_GET_TRI_VERT(tri, n) ((&(tri)->v1)[n])
#define TM_GET_TRI_EDGE(tri, n) ((&(tri)->e1)[n])
#define TM_GET_TRI_LOOP(tri, n) ((&(tri)->l1)[n])

BLI_INLINE TMLoopData *TM_GET_TRI_LOOP_EDGE(TMFace *t, TMEdge *e)
{
  if (e == t->e1) {
    return t->l1;
  }
  if (e == t->e2) {
    return t->l2;
  }

  return t->l3;
}

BLI_INLINE TMLoopData *TM_GET_TRI_LOOP_VERTEX(TMFace *t, TMVert *v)
{
  if (v == t->v1) {
    return t->l1;
  }
  else if (v == t->v2) {
    return t->l2;
  }
  else {
    return t->l3;
  }
}

TMVert *TM_split_edge(TM_TriMesh *tm, TMEdge *e, float fac, bool skipcd);
void TM_collapse_edge(TM_TriMesh *tm, TMEdge *e);

typedef struct TriMeshLog TriMeshLog;
TriMeshLog *TM_log_new(TM_TriMesh *tm, int cd_vert_mask_index);
void TM_log_free(TriMeshLog *log);
int TM_log_vert_add(TriMeshLog *log, TMVert *v, const int cd_mask_offset, bool skipcd);
int BLI_trimesh_log_tri(TriMeshLog *log, TMFace *tri, bool skipcd);
int BLI_trimesh_log_vert_kill(TriMeshLog *log, TMVert *v);
int BLI_trimesh_log_tri_kill(TriMeshLog *log, TMFace *tri, int kill_verts, int kill_edges);
int BLI_trimesh_log_vert_state(TriMeshLog *log, TMVert *v);
void TM_log_original_vert_data(TriMeshLog *tlog,
                               TMVert *v,
                               const float **r_co,
                               const short **r_no);
float *TM_log_original_vert_co(TriMeshLog *tlog, TMVert *v);
float TM_log_original_mask(TriMeshLog *tlog, TMVert *v);
void TM_data_layer_add(TM_TriMesh *tm, struct CustomData *data, int type);
void TM_mesh_cd_flag_apply(TM_TriMesh *bm, const char cd_flag);
void TM_data_layer_add_named(TM_TriMesh *bm, CustomData *data, int type, const char *name);

#define TM_elem_flag_enable(elem, f) ((elem)->flag |= (f))
#define TM_elem_flag_disable(elem, f) ((elem)->flag &= ~(f))
#define TM_elem_flag_test(elem, f) ((elem)->flag & (f))
#define TM_elem_flag_set(elem, f, v) ((v) ? ((elem)->flag |= (f)) : ((elem)->flag &= ~(f)))

void TM_mesh_normals_update(TM_TriMesh *tm);

BLI_INLINE void TM_calc_tri_normal(TMFace *tri)
{
  normal_tri_v3(tri->no, tri->v1->co, tri->v2->co, tri->v3->co);
}

BLI_INLINE void TM_calc_vert_normal(TMVert *v, bool recalc_tri_normals)
{
  int tot = 0;

  zero_v3(v->no);

  for (int i = 0; i < v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];

    for (int j = 0; j < e->tris.length; j++) {
      TMFace *tri = e->tris.items[j];

      if (recalc_tri_normals) {
        normal_tri_v3(tri->no, tri->v1->co, tri->v2->co, tri->v3->co);
      }

      add_v3_v3(v->no, tri->no);
      tot++;
    }
  }

  if (tot == 0) {
    v->no[2] = 1.0f;
  }
  else {
    normalize_v3(v->no);
  }
}
BLI_INLINE float TM_edge_calc_length_squared(TMEdge *e)
{
  float f = 0.0;

  f += (e->v2->co[0] - e->v1->co[0]) * (e->v2->co[0] - e->v1->co[0]);
  f += (e->v2->co[1] - e->v1->co[1]) * (e->v2->co[1] - e->v1->co[1]);
  f += (e->v2->co[2] - e->v1->co[2]) * (e->v2->co[2] - e->v1->co[2]);

  return f;
}

BLI_INLINE TMEdge *TM_nextEdgeInTri(TMFace *t, TMEdge *e)
{
  if (e == t->e1)
    return t->e2;
  if (e == t->e2)
    return t->e3;
  return t->e1;
}

BLI_INLINE TMEdge *TM_prevEdgeInTri(TMFace *t, TMEdge *e)
{
  if (e == t->e3)
    return t->e2;
  if (e == t->e2)
    return t->e1;
  return t->e3;
}

BLI_INLINE TMVert *TM_nextVertInTri(TMFace *t, TMVert *v)
{
  if (v == t->v1)
    return t->v2;
  if (v == t->v2)
    return t->v3;
  return t->v1;
}

BLI_INLINE TMVert *TM_prevVertInTri(TMFace *t, TMVert *v)
{
  if (v == t->v3)
    return t->v2;
  if (v == t->v2)
    return t->v1;
  return t->v3;
}

BLI_INLINE int TM_edgeTriIndex(TMEdge *e, TMFace *t)
{
  if (!e || !t)
    return -1;

  for (int i = 0; i < e->tris.length; i++) {
    TMFace *t2 = e->tris.items[i];

    if (t2 == t) {
      return i;
    }
  }

  return -1;
}

BLI_INLINE TMVert *TM_other_vert(TMEdge *e, TMVert *v)
{
  return v == e->v1 ? e->v2 : e->v1;
}

BLI_INLINE TMFace *TM_nextTriInEdge(TMEdge *e, TMFace *t)
{
  int i = TM_edgeTriIndex(e, t);

  if (i < 0) {
    fprintf(stderr, "Error in %s\n", __func__);
    // return NULL;
    return t;
  }

  i = (i + 1) % e->tris.length;

  return e->tris.items[i];
}

BLI_INLINE TMFace *TM_prevTriInEdge(TMEdge *e, TMFace *t)
{
  int i = TM_edgeTriIndex(e, t);

  if (i < 0) {
    fprintf(stderr, "Error in %s\n", __func__);
    // return NULL;
    return t;
  }

  i = (i - 1 + e->tris.length) % e->tris.length;

  return e->tris.items[i];
}

BLI_INLINE void TM_orderEdgeInTri(TMFace *t, TMEdge *e, TMVert **r_v1, TMVert **r_v2)
{
  bool swap = false;

  if (e->v1 == t->v1) {
    swap = e->v2 == t->v3;
  }
  else if (e->v1 == t->v2) {
    swap = e->v2 == t->v1;
  }
  else if (e->v1 == t->v3) {
    swap = e->v2 == t->v2;
  }

  if (swap) {
    *r_v1 = e->v2;
    *r_v2 = e->v1;
  }
  else {
    *r_v1 = e->v1;
    *r_v2 = e->v2;
  }
}

BLI_INLINE TMVert *TM_getAdjVert(TMEdge *e, TMFace *t)
{
  if (e == t->e1) {
    return t->v3;
  }

  if (e == t->e2) {
    return t->v1;
  }

  if (e == t->e3) {
    return t->v2;
  }

  fprintf(stderr, "Error in %s\n", __func__);
  return t->v1;  // NULL?
}

BLI_INLINE int TM_vert_in_tri(TMFace *t, TMVert *v)
{
  if (v == t->v1)
    return true;
  if (v == t->v2)
    return true;
  if (v == t->v3)
    return true;

  return false;
}

BLI_INLINE int TM_edge_in_tri(TMFace *t, TMEdge *e)
{
  if (e == t->e1)
    return true;
  if (e == t->e2)
    return true;
  if (e == t->e3)
    return true;

  return false;
}

BLI_INLINE TMFace *TM_tri_exists(TMVert *v1, TMVert *v2, TMVert *v3)
{
  TMEdge *e = TM_edge_exists(v1, v2);

  if (!e) {
    return NULL;
  }

  for (int i = 0; i < e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    if (TM_vert_in_tri(tri, v3)) {
      return tri;
    }
  }

  return NULL;
}

#define TM_ITER_VERT_TRIS(v, tname) \
  for (int _i = 0; _i < v->edges.length; _i++) { \
    TMEdge *_e = v->edges.items[_i]; \
    for (int _j = 0; _j < _e->tris.length; _j++) { \
      TMFace *tname = _e->tris.items[_j]; \
      { \
        bool ok = true; \
        for (int _k = 0; ok && _k <= _i; _k++) { \
          TMEdge *e = v->edges.items[_k]; \
          for (int _l = 0; _l < e->tris.length; _l++) { \
            TMFace *_t = e->tris.items[_l]; \
            if (_t == tname && (_k != _i || _l != _j)) { \
              ok = false; \
              break; \
            } \
          } \
        } \
        if (!ok) { \
          continue; \
        } \
      }

#define TM_ITER_VERT_TRIS_END \
  } \
  }

#define TM_ITER_VERT_TRIEDGES(v, tname, ename) \
  for (int _i = 0; _i < v->edges.length; _i++) { \
    TMEdge *ename = v->edges.items[_i]; \
    for (int _j = 0; _j < ename->tris.length; _j++) { \
      TMFace *tname = ename->tris.items[_j]; \
      { \
        bool ok = true; \
        for (int _k = 0; ok && _k <= _i; _k++) { \
          TMEdge *e = v->edges.items[_k]; \
          for (int _l = 0; _l < e->tris.length; _l++) { \
            TMFace *_t = e->tris.items[_l]; \
            if (_t == tname && (_k != _i || _l != _j)) { \
              ok = false; \
              break; \
            } \
          } \
        } \
        if (!ok) { \
          continue; \
        } \
      }

#define TM_ITER_VERT_TRIEDGES_END \
  } \
  }

// returns true if any faces exist around v
BLI_INLINE int TM_vert_face_check(TMVert *v)
{
  for (int i = 0; i < v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];

    if (e->tris.length > 0) {
      return true;
    }
  }

  return false;
}

#define TM_elem_flag_test_bool(elem, f) (!!((elem)->flag & (f)))

struct TMeshToMeshParams {
  /** Update object hook indices & vertex parents. */
  uint calc_object_remap : 1;
  /**
   * This re-assigns shape-key indices. Only do if the BMesh will have continued use
   * to update the mesh & shape key in the future.
   * In the case the BMesh is freed immediately, this can be left false.
   *
   * This is needed when flushing changes from edit-mode into object mode,
   * so a second flush or edit-mode exit doesn't run with indices
   * that have become invalid from updating the shape-key, see T71865.
   */
  uint update_shapekey_indices : 1;
  struct CustomData_MeshMasks cd_mask_extra;
};
void TM_mesh_bm_to_me(struct Main *bmain,
                      TM_TriMesh *tm,
                      struct Mesh *me,
                      const struct TMeshToMeshParams *params);

void TM_mesh_elem_table_ensure(TM_TriMesh *tm, int typemask);
void TM_mesh_elem_index_ensure(TM_TriMesh *tm, int typemask);

BLI_INLINE TMVert *TM_vert_at_index(TM_TriMesh *bm, const int index)
{
  BLI_assert((index >= 0) && (index < bm->totvert));
  BLI_assert((bm->elem_table_dirty & TM_VERTEX) == 0);

  return (TMVert *)bm->vtable.table[index];
}

TM_ElemTable *TM_getElemTable(TM_TriMesh *tm, int type);

BLI_INLINE TMEdge *TM_edge_at_index(TM_TriMesh *bm, const int index)
{
  BLI_assert((index >= 0) && (index < bm->totedge));
  BLI_assert((bm->elem_table_dirty & TM_EDGE) == 0);

  return (TMEdge *)bm->etable.table[index];
}

BLI_INLINE TMFace *TM_tri_at_index(TM_TriMesh *bm, const int index)
{
  BLI_assert((index >= 0) && (index < bm->tottri));
  BLI_assert((bm->elem_table_dirty & TM_TRI) == 0);

  return (TMFace *)bm->ttable.table[index];
}

#if 0  // defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#  define TM_ELEM_CD_GET_VOID_P(ele, offset) \
    (BLI_assert(offset != -1), \
     _Generic(ele, \
              GENERIC_TYPE_ANY(POINTER_OFFSET((ele)->customdata, offset), \
                               _BM_GENERIC_TYPE_ELEM_NONCONST), \
              GENERIC_TYPE_ANY((const void *)POINTER_OFFSET((ele)->customdata, offset), \
                               _BM_GENERIC_TYPE_ELEM_CONST)))
#else
#  define TM_ELEM_CD_GET_VOID_P(ele, offset) \
    (BLI_assert(offset != -1), (void *)((char *)(ele)->customdata + (offset)))
#endif

BLI_INLINE int TM_vert_is_boundary(TMVert *v)
{
  if (v->edges.length == 0) {
    return false;  // bmesh returns false in this situation
  }
  for (int i = 0; i < v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];
    if (e->tris.length < 2) {
      return true;
    }
  }

  return false;
}

BLI_INLINE int TM_vert_face_count(TMVert *v)
{
  int i = 0;

  TM_ITER_VERT_TRIS (v, tri) {
    i++;
  }
  TM_ITER_VERT_TRIS_END

  return i;
}

BLI_INLINE bool TM_edge_is_boundary(TMEdge *e)
{
  return e->tris.length < 2;
}

BLI_INLINE int TM_vert_face_count_at_most(TMVert *v, int n)
{
  int i = 0;

  TM_ITER_VERT_TRIS (v, tri) {
    if (i >= n) {
      return n;
    }
    i++;
  }
  TM_ITER_VERT_TRIS_END

  return i;
}

BLI_INLINE void TM_getOtherVerts(TMFace *t, TMVert *v, TMVert **vs)
{
  vs[0] = TM_prevVertInTri(t, v);
  vs[1] = TM_nextVertInTri(t, v);
}

struct TriMeshFromMeshParams {
  uint calc_face_normal : 1;
  /* add a vertex CD_SHAPE_KEYINDEX layer */
  uint add_key_index : 1;
  /* set vertex coordinates from the shapekey */
  uint use_shapekey : 1;
  /* define the active shape key (index + 1) */
  int active_shapekey;
  struct CustomData_MeshMasks cd_mask_extra;
};

struct Mesh;
void TM_mesh_tm_from_me(TM_TriMesh *bm,
                        const struct Mesh *me,
                        const struct TriMeshFromMeshParams *params);

BLI_INLINE int TM_mesh_elem_count(TM_TriMesh *tm, int type)
{
  switch (type) {
    case TM_VERTEX:
      return tm->totvert;
    case TM_EDGE:
      return tm->totedge;
    case TM_TRI:
      return tm->tottri;
  }

  return 0;
}

#endif /* _TRIMESH_H */
