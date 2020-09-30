//optimized thread-safe triangle mesh library with topological info

#include "BLI_threads.h"
#include "BLI_threadsafe_mempool.h"
#include "DNA_customdata_types.h"
#include "DNA_meshdata_types.h"

#include <stdint.h>

struct OptTriVert;
struct OptTriEdge;
struct OptTri;

enum {
  TRIMESH_SELECT = 1<<1,
  TRIMESH_HIDE = 1<<1,
  TRIMESH_SMOOTH = 1<<3,
  TRIMESH_VT_ITERFLAG = 1<<9
};

static uint8_t BLI_trimesh_vert_flag_to_mflag(int16_t f)  {
  uint8_t r = 0;

  r = f & TRIMESH_SELECT;
  if (f & TRIMESH_HIDE) {
    r |= ME_HIDE;
  }

  return r;
}


static uint8_t BLI_trimesh_edge_flag_to_mflag(int16_t f)  {
  uint8_t r = 0;

  //XXX
  r = f & 255;

  return r;
}


static uint8_t BLI_trimesh_tri_flag_to_mflag(int16_t f)  {
  uint8_t r = 0;

  if (f & TRIMESH_SELECT) {
    r |= ME_FACE_SEL;
  }
  if (f & TRIMESH_SMOOTH) {
    r |= ME_SMOOTH;
  }
  if (f & TRIMESH_HIDE) {
    r |= ME_HIDE;
  }

  return r;
}


#define WITH_TRIMESH_CUSTOMDATA

#ifdef WITH_TRIMESH_CUSTOMDATA
#define OPTELEM_HEAD(type) \
struct type *next, *prev; \
int16_t threadtag; \
int16_t flag; \
int index; \
void *customdata;
#else
#define OPTELEM_HEAD(type) \
struct type *next, *prev; \
int16_t threadtag; flag;\
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
#define MAX_TRIMESH_POOLS 6
#else
#define MAX_TRIMESH_POOLS 5
#endif

typedef struct TM_TriMesh {
 struct BLI_ThreadSafePool* pools[MAX_TRIMESH_POOLS];

  int totvert, tottri, totedge;
  int maxthread;
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
#define TRIMESH_TRI  4

void TM_index_update(TM_TriMesh *tm);

TM_TriMesh* TMesh_new(int maxthread);
void TMesh_free(TM_TriMesh *tm);
void TM_vert_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter);
void TM_edge_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter);
void TM_tri_iternew(TM_TriMesh *tm, TM_TriMeshIter* iter);
void *TM_iterstep(TM_TriMeshIter* iter);
void TM_add(TM_TriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr, bool skipcd);

TMVert *TM_make_vert(TM_TriMesh *tm, float co[3], float no[3], int threadnr, bool skipcd);

static TMEdge *TM_edge_exists(TMVert *v1, TMVert *v2) {
  for (int i=0; i<v1->edges.length; i++) {
    TMEdge *e = v1->edges.items[i];

    if (e->v1 == v2 || e->v2 == v2) {
      return e;
    }
  }

  return NULL;
}

#define TM_edge_is_wire(e) ((e)->tris.length == 0)

//only creates an edge if one doesn't already exist between v1 and v2
TMEdge *TM_get_edge(TM_TriMesh *tm, TMVert *v1, TMVert *v2, int threadnr, bool skipcd);
TMFace *TM_make_tri(TM_TriMesh *tm, TMVert *v1, TMVert *v2, TMVert *v3, int threadnr, bool skipcd);

void TM_kill_edge(TM_TriMesh *tm, TMEdge *e, int threadnr, bool kill_verts);
void TM_kill_vert(TM_TriMesh *tm, TMVert *v, int threadnr);
void TM_kill_tri(TM_TriMesh *tm, TMFace *tri, int threadnr, bool kill_edges, bool kill_verts);

//primary interface to run threaded jobs

typedef void (*OptTriMeshJob)(TM_TriMesh *tm, void **elements, int totelem, int threadnr, void *userdata);
void TM_foreach_tris(TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

//yes, the input is an array of triangles, even though the jobs are fed vertices.
void TM_foreach_verts(TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

//low-level api functions used by BLI_trimesh_foreach_XXX
//if tottris is -1 then all triangles will be tagged
void TM_thread_tag(TM_TriMesh *tm, TMFace** tris, int tottri);
void TM_clear_threadtags(TM_TriMesh *tm);
void TM_tag_thread_boundaries(TM_TriMesh *tm, TMFace **tris, int tottri);
void TM_tag_thread_boundaries_once(TM_TriMesh *tm, TMFace **tris, int tottri);

//called after BLI_trimesh_thread_tag
//last island is always boundary triangles
void TM_build_islands(TM_TriMesh *tm, TMFace **tris, int tottri, TMTriIsland** r_islands, int *r_totisland);

#define TRIMESH_ELEM_CD_SET_INT(ele, offset, f) (*((int *)((char *)(ele)->customdata + (offset))) = (f))
#define TRIMESH_ELEM_CD_GET_INT(ele, offset) (*((int *)((char *)(ele)->customdata + (offset))))
#define TRIMESH_ELEM_CD_SET_FLOAT(ele, offset, f) (*((float *)((char *)(ele)->customdata + (offset))) = (f))
#define TRIMESH_ELEM_CD_GET_FLOAT(ele, offset) (*((float *)((char *)(ele)->customdata + (offset))))

#define TRIMESH_ELEM_CD_GET_FLOAT_AS_UCHAR(ele, offset) \
  (assert(offset != -1), (uchar)(TRIMESH_ELEM_CD_GET_FLOAT(ele, offset) * 255.0f))

#define TRIMESH_GET_TRI_VERT(tri, n) ((&(tri)->v1)[n])
#define TRIMESH_GET_TRI_EDGE(tri, n) ((&(tri)->e1)[n])
#define TRIMESH_GET_TRI_LOOP(tri, n) ((&(tri)->l1)[n])

TMVert *TM_split_edge(TM_TriMesh *tm, TMEdge *e, int threadnr, float fac, bool skipcd);
void TM_collapse_edge(TM_TriMesh *tm, TMEdge *e, int threadnr);

typedef struct TriMeshLog TriMeshLog;
TriMeshLog *TM_log_new();
void TM_log_free(TriMeshLog *log);
int TM_log_vert_add(TriMeshLog *log, TMVert *v, const int cd_mask_offset, bool skipcd);
int BLI_trimesh_log_tri(TriMeshLog *log, TMFace *tri, bool skipcd);
int BLI_trimesh_log_vert_kill(TriMeshLog *log, TMVert *v);
int BLI_trimesh_log_tri_kill(TriMeshLog *log, TMFace *tri);
int BLI_trimesh_log_vert_state(TriMeshLog *log, TMVert *v);

#define TRIMESH_elem_flag_enable(elem, f) ((elem)->flag |= (f))
#define TRIMESH_elem_flag_disable(elem, f) ((elem)->flag &= ~(f))
#define TRIMESH_elem_flag_test(elem, f) ((elem)->flag & (f))
#define TRIMESH_elem_flag_set(elem, f, v) ((v) ? ((elem)->flag |= (f)) : ((elem)->flag &= ~(f)))

#define TRIMESH_TEMP_TAG (1<<4)

static void TM_calc_tri_normal(TMFace *tri, int threadnr) {
  normal_tri_v3(tri->no, tri->v1->co, tri->v2->co, tri->v3->co);
}

static void TM_calc_vert_normal(TMVert *v, int threadnr, bool recalc_tri_normals) {
  int tot = 0;

  zero_v3(v->no);

  for (int i=0; i<v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];

    for (int j=0; j<e->tris.length; j++) {
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
  } else {
    normalize_v3(v->no);
  }
}
static float TM_edge_calc_length_squared(TMEdge *e) {
  float f = 0.0;

  f += (e->v2->co[0] - e->v1->co[0])*(e->v2->co[0] - e->v1->co[0]);
  f += (e->v2->co[1] - e->v1->co[1])*(e->v2->co[1] - e->v1->co[1]);
  f += (e->v2->co[2] - e->v1->co[2])*(e->v2->co[2] - e->v1->co[2]);

  return f;
}

static TMEdge *TM_nextEdgeInTri(TMFace *t, TMEdge *e) {
  if (e == t->e1)
    return t->e2;
  if (e == t->e2)
    return t->e3;
  return t->e1;
}

static TMEdge *TM_prevEdgeInTri(TMFace *t, TMEdge *e) {
  if (e == t->e3)
    return t->e2;
  if (e == t->e2)
    return t->e1;
  return t->e3;
}

static TMVert *TM_nextVertInTri(TMFace *t, TMVert *v) {
  if (v == t->v1)
    return t->v2;
  if (v == t->v2)
    return t->v3;
  return t->v1;
}

static TMVert *TM_prevVertInTri(TMFace *t, TMVert *v) {
  if (v == t->v3)
    return t->v2;
  if (v == t->v2)
    return t->v1;
  return t->v3;
}

static int TM_edgeTriIndex(TMEdge *e, TMFace *t) {
  for (int i=0; i<e->tris.length; i++) {
    TMFace *t2 = e->tris.items[i];

    if (t2 == t) {
      return i;
    }
  }

  return -1;
}

static TMFace *TM_nextTriInEdge(TMEdge *e, TMFace *t) {
  int i = TM_edgeTriIndex(e, t);

  if (i < 0) {
    fprintf(stderr, "Evil!\n");
    //return NULL;
    return t;
  }

  i = (i + 1) % e->tris.length;

  return e->tris.items[i];
}

static TMFace *TM_prevTriInEdge(TMEdge *e, TMFace *t) {
  int i = TM_edgeTriIndex(e, t);

  if (i < 0) {
    fprintf(stderr, "Evil!\n");
    //return NULL;
    return t;
  }

  i = (i - 1 + e->tris.length) % e->tris.length;

  return e->tris.items[i];
}

static TMVert *TM_getAdjVert(TMEdge *e, TMFace *t) {
  if (e == t->e1) {
    return t->v3;
  }

  if (e == t->e2) {
    return t->v1;
  }

  if (e == t->e3) {
    return t->v2;
  }

  fprintf(stderr, "evil! %s:%i\n", __FILE__, __LINE__);
  return t->v1; //NULL?
}

static int TM_vert_in_tri(TMFace *t, TMVert *v) {
  if (v == t->v1) return true;
  if (v == t->v2) return true;
  if (v == t->v3) return true;

  return false;
}

static int TM_edge_in_tri(TMFace *t, TMEdge *e) {
  if (e == t->e1) return true;
  if (e == t->e2) return true;
  if (e == t->e3) return true;

  return false;
}

static TMFace *TM_tri_exists(TMVert *v1, TMVert *v2, TMVert *v3) {
  TMEdge *e = TM_edge_exists(v1, v2);

  if (!e) {
    return NULL;
  }

  for (int i=0; i<e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    if (TM_vert_in_tri(tri, v3)) {
      return tri;
    }
  }

  return NULL;
}

#define TM_ITER_VERT_TRIS(v, tname)\
for (int _i=0; _i<v->edges.length; _i++) {\
  TMEdge *e = v->edges.items[_i];\
  for (int _j=0; _j<v->edges.length; _j++) {\
    TMFace *t = e->tris.items[_j];\
    t->flag &= ~TRIMESH_VT_ITERFLAG;\
  }\
}\
for (int _i=0; _i<v->edges.length; _i++) {\
  TMEdge *e = v->edges.items[_i];\
  for (int _j=0; _j<e->tris.length; _j++) {\
    TMFace *tname = e->tris.items[_j];\
    if (tname->flag & TRIMESH_VT_ITERFLAG) {\
      continue;\
    }\
    tname->flag |= TRIMESH_VT_ITERFLAG;

#define TM_ITER_VERT_TRIS_END }}

#define TM_ITER_VERT_TRIEDGES(v, tname, ename)\
for (int _i=0; _i<v->edges.length; _i++) {\
  TMEdge *e = v->edges.items[_i];\
  for (int _j=0; _j<v->edges.length; _j++) {\
    TMFace *t = e->tris.items[_j];\
    t->flag &= ~TRIMESH_VT_ITERFLAG;\
  }\
}\
for (int _i=0; _i<v->edges.length; _i++) {\
  TMEdge *ename = v->edges.items[_i];\
  for (int _j=0; _j<ename->tris.length; _j++) {\
    TMFace *tname = ename->tris.items[_j];\
    if (tname->flag & TRIMESH_VT_ITERFLAG) {\
      continue;\
    }\
    tname->flag |= TRIMESH_VT_ITERFLAG;

#define TM_ITER_VERT_TRIEDGES_END }}


//returns true if any faces exist around v
static int TM_vert_face_check(TMVert *v) {
  for (int i=0; i<v->edges.length; i++) {
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

