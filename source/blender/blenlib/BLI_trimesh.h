//optimized thread-safe triangle mesh library with topological info

#include "BLI_threads.h"
#include "BLI_threadsafe_mempool.h"
#include "DNA_customdata_types.h"

#include <stdint.h>

struct OptTriVert;
struct OptTriEdge;
struct OptTri;

#define TRIMESH_SELECT    (1<<1)
#define TRIMESH_HIDE      (1<<2)
#define TRIMESH_VT_ITERFLAG  (1<<7)

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
} TMFace;

typedef struct OptTriIsland {
  TMFace **tris;
  int tottri;
  TMVert **verts;
  int totvert;
  int tag;
} OptTriIsland;

struct BLI_ThreadSafePool;

#ifdef WITH_TRIMESH_CUSTOMDATA
#define MAX_TRIMESH_POOLS 6
#else
#define MAX_TRIMESH_POOLS 5
#endif

typedef struct BLI_TriMesh {
 struct BLI_ThreadSafePool* pools[MAX_TRIMESH_POOLS];

  int totvert, tottri, totedge;
  int maxthread;
  SpinLock global_lock;

#ifdef WITH_TRIMESH_CUSTOMDATA
  CustomData vdata, edata, ldata, tdata;
#endif
} BLI_TriMesh;

typedef struct BLI_TriMeshIter {
  int pool;
  ThreadSafePoolIter iter;
} BLI_TriMeshIter;

#define TRIMESH_NEED_TAG -1
#define TRIMESH_BOUNDARY -2
#define TRIMESH_BOUNDARY_TEMP -3
#define TRIMESH_TAG_CLEAR -4

#define TRIMESH_VERT 1
#define TRIMESH_EDGE 2
#define TRIMESH_TRI  4

void BLI_trimesh_index_update(BLI_TriMesh *tm);

BLI_TriMesh* BLI_trimesh_new(int maxthread);
void BLI_trimesh_free(BLI_TriMesh *tm);
void BLI_trimesh_vert_iternew(BLI_TriMesh *tm, BLI_TriMeshIter* iter);
void BLI_trimesh_edge_iternew(BLI_TriMesh *tm, BLI_TriMeshIter* iter);
void BLI_trimesh_tri_iternew(BLI_TriMesh *tm, BLI_TriMeshIter* iter);
void *BLI_trimesh_iterstep(BLI_TriMeshIter* iter);
void BLI_trimesh_add(BLI_TriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr, bool skipcd);

TMVert *BLI_trimesh_make_vert(BLI_TriMesh *tm, float co[3], float no[3], int threadnr, bool skipcd);

static TMEdge *BLI_trimesh_edge_exists(TMVert *v1, TMVert *v2) {
  for (int i=0; i<v1->edges.length; i++) {
    TMEdge *e = v1->edges.items[i];

    if (e->v1 == v2 || e->v2 == v2) {
      return e;
    }
  }

  return NULL;
}

#define BLI_trimesh_edge_is_wire(e) ((e)->tris.length == 0)

//only creates an edge if one doesn't already exist between v1 and v2
TMEdge *BLI_trimesh_get_edge(BLI_TriMesh *tm, TMVert *v1, TMVert *v2, int threadnr, bool skipcd);
TMFace *BLI_trimesh_make_tri(BLI_TriMesh *tm, TMVert *v1, TMVert *v2, TMVert *v3, int threadnr, bool skipcd);

void BLI_trimesh_kill_edge(BLI_TriMesh *tm, TMEdge *e, int threadnr, bool kill_verts);
void BLI_trimesh_kill_vert(BLI_TriMesh *tm, TMVert *v, int threadnr);
void BLI_trimesh_kill_tri(BLI_TriMesh *tm, TMFace *tri, int threadnr, bool kill_edges, bool kill_verts);

//primary interface to run threaded jobs
typedef void (*OptTriMeshJob)(BLI_TriMesh *tm, void **elements, int totelem, int threadnr, void *userdata);
void BLI_trimesh_foreach_tris(BLI_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

//yes, the input is an array of triangles, even though the jobs are fed vertices.
void BLI_trimesh_foreach_verts(BLI_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

//low-level api functions used by BLI_trimesh_foreach_XXX
//if tottris is -1 then all triangles will be tagged
void BLI_trimesh_thread_tag(BLI_TriMesh *tm, TMFace** tris, int tottri);
void BLI_trimesh_clear_threadtags(BLI_TriMesh *tm);
void BLI_trimesh_tag_thread_boundaries(BLI_TriMesh *tm, TMFace **tris, int tottri);
void BLI_trimesh_tag_thread_boundaries_once(BLI_TriMesh *tm, TMFace **tris, int tottri);

//called after BLI_trimesh_thread_tag
//last island is always boundary triangles
void BLI_trimesh_build_islands(BLI_TriMesh *tm, TMFace **tris, int tottri, OptTriIsland** r_islands, int *r_totisland);

#define TRIMESH_ELEM_CD_SET_INT(ele, offset, f) (*((int *)((char *)(ele)->customdata + (offset))) = (f))
#define TRIMESH_ELEM_CD_GET_INT(ele, offset, f) (*((int *)((char *)(ele)->customdata + (offset))))
#define TRIMESH_ELEM_CD_SET_FLOAT(ele, offset, f) (*((float *)((char *)(ele)->customdata + (offset))) = (f))
#define TRIMESH_ELEM_CD_GET_FLOAT(ele, offset, f) (*((float *)((char *)(ele)->customdata + (offset))))

#define TRIMESH_GET_TRI_VERT(tri, n) ((&(tri)->v1)[n])
#define TRIMESH_GET_TRI_EDGE(tri, n) ((&(tri)->e1)[n])
#define TRIMESH_GET_TRI_LOOP(tri, n) ((&(tri)->l1)[n])

TMVert *BLI_trimesh_split_edge(BLI_TriMesh *tm, TMEdge *e, int threadnr, float fac, bool skipcd);
void BLI_trimesh_collapse_edge(BLI_TriMesh *tm, TMEdge *e, int threadnr);

typedef struct TriMeshLog TriMeshLog;
TriMeshLog *BLI_trimesh_log_new();
void BLI_trimesh_log_free(TriMeshLog *log);
int BLI_trimesh_log_vert_add(TriMeshLog *log, TMVert *v, const int cd_mask_offset, bool skipcd);
int BLI_trimesh_log_tri(TriMeshLog *log, TMFace *tri, bool skipcd);
int BLI_trimesh_log_vert_kill(TriMeshLog *log, TMVert *v);
int BLI_trimesh_log_tri_kill(TriMeshLog *log, TMFace *tri);
int BLI_trimesh_log_vert_state(TriMeshLog *log, TMVert *v);

#define TRIMESH_elem_flag_enable(elem, f) ((elem)->flag |= (f))
#define TRIMESH_elem_flag_disable(elem, f) ((elem)->flag &= ~(f))
#define TRIMESH_elem_flag_test(elem, f) ((elem)->flag & (f))

#define TRIMESH_TEMP_TAG (1<<4)

static void BLI_trimesh_calc_tri_normal(TMFace *tri, int threadnr) {
  normal_tri_v3(tri->no, tri->v1->co, tri->v2->co, tri->v3->co);
}

static void BLI_trimesh_calc_vert_normal(TMVert *v, int threadnr, bool recalc_tri_normals) {
  int tot;

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
static float TRIMESH_edge_calc_length_squared(TMEdge *e) {
  float f = 0.0;

  f += (e->v2->co[0] - e->v1->co[0])*(e->v2->co[0] - e->v1->co[0]);
  f += (e->v2->co[1] - e->v1->co[1])*(e->v2->co[1] - e->v1->co[1]);
  f += (e->v2->co[2] - e->v1->co[2])*(e->v2->co[2] - e->v1->co[2]);

  return f;
}

static TMEdge *BLI_trimesh_nextEdgeInTri(TMFace *t, TMEdge *e) {
  if (e == t->e1)
    return t->e2;
  if (e == t->e2)
    return t->e3;
  return t->e1;
}

static TMEdge *BLI_trimesh_prevEdgeInTri(TMFace *t, TMVert *e) {
  if (e == t->e3)
    return t->e2;
  if (e == t->e2)
    return t->e1;
  return t->e3;
}

static TMVert *BLI_trimesh_nextVertInTri(TMFace *t, TMVert *v) {
  if (v == t->v1)
    return t->v2;
  if (v == t->v2)
    return t->v3;
  return t->v1;
}

static TMVert *BLI_trimesh_prevVertInTri(TMFace *t, TMVert *v) {
  if (v == t->v3)
    return t->v2;
  if (v == t->v2)
    return t->v1;
  return t->v3;
}

static int BLI_trimesh_edgeTriIndex(TMEdge *e, TMFace *t) {
  for (int i=0; i<e->tris.length; i++) {
    TMFace *t2 = e->tris.items[i];

    if (t2 == t) {
      return i;
    }
  }

  return -1;
}

static TMFace *BLI_trimesh_nextTriInEdge(TMEdge *e, TMFace *t) {
  int i = BLI_trimesh_edgeTriIndex(e, t);

  if (i < 0) {
    fprintf(stderr, "Evil!\n");
    //return NULL;
    return t;
  }

  i = (i + 1) % e->tris.length;

  return e->tris.items[i];
}

static TMFace *BLI_trimesh_prevTriInEdge(TMEdge *e, TMFace *t) {
  int i = BLI_trimesh_edgeTriIndex(e, t);

  if (i < 0) {
    fprintf(stderr, "Evil!\n");
    //return NULL;
    return t;
  }

  i = (i - 1 + e->tris.length) % e->tris.length;

  return e->tris.items[i];
}

static TMVert *BLI_trimesh_getAdjVert(TMEdge *e, TMFace *t) {
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

static int BLI_trimesh_vert_in_tri(TMFace *t, TMVert *v) {
  if (v == t->v1) return true;
  if (v == t->v2) return true;
  if (v == t->v3) return true;

  return false;
}

static int BLI_trimesh_edge_in_tri(TMFace *t, TMEdge *e) {
  if (e == t->e1) return true;
  if (e == t->e2) return true;
  if (e == t->e3) return true;

  return false;
}

static TMFace *BLI_trimesh_tri_exists(TMVert *v1, TMVert *v2, TMVert *v3) {
  TMEdge *e = BLI_trimesh_edge_exists(v1, v2);

  if (!e) {
    return NULL;
  }

  for (int i=0; i<e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    if (BLI_trimesh_vert_in_tri(tri, v3)) {
      return tri;
    }
  }

  return NULL;
}

#define TRIMESH_ITER_VERT_TRIS(v, tname)\
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

#define TRIMESH_ITER_VERT_TRIS_END }}

#define TRIMESH_ITER_VERT_TRIEDGES(v, tname, ename)\
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

#define TRIMESH_ITER_VERT_TRIEDGES_END }}


//returns true if any faces exist around v
static int BLI_trimesh_vert_face_check(TMVert *v) {
  for (int i=0; i<v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];

    if (e->tris.length > 0) {
      return true;
    }
  }

  return false;
}

#define TRIMESH_elem_flag_test_bool(elem, f) (!!((elem)->flag & (f)))
