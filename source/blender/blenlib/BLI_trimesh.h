//optimized thread-safe triangle mesh library with topological info

#include "BLI_threads.h"
#include "BLI_threadsafe_mempool.h"
#include "DNA_customdata_types.h"


struct OptTriVert;
struct OptTriEdge;
struct OptTri;

#define WITH_TRIMESH_CUSTOMDATA

#ifdef WITH_TRIMESH_CUSTOMDATA
#define OPTELEM_HEAD(type) \
struct type *next, *prev; \
int threadtag; \
int index; \
void *customdata;
#else
#define OPTELEM_HEAD(type) \
struct type *next, *prev; \
int threadtag;\
int index;
#endif

typedef struct OptTriElem {
  OPTELEM_HEAD(OptTriElem)
} OptTriElem;

typedef struct optmesh_simplelist {
  void **items;
  int _size, length;
  bool is_pool_allocd;
} optmesh_simplelist;

typedef struct OptTriVert {
  OPTELEM_HEAD(OptTriVert)
  float co[3];
  float no[3];

  optmesh_simplelist edges;
} OptTriVert;

typedef struct OptTriEdge {
  OPTELEM_HEAD(OptTriEdge)
  OptTriVert *v1, *v2;
  optmesh_simplelist tris;
} OptTriEdge;

#ifdef WITH_TRIMESH_CUSTOMDATA
typedef struct OptTriLoop {
  OPTELEM_HEAD(OptTriLoop)
} OptTriLoop;
#endif

typedef struct OptTri {
  OPTELEM_HEAD(OptTri)

  OptTriVert *v1, *v2, *v3;
  OptTriEdge *e1, *e2, *e3;

 #ifdef WITH_TRIMESH_CUSTOMDATA
  OptTriLoop *l1, *l2, *l3;
 #endif

  float no[3];
} OptTri;

typedef struct OptTriIsland {
  OptTri **tris;
  int tottri;
  OptTriVert **verts;
  int totvert;
  int tag;
} OptTriIsland;

struct BLI_ThreadSafePool;

#ifdef WITH_TRIMESH_CUSTOMDATA
#define MAX_TRIMESH_POOLS 6
#else
#define MAX_TRIMESH_POOLS 5
#endif

typedef struct OptTriMesh {
 struct BLI_ThreadSafePool* pools[MAX_TRIMESH_POOLS];

  int totvert, tottri, totedge;
  int maxthread;
  SpinLock global_lock;

#ifdef WITH_TRIMESH_CUSTOMDATA
  CustomData vdata, edata, ldata, tdata;
#endif
} OptTriMesh;

typedef struct OptTriMeshIter {
  int pool;
  ThreadSafePoolIter iter;
} OptTriMeshIter;

#define TRIMESH_NEED_TAG -1
#define TRIMESH_BOUNDARY -2
#define TRIMESH_BOUNDARY_TEMP -3
#define TRIMESH_TAG_CLEAR -4

#define TRIMESH_VERT 1
#define TRIMESH_EDGE 2
#define TRIMESH_TRI  4

void BLI_trimesh_index_update(OptTriMesh *tm);

OptTriMesh* BLI_trimesh_new(int maxthread);
void BLI_trimesh_free(OptTriMesh *tm);
void BLI_trimesh_vert_iternew(OptTriMesh *tm, OptTriMeshIter* iter);
void BLI_trimesh_edge_iternew(OptTriMesh *tm, OptTriMeshIter* iter);
void BLI_trimesh_tri_iternew(OptTriMesh *tm, OptTriMeshIter* iter);
void BLI_trimesh_iterstep(OptTriMeshIter* iter);
void BLI_trimesh_add(OptTriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr, bool skipcd);

OptTriVert *BLI_trimesh_make_vert(OptTriMesh *tm, float co[3], float no[3], int threadnr, bool skipcd);

//only creates an edge if one doesn't already exist between v1 and v2
OptTriEdge *BLI_trimesh_get_edge(OptTriMesh *tm, OptTriVert *v1, OptTriVert *v2, int threadnr, bool skipcd);
OptTri *BLI_trimesh_make_tri(OptTriMesh *tm, OptTriVert *v1, OptTriVert *v2, OptTriVert *v3, int threadnr, bool skipcd);

void BLI_trimesh_kill_edge(OptTriMesh *tm, OptTriEdge *e, int threadnr, bool kill_verts);
void BLI_trimesh_kill_vert(OptTriMesh *tm, OptTriVert *v, int threadnr);
void BLI_trimesh_kill_tri(OptTriMesh *tm, OptTri *tri, int threadnr, bool kill_edges, bool kill_verts);

//primary interface to run threaded jobs
typedef void (*OptTriMeshJob)(OptTriMesh *tm, void **elements, int totelem, int threadnr, void *userdata);
void BLI_trimesh_foreach_tris(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

//yes, the input is an array of triangles, even though the jobs are fed vertices.
void BLI_trimesh_foreach_verts(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);

//low-level api functions used by BLI_trimesh_foreach_XXX
//if tottris is -1 then all triangles will be tagged
void BLI_trimesh_thread_tag(OptTriMesh *tm, OptTri** tris, int tottri);
void BLI_trimesh_clear_threadtags(OptTriMesh *tm);
void BLI_trimesh_tag_thread_boundaries(OptTriMesh *tm, OptTri **tris, int tottri);
void BLI_trimesh_tag_thread_boundaries_once(OptTriMesh *tm, OptTri **tris, int tottri);

//called after BLI_trimesh_thread_tag
//last island is always boundary triangles
void BLI_trimesh_build_islands(OptTriMesh *tm, OptTri **tris, int tottri, OptTriIsland** r_islands, int *r_totisland);

#define TRIMESH_ELEM_CD_SET_INT(ele, offset, f) (*((int *)((char *)(ele)->customdata + (offset))) = (f))
#define TRIMESH_ELEM_CD_GET_INT(ele, offset, f) (*((int *)((char *)(ele)->customdata + (offset))))
#define TRIMESH_ELEM_CD_SET_FLOAT(ele, offset, f) (*((float *)((char *)(ele)->customdata + (offset))) = (f))
#define TRIMESH_ELEM_CD_GET_FLOAT(ele, offset, f) (*((float *)((char *)(ele)->customdata + (offset))))

#define TRIMESH_GET_TRI_VERT(tri, n) ((&(tri)->v1)[n])
#define TRIMESH_GET_TRI_EDGE(tri, n) ((&(tri)->e1)[n])
#define TRIMESH_GET_TRI_LOOP(tri, n) ((&(tri)->l1)[n])

OptTriVert *BLI_trimesh_split_edge(OptTriMesh *tm, OptTriEdge *e, int threadnr, float fac, bool skipcd);
void BLI_trimesh_collapse_edge(OptTriMesh *tm, OptTriEdge *e, int threadnr);

typedef struct TriMeshLog TriMeshLog;
TriMeshLog *BLI_trimesh_log_new();
void BLI_trimesh_log_free(TriMeshLog *log);
int BLI_trimesh_log_vert_add(TriMeshLog *log, OptTriVert *v, const int cd_mask_offset, bool skipcd);
int BLI_trimesh_log_tri(TriMeshLog *log, OptTri *tri, int cd_vert_mask_offset, bool skipcd);
int BLI_trimesh_log_vert_kill(TriMeshLog *log, OptTriVert *v);
int BLI_trimesh_log_tri_kill(TriMeshLog *log, OptTri *tri);
