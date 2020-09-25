//optimized thread-safe triangle mesh library with topological info

#include "BLI_threads.h"
#include "BLI_threadsafe_mempool.h"


struct OptTriVert;
struct OptTriEdge;
struct OptTri;

#define OPTELEM_HEAD(type) struct type *next, *prev; int threadtag;

typedef struct OptElem {
  OPTELEM_HEAD(OptElem)
} OptElem;

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

typedef struct OptTri {
  OPTELEM_HEAD(OptTri)

  OptTriVert *v1, *v2, *v3;
  OptTriEdge *e1, *e2, *e3;

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

#define MAX_TRIMESH_POOLS 5

typedef struct OptTriMesh {
 struct BLI_ThreadSafePool* pools[MAX_TRIMESH_POOLS];

  int totvert, tottri, totedge;
  int maxthread;
  SpinLock global_lock;
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

OptTriMesh* BLI_trimesh_new(int maxthread);
void BLI_trimesh_free(OptTriMesh *tm);
void BLI_trimesh_vert_iternew(OptTriMesh *tm, OptTriMeshIter* iter);
void BLI_trimesh_edge_iternew(OptTriMesh *tm, OptTriMeshIter* iter);
void BLI_trimesh_tri_iternew(OptTriMesh *tm, OptTriMeshIter* iter);
void BLI_trimesh_iterstep(OptTriMeshIter* iter);
void BLI_trimesh_add(OptTriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr);

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

//called after BLI_trimesh_thread_tag
//last island is always boundary triangles
void BLI_trimesh_build_islands(OptTriMesh *tm, OptTri **tris, int tottri, OptTriIsland** r_islands, int *r_totisland);
