#include "BLI_hive_alloc.hh"

#include "bmesh.h"
#include "bmesh_structure.h"

#include "bmesh_hive_alloc.h"
#include "bmesh_hive_alloc_intern.h"
#include "bmesh_hive_alloc_intern.hh"

void *make_vert_hive(BMesh *bm)
{
  return static_cast<void *>(MEM_new<VertHive>("VertHive", bm));
}
void free_vert_hive(void *hive)
{
  MEM_delete<VertHive>(static_cast<VertHive *>(hive));
}
void *make_edge_hive(BMesh *bm)
{
  return static_cast<void *>(MEM_new<EdgeHive>("EdgeHive", bm));
}
void free_edge_hive(void *hive)
{
  MEM_delete<EdgeHive>(static_cast<EdgeHive *>(hive));
}

void *make_loop_hive(BMesh *bm)
{
  return static_cast<void *>(MEM_new<LoopHive>("LoopHive", bm));
}
void free_loop_hive(void *hive)
{
  MEM_delete<LoopHive>(static_cast<LoopHive *>(hive));
}

void *make_face_hive(BMesh *bm)
{
  return static_cast<void *>(MEM_new<FaceHive>("FaceHive", bm));
}
void free_face_hive(void *hive)
{
  MEM_delete<FaceHive>(static_cast<FaceHive *>(hive));
}

template<typename HiveType> static void bm_hive_iternew(HiveType *hive, HiveIter *iter)
{
  *reinterpret_cast<typename HiveType::Iterator *>(iter->reserved) = hive->begin();
}

template<typename HiveType> static bool bm_hive_iterdone(HiveIter *iter)
{
  typename HiveType::Iterator *real_iter = reinterpret_cast<typename HiveType::Iterator *>(
      iter->reserved);

  return real_iter->done();
}

template<typename HiveType> static void *bm_hive_iterstep(HiveIter *iter)
{
  typename HiveType::Iterator *real_iter = reinterpret_cast<typename HiveType::Iterator *>(
      iter->reserved);

  void *ret = real_iter->done() ? nullptr : static_cast<void *>(*(*real_iter));
  real_iter->operator++();

  return ret;
}

void BM_hive_iternew(void *hive, HiveIter *iter, char htype)
{
  using namespace blender;

  iter->htype = htype;
  iter->hive = hive;

  switch (htype) {
    case BM_VERT:
      bm_hive_iternew<VertHive>(static_cast<VertHive *>(hive), iter);
      break;
    case BM_EDGE:
      bm_hive_iternew<EdgeHive>(static_cast<EdgeHive *>(hive), iter);
      break;
    case BM_LOOP:
      bm_hive_iternew<LoopHive>(static_cast<LoopHive *>(hive), iter);
      break;
    case BM_FACE:
      bm_hive_iternew<FaceHive>(static_cast<FaceHive *>(hive), iter);
      break;
  }
}

bool BM_hive_iterdone(HiveIter *iter)
{
  using namespace blender;

  switch (iter->htype) {
    case BM_VERT:
      return bm_hive_iterdone<VertHive>(iter);
    case BM_EDGE:
      return bm_hive_iterdone<EdgeHive>(iter);
    case BM_LOOP:
      return bm_hive_iterdone<LoopHive>(iter);
    case BM_FACE:
      return bm_hive_iterdone<FaceHive>(iter);
  }

  BLI_assert_unreachable();
  return true;
}

void *BM_hive_iterstep(HiveIter *iter)
{
  using namespace blender;

  switch (iter->htype) {
    case BM_VERT:
      return bm_hive_iterstep<VertHive>(iter);
    case BM_EDGE:
      return bm_hive_iterstep<EdgeHive>(iter);
    case BM_LOOP:
      return bm_hive_iterstep<LoopHive>(iter);
    case BM_FACE:
      return bm_hive_iterstep<FaceHive>(iter);
  }

  BLI_assert_unreachable();
  return nullptr;
}

template<typename T, typename HiveType = VertHive>
static void bm_task_parallel_memhive(void *vhive,
                                     void *userdata,
                                     TaskParallelMempoolFunc func,
                                     const TaskParallelSettings *settings)
{
  /*TODO: implement parallelism code. */
  HiveType *hive = static_cast<HiveType *>(vhive);
  TaskParallelTLS tls;

  tls.userdata_chunk = settings->userdata_chunk;

  for (T *elem : *hive) {
    func(userdata, reinterpret_cast<MempoolIterData *>(elem), &tls);
  }
}

void BM_task_parallel_memhive(void *hive,
                              char htype,
                              void *userdata,
                              TaskParallelMempoolFunc func,
                              const TaskParallelSettings *settings)
{
  switch (htype) {
    case BM_VERT:
      bm_task_parallel_memhive<BMVert, VertHive>(hive, userdata, func, settings);
      break;
    case BM_EDGE:
      bm_task_parallel_memhive<BMEdge, EdgeHive>(hive, userdata, func, settings);
      break;
    case BM_LOOP:
      bm_task_parallel_memhive<BMLoop, LoopHive>(hive, userdata, func, settings);
      break;
    case BM_FACE:
      bm_task_parallel_memhive<BMFace, FaceHive>(hive, userdata, func, settings);
      break;
  }
}

BMVert *bm_alloc_vert(BMesh *bm, int hive)
{
  return static_cast<VertHive *>(bm->vhive)->alloc(hive);
}
void bm_free_vert(BMesh *bm, BMVert *v)
{
  // printf("freeing vert");
  static_cast<VertHive *>(bm->vhive)->free(v);
}
int bm_vert_hive_get(struct BMesh *bm, const BMVert *v)
{
  return max_ii(static_cast<VertHive *>(bm->vhive)->find_hive(v), 0);
}

BMEdge *bm_alloc_edge(BMesh *bm, int hive)
{
  return static_cast<EdgeHive *>(bm->ehive)->alloc(hive);
}
void bm_free_edge(BMesh *bm, BMEdge *e)
{
  // printf("freeing edge");
  static_cast<EdgeHive *>(bm->ehive)->free(e);
}
int bm_edge_hive_get(struct BMesh *bm, const BMEdge *e)
{
  return max_ii(static_cast<EdgeHive *>(bm->ehive)->find_hive(e), 0);
}

BMLoop *bm_alloc_loop(BMesh *bm, int hive)
{
  return static_cast<LoopHive *>(bm->lhive)->alloc(hive);
}
void bm_free_loop(BMesh *bm, BMLoop *l)
{
  // printf("freeing loop");
  static_cast<LoopHive *>(bm->lhive)->free(l);
}
int bm_loop_hive_get(struct BMesh *bm, const BMLoop *l)
{
  return max_ii(static_cast<LoopHive *>(bm->lhive)->find_hive(l), 0);
}

BMFace *bm_alloc_face(BMesh *bm, int hive)
{
  return static_cast<FaceHive *>(bm->fhive)->alloc(hive);
}
void bm_free_face(BMesh *bm, BMFace *f)
{
  // printf("freeing face");
  static_cast<FaceHive *>(bm->fhive)->free(f);
}
int bm_face_hive_get(struct BMesh *bm, const BMFace *f)
{
  return max_ii(static_cast<FaceHive *>(bm->fhive)->find_hive(f), 0);
}

void *customdata_hive_alloc(void *hivealloc, int hive)
{
  return static_cast<void *>(static_cast<CustomDataHive *>(hivealloc)->alloc(hive));
}

void customdata_hive_free(void *hivealloc, void *ptr)
{
  static_cast<CustomDataHive *>(hivealloc)->free(static_cast<int *>(ptr));
}

void customdata_hive_destroy(void *hivealloc)
{
  MEM_delete<CustomDataHive>(static_cast<CustomDataHive *>(hivealloc));
}

int customdata_hive_get_size(void *hivealloc)
{
  return int(static_cast<CustomDataHive *>(hivealloc)->get_mem_size());
}
