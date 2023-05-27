#pragma once

#include "BLI_task.h"

typedef struct HiveIter {
  char reserved[255];
  void *hive;
  char htype;
} HiveIter;

struct BMesh;
struct BMVert;
struct BMEdge;
struct BMLoop;
struct BMFace;

#ifdef __cplusplus
extern "C" {
#endif

void BM_hive_iternew(void *hive, HiveIter *iter, char htype);
bool BM_hive_iterdone(HiveIter *iter);
void *BM_hive_iterstep(HiveIter *iter);

void *make_vert_hive(struct BMesh *bm);
struct BMVert *bm_alloc_vert(struct BMesh *bm);
void bm_free_vert(struct BMesh *bm, BMVert *v);
void free_vert_hive(void *hive);

void *make_edge_hive(struct BMesh *bm);
struct BMEdge *bm_alloc_edge(struct BMesh *bm);
void bm_free_edge(struct BMesh *bm, BMEdge *e);
void free_edge_hive(void *hive);

void *make_loop_hive(struct BMesh *bm);
struct BMLoop *bm_alloc_loop(struct BMesh *bm);
void bm_free_loop(struct BMesh *bm, BMLoop *l);
void free_loop_hive(void *hive);

void *make_face_hive(struct BMesh *bm);
struct BMFace *bm_alloc_face(struct BMesh *bm);
void bm_free_face(struct BMesh *bm, BMFace *f);
void free_face_hive(void *hive);

void *customdata_hive_alloc(void *hive);
void customdata_hive_free(void *hive, void *ptr);
void customdata_hive_destroy(void *hive);
int customdata_hive_get_size(void *hive);

void BM_task_parallel_memhive(void *hive,
                              char htype,
                              void *userdata,
                              TaskParallelMempoolFunc func,
                              const TaskParallelSettings *settings);

#ifdef __cplusplus
}
#endif
