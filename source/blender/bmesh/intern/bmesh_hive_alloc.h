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
struct BMVert *bm_alloc_vert(struct BMesh *bm, int hive);
void bm_free_vert(struct BMesh *bm, BMVert *v);
void free_vert_hive(void *hive);
int bm_vert_hive_get(struct BMesh *bm, const BMVert *v);

void *make_edge_hive(struct BMesh *bm);
struct BMEdge *bm_alloc_edge(struct BMesh *bm, int hive);
void bm_free_edge(struct BMesh *bm, BMEdge *e);
void free_edge_hive(void *hive);
int bm_edge_hive_get(struct BMesh *bm, const BMEdge *e);

void *make_loop_hive(struct BMesh *bm);
struct BMLoop *bm_alloc_loop(struct BMesh *bm, int hive);
void bm_free_loop(struct BMesh *bm, BMLoop *l);
void free_loop_hive(void *hive);
int bm_loop_hive_get(struct BMesh *bm, const BMLoop *l);

void *make_face_hive(struct BMesh *bm);
struct BMFace *bm_alloc_face(struct BMesh *bm, int hive);
void bm_free_face(struct BMesh *bm, BMFace *f);
void free_face_hive(void *hive);
int bm_face_hive_get(struct BMesh *bm, const BMFace *f);

void *customdata_hive_alloc(void *hivealloc, int hive);
void customdata_hive_free(void *hivealloc, void *ptr);
void customdata_hive_destroy(void *hivealloc);
int customdata_hive_get_size(void *hivealloc);

void BM_task_parallel_memhive(void *hive,
                              char htype,
                              void *userdata,
                              TaskParallelMempoolFunc func,
                              const TaskParallelSettings *settings);

#ifdef __cplusplus
}
#endif
