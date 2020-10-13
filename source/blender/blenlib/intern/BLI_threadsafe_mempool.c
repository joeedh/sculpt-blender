#pragma once

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "BLI_threadsafe_mempool.h"
#include "BLI_threads.h"
#include "BLI_utildefines.h"
#include "MEM_guardedalloc.h"
#include "atomic_ops.h"
#include "BLI_listbase.h"

#define POOL_CHUNK_MAGIC ('p' | ('o' << 8) | ('o' << 16) | ('l' || 24))

//#define BLI_SAFEPOOL_HAVE_LENGTH

static int getalign(int size) {
  if (size & 15) {
    return size + (16 - (size & 15));
  }

  return size;
}

/*not sure how to pass thread number to customdata functions, so this evilness here
  is used for now*/
ThreadLocal(int) curthread = 0;
BLI_thread_local_create(curthread);

void BLI_safepool_threadnr_set(int threadnr) {
  BLI_thread_local_set(curthread, threadnr);
}

static size_t get_chunk_size(BLI_ThreadSafePool* pool) {
  return getalign(sizeof(pool_thread_data)) + pool->esize*pool->csize;

  //return getalign(sizeof(pool_thread_data) + pool->esize*pool->csize);
}

#define getelem(elem) ((poolelem*) (((char*)(elem)) - sizeof(void*)))

static pool_thread_data* get_poolthread_from_elem(BLI_ThreadSafePool *pool, void* elem) {
  //version of code for if elements are allowed to link themselves to other thread pools
  poolelem *de = getelem(elem);

  return de->poolthread;
}

static poolchunk *new_chunk(BLI_ThreadSafePool *pool, pool_thread_data* thread_data) {
  size_t size = get_chunk_size(pool);
  size_t esize = pool->esize;

  //align size?
  /*
  if (size & 7) {
    size += 8 - (size & 7);
  };*/

  poolchunk* chunk = MEM_mallocN(size, "safepool chunk");

  chunk->magic = POOL_CHUNK_MAGIC;
  chunk->threadnr = thread_data - pool->threadchunks;
  
  BLI_addtail(&thread_data->chunks, chunk);
  poolelem *first = NULL;

  for (size_t i = 0; i < pool->csize-1; i++) {
    poolelem *de = (poolelem*)(((char*)chunk) + sizeof(poolchunk) + esize*i);
    poolelem *next = (poolelem*)(((char*)chunk) + sizeof(poolchunk) + esize*(i+1));

    if (i ==  0) {
      first = de;
    }

    de->next = next;
    de->dead_magic = DEAD_MAGIC;
  }

  poolelem *de = (poolelem*)(((char*)chunk) + sizeof(poolchunk) + esize*(pool->csize-1));
  de->next = thread_data->freehead;
  de->dead_magic = DEAD_MAGIC;

  thread_data->freehead = first;

  return chunk;
}

BLI_ThreadSafePool* BLI_safepool_create(int elemsize, int chunksize, int maxthread) {
  BLI_ThreadSafePool* pool = MEM_callocN(sizeof(*pool), "BLI_ThreadSafePool");

  //align to pointer size
  if (elemsize & 7) {
    elemsize += 8 - (elemsize & 7);
  }

  //add header pointer to owning chunk
  elemsize = MAX2(elemsize + sizeof(void*), sizeof(void*)*2);
  maxthread = MAX2(maxthread, 1);

  pool->maxthread = maxthread;
  pool->threadchunks = MEM_callocN(sizeof(pool_thread_data) * maxthread, "pool->threadchunks");
  pool->esize = elemsize; 
  pool->csize = chunksize;

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  BLI_rw_mutex_init(&pool->length_lock);
#endif

  for (int i = 0; i < maxthread; i++) {
    BLI_rw_mutex_init(&pool->threadchunks[i].lock);
    pool->threadchunks[i].threadnr = i;
    memset(pool->threadchunks + i, 0, sizeof(pool_thread_data));
    new_chunk(pool, pool->threadchunks + i);
  }

  return pool;
}

int BLI_safepool_elem_is_dead(void *elem) {
  poolelem *de = getelem(elem);

  return de->dead_magic == DEAD_MAGIC;
}

void BLI_safepool_destroy(BLI_ThreadSafePool* pool) {
  poolchunk* chunk;
  int i;

  //wait for all threads to end?

  for (i = 0; i < pool->maxthread; i++) {
    BLI_rw_mutex_end(&pool->threadchunks[i].lock);
    poolchunk* next;

    for (chunk = pool->threadchunks[i].chunks.first; chunk; chunk = next) {
      next = chunk->next;
      MEM_freeN(chunk);
    }
  }
#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  BLI_rw_mutex_end(&pool->length_lock);
#endif

  MEM_freeN(pool->threadchunks);
  MEM_freeN(pool);
}

void* BLI_safepool_alloc(BLI_ThreadSafePool *pool) {
  int thread = BLI_thread_local_get(curthread);

  pool_thread_data *tdata = pool->threadchunks + thread;

  BLI_rw_mutex_lock(&tdata->lock, THREAD_LOCK_WRITE);

  if (tdata->freehead) {
    poolelem *de = (poolelem*) tdata->freehead;
    tdata->freehead = de->next;
    tdata->used++;

    BLI_rw_mutex_unlock(&tdata->lock);

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
    BLI_rw_mutex_lock(&pool->lengthlock, THREAD_LOCK_WRITE);
    pool->length++;
    BLI_rw_mutex_unlock(&pool->lengthlock);
#endif
    return (void*) &de->next;
  }

  new_chunk(pool, tdata);

  BLI_rw_mutex_unlock(&tdata->lock);
  return BLI_safepool_alloc(pool);
}

int get_elem_thread(BLI_ThreadSafePool* pool, void* elem) {
  pool_thread_data *threadpool = get_poolthread_from_elem(pool, elem);
  
  if (!threadpool) {
    return -1;
  }

  int ret = threadpool->threadnr;
  return ret;
}

void BLI_safepool_free(BLI_ThreadSafePool* pool, void* elem) {
  /*hrm.  I could add to the current pool's freelist, couldn't I
    let's try that

    int thread = get_elem_thread(pool, elem);
   */

  int thread = BLI_thread_local_get(curthread);

  if (!pool) {
    printf("error!\n");
    return;
  }

  pool_thread_data *tdata = pool->threadchunks + thread;
  BLI_rw_mutex_lock(&tdata->lock, THREAD_LOCK_WRITE);

  poolelem *de = getelem(elem);

  de->next = tdata->freehead;
  de->dead_magic = DEAD_MAGIC;
  tdata->freehead = de;
  
  tdata->used--;

  BLI_rw_mutex_unlock(&tdata->lock);

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  BLI_rw_mutex_lock(&pool->lengthlock, THREAD_LOCK_WRITE);
  pool->length--;
  BLI_rw_mutex_unlock(&pool->lengthlock);
#endif
}

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
int BLI_safepool_length(BLI_ThreadSafePool *pool) {
  return pool->length;
}
#endif
