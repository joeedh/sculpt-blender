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
#define DEAD_MAGIC ('d' | ('e' << 8) | ('a' << 16) | ('d' || 24))

//#define BLI_SAFEPOOL_HAVE_LENGTH

//keep eight-byte aligned!
typedef struct poolchunk {
  struct poolchunk* next, * prev;
  unsigned int threadnr, magic;
} poolchunk;

typedef struct deadelem {
  struct deadelem *next;
  int dead_magic;
} deadelem;

typedef struct pool_thread_data {
  ListBase chunks;
  void* freehead;
  SpinLock lock;
  unsigned int used;
} pool_thread_data;

typedef struct BLI_ThreadSafePool {
  pool_thread_data* threadchunks;
  int maxthread;
  SpinLock global_lock;

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  unsigned int length;
#endif
  size_t esize, csize;
} BLI_ThreadSafePool;

static int getalign(int size) {
  if (size & 15) {
    return size + (16 - (size & 15));
  }

  return size;
}

static size_t get_chunk_size(BLI_ThreadSafePool* pool) {
  return getalign(sizeof(pool_thread_data) + pool->esize*pool->csize);
}

static poolchunk* get_chunk_from_elem(BLI_ThreadSafePool *pool, void* elem) {
  uintptr_t addr = (uintptr_t) elem;
  uintptr_t size = get_chunk_size(pool);

  if (!elem) {
    return NULL;
  }

  addr = addr - (addr % size);

  poolchunk *chunk = (poolchunk*)addr;

  if (chunk->magic != POOL_CHUNK_MAGIC) {
    return NULL;
  }

  return chunk;
}

static poolchunk *new_chunk(BLI_ThreadSafePool *pool, pool_thread_data* thread_data) {
  size_t size = get_chunk_size(pool);
  size_t esize = pool->esize;

  //align size?
  /*
  if (size & 7) {
    size += 8 - (size & 7);
  };*/

  poolchunk* chunk = MEM_mallocN_aligned(size, getalign(size), "safepool chunk");

  chunk->magic = POOL_CHUNK_MAGIC;
  chunk->threadnr = thread_data - pool->threadchunks;

  BLI_addtail(&thread_data->chunks, chunk);
  deadelem *first = NULL;

  for (size_t i = 0; i < pool->csize-1; i++) {
    deadelem *de = (deadelem*)(((char*)chunk) + sizeof(poolchunk) + esize*i);

    if (i ==  0) {
      first = de;
    }

    de->next = de + 1;
    de->dead_magic = DEAD_MAGIC;
  }

  deadelem *de = (deadelem*)(((char*)chunk) + sizeof(poolchunk) + esize*(pool->csize-1));
  de->next = thread_data->freehead;
  de->dead_magic = DEAD_MAGIC;

  thread_data->freehead = first;

  return chunk;
}

BLI_ThreadSafePool* BLI_safepool_create(int elemsize, int chunksize, int maxthread) {
  BLI_ThreadSafePool* pool = MEM_callocN(sizeof(*pool), "BLI_ThreadSafePool");

  elemsize = MAX2(elemsize, sizeof(void*)*2);

  pool->maxthread = maxthread;
  pool->threadchunks = MEM_callocN(sizeof(pool_thread_data) * maxthread, "pool->threadchunks");
  pool->esize = elemsize;
  pool->csize = chunksize;

  BLI_spin_init(&pool->global_lock);

  for (int i = 0; i < maxthread; i++) {
    BLI_spin_init(&pool->threadchunks[i].lock);
    new_chunk(pool, pool->threadchunks);
  }

  return pool;
}

int BLI_safepool_elem_is_dead(void *elem) {
  deadelem *de = (deadelem*)elem;

  return de->dead_magic == DEAD_MAGIC;
}

void BLI_safepool_destroy(BLI_ThreadSafePool* pool) {
  poolchunk* chunk;
  int i;

  //wait for all threads to end?
  //BLI_spin_lock(&pool->global_lock);
  //BLI_spin_unlock(&pool->global_lock);

  for (i = 0; i < pool->maxthread; i++) {
    BLI_spin_end(&pool->threadchunks[i].lock);
    poolchunk* next;

    for (chunk = pool->threadchunks[i].chunks.first; chunk; chunk = next) {
      next = chunk->next;
      MEM_freeN(chunk);
    }
  }

  BLI_spin_end(&pool->global_lock);
  MEM_freeN(pool->threadchunks);
  MEM_freeN(pool);
}

void* BLI_safepool_alloc(BLI_ThreadSafePool *pool, int thread) {
  pool_thread_data *tdata = pool->threadchunks + thread;

  if (tdata->freehead) {
    //lock?
    BLI_spin_lock(&tdata->lock);

    deadelem *de = (deadelem*) tdata->freehead;
    tdata->freehead = de->next;
    tdata->used++;

    BLI_spin_unlock(&tdata->lock);

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
    BLI_spin_lock(&pool->global_lock);
    pool->length++;
    BLI_spin_unlock(&pool->global_lock);
#endif
    return (void*) de;
  }

  new_chunk(pool, tdata);

  return BLI_safepool_alloc(pool, thread);
}


int get_elem_thread(BLI_ThreadSafePool* pool, void* elem) {
  for (int i = 0; i < pool->maxthread; i++) {
    BLI_spin_lock(&pool->threadchunks[i].lock);
  }

  poolchunk *chunk = get_chunk_from_elem(pool, elem);

  if (!chunk) {
    for (int i = 0; i < pool->maxthread; i++) {
      BLI_spin_unlock(&pool->threadchunks[i].lock);
    }

    return -1;
  }

  int ret = chunk->threadnr;

  for (int i = 0; i < pool->maxthread; i++) {
    BLI_spin_unlock(&pool->threadchunks[i].lock);
  }

  return ret;
}

void BLI_safepool_threaded_free(BLI_ThreadSafePool* pool, void* elem, int thread) {
  pool_thread_data *tdata = pool->threadchunks + thread;

  BLI_spin_lock(&tdata->lock);

  deadelem *de = (deadelem*)elem;

  de->next = tdata->freehead;
  de->dead_magic = DEAD_MAGIC;
  tdata->freehead = de;
  
  tdata->used--;

  BLI_spin_unlock(&tdata->lock);

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  BLI_spin_lock(&pool->global_lock);
  pool->length--;
  BLI_spin_unlock(&pool->global_lock);
#endif
}

void BLI_safepool_free(BLI_ThreadSafePool *pool, void *elem) {
  int i = get_elem_thread(pool, elem);

  if (i < 0) {
    fprintf(stderr, "Elem not in pool! %p\n", elem);
    return;
  }

  BLI_safepool_threaded_free(pool, elem, i);
}

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
int BLI_safepool_length(BLI_ThreadSafePool *pool) {
  return pool->length;
}
#endif

void lock_all_threads(BLI_ThreadSafePool* pool) {
  for (int i = 0; i < pool->maxthread; i++) {
    BLI_spin_lock(&pool->threadchunks[i].lock);
  }
}

void unlock_all_threads(BLI_ThreadSafePool* pool) {
  for (int i = 0; i < pool->maxthread; i++) {
    BLI_spin_unlock(&pool->threadchunks[i].lock);
  }
}

void BLI_safepool_iternew(struct BLI_ThreadSafePool* pool, ThreadSafePoolIter* iter) {
  lock_all_threads(pool);

  memset(iter, 0, sizeof(*iter));
  iter->pool = pool;

  iter->thread = 0;
  iter->chunk = pool->threadchunks[0].chunks.first;
  iter->i = 0;
}

void BLI_safepool_iterfree(ThreadSafePoolIter* iter) {
  unlock_all_threads(iter->pool);
}

void* BLI_safepool_iterstep(ThreadSafePoolIter* iter) {
  BLI_ThreadSafePool *pool = iter->pool;
  poolchunk *chunk = iter->chunk;

  if (iter->i < 0) { //end of iteration
    return NULL;
  }

  char *ptr = ((char*)chunk) + sizeof(poolchunk);
  ptr += pool->esize * iter->i;

  iter->i++;
  if (iter->i >= pool->csize) {
    iter->i = 0;
    chunk = iter->chunk = chunk->next;
  }

  if (!chunk && iter->thread < pool->maxthread-1) {
    iter->thread++;
    iter->chunk = pool->threadchunks[iter->thread].chunks.first;
  } else if (!chunk) {
    iter->i = -1; //flag end of iteration
  }

  deadelem *de = (deadelem*) ptr;

  if (!ptr || de->dead_magic == DEAD_MAGIC) {
    return BLI_safepool_iterstep(iter);
  }

  return (void*) ptr;
}

