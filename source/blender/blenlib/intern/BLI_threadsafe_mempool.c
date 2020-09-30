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

struct pool_thread_data;

/*this structure is always aligned to the pointer size*/
typedef struct poolelem { 
  struct pool_thread_data *poolthread;
  struct poolelem *next; //eats into returned memory
  intptr_t dead_magic; //eats into returned memory
} poolelem;

typedef struct pool_thread_data {
  ListBase chunks;
  void* freehead;
  ThreadRWMutex lock;
  unsigned int used, threadnr;
} pool_thread_data;

typedef struct BLI_ThreadSafePool {
  pool_thread_data* threadchunks;
  int maxthread;

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  unsigned int length;
  ThreadRWMutex lengthlock;
#endif
  size_t esize, csize;
} BLI_ThreadSafePool;

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
  return getalign(sizeof(pool_thread_data) + pool->esize*pool->csize);
}

#define getelem(elem) ((poolelem*) ((char*) (elem) - sizeof(void*)))

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

  poolchunk* chunk = MEM_mallocN_aligned(size, getalign(size), "safepool chunk");

  chunk->magic = POOL_CHUNK_MAGIC;
  chunk->threadnr = thread_data - pool->threadchunks;

  BLI_addtail(&thread_data->chunks, chunk);
  poolelem *first = NULL;

  for (size_t i = 0; i < pool->csize-1; i++) {
    poolelem *de = (poolelem*)(((char*)chunk) + sizeof(poolchunk) + esize*i);

    if (i ==  0) {
      first = de;
    }

    de->next = de + 1;
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

  elemsize = MAX2(elemsize, sizeof(void*)*2);

  pool->maxthread = maxthread;
  pool->threadchunks = MEM_callocN(sizeof(pool_thread_data) * maxthread, "pool->threadchunks");
  pool->esize = elemsize + sizeof(void*); //add header pointer to owning chunk
  pool->csize = chunksize;

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  BLI_rw_mutex_init(&pool->length_lock);
#endif

  for (int i = 0; i < maxthread; i++) {
    BLI_rw_mutex_init(&pool->threadchunks[i].lock);
    pool->threadchunks[i].threadnr = i;
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
  return BLI_safepool_alloc(pool, thread);
}

int get_elem_thread(BLI_ThreadSafePool* pool, void* elem) {
  pool_thread_data *threadpool = get_poolthread_from_elem(pool, elem);
  
  if (!threadpool) {
    return -1;
  }

  int ret = threadpool->threadnr;
  return ret;
}

void BLI_safepool_threaded_free(BLI_ThreadSafePool* pool, void* elem) {
  /*hrm.  I could add to the current pool's freelist, couldn't I
    let's try that

    int thread = get_elem_thread(pool, elem);
   */

  int thread = BLI_thread_local_get(curthread);

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
    BLI_rw_mutex_lock(&pool->threadchunks[i].lock, THREAD_LOCK_WRITE);
  }
}

void unlock_all_threads(BLI_ThreadSafePool* pool) {
  for (int i = 0; i < pool->maxthread; i++) {
    BLI_rw_mutex_unlock(&pool->threadchunks[i].lock);
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

  poolelem *de = getelem(ptr);

  if (!ptr || de->dead_magic == DEAD_MAGIC) {
    return BLI_safepool_iterstep(iter);
  }

  return (void*) &de->next;
}

