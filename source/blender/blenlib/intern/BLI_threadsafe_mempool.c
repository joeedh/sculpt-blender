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
static ThreadLocal(int) curthread = -1;
BLI_thread_local_create(curthread);

void BLI_safepool_threadnr_set(int threadnr) {
  BLI_thread_local_set(curthread, threadnr);
}

static int get_curthread(BLI_ThreadSafePool *pool) {
  return BLI_thread_local_get(curthread) % pool->maxthread;
}

static size_t get_chunk_size(BLI_ThreadSafePool* pool) {
  return getalign(sizeof(pool_thread_data)) + pool->esize*pool->csize;

  //return getalign(sizeof(pool_thread_data) + pool->esize*pool->csize);
}

static pool_thread_data* get_poolthread_from_elem(BLI_ThreadSafePool *pool, void* elem) {
  //version of code for if elements are allowed to link themselves to other thread pools
  poolelem *de = bli_safepool_getelem(elem);

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

  //XXX temporarily ensure maxthread is at least 4, delete me
  maxthread = MAX2(maxthread, 4);

#ifdef BYPASS_POOL
  pool->nodeset = BLI_gset_ptr_new("safepool nodeset");
#endif

  pool->checkmagic = _SPOOL_MAGIC;

  //align to pointer size
  if (elemsize & (sizeof(void*)-1)) {
    elemsize += sizeof(void*) - (elemsize & (sizeof(void*)-1));
  }

  maxthread = MAX2(maxthread, 1);

#ifndef DEBUG_SAFEPOOL
  //add header pointer to size
  elemsize = MAX2(elemsize + sizeof(void*), sizeof(void*)*3);

  //rest of poolelem struct will eat into returned client memory
#else
  //don't re-use client memory for next pointer if DEBUG_SAFEPOOL is set,
  //and also add a tail checkpoint element
  elemsize += sizeof(poolelem) + sizeof(void*);
#endif

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

void BLI_safepool_destroy(BLI_ThreadSafePool* pool) {
#ifdef BYPASS_POOL
  Link *node, *next;

  for (node=pool->nodes.first; node; node=next) {
    next = node->next;
    free(node);
  }
#else
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

  pool->checkmagic = 0;
#endif

  MEM_freeN(pool->threadchunks);
  MEM_freeN(pool);
}

static volatile int curthread_gen = 1;
static volatile ThreadMutex curthread_gen_mutex = BLI_MUTEX_INITIALIZER;

static void check_curthread() {
  if (BLI_thread_local_get(curthread) <= 0) {
    BLI_mutex_lock(&curthread_gen_mutex);
    BLI_thread_local_set(curthread, curthread_gen++);
    BLI_mutex_unlock(&curthread_gen_mutex);
  }
}

CUSTOMALLOC void* BLI_safepool_alloc(BLI_ThreadSafePool *pool) {
#ifdef BYPASS_POOL
  void *ret = malloc(pool->esize + sizeof(void*)*2);

  memset(ret, 0, pool->esize+sizeof(void*)*2);

  BLI_gset_insert(pool->nodeset, ret);
  BLI_addtail(&pool->nodes, ret);

  char *addr = (char*) ret;
  addr += sizeof(void*)*2;

  return (void*)addr;
#else
  check_curthread();

  int thread = get_curthread(pool);

  if (!pool || pool->checkmagic != _SPOOL_MAGIC) {
    printf("bad call to BLI_safepool_alloc! %p\n", pool);
    return NULL;
  }

  pool_thread_data *tdata = pool->threadchunks + thread;

  BLI_rw_mutex_lock(&tdata->lock, THREAD_LOCK_WRITE);

  if (tdata->freehead) {
    poolelem *de = (poolelem*) tdata->freehead;
    if (de->dead_magic != DEAD_MAGIC) {
      printf("mempool corruption: %p\n", de);
      return NULL;
    }

    tdata->freehead = de->next;
    tdata->used++;

    BLI_rw_mutex_unlock(&tdata->lock);

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
    BLI_rw_mutex_lock(&pool->lengthlock, THREAD_LOCK_WRITE);
    pool->length++;
    BLI_rw_mutex_unlock(&pool->lengthlock);
#endif

#ifndef DEBUG_SAFEPOOL
    de->dead_magic = 0;
#else
    {
      de->dead_magic = LIVE_MAGIC;
      char *ptr = (char*)de;
      ptr += pool->esize - sizeof(void*);

      uintptr_t *i = (uintptr_t*)ptr;
      *i = TAIL_MAGIC;
    }
#endif

#ifndef DEBUG_SAFEPOOL
    return (void*) &de->next;
#else
    return (void*)(de + 1);
#endif
  }

  new_chunk(pool, tdata);

  BLI_rw_mutex_unlock(&tdata->lock);
  return BLI_safepool_alloc(pool);

#endif
}

int get_elem_thread(BLI_ThreadSafePool* pool, void* elem) {
  pool_thread_data *threadpool = get_poolthread_from_elem(pool, elem);
  
  if (!threadpool) {
    return -1;
  }

  int ret = threadpool->threadnr;
  return ret;
}

static bool memcheck(void *mem) {
  bool bad = !mem;
  bad = bad || (((intptr_t)mem) & 0x7);

  return bad;
}

bool check_safepool_elem(BLI_ThreadSafePool* pool, void* elem) {
  if (pool->checkmagic != _SPOOL_MAGIC) {
    printf("corrupted pool in safepool free! %p\n", pool);
    return false;
  }

  if (memcheck((void*)pool)) {
    printf("bad pool in safepool free! %p\n", pool);
    return false;
  }

  bool bad = !elem;
  bad = bad || (((intptr_t)elem) & 0x7);

  if (bad) {
    printf("bad memory in safepool free! %p\n", elem);
    return false;
  }

  return true;

  for (int i=0; i<pool->maxthread; i++) {
    pool_thread_data *data = pool->threadchunks + i;
    poolchunk *chunk;

    for (chunk = (poolchunk*) data->chunks.first; chunk; chunk=chunk->next) {
      char *p1 = (char*) chunk;
      char *p2 = (char*) elem;

      if (p2 >= p1 && p2 < p1 + sizeof(poolchunk) + pool->esize*pool->csize) {
        return true;
      }
    }
  }

  return false;
}

void BLI_safepool_free(BLI_ThreadSafePool* pool, void* elem) {
#ifdef BYPASS_POOL
  elem = (void*)(((void**)elem) - 2);

  BLI_remlink(&pool->nodes, elem);
  BLI_gset_remove(pool->nodeset, elem, NULL);

  free(elem);
#else
  //XXX

  check_curthread();

  if (!check_safepool_elem(pool, elem)) {
    return;
  }

  //add to current thread's free list
  int thread = get_curthread(pool);

  if (!pool) {
    printf("error!\n");
    return;
  }

  pool_thread_data *tdata = pool->threadchunks + thread;
  BLI_rw_mutex_lock(&tdata->lock, THREAD_LOCK_WRITE);

  poolelem *de = bli_safepool_getelem(elem);

#ifdef DEBUG_SAFEPOOL
  {
    char *addr = (char*)de;
    addr = addr + pool->esize - sizeof(void*);

    uintptr_t *i = (uintptr_t*)addr;
    if (*i != TAIL_MAGIC) {
      printf("eek! %p %p %c%c%c%c%c%c%c%c\n", (void*)*i, (void*)TAIL_MAGIC, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
    }
  }
#endif

  if (de->dead_magic == DEAD_MAGIC) {
    printf("error: double free in mem pool %p\n", elem);
    return;
  }

  de->next = (poolelem*) tdata->freehead;
  de->dead_magic = DEAD_MAGIC;
  tdata->freehead = de;
  
  tdata->used--;

  BLI_rw_mutex_unlock(&tdata->lock);

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  BLI_rw_mutex_lock(&pool->lengthlock, THREAD_LOCK_WRITE);
  pool->length--;
  BLI_rw_mutex_unlock(&pool->lengthlock);
#endif
#endif
}

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
int BLI_safepool_length(BLI_ThreadSafePool *pool) {
  return pool->length;
}
#endif
