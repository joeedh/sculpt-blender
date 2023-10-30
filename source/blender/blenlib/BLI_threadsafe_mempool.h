/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2008 Blender Foundation.
 * All rights reserved.
 */
#ifndef _BLI_THREADSAFE_MEMPOOL_H
#define _BLI_THREADSAFE_MEMPOOL_H

#include "BLI_threads.h"
#include "BLI_compiler_compat.h"
#include "BLI_listbase.h"

//#define BYPASS_POOL
#define CUSTOMALLOC __declspec(allocator)

#define DEBUG_SAFEPOOL

#ifdef BYPASS_POOL
#include "BLI_ghash.h"
#endif

struct BLI_ThreadSafePool;
typedef struct BLI_ThreadSafePool BLI_ThreadSafePool;

struct BLI_ThreadSafePool* BLI_safepool_create(int elemsize, int chunksize, int maxthread);
CUSTOMALLOC void* BLI_safepool_alloc(struct BLI_ThreadSafePool *pool);

void BLI_safepool_free(struct BLI_ThreadSafePool*pool, void *elem);

void BLI_safepool_threaded_free(struct BLI_ThreadSafePool*pool, void *elem, int threadnr);
#ifdef BLI_SAFEPOOL_HAVE_LENGTH
int BLI_safepool_length(struct BLI_ThreadSafePool*pool);
#endif
void BLI_safepool_destroy(struct BLI_ThreadSafePool* pool);

typedef struct ThreadSafePoolIter {
  struct BLI_ThreadSafePool* pool;
  int thread;
  void *chunk;
  int i;
#ifdef BYPASS_POOL
  Link *node;
#endif
} ThreadSafePoolIter;

void BLI_safepool_iternew(struct BLI_ThreadSafePool* pool, ThreadSafePoolIter* iter);
void BLI_safepool_iterfree(ThreadSafePoolIter* iter);
void* BLI_safepool_iterstep(ThreadSafePoolIter* iter);

/*not sure how to pass thread number to customdata functions, so this evilness here
is used for now*/
void BLI_safepool_threadnr_set(int threadnr);


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
  uintptr_t dead_magic; //eats into returned memory, unless DEBUG_SAFEPOOL is defined
} poolelem;

typedef struct pool_thread_data {
  ListBase chunks;
  void* freehead;
  ThreadRWMutex lock;
  unsigned int used, threadnr;
} pool_thread_data;

typedef struct BLI_ThreadSafePool {
  pool_thread_data* threadchunks;
  int maxthread, checkmagic;

#ifdef BLI_SAFEPOOL_HAVE_LENGTH
  unsigned int length;
  ThreadRWMutex lengthlock;
#endif
  size_t esize, csize;
#ifdef BYPASS_POOL
  ListBase nodes;
  GSet *nodeset;
#endif
} BLI_ThreadSafePool;


#define DEAD_MAGIC ('d' | ('e' << 8) | ('a' << 16) | ('d' << 24))
#define _SPOOL_MAGIC ('s' | ('p' << 8) | ('o' << 16) | ('l' << 24))

#ifdef DEBUG_SAFEPOOL
#define LIVE_MAGIC ('a' | ('l' << 8) | ('i' << 16) | ('v' << 24))
#define _TAIL_MAGIC1 ((long long)('t' | ('a' << 8) | ('i' << 16) | ('l' << 24)))
#define _TAIL_MAGIC2 ((long long)('1' | ('2' << 8) | ('3' << 16) | ('4' << 24)))
#define TAIL_MAGIC (sizeof(void*) == 8 ? (_TAIL_MAGIC1 | (_TAIL_MAGIC2<<32)) : _TAIL_MAGIC1)
#endif

#ifndef DEBUG_SAFEPOOL
#define bli_safepool_getelem(elem) ((poolelem*) (((char*)(elem)) - sizeof(void*)))
#else
#define bli_safepool_getelem(elem) ((poolelem*) (((char*)(elem)) - sizeof(poolelem)))
#endif


BLI_INLINE void lock_all_threads(BLI_ThreadSafePool* pool) {
  for (int i = 0; i < pool->maxthread; i++) {
    BLI_rw_mutex_lock(&pool->threadchunks[i].lock, THREAD_LOCK_WRITE);
  }
}

BLI_INLINE void unlock_all_threads(BLI_ThreadSafePool* pool) {
  for (int i = 0; i < pool->maxthread; i++) {
    BLI_rw_mutex_unlock(&pool->threadchunks[i].lock);
  }
}

BLI_INLINE bool BLI_safepool_elem_is_dead(void *elem) {
  poolelem *de = bli_safepool_getelem(elem);

  return de->dead_magic == DEAD_MAGIC;
}

BLI_INLINE void BLI_safepool_iternew(struct BLI_ThreadSafePool *pool, ThreadSafePoolIter *iter) {
  lock_all_threads(pool);

  iter->pool = pool;

  iter->thread = 0;
  iter->chunk = pool->threadchunks[0].chunks.first;
  iter->i = 0;

#ifdef BYPASS_POOL
  iter->node = pool->nodes.first;
#endif

  unlock_all_threads(pool);
}

BLI_INLINE void BLI_safepool_iterfree(ThreadSafePoolIter* iter) {
  //unlock_all_threads(iter->pool);
}

BLI_INLINE void* BLI_safepool_iterstep(ThreadSafePoolIter* iter) {
  BLI_ThreadSafePool *pool = iter->pool;

#ifdef BYPASS_POOL
  void *ret = iter->node ? (void*)(((char*)iter->node) + sizeof(void*)*2) : NULL;

  if (iter->node) {
    iter->node = iter->node->next;
  }

  return ret;
#else
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
    iter->i = 0;
  } else if (!chunk) {
    iter->i = -1; //flag end of iteration
  }

  poolelem *de = (poolelem*) ptr;

  if (!ptr || de->dead_magic == DEAD_MAGIC) {
    return BLI_safepool_iterstep(iter);
  }
#ifndef DEBUG_SAFEPOOL
  return (void*) &de->next;
#else
  return (void*) (de + 1);
#endif
#endif
}

#endif /* _BLI_THREADSAFE_MEMPOOL_H */
