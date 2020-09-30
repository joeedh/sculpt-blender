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

struct BLI_ThreadSafePool;
typedef struct BLI_ThreadSafePool BLI_ThreadSafePool;

struct BLI_ThreadSafePool* BLI_safepool_create(int elemsize, int chunksize, int maxthread);
void* BLI_safepool_alloc(struct BLI_ThreadSafePool *pool);

void BLI_safepool_free(struct BLI_ThreadSafePool*pool, void *elem);
int BLI_safepool_elem_is_dead(void *elem);

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
} ThreadSafePoolIter;

void BLI_safepool_iternew(struct BLI_ThreadSafePool* pool, ThreadSafePoolIter* iter);
void BLI_safepool_iterfree(ThreadSafePoolIter* iter);
void* BLI_safepool_iterstep(ThreadSafePoolIter* iter);
void BLI_safepool_threaded_free(struct BLI_ThreadSafePool* pool, void* elem, int thread);

/*not sure how to pass thread number to customdata functions, so this evilness here
is used for now*/
void BLI_safepool_threadnr_set(int threadnr);

#endif /* _BLI_THREADSAFE_MEMPOOL_H */

