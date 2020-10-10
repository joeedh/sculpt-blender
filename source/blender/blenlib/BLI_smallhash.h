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

#pragma once

/** \file
 * \ingroup bli
 */

#define USE_SMALLHASH_REMOVE

#include "BLI_utildefines.h"

#include "BLI_compiler_attrs.h"
#include "BLI_compiler_compat.h"
#include "BLI_compiler_typecheck.h"
#include "BLI_assert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uintptr_t key;
  void *val;
} SmallHashEntry;

/* how much stack space to use before dynamically allocating memory.
 * set to match one of the values in 'hashsizes' to avoid too many mallocs  */
#define SMSTACKSIZE 131
typedef struct SmallHash {
  unsigned int nbuckets;
  unsigned int nentries;
  unsigned int cursize;

  SmallHashEntry *buckets;
  SmallHashEntry buckets_stack[SMSTACKSIZE];
} SmallHash;

typedef struct {
  const SmallHash *sh;
  uintptr_t key;
  void *val;
  unsigned int i, done;
} SmallHashIter;


#define SMHASH_KEY_UNUSED ((uintptr_t)(UINTPTR_MAX - 0))
#define SMHASH_CELL_FREE ((void *)(UINTPTR_MAX - 1))
#define SMHASH_CELL_UNUSED ((void *)(UINTPTR_MAX - 2))

/* typically this re-assigns 'h' */
#define SMHASH_NEXT(h, hoff) \
  (CHECK_TYPE_INLINE(&(h), uintptr_t *), \
   CHECK_TYPE_INLINE(&(hoff), uintptr_t *), \
   ((h) + (((hoff) = ((hoff)*2) + 1), (hoff))))

BLI_INLINE int smallhash_val_is_used(const void *val)
{
#ifdef USE_SMALLHASH_REMOVE
  return !ELEM(val, SMHASH_CELL_FREE, SMHASH_CELL_UNUSED);
#else
  return (val != SMHASH_CELL_FREE);
#endif
}

BLI_INLINE uintptr_t smallhash_key(const uintptr_t key)
{
  return key;
}

BLI_INLINE SmallHashEntry *smallhash_lookup(const SmallHash *sh, const uintptr_t key)
{
  SmallHashEntry *e;
  uintptr_t h = smallhash_key(key);
  uintptr_t hoff = 1;

  BLI_assert(key != SMHASH_KEY_UNUSED);

  /* note: there are always more buckets than entries,
  * so we know there will always be a free bucket if the key isn't found. */
  for (e = &sh->buckets[h % sh->nbuckets]; e->val != SMHASH_CELL_FREE;)
  {
    if (e->key == key) {
      /* should never happen because unused keys are zero'd */
      BLI_assert(e->val != SMHASH_CELL_UNUSED);

      return e;
    }

    h = SMHASH_NEXT(h, hoff);
    e = &sh->buckets[h % sh->nbuckets];
  }

  return NULL;
}

BLI_INLINE void *BLI_smallhash_lookup(const SmallHash *sh, uintptr_t key)
{
  SmallHashEntry *e = smallhash_lookup(sh, key);

  return e ? e->val : NULL;
}

BLI_INLINE void **BLI_smallhash_lookup_p(const SmallHash *sh, uintptr_t key)
{
  SmallHashEntry *e = smallhash_lookup(sh, key);

  return e ? &e->val : NULL;
}

BLI_INLINE bool BLI_smallhash_haskey(const SmallHash *sh, uintptr_t key)
{
  SmallHashEntry *e = smallhash_lookup(sh, key);

  return (e != NULL);
}

BLI_INLINE int BLI_smallhash_len(const SmallHash *sh)
{
  return (int)sh->nentries;
}

BLI_INLINE SmallHashEntry *smallhash_iternext(SmallHashIter *iter, uintptr_t *key)
{
  while (iter->i < iter->sh->nbuckets) {
    if (smallhash_val_is_used(iter->sh->buckets[iter->i].val)) {
      SmallHashEntry *e = iter->sh->buckets + iter->i;

      iter->key = e->key;
      iter->val = e->val;

      if (key) {
        *key = iter->key;
      }

      iter->i++;
      return e;
    }

    iter->i++;
  }

  iter->done = 1;
  return NULL;
}

BLI_INLINE void *BLI_smallhash_iternext(SmallHashIter *iter, uintptr_t *key)
{
  SmallHashEntry *e = smallhash_iternext(iter, key);

  return e ? e->val : NULL;
}

BLI_INLINE void **BLI_smallhash_iternext_p(SmallHashIter *iter, uintptr_t *key)
{
  SmallHashEntry *e = smallhash_iternext(iter, key);

  return e ? &e->val : NULL;
}

BLI_INLINE void *BLI_smallhash_iternew(const SmallHash *sh, SmallHashIter *iter, uintptr_t *key)
{
  iter->sh = sh;
  iter->i = 0;
  iter->key = 0;
  iter->val = NULL;
  iter->done = 0;

  return BLI_smallhash_iternext(iter, key);
}
void BLI_smallhash_clear(SmallHash *sh);

BLI_INLINE void smallhash_resize_buckets(SmallHash *sh, const uint nbuckets);
extern const uint BLI_ghash_hash_sizes[];

/**
* Check if the number of items in the smallhash is large enough to require more buckets.
*/
BLI_INLINE bool smallhash_test_expand_buckets(const uint nentries, const uint nbuckets)
{
  /* (approx * 1.5) */
  return (nentries + (nentries >> 1)) > nbuckets;
}

BLI_INLINE SmallHashEntry *smallhash_lookup_first_free(SmallHash *sh, const uintptr_t key)
{
  SmallHashEntry *e;
  uintptr_t h = smallhash_key(key);
  uintptr_t hoff = 1;

  for (e = &sh->buckets[h % sh->nbuckets]; smallhash_val_is_used(e->val);
    h = SMHASH_NEXT(h, hoff), e = &sh->buckets[h % sh->nbuckets]) {
    /* pass */
  }

  return e;
}

BLI_INLINE void smallhash_init_empty(SmallHash *sh)
{
  uint i;

  for (i = 0; i < sh->nbuckets; i++) {
    sh->buckets[i].key = SMHASH_KEY_UNUSED;
    sh->buckets[i].val = SMHASH_CELL_FREE;
  }
}

BLI_INLINE void smallhash_resize_buckets(SmallHash *sh, const uint nbuckets)
{
  SmallHashEntry *buckets_old = sh->buckets;
  const uint nbuckets_old = sh->nbuckets;
  const bool was_alloc = (buckets_old != sh->buckets_stack);
  uint i = 0;

  BLI_assert(sh->nbuckets != nbuckets);
  if (nbuckets <= SMSTACKSIZE) {
    const size_t size = sizeof(*buckets_old) * nbuckets_old;
    buckets_old = alloca(size);
    memcpy(buckets_old, sh->buckets, size);

    sh->buckets = sh->buckets_stack;
  }
  else {
    sh->buckets = MEM_mallocN(sizeof(*sh->buckets) * nbuckets, __func__);
  }

  sh->nbuckets = nbuckets;

  smallhash_init_empty(sh);

  for (i = 0; i < nbuckets_old; i++) {
    if (smallhash_val_is_used(buckets_old[i].val)) {
      SmallHashEntry *e = smallhash_lookup_first_free(sh, buckets_old[i].key);
      e->key = buckets_old[i].key;
      e->val = buckets_old[i].val;
    }
  }

  if (was_alloc) {
    MEM_freeN(buckets_old);
  }
}

BLI_INLINE bool BLI_smallhash_ensure_p(SmallHash *sh, uintptr_t key, void ***item) {
  SmallHashEntry *e;

  if (UNLIKELY(smallhash_test_expand_buckets(++sh->nentries, sh->nbuckets))) {
    smallhash_resize_buckets(sh, BLI_ghash_hash_sizes[++sh->cursize]);
  }

  uintptr_t h = smallhash_key(key);
  uintptr_t hoff = 1;

  e = smallhash_lookup(sh, key);
  //*
  for (e = &sh->buckets[h % sh->nbuckets]; e->key != key && smallhash_val_is_used((void*)e->val);
  h = SMHASH_NEXT(h, hoff), e = &sh->buckets[h % sh->nbuckets]) {  
  }//*/

  bool ret = true;

  if (e->key != key) {
    //e = smallhash_lookup_first_free(sh, key);
    ret = false;
    e->key = key;
  } else {
    e->val = NULL;
  }

  *item = &e->val;
  return ret;
}


BLI_INLINE uintptr_t BLI_smallhash_iterkey(SmallHashIter *iter) {
  return iter->key;
}

BLI_INLINE void *BLI_smallhash_iterval(SmallHashIter *iter) {
  return (void*)iter->val;
}

BLI_INLINE void **BLI_smallhash_iternew_p(const SmallHash *sh, SmallHashIter *iter, uintptr_t *key)
{
  iter->sh = sh;
  iter->i = 0;
  iter->key = 0;
  iter->val = NULL;
  iter->done = 0;

  return BLI_smallhash_iternext_p(iter, key);
}

#define SMALLHASH_ITER(iter, sh)\
for (BLI_smallhash_iternew(sh, &(iter), NULL); !(iter).done; BLI_smallhash_iternext(&(iter), NULL))

void BLI_smallhash_init_ex(SmallHash *sh, const unsigned int nentries_reserve) ATTR_NONNULL(1);
void BLI_smallhash_init(SmallHash *sh) ATTR_NONNULL(1);
void BLI_smallhash_release(SmallHash *sh) ATTR_NONNULL(1);
void BLI_smallhash_insert(SmallHash *sh, uintptr_t key, void *item) ATTR_NONNULL(1);
bool BLI_smallhash_reinsert(SmallHash *sh, uintptr_t key, void *item) ATTR_NONNULL(1);
bool BLI_smallhash_remove(SmallHash *sh, uintptr_t key);
void BLI_smallhash_reserve(SmallHash *sh, uint size);

SmallHash *BLI_smallhash_new();
SmallHash *BLI_smallhash_new_ex(int reserve);
void BLI_smallhash_free(SmallHash *sh);

//void *BLI_smallhash_lookup(const SmallHash *sh, uintptr_t key)
//    ATTR_NONNULL(1) ATTR_WARN_UNUSED_RESULT;
//void **BLI_smallhash_lookup_p(const SmallHash *sh, uintptr_t key)
//    ATTR_NONNULL(1) ATTR_WARN_UNUSED_RESULT;
//bool BLI_smallhash_haskey(const SmallHash *sh, uintptr_t key) ATTR_NONNULL(1);
//int BLI_smallhash_len(const SmallHash *sh) ATTR_NONNULL(1);
/* void BLI_smallhash_print(SmallHash *sh); */ /* UNUSED */

#ifdef DEBUG
double BLI_smallhash_calc_quality(SmallHash *sh);
#endif

#ifdef __cplusplus
}
#endif
