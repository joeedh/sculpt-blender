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

/** \file
 * \ingroup bli
 *
 * A light stack-friendly hash library, it uses stack space for relatively small,
 * fixed size hash tables but falls back to heap memory once the stack limits reached
 * (#SMSTACKSIZE).
 *
 * based on a doubling hashing approach (non-chaining) which uses more buckets then entries
 * stepping over buckets when two keys share the same hash so any key can find a free bucket.
 *
 * See: https://en.wikipedia.org/wiki/Double_hashing
 *
 * \warning This should _only_ be used for small hashes
 * where allocating a hash every time is unacceptable.
 * Otherwise #GHash should be used instead.
 *
 * #SmallHashEntry.key
 * - ``SMHASH_KEY_UNUSED`` means the key in the cell has not been initialized.
 *
 * #SmallHashEntry.val
 * - ``SMHASH_CELL_UNUSED`` means this cell is inside a key series.
 * - ``SMHASH_CELL_FREE`` means this cell terminates a key series.
 *
 * Note that the values and keys are often pointers or index values,
 * use the maximum values to avoid real pointers colliding with magic numbers.
 */

#include <stdlib.h>
#include <string.h>

#include "BLI_sys_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"

#include "BLI_smallhash.h"
#include "BLI_strict_flags.h"

/* nothing uses BLI_smallhash_remove yet */


extern const uint BLI_ghash_hash_sizes[];
#define hashsizes BLI_ghash_hash_sizes

/**
 * Increase initial bucket size to match a reserved amount.
 */
BLI_INLINE void smallhash_buckets_reserve(SmallHash *sh, const uint nentries_reserve)
{
  while (smallhash_test_expand_buckets(nentries_reserve, hashsizes[++sh->cursize])) {
    sh->nbuckets = hashsizes[++sh->cursize];
  }
}


void BLI_smallhash_init_ex(SmallHash *sh, const uint nentries_reserve)
{
  /* assume 'sh' is uninitialized */

  sh->nentries = 0;
  sh->cursize = 2;
  sh->nbuckets = hashsizes[sh->cursize];

  sh->buckets = sh->buckets_stack;

  if (nentries_reserve) {
    smallhash_buckets_reserve(sh, nentries_reserve);

    if (sh->nbuckets > SMSTACKSIZE) {
      sh->buckets = MEM_mallocN(sizeof(*sh->buckets) * sh->nbuckets, __func__);
    }
  }

  smallhash_init_empty(sh);
}

void BLI_smallhash_init(SmallHash *sh)
{
  BLI_smallhash_init_ex(sh, 0);
}

/* NOTE: does *not* free *sh itself!  only the direct data! */
void BLI_smallhash_release(SmallHash *sh)
{
  if (sh->buckets != sh->buckets_stack) {
    MEM_freeN(sh->buckets);
  }
}

void BLI_smallhash_clear(SmallHash *sh) {
  unsigned int i;
  SmallHashEntry *entry = sh->buckets;

  for (i=0; i<sh->nbuckets; i++, entry++) {
    entry->key = SMHASH_KEY_UNUSED;
    entry->val = SMHASH_CELL_FREE;
  }

  sh->nentries = 0;
}

void BLI_smallhash_reserve(SmallHash *sh, uint size) {
  int cursize = sh->cursize;

  while (smallhash_test_expand_buckets(size, hashsizes[cursize])) {
    cursize++;
  }

  sh->cursize = cursize;
  smallhash_resize_buckets(sh, hashsizes[cursize]);
}

void BLI_smallhash_insert(SmallHash *sh, uintptr_t key, void *item)
{
  SmallHashEntry *e;

  BLI_assert(key != SMHASH_KEY_UNUSED);
  BLI_assert(smallhash_val_is_used(item));
  BLI_assert(BLI_smallhash_haskey(sh, key) == false);

  if (UNLIKELY(smallhash_test_expand_buckets(++sh->nentries, sh->nbuckets))) {
    smallhash_resize_buckets(sh, hashsizes[++sh->cursize]);
  }

  e = smallhash_lookup_first_free(sh, key);
  e->key = key;
  e->val = item;
}


/**
 * Inserts a new value to a key that may already be in ghash.
 *
 * Avoids #BLI_smallhash_remove, #BLI_smallhash_insert calls (double lookups)
 *
 * \returns true if a new key has been added.
 */
bool BLI_smallhash_reinsert(SmallHash *sh, uintptr_t key, void *item)
{
  SmallHashEntry *e = smallhash_lookup(sh, key);
  if (e) {
    e->val = item;
    return false;
  }

  BLI_smallhash_insert(sh, key, item);
  return true;
}

#ifdef USE_SMALLHASH_REMOVE
bool BLI_smallhash_remove(SmallHash *sh, uintptr_t key)
{
  SmallHashEntry *e = smallhash_lookup(sh, key);

  if (e) {
    e->key = SMHASH_KEY_UNUSED;
    e->val = SMHASH_CELL_UNUSED;
    sh->nentries--;

    return true;
  }
  else {
    return false;
  }
}
#endif



/** \name Debugging & Introspection
 * \{ */

/* note, this was called _print_smhash in knifetool.c
 * it may not be intended for general use - campbell */
#if 0
void BLI_smallhash_print(SmallHash *sh)
{
  uint i, linecol = 79, c = 0;

  printf("{");
  for (i = 0; i < sh->nbuckets; i++) {
    if (sh->buckets[i].val == SMHASH_CELL_UNUSED) {
      printf("--u-");
    }
    else if (sh->buckets[i].val == SMHASH_CELL_FREE) {
      printf("--f-");
    }
    else {
      printf("%2x", (uint)sh->buckets[i].key);
    }

    if (i != sh->nbuckets - 1) {
      printf(", ");
    }

    c += 6;

    if (c >= linecol) {
      printf("\n ");
      c = 0;
    }
  }

  fflush(stdout);
}
#endif

#ifdef DEBUG
/**
 * Measure how well the hash function performs
 * (1.0 is perfect - no stepping needed).
 *
 * Smaller is better!
 */
double BLI_smallhash_calc_quality(SmallHash *sh)
{
  uint64_t sum = 0;
  uint i;

  if (sh->nentries == 0) {
    return -1.0;
  }

  for (i = 0; i < sh->nbuckets; i++) {
    if (sh->buckets[i].key != SMHASH_KEY_UNUSED) {
      uint64_t count = 0;
      SmallHashEntry *e, *e_final = &sh->buckets[i];
      uintptr_t h = smallhash_key(e_final->key);
      uintptr_t hoff = 1;

      for (e = &sh->buckets[h % sh->nbuckets]; e != e_final;
           h = SMHASH_NEXT(h, hoff), e = &sh->buckets[h % sh->nbuckets]) {
        count += 1;
      }

      sum += count;
    }
  }
  return ((double)(sh->nentries + sum) / (double)sh->nentries);
}
#endif

/** \} */
SmallHash *BLI_smallhash_new() {
  SmallHash *sh = MEM_callocN(sizeof(SmallHash), "SmallHash");
  BLI_smallhash_init(sh);
  return sh;
}

SmallHash *BLI_smallhash_new_ex(int reserve) {
  SmallHash *sh = MEM_callocN(sizeof(SmallHash), "SmallHash");
  BLI_smallhash_init_ex(sh, reserve);
  return sh;
}

void BLI_smallhash_free(SmallHash *sh) {
  BLI_smallhash_release(sh);
  MEM_freeN(sh);
}
