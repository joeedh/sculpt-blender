#ifndef BLI_HASHMAP_H
#define BLI_HASHMAP_H

/*
if only we didn't need fast hash maps. . .
*/

#include <stdint.h>
#include <stdbool.h>

typedef void* HashP;
typedef int32_t HashInt;
typedef uint32_t HashUint;
typedef char* HashString;

#define BLI_hashmap_new(keytype, valtype) BLI_map_##keytype##valtype##_new
#define BLI_hashmap_free(keytype, valtype) BLI_map_##keytype##valtype##_free
#define BLI_hashmap_insert(keytype, valtype) BLI_map_##keytype##valtype##_insert
#define BLI_hashmap_reinsert(keytype, valtype) BLI_map_##keytype##valtype##_reinsert
#define BLI_hashmap_lookup(keytype, valtype) BLI_map_##keytype##valtype##_lookup
#define BLI_hashmap_remove(keytype, valtype) BLI_map_##keytype##valtype##_remove
#define BLI_hashmap_lookup_p(keytype, valtype) BLI_map_##keytype##valtype##_lookup_p
#define BLI_hashmap_has(keytype, valtype) BLI_map_##keytype##valtype##_has
#define BLI_hashmap_iternew(keytype, valtype) BLI_map_##keytype##valtype##_iternew
#define BLI_hashmap_iterstep(keytype, valtype) BLI_map_##keytype##valtype##_iterstep

#define BLI_HashMapIter(keytype, valtype) keytype##valtype##Iter

/*
since there's no way to inline code across the c/c++ barrier, iterators instead
pass data in chunks to amortize function calls and hopefully be more cache efficient
*/
#define BLI_HASH_ITER(map, iter, keytype, valtype) BLI_hashmap_iternew(keytype, valtype)(map, &iter);\
do {\
    BLI_hashmap_iterstep(keytype, valtype)(map, &iter);\
    for (iter.i=0; iter.i<iter.totitem; iter.i++) {

#define BLI_HASH_ITER_END } } while (!iter.isdead);

#define BLI_hashiter_key(iter) iter.keys[iter.i]
#define BLI_hashiter_value(iter) iter.values[iter.i]

#define BLI_HashMap(keytype, valtype) BLI_Map##keytype##valtype

//DO NOT CHANGE THIS, see ITEMS_PER_ITER in hashmap_gen.h
#define _ITEMS_PER_ITER 32
#define ITER_RESERVE_SIZE 64 //keep in sync with version in hashmap.cc code!

#define GEN_API(key, val)\
    typedef struct BLI_Map##key##val BLI_Map##key##val;\
    typedef struct key##val##Iter key##val##Iter;\
    BLI_Map##key##val *BLI_map_##key##val##_new();\
    void BLI_map_##key##val##_free(BLI_Map##key##val *map);\
    void BLI_map_##key##val##_insert(BLI_Map##key##val *map, key k, val v);\
    bool BLI_map_##key##val##_reinsert(BLI_Map##key##val *map, key k, val v);\
    void BLI_map_##key##val##_remove(BLI_Map##key##val *map, key k);\
    val BLI_map_##key##val##_lookup(BLI_Map##key##val *map, key k);\
    bool BLI_map_##key##val##_lookup_p(BLI_Map##key##val *map, key k, val *out);\
    bool BLI_map_##key##val##_has(BLI_Map##key##val *map, key k);\
    typedef struct key##val##Iter {\
        BLI_Map##key##val *map;\
        int32_t totitem, isdead, i, pad[3];\
        char reserved[ITER_RESERVE_SIZE];\
        key *keys[_ITEMS_PER_ITER];\
        val *values[_ITEMS_PER_ITER];\
    } key##val##Iter;\
    void BLI_map_##key##val##_iternew(BLI_Map##key##val *map, key##val##Iter *iter);\
    void BLI_map_##key##val##_iterstep(BLI_Map##key##val *map, key##val##Iter *iter);\

GEN_API(HashP, HashP)
GEN_API(HashInt, HashP)
GEN_API(HashP, HashInt)
GEN_API(HashInt, HashInt)

#endif // BLI_HASHMAP_H
