#include "../../../../extern/parallel-hashmap/phmap.h"
#include <cstdint>

using namespace phmap;

typedef unsigned long long HashP;
typedef int HashInt;
typedef uint HashUint;
typedef char* HashString;

#include <cstdio>
#include <cstring>

//for a seriously dumb hack
#define ITER_RESERVE_SIZE 64

void test() {
    parallel_flat_hash_map<void*, void*> map;
    void *a=0, *b=0;

    parallel_flat_hash_map<void*, void*>::iterator it;
    map.insert(std::pair<void*, void*>(a, b));
}
/*
int main() {
    parallel_flat_hash_map<void*, void*> map;
    void *a=0, *b=0;

    parallel_flat_hash_map<void*, void*>::iterator it;
    printf("sizeof(iter): %d\n", (int)sizeof(it));
    return 0;
}*/

#define BLI_fastmap_new(ktype, vtype) BLI_map_##keytype##valtype##_new

#define MYKEYTYPE HashP
#define MYVALTYPE HashP
#define MYHASHNAME BLI_MapHashPHashP
#define MYAPIPREFIX BLI_map_HashPHashP_
#define MYITERNAME HashPHashPIter

#include "hashmap_gen.h"
