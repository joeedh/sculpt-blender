#include "../../../../extern/parallel-hashmap/phmap.h"
#include <cstdint>

using namespace phmap;

typedef uintptr_t HashP;
typedef int HashInt;
typedef unsigned int HashUint;
typedef char* HashString;

#include <cstdio>
#include <cstring>

//for a seriously dumb hack
#define ITER_RESERVE_SIZE 64

/*
int main() {
    parallel_flat_hash_map<void*, void*> map;
    void *a=0, *b=0;

    parallel_flat_hash_map<void*, void*>::iterator it;
    printf("sizeof(iter): %d\n", (int)sizeof(it));
    return 0;
}*/

#define MYKEYTYPE HashP
#define MYVALTYPE HashP
#define MYHASHNAME BLI_MapHashPHashP
#define MYAPIPREFIX BLI_map_HashPHashP_
#define MYITERNAME HashPHashPIter

#include "hashmap_gen.h"

#undef MYKEYTYPE
#undef MYVALTYPE
#undef MYHASHNAME
#undef MYAPIPREFIX
#undef MYITERNAME

#define MYKEYTYPE HashInt
#define MYVALTYPE HashP
#define MYHASHNAME BLI_MapHashIntHashP
#define MYAPIPREFIX BLI_map_HashIntHashP_
#define MYITERNAME HashIntHashPIter

#include "hashmap_gen.h"

#undef MYKEYTYPE
#undef MYVALTYPE
#undef MYHASHNAME
#undef MYAPIPREFIX
#undef MYITERNAME

#define MYKEYTYPE HashP
#define MYVALTYPE HashInt
#define MYHASHNAME BLI_MapHashPHashInt
#define MYAPIPREFIX BLI_map_HashPHashInt_
#define MYITERNAME HashPHashIntIter

#include "hashmap_gen.h"

#undef MYKEYTYPE
#undef MYVALTYPE
#undef MYHASHNAME
#undef MYAPIPREFIX
#undef MYITERNAME

#define MYKEYTYPE HashInt
#define MYVALTYPE HashInt
#define MYHASHNAME BLI_MapHashIntHashInt
#define MYAPIPREFIX BLI_map_HashIntHashInt_
#define MYITERNAME HashIntHashIntIter

#include "hashmap_gen.h"
