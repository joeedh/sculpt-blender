//so, apparently there's no way to make C++ templates generate cdecl functions?
//*
//so autocomplete works
#ifndef MYKEYTYPE
#define MYKEYTYPE void*
#define MYVALTYPE void*
#define MYHASHNAME example
#define MYAPIPREFIX bli_example
#define MYITERNAME bli_example_iter
#endif
//*/

#include <cstdint>
#include <cstdio>

#ifdef APIDECL
#undef APIDECL
#endif

#ifdef KEYPAIR
#undef KEYPAIR
#endif

#define KEYPAIR(a, b) std::pair<MYKEYTYPE, MYVALTYPE>(a, b)

#define _APIJOIN(a, b) a ## b
#define APIDECL(k, b) _APIJOIN(b,k)

#ifndef ITEMS_PER_ITER
#define ITEMS_PER_ITER 32
#endif

typedef parallel_flat_hash_map<MYKEYTYPE, MYVALTYPE> MYHASHNAME;

typedef struct MYITERNAME {
    MYHASHNAME *map;
    int32_t totitem, isdead, i, pad[3]; //i is used by BLI_HASH_ITER macro

    //somewhat worried about alignment issues, so I'm putting the c++ iterator before keys/values
    parallel_flat_hash_map<MYKEYTYPE, MYVALTYPE>::iterator it;
    char cpad[ITER_RESERVE_SIZE - sizeof(parallel_flat_hash_map<MYKEYTYPE, MYVALTYPE>::iterator)];

    MYKEYTYPE keys[ITEMS_PER_ITER];
    MYVALTYPE values[ITEMS_PER_ITER];
} MYITERNAME;

using namespace phmap;
static_assert (sizeof(parallel_flat_hash_map<MYKEYTYPE, MYVALTYPE>::iterator) < ITER_RESERVE_SIZE, "ITER_RESERVE_SIZE is too small");

extern "C" void APIDECL(iternew, MYAPIPREFIX)(MYHASHNAME *map, MYITERNAME *iter) {
    //forcibly zero memory to hopefull avoid garbage getting into any weird
    //iterator destructors
    //void *mem = reinterpret_cast<void*>(iter);
    //memset(mem, 0, sizeof(MYITERNAME));

    iter->it = map->begin();

    iter->i = 0;
    iter->map = map;
    iter->totitem = 0;
    iter->isdead = 0;
}

extern "C" void APIDECL(iterstep, MYAPIPREFIX)(MYHASHNAME *map, MYITERNAME *iter) {
    int i;

    for (i=0; i<ITEMS_PER_ITER; i++) {
        if (iter->it == map->end()) {
            iter->isdead = true;
            break;
        }

        iter->keys[i] = (*iter->it).first;
        iter->values[i] = (*iter->it).second;

        iter->it++;
    }

    iter->totitem = i;
}

extern "C" MYHASHNAME *APIDECL(new, MYAPIPREFIX)() {
    return new MYHASHNAME();
}
extern "C" void APIDECL(free, MYAPIPREFIX)(MYHASHNAME *map) {
    delete map;
}
extern "C" void APIDECL(insert, MYAPIPREFIX)(MYHASHNAME *map, MYKEYTYPE key, MYVALTYPE val) {
    map->insert(KEYPAIR(key, val));
}
extern "C" bool APIDECL(reinsert, MYAPIPREFIX)(MYHASHNAME *map, MYKEYTYPE key, MYVALTYPE val) {
    return map->insert_or_assign(key, val).second;
}
extern "C" MYVALTYPE APIDECL(lookup, MYAPIPREFIX)(MYHASHNAME *map, MYKEYTYPE key) {
    return map->at(key);
}
extern "C" void APIDECL(remove, MYAPIPREFIX)(MYHASHNAME *map, MYKEYTYPE key) {
    map->erase(key);
}

extern "C" bool APIDECL(lookup_p, MYAPIPREFIX)(MYHASHNAME *map, MYKEYTYPE key, MYVALTYPE *out) {
    //auto it = map->find(key)->iterator;
    auto it = map->find(key);

    if (it == map->end()) {
        *out = 0;
        return false;
    } else {
        *out = (*it).second;
        return true;
    }
}

extern "C" bool APIDECL(has, MYAPIPREFIX)(MYHASHNAME *map, MYKEYTYPE key) {
    auto it = map->find(key);

    return it != map->end();
}

extern "C" int APIDECL(len, MYAPIPREFIX)(MYHASHNAME *map) {
    return (int) map->size();
}
