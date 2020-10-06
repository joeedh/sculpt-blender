#include <stdio.h>
#include "../BLI_hashmap.h"
#include <string.h>

int main() {
    printf("testing hash map\n");
    fflush(stdout);

    BLI_HashMap(HashP,HashP) *map = BLI_hashmap_new(HashP,HashP)();

    void *a=NULL, *b=(void*)1234;

    BLI_hashmap_insert(HashP,HashP)(map, a, b);

    printf("test: %s\n", BLI_hashmap_has(HashP,HashP)(map, a) ? "success" : "failure");
    fflush(stdout);

    BLI_hashmap_remove(HashP,HashP)(map, a);

    printf("test: %s\n", !BLI_hashmap_has(HashP,HashP)(map, a) ? "success" : "failure");
    fflush(stdout);

    for (int i=0; i<500; i++) {
        BLI_hashmap_insert(HashP,HashP)(map, (void*)i, (void*)(i+100));
    }

    int bad = true;
    int tot = 0;
    BLI_HashMapIter(HashP, HashP) iter;
    BLI_HASH_ITER(map, iter, HashP, HashP) {
        if ((long long)BLI_hashiter_key(iter) != ((long long)BLI_hashiter_value(iter)) + 100) {
            bad = false;
        }
        tot++;
        //printf(" %Li:%Li", (long long)BLI_hashiter_key(iter), (long long)BLI_hashiter_value(iter));
        //fflush(stdout);
    } BLI_HASH_ITER_END;

    bad = bad || tot != 500;

    if (bad) {
      printf("tot: %d\n", tot);
    }

    printf("test: %s\n", bad ? "failure" : "success");

    return 0;
}
