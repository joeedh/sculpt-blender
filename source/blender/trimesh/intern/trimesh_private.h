#include "trimesh.h"

#define TRIVERT(tri, n) ((&(tri)->v1)[n])
#define TRIEDGE(tri, n) ((&(tri)->e1)[n])

#define TRILOOP(tri, n) ((&(tri)->l1)[n])

enum {
  POOL_VERTEX = 0,
  POOL_EDGE = 1,
  POOL_TRI = 2,
  POOL_ELIST = 3,  // pool for lists of edges around vertices
  POOL_TLIST = 4,  // pool for lists of triangles around edges
#ifdef WITH_TRIMESH_CUSTOMDATA
  POOL_LOOP = 5
#endif
};

#define V_ELIST_ESIZE 9
#define E_TLIST_ESIZE 3

struct CustomData;
void trimesh_element_init(void *elem, struct CustomData *customdata, bool skipcd);
void trimesh_element_destroy(void *elem, struct CustomData *customdata);

struct TM_TriMesh;
struct optmesh_simplelist;

static void trimesh_simplelist_remove(struct TM_TriMesh *tm,
                                      struct optmesh_simplelist *list,
                                      void *item,
                                      int pool)
{
  if (list->length == 0) {
    return;
  }

  for (int i = 0; i < list->length; i++) {
    if (list->items[i] == item) {
      // swap with last
      list->items[i] = list->items[list->length - 1];

      /*
      while (i < list->length-1) {
        list->items[i] = list->items[i+1];
        i++;
      }*/

      list->items[list->length - 1] = NULL;
      list->length--;

      return;
    }
  }
}

static void trimesh_simplelist_free(TM_TriMesh *tm, optmesh_simplelist *list, int pool)
{
  if (list->is_pool_allocd) {
    BLI_safepool_free(tm->pools[pool], list->items);
  }
  else {
    MEM_freeN(list->items);
  }
}

static void trilist_simplelist_init(TM_TriMesh *tm, optmesh_simplelist *list, int size, int pool)
{
  list->_size = size;
  list->items = BLI_safepool_alloc(tm->pools[pool]);
  list->length = 0;
  list->is_pool_allocd = true;
}

static void trilist_simplelist_append(TM_TriMesh *tm,
                                      optmesh_simplelist *list,
                                      void *item,
                                      int pool)
{
  list->length++;

  if (list->length > list->_size) {
    if (list->is_pool_allocd) {
      list->is_pool_allocd = false;

      void **items = MEM_mallocN(sizeof(void *) * list->_size * 2, "simplelist_append");
      memcpy(items, list->items, sizeof(void *) * list->_size);

      BLI_safepool_free(tm->pools[pool], list->items);
      list->items = items;
    }
    else {
      list->items = MEM_reallocN(list->items, sizeof(void *) * list->_size * 2);
    }

    list->_size *= 2;
  }

  list->items[list->length - 1] = item;
}

struct CustomData;
struct CustomData *trimesh_get_customdata(TM_TriMesh *tm, int type);
