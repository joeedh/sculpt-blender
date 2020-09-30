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
 * The Original Code is Copyright (C) 2008 by Blender Foundation.
 * All rights reserved.
 */

 /** \file
  * \ingroup bli
  *
  * optimized thread-safe triangle mesh library with topological info
  *
  */

#include <stdlib.h>
#include <string.h>

#include "BLI_listbase.h"

#include "BLI_math.h"
#include "BLI_trimesh.h"
#include "BLI_threadsafe_mempool.h"
#include "BLI_array.h"

#include "../blenkernel/BKE_customdata.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BLI_utildefines.h"
#include "atomic_ops.h"

#include "../blenkernel/BKE_customdata.h"
#include "../blenkernel/BKE_mesh.h"
#include "../blenkernel/BKE_mesh_runtime.h"
#include "../blenkernel/BKE_multires.h"

#include "../blenkernel/BKE_key.h"
#include "../blenkernel/BKE_main.h"

#include "MEM_guardedalloc.h"
#ifdef WITH_MEM_VALGRIND
#  include "valgrind/memcheck.h"
#endif


#define TRIVERT(tri, n) ((&(tri)->v1)[n])
#define TRIEDGE(tri, n) ((&(tri)->e1)[n])

#define TRILOOP(tri, n) ((&(tri)->l1)[n])

enum {
  TM_VERTEX = 1,
  TM_EDGE = 2,
  TM_LOOP = 4,
  TM_TRI = 8
};

enum {
POOL_VERTEX = 0,
POOL_EDGE = 1,
//POOL_LOOP = 2,
POOL_TRI = 2,
POOL_ELIST = 3, //pool for lists of edges around vertices
POOL_TLIST = 4, //pool for lists of triangles around edges
#ifdef WITH_TRIMESH_CUSTOMDATA
POOL_LOOP = 5
#endif
};

#define V_ELIST_ESIZE 5
#define E_TLIST_ESIZE 2

//keep up to sync with MAX_TRIMESH_POOLS in BLI_trimesh.h
static int poolsizes[] = {
  sizeof(TMVert),
  sizeof(TMEdge),
  sizeof(TMFace),
  sizeof(void*)*V_ELIST_ESIZE,
  sizeof(void*)*E_TLIST_ESIZE,
#ifdef WITH_TRIMESH_CUSTOMDATA
  sizeof(TMLoopData)
#endif
};

#ifdef WITH_TRIMESH_CUSTOMDATA
void trimesh_element_init(void *elem, CustomData *customdata) {
#else
void trimesh_element_init(void *elem) {
#endif

#ifdef WITH_TRIMESH_CUSTOMDATA
  TMElement *e = elem;

  CustomData_bmesh_set_default(customdata, &e->customdata);
#endif
}

#ifdef WITH_TRIMESH_CUSTOMDATA
void trimesh_element_destroy(void *elem, int threadnr, CustomData *customdata) {
#else
void trimesh_element_destroy(void *elem, int threadnr) {
#endif

#ifdef WITH_TRIMESH_CUSTOMDATA
  TMElement *e = elem;

  CustomData_bmesh_free_block(customdata, &e->customdata);
#endif
}

#ifdef WITH_TRIMESH_CUSTOMDATA
static TMLoopData *trimesh_make_loop(BLI_TriMesh *tm, int threadnr) {
  TMLoopData *loop = BLI_safepool_alloc(tm->pools[POOL_LOOP], threadnr);
  trimesh_element_init(loop, &tm->ldata);

  return loop;
}

static void trimesh_kill_loop(BLI_TriMesh *tm, TMLoopData *l, int threadnr) {
  trimesh_element_destroy(l, threadnr, &tm->ldata);
}
#endif

BLI_TriMesh* BLI_trimesh_new(int maxthread) {
  BLI_TriMesh* tm = MEM_callocN(sizeof(*tm), "OptTriMesh");
  int i;

  for (i = 0; i < MAX_TRIMESH_POOLS; i++) {
    tm->pools[i] = BLI_safepool_create(poolsizes[i], 0, maxthread);
  }

  return tm;
}


static void simplelist_remove(BLI_TriMesh *tm, optmesh_simplelist *list, void *item, int pool, int threadnr) {
  if (list->length == 0) {
    return;
  }

  for (int i=0; i<list->length; i++) {
    if (list->items[i] == item) {
      while (i < list->length-1) {
        list->items[i] = list->items[i+1];
      }

      list->items[list->length-1] = NULL;
      list->length--;

      return;
    }
  }
}

static void simplelist_free(BLI_TriMesh *tm, optmesh_simplelist *list, int pool, int threadnr) {
  if (list->is_pool_allocd) {
    BLI_safepool_free(tm->pools[pool], list->items);
  } else {
    MEM_freeN(list->items);
  }
}

static void simplelist_append(BLI_TriMesh* tm, optmesh_simplelist* list, void *item, int pool, int threadnr) {
  list->length++;

  if (list->length > list->_size) {
    if (list->is_pool_allocd) {
      list->is_pool_allocd = false;

      void **items = MEM_mallocN(sizeof(void*)*list->_size*2, "simplelist_append");
      memcpy(items, list->items, sizeof(void*)*list->_size);

      BLI_safepool_threaded_free(tm->pools[pool], list->items, threadnr);
      list->items = items;
    } else {
      list->items = MEM_reallocN(list->items, sizeof(void*)*list->_size*2);
    }

    list->_size *= 2;
  }

  list->items[list->length-1] = item;
}

#define OTHER_VERT(e, v) ((v) == (e)->v1 ? (e)->v2 : (e)->v1)

static TMEdge *ensure_edge(BLI_TriMesh* tm, TMVert* v1, TMVert* v2, int threadnr, bool skipcd) {
  for (int i = 0; i < v1->edges.length; i++) {
    TMEdge *e = v1->edges.items[i];

    if (OTHER_VERT(e, v1) == v2) {
      return e;
    }
  }

  TMEdge *e = BLI_safepool_alloc(tm->pools[POOL_EDGE], threadnr);
  memset(e, 0, sizeof(*e));

  e->v1 = v1;
  e->v2 = v2;

  simplelist_append(tm, &e->v1->edges, e, POOL_ELIST, threadnr);
  simplelist_append(tm, &e->v2->edges, e, POOL_ELIST, threadnr);

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_element_init(e, &tm->edata);
#else
  trimesh_element_init(e);
#endif

  return e;
}

void BLI_trimesh_vert_iternew(BLI_TriMesh *tm, BLI_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_VERTEX;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter->iter);
}

void BLI_trimesh_edge_iternew(BLI_TriMesh *tm, BLI_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_EDGE;

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter->iter);
}

void BLI_trimesh_tri_iternew(BLI_TriMesh *tm, BLI_TriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_TRI;

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter->iter);
}

void *BLI_trimesh_iterstep(BLI_TriMeshIter* iter) {
  return BLI_safepool_iterstep(&iter->iter);
}

static TMEdge *edge_add_tri(BLI_TriMesh* tm, TMVert* v1, TMVert* v2, TMFace* tri, int threadnr, bool skipcd) {
  TMEdge *e = ensure_edge(tm, v1, v2, threadnr, skipcd);
  simplelist_append(tm, &e->tris, tri, POOL_TLIST, threadnr);

  return e;
}

TMVert *BLI_trimesh_make_vert(BLI_TriMesh *tm, float co[3], float no[3], int threadnr, bool skipcd) {
  TMVert *v = BLI_safepool_alloc(tm->pools[POOL_VERTEX], threadnr);

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_element_init(v, &tm->vdata);
#else
  trimesh_element_init(v);
#endif

  memset(v, 0, sizeof(*v));

  copy_v3_v3(v->co, co);
  copy_v3_v3(v->no, no);
}

TMEdge *BLI_trimesh_get_edge(BLI_TriMesh *tm, TMVert *v1, TMVert *v2, int threadnr, bool skipcd) {
  return ensure_edge(tm, v1, v2, threadnr, skipcd);
}

TMFace *BLI_trimesh_make_tri(BLI_TriMesh *tm, TMVert *v1, TMVert *v2, TMVert *v3, int threadnr, bool skipcd) {
  TMFace *tri = BLI_safepool_alloc(tm->pools[POOL_TRI], threadnr);

  memset(tri, 0, sizeof(*tri));

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_element_init(tri, &tm->tdata);
#else
  trimesh_element_init(tri);
#endif

#ifdef WITH_TRIMESH_CUSTOMDATA
  tri->l1 = trimesh_make_loop(tm, threadnr, skipcd);
  tri->l2 = trimesh_make_loop(tm, threadnr, skipcd);
  tri->l3 = trimesh_make_loop(tm, threadnr, skipcd);
#endif

  tri->v1 = v1;
  tri->v2 = v2;
  tri->v3 = v3;

  tri->e1 = edge_add_tri(tm, v1, v2, tri, threadnr, skipcd);
  tri->e2 = edge_add_tri(tm, v2, v3, tri, threadnr, skipcd);
  tri->e3 = edge_add_tri(tm, v3, v1, tri, threadnr, skipcd);

  return tri;
}

void BLI_trimesh_add(BLI_TriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr, bool skipcd) {
  float* vco, * vno;
  BLI_ThreadSafePool *vpool = tm->pools[POOL_VERTEX];
  BLI_ThreadSafePool *fpool = tm->pools[POOL_EDGE];
  BLI_ThreadSafePool *epool = tm->pools[POOL_TRI];
  int i;

  //abuse the normals array to store pointers?
  //or just allocate one?

  TMVert** vmap = MEM_mallocN(sizeof(*vmap) * totvert, "BLI_trimesh_add:vmap temporary");
  //OptTriVertex** vmap = (OptTriVertex**)vertNos;

  vco = vertCos;
  vno = vertNos;

  for (i = 0; i < totvert; i++, vco += 3, vno += 3) {
    TMVert *v = BLI_trimesh_make_vert(tm, vco, vno, threadnr, skipcd);
    vmap[i] = v;
  }

  int *tris = triIndices;
  for (i = 0; i < tottri; triIndices += 3) {
    TMVert *v1 = vmap[triIndices[0]];
    TMVert *v2 = vmap[triIndices[1]];
    TMVert *v3 = vmap[triIndices[2]];

    TMFace *tri = BLI_trimesh_make_tri(tm, v1, v2, v3, threadnr, skipcd);
  }

  MEM_freeN(vmap);
}


//we do somewhat weird things with stack, it's returned by this function
static void **trimesh_tag_step(BLI_TriMesh* tm, TMVert* v, void** stack, int tag, int maxelem) {
  BLI_array_declare(stack);

  v->threadtag = tag;
  int totelem = 0;

  for (int i = 0; i < v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];

    for (int j = 0; j < e->tris.length; j++) {
      TMFace *tri = e->tris.items[j];

      if (tri->threadtag == TRIMESH_NEED_TAG) {
        tri->threadtag = tag;
        BLI_array_append(stack, tri);
        totelem++;
      }
    }
  }

  int len = BLI_array_len(stack);
  for (; len && totelem < maxelem; len = BLI_array_len(stack)) {
    TMFace *tri = BLI_array_pop(stack);

    for (int i = 0; i < 3; i++) {
      TMEdge *e = i == 0 ? tri->e1 : (i == 1 ? tri->e2 : tri->e3);

      for (int j = 0; j < e->tris.length; j++) {
        TMFace *tri2 = e->tris.items[j];

        if (tri2->threadtag == TRIMESH_NEED_TAG) {
          tri2->threadtag = tag;
          totelem++;
          BLI_array_append(stack, tri2);
        }
      }
    }
  }

  return stack;
}

//if tottris is -1 then all triangles will be tagged
void BLI_trimesh_thread_tag(BLI_TriMesh *tm, TMFace** tris, int tottri) {
  void **stack = NULL;
  BLI_array_declare(stack);

  if (tottri == -1) {
    int maxtag = MAX2(tm->tottri / tm->maxthread, 1);

    ThreadSafePoolIter iter;

    BLI_safepool_iternew(tm->pools[POOL_TRI], &iter);
    TMFace *t = BLI_safepool_iterstep(&iter);
    for (; t; t = BLI_safepool_iterstep(&iter)) {
      t->threadtag = TRIMESH_NEED_TAG;
    }
    BLI_safepool_iterfree(&iter);

    bool stop = false;

    int tag = 0;

    while (!stop) {
      stop = true;

      BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter);
      TMVert *v = BLI_safepool_iterstep(&iter);

      for (; v; v = BLI_safepool_iterstep(&iter)) {
        if (v->threadtag == TRIMESH_NEED_TAG) {
          stop = false;
          v->threadtag = tag;

          stack = trimesh_tag_step(tm, v, stack, tag, maxtag);
          tag = (tag + 1) % tm->maxthread;
        }
      }
    }

    BLI_array_free(stack);

    return;
  }

  int maxelem = tottri;
  int maxtag = tm->maxthread;
  int tag = 0;
  maxelem = MAX2(maxelem, 1);

  for (int i = 0; i < tottri; i++) {
    TMFace *tri = tris[i];

    tri->threadtag = TRIMESH_NEED_TAG;
    tri->v1->threadtag = TRIMESH_NEED_TAG;
    tri->v2->threadtag = TRIMESH_NEED_TAG;
    tri->v3->threadtag = TRIMESH_NEED_TAG;
  }

  bool stop = false;
  while (1) {
    stop = true;

    for (int i = 0; i < tottri; i++) {
      TMFace *tri = tris[i];

      if (tri->threadtag != TRIMESH_NEED_TAG) {
        continue;
      }

      for (int j = 0; j < 3; j++) {
        TMVert *v = (&tri->v1)[j];

        if (v->threadtag == TRIMESH_NEED_TAG) {
          stack = trimesh_tag_step(tm, v, stack, tag, maxelem);
          tag = (tag + 1) % maxtag;
        }
      }
    }
  }

  BLI_array_free(stack);
}

void BLI_trimesh_clear_threadtags(BLI_TriMesh *tm) {
  for (int i=0; i<3; i++) {
    ThreadSafePoolIter iter;
    TMElement *item;

    BLI_safepool_iternew(tm->pools[i], &iter);
    item = BLI_safepool_iterstep(&iter);

    for (; item; item = BLI_safepool_iterstep(&iter)) {
      item->threadtag = TRIMESH_TAG_CLEAR;
    }
  }
}

void BLI_trimesh_tag_thread_boundaries(BLI_TriMesh *tm, TMFace **tris, int tottri) {
  //propegate boundary tag twice
  for (int i=0; i<2; i++) {
    BLI_trimesh_tag_thread_boundaries_once(tm, tris, tottri);

    //needed to avoid triggering double tagging detection code
    for (int i=0; i<tottri; i++) {
      tris[i]->threadtag = TRIMESH_BOUNDARY_TEMP;
    }
  }

  for (int i=0; i<tottri; i++) {
    if (tris[i]->threadtag == TRIMESH_BOUNDARY_TEMP) {
      tris[i]->threadtag = TRIMESH_BOUNDARY;
    }
  }
}

void BLI_trimesh_tag_thread_boundaries_once(BLI_TriMesh *tm, TMFace **tris, int tottri) {
  for (int i = 0; i < tottri; i++) {
    TMFace *tri = tris[i];

    //avoid double tagging
    if (tri->threadtag == TRIMESH_BOUNDARY) {
      continue;
    }

    for (int j = 0; j < 3; j++) {
      TMEdge *e = TRIEDGE(tri, j);
      for (int k = 0; k < e->tris.length; k++) {
        TMFace *tri2 = e->tris.items[k];

        if (tri2->threadtag != TRIMESH_TAG_CLEAR && tri2->threadtag != tri->threadtag) {
          tri2->threadtag = TRIMESH_BOUNDARY;
          tri->threadtag = TRIMESH_BOUNDARY;
        }
      }
    }
  }
}

//called after BLI_trimesh_thread_tag
//last island is always boundary triangles
void BLI_trimesh_build_islands(BLI_TriMesh *tm, TMFace **tris, int tottri, OptTriIsland** r_islands, int *r_totisland) {
  OptTriIsland *islands = *r_islands = MEM_callocN((tm->maxthread+1)*sizeof(*islands), "OptTriIsland");

  *r_totisland = tm->maxthread+1;

  for (int i = 0; i < tottri; i++) {
    TMFace *tri = tris[i];
    int threadnr = tri->threadtag;

    if (threadnr == TRIMESH_BOUNDARY) {
      threadnr = tm->maxthread;
    } else if (threadnr < 0 || threadnr >= tm->maxthread) {
      fprintf(stderr, "Bad thread tag\n");
      continue;
    }

    OptTriIsland *island = islands + threadnr;

    TMFace **list = island->tris;
    BLI_array_declare(list);

    BLI_array_append(list, tri);

    island->tris = list;
    island->tottri = BLI_array_len(list);
  }

  r_totisland = tm->maxthread;
}

void BLI_trimesh_free_islands(OptTriIsland* islands, int totisland, bool free_islands) {
  for (int i = 0; i < totisland; i++) {
    if (islands[i].tris) {
      MEM_freeN(islands[i].tris);
    }

    if (islands[i].verts) {
      MEM_freeN(islands[i].verts);
    }
  }

  if (free_islands) {
    MEM_freeN(islands);
  }
}

CustomData *get_customdata(BLI_TriMesh *tm, int type) {
  switch (type) {
  case TM_VERTEX:
    return &tm->vdata;
  case TM_EDGE:
    return &tm->edata;
  case TM_LOOP:
    return &tm->ldata;
  case TM_TRI:
    return &tm->tdata;
  }
}

typedef struct threadjob {
  BLI_TriMesh *tm;
  OptTriMeshJob job;
  void **elems;
  int totelem;
  void *userdata;
  bool done;
} threadjob;

typedef struct meshthread {
  ThreadQueue *queue;
  int threadnr;
} meshthread;

static void thread_job(meshthread *thread) {
  while (!BLI_thread_queue_is_empty(thread->queue)) {
    threadjob *job = BLI_thread_queue_pop(thread->queue);

    job->job(job->tm, job->elems, job->totelem, thread->threadnr, job->userdata);
    job->done = true;
  }
}

void BLI_trimesh_foreach_tris(BLI_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata) {
  tm->maxthread = maxthread;

  OptTriIsland *islands;
  int totisland;

  BLI_trimesh_thread_tag(tm, tris, tottri);
  BLI_trimesh_build_islands(tm, tris, tottri, &islands, &totisland);
  BLI_trimesh_tag_thread_boundaries(tm, tris, tottri);

  meshthread *threads = MEM_callocN(sizeof(meshthread)*maxthread, "meshthread");
  threadjob *jobs = MEM_callocN(sizeof(threadjob)*totisland, "threadjob");
  ThreadQueue *queue = BLI_thread_queue_init();

  for (int i=0; i<totisland; i++) {
    islands[i].tag = 0;
  }

  for (int i=0; i<maxthread; i++) {
    threads[i].queue = queue;
    threads[i].threadnr = i;
  }

  ListBase threadpool = {NULL, NULL};

  for (int i=0; i<totisland; i++) {
    jobs[i].elems = islands[i].tris;
    jobs[i].totelem = islands[i].tottri;
    jobs[i].userdata = userdata;

    jobs[i].job = job;
    jobs[i].done = false;
    jobs[i].tm = tm;

    //save boundary triangles for main thread
    if (i < totisland - 1) {
      BLI_thread_queue_push(queue, jobs + i);
    }
  }

  BLI_threadpool_init(&threadpool, thread_job, maxthread);

  for (int i=0; i<maxthread; i++) {
    BLI_threadpool_insert(&threadpool, threads+i);
  }

  BLI_thread_queue_wait_finish(queue);
  BLI_thread_queue_free(queue);

  BLI_threadpool_end(&threadpool);

  //do triangles on thread island boundaries last
  if (totisland > 0) {
    OptTriIsland *island = islands + totisland - 1;

    job(tm, island->tris, island->tottri, 0, userdata);
  }

  MEM_freeN(jobs);
  BLI_trimesh_free_islands(islands, totisland, true);

  for (int i=0; i<tottri; i++) {
    tris[i]->threadtag = TRIMESH_TAG_CLEAR;
  }
}

typedef struct foreach_vert_data {
  OptTriMeshJob job;
  void *userdata;
} foreach_vert_data;

static void for_vert_callback(BLI_TriMesh *tm, TMFace **tris, int tottri, int threadnr, foreach_vert_data *userdata) {
  TMVert **verts = NULL;
  BLI_array_declare(verts);

  for (int i=0; i<tottri; i++) {
    tris[i]->v1->threadtag = TRIMESH_TAG_CLEAR;
    tris[i]->v2->threadtag = TRIMESH_TAG_CLEAR;
    tris[i]->v3->threadtag = TRIMESH_TAG_CLEAR;
  }

  for (int i=0; i<tottri; i++) {
    TMFace *tri = tris[i];

    for (int j=0; j<3; j++) {
      TMVert *v = TRIVERT(tri, j);

      if (v->threadtag == TRIMESH_TAG_CLEAR) {
        v->threadtag = 1;

        BLI_array_append(verts, v);
      }
    }
  }


  if (BLI_array_len(verts) != 0) {
    userdata->job(tm, verts, BLI_array_len(verts), threadnr, userdata->userdata);
  }
}

void BLI_trimesh_foreach_verts(BLI_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata) {
  foreach_vert_data data;

  data.job = job;
  data.userdata = userdata;

  //void BLI_trimehs_foreach_tris(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata) {
  BLI_trimesh_foreach_tris(tm, tris, tottri, for_vert_callback, maxthread, &data); 
}

void BLI_trimesh_kill_vert(BLI_TriMesh *tm, TMVert *v, int threadnr) {
  while (v->edges.length > 0) {
    BLI_trimesh_kill_edge(tm, v->edges.items[0], threadnr, false);
  }

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_element_destroy(tm, v, &tm->vdata);
#else
  trimesh_element_destroy(tm, v);
#endif

  simplelist_free(tm, &v->edges, POOL_ELIST, threadnr);
  BLI_safepool_free(tm->pools[POOL_VERTEX], v);
}

//if kill_verts is true verts with no edges will be deleted
void BLI_trimesh_kill_edge(BLI_TriMesh *tm, TMEdge *e, int threadnr, bool kill_verts) {
  while (e->tris.length > 0) {
    TMFace *tri = e->tris.items[0];
    BLI_trimesh_kill_tri(tm, tri, threadnr, false, false);
  }

  simplelist_remove(tm, &e->v1->edges, e, POOL_ELIST, threadnr);
  simplelist_remove(tm, &e->v2->edges, e, POOL_ELIST, threadnr);

  if (kill_verts) {
    if (e->v1->edges.length == 0) {
      BLI_trimesh_kill_vert(tm, e->v1, threadnr);
    }

    if (e->v2->edges.length == 0) {
      BLI_trimesh_kill_vert(tm, e->v2, threadnr);
    }
  }

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_element_destroy(tm, e, &tm->edata);
#else
  trimesh_element_destroy(tm, e);
#endif

  simplelist_free(tm, &e->tris, POOL_TLIST, threadnr);
  BLI_safepool_free(tm->pools[POOL_EDGE], e);
}

//kill_edges/verts is whether to automatically kill verts/edges that belong to no triangles
//note that threadnr doesn't refer to whichever thread created tri, but the calling thread
void BLI_trimesh_kill_tri(BLI_TriMesh *tm, TMFace *tri, int threadnr, bool kill_edges, bool kill_verts) {
  //static void simplelist_remove(OptTriMesh *tm, optmesh_simplelist *list, void *item, int pool, int threadnr) {

  simplelist_remove(tm, &tri->e1->tris, tri, POOL_TLIST, threadnr);
  simplelist_remove(tm, &tri->e2->tris, tri, POOL_TLIST, threadnr);
  simplelist_remove(tm, &tri->e3->tris, tri, POOL_TLIST, threadnr);

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_kill_loop(tm, tri->l1, threadnr);
  trimesh_kill_loop(tm, tri->l2, threadnr);
  trimesh_kill_loop(tm, tri->l3, threadnr);
#endif

  if (tri->e1->tris.length == 0) {
    BLI_trimesh_kill_edge(tm, tri->e1, threadnr, kill_verts);
  }
  if (tri->e2->tris.length == 0) {
    BLI_trimesh_kill_edge(tm, tri->e2, threadnr, kill_verts);
  }
  if (tri->e3->tris.length == 0) {
    BLI_trimesh_kill_edge(tm, tri->e3, threadnr, kill_verts);
  }


#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_element_destroy(tm, tri, &tm->tdata);
#else
  trimesh_element_destroy(tm, tri);
#endif

  BLI_safepool_free(tm->pools[POOL_TRI], tri);
}

static void weld_verts(BLI_TriMesh *tm, TMVert *v1, TMVert *v2, int threadnr) {
  for (int i=0; i<v2->edges.length; i++) {
    TMEdge *e = v2->edges.items[i];
    TMVert *vb = OTHER_VERT(e, v1);

    if (vb == v1) {
      //not sure this will ever happen; force deletion of edge?
      fprintf(stdout, "auto deleting edge in BLI_trimesh code\n");
      fflush(stdout);

      BLI_trimesh_kill_edge(tm, e, threadnr, false);

      continue;
    }

    //swap out vertex references for triangles
    for (int j=0; j<e->tris.length; j++) {
      TMFace *tri = e->tris.items[j];
      int tot = 0;

      //make sure we delete triangles that end up with duplicate verts
      if (tri->v1 == v2) {
        tri->v1 = v1;
        tot++;
      }

      if (tri->v2 == v2) {
        tri->v2 = v1;
        tot++;
      }

      if (tri->v3 == v2) {
        tri->v3 = v1;
        tot++;
      }

      if (tot > 1) {
        BLI_trimesh_kill_tri(tm, tri, threadnr, false, false);
      }
    }

    //swap out vertex references for edge
    if (e->v1 == v2) {
      e->v1 = v1;
    } else {
      e->v2 = v1;
    }

    simplelist_append(tm, &v1->edges, e, POOL_ELIST, threadnr);
  }

  v2->edges.length = 0;
  BLI_trimesh_kill_vert(tm, v2, threadnr);
}

void BLI_trimesh_elem_is_dead(void *elem) {
  return BLI_safepool_elem_is_dead(elem);
}

void BLI_trimesh_collapse_edge(BLI_TriMesh *tm, TMEdge *e, int threadnr) {
  TMVert *v1 = e->v1, *v2 = e->v2;

  BLI_trimesh_kill_edge(tm, e, threadnr, false);
  weld_verts(tm, v1, v2, threadnr);
}


TMVert *BLI_trimesh_split_edge(BLI_TriMesh *tm, TMEdge *e, int threadnr, float fac, bool skipcd) {
  float co[3];
  float no[3];

  interp_v3_v3v3(co, e->v1->co, e->v2->co, fac);
  add_v3_v3v3(no, e->v1->no, e->v2->no);
  normalize_v3(no);

  TMVert *vc = BLI_trimesh_make_vert(tm, co, no, threadnr, skipcd);

  TMEdge *e2 = ensure_edge(tm, e->v1, vc, threadnr, skipcd);
  TMEdge *e3 = ensure_edge(tm, vc, e->v2, threadnr, skipcd);

#ifdef WITH_TRIMESH_CUSTOMDATA
  if (!skipcd) {
    float src_weights[2] = {0.5f, 0.5f};
    void *src_blocks[2] = {e->v1->customdata, e->v2->customdata};
  
    CustomData_bmesh_interp(&tm->vdata, src_blocks, src_weights, NULL, 2, vc->customdata);
    CustomData_bmesh_copy_data(&tm->edata, &tm->edata, &e->customdata, &e2->customdata);
    CustomData_bmesh_copy_data(&tm->edata, &tm->edata, &e->customdata, &e3->customdata);
  }
#endif

  for (int i=0; i<e->tris.length; i++) {
    TMFace *tri = e->tris.items[0];

    TMVert *tv1, *tv2, *tv3;
    int vi = 0;

    for (int j=0; j<3; j++) {
      if (TRIEDGE(tri, j) == e) {
        vi = j;
        break;
      }
    }

    tv1 = TRIVERT(tri, vi);
    tv2 = TRIVERT(tri, (vi+1)%3);
    tv3 = TRIVERT(tri, (vi+2)%3);

    TMFace *t1 = BLI_trimesh_make_tri(tm, tv1, vc, tv3, threadnr, skipcd);
    TMFace *t2 = BLI_trimesh_make_tri(tm, vc, tv2, tv3, threadnr, skipcd);

#ifdef WITH_TRIMESH_CUSTOMDATA
    if (!skipcd) {
      TMLoopData *l1 = TRILOOP(tri, vi);
      TMLoopData *l2 = TRILOOP(tri, (vi+1)%3);
      TMLoopData *l3 = TRILOOP(tri, (vi+2)%3);

      float src_weights[2] = {0.5f, 0.5f};
      void *src_blocks[2] = {l1->customdata, l3->customdata};

      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l1->customdata, t1->l1->customdata);
      CustomData_bmesh_interp(&tm->ldata, src_blocks, src_weights, NULL, 2, t1->l2->customdata);
      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l3->customdata, t1->l3->customdata);

      src_blocks[0] = l2->customdata;
      src_blocks[1] = l3->customdata;

      CustomData_bmesh_interp(&tm->ldata, src_blocks, src_weights, NULL, 2, t2->l1->customdata);
      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l1->customdata, t1->l2->customdata);
      CustomData_bmesh_copy_data(&tm->ldata, &tm->ldata, l3->customdata, t1->l3->customdata);
    }
#endif
    BLI_trimesh_kill_tri(tm, tri, threadnr, false, false);
  }

  BLI_trimesh_kill_edge(tm, e, threadnr, false);
  BLI_trimesh_calc_vert_normal(vc, threadnr, true);

  return vc;
}

void BLI_trimesh_index_update(BLI_TriMesh *tm) {
  ThreadSafePoolIter iter;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter);
  TMVert *v = BLI_safepool_iterstep(&iter);
  for (int i=0; v; v = BLI_safepool_iterstep(&iter), i++) {
    v->index = i;
  }

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter);
  TMEdge *e = BLI_safepool_iterstep(&iter);
  for (int i=0; e; e = BLI_safepool_iterstep(&iter), i++) {
    e->index = i;
  }

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter);
  TMFace *t = BLI_safepool_iterstep(&iter);
  for (int i=0; t; t = BLI_safepool_iterstep(&iter), i++) {
    t->index = i;
  }
}

#include "BLI_ghash.h"

typedef struct LogEntry {
  union {
    void *ptr;
    float vec3[3];
    float f;
    int i;
    int ivec3[3];
    int ivec4[4];
  } value;
} LogEntry;

typedef struct TriMeshLog {
  GHash *elemhash_ptr; //maps ids to pointers
  GHash *elemhash_id; //maps pointers to ids
  GHash *elemhash_entry; //maps ids to entries indices

  LogEntry *entries;
  int *groups;
  int totgroup;
  int totentries;
  int idgen;

  BLI_TriMesh *tm;

  int cd_vert_mask_index;

  int curgroup;
} TriMeshLog;

static void trimesh_add_group(TriMeshLog *log, bool set_curgroup) {
  int *groups = log->groups;
  BLI_array_declare(groups);

  BLI_array_len_set(groups, log->totgroup);
  BLI_array_append(groups, log->totentries);

  log->groups = groups;
  log->totgroup = BLI_array_len(groups);

  if (set_curgroup) {
    log->curgroup = log->totgroup-1;
  }
}

TriMeshLog *BLI_trimesh_log_new(BLI_TriMesh *tm, int cd_vert_mask_index) {
  TriMeshLog *log = MEM_callocN(sizeof(*log), "TriMeshLog");

  log->elemhash_ptr = BLI_ghash_ptr_new("TriMeshLog ghash ptr");
  log->elemhash_id = BLI_ghash_int_new("TriMeshLog ghash int");
  log->elemhash_entry = BLI_ghash_int_new("TriMeshLog ghash entry");
  log->cd_vert_mask_index = cd_vert_mask_index;
  log->idgen = 1;

  log->tm = tm;

  trimesh_add_group(log, true);

  return log;
}

void BLI_trimesh_log_free(TriMeshLog *log) {
  BLI_ghash_free(log->elemhash_ptr, NULL, NULL);
  BLI_ghash_free(log->elemhash_id, NULL, NULL);
  BLI_ghash_free(log->elemhash_entry, NULL, NULL);
}

static void tlog_truncate(TriMeshLog *log) {
  if (log->curgroup == log->totgroup-1) {
    return;
  }

  log->totentries = log->groups[log->curgroup+1];
  log->totgroup = log->curgroup+1;
}

static LogEntry *tlog_push(TriMeshLog *log) {
  LogEntry *entries = log->entries;
  LogEntry e = {0,};

  BLI_array_declare(entries);

  if (log->curgroup != log->totgroup) {
    tlog_truncate(log);
  }

  BLI_array_append(entries, e);

  log->entries = entries;
  log->totentries = BLI_array_len(entries);
}

static void tlog_f(TriMeshLog *log, float f) {
  LogEntry *e = tlog_push(log);
  e->value.f = f;
}

static void tlog_v3(TriMeshLog *log, float f[3]) {
  LogEntry *e = tlog_push(log);
  copy_v3_v3(e->value.vec3, f);
}

static void tlog_i3(TriMeshLog *log, int f[3]) {
  LogEntry *e = tlog_push(log);

  e->value.ivec3[0] = f[0];
  e->value.ivec3[1] = f[1];
  e->value.ivec3[2] = f[2];
}

static void tlog_i4(TriMeshLog *log, int f[4]) {
  LogEntry *e = tlog_push(log);

  e->value.ivec4[0] = f[0];
  e->value.ivec4[1] = f[2];
  e->value.ivec4[2] = f[3];
  e->value.ivec4[3] = f[4];
}


static void tlog_i(TriMeshLog *log, int f) {
  LogEntry *e = tlog_push(log);
  e->value.i = f;
}

static void tlog_ptr(TriMeshLog *log, void *f) {
  LogEntry *e = tlog_push(log);
  e->value.ptr = f;
}

static int tlog_start(TriMeshLog *log, int code) {
  int i = log->totentries;

  tlog_i(log, code);

  return i;
}

static void tlog_end(TriMeshLog *log, int entry_i) {
  tlog_i(log, entry_i);
}

enum {
  LOG_VERT_ADD = 0,
  LOG_EDGE_ADD = 1,
  LOG_TRI_ADD = 2,
  LOG_VERT_KILL = 3,
  LOG_EDGE_KILL = 4,
  LOG_TRI_KILL = 5,
  LOG_SPLIT_EDGE = 6,
  LOG_COLLAPSE_EDGE = 7,
  LOG_CD = 8,
  LOG_VERT_STATE = 9
};

/* Set a vertex's paint-mask value
*
* Has no effect is no paint-mask layer is present */
static void vert_mask_set(TMVert *v, const float new_mask, const int cd_vert_mask_offset)
{
  if (cd_vert_mask_offset != -1) {
    TRIMESH_ELEM_CD_SET_FLOAT(v, cd_vert_mask_offset, new_mask);
  }
}

static float vert_mask_get(TMVert *v, const int cd_vert_mask_offset)
{
  if (cd_vert_mask_offset != -1) {
    return TRIMESH_ELEM_CD_GET_FLOAT(v, cd_vert_mask_offset);
  }
  else {
    return 0.0f;
  }
}

static void elemhash_add(TriMeshLog *log, void *elem, int id, int entryidx) {
  BLI_ghash_insert(log->elemhash_id, (void*)id, elem);
  BLI_ghash_insert(log->elemhash_ptr, elem, (void*)id);
  BLI_ghash_insert(log->elemhash_entry, (void*)id, (void*)entryidx);
}

static void *elemhash_lookup_id(TriMeshLog *log, int id) {
  return BLI_ghash_lookup(log->elemhash_id, (void*)id);
}

static int elemhash_get_id(TriMeshLog *log, void *elem) {
  void **ret = NULL;

  BLI_ghash_lookup_p(log->elemhash_ptr, elem);

  if (!ret || !*ret) {
    return -1;
  }

  return (int)(*ret);
}

static int elemhash_ensure_id(TriMeshLog *log, void *elem) {
  void **ret = NULL;

  BLI_ghash_lookup_p(log->elemhash_ptr, elem);

  if (!ret || !*ret) {
    int id = log->idgen++;
    elemhash_add(log, elem, id, -1);

    return id;
  }

  return (int)(*ret);
}

int BLI_trimesh_log_vert_add(TriMeshLog *log, TMVert *v, const int cd_mask_offset, bool skipcd) {
  int id = log->idgen++;

  elemhash_add(log, v, id, log->totentries);
  
  int start = tlog_start(log, LOG_VERT_ADD);

  tlog_i(log, id);
  tlog_v3(log, v->co);
  tlog_v3(log, v->no);
  tlog_f(log, vert_mask_get(v, cd_mask_offset));
  tlog_i(log, skipcd);

  tlog_end(log, start);

  return id;
}

int elemhash_has_id(TriMeshLog *log, void *elem) {
  return elemhash_get_id(log, elem) != NULL;
}

int elemhash_get_vert_id(TriMeshLog *log, TMVert *v, int cd_vert_mask_offset) {
  int ret = elemhash_get_id(log, v);

  if (!v) {
    return BLI_trimesh_log_vert_add(log, v, cd_vert_mask_offset, false);
  }

  return ret;
}

int BLI_trimesh_log_edge_add(TriMeshLog *log, TMEdge *e, const int cd_mask_offset, int skipcd) {
  int id = log->idgen++;

  elemhash_add(log, e, id, log->totentries);

  int v1id = elemhash_get_vert_id(log, e->v1, cd_mask_offset);
  int v2id = elemhash_get_vert_id(log, e->v2, cd_mask_offset);

  int start = tlog_start(log, LOG_EDGE_ADD);
  tlog_i(log, id);
  tlog_i(log, v1id);
  tlog_i(log, v2id);
  tlog_i(log, skipcd);

  tlog_end(log, start);

  return id;
}

static int elemhash_get_edge_id(TriMeshLog *log, TMEdge *e, int cd_vert_mask_offset) {
  int ret = elemhash_get_id(log, e);

  if (!e) {
    return BLI_trimesh_log_edge_add(log, e, cd_vert_mask_offset, false);
  }

  return ret;
}


static int trimesh_log_cdata(TriMeshLog *log, void *velem, int type) {
  TMElement *elem = velem;
  CustomData *cdata = get_customdata(log->tm, type);

  int size = !elem->customdata ? 0 : cdata->totsize;
  if (!size) {
    tlog_i(log, 0);
    tlog_i(log, 0);
    return;
  }

  tlog_i(log, size);

  int totentries = size>>2 + 1;
  int cur = 0;

  tlog_i(log, totentries);

  char *addr = (char*) elem->customdata;

  for (int i=0; i<totentries; i++) {
    int tot;

    if (i == totentries-1) {
      tot = size % 4;
    } else {
      tot = 4;
    }

    for (int j=0; j<tot; j++) {
      int iv[4];

      memcpy(iv, addr, sizeof(int)*tot);
      addr += sizeof(int)*tot;

      tlog_i4(log, iv);
    }
  }
}

static int trimesh_log_read_cdata(TriMeshLog *tlog, int entry_i, void *velem, int type) {
  CustomData *cdata = get_customdata(tlog->tm, type);
  TMElement *elem = velem;
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int size = log[i--].value.i;
  int totchunk = log[i--].value.i;

  void *data = elem->customdata;
  char *addr = (char*)data;

  for (int j=0; j<totchunk; j++) {
    int tot;
    int *iv = log[i--].value.ivec4;

    if (i == totchunk-1) {
      tot = size % 4;
    } else {
      tot = 4;
    }

    memcpy(addr, iv, sizeof(int)*tot);
    addr += sizeof(int)*tot;
  }

  return i;
}


static int trimesh_log_skip_cdata(TriMeshLog *tlog, int entry_i) {
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int size = log[i--].value.i;
  int totchunk = log[i--].value.i;

  return i - totchunk;
}

static int trimesh_skip_loop(TriMeshLog *log, int entry_i) {
  return trimesh_log_skip_cdata(log, entry_i+1);
}

static int trimesh_log_loop(TriMeshLog *log, TMFace *tri, TMLoopData *loop) {
  trimesh_log_cdata(log, loop, TM_LOOP);
}

static int trimesh_read_loop(TriMeshLog *tlog, TMLoopData *l, int entry_i) {
  int i = entry_i;

  i++; //skip entry tag
  i = trimesh_log_read_cdata(tlog, i, l, TM_LOOP);

  return i;
}

int BLI_trimesh_log_tri(TriMeshLog *log, TMFace *tri, bool skipcd) {
  int id = log->idgen++;

  int cd_vert_mask_offset = log->cd_vert_mask_index;

  elemhash_add(log, tri, id, log->totentries);

  int e1 = elemhash_get_edge_id(log, tri->e1, cd_vert_mask_offset);
  int e2 = elemhash_get_edge_id(log, tri->e2, cd_vert_mask_offset);
  int e3 = elemhash_get_edge_id(log, tri->e3, cd_vert_mask_offset);

  int v1 = elemhash_get_vert_id(log, tri->v1, cd_vert_mask_offset);
  int v2 = elemhash_get_vert_id(log, tri->v1, cd_vert_mask_offset);
  int v3 = elemhash_get_vert_id(log, tri->v1, cd_vert_mask_offset);
  int start = tlog_start(log, LOG_TRI_ADD);

  tlog_i(log, id);
  tlog_i(log, skipcd);

  tlog_i(log, v1);
  tlog_i(log, v2);
  tlog_i(log, v3);

  tlog_i(log, e1);
  tlog_i(log, e2);
  tlog_i(log, e3);

#ifdef WITH_TRIMESH_CUSTOMDATA
  if (!skipcd) {
    trimesh_log_loop(log, tri, tri->l1);
    trimesh_log_loop(log, tri, tri->l2);
    trimesh_log_loop(log, tri, tri->l3);
  }
#endif

  return id;
}

static int elemhash_get_tri_id(TriMeshLog *log, TMFace *tri) {
  int ret = elemhash_get_id(log, tri);

  if (!ret) {
    return BLI_trimesh_log_tri(log, tri, log->cd_vert_mask_index, false);
  }

  return ret;
}

int BLI_trimesh_log_vert_kill(TriMeshLog *log, TMVert *v, int cd_mask_offset) {
  int id = elemhash_get_id(log, v);

  if (!id) {
    return; //invalid element
  }

  //make sure edge entries exist
  for (int i=0; i<v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];
    elemhash_get_edge_id(log, e, cd_mask_offset);
  }

  int start = tlog_start(log, LOG_VERT_KILL);
  tlog_i(log, id);

  //create a wind list
  tlog_i(log, v->edges.length);
  for (int i=0; i<v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];
    tlog_i(log, elemhash_get_edge_id(log, e, cd_mask_offset));
  }

  tlog_end(log, start);

  return id;
}

int BLI_trimesh_log_edge_kill(TriMeshLog *log, TMEdge *e, int kill_verts) {
  int id = elemhash_get_id(log, e);

  if (!id) {
    return; //invalid element
  }

  //make sure tri entries exist
  for (int i=0; i<e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    elemhash_get_tri_id(log, tri);
  }

  int start = tlog_start(log, LOG_EDGE_KILL);
  tlog_i(log, id);
  tlog_i(log, kill_verts);

  tlog_i(log, e->tris.length);
  for (int i=0; i<e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    tlog_i(log, elemhash_get_tri_id(log, tri));
  }

  tlog_end(log, start);
}

int BLI_trimesh_log_tri_kill(TriMeshLog *log, TMFace *tri, int kill_verts, int kill_edges) {
  int id = elemhash_get_id(log, tri);

  if (!id) {
    return; //invalid element
  }

  int start = tlog_start(log, LOG_TRI_KILL);
  tlog_i(log, id);
  tlog_i(log, kill_verts);
  tlog_i(log, kill_edges);

  //make wind list
  tlog_i(log, 6);
  tlog_i(log, elemhash_get_edge_id(log, tri->e1, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_edge_id(log, tri->e2, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_edge_id(log, tri->e3, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_vert_id(log, tri->v1, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_vert_id(log, tri->v2, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_vert_id(log, tri->v3, log->cd_vert_mask_index));

#ifdef WITH_TRIMESH_CUSTOMDATA
  trimesh_log_cdata(log, tri->l1, TM_LOOP);
  trimesh_log_cdata(log, tri->l2, TM_LOOP);
  trimesh_log_cdata(log, tri->l3, TM_LOOP);
#endif

  tlog_end(log, start);
}

void BLI_trimesh_log_split_edge(TriMeshLog *log, TMEdge *e, float fac, TMVert *newvert, int cd_vert_mask_index) {
  int start = tlog_start(log, LOG_SPLIT_EDGE);

  int nvid = elemhash_get_vert_id(log, newvert, log->cd_vert_mask_index);
  int v1 = elemhash_get_vert_id(log, e->v1, log->cd_vert_mask_index);
  int v2 = elemhash_get_vert_id(log, e->v2, log->cd_vert_mask_index);
  int eid = elemhash_get_edge_id(log, e, log->cd_vert_mask_index);

  tlog_i(log, eid);
  tlog_i(log, v1);
  tlog_i(log, v2);
  tlog_i(log, nvid);
  tlog_f(log, fac);

  tlog_end(log, start);
}

void BLI_trimesh_log_collapse_edge(TriMeshLog *log, TMEdge *e) {
  int start = tlog_start(log, LOG_COLLAPSE_EDGE);
  tlog_i(log, elemhash_get_id(log, e->v1));
  tlog_i(log, elemhash_get_id(log, e->v2));

  tlog_end(log, start);
}

//based on BM_log_vert_before_modified
//saves coordinates and flags
int BLI_trimesh_log_vert_state(TriMeshLog *log, TMVert *v) {
  int id = elemhash_get_vert_id(log, v, log->cd_vert_mask_index);

  int start = tlog_start(log, LOG_VERT_STATE);
  int ivec[3] = {v->flag, v->index, v->threadtag};

  tlog_i(log, id);
  tlog_v3(log, v->co);
  tlog_v3(log, v->no);
  tlog_i3(log, ivec);
  tlog_f(log, TRIMESH_ELEM_CD_GET_FLOAT(v, log->cd_vert_mask_index));
}

void BLI_log_add_setpoint(TriMeshLog *log, int setgroup) {
  tlog_truncate(log);
  trimesh_add_group(log, setgroup);
}

static int meshlog_wind(TriMeshLog *tlog, int entry_i, int threadnr) {
  int i = entry_i;

  LogEntry *log = tlog->entries;
  LogEntry *item = &log[i++];

  switch (item->value.i) {
  case LOG_VERT_STATE: {
    //update vert state log entry?
    break;
  }

  case LOG_VERT_ADD: {
    int id = log[i++].value.i;
    float *co = log[i++].value.vec3;
    float *no = log[i++].value.vec3;
    float mask = log[i++].value.f;
    int skipcd = log[i++].value.i;

    TMVert *v = BLI_trimesh_make_vert(tlog->tm, co, no, threadnr, skipcd);

    if (tlog->cd_vert_mask_index >= 0) {
      TRIMESH_ELEM_CD_SET_FLOAT(v, tlog->cd_vert_mask_index, mask);
    }

    elemhash_add(tlog, v, id, entry_i);

    break;
  }
  }

  return i;
}

static int mesh_wind_list(TriMeshLog *tlog, int entry_i, int threadnr) {
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int tot = log[i++].value.i;

  for (int j=0; j<tot; j++) {
    int tid = log[i++].value.i;
    int **ret = (int**) BLI_ghash_lookup_p(tlog->elemhash_entry, tid);

    if (!ret) {
      continue; //error!
    }

    int ti = (int)*ret;
    meshlog_wind(tlog, ti, threadnr);
  }

  return i;
}

static int meshlog_unwind(TriMeshLog *tlog, int entry_i, int threadnr);

static int mesh_unwind_list(TriMeshLog *tlog, int entry_i, int threadnr) {
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int tot = log[i++].value.i;

  for (int j=0; j<tot; j++) {
    int tid = log[i++].value.i;
    int **ret = (int**) BLI_ghash_lookup_p(tlog->elemhash_entry, tid);

    if (!ret) {
      continue; //error!
    }

    int ti = (int)*ret;
    meshlog_unwind(tlog, ti, threadnr);
  }

  return i;
}

static int meshlog_unwind(TriMeshLog *tlog, int entry_i, int threadnr) {
  int i = entry_i;

  LogEntry *log = tlog->entries;
  LogEntry *item = &log[i++];

  switch (item->value.i) {
  case LOG_VERT_ADD: {
    int id = log[i++].value.i;
    float *co = log[i++].value.vec3;
    float *no = log[i++].value.vec3;
    float mask = log[i++].value.f;
    int skipcd = log[i++].value.i;

    TMVert *v = elemhash_lookup_id(tlog, id);

    BLI_trimesh_kill_vert(tlog->tm, v, 0);
    break;
  }
  case LOG_VERT_STATE:{
    int id = log[i++].value.i;
    float *co = log[i++].value.vec3;
    float *no = log[i++].value.vec3;
    int *ivec = log[i++].value.ivec3;
    float mask = log[i++].value.f;

    TMVert *v = elemhash_lookup_id(tlog, id);
    copy_v3_v3(v->co, co);
    copy_v3_v3(v->no, no);
    v->flag = ivec[0];
    v->index = ivec[1];
    v->threadtag = ivec[2];
    TRIMESH_ELEM_CD_SET_FLOAT(v, tlog->cd_vert_mask_index, mask);

    break;
  }

  case LOG_EDGE_ADD: {
    int id = log[i++].value.i;
    int v1id = log[i++].value.i;
    int v2id = log[i++].value.i;
    int skipcd = log[i++].value.i;
    break;
  }
  case LOG_TRI_ADD: {
    int id = log[i++].value.i;
    int v1id = log[i++].value.i;
    int v2id = log[i++].value.i;
    int v3id = log[i++].value.i;
    int skipcd = log[i++].value.i;

    if (!skipcd) {
      //remember that customdata logs usually don't handle their own heading tag
      i = trimesh_skip_loop(tlog, i);
      i = trimesh_skip_loop(tlog, i);
      i = trimesh_skip_loop(tlog, i);
    }
    break;
  }
  case LOG_VERT_KILL: {
    int id = log[i++].value.i;
    int totedge = log[i++].value.i;

    for (int j=0; j<totedge; j++) {
      int eid = log[i++].value.i;
      int **ret = (int**) BLI_ghash_lookup_p(tlog->elemhash_entry, eid);

      if (!ret) {
        continue; //error!
      }

      int ei = (int)*ret;
      meshlog_wind(tlog, ei, threadnr);
    }
    break;
  }

  case LOG_EDGE_KILL: {
    int id = log[i++].value.i;
    int kill_verts = log[i++].value.i;
    mesh_wind_list(tlog, i, threadnr);
    break;
  }

  case LOG_TRI_KILL: {
    int id = log[i++].value.i;
    int kill_verts = log[i++].value.i;
    int kill_edges = log[i++].value.i;
    TMVert *vs[3], *es[3];

    mesh_wind_list(tlog, i, threadnr);

    i += 1; //skip wind list length (which is 6)

    for (int j=0; j<3; j++) {
      int id = log[i++].value.i;

      vs[j] = BLI_ghash_lookup(tlog->elemhash_ptr, (void*)id);
    }

    i -= 6;
    TMFace *tri = BLI_trimesh_make_tri(tlog->tm, vs[0], vs[1], vs[2], threadnr, false);

#ifdef WITH_TRIMESH_CUSTOMDATA
    i = trimesh_read_loop(tlog, tri->l1, i);
    i = trimesh_read_loop(tlog, tri->l2, i);
    i = trimesh_read_loop(tlog, tri->l3, i);
#endif
    break;
  }

  case LOG_SPLIT_EDGE: {
    int v1id = log[i++].value.i;
    int v2id = log[i++].value.i;
    float fac = log[i++].value.f;
    int logged_newvert = log[i++].value.i;
    break;
  }
  case LOG_COLLAPSE_EDGE: {
    int v1id = log[i++].value.i;
    int v2id = log[i++].value.i;
    break;
  }
  }

  return i;
}

void BLI_log_unwind(TriMeshLog *tlog, int threadnr) {
  int start;
  int end;

  if (tlog->curgroup == tlog->totgroup-1) {
    start = tlog->totentries - 1;
  } else {
    start = tlog->groups[tlog->totgroup+1];
  }

  end = tlog->groups[tlog->curgroup];
  LogEntry *log = tlog->entries;

  for (int i=start; i >= end; i = log[i].value.i) {
    meshlog_unwind(tlog, i, threadnr);
  }
}

char BLI_trimesh_mesh_cd_flag_from_bmesh(BLI_TriMesh *tm)
{
  char cd_flag = 0;
  if (CustomData_has_layer(&tm->vdata, CD_BWEIGHT)) {
    cd_flag |= ME_CDFLAG_VERT_BWEIGHT;
  }
  if (CustomData_has_layer(&tm->edata, CD_BWEIGHT)) {
    cd_flag |= ME_CDFLAG_EDGE_BWEIGHT;
  }
  if (CustomData_has_layer(&tm->edata, CD_CREASE)) {
    cd_flag |= ME_CDFLAG_EDGE_CREASE;
  }
  return cd_flag;
}

BLI_INLINE void tmesh_quick_edgedraw_flag(MEdge *med, TMEdge *e)
{
  /* This is a cheap way to set the edge draw, its not precise and will
  * pick the first 2 faces an edge uses.
  * The dot comparison is a little arbitrary, but set so that a 5 subd
  * IcoSphere won't vanish but subd 6 will (as with pre-bmesh Blender). */

  if (e->tris.length > 1) {
    TMFace *t1 = e->tris.items[0];
    TMFace *t2 = e->tris.items[1];

    if (dot_v3v3(t1->no, t2->no) > 0.9995f) {
      med->flag &= ~ME_EDGEDRAW;
    } else {
      med->flag |= ME_EDGEDRAW;
    }
  }
}


/**
* \brief BMesh -> Mesh
*/
static TMVert **tm_to_mesh_vertex_map(BLI_TriMesh *bm, int ototvert)
{
  const int cd_shape_keyindex_offset = CustomData_get_offset(&bm->vdata, CD_SHAPE_KEYINDEX);
  TMVert **vertMap = NULL;
  TMVert *eve;
  int i = 0;
  BLI_TriMeshIter iter;

  /* Caller needs to ensure this. */
  BLI_assert(ototvert > 0);

  vertMap = MEM_callocN(sizeof(*vertMap) * ototvert, "vertMap");
  if (cd_shape_keyindex_offset != -1) {
    BLI_trimesh_vert_iternew(bm, &iter);
    eve = BLI_trimesh_iterstep(&iter);

    for (; eve; eve = BLI_trimesh_iterstep(&iter), i++) {
      const int keyi = TRIMESH_ELEM_CD_GET_INT(eve, cd_shape_keyindex_offset);
      if ((keyi != ORIGINDEX_NONE) && (keyi < ototvert) &&
        /* Not fool-proof, but chances are if we have many verts with the same index,
        * we will want to use the first one,
        * since the second is more likely to be a duplicate. */
        (vertMap[keyi] == NULL)) {
        vertMap[keyi] = eve;
      }
    }
  }
  else {
    BLI_trimesh_vert_iternew(bm, &iter);
    eve = BLI_trimesh_iterstep(&iter);

    for (; eve; eve = BLI_trimesh_iterstep(&iter), i++) {
      if (i < ototvert) {
        vertMap[i] = eve;
      }
      else {
        break;
      }
    }
  }

  return vertMap;
}

void BLI_trimesh_mesh_bm_to_me(struct Main *bmain,
  BLI_TriMesh *tm,
  struct Mesh *me,
  const struct TMeshToMeshParams *params)
{
  TMVert *v;
  TMEdge *e;
  TMFace *f;

  const int cd_vert_bweight_offset = CustomData_get_offset(&tm->vdata, CD_BWEIGHT);
  const int cd_edge_bweight_offset = CustomData_get_offset(&tm->edata, CD_BWEIGHT);
  const int cd_edge_crease_offset = CustomData_get_offset(&tm->edata, CD_CREASE);
  const int cd_shape_keyindex_offset = CustomData_get_offset(&tm->vdata, CD_SHAPE_KEYINDEX);

  MVert *oldverts = NULL;
  const int ototvert = me->totvert;

  if (me->key && (cd_shape_keyindex_offset != -1)) {
    /* Keep the old verts in case we are working on* a key, which is done at the end. */

    /* Use the array in-place instead of duplicating the array. */
#if 0
    oldverts = MEM_dupallocN(me->mvert);
#else
    oldverts = me->mvert;
    me->mvert = NULL;
    CustomData_update_typemap(&me->vdata);
    CustomData_set_layer(&me->vdata, CD_MVERT, NULL);
#endif
  }

  /* Free custom data. */
  CustomData_free(&me->vdata, me->totvert);
  CustomData_free(&me->edata, me->totedge);
  CustomData_free(&me->fdata, me->totface);
  CustomData_free(&me->ldata, me->totloop);
  CustomData_free(&me->pdata, me->totpoly);

  /* Add new custom data. */
  me->totvert = tm->totvert;
  me->totedge = tm->totedge;
  me->totloop = tm->tottri*3;
  me->totpoly = tm->tottri;
  me->totface = tm->tottri;
  me->act_face = -1;

  {
    CustomData_MeshMasks mask = CD_MASK_MESH;
    CustomData_MeshMasks_update(&mask, &params->cd_mask_extra);
    CustomData_copy(&tm->vdata, &me->vdata, mask.vmask, CD_CALLOC, me->totvert);
    CustomData_copy(&tm->edata, &me->edata, mask.emask, CD_CALLOC, me->totedge);
    CustomData_copy(&tm->ldata, &me->ldata, mask.lmask, CD_CALLOC, me->totloop);
    CustomData_copy(&tm->tdata, &me->pdata, mask.pmask, CD_CALLOC, me->totpoly);
  }

  MVert *mvert = me->totvert ? MEM_callocN(sizeof(MVert) * me->totvert, "tm_to_me.vert") : NULL;
  MEdge *medge = me->totedge ? MEM_callocN(sizeof(MEdge) * me->totedge, "tm_to_me.edge") : NULL;
  MLoop *mloop = me->totloop ? MEM_callocN(sizeof(MLoop) * me->totloop, "tm_to_me.loop") : NULL;
  MPoly *mpoly = me->totface ? MEM_callocN(sizeof(MPoly) * me->totpoly, "tm_to_me.poly") : NULL;
  MFace *mface = me->totface ? MEM_callocN(sizeof(MFace) * me->totface, "tm_to_me.face") : NULL;

  CustomData_add_layer(&me->vdata, CD_MVERT, CD_ASSIGN, mvert, me->totvert);
  CustomData_add_layer(&me->edata, CD_MEDGE, CD_ASSIGN, medge, me->totedge);
  CustomData_add_layer(&me->ldata, CD_MLOOP, CD_ASSIGN, mloop, me->totloop);
  CustomData_add_layer(&me->pdata, CD_MPOLY, CD_ASSIGN, mpoly, me->totpoly);

  me->cd_flag = BLI_trimesh_mesh_cd_flag_from_bmesh(tm);

  /* This is called again, 'dotess' arg is used there. */
  BKE_mesh_update_customdata_pointers(me, 0);

  BLI_TriMeshIter iter;
  BLI_trimesh_vert_iternew(tm, &iter);
  v = BLI_trimesh_iterstep(&iter);
  int i = 0;

  for (; v; v = BLI_trimesh_iterstep(&iter), mvert++, i++) {
    copy_v3_v3(mvert->co, v->co);
    normal_float_to_short_v3(mvert->no, v->no);
    mvert->flag = BLI_trimesh_vert_flag_to_mflag(v);
    v->index = i;

    /* Copy over custom-data. */
    CustomData_from_bmesh_block(&tm->vdata, &me->vdata, v->customdata, i);

    if (cd_vert_bweight_offset != -1) {
      mvert->bweight = TRIMESH_ELEM_CD_GET_FLOAT_AS_UCHAR(v, cd_vert_bweight_offset);
    }
  }

  BLI_trimesh_edge_iternew(tm, &iter);
  e = BLI_trimesh_iterstep(&iter);
  i = 0;

  MEdge *med = medge;
  for (; e; e = BLI_trimesh_iterstep(&iter), i++, med++) {
    med->v1 = e->v1->index;
    med->v2 = e->v2->index;
    e->index = i;

    med->flag = BLI_trimesh_edge_flag_to_mflag(e);

    /* Copy over custom-data. */
    CustomData_from_bmesh_block(&tm->edata, &me->edata, e->customdata, i);

    tmesh_quick_edgedraw_flag(med, e);

    if (cd_edge_crease_offset != -1) {
      med->crease = TRIMESH_ELEM_CD_GET_FLOAT_AS_UCHAR(e, cd_edge_crease_offset);
    }
    if (cd_edge_bweight_offset != -1) {
      med->bweight = TRIMESH_ELEM_CD_GET_FLOAT_AS_UCHAR(e, cd_edge_bweight_offset);
    }
  }

  BLI_trimesh_tri_iternew(tm, &iter);
  f = BLI_trimesh_iterstep(&iter);
  i = 0;

  int j = 0;
  for (; f; f = BLI_trimesh_iterstep(&iter), i++, mpoly++) {
    mpoly->loopstart = j;
    mpoly->totloop = 3;
    mpoly->mat_nr = f->mat_nr;

    for (int k=0; k<3; k++, j++, mloop++) {
      TMLoopData *ld = TRIMESH_GET_TRI_LOOP(f, k);

      mloop->e = TRIMESH_GET_TRI_EDGE(f, k)->index;
      mloop->e = TRIMESH_GET_TRI_VERT(f, k)->index;

      /* Copy over custom-data. */
      CustomData_from_bmesh_block(&tm->ldata, &me->ldata, ld->customdata, j);
    }

    /* Copy over custom-data. */
    CustomData_from_bmesh_block(&tm->tdata, &me->pdata, f->customdata, i);
  }

  /* Patch hook indices and vertex parents. */
  if (params->calc_object_remap && (ototvert > 0)) {
    BLI_assert(bmain != NULL);
    Object *ob;
    ModifierData *md;
    TMVert **vertMap = NULL;
    TMVert *eve = NULL;

    for (ob = bmain->objects.first; ob; ob = ob->id.next) {
      if ((ob->parent) && (ob->parent->data == me) && ELEM(ob->partype, PARVERT1, PARVERT3)) {

        if (vertMap == NULL) {
          vertMap = tm_to_mesh_vertex_map(tm, ototvert);
        }

        if (ob->par1 < ototvert) {
          eve = vertMap[ob->par1];
          if (eve) {
            ob->par1 = eve->index;
          }
        }
        if (ob->par2 < ototvert) {
          eve = vertMap[ob->par2];
          if (eve) {
            ob->par2 = eve->index;
          }
        }
        if (ob->par3 < ototvert) {
          eve = vertMap[ob->par3];
          if (eve) {
            ob->par3 = eve->index;
          }
        }
      }
      if (ob->data == me) {
        for (md = ob->modifiers.first; md; md = md->next) {
          if (md->type == eModifierType_Hook) {
            HookModifierData *hmd = (HookModifierData *)md;

            if (vertMap == NULL) {
              vertMap = tm_to_mesh_vertex_map(tm, ototvert);
            }

            for (i = j = 0; i < hmd->totindex; i++) {
              if (hmd->indexar[i] < ototvert) {
                eve = vertMap[hmd->indexar[i]];

                if (eve) {
                  hmd->indexar[j++] = eve->index;
                }
              }
              else {
                j++;
              }
            }

            hmd->totindex = j;
          }
        }
      }
    }

    if (vertMap) {
      MEM_freeN(vertMap);
    }
  }

  BKE_mesh_update_customdata_pointers(me, false);
}

