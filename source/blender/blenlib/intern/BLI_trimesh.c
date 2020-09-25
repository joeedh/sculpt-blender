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

#include "BLI_utildefines.h"
#include "atomic_ops.h"

#include "MEM_guardedalloc.h"
#ifdef WITH_MEM_VALGRIND
#  include "valgrind/memcheck.h"
#endif


#define TRIVERT(tri, n) ((&(tri)->v1)[n])
#define TRIEDGE(tri, n) ((&(tri)->e1)[n])


enum {
POOL_VERTEX = 0,
POOL_EDGE = 1,
//POOL_LOOP = 2,
POOL_TRI = 2,
POOL_ELIST = 3, //pool for lists of edges around vertices
POOL_TLIST = 4 //pool for lists of triangles around edges
};

#define V_ELIST_ESIZE 5
#define E_TLIST_ESIZE 2

//#define 
static int poolsizes[] = {
  sizeof(OptTriVert),
  sizeof(OptTriEdge),
  sizeof(OptTri),
  sizeof(void*)*V_ELIST_ESIZE,
  sizeof(void*)*E_TLIST_ESIZE,
};

OptTriMesh* BLI_trimesh_new(int maxthread) {
  OptTriMesh* tm = MEM_callocN(sizeof(*tm), "OptTriMesh");
  int i;

  for (i = 0; i < MAX_TRIMESH_POOLS; i++) {
    tm->pools[i] = BLI_safepool_create(poolsizes[i], 0, maxthread);
  }

  return tm;
}

static void simplelist_remove(OptTriMesh *tm, optmesh_simplelist *list, void *item, int pool, int threadnr) {
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

static void simplelist_free(OptTriMesh *tm, optmesh_simplelist *list, int pool, int threadnr) {
  if (list->is_pool_allocd) {
    BLI_safepool_free(tm->pools[pool], list->items);
  } else {
    MEM_freeN(list->items);
  }
}

static void simplelist_append(OptTriMesh* tm, optmesh_simplelist* list, void *item, int pool, int threadnr) {
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

static OptTriEdge *ensure_edge(OptTriMesh* tm, OptTriVert* v1, OptTriVert* v2, int threadnr) {
  for (int i = 0; i < v1->edges.length; i++) {
    OptTriEdge *e = v1->edges.items[i];

    if (OTHER_VERT(e, v1) == v2) {
      return e;
    }
  }

  OptTriEdge *e = BLI_safepool_alloc(tm->pools[POOL_EDGE], threadnr);
  memset(e, 0, sizeof(*e));

  e->v1 = v1;
  e->v2 = v2;

  simplelist_append(tm, &e->v1->edges, e, POOL_ELIST, threadnr);
  simplelist_append(tm, &e->v2->edges, e, POOL_ELIST, threadnr);

  return e;
}

void BLI_trimesh_vert_iternew(OptTriMesh *tm, OptTriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_VERTEX;

  BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter->iter);
}

void BLI_trimesh_edge_iternew(OptTriMesh *tm, OptTriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_EDGE;

  BLI_safepool_iternew(tm->pools[POOL_EDGE], &iter->iter);
}

void BLI_trimesh_tri_iternew(OptTriMesh *tm, OptTriMeshIter* iter) {
  memset(iter, 0, sizeof(*iter));
  iter->pool = POOL_TRI;

  BLI_safepool_iternew(tm->pools[POOL_TRI], &iter->iter);
}

void BLI_trimesh_iterstep(OptTriMeshIter* iter) {
  return BLI_safepool_iterstep(&iter->iter);
}

static OptTriEdge *edge_add_tri(OptTriMesh* tm, OptTriVert* v1, OptTriVert* v2, OptTri* tri, int threadnr) {
  OptTriEdge *e = ensure_edge(tm, v1, v2, threadnr);
  simplelist_append(tm, &e->tris, tri, POOL_TLIST, threadnr);

  return e;
}

OptTriVert *BLI_trimesh_make_vert(OptTriMesh *tm, float co[3], float no[3], int threadnr) {
  OptTriVert *v = BLI_safepool_alloc(tm->pools[POOL_VERTEX], threadnr);

  memset(v, 0, sizeof(*v));

  copy_v3_v3(v->co, co);
  copy_v3_v3(v->no, no);

}
OptTri *BLI_trimesh_make_tri(OptTriMesh *tm, OptTriVert *v1, OptTriVert *v2, OptTriVert *v3, int threadnr) {
  OptTri *tri = BLI_safepool_alloc(tm->pools[POOL_TRI], threadnr);

  memset(tri, 0, sizeof(*tri));

  tri->v1 = v1;
  tri->v2 = v2;
  tri->v3 = v3;

  tri->e1 = edge_add_tri(tm, v1, v2, tri, threadnr);
  tri->e2 = edge_add_tri(tm, v2, v3, tri, threadnr);
  tri->e3 = edge_add_tri(tm, v3, v1, tri, threadnr);

  return tri;
}

OptTri *BLI_trimesh_get_edge(OptTriMesh *tm, OptTriVert *v1, OptTriVert *v2, int threadnr) {
  return ensure_edge(tm, v1, v2, threadnr);
}

void BLI_trimesh_add(OptTriMesh *tm, float* vertCos, float* vertNos, int totvert, int* triIndices, int tottri, int threadnr) {
  float* vco, * vno;
  BLI_ThreadSafePool *vpool = tm->pools[POOL_VERTEX];
  BLI_ThreadSafePool *fpool = tm->pools[POOL_EDGE];
  BLI_ThreadSafePool *epool = tm->pools[POOL_TRI];
  int i;

  //abuse the normals array to store pointers?
  //or just allocate one?

  OptTriVert** vmap = MEM_mallocN(sizeof(*vmap) * totvert, "BLI_trimesh_add:vmap temporary");
  //OptTriVertex** vmap = (OptTriVertex**)vertNos;

  vco = vertCos;
  vno = vertNos;

  for (i = 0; i < totvert; i++, vco += 3, vno += 3) {
    OptTriVert*v = BLI_safepool_alloc(vpool, threadnr);
    vmap[i] = v;

    copy_v3_v3(v->co, vco);
    copy_v3_v3(v->no, vno);
  }

  int *tris = triIndices;
  for (i = 0; i < tottri; triIndices += 3) {
    OptTriVert *v1 = vmap[triIndices[0]];
    OptTriVert *v2 = vmap[triIndices[1]];
    OptTriVert *v3 = vmap[triIndices[2]];

    OptTri *tri = BLI_safepool_alloc(tm->pools[POOL_TRI], threadnr);

    tri->v1 = v1;
    tri->v2 = v2;
    tri->v3 = v3;

    tri->e1 = ensure_edge(tm, v1, v2, threadnr);
    tri->e2 = ensure_edge(tm, v2, v3, threadnr);
    tri->e3 = ensure_edge(tm, v3, v1, threadnr);
  }

  MEM_freeN(vmap);
}


//we do somewhat weird things with stack, it's returned by this function
static void **trimesh_tag_step(OptTriMesh* tm, OptTriVert* v, void** stack, int tag, int maxelem) {
  BLI_array_declare(stack);

  v->threadtag = tag;
  int totelem = 0;

  for (int i = 0; i < v->edges.length; i++) {
    OptTriEdge *e = v->edges.items[i];

    for (int j = 0; j < e->tris.length; j++) {
      OptTri *tri = e->tris.items[j];

      if (tri->threadtag == TRIMESH_NEED_TAG) {
        tri->threadtag = tag;
        BLI_array_append(stack, tri);
        totelem++;
      }
    }
  }

  while (BLI_array_len(stack) && totelem < maxelem) {
    OptTri *tri = BLI_array_pop(stack);

    for (int i = 0; i < 3; i++) {
      OptTriEdge *e = i == 0 ? tri->e1 : (i == 1 ? tri->e2 : tri->e3);

      for (int j = 0; j < e->tris.length; j++) {
        OptTri *tri2 = e->tris.items[j];

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
void BLI_trimesh_thread_tag(OptTriMesh *tm, OptTri** tris, int tottri) {
  void **stack = NULL;
  BLI_array_declare(stack);

  if (tottri == -1) {
    int maxtag = MAX2(tm->tottri / tm->maxthread, 1);

    ThreadSafePoolIter iter;

    BLI_safepool_iternew(tm->pools[POOL_TRI], &iter);
    OptTri *t = BLI_safepool_iterstep(&iter);
    for (; t; t = BLI_safepool_iterstep(&iter)) {
      t->threadtag = TRIMESH_NEED_TAG;
    }
    BLI_safepool_iterfree(&iter);

    bool stop = false;

    int tag = 0;

    while (!stop) {
      stop = true;

      BLI_safepool_iternew(tm->pools[POOL_VERTEX], &iter);
      OptTriVert *v = BLI_safepool_iterstep(&iter);

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
    OptTri *tri = tris[i];

    tri->threadtag = TRIMESH_NEED_TAG;
    tri->v1->threadtag = TRIMESH_NEED_TAG;
    tri->v2->threadtag = TRIMESH_NEED_TAG;
    tri->v3->threadtag = TRIMESH_NEED_TAG;
  }

  bool stop = false;
  while (1) {
    stop = true;

    for (int i = 0; i < tottri; i++) {
      OptTri *tri = tris[i];

      if (tri->threadtag != TRIMESH_NEED_TAG) {
        continue;
      }

      for (int j = 0; j < 3; j++) {
        OptTriVert *v = (&tri->v1)[j];

        if (v->threadtag == TRIMESH_NEED_TAG) {
          stack = trimesh_tag_step(tm, v, stack, tag, maxelem);
          tag = (tag + 1) % maxtag;
        }
      }
    }
  }

  BLI_array_free(stack);
}

void BLI_trimesh_clear_threadtags(OptTriMesh *tm) {
  for (int i=0; i<3; i++) {
    ThreadSafePoolIter iter;
    OptElem *item;

    BLI_safepool_iternew(tm->pools[i], &iter);
    item = BLI_safepool_iterstep(&iter);

    for (; item; item = BLI_safepool_iterstep(&iter)) {
      item->threadtag = TRIMESH_TAG_CLEAR;
    }
  }
}

void BLI_trimesh_tag_thread_boundaries(OptTriMesh *tm, OptTri **tris, int tottri) {
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

void BLI_trimesh_tag_thread_boundaries_once(OptTriMesh *tm, OptTri **tris, int tottri) {
  for (int i = 0; i < tottri; i++) {
    OptTri *tri = tris[i];

    //avoid double tagging
    if (tri->threadtag == TRIMESH_BOUNDARY) {
      continue;
    }

    for (int j = 0; j < 3; j++) {
      OptTriEdge *e = TRIEDGE(tri, j);
      for (int k = 0; k < e->tris.length; k++) {
        OptTri *tri2 = e->tris.items[k];

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
void BLI_trimesh_build_islands(OptTriMesh *tm, OptTri **tris, int tottri, OptTriIsland** r_islands, int *r_totisland) {
  OptTriIsland *islands = *r_islands = MEM_callocN((tm->maxthread+1)*sizeof(*islands), "OptTriIsland");

  *r_totisland = tm->maxthread+1;

  for (int i = 0; i < tottri; i++) {
    OptTri *tri = tris[i];
    int threadnr = tri->threadtag;

    if (threadnr == TRIMESH_BOUNDARY) {
      threadnr = tm->maxthread;
    } else if (threadnr < 0 || threadnr >= tm->maxthread) {
      fprintf(stderr, "Bad thread tag\n");
      continue;
    }

    OptTriIsland *island = islands + threadnr;

    OptTri **list = island->tris;
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

typedef void (*OptTriMeshJob)(OptTriMesh *tm, void **elements, int totelem, int threadnr, void *userdata);
typedef struct threadjob {
  OptTriMesh *tm;
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

void BLI_trimesh_foreach_tris(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata) {
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

static void for_vert_callback(OptTriMesh *tm, OptTri **tris, int tottri, int threadnr, foreach_vert_data *userdata) {
  OptTriVert **verts = NULL;
  BLI_array_declare(verts);

  for (int i=0; i<tottri; i++) {
    tris[i]->v1->threadtag = TRIMESH_TAG_CLEAR;
    tris[i]->v2->threadtag = TRIMESH_TAG_CLEAR;
    tris[i]->v3->threadtag = TRIMESH_TAG_CLEAR;
  }

  for (int i=0; i<tottri; i++) {
    OptTri *tri = tris[i];

    for (int j=0; j<3; j++) {
      OptTriVert *v = TRIVERT(tri, j);

      if (v->threadtag == TRIMESH_TAG_CLEAR) {
        v->threadtag = 1;

        BLI_array_append(verts, v);
      }
    }
  }


  if (BLI_array_len(verts)) {
    userdata->job(tm, verts, BLI_array_len(verts), threadnr, userdata->userdata);
  }
}

void BLI_trimesh_foreach_verts(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata) {
  foreach_vert_data data;

  data.job = job;
  data.userdata = userdata;

  //void BLI_trimehs_foreach_tris(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata) {
  BLI_trimesh_foreach_tris(tm, tris, tottri, for_vert_callback, maxthread, &data); 
}

void BLI_trimesh_kill_vert(OptTriMesh *tm, OptTriVert *v, int threadnr) {
  while (v->edges.length > 0) {
    BLI_trimesh_kill_edge(tm, v->edges.items[0], false);
  }

  simplelist_free(tm, &v->edges, POOL_ELIST, threadnr);
  BLI_safepool_free(tm->pools[POOL_VERTEX], v);
}

//if kill_verts is true verts with no edges will be deleted
void BLI_trimesh_kill_edge(OptTriMesh *tm, OptTriEdge *e, int threadnr, bool kill_verts) {
  while (e->tris.length > 0) {
    OptTri *tri = e->tris.items[0];
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

  simplelist_free(tm, &e->tris, POOL_TLIST, threadnr);
  BLI_safepool_free(tm->pools[POOL_EDGE], e);
}

//kill_edges/verts is whether to automatically kill verts/edges that belong to no triangles
//note that threadnr doesn't refer to whichever thread created tri, but the calling thread
void BLI_trimesh_kill_tri(OptTriMesh *tm, OptTri *tri, int threadnr, bool kill_edges, bool kill_verts) {
  //static void simplelist_remove(OptTriMesh *tm, optmesh_simplelist *list, void *item, int pool, int threadnr) {

  simplelist_remove(tm, &tri->e1->tris, tri, POOL_TLIST, threadnr);
  simplelist_remove(tm, &tri->e2->tris, tri, POOL_TLIST, threadnr);
  simplelist_remove(tm, &tri->e3->tris, tri, POOL_TLIST, threadnr);

  if (tri->e1->tris.length == 0) {
    BLI_trimesh_kill_edge(tm, tri->e1, threadnr, kill_verts);
  }
  if (tri->e2->tris.length == 0) {
    BLI_trimesh_kill_edge(tm, tri->e2, threadnr, kill_verts);
  }
  if (tri->e3->tris.length == 0) {
    BLI_trimesh_kill_edge(tm, tri->e3, threadnr, kill_verts);
  }

  BLI_safepool_free(tm, tm->pools[POOL_TRI], tri);
}

static void weld_verts(OptTriMesh *tm, OptTriVert *v1, OptTriVert *v2, int threadnr) {
  for (int i=0; i<v2->edges.length; i++) {
    OptTriEdge *e = v2->edges.items[i];
    OptTriVert *vb = OTHER_VERT(e, v1);

    if (vb == v1) {
      //not sure this will ever happen; force deletion of edge?
      fprintf(stdout, "auto deleting edge in BLI_trimesh code\n");
      fflush(stdout);

      BLI_trimesh_kill_edge(tm, e, threadnr, false);

      continue;
    }

    //swap out vertex references for triangles
    for (int j=0; j<e->tris.length; j++) {
      OptTri *tri = e->tris.items[j];
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

void BLI_trimesh_collapse_edge(OptTriMesh *tm, OptTriEdge *e, int threadnr) {
  OptTriVert *v1 = e->v1, *v2 = e->v2;

  BLI_trimesh_kill_edge(tm, e, threadnr, false);
  weld_verts(tm, v1, v2, threadnr);
}

void BLI_trimesh_calc_tri_normal(OptTriMesh *tm, OptTri *tri, int threadnr) {
  normal_tri_v3(tri->no, tri->v1->co, tri->v2->co, tri->v3->co);
}

void BLI_trimesh_calc_vert_normal(OptTriMesh *em, OptTriVert *v, int threadnr, bool recalc_tri_normals) {
  int tot;

  zero_v3(v->no);

  for (int i=0; i<v->edges.length; i++) {
    OptTriEdge *e = v->edges.items[i];

    for (int j=0; j<e->tris.length; j++) {
      OptTri *tri = e->tris.items[j];

      if (recalc_tri_normals) {
        normal_tri_v3(tri->no, tri->v1->co, tri->v2->co, tri->v3->co);
      }

      add_v3_v3(v->no, tri->no);
      tot++;
    }
  }

  if (tot == 0) {
    v->no[2] = 1.0f;
  } else {
    normalize_v3(v->no);
  }
}

OptTriVert *BLI_trimesh_split_edge(OptTriMesh *tm, OptTriEdge *e, int threadnr, float fac) {
  float co[3];
  float no[3];

  interp_v3_v3v3(co, e->v1->co, e->v2->co, fac);
  add_v3_v3v3(no, e->v1->no, e->v2->no);
  normalize_v3(no);

  OptTriVert *vc = BLI_trimesh_make_vert(tm, co, no, threadnr);

  for (int i=0; i<e->tris.length; i++) {
    OptTri *tri = e->tris.items[0];

    OptTriVert *tv1, *tv2, *tv3;
    int vi = 0;

    for (int i=0; j<3; j++) {
      if (TRIEDGE(tri, j) == e) {
        vi = j;
        break;
      }
    }

    tv1 = TRIVERT(tri, vi);
    tv1 = TRIVERT(tri, (vi+1)%3);
    tv1 = TRIVERT(tri, (vi+2)%3);

    OptTri *t1 = BLI_trimesh_make_tri(tm, tv1, vc, tv3, threadnr);
    OptTri *t2 = BLI_trimesh_make_tri(tm, vc, tv2, tv3, threadnr);

    BLI_trimesh_kill_tri(tm, tri, threadnr, false, false);
  }

  BLI_trimesh_kill_edge(tm, e, threadnr, false);
  BLI_trimesh_calc_vert_normal(tm, v, threadnr, true);

  return vc;
}
