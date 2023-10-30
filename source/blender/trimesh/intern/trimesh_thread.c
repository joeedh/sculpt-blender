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
 * \ingroup trimesh
 *
 * optimized thread-safe triangle mesh library with topological info
 *
 */

#include <stdlib.h>
#include <string.h>

#include "BLI_listbase.h"

#include "BLI_array.h"
#include "BLI_math.h"
#include "BLI_task.h"
#include "BLI_threadsafe_mempool.h"
#include "trimesh.h"

#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BLI_utildefines.h"
#include "atomic_ops.h"

#include "BKE_customdata.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_multires.h"

#include "BKE_key.h"
#include "BKE_main.h"

#include "MEM_guardedalloc.h"
#ifdef WITH_MEM_VALGRIND
#  include "valgrind/memcheck.h"
#endif

#include "trimesh_private.h"

// we do somewhat weird things with stack, it's returned by this function
static void **trimesh_tag_step(TM_TriMesh *tm, TMVert *v, void **stack, int tag, int maxelem)
{
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

// if tottris is -1 then all triangles will be tagged
void TM_thread_tag(TM_TriMesh *tm, TMFace **tris, int tottri)
{
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

  int maxelem = tottri / tm->maxthread;
  maxelem = MIN2(MAX2(maxelem, 512), tottri);
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
  while (!stop) {
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

void TM_clear_threadtags(TM_TriMesh *tm)
{
  for (int i = 0; i < 3; i++) {
    ThreadSafePoolIter iter;
    TMElement *item;

    BLI_safepool_iternew(tm->pools[i], &iter);
    item = BLI_safepool_iterstep(&iter);

    for (; item; item = BLI_safepool_iterstep(&iter)) {
      item->threadtag = TRIMESH_TAG_CLEAR;
    }
  }
}

void TM_tag_thread_boundaries(TM_TriMesh *tm, TMFace **tris, int tottri)
{
  // propegate boundary tag twice
  for (int i = 0; i < 2; i++) {
    TM_tag_thread_boundaries_once(tm, tris, tottri);

    // needed to avoid triggering double tagging detection code
    for (int i = 0; i < tottri; i++) {
      tris[i]->threadtag = TRIMESH_BOUNDARY_TEMP;
    }
  }

  for (int i = 0; i < tottri; i++) {
    if (tris[i]->threadtag == TRIMESH_BOUNDARY_TEMP) {
      tris[i]->threadtag = TRIMESH_BOUNDARY;
    }
  }
}

void TM_tag_thread_boundaries_once(TM_TriMesh *tm, TMFace **tris, int tottri)
{
  for (int i = 0; i < tottri; i++) {
    TMFace *tri = tris[i];

    // avoid double tagging
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

// called after BLI_trimesh_thread_tag
// last island is always boundary triangles
void TM_build_islands(
    TM_TriMesh *tm, TMFace **tris, int tottri, TMTriIsland **r_islands, int *r_totisland)
{
  TMTriIsland *islands = *r_islands = MEM_callocN((tm->maxthread + 1) * sizeof(*islands),
                                                  "OptTriIsland");

  *r_totisland = tm->maxthread + 1;

  for (int i = 0; i < tottri; i++) {
    TMFace *tri = tris[i];
    int threadnr = tri->threadtag;

    if (threadnr == TRIMESH_BOUNDARY) {
      threadnr = tm->maxthread;
    }
    else if (threadnr < 0 || threadnr >= tm->maxthread) {
      fprintf(stderr, "Bad thread tag\n");
      continue;
    }

    TMTriIsland *island = islands + threadnr;

    TMFace **list = island->tris;
    BLI_array_declare(list);
    BLI_array_len_set(list, island->tottri);

    BLI_array_append(list, tri);

    island->tris = list;
    island->tottri = BLI_array_len(list);
  }

  if (r_totisland) {
    *r_totisland = tm->maxthread;
  }
}

void TM_free_islands(TMTriIsland *islands, int totisland, bool free_islands)
{
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

typedef struct threadjob {
  TM_TriMesh *tm;
  OptTriMeshJob job;
  void **elems;
  int totelem;
  void *userdata;
  bool done;
} threadjob;

static void thread_job(void *__restrict userdata, const int n, const TaskParallelTLS *__restrict tls) {
  threadjob *job = ((threadjob*)userdata) + n;

  if (job->totelem == 0) {
    job->done = true;
    return;
  }

  //XXX remove threadnr argument
  job->job(job->tm, job->elems, job->totelem, 0, job->userdata);
  job->done = true;
}

void TM_foreach_tris(
    TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata)
{
  //XXX remove maxthread parameter
  maxthread = tm->maxthread;
  //tm->maxthread = maxthread;

  TMTriIsland *islands;
  int totisland;

  TM_thread_tag(tm, tris, tottri);
  TM_build_islands(tm, tris, tottri, &islands, &totisland);
  TM_tag_thread_boundaries(tm, tris, tottri);

  threadjob *jobs = MEM_callocN(sizeof(threadjob) * totisland, "threadjob");

  for (int i = 0; i < totisland; i++) {
    islands[i].tag = 0;
  }

  for (int i = 0; i < totisland; i++) {
    jobs[i].elems = islands[i].tris;
    jobs[i].totelem = islands[i].tottri;
    jobs[i].userdata = userdata;

    jobs[i].job = job;
    jobs[i].done = false;
    jobs[i].tm = tm;
  }

  if (totisland > 1) {
    TaskParallelSettings settings;
    BLI_parallel_range_settings_defaults(&settings);
    BLI_task_parallel_range(0, totisland - 1, jobs, thread_job, &settings);
  }

  // do triangles on thread island boundaries last
  if (totisland > 0) {
    TMTriIsland *island = islands + totisland - 1;

    job(tm, island->tris, island->tottri, 0, userdata);
  }

  MEM_freeN(jobs);
  TM_free_islands(islands, totisland, true);

  for (int i = 0; i < tottri; i++) {
    tris[i]->threadtag = TRIMESH_TAG_CLEAR;
  }
}

typedef struct foreach_vert_data {
  OptTriMeshJob job;
  void *userdata;
} foreach_vert_data;

static void for_vert_callback(
    TM_TriMesh *tm, TMFace **tris, int tottri, int threadnr, foreach_vert_data *userdata)
{
  TMVert **verts = NULL;
  BLI_array_declare(verts);

  for (int i = 0; i < tottri; i++) {
    tris[i]->v1->threadtag = TRIMESH_TAG_CLEAR;
    tris[i]->v2->threadtag = TRIMESH_TAG_CLEAR;
    tris[i]->v3->threadtag = TRIMESH_TAG_CLEAR;
  }

  for (int i = 0; i < tottri; i++) {
    TMFace *tri = tris[i];

    for (int j = 0; j < 3; j++) {
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

void TM_foreach_verts(
    TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata)
{
  foreach_vert_data data;

  data.job = job;
  data.userdata = userdata;

  // void BLI_trimehs_foreach_tris(OptTriMesh *tm, OptTri **tris, int tottri, OptTriMeshJob job,
  // int maxthread, void *userdata) {
  TM_foreach_tris(tm, tris, tottri, for_vert_callback, maxthread, &data);
}
