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
#include "BLI_hashmap.h"
#include "BLI_math.h"
#include "BLI_threads.h"
#include "BLI_threadsafe_mempool.h"
#include "trimesh.h"

#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "atomic_ops.h"

#include "BLI_utildefines.h"

#include "BKE_customdata.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_multires.h"

#include "BKE_global.h"
#include "BKE_key.h"
#include "BKE_main.h"

#include "MEM_guardedalloc.h"
#ifdef WITH_MEM_VALGRIND
#  include "valgrind/memcheck.h"
#endif

#include "trimesh_private.h"

typedef struct LogEntry {
  union {
    void *ptr;
    float vec3[3];
    float f;
    int i;
    short hvec3[3];
    int ivec3[3];
    int ivec4[4];
  } value;
} LogEntry;

typedef struct TriMeshLog {
  BLI_HashMap(HashInt, HashP) * elemhash_ptr;      // maps ids to pointers
  BLI_HashMap(HashP, HashInt) * elemhash_id;       // maps pointers to ids
  BLI_HashMap(HashInt, HashInt) * elemhash_entry;  // maps ids to entries indices

  LogEntry *entries;
  int *groups;
  int totgroup;
  int totentries;
  int idgen;

  TM_TriMesh *tm;

  int cd_vert_mask_index;

  ThreadRWMutex lock;

  int curgroup;
} TriMeshLog;

ThreadLocal(bool) havelock = false;

static void tmlog_lock(TriMeshLog *log, int mode)
{
  if (G.debug_value == 112) {
    return;
  }

  if (havelock) {
    return;
  }

  BLI_rw_mutex_lock(&log->lock, mode);
  havelock = true;
}

static void tmlog_unlock(TriMeshLog *log)
{
  if (G.debug_value == 112) {
    return;
  }

  if (!havelock) {
    return;
  }
  BLI_rw_mutex_unlock(&log->lock);
  havelock = false;
}

static void trimesh_add_group(TriMeshLog *log, bool set_curgroup)
{
  int *groups = log->groups;
  BLI_array_declare(groups);

  BLI_array_len_set(groups, log->totgroup);
  BLI_array_append(groups, log->totentries);

  log->groups = groups;
  log->totgroup = BLI_array_len(groups);

  if (set_curgroup) {
    log->curgroup = log->totgroup - 1;
  }
}

TriMeshLog *TM_log_new(TM_TriMesh *tm, int cd_vert_mask_index)
{
  TriMeshLog *log = MEM_callocN(sizeof(*log), "TriMeshLog");

  BLI_rw_mutex_init(&log->lock);

  log->elemhash_ptr = BLI_hashmap_new(HashInt, HashP)();
  log->elemhash_id = BLI_hashmap_new(HashP, HashInt)();
  log->elemhash_entry = BLI_hashmap_new(HashInt, HashInt)();
  log->cd_vert_mask_index = cd_vert_mask_index;
  log->idgen = 1;

  log->tm = tm;

  trimesh_add_group(log, true);

  return log;
}

void TM_log_free(TriMeshLog *log)
{
  BLI_rw_mutex_end(&log->lock);

  BLI_hashmap_free(HashP, HashInt)(log->elemhash_id);
  BLI_hashmap_free(HashInt, HashP)(log->elemhash_ptr);
  BLI_hashmap_free(HashInt, HashInt)(log->elemhash_entry);
}

static void tlog_truncate(TriMeshLog *log)
{
  if (log->curgroup == log->totgroup - 1) {
    return;
  }

  log->totentries = log->groups[log->curgroup + 1];
  log->totgroup = log->curgroup + 1;
}

static LogEntry *tlog_push(TriMeshLog *log)
{
  // tmlog_lock(log, THREAD_LOCK_WRITE);

  LogEntry *entries = log->entries;
  LogEntry e = {
      0,
  };

  BLI_array_declare(entries);

  if (log->curgroup != log->totgroup) {
    tlog_truncate(log);
  }

  BLI_array_append(entries, e);

  log->entries = entries;
  log->totentries = BLI_array_len(entries);

  LogEntry *ret = log->entries + log->totentries - 1;
  // tmlog_unlock(log);

  return ret;
}

static void tlog_f(TriMeshLog *log, float f)
{
  LogEntry *e = tlog_push(log);
  e->value.f = f;
}

static void tlog_v3(TriMeshLog *log, float f[3])
{
  LogEntry *e = tlog_push(log);
  copy_v3_v3(e->value.vec3, f);
}

static void tlog_i3(TriMeshLog *log, int f[3])
{
  LogEntry *e = tlog_push(log);

  e->value.ivec3[0] = f[0];
  e->value.ivec3[1] = f[1];
  e->value.ivec3[2] = f[2];
}

static void tlog_i4(TriMeshLog *log, int f[4])
{
  LogEntry *e = tlog_push(log);

  e->value.ivec4[0] = f[0];
  e->value.ivec4[1] = f[2];
  e->value.ivec4[2] = f[3];
  e->value.ivec4[3] = f[4];
}

static void tlog_i(TriMeshLog *log, int f)
{
  LogEntry *e = tlog_push(log);
  e->value.i = f;
}

static void tlog_ptr(TriMeshLog *log, void *f)
{
  LogEntry *e = tlog_push(log);
  e->value.ptr = f;
}

static int tlog_start(TriMeshLog *log, int code)
{
  int i = log->totentries;

  tmlog_lock(log, THREAD_LOCK_WRITE);
  tlog_i(log, code);
  // tmlog_unlock(log);

  return i;
}

static void tlog_end(TriMeshLog *log, int entry_i)
{
  // tmlog_lock(log, THREAD_LOCK_WRITE);
  tlog_i(log, entry_i);
  tmlog_unlock(log);
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
    TM_ELEM_CD_SET_FLOAT(v, cd_vert_mask_offset, new_mask);
  }
}

static float vert_mask_get(TMVert *v, const int cd_vert_mask_offset)
{
  if (cd_vert_mask_offset != -1) {
    return TM_ELEM_CD_GET_FLOAT(v, cd_vert_mask_offset);
  }
  else {
    return 0.0f;
  }
}

static void elemhash_add(TriMeshLog *log, void *elem, int id, int entryidx)
{
  tmlog_lock(log, THREAD_LOCK_WRITE);
  BLI_HashMapIter(HashInt, HashP) iter;

  /*
  BLI_HASH_ITER(log->elemhash_ptr, iter, HashInt, HashP) {
    printf("%d %p\n", BLI_hashiter_key(iter), BLI_hashiter_value(iter));

  } BLI_HASH_ITER_END
  */

  BLI_hashmap_insert(HashP, HashInt)(log->elemhash_id, (void *)id, elem);
  BLI_hashmap_insert(HashInt, HashP)(log->elemhash_ptr, elem, (void *)id);
  BLI_hashmap_insert(HashInt, HashInt)(log->elemhash_entry, (void *)id, (void *)entryidx);
  tmlog_unlock(log);
}

static void *elemhash_lookup_id(TriMeshLog *log, int id)
{
  return BLI_hashmap_lookup(HashP, HashInt)(log->elemhash_id, (void *)id);
}

static int elemhash_get_id(TriMeshLog *log, void *elem)
{
  void *ret = NULL;

  // XXX lock contention issue?
  tmlog_lock(log, THREAD_LOCK_READ);
  bool exists = BLI_hashmap_lookup_p(HashInt, HashP)(log->elemhash_ptr, elem, &ret);
  tmlog_unlock(log);

  if (!exists) {
    return -1;
  }

  return (int)ret;
}

static int elemhash_ensure_id(TriMeshLog *log, void *elem)
{
  void *ret = NULL;

  tmlog_lock(log, THREAD_LOCK_READ);
  bool exists = BLI_hashmap_lookup_p(HashInt, HashP)(log->elemhash_ptr, elem, &ret);

  if (!exists) {
    int id = log->idgen++;
    tmlog_unlock(log);

    elemhash_add(log, elem, id, -1);

    return id;
  }
  else {
    int ret2 = (int)ret;
    tmlog_unlock(log);
    return ret2;
  }
}

static int elemhash_get_entry(TriMeshLog *log, int id)
{
  void *ret = NULL;

  tmlog_lock(log, THREAD_LOCK_READ);
  bool exists = BLI_hashmap_lookup_p(HashInt, HashP)(log->elemhash_ptr, (void *)id, &ret);

  int ret2 = exists ? (int)ret : -1;
  tmlog_unlock(log);

  return ret2;
}

int TM_log_vert_add(TriMeshLog *log, TMVert *v, const int cd_mask_offset, bool skipcd)
{
  int start = tlog_start(log, LOG_VERT_ADD);

  int id = log->idgen++;
  elemhash_add(log, v, id, log->totentries);

  tlog_i(log, id);
  tlog_v3(log, v->co);
  tlog_v3(log, v->no);
  tlog_f(log, vert_mask_get(v, cd_mask_offset));
  tlog_i(log, skipcd);

  tlog_end(log, start);

  return id;
}

int elemhash_has_id(TriMeshLog *log, void *elem)
{
  return elemhash_get_id(log, elem) != NULL;
}

int elemhash_get_vert_id(TriMeshLog *log, TMVert *v, int cd_vert_mask_offset)
{
  int ret = elemhash_get_id(log, v);

  if (ret < 0) {
    return TM_log_vert_add(log, v, cd_vert_mask_offset, false);
  }

  return ret;
}

int BLI_trimesh_log_edge_add(TriMeshLog *log, TMEdge *e, const int cd_mask_offset, int skipcd)
{
  int v1id = elemhash_get_vert_id(log, e->v1, cd_mask_offset);
  int v2id = elemhash_get_vert_id(log, e->v2, cd_mask_offset);

  int start = tlog_start(log, LOG_EDGE_ADD);
  int id = log->idgen++;
  elemhash_add(log, e, id, log->totentries);

  tlog_i(log, id);
  tlog_i(log, v1id);
  tlog_i(log, v2id);
  tlog_i(log, skipcd);

  tlog_end(log, start);

  return id;
}

static int elemhash_get_edge_id(TriMeshLog *log, TMEdge *e, int cd_vert_mask_offset)
{
  int ret = elemhash_get_id(log, e);

  if (!e) {
    return BLI_trimesh_log_edge_add(log, e, cd_vert_mask_offset, false);
  }

  return ret;
}

static int trimesh_log_cdata(TriMeshLog *log, void *velem, int type)
{
  TMElement *elem = velem;
  CustomData *cdata = trimesh_get_customdata(log->tm, type);

  tmlog_lock(log, THREAD_LOCK_WRITE);

  int size = !elem->customdata ? 0 : cdata->totsize;
  if (!size) {
    tlog_i(log, 0);
    tlog_i(log, 0);
    tmlog_unlock(log);
    return -1;
  }

  tlog_i(log, size);

  int totentries = (size >> 2) + 1;
  int cur = 0;

  tlog_i(log, totentries);

  char *addr = (char *)elem->customdata;

  for (int i = 0; i < totentries; i++) {
    int tot;

    if (i == totentries - 1) {
      tot = size % 4;
    }
    else {
      tot = 4;
    }

    for (int j = 0; j < tot; j++) {
      int iv[4];

      memcpy(iv, addr, sizeof(int) * tot);
      addr += sizeof(int) * tot;

      tlog_i4(log, iv);
    }
  }

  tmlog_unlock(log);
  return 0;
}

static int trimesh_log_read_cdata(TriMeshLog *tlog, int entry_i, void *velem, int type)
{
  CustomData *cdata = trimesh_get_customdata(tlog->tm, type);
  TMElement *elem = velem;
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int size = log[i--].value.i;
  int totchunk = log[i--].value.i;

  void *data = elem->customdata;
  char *addr = (char *)data;

  for (int j = 0; j < totchunk; j++) {
    int tot;
    int *iv = log[i--].value.ivec4;

    if (i == totchunk - 1) {
      tot = size % 4;
    }
    else {
      tot = 4;
    }

    memcpy(addr, iv, sizeof(int) * tot);
    addr += sizeof(int) * tot;
  }

  return i;
}

static int trimesh_log_skip_cdata(TriMeshLog *tlog, int entry_i)
{
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int size = log[i--].value.i;
  int totchunk = log[i--].value.i;

  return i - totchunk;
}

static int trimesh_skip_loop(TriMeshLog *log, int entry_i)
{
  return trimesh_log_skip_cdata(log, entry_i + 1);
}

static int trimesh_log_loop(TriMeshLog *log, TMFace *tri, TMLoopData *loop)
{
  trimesh_log_cdata(log, loop, TM_LOOP);
  return 0;
}

static int trimesh_read_loop(TriMeshLog *tlog, TMLoopData *l, int entry_i)
{
  int i = entry_i;

  i++;  // skip entry tag
  i = trimesh_log_read_cdata(tlog, i, l, TM_LOOP);

  return i;
}

int BLI_trimesh_log_tri(TriMeshLog *log, TMFace *tri, bool skipcd)
{

  int cd_vert_mask_offset = log->cd_vert_mask_index;

  int e1 = elemhash_get_edge_id(log, tri->e1, cd_vert_mask_offset);
  int e2 = elemhash_get_edge_id(log, tri->e2, cd_vert_mask_offset);
  int e3 = elemhash_get_edge_id(log, tri->e3, cd_vert_mask_offset);

  int v1 = elemhash_get_vert_id(log, tri->v1, cd_vert_mask_offset);
  int v2 = elemhash_get_vert_id(log, tri->v1, cd_vert_mask_offset);
  int v3 = elemhash_get_vert_id(log, tri->v1, cd_vert_mask_offset);

  int start = tlog_start(log, LOG_TRI_ADD);
  int id = log->idgen++;
  elemhash_add(log, tri, id, log->totentries);

  tlog_i(log, id);
  tlog_i(log, skipcd);

  tlog_i(log, v1);
  tlog_i(log, v2);
  tlog_i(log, v3);

  tlog_i(log, e1);
  tlog_i(log, e2);
  tlog_i(log, e3);

  if (!skipcd) {
    trimesh_log_loop(log, tri, tri->l1);
    trimesh_log_loop(log, tri, tri->l2);
    trimesh_log_loop(log, tri, tri->l3);
  }

  tlog_end(log, start);

  return id;
}

static int elemhash_get_tri_id(TriMeshLog *log, TMFace *tri)
{
  int ret = elemhash_get_id(log, tri);

  if (!ret) {
    return BLI_trimesh_log_tri(log, tri, false);
  }

  return ret;
}

int BLI_trimesh_log_vert_kill(TriMeshLog *log, TMVert *v)
{
  int id = elemhash_get_id(log, v);

  if (!id) {
    return -1;  // invalid element
  }

  // make sure edge entries exist
  for (int i = 0; i < v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];
    elemhash_get_edge_id(log, e, log->cd_vert_mask_index);
  }

  int start = tlog_start(log, LOG_VERT_KILL);
  tlog_i(log, id);

  // create a wind list
  tlog_i(log, v->edges.length);
  for (int i = 0; i < v->edges.length; i++) {
    TMEdge *e = v->edges.items[i];
    tlog_i(log, elemhash_get_edge_id(log, e, log->cd_vert_mask_index));
  }

  tlog_end(log, start);

  return id;
}

int BLI_trimesh_log_edge_kill(TriMeshLog *log, TMEdge *e, int kill_verts)
{
  int id = elemhash_get_id(log, e);

  if (!id) {
    return -1;  // invalid element
  }

  // make sure tri entries exist
  for (int i = 0; i < e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    elemhash_get_tri_id(log, tri);
  }

  int start = tlog_start(log, LOG_EDGE_KILL);
  tlog_i(log, id);
  tlog_i(log, kill_verts);

  tlog_i(log, e->tris.length);
  for (int i = 0; i < e->tris.length; i++) {
    TMFace *tri = e->tris.items[i];

    tlog_i(log, elemhash_get_tri_id(log, tri));
  }

  tlog_end(log, start);
  return id;
}

int BLI_trimesh_log_tri_kill(TriMeshLog *log, TMFace *tri, int kill_verts, int kill_edges)
{
  int id = elemhash_get_id(log, tri);

  if (!id) {
    return -1;  // invalid element
  }

  int start = tlog_start(log, LOG_TRI_KILL);
  tlog_i(log, id);
  tlog_i(log, kill_verts);
  tlog_i(log, kill_edges);

  // make wind list
  tlog_i(log, 6);
  tlog_i(log, elemhash_get_edge_id(log, tri->e1, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_edge_id(log, tri->e2, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_edge_id(log, tri->e3, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_vert_id(log, tri->v1, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_vert_id(log, tri->v2, log->cd_vert_mask_index));
  tlog_i(log, elemhash_get_vert_id(log, tri->v3, log->cd_vert_mask_index));

  trimesh_log_cdata(log, tri->l1, TM_LOOP);
  trimesh_log_cdata(log, tri->l2, TM_LOOP);
  trimesh_log_cdata(log, tri->l3, TM_LOOP);

  tlog_end(log, start);
  return id;
}

void BLI_trimesh_log_split_edge(
    TriMeshLog *log, TMEdge *e, float fac, TMVert *newvert, int cd_vert_mask_index)
{
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

void BLI_trimesh_log_collapse_edge(TriMeshLog *log, TMEdge *e)
{
  int start = tlog_start(log, LOG_COLLAPSE_EDGE);
  tlog_i(log, elemhash_get_id(log, e->v1));
  tlog_i(log, elemhash_get_id(log, e->v2));

  tlog_end(log, start);
}

// based on BM_log_vert_before_modified
// saves coordinates and flags
int BLI_trimesh_log_vert_state(TriMeshLog *log, TMVert *v)
{
  int id = elemhash_get_vert_id(log, v, log->cd_vert_mask_index);

  int start = tlog_start(log, LOG_VERT_STATE);
  int ivec[3] = {v->flag, v->index, v->threadtag};

  tlog_i(log, id);
  tlog_v3(log, v->co);
  tlog_v3(log, v->no);
  tlog_i3(log, ivec);
  tlog_f(log, TM_ELEM_CD_GET_FLOAT(v, log->cd_vert_mask_index));

  tlog_end(log, start);

  return id;
}

void BLI_log_add_setpoint(TriMeshLog *log, int setgroup)
{
  tlog_truncate(log);
  trimesh_add_group(log, setgroup);
}

static int meshlog_wind(TriMeshLog *tlog, int entry_i, int threadnr)
{
  int i = entry_i;

  LogEntry *log = tlog->entries;
  LogEntry *item = &log[i++];

  switch (item->value.i) {
    case LOG_VERT_STATE: {
      int id = log[i++].value.i;
      float *co = log[i++].value.vec3;
      float *no = log[i++].value.vec3;
      int *ivec = log[i++].value.ivec3;
      float mask = log[i++].value.f;

      TMVert *v = elemhash_lookup_id(tlog, id);
      break;
    }

    case LOG_VERT_KILL: {
      int id = log[i++].value.i;
      TMVert *v = elemhash_lookup_id(tlog, id);
      TM_kill_vert(tlog->tm, v);

      break;
    }

    case LOG_EDGE_KILL: {
      int id = log[i++].value.i;
      int kill_verts = log[i++].value.i;

      TMEdge *e = elemhash_lookup_id(tlog, id);
      TM_kill_edge(tlog->tm, e, !!kill_verts);

      break;
    }

    case LOG_TRI_KILL: {
      int id = log[i++].value.i;
      int kill_verts = log[i++].value.i;
      int kill_edges = log[i++].value.i;

      TMFace *tri = elemhash_lookup_id(tlog, id);
      TM_kill_tri(tlog->tm, tri, !!kill_edges, !!kill_verts);

      break;
    }

    case LOG_VERT_ADD: {
      int id = log[i++].value.i;
      float *co = log[i++].value.vec3;
      float *no = log[i++].value.vec3;
      float mask = log[i++].value.f;
      int skipcd = log[i++].value.i;

      TMVert *v = TM_make_vert(tlog->tm, co, no, skipcd);

      if (tlog->cd_vert_mask_index >= 0) {
        TM_ELEM_CD_SET_FLOAT(v, tlog->cd_vert_mask_index, mask);
      }

      elemhash_add(tlog, v, id, entry_i);

      break;
    }
    case LOG_EDGE_ADD: {
      int id = log[i++].value.i;
      int v1id = log[i++].value.i;
      int v2id = log[i++].value.i;
      int skipcd = log[i++].value.i;

      TMVert *v1 = elemhash_lookup_id(tlog, v1id);
      TMVert *v2 = elemhash_lookup_id(tlog, v2id);
      TMEdge *e = TM_get_edge(tlog->tm, v1, v2, skipcd);

      elemhash_add(tlog, e, id, entry_i);

      break;
    }
    case LOG_TRI_ADD: {
      int id = log[i++].value.i;
      int skipcd = log[i++].value.i;
      int v1id = log[i++].value.i;
      int v2id = log[i++].value.i;
      int v3id = log[i++].value.i;

      TMVert *v1 = elemhash_lookup_id(tlog, v1id);
      TMVert *v2 = elemhash_lookup_id(tlog, v2id);
      TMVert *v3 = elemhash_lookup_id(tlog, v3id);

      TMFace *f = TM_make_tri(tlog->tm, v1, v2, v3, !!skipcd);
      elemhash_add(tlog, f, id, entry_i);
    }
  }

  return i;
}

static int mesh_wind_list(TriMeshLog *tlog, int entry_i, int threadnr)
{
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int tot = log[i++].value.i;

  for (int j = 0; j < tot; j++) {
    int tid = log[i++].value.i;
    int *ret = NULL;
    bool exists = BLI_hashmap_lookup_p(HashInt, HashInt)(tlog->elemhash_entry, tid, &ret);

    if (!exists) {
      continue;  // error!
    }

    int ti = (int)ret;
    meshlog_wind(tlog, ti, threadnr);
  }

  return i;
}

static int meshlog_unwind(TriMeshLog *tlog, int entry_i, int threadnr);

static int mesh_unwind_list(TriMeshLog *tlog, int entry_i, int threadnr)
{
  LogEntry *log = tlog->entries;
  int i = entry_i;

  int tot = log[i++].value.i;

  for (int j = 0; j < tot; j++) {
    int tid = log[i++].value.i;
    int *ret = NULL;
    bool exists = BLI_hashmap_lookup_p(HashInt, HashInt)(tlog->elemhash_entry, tid, &ret);

    if (!exists) {
      continue;  // error!
    }

    int ti = (int)ret;
    meshlog_unwind(tlog, ti, threadnr);
  }

  return i;
}

static int meshlog_unwind(TriMeshLog *tlog, int entry_i, int threadnr)
{
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

      TM_kill_vert(tlog->tm, v);
      break;
    }
    case LOG_VERT_STATE: {
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
      TM_ELEM_CD_SET_FLOAT(v, tlog->cd_vert_mask_index, mask);

      break;
    }

    case LOG_EDGE_ADD: {
      int id = log[i++].value.i;
      int v1id = log[i++].value.i;
      int v2id = log[i++].value.i;
      int skipcd = log[i++].value.i;

      TMEdge *e = elemhash_lookup_id(tlog, id);
      TM_kill_edge(tlog->tm, e, false);

      break;
    }

    case LOG_TRI_ADD: {
      int id = log[i++].value.i;
      TMFace *f = elemhash_lookup_id(tlog, id);

      TM_kill_tri(tlog->tm, f, false, false);

      break;
    }

    case LOG_VERT_KILL: {
      int id = log[i++].value.i;
      int totedge = log[i++].value.i;

      for (int j = 0; j < totedge; j++) {
        int eid = log[i++].value.i;
        int ei = elemhash_get_entry(tlog, eid);

        if (!ei < 0) {
          printf("error! %s:%d\n", __FILE__, __LINE__);
          continue;
        }

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

      i += 1;  // skip wind list length (which is 6)

      for (int j = 0; j < 3; j++) {
        int id = log[i++].value.i;

        vs[j] = BLI_hashmap_lookup(HashInt, HashP)(tlog->elemhash_ptr, id);
      }

      i -= 6;
      TMFace *tri = TM_make_tri(tlog->tm, vs[0], vs[1], vs[2], false);

      i = trimesh_read_loop(tlog, tri->l1, i);
      i = trimesh_read_loop(tlog, tri->l2, i);
      i = trimesh_read_loop(tlog, tri->l3, i);
      break;
    }
  }

  return i;
}

void BLI_log_unwind(TriMeshLog *tlog, int threadnr)
{
  int start;
  int end;

  if (tlog->curgroup == tlog->totgroup - 1) {
    start = tlog->totentries - 1;
  }
  else {
    start = tlog->groups[tlog->totgroup + 1];
  }

  end = tlog->groups[tlog->curgroup];
  LogEntry *log = tlog->entries;

  for (int i = start; i >= end; i = log[i].value.i) {
    meshlog_unwind(tlog, i, threadnr);
  }
}

float *TM_log_original_vert_co(TriMeshLog *tlog, TMVert *v)
{
  float *co = NULL, *no = NULL;

  TM_log_original_vert_data(tlog, v, &co, NULL);
  return co;
}

void TM_log_original_vert_data(TriMeshLog *tlog, TMVert *v, const float **r_co, const short **r_no)
{
  normal_float_to_short_v3(v->ono, v->no);
  copy_v3_v3(v->oco, v->co);

  *r_co = v->oco;
  *r_no = v->ono;

  return;

  int id = elemhash_get_vert_id(tlog, v, tlog->cd_vert_mask_index);

  int entry = elemhash_get_entry(tlog, id);

  entry += 2;

  LogEntry *log = tlog->entries;

  *r_co = log[entry++].value.vec3;
  float no[3];

  copy_v3_v3(no, log[entry++].value.vec3);
  normal_float_to_short_v3(log[entry++].value.hvec3, no);

  if (r_no) {
    *r_no = log[entry++].value.hvec3;
  }
}

/* Get the logged mask of a vertex
 *
 * Does not modify the log or the vertex */
float TM_log_original_mask(TriMeshLog *tlog, TMVert *v)
{
  int id = elemhash_get_vert_id(tlog, v, tlog->cd_vert_mask_index);
  int entry = elemhash_get_entry(tlog, id);

  LogEntry *log = tlog->entries;

  return log[entry + 5].value.f;
}
