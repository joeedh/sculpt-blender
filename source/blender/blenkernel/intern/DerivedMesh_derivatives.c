/*
* ***** BEGIN GPL LICENSE BLOCK *****
*
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
* The Original Code is Copyright (C) 2005 Blender Foundation.
* All rights reserved.
*
* The Original Code is: all of this file.
*
* Contributor(s): none yet.
*
* ***** END GPL LICENSE BLOCK *****
*/

/** \file blender/blenkernel/intern/DerivedMesh.c
*  \ingroup bke
*/


#include <string.h>
#include <limits.h>

#include "MEM_guardedalloc.h"

#include "DNA_cloth_types.h"
#include "DNA_key_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_array.h"
#include "BLI_blenlib.h"
#include "BLI_bitmap.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BLI_linklist.h"
#include "BLI_task.h"
#include "BLI_memarena.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_colorband.h"
#include "BKE_editmesh.h"
#include "BKE_key.h"
#include "BKE_layer.h"
#include "BKE_library.h"
#include "BKE_material.h"
#include "BKE_modifier.h"
#include "BKE_mesh.h"
#include "BKE_mesh_iterators.h"
#include "BKE_mesh_mapping.h"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_tangent.h"
#include "BKE_object.h"
#include "BKE_object_deform.h"
#include "BKE_paint.h"
#include "BKE_multires.h"
#include "BKE_bvhutils.h"
#include "BKE_deform.h"

#include "BLI_sys_types.h" /* for intptr_t support */

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"
#include "BKE_shrinkwrap.h"

#ifdef WITH_OPENSUBDIV
#  include "DNA_userdef_types.h"
#endif

#include "BKE_DerivedMesh_Derivatives.h"

typedef struct VertBucket {
	int len, used;
	int es[];
} VertBucket;

#define VERTIDX(v, mvert) (int)(v - mvert)
#define EDGEIDX(e, medge) (int)(e - medge)
#define VBUCKETSIZE	8

static void vbucket_insert(VertBucket **vert_emap, MemArena *arena, int e, int v) {
	if (!vert_emap[v]) {
		vert_emap[v] = BLI_memarena_calloc(arena, sizeof(int)*(VBUCKETSIZE + 2));
		vert_emap[v]->len = VBUCKETSIZE;
		vert_emap[v]->used = 0;
	} else if (vert_emap[v]->used >= vert_emap[v]->len) {
		VertBucket *vold = vert_emap[v];
		VertBucket *vnew = BLI_memarena_calloc(arena, sizeof(int)*((vold->len >> 1) + 2));

		memcpy(vnew, vold, sizeof(int)*(2 + (vold->len>>2)));
		vnew->len = vnew->len >> 1;
	}

	vert_emap[v]->es[vert_emap[v]->used++] = e;
}

static int find_opposing_edge(int v, int e, VertBucket **vert_emap, MVert *mverts, MEdge *medges) {
	float t1[3], t2[3];
	float minth = 1e17;
	int ilen, i, j, *es, mine=-1;

	//sanity check
	if (!vert_emap[v])
		return -1;

	ilen = vert_emap[v]->used;
	es = vert_emap[v]->es;

	sub_v3_v3v3(t1, mverts[medges[e].v2].co, mverts[medges[e].v1].co);
	normalize_v3(t1);

	for (i = 0; i < ilen; i++, es++) {
		float th;
		int e2 = *es;

		if (medges[e].v1 == v) {
			sub_v3_v3v3(t2, mverts[medges[e2].v2].co, mverts[medges[e2].v1].co);
		} else {
			sub_v3_v3v3(t2, mverts[medges[e2].v2].co, mverts[medges[e2].v1].co);
		}

		normalize_v3(t2);
		th = dot_v3v3(t1, t2);

		//find edge with largest opposing angle
		if (th < minth) {
			mine = e2;
			minth = th;
		}
	}

	return mine == -1 ? e : mine;
}

static int calc_vert_edge_dv(MVert *mverts, MEdge *medges, VertBucket *vert_emap, int e, int v, DMDvVert *out) {

}

DMCageDv *DM_calc_cage_derivatives(DerivedMesh *cageDM, DerivedMesh *finalDM) {
	MVert *cagev, *finalv, *mvert;
	MEdge *cagee, *finale, *medge;
	int *origvs, *origes, *origv, *orige;
	MemArena *arena;
	int totedge_cage = cageDM->getNumEdges(cageDM), totedge_final = finalDM->getNumEdges(finalDM);
	int totvert_cage = cageDM->getNumVerts(cageDM), totvert_final = finalDM->getNumVerts(finalDM);
	VertBucket **vert_emap;
	DMCageDv *cagedv;
	int i, j;
	
	arena = BLI_memarena_new(8192, "DM_calc_cage_derivatives");

	cagev = cageDM->getVertArray(cageDM);
	cagee = cageDM->getEdgeArray(cageDM);
	finalv = finalDM->getVertArray(finalDM);
	finale = finalDM->getEdgeArray(finalDM);
	origvs = finalDM->getVertDataArray(finalDM, CD_ORIGINDEX);
	origes = finalDM->getEdgeDataArray(finalDM, CD_ORIGINDEX);

	if (!cagev || !cagee || !finalv || !finale || !origv || !orige) {
		BLI_memarena_free(arena);
		return NULL; //failed
	}

	cagedv = MEM_callocN(sizeof(*cagedv), "DMCageDv");
	cagedv->edges = MEM_callocN(sizeof(DMDvEdge)*totedge_cage, "DM_calc_cage_derivatvies cagedv->edges");
	cagedv->totedge = totedge_cage;

	vert_emap = MEM_callocN(sizeof(VertBucket*)*totvert_final, "DM_calc_cage_derivatives vert_emap");


	//try to build minimal adjacency info to save on both processing and memory
	for (i = 0, medge=finale; i < totedge_final; i++, medge++) {
		vbucket_insert(vert_emap, arena, i, medge->v1);
		vbucket_insert(vert_emap, arena, i, medge->v2);
	}


	orige = origes;
	for (i = 0, mvert = finalv; i < totvert_final; i++, mvert++, orige++) {
		VertBucket *vb = vert_emap[i];
		if (*orige == ORIGINDEX_NONE || *orige < 0 || *orige >= totedge_cage)
			continue;

		if (!vb)
			continue;

		for (j = 0; j < vb->used; j++) {
			DMDvEdge *dve = cagedv->edges + *orige;
			int e2 = vb->es[j];

			calc_vert_edge_dv(finalv, finale, vert_emap, e2, i, &dve->vs[0]);
			calc_vert_edge_dv(finalv, finale, vert_emap, e2, i, &dve->vs[1]);
			//calc_vert_edge_dv(MVert *mverts, MEdge *medges, VertBucket *vert_emap, int e, int v, DMDvVert *out) {
		}
	}

	BLI_memarena_free(arena);
	MEM_freeN(vert_emap);

	return cagedv;
}

void DM_free_derivatives(DMCageDv *cagedv) {
	if (cagedv->edges)
		MEM_freeN(cagedv->edges);

	MEM_freeN(cagedv);
}
