/*
* ***** BEGIN GPL LICENSE BLOCK *****
*
* This program is free software; you can redistribute it and/or
*modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
*but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software  Foundation,
*Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110 - 1301, USA.
*
* The Original Code is Copyright(C) 2006 Blender Foundation.
* All rights reserved.
*
* The Original Code is : all of this file.
*
* Contributor(s) : Joseph Eagar (joeedh@gmail.com)
*
* ***** END GPL LICENSE BLOCK *****
* /

/** \file BKE_DerivedMesh_Derivatives.h
*  \ingroup bke
*  \section Utilities to calculate first and second derives of DerivedMesh cages
*/

#ifndef _BKE_DERIVEDMESH_DERIVATIVES

/*
for each edge we calculate a pair of first and second derivatives,
one set for each vertex.
*/

struct DerivedMesh;

typedef struct DMDvVert {
	int vold; //original vertex index
	int vnew; //new index inside of derived mesh
	float co[3]; //new position
	float dv1[3], dv2[3]; //firsts and second derivatives
	int flag;
} DMDvVert;

typedef struct DMDvEdge {
	int eold; //original edge index
	int enew; //new index in side of derived mesh
	DMDvVert vs[2];
	int flag;
} DMDvEdge;

typedef struct DMCageDv {
	DMDvEdge *edges;
	int totedge;
} DMCageDv;

DMCageDv *DM_calc_cage_derivatives(struct DerivedMesh *cageDM, struct DerivedMesh *finalDM);
void DM_free_derivatives(DMCageDv *cagedvs);

#define _BKE_DERIVEDMESH_DERIVATIVES
#endif /* _BKE_DERIVEDMESH_DERIVATIVES */


