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
* The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
* All rights reserved.
*
* The Original Code is: all of this file.
*x`
* Contributor(s): Joseph Eagar
*
* ***** END GPL LICENSE BLOCK *****
*/

#ifndef _BKE_CLOTHOID_H
#define _BKE_CLOTHOID_H

#define MAX_CLOTHOID_DEGREE 4

/** \file BKE_clothoid.h
*  \ingroup bke
*  \since March 2001
*  \author nzc
*/

typedef struct ClothoidData {
	float v1[3], v2[3], straightlen;
	float tan1[3], tan2[3]; //optional
	int kdegree, tdegree; //degree of curvature and torsion polynomials
	int flag;
	float ks[64];
	void *owner;
	float start_roll;
} ClothoidData;


struct Curve;
typedef struct ClothoidSpline {
	struct Curve *curve;
};

/* forward differencing method for bezier curve */
void BKE_curve_forward_diff_bezier(float q0, float q1, float q2, float q3, float *p, int it, int stride)
{
	float rt0, rt1, rt2, rt3, f;
	int a;

	f = (float)it;
	rt0 = q0;
	rt1 = 3.0f * (q1 - q0) / f;
	f *= f;
	rt2 = 3.0f * (q0 - 2.0f * q1 + q2) / f;
	f *= it;
	rt3 = (q3 - q0 + 3.0f * (q1 - q2)) / f;

	q0 = rt0;
	q1 = rt1 + rt2 + rt3;
	q2 = 2 * rt2 + 6 * rt3;
	q3 = 6 * rt3;

	for (a = 0; a <= it; a++) {
		*p = q0;
		p = POINTER_OFFSET(p, stride);
		q0 += q1;
		q1 += q2;
		q2 += q3;
	}
}

/* forward differencing method for first derivative of cubic bezier curve */
void BKE_curve_forward_diff_tangent_bezier(float q0, float q1, float q2, float q3, float *p, int it, int stride);

//tan_out, nor_out may be NULL
void BKE_clothoid_evaluate(ClothoidData *cd, float r[3], float t, float *tan_out, float *nor_out);
void BKE_clothoid_tangent(ClothoidData *cd, float s, float *p);
void BKE_clothoid_normal(ClothoidData *cd, float s, float *p);
void BKE_clothoid_binormal(ClothoidData *cd, float s, float *p);
struct Nurbs;

void BKE_clothspline_update(struct Nurbs *nu);
void BKE_clothspline_minmax(struct Nurbs *nu, bool use_radius, float *min, float *max);

float BKE_clothspline_get_length(struct Nurbs *nu);
void BKE_clothoid_update(struct Nurbs *nu);

#define CLOTHOID_UPDATE 2
#endif /* _BKE_CLOTHOID_H */
