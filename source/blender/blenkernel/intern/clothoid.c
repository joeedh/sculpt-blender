// libfair.cpp : Defines the exported functions for the DLL application.
//

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"
#include "BLI_blenlib.h"
#include "BLI_memarena.h"

#include "BKE_clothoid.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define MAX_DEGREE MAX_CLOTHOID_DEGREE

#define KSTEPS 24

#define KKS 0 //curvature coeffs
#define KTS MAX_DEGREE //torsion coeffs
#define KSCALE (MAX_DEGREE*2)
 
//(automatically calculuated) start offset
#define KSTARTX  (KSCALE + 1)
#define KSTARTY  (KSCALE + 2)
#define KSTARTZ  (KSCALE + 3)

//(automatically calculated) final rotation
#define KQUATX  (KSCALE + 4)
#define KQUATY  (KSCALE + 5)
#define KQUATZ  (KSCALE + 6)
#define KQUATW  (KSCALE + 7)

//starting condition tangent
#define KTANX  (KSCALE + 8)
#define KTANY  (KSCALE + 9)
#define KTANZ  (KSCALE + 10)

//starting condition binormal
#define KBINX  (KSCALE + 11)
#define KBINY  (KSCALE + 12)
#define KBINZ  (KSCALE + 13)

#define KFLAG (KSCALE + 14)
#define KTOT (KSCALE + 15)

#define FLAG_UPDATE 1

int rotate(float vec[3], float axis[3], float th) {
	//th = -th;
	float ax, ay, az, vx, vy, vz, x, y, z, sinth, costh, ax2, ay2, az2, sinth2, costh2;

	ax = axis[0];
	ay = axis[1];
	az = axis[2];

	vx = vec[0];
	vy = vec[1];
	vz = vec[2];

	ax2 = ax * ax;
	ay2 = ay * ay;
	az2 = az * az;

	sinth = sin(th*0.5);
	costh = cos(th*0.5);

	sinth2 = sinth * sinth;
	costh2 = costh * costh;

	x = -((az*sinth*vx + 2 * costh*vy)*az*sinth + (sinth + 1)*(sinth - 1)*
		vx + (ay*sinth*vx - 2 * costh*vz)*ay*sinth) + (2 * (ay*vy + az * vz) + ax * vx)*ax*sinth2;
	y = -((az*sinth*vy - 2 * costh*vx)*az*sinth + (sinth + 1)*(sinth - 1)
		*vy - (ay*vy + 2 * az*vz)*ay*sinth2) + (2 * (ay*sinth*vx - costh * vz) - ax * sinth*vy)*ax*sinth;
	z = (2 * (az*sinth*vy - costh * vx) - ay * sinth*vz)*ay*sinth + (az2 *
		sinth2 - sinth2 + 1)*vz + (2 * (az*sinth*vx + costh * vy) - ax * sinth*vz)*ax*sinth;

	vec[0] = x;
	vec[1] = y;
	vec[2] = z;

	return 1;
}

/*space curve integration based on a Frenet–Serret frame*/

/*curvature and twist polynomials (Bezier/Bernstein)*/

//curvakure
static float kpoly(float *ks, float s, int degree) {
	float k1 = ks[0];

	switch (degree) {
	case 1:
		return k1;
	case 2: {
		float k2 = ks[1];
		return (k1 + (k2 - k1)*(s + 0.5));
	} case 3: {
		float k2 = ks[1], k3 = ks[2];
		return ((4.0*k1*s*s - 4.0*k1*s + k1 - 8.0*k2*s*s + 2.0*k2 + 4.0*k3*s*s + 4.0*k3*s + k3) / 4.0);
	} case 4: {
		float k2 = ks[1], k3 = ks[2], k4 = ks[3];
		return ((-(((6.0*k3*s - 3.0*k3 - 2.0*k4*s - k4)*(2.0*s + 1.0) - 3.0*(2.0*s - 1.0)*(2.0*s - 1.0) * k2)*(2.0*s + 1.0) + (2.0*s - 1.0)*(2.0*s - 1.0) *(2.0*s - 1.0) * k1)) / 8.0);
	} default:
		return 0.0;
	}
}

//twist
static float tpoly(float *ks, float s, int degree) {
	float t1 = ks[KTS];

	switch (degree) {
	case 1:
		return t1;
	case 2: {
		float t2 = ks[KTS + 1];
		return (t1 + (t2 - t1)*(s + 0.5));
	} case 3: {
		float t2 = ks[KTS + 1], t3 = ks[KTS + 2];
		return ((4.0*t1*s*s - 4.0*t1*s + t1 - 8.0*t2*s*s + 2.0*t2 + 4.0*t3*s*s + 4.0*t3*s + t3) / 4.0);
	} case 4: {
		float t2 = ks[KTS + 1], t3 = ks[KTS + 2], t4 = ks[KTS + 3];
		return ((-(((6.0*t3*s - 3.0*t3 - 2.0*t4*s - t4)*(2.0*s + 1.0) - 3.0*(2.0*s - 1.0)*(2.0*s - 1.0) * t2)*(2.0*s + 1.0) + (2.0*s - 1.0)*(2.0*s - 1.0) *(2.0*s - 1.0) * t1)) / 8.0);
	} default:
		return 0.0;
	}
}

static float tpoly_df(float *ks, float s, int degree) {
	float t1 = ks[KTS];

	switch (degree) {
	case 1:
		return 0.0;
	case 2: {
		float t2 = ks[KTS + 1];
		return (t2 - t1);
	} case 3: {
		float t2 = ks[KTS + 1], t3 = ks[KTS + 2];
		return (2 * s*t1 - 4 * s*t2 + 2 * s*t3 - t1 + t3);
	} case 4: {
		float t2 = ks[KTS + 1], t3 = ks[KTS + 2], t4 = ks[KTS + 3];
		return ((-3.0*(4.0*s*s*t1 - 12.0*s*s*t2 + 12.0*s*s*t3 - 4.0*s*s*t4 - 4.0*s*t1 + 4.0*s*t2 + 4.0*s*t3 - 4.0*s*t4 + t1 + t2 - t3 - t4)) / 4.0);
	} default:
		return 0.0;
	}
}

int quadrature(float *ks, float s1, float p[3], float *nor_out, float *tan_out, int kdegree, int tdegree) {
	float t[3], b[3], n[3], lastt[3];
	float s, ds, t1, t2, k1, k2, k3, k4, tr;

	p[0] = p[1] = p[2] = 0.0;

	t[0] = ks[KTANX];
	t[1] = ks[KTANY];
	t[2] = ks[KTANZ];

	//binormal
	n[0] = ks[KBINX];
	n[1] = ks[KBINY];
	n[2] = ks[KBINZ];

	t1 = ks[KTS];
	t2 = ks[KTS + 1];

	tr = tpoly(ks, 0.0, tdegree); //TPOLY(0.0);

								 //apply initial torsial rotation
	rotate(n, t, tr);

	//normal
	cross_v3_v3v3(b, t, n);

	k1 = ks[0];
	k2 = ks[1];
	k3 = ks[2];
	k4 = ks[3];

	//little trick; integrate from the middle of the curve, not an endpoint
	//this is accounted for in the polynomial math
	s1 = s1 - 0.5;

	ds = s1 / (float)KSTEPS;
	s = 0.0;

	lastt[0] = t[0];
	lastt[1] = t[1];
	lastt[2] = t[2];

	for (int i = 0; i < KSTEPS; i++, s += ds) {
		float kk = kpoly(ks, s, kdegree);
		float kt = tpoly_df(ks, s, tdegree); //TPOLY_DF(s);

#if 0
		float dx2 = (t[0] - lastt[0]) / ds;
		float dy2 = (t[1] - lastt[1]) / ds;
		float dz2 = (t[2] - lastt[2]) / ds;
#endif
		//dx2 = dy2 = dz2 = 0.0;
#if 1
		float dx2 = n[0] * kk;
		float dy2 = n[1] * kk;
		float dz2 = n[2] * kk;
#endif

		float dx3 = kk * (-kk * t[0] + tr * b[0]);
		float dy3 = kk * (-kk * t[1] + tr * b[1]);
		float dz3 = kk * (-kk * t[2] + tr * b[2]);

		//dx3 = dy3 = dz3 = 0.0;
		//dx3 = dy3 = dz3 = dx2 = dy2 = dz2 = 0.0;

		p[0] += t[0] * ds + 0.5*dx2*ds*ds + (1.0 / 6.0)*dx3*ds*ds*ds;
		p[1] += t[1] * ds + 0.5*dy2*ds*ds + (1.0 / 6.0)*dy3*ds*ds*ds;
		p[2] += t[2] * ds + 0.5*dz2*ds*ds + (1.0 / 6.0)*dz3*ds*ds*ds;

		tr += kt * ds;

		lastt[0] = t[0];
		lastt[1] = t[1];
		lastt[2] = t[2];

		rotate(t, b, kk*ds);
		rotate(n, t, kt*ds);

		cross_v3_v3v3(b, t, n);
		cross_v3_v3v3(n, b, t);

		normalize_v3(b);
		normalize_v3(n);
	}

	float sgn = -1.0; //ds < 0.0 ? -1.0 : 1.0;

	if (nor_out) {
		nor_out[0] = n[0] * sgn;
		nor_out[1] = n[1] * sgn;
		nor_out[2] = n[2] * sgn;
	}

	if (tan_out) {
		tan_out[0] = t[0] * sgn;
		tan_out[1] = t[1] * sgn;
		tan_out[2] = t[2] * sgn;
	}

	return 1;
}

//tan and nor are optional, can be NULL
void BKE_clothoid_evaluate(ClothoidData *data, float r[3], float t, float *tan_out, float *nor_out) {
	if (data->flag & CLOTHOID_UPDATE) {
		float start[3], end[3], q[4], t1[3], t2[3], line[3];
		float len, th;

		//int quadrature(float *ks, float s1, float p[3], float *nor_out, float *tan_out, int kdegree, int tdegree) {
		quadrature(data->ks, 0.0, start, NULL, NULL, data->kdegree, data->tdegree);
		quadrature(data->ks, 1.0, end, NULL, NULL, data->kdegree, data->tdegree);

		sub_v3_v3(end, start);
		len = len_v3(end);

		data->ks[KSCALE] = data->straightlen / len;

		sub_v3_v3v3(line, data->v2, data->v1);

		normalize_v3(end);

		cross_v3_v3v3(t1, end, line);

		normalize_v3(t1);

		th = acos(dot_v3v3(end, line));
		axis_angle_normalized_to_quat(q, t1, -th);

		data->ks[KQUATX] = q[0];
		data->ks[KQUATY] = q[1];
		data->ks[KQUATZ] = q[2];
		data->ks[KQUATW] = q[3];

		data->ks[KSTARTX] = start[0];
		data->ks[KSTARTY] = start[1];
		data->ks[KSTARTZ] = start[2];

		data->ks[KTANX] = line[0];
		data->ks[KTANY] = line[1];
		data->ks[KTANZ] = line[2];

		//make starting binormal
		t1[0] = 0.0f, t1[1] = 0.0f, t1[2] = 1.0f;
		cross_v3_v3v3(t2, line, t1);
	}

	quadrature(data->ks, t, r, tan_out, nor_out, data->kdegree, data->tdegree);

	sub_v3_v3(r, data->ks + KSTARTX);
	mul_qt_v3(data->ks + KQUATX, r);
	mul_v3_fl(r, data->ks[KSCALE]);
	add_v3_v3(r, data->v1);
}

void update_clothoid_v3(ClothoidData *data) {
	data->flag |= CLOTHOID_UPDATE;
}
