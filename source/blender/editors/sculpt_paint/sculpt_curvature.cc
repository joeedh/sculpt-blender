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
 * The Original Code is Copyright (C) 2021 by Joseph Eagar
 * All rights reserved.
 * Implements curvature analysis for sculpt tools
 */

/** \file
 * \ingroup edsculpt
 */

#include "MEM_guardedalloc.h"

#include "BLI_math_matrix.h"
#include "BLI_math_solvers.h"

#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"

#include "sculpt_intern.hh"

#include <cmath>

using namespace blender::bke::paint;

/*
If you're working with uniform triangle tesselations, the math for
calculating principle curvatures reduces to doing an eigen decomposition
of the smoothed normal covariance matrix.

The normal covariance matrix is just:

nx*nx nx*ny nx*nz
ny*nx ny*ny ny*nz
nz*nx nz*ny nz*nz

To find principle curvatures, simply subtract neighboring covariance matrices.
You can do this over any number of neighborhood rings to get more accurate result

*/

BLI_INLINE void normal_covariance(float mat[3][3], float no[3])
{
  mat[0][0] = no[0] * no[0];
  mat[0][1] = no[0] * no[1];
  mat[0][2] = no[0] * no[2];
  mat[1][0] = no[1] * no[0];
  mat[1][1] = no[1] * no[1];
  mat[1][2] = no[1] * no[2];
  mat[2][0] = no[2] * no[0];
  mat[2][1] = no[2] * no[1];
  mat[2][2] = no[2] * no[2];
}

bool SCULPT_calc_principle_curvatures(SculptSession *ss,
                                      PBVHVertRef vertex,
                                      SculptCurvatureData *out,
                                      bool useAccurateSolver)
{
  SculptVertexNeighborIter ni;
  float nmat[3][3], nmat2[3][3];
  float no[3], no2[3];

  memset(out, 0, sizeof(SculptCurvatureData));

  SCULPT_vertex_normal_get(ss, vertex, no);
  normal_covariance(nmat, no);

  /* TODO: review the math here. We're deriving the curvature
   * via an eigen decomposition of the weighted summed
   * normal covariance matrices of the surrounding topology.
   */
  SCULPT_VERTEX_NEIGHBORS_ITER_BEGIN (ss, vertex, ni) {
    SCULPT_vertex_normal_get(ss, ni.vertex, no2);
    sub_v3_v3(no2, no);

    SculptVertexNeighborIter ni2;
    SCULPT_VERTEX_NEIGHBORS_ITER_BEGIN (ss, ni.vertex, ni2) {
      float no3[3];
      SCULPT_vertex_normal_get(ss, ni2.vertex, no3);

      normal_covariance(nmat2, no3);
      madd_m3_m3m3fl(nmat, nmat, nmat2, 1.0f / ni2.size);
    }
    SCULPT_VERTEX_NEIGHBORS_ITER_END(ni2);

    normal_covariance(nmat2, no2);
    madd_m3_m3m3fl(nmat, nmat, nmat2, 1.0f / ni.size);
  }
  SCULPT_VERTEX_NEIGHBORS_ITER_END(ni);

  if (!useAccurateSolver || !BLI_eigen_solve_selfadjoint_m3(nmat, out->ks, out->principle)) {
    /* Do simple power solve in one direction. */

    float t[3];
    float t2[3];

    SCULPT_vertex_normal_get(ss, vertex, no);
    copy_v3_v3(t, no);

    for (int i = 0; i < 25; i++) {
      if (i > 0) {
        normalize_v3(t);

        if (i > 5 && len_squared_v3v3(t, t2) < 0.000001f) {
          break;
        }

        copy_v3_v3(t2, t);
      }

      mul_m3_v3(nmat, t);
    }

    out->ks[1] = normalize_v3(t);
    copy_v3_v3(out->principle[1], t);

    cross_v3_v3v3(out->principle[0], out->principle[1], no);
    if (dot_v3v3(out->principle[0], out->principle[0]) > FLT_EPSILON * 50.0f) {
      normalize_v3(out->principle[0]);
    }
    else {
      zero_v3(out->principle[0]);
    }
  }

  if (is_zero_v3(out->principle[0])) {
    /* Choose an orthoganoal direction. */
    copy_v3_v3(out->principle[0], no);
    float axis[3] = {0.0f, 0.0f, 0.0f};

    if (fabsf(no[0]) > fabs(no[1]) && fabs(no[0]) >= fabs(no[2])) {
      axis[1] = 1.0f;
    }
    else if (fabsf(no[1]) > fabs(no[0]) && fabs(no[1]) >= fabs(no[2])) {
      axis[2] = 1.0f;
    }
    else {
      axis[0] = 1.0f;
    }

    cross_v3_v3v3(out->principle[0], no, axis);
    cross_v3_v3v3(out->principle[1], out->principle[0], no);
    copy_v3_v3(out->principle[2], no);

    normalize_v3(out->principle[0]);
    normalize_v3(out->principle[1]);
  }

  return true;
}

void SCULPT_curvature_dir_get(SculptSession *ss,
                              PBVHVertRef v,
                              float dir[3],
                              bool useAccurateSolver)
{
  if (BKE_pbvh_type(ss->pbvh) != PBVH_BMESH) {
    SculptCurvatureData curv;
    SCULPT_calc_principle_curvatures(ss, v, &curv, useAccurateSolver);

    copy_v3_v3(dir, curv.principle[0]);
    return;
  }

  copy_v3_v3(dir, vertex_attr_ptr<float>(v, ss->attrs.curvature_dir));
}

void SCULPT_curvature_begin(SculptSession *ss, struct PBVHNode *node, bool useAccurateSolver)
{
  if (BKE_pbvh_type(ss->pbvh) != PBVH_BMESH) {
    /* Caching only happens for bmesh for now. */
    return;
  }

  if (BKE_pbvh_curvature_update_get(node)) {
    PBVHVertexIter vi;

    BKE_pbvh_curvature_update_set(node, false);

    BKE_pbvh_vertex_iter_begin (ss->pbvh, node, vi, PBVH_ITER_UNIQUE) {
      SculptCurvatureData curv;
      SCULPT_calc_principle_curvatures(ss, vi.vertex, &curv, useAccurateSolver);

      copy_v3_v3(vertex_attr_ptr<float>(vi.vertex, ss->attrs.curvature_dir), curv.principle[0]);
    }
    BKE_pbvh_vertex_iter_end;
  }
}
