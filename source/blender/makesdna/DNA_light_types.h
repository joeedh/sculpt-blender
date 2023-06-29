/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_MTEX
#  define MAX_MTEX 18
#endif

struct AnimData;
struct CurveMapping;
struct Ipo;
struct bNodeTree;

typedef struct Light {
  DNA_DEFINE_CXX_METHODS(Light)

  ID id;
  /** Animation data (must be immediately after id for utilities to use it). */
  struct AnimData *adt;

  short type, flag;
  int mode;

  float r, g, b, k;
  float shdwr, shdwg, shdwb, shdwpad;

  float energy, dist, spotsize, spotblend;

  /** Quad1 and Quad2 attenuation. */
  float att1, att2;
  float coeff_const, coeff_lin, coeff_quad;
  char _pad0[4];
  struct CurveMapping *curfalloff;
  short falloff_type;
  char _pad2[2];

  float clipsta, clipend;
  float bias;
  float radius;
  short bufsize, samp, buffers, filtertype;
  char bufflag, buftype;

  short area_shape;
  float area_size, area_sizey, area_sizez;
  float area_spread;

  float sun_angle;

  /* texact is for buttons */
  short texact, shadhalostep;

  /** Old animation system, deprecated for 2.5. */
  struct Ipo *ipo DNA_DEPRECATED;
  short pr_texture, use_nodes;

  /* Eevee */
  float cascade_max_dist;
  float cascade_exponent;
  float cascade_fade;
  int cascade_count;

  float contact_dist;
  float contact_bias;
  float contact_thickness;

  float diff_fac, volume_fac;
  float spec_fac, att_dist;

  /* preview */
  struct PreviewImage *preview;

  /* nodes */
  struct bNodeTree *nodetree;
} Light;

/* **************** LIGHT ********************* */

/** #Light::flag */
enum {
  LA_DS_EXPAND = 1 << 0,
  /**
   * NOTE: this must have the same value as #MA_DS_SHOW_TEXS,
   * otherwise anim-editors will not read correctly.
   */
  LA_DS_SHOW_TEXS = 1 << 2,
};

/** #Light::type */
enum {
  LA_LOCAL = 0,
  LA_SUN = 1,
  LA_SPOT = 2,
  // LA_HEMI = 3, /* Deprecated. */
  LA_AREA = 4,
};

/** #Light::mode */
enum {
  LA_SHADOW = 1 << 0,
  // LA_HALO = 1 << 1, /* Deprecated. .*/
  // LA_LAYER = 1 << 2, /* Deprecated. */
  // LA_QUAD = 1 << 3, /* Deprecated. */
  // LA_NEG = 1 << 4, /* Deprecated. */
  // LA_ONLYSHADOW = 1 << 5, /* Deprecated. */
  // LA_SPHERE = 1 << 6, /* Deprecated. */
  LA_SQUARE = 1 << 7,
  // LA_TEXTURE = 1 << 8, /* Deprecated. */
  // LA_OSATEX = 1 << 9, /* Deprecated. */
  // LA_DEEP_SHADOW = 1 << 10, /* Deprecated. */
  // LA_NO_DIFF = 1 << 11, /* Deprecated. */
  // LA_NO_SPEC = 1 << 12, /* Deprecated. */
  LA_SHAD_RAY = 1 << 13, /* Deprecated, cleaned. */
  /**
   * YAFRAY: light shadow-buffer flag, soft-light.
   * Since it is used with LOCAL light, can't use LA_SHAD.
   * */
  // LA_YF_SOFT = 1 << 14, /* Deprecated. */
  // LA_LAYER_SHADOW = 1 << 15, /* Deprecated. */
  // LA_SHAD_TEX = 1 << 16, /* Deprecated. */
  LA_SHOW_CONE = 1 << 17,
  // LA_SHOW_SHADOW_BOX = 1 << 18,
  LA_SHAD_CONTACT = 1 << 19,
  LA_CUSTOM_ATTENUATION = 1 << 20,
};

/** #Light::falloff_type */
enum {
  LA_FALLOFF_CONSTANT = 0,
  LA_FALLOFF_INVLINEAR = 1,
  LA_FALLOFF_INVSQUARE = 2,
  LA_FALLOFF_CURVE = 3,
  LA_FALLOFF_SLIDERS = 4,
  LA_FALLOFF_INVCOEFFICIENTS = 5,
};

/** #Light::area_shape */
enum {
  LA_AREA_SQUARE = 0,
  LA_AREA_RECT = 1,
  // LA_AREA_CUBE = 2, /* Deprecated. */
  // LA_AREA_BOX = 3,  /* Deprecated. */
  LA_AREA_DISK = 4,
  LA_AREA_ELLIPSE = 5,
};

#ifdef __cplusplus
}
#endif
