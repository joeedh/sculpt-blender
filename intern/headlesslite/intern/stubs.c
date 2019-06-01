#include "DNA_object_types.h"
#include "DNA_meshdata_types.h"

#include "stdio.h"

const char *BKE_main_blendfile_path_from_global() {
    return "bad_call_to_BKE_main_blendfile_path_from_global_stub";
}

#define REPORT(msg) fprintf(stderr, "%s\n", msg);

#define STUB(func) void func() {fprintf(stderr, "call to stub %s\n", #func);}
#define SILENTSTUB(func) void func() {}

//hackish stub for G global
char G[1024] = { 0, };

STUB(ccgSubSurf_updateFromFaces)
STUB(ccgSubSurf_updateToFaces)
STUB(ccgSubSurf_updateNormals)
STUB(ccgSubSurf_updateLevels)
STUB(ccgSubSurf_stitchFaces)
STUB(ccgSubSurf_getNumVerts)
STUB(ccgSubSurf_getNumFaces)
STUB(ccgSubSurf_getNumEdges)
STUB(ccgSubSurf_getEdgeSize)
STUB(ccgSubSurf_getGridSize)
STUB(ccgSubSurf_getVertData)
STUB(ccgSubSurf_getEdgeData)
STUB(ccgSubSurf_getFaceNumVerts)
STUB(ccgSubSurf_getFaceCenterData)
STUB(ccgSubSurf_getFaceGridEdgeData)
STUB(ccgSubSurf_getFaceGridData)
STUB(modifierNew)
STUB(plConvexHullCompute)
STUB(BKE_libblock_alloc);
STUB(BKE_libblock_init_empty)
STUB(BKE_id_new_nomain)
STUB(BKE_id_copy)
STUB(BKE_id_copy_ex)
STUB(test_object_materials)
STUB(test_object_modifiers)
STUB(BKE_object_obdata_texspace_get)
STUB(DEG_debug_print_eval)
STUB(bvhcache_free)
STUB(BKE_subdiv_ccg_destroy)
STUB(BKE_shrinkwrap_discard_boundary_data)
STUB(BKE_pbvh_free)
STUB(BKE_pbvh_type)
STUB(BKE_pbvh_get_grid_updates)
STUB(BKE_subdiv_ccg_average_stitch_faces)
STUB(get_render_subsurf_level)
STUB(subsurf_make_derived_from_derived)
STUB(BKE_ccg_gridsize)
STUB(BKE_ccg_factor)
STUB(BKE_object_scale_to_mat3)
STUB(DEG_get_evaluated_object)
STUB(plConvexHullDelete)
STUB(plConvexHullNumVertices)
STUB(plConvexHullNumFaces)
STUB(plConvexHullGetVertex)
STUB(plConvexHullGetFaceSize)
STUB(plConvexHullGetFaceVertices)
STUB(BKE_curve_forward_diff_bezier) //XXX might need this, used in bmo_subdivide.c
STUB(DM_set_only_copy)
STUB(mesh_get_eval_deform)
STUB(modifier_new)
STUB(modifier_isEnabled)
STUB(modifiers_findByType)
STUB(modifierType_getInfo)
STUB(multiresModifier_reshapeFromCCG)
STUB(paint_grid_paint_mask)
STUB(BKE_keyblock_add)
STUB(BKE_keyblock_is_basis)
STUB(conv_utf_16_to_8)
STUB(alloc_utf16_from_8)
STUB(gzread)
STUB(gzclose)
STUB(gzopen_w)
STUB(ufopen)
STUB(uopen)
STUB(uaccess)
STUB(urename)
STUB(umkdir)
STUB(numaAPI_Initialize)
STUB(mk_wcswidth)
STUB(mk_wcwidth)
STUB(count_utf_8_from_16)
STUB(uputenv)
STUB(CDDM_from_mesh)
STUB(CDDM_copy)

void BKE_boundbox_init_from_minmax(BoundBox *bb, const float min[3], const float max[3])
{
  bb->vec[0][0] = bb->vec[1][0] = bb->vec[2][0] = bb->vec[3][0] = min[0];
  bb->vec[4][0] = bb->vec[5][0] = bb->vec[6][0] = bb->vec[7][0] = max[0];

  bb->vec[0][1] = bb->vec[1][1] = bb->vec[4][1] = bb->vec[5][1] = min[1];
  bb->vec[2][1] = bb->vec[3][1] = bb->vec[6][1] = bb->vec[7][1] = max[1];

  bb->vec[0][2] = bb->vec[3][2] = bb->vec[4][2] = bb->vec[7][2] = min[2];
  bb->vec[1][2] = bb->vec[2][2] = bb->vec[5][2] = bb->vec[6][2] = max[2];
}



SILENTSTUB(bpy_bm_generic_invalidate)
SILENTSTUB(BKE_animdata_free)
SILENTSTUB(id_us_plus)
SILENTSTUB(id_us_min)
SILENTSTUB(BKE_id_make_local_generic)

MDeformWeight *defvert_find_index(const MDeformVert *dvert, const int defgroup)
{
  if (dvert && defgroup >= 0) {
    MDeformWeight *dw = dvert->dw;
    unsigned int i;

    for (i = dvert->totweight; i != 0; i--, dw++) {
      if (dw->def_nr == defgroup) {
        return dw;
      }
    }
  } else {
    //BLI_assert(0);
  }

  return NULL;
}

float defvert_find_weight(const struct MDeformVert *dvert, const int defgroup)
{
  MDeformWeight *dw = defvert_find_index(dvert, defgroup);
  return dw ? dw->weight : 0.0f;
}

