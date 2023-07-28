/*
 */
#pragma once

/*
 * Creating a C++ API that's both decent to use and compiles
 * to something as fast as the old BKE_pbvh_vertex_iter_begin() macro
 * is surprisingly complicated.
 */

#include "BLI_array.hh"
#include "BLI_compiler_attrs.h"
#include "BLI_compiler_compat.h"
#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_threads.h"
#include "BLI_timeit.hh"
#include "BLI_vector.hh"

#include "BKE_pbvh.h"
#include "BKE_pbvh_api.hh"
#include "BKE_dyntopo_set.hh"
#include "intern/pbvh_intern.hh"

#include <functional>
#include <type_traits>
#include <utility>

/* NotForPR: Needed for debugging but already been rejected in
 * a standalone PR.
 * Disable optimization for a function (for debugging use only!)
 */
#ifdef __clang__
#  define ATTR_NO_OPT __attribute__((optnone))
#elif defined(_MSC_VER)
#  define ATTR_NO_OPT __pragma(optimize("", off))
#elif defined(__GNUC__)
#  define ATTR_NO_OPT __attribute__((optimize("O0")))
#else
#  define ATTR_NO_OPT
#endif

namespace blender::bke::pbvh {

struct FilterResultAbstract {
  bool in_range;

  FilterResultAbstract(bool in_range_) : in_range(in_range_) {}
};

/** PBVHNode vertex iterator. */
template<typename NodeData,
         PBVHType pbvh_type,
         typename FilterResult = void, /* struct { bool in_range; } */
         typename FilterFunc = void *> /* (PBVHVertRef vertex, const float *co, const float *no)
                                           Should return a FilterResult if set. */
struct PBVHNodeVertRange {
  struct iterator {
    PBVHVertRef vertex;
    int index;

    float *co;
    float *no;
    float *mask;
    int vertex_node_index; /* Index of vertex within the node. */

    PBVHNode *node;
    NodeData *node_data;

    FilterResult *result = nullptr;

    bool is_mesh = pbvh_type == PBVH_FACES;

    inline iterator(PBVH *pbvh,
                    PBVHNode *node_,
                    NodeData *node_data_,
                    FilterFunc &filter_func) noexcept
        : node(node_), node_data(node_data_), filter_func_(filter_func), pbvh_(pbvh)
    {
      /* Zero vd_. */
      vd_ = {};

      pbvh_vertex_iter_init(pbvh, node, &vd_, PBVH_ITER_UNIQUE);

      if (vd_.i < vd_.totvert && apply_filter()) {
        iterator::operator++();
      }
    }

    inline iterator(const iterator &b) noexcept : filter_func_(b.filter_func_)
    {
      vertex = b.vertex;
      index = b.index;
      co = b.co;
      no = b.no;
      mask = b.mask;
      vertex_node_index = b.vertex_node_index;
      result_data_ = b.result_data_;

      if (b.result) {
        result = &result_data_;
      }

      node = b.node;
      node_data = b.node_data;

      vd_ = b.vd_;
    }

    inline iterator &operator*() noexcept
    {
      return *this;
    }

    inline bool apply_filter() noexcept
    {
      load_data();

      if constexpr (std::is_invocable_v<FilterFunc, PBVHVertRef, const float *, const float *>) {
        result_data_ = filter_func_(vertex, co, no);
        result = &result_data_;

        return !result_data_.in_range;
      }

      return false;
    }

     inline iterator &operator++() noexcept
    {
      if constexpr (pbvh_type == PBVH_GRIDS) {
        bool visible = true;

        do {
          vd_.gx++;
          vd_.i++;
          vd_.index++;
          vd_.vertex.i++;
          vd_.grid = CCG_elem_next(&vd_.key, vd_.grid);

          if (vd_.gx >= vd_.gridsize) {
            vd_.gx = 0;
            vd_.gy++;
          }

          if (vd_.gy >= vd_.gridsize) {
            vd_.gy = 0;
            vd_.g++;

            if (vd_.g < vd_.totgrid) {
              vd_.grid = vd_.grids[vd_.grid_indices[vd_.g]];
              vd_.gh = vd_.grid_hidden ? vd_.grid_hidden[vd_.grid_indices[vd_.g]] : nullptr;
              vd_.index = vd_.vertex.i = vd_.grid_indices[vd_.g] * vd_.key.grid_area;
            }
            else {
              vd_.index = vd_.vertex.i = -1;
              vd_.grid = nullptr;
              vd_.gh = nullptr;
            }
          }

          visible = vd_.gh ? !BLI_BITMAP_TEST(vd_.gh, vd_.gy * vd_.gridsize + vd_.gx) : true;
          if (vd_.grid && apply_filter()) {
            visible = false;
          }
        } while (vd_.grid && !visible);
      }
      else if constexpr (pbvh_type == PBVH_FACES) {
        bool visible = false;

        do {
          vd_.i++;

          visible = vd_.i < vd_.totvert ?
                        !(vd_.hide_vert && vd_.hide_vert[vd_.vert_indices[vd_.i]]) :
                        true;

          if (vd_.i < vd_.totvert && apply_filter()) {
            visible = false;
          }
        } while (!visible);
      }
      else {
        bool visible = false;

        do {
          if (vd_.bm_iter != vd_.bm_iter_end) {
            vd_.bm_vert = *vd_.bm_iter;
            ++vd_.bm_iter;
          }
          else {
            vd_.bm_vert = nullptr;
          }

          vd_.i++;

          visible = vd_.bm_vert ? !BM_elem_flag_test_bool(vd_.bm_vert, BM_ELEM_HIDDEN) : true;
          if (vd_.bm_vert && apply_filter()) {
            visible = false;
          }
        } while (!visible);

        if (!vd_.bm_vert) {
          vd_.i = vd_.totvert; /* Signal end condition. */
        }
      }

      return *this;
    }

    inline bool operator==(const iterator &b) const noexcept
    {
      return b.vd_.i == vd_.i;
    }

    inline bool operator!=(const iterator &b) const noexcept
    {
      return b.vd_.i != vd_.i;
    }

   private:
     inline void load_data() noexcept
    {
      if constexpr (pbvh_type == PBVH_GRIDS) {
        vertex = vd_.vertex;
        index = vd_.index;
        co = CCG_elem_co(&vd_.key, vd_.grid);
        no = CCG_elem_no(&vd_.key, vd_.grid);
        mask = vd_.key.has_mask ? CCG_elem_mask(&vd_.key, vd_.grid) : nullptr;
      }
      else if constexpr (pbvh_type == PBVH_FACES) {
        index = vertex.i = vd_.vert_indices[vd_.i];
        co = vd_.vert_positions[index];
        no = vd_.vert_normals[index];
        mask = vd_.vmask ? vd_.vmask + vd_.index : nullptr;
      }
      else {
        vertex = BKE_pbvh_make_vref((intptr_t)vd_.bm_vert);
        index = BM_elem_index_get(vd_.bm_vert);
        co = vd_.bm_vert->co;
        no = vd_.bm_vert->no;
        mask = (float *)BM_ELEM_CD_GET_VOID_P(vd_.bm_vert, vd_.cd_vert_mask_offset);
      }

      vertex_node_index = vd_.i;
    }

    friend struct PBVHNodeVertRange;

    FilterFunc &filter_func_;
    PBVH *pbvh_;
    PBVHVertexIter vd_;
    FilterResult result_data_;
  };

  inline iterator begin() noexcept
  {
    return iterator(pbvh_, node_, node_data_, filter_func_);
  }

  inline const iterator &end() noexcept
  {
#if 0
    iterator ret = iterator(pbvh_, node_, node_data_, filter_func_);

    ret.vd_.i = ret.vd_.totvert;

    return ret;
#else
    return end_;
#endif
  }

  inline PBVHNodeVertRange(PBVH *pbvh,
                           PBVHNode *node,
                           NodeData *node_data,
                           FilterFunc &filter_func) noexcept
      : pbvh_(pbvh),
        node_(node),
        node_data_(node_data),
        filter_func_(filter_func),
        end_(pbvh, node, node_data, filter_func)
  {
    end_.vd_.i = end_.vd_.totvert;
  }

 private:
  PBVH *pbvh_;
  PBVHNode *node_;
  NodeData *node_data_;
  FilterFunc &filter_func_;
  iterator end_;
};

/** Vertex list range.
 *
 * See foreach_brush_flatlist.
 */
template<typename NodeData, PBVHType pbvh_type, typename FilterResult = FilterResultAbstract>
struct VertexListRange {
  struct VertTriplet {
    PBVHVertRef vertex;
    int node_index;        /* Index of node within the brush nodes vector. */
    int vertex_node_index; /* Index of vertex inside node list. */
    FilterResult filter_result;
  };

  struct iterator {
    PBVHVertRef vertex;
    int index; /* Vertex index. */

    float *co, *mask;
    const float *no;
    const FilterResult *result;

    PBVHNode *node;
    NodeData *node_data;
    int vertex_node_index; /* Index of vertex inside node list. */

    bool is_mesh;

    inline iterator(IndexRange _range, VertexListRange &_owner, int _i) noexcept
        : i(_i), owner(_owner), range(_range)
    {
      is_mesh = pbvh_type == PBVH_FACES;

      if (i >= owner.range.start() && i <= owner.range.last()) {
        load_data();
      }
    }

    inline iterator(const iterator &b) noexcept : i(b.i), owner(b.owner), range(b.range)
    {
      is_mesh = pbvh_type == PBVH_FACES;

      if (i >= owner.range.start() && i <= owner.range.last()) {
        load_data();
      }
    }

    inline iterator &operator*() const noexcept
    {
      return *this;
    }

    inline bool operator==(const iterator &b) const noexcept
    {
      return b.i == i;
    }

    inline bool operator!=(const iterator &b) const noexcept
    {
      return b.i != i;
    }

    inline iterator &operator++() noexcept
    {
      i++;
      load_data();
      return *this;
    }

     inline void load_data() noexcept
    {
      if (i == range.start() + range.size()) {
        return;
      }

      const VertTriplet &vt = owner.verts[i];

      node = owner.nodes[vt.node_index];
      vertex = vt.vertex;
      index = BKE_pbvh_vertex_to_index(owner.pbvh, vertex);

      node_data = &owner.node_data[vt.node_index];
      vertex_node_index = vt.vertex_node_index;
      result = &vt.filter_result;

      if constexpr (pbvh_type == PBVH_BMESH) {
        BMVert *v = reinterpret_cast<BMVert *>(vertex.i);

        co = v->co;
        no = v->no;

        mask = owner.cd_vert_mask_offset != -1 ?
                   static_cast<float *>(BM_ELEM_CD_GET_VOID_P(v, owner.cd_vert_mask_offset)) :
                   nullptr;
      }
      else if constexpr (pbvh_type == PBVH_FACES) {
        mask = owner.vert_mask ? owner.vert_mask + vertex.i : nullptr;
        co = owner.vert_positions[vertex.i];
        no = owner.vert_normals[vertex.i];
      }
      else if constexpr (pbvh_type == PBVH_GRIDS) {
        const int grid_index = vertex.i / owner.key->grid_area;
        const int vertex_index = vertex.i - grid_index * owner.key->grid_area;
        CCGElem *elem = owner.grids[grid_index];

        co = CCG_elem_co(owner.key, CCG_elem_offset(owner.key, elem, vertex_index));
      }
    }

   private:
    int i;
    VertexListRange &owner;
    IndexRange range;
  };

  PBVH *pbvh;
  Span<VertTriplet> verts;
  IndexRange range;
  float *vert_mask;
  MutableSpan<NodeData> node_data;
  Span<PBVHNode *> nodes;

  /* PBVH_FACES */
  float (*vert_positions)[3];
  const float (*vert_normals)[3];

  /* PBVH_GRIDS */
  const CCGKey *key;
  CCGElem **grids;

  /* PBVH_BMESH */
  int cd_vert_mask_offset;

  inline VertexListRange(PBVH *_pbvh,
                         Span<VertTriplet> _verts,
                         Span<PBVHNode *> _nodes,
                         MutableSpan<NodeData> _node_data,
                         IndexRange _range) noexcept
      : pbvh(_pbvh), verts(_verts), range(_range), node_data(_node_data), nodes(_nodes)
  {
    if constexpr (pbvh_type == PBVH_FACES) {
      vert_mask = static_cast<float *>(
          CustomData_get_layer_for_write(pbvh->vdata, CD_PAINT_MASK, pbvh->totvert));
      vert_positions = BKE_pbvh_get_vert_positions(pbvh);
      vert_normals = BKE_pbvh_get_vert_normals(pbvh);
    }
    else if constexpr (pbvh_type == PBVH_BMESH) {
      cd_vert_mask_offset = CustomData_get_offset(&BKE_pbvh_get_bmesh(pbvh)->vdata, CD_PAINT_MASK);
    }
    else if constexpr (pbvh_type == PBVH_GRIDS) {
      key = BKE_pbvh_get_grid_key(pbvh);
      grids = BKE_pbvh_get_grids(pbvh);
    }
  }

  inline iterator begin() noexcept
  {
    return iterator(range, *this, range.start());
  }

  inline iterator end() noexcept
  {
    return iterator(range, *this, range.start() + range.size());
  }
};

/*
 * Vertex filter callback.  The callback should
 * return an object with an in_range boolean.
 *
 * Abstract class, can be a lambda.
 */
struct PBVHVertexFilter {
  struct Result {
    bool in_range = true;
  };

  Result operator()(PBVHVertRef /*vertex*/, const float * /*co*/, const float * /*no*/) noexcept
  {
    return Result();
  }
};

template<typename NodeData,
         PBVHType pbvh_type,
         typename ExecFunc,
         typename FilterFunc = blender::bke::pbvh::PBVHVertexFilter>
void foreach_brush_verts_simple_intern(
    PBVH *pbvh,
    Span<PBVHNode *> nodes,
    bool threaded,
    FilterFunc filter_verts,
    std::function<NodeData(PBVHNode *node)> node_visit_pre,
    ExecFunc exec,
    std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
    int repeat = 0) noexcept
{
  Vector<NodeData> node_datas;

  using FilterResult =
      typename std::invoke_result_t<FilterFunc, PBVHVertRef, const float *, const float *>;
  using PBVHNodeVertRangeImpl = PBVHNodeVertRange<NodeData, pbvh_type, FilterResult, FilterFunc>;

  for (PBVHNode *node : nodes) {
    if (node_visit_pre) {
      node_datas.append(node_visit_pre(node));
    }
    else {
      node_datas.resize(node_datas.size() + 1);
    }
  }

  for (int iteration : IndexRange(repeat + 1)) {
    if (!threaded) {
      for (int i : nodes.index_range()) {
        exec(PBVHNodeVertRangeImpl(pbvh, nodes[i], &node_datas[i], filter_verts));
      }
    }
    else {
      threading::parallel_for_each(nodes.index_range(), [&](int i) {
        exec(PBVHNodeVertRangeImpl(pbvh, nodes[i], &node_datas[i], filter_verts));
      });
    }
  }

  if (node_visit_post) {
    for (int i : nodes.index_range()) {
      node_visit_post(nodes[i], &node_datas[i]);
    }
  }
}

template<typename NodeData,
         typename ExecFunc,
         typename FilterFunc = blender::bke::pbvh::PBVHVertexFilter>
void foreach_brush_verts_simple(
    PBVH *pbvh,
    Span<PBVHNode *> nodes,
    bool threaded,
    FilterFunc filter_verts,
    std::function<NodeData(PBVHNode *node)> node_visit_pre,
    ExecFunc exec,
    std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
    int repeat = 0) noexcept
{
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_GRIDS:
      foreach_brush_verts_simple_intern<NodeData, PBVH_GRIDS>(
          pbvh, nodes, threaded, filter_verts, node_visit_pre, exec, node_visit_post, repeat);
      break;
    case PBVH_FACES:
      foreach_brush_verts_simple_intern<NodeData, PBVH_FACES>(
          pbvh, nodes, threaded, filter_verts, node_visit_pre, exec, node_visit_post, repeat);
      break;
    case PBVH_BMESH:
      foreach_brush_verts_simple_intern<NodeData, PBVH_BMESH>(
          pbvh, nodes, threaded, filter_verts, node_visit_pre, exec, node_visit_post, repeat);
      break;
  }
}

template<typename NodeData,
         PBVHType pbvh_type,
         typename ExecFunc,
         typename FilterFunc = blender::bke::pbvh::PBVHVertexFilter>
 void foreach_brush_verts_flatlist_intern(
    PBVH *pbvh,
    Span<PBVHNode *> nodes,
    bool threaded,
    FilterFunc filter_verts,
    std::function<NodeData(PBVHNode *node)> node_visit_pre,
    ExecFunc exec,
    std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
    int repeat = 0) noexcept
{
  // SCOPED_TIMER(__func__);

  /* Should return a type compatible with FilterResultAbstract. */
  using FilterResult =
      typename std::invoke_result_t<FilterFunc, PBVHVertRef, const float *, const float *>;
  using VertexListRangeImpl = VertexListRange<NodeData, pbvh_type, FilterResult>;
  using VertTriplet = typename VertexListRangeImpl::VertTriplet;

  Array<NodeData> node_data(nodes.size());
  Array<bool> used_nodes(nodes.size());
  Array<Vector<VertTriplet>> node_verts(nodes.size());

  Vector<VertTriplet> verts;

  if (threaded) {
    threading::parallel_for_each(nodes.index_range(), [&](int i) {
      node_data[i] = node_visit_pre(nodes[i]);  //
    });
  }
  else {
    for (int i : nodes.index_range()) {
      node_data[i] = node_visit_pre(nodes[i]);
    }
  }

  threading::parallel_for(nodes.index_range(), 1, [&](IndexRange range) {
    for (int i : range) {
      PBVHNode *node = nodes[i];

      bool used = false;

      PBVHVertexIter vd;
      BKE_pbvh_vertex_iter_begin (pbvh, node, vd, PBVH_ITER_UNIQUE) {
        FilterResult result = filter_verts(vd.vertex, vd.co, vd.no ? vd.no : vd.fno);

        if (result.in_range) {
          node_verts[i].append({vd.vertex, i, vd.i, result});
          used = true;
        }
      }
      BKE_pbvh_vertex_iter_end;

      used_nodes[i] = used;
    }
  });

  for (auto &verts2 : node_verts) {
    verts.extend(verts2);
  }

  int thread_count = BLI_system_thread_count();
  const int grain_size = max_ii(verts.size() / thread_count / 4, 4);

#if 0
  printf("%s: threads: %d nodes: %d, grain_size: %d, vert_size: %d\n",
         __func__,
         thread_count,
         int(nodes.size()),
         grain_size,
         int(verts.size()));
#endif

  for (int iteration : IndexRange(repeat + 1)) {
    if (!threaded) {
      exec(VertexListRangeImpl(pbvh, verts, nodes, node_data, verts.index_range()));
    }
    else {
      threading::parallel_for(verts.index_range(), grain_size, [&](IndexRange range) {
        exec(VertexListRangeImpl(pbvh, verts, nodes, node_data, range));
      });
    }
  }

  for (int i : nodes.index_range()) {
    node_visit_post(nodes[i], &node_data[i]);
  }
}

/**
 *
 * PBVHNodes make a poor boundary for parallel task execution due to
 * their large size, which is done for GPU bandwidth reasons.
 *
 * This version of foreach_brush_verts puts the filtered vertex list
 * into a flat array and feeds that to parallel_for with a finer grain size.
 *
 * NodeData is per-node data provided by the client.
 */
template<typename NodeData,
         typename ExecFunc,
         typename FilterFunc = blender::bke::pbvh::PBVHVertexFilter>
void foreach_brush_verts_flatlist(
    PBVH *pbvh,
    Span<PBVHNode *> nodes,
    bool threaded,
    FilterFunc filter_func, /* Apply brush radius test here, r_dist_squared must be set. */
    std::function<NodeData(PBVHNode *node)> node_visit_pre, /* Visit nodes before exec.*/
    ExecFunc exec, /* Main execution function. [&](auto range) { for (auto &vd : range) {} } */
    std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
    int repeat = 0) /* Visit nodes after exec. */ noexcept
{
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES:
      foreach_brush_verts_flatlist_intern<NodeData, PBVH_FACES, ExecFunc, FilterFunc>(
          pbvh,
          nodes,
          threaded,
          filter_func,
          node_visit_pre,
          std::forward<ExecFunc>(exec),
          node_visit_post,
          repeat);
      break;
    case PBVH_BMESH:
      foreach_brush_verts_flatlist_intern<NodeData, PBVH_BMESH, ExecFunc, FilterFunc>(
          pbvh, nodes, threaded, filter_func, node_visit_pre, exec, node_visit_post, repeat);
      break;
    case PBVH_GRIDS:
      foreach_brush_verts_flatlist_intern<NodeData, PBVH_GRIDS, ExecFunc, FilterFunc>(
          pbvh, nodes, threaded, filter_func, node_visit_pre, exec, node_visit_post, repeat);
      break;
  }
}

/** Parallelized interface for executing a task over PBVH node vertices.
 *  For a higher-level version of this API see blender::editors::sculpt::exec_brush
 *
 * To maximize CPU saturation one of two different implementations will be
 * used, foreach_brush_simple (which splits tasks by node)
 * or foreach_brush_flatlist (which builds a flat, filtered list of vertices
 * to feed to parallel_for).
 *
 * Example:
 *
 * struct NodeTaskData {
 *   // put any per-node data this task needs here.
 * };
 * foreach_brush_verts<NodeTaskData>(
 *                     pbvh,
 *                     nodes,
 *                     true,
 *                     [](PBVHVertex vertex, float *co, float *no) {
 *                       // Filter verts here.  Return a
 *                       // struct with an in_range bool.
 *                       // This design is necassary for blender::editors::exec_brush
 *
 *                      },
 *                     [](PBVHNode *node) {
 *                       return NodeTaskData();
 *                     },
 *                     [](auto &range) {
 *                        for (auto &vd : range) {
 *                          // do something with vd.co, vd.no, vd.vertex, etc.
 *                        }
 *                     },
 *                     [](PBVHNode *node, MyNodeData *data) {
 *                       //post-execution per-node callback.
 *                     });
 *
 */
template<typename NodeData,
         typename ExecFunc,
         typename FilterFunc = blender::bke::pbvh::PBVHVertexFilter>
void foreach_brush_verts(
    PBVH *pbvh,
    Span<PBVHNode *> nodes,
    bool threaded,
    FilterFunc filter_func, /* Apply brush radius test here, r_dist_squared must be set. */
    std::function<NodeData(PBVHNode *node)> node_visit_pre, /* Visit nodes before exec.*/
    ExecFunc exec, /* Main execution function. [&](auto range) { for (auto &vd : range) {} } */
    std::function<void(PBVHNode *node, NodeData *node_data)>
        node_visit_post, /* Visit nodes after exec. */
    int repeat = 0)
{

  //printf("limit count: %d\n", int(pbvh->leaf_limit * nodes.size()));

  if (pbvh->leaf_limit * nodes.size() > 70000) {
    foreach_brush_verts_simple<NodeData>(
        pbvh, nodes, threaded, filter_func, node_visit_pre, exec, node_visit_post, repeat);
  }
  else {
    foreach_brush_verts_flatlist<NodeData>(
        pbvh, nodes, threaded, filter_func, node_visit_pre, exec, node_visit_post, repeat);
  }
}

}  // namespace blender::bke::pbvh
