#pragma once

#include "BLI_compiler_compat.h"
#include "BLI_map.hh"
#include "BLI_math.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "BKE_paint.h"
#include "BKE_pbvh.h"
#include "BKE_pbvh_iter.hh"

#include "sculpt_intern.hh"
#include <functional>

namespace blender::editors::sculpt {
/* Abstract class. */
class BrushTester {
  struct Result {
    float dist_squared;
    bool in_range;
    float calc_falloff() const
    {
      return 1.0f;
    }
  };

  Result operator()(PBVHVertRef /*vertex*/, const float * /*co*/, const float * /*no*/)
  {
    return Result();
  }
};

class SphereTester {
  float3 location_;
  SculptSession *ss_ = nullptr;
  const Brush *brush_ = nullptr;
  bool apply_mask_ = true;

 public:
  struct Result {
    bool in_range;
    float dist_squared;
    const float *co, *no;
    PBVHVertRef vertex;

    inline Result() noexcept {}
    inline Result(SphereTester *owner_) noexcept : owner(owner_) {}

    ATTR_NO_OPT inline float calc_falloff(int thread_id,
                                          AutomaskingNodeData *automask_data) const noexcept
    {
      float mask = owner->apply_mask_ ? SCULPT_vertex_mask_get(owner->ss_, vertex) : 0.0f;

      return SCULPT_brush_strength_factor(owner->ss_,
                                          owner->brush_,
                                          co,
                                          sqrtf(dist_squared),
                                          no,
                                          no,
                                          mask,
                                          vertex,
                                          thread_id,
                                          automask_data);
    }

   private:
    SphereTester *owner;
  };

  inline SphereTester(SculptSession *ss, const Brush *brush, bool apply_mask) noexcept
      : ss_(ss), brush_(brush), apply_mask_(apply_mask)
  {
    location_ = ss->cache->location;
  }

  inline SphereTester() noexcept {}

  ATTR_NO_OPT inline Result operator()(PBVHVertRef vertex,
                                       const float *co,
                                       const float *no) noexcept
  {
    Result res(this);

    res.dist_squared = len_squared_v3v3(location_, co);
    res.in_range = res.dist_squared < ss_->cache->radius_squared;
    res.co = co;
    res.no = no;
    res.vertex = vertex;

    return res;
  }
};

template<typename NodeData, typename VertexRange> struct ForwardVertexIter {
  using base_iterator =
      typename VertexRange::iterator;  // std::invoke_result_t<decltype(&VertexRange::begin)()>;

  struct iterator {
    PBVHVertRef vertex;
    int index;

    float falloff;

    float *co;
    const float *no;
    float *mask;

    AutomaskingNodeData *automask_data;
    SculptOrigVertData *orig_data;

    int vertex_node_index;
    int thread_id;

    bool is_mesh;

    inline iterator(SculptSession *ss, base_iterator vd, base_iterator end_vd) noexcept
        : vd_(vd), end_(end_vd), ss_(ss)
    {
      const int thread_id = BLI_task_parallel_thread_id(nullptr);

      if (vd_ != end_) {
        if (!vd_.result->in_range) {
          falloff = 0.0f; /* Signal find_valid_iter to skip the first element. */
        }
        else {
          load_data();
        }

        find_valid_iter();
      }
    }

    inline iterator(const iterator &b) noexcept
    {
      vd_ = b.vd_;
      end_ = b.end_;
      thread_id = b.thread_id;

      load_data();
    }

    inline iterator &operator*() noexcept
    {
      return *this;
    }

    inline iterator &operator++() noexcept
    {
      ++vd_;

      if (vd_ != end_) {
        load_data();
        find_valid_iter();
      }

      return *this;
    }

    inline bool operator==(const iterator &b) noexcept
    {
      return b.vd_ == vd_;
    }

    inline bool operator!=(const iterator &b) noexcept
    {
      return b.vd_ != vd_;
    }

   private:
    inline void load_data() noexcept
    {
      //
      if (vd_ == end_) {
        return;
      }

      vertex = vd_.vertex;
      vertex_node_index = vd_.vertex_node_index;
      index = vd_.index;

      automask_data = &vd_.node_data->automask_data;
      orig_data = &vd_.node_data->orig_data;

      is_mesh = vd_.is_mesh;

      SCULPT_automasking_node_update<NodeData>(ss_, automask_data, vd_);

      co = vd_.co;
      no = vd_.no;
      mask = vd_.mask;

      falloff = vd_.result->calc_falloff(thread_id, &vd_.node_data->automask_data);
    }

    inline void find_valid_iter() noexcept
    {
      while (vd_ != end_ && falloff == 0.0f) {
        ++vd_;

        if (vd_.result->in_range && vd_ != end_) {
          load_data();
        }
      }
    }

    base_iterator vd_, end_;
    SculptSession *ss_;
  };

  inline ForwardVertexIter(SculptSession *ss, VertexRange &range) noexcept : range_(range), ss_(ss)
  {
  }

  inline iterator begin() noexcept
  {
    return iterator(ss_, range_.begin(), range_.end());
  }

  inline iterator end() noexcept
  {
    return iterator(ss_, range_.end(), range_.end());
  }

 private:
  VertexRange &range_;
  SculptSession *ss_;
};

template<typename NodeData, typename ExecFunc, typename Tester = BrushTester>
ATTR_NO_OPT void exec_brush_intern(
    Object *ob,
    Span<PBVHNode *> nodes,
    bool threaded,         //
    Tester &brush_tester,  //
    std::function<NodeData(PBVHNode *node)> node_visit_pre,
    ExecFunc &exec, /* [&](auto &vertex_range) {} */
    std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
    int repeat = 0,
    bool apply_mask = true,
    bool needs_original = false,
    SculptUndoType original_type = SCULPT_UNDO_COORDS

    ) noexcept
{
  SculptSession *ss = ob->sculpt;

  struct BaseNodeData {
    AutomaskingNodeData automask_data;
    SculptOrigVertData orig_data;
    NodeData user_data;
  };

  blender::bke::pbvh::foreach_brush_verts<BaseNodeData>(  //
      ss->pbvh,
      nodes,
      threaded,
      brush_tester,
      [&](PBVHNode *node) {
        BaseNodeData data = {};

        SCULPT_automasking_node_begin(ob, ss, ss->cache->automasking, &data.automask_data, node);

        if (needs_original) {
          if (data.automask_data.have_orig_data && original_type == SCULPT_UNDO_COORDS) {
            data.orig_data = data.automask_data.orig_data;
          }
          else {
            SCULPT_orig_vert_data_init(&data.orig_data, ob, node, original_type);
          }
        }

        if (node_visit_pre) {
          data.user_data = node_visit_pre(node);
        }

        return data;
      },
      [&](auto range) { exec(ForwardVertexIter<BaseNodeData, decltype(range)>(ss, range)); },
      [&](PBVHNode *node, BaseNodeData *node_data) {
        if (node_visit_post) {
          node_visit_post(node, &node_data->user_data);
        }
      },
      repeat);
}

template<typename NodeData, typename ExecFunc>
void exec_brush(Object *ob,
                Span<PBVHNode *> nodes,
                bool threaded,  //
                std::function<NodeData(PBVHNode *node)> node_visit_pre,
                ExecFunc exec, /* [&](auto &vertex_range) {} */
                std::function<void(PBVHNode *node, NodeData *node_data)> node_visit_post,
                int repeat = 0,
                bool apply_mask = true,
                bool needs_original = false,
                SculptUndoType original_type = SCULPT_UNDO_COORDS

                ) noexcept
{
  SculptSession *ss = ob->sculpt;

  SphereTester brush_tester(ss, ss->cache->brush, apply_mask);

  exec_brush_intern<NodeData>(ob,
                              nodes,
                              threaded,
                              brush_tester,
                              node_visit_pre,
                              exec,
                              node_visit_post,
                              repeat,
                              apply_mask,
                              needs_original,
                              original_type);
}

}  // namespace blender::editors::sculpt
