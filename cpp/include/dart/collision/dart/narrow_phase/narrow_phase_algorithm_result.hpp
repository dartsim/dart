/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "dart/collision/contact_point.hpp"
#include "dart/collision/export.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/geometry/geometry3.hpp"
#include "dart/math/lie_group/se3.hpp"
#include "dart/math/type.hpp"

namespace dart::collision::detail {

template <typename Scalar_>
struct NarrowPhaseAlgorithmBatchTask
{
  // Type aliases
  using Scalar = Scalar_;

  struct Task
  {
    uint64_t overlapping_pairs_id;

    math::Geometry3<Scalar>* collision_geometry_a;

    math::Geometry3<Scalar>* collision_geometry_b;

    math::SE3<Scalar> pose_a;

    math::SE3<Scalar> pose_b;

    bool need_contact_point;

    bool is_colliding;

    common::vector<ContactPoint<Scalar>> contact_points;

    Task(common::MemoryAllocator& allocator) : contact_points(allocator)
    {
      // Do nothing
    }
  };

  /// Constructor
  NarrowPhaseAlgorithmBatchTask(common::MemoryAllocator& allocator)
    : m_tasks(allocator)
  {
    // Do nothing
  }

  /// Destructor
  virtual ~NarrowPhaseAlgorithmBatchTask() = default;

  virtual void add_contact_point(unsigned int index)
  {
    DART_NOT_IMPLEMENTED;
    DART_UNUSED(index);
    //    ContactPoint<Scalar> contact_point;
    //    contact_points[index].emplace_back(std::move(contact_point));
  }

  virtual void clear()
  {
    m_tasks.clear();
  }

private:
  common::vector<Task> m_tasks;
};

} // namespace dart::collision::detail
