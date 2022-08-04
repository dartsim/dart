/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <vector>

#include "dart/collision/dart/dart_type.hpp"
#include "dart/collision/dart/narrow_phase/type.hpp"
#include "dart/common/macro.hpp"

namespace dart::collision::detail {

template <typename Scalar_>
class NarrowPhaseManager
{
public:
  using Scalar = Scalar_;

  explicit NarrowPhaseManager(
      common::MemoryAllocator& allocator,
      CollisionAlgorithmSelector<Scalar>& collision_algorithm_dispatch)
    : m_collision_algorithm_dispatch(collision_algorithm_dispatch),
      m_active_algorithms(allocator)
  {
    // Do nothing
  }

  virtual ~NarrowPhaseManager() = default;

  virtual void request_collision_check(
      DartObject<Scalar>* object_a, DartObject<Scalar>* object_b)
  {
    DART_UNUSED(object_a, object_b);
    // 1. Get or create algorithm by the geometry type

    // 2. Assign the request to the algorithm
    //    In this step, the concrete manager or the concrete algorithm split the
    //    job if it's needed.
  }

  virtual void check_collision()
  {
    // Ask all the (active) algorithms to perform collision checking.
    // It's assumed that the algorithms can run in parallel.
    for (auto& algorithm : m_active_algorithms) {
      algorithm->compute_collision_batch(nullptr);
    }
  }

protected:
private:
  CollisionAlgorithmSelector<Scalar>& m_collision_algorithm_dispatch;
  common::vector<CollisionAlgorithm<Scalar>*> m_active_algorithms;
};

} // namespace dart::collision::detail

#include "dart/collision/dart/narrow_phase/collision_algorithm_manager_impl.hpp"
