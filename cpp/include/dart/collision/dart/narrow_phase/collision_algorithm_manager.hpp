/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/collision/dart/dart_type.hpp"
#include "dart/collision/dart/narrow_phase/type.hpp"

namespace dart::collision::detail {

template <typename Scalar_>
class CollisionAlgorithmManager
{
public:
  using Scalar = Scalar_;

  CollisionAlgorithmManager();

  ~CollisionAlgorithmManager() = default;

  CollisionAlgorithm<Scalar>* create_algorithm(
      DartObject<Scalar>* object_a, DartObject<Scalar>* object_b)
  {
    DART_ASSERT(object_a);
    DART_ASSERT(object_b);

    DART_ASSERT(object_a->get_geometry());
    DART_ASSERT(object_b->get_geometry());

    const auto type_a = object_a->get_geometry()->get_type();
    const auto type_b = object_b->get_geometry()->get_type();

    auto& create_function = m_collision_matrix[type_a][type_b];
    DART_ASSERT(create_function);

    return create_function->create(object_a, object_b, this);
  }

  bool destroy_algorithm(CollisionAlgorithm<Scalar>* algorithm)
  {
    DART_UNUSED(algorithm);
    return true;
  }

protected:
  CollisionAlgorithmCreateFunc<Scalar>* m_collision_matrix[16][16];

private:
};

} // namespace dart::collision::detail

#include "dart/collision/dart/narrow_phase/collision_algorithm_manager_impl.hpp"
