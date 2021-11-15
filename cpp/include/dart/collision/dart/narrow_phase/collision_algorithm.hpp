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
#include "dart/collision/dart/narrow_phase/narrow_phase_callback.hpp"
#include "dart/collision/dart/narrow_phase/type.hpp"

namespace dart::collision::detail {

template <typename Scalar_>
struct CollisionAlgorithmCreateFunc
{
  using Scalar = Scalar_;

  bool swapped;

  CollisionAlgorithmCreateFunc() : swapped(false)
  {
    // Do nothing
  }

  virtual ~CollisionAlgorithmCreateFunc() = default;

  virtual CollisionAlgorithm<Scalar>* create(
      const Object<Scalar>* object_a,
      const Object<Scalar>* object_b,
      CollisionAlgorithmManager<Scalar>* manager)
      = 0;
};

template <typename AlgorithmT>
struct StatelessCollisionAlgorithmCreateFunc
  : public CollisionAlgorithmCreateFunc<typename AlgorithmT::Scalar>
{
  using Scalar = typename AlgorithmT::Scalar;

  CollisionAlgorithm<Scalar>* create(
      const Object<Scalar>* object_a,
      const Object<Scalar>* object_b,
      CollisionAlgorithmManager<Scalar>* manager) final
  {
    DART_UNUSED(object_a, object_b, manager);
    static AlgorithmT algorithm;
    return &algorithm;
  }
};

template <typename Scalar_>
class CollisionAlgorithm
{
public:
  using Scalar = Scalar_;

  CollisionAlgorithm();

  virtual ~CollisionAlgorithm();

  virtual bool run(
      const Object<Scalar>& o1,
      const Object<Scalar>& o2,
      NarrowPhaseCallback<Scalar>* callback)
      = 0;

protected:
private:
};

} // namespace dart::collision::detail

#include "dart/collision/dart/narrow_phase/collision_algorithm_impl.hpp"
