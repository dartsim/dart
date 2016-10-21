/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COLLISION_DART_NARROWPHASEALGORITHMS_HPP_
#define DART_COLLISION_DART_NARROWPHASEALGORITHMS_HPP_

#include <Eigen/Dense>

#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace collision {

struct NarrowPhaseCallback
{
  /// Return true if no more this function needs to be called.
  virtual bool notifyContact(const dynamics::Shape* shapeA,
                             const dynamics::Shape* shapeB,
                             const Eigen::Vector3d& point,
                             const Eigen::Vector3d& normal,
                             double depth) = 0;
};

class NarrowPhaseAlgorithms
{
public:
  using CollisionFunction =
      void (*)(const dynamics::Shape* shape1,
               const Eigen::Isometry3d& tf1,
               const dynamics::Shape* shape2,
               const Eigen::Isometry3d& tf2,
               NarrowPhaseCallback* callback);

  virtual ~NarrowPhaseAlgorithms() = default;

  const CollisionFunction& getAlgorithm(
      dynamics::Shape::ShapeType typeA,
      dynamics::Shape::ShapeType typeB) const;

protected:
  NarrowPhaseAlgorithms() = default;

  std::array<
      std::array<CollisionFunction, dynamics::Shape::COUNT>,
      dynamics::Shape::COUNT
  > mTable;
};

class DefaultNarrowPhaseAlgorithms : public NarrowPhaseAlgorithms
{
public:
  DefaultNarrowPhaseAlgorithms();
};
// TODO(JS): move out this to a separate file

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_NARROWPHASEALGORITHM_HPP_
