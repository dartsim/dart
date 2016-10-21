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

#include "dart/collision/dart/PrimitiveShapeAlgorithms.hpp"

#include "dart/collision/dart/CollideSphereSphere.hpp"
#include "dart/collision/dart/CollideBoxBox.hpp"
#include "dart/collision/dart/CollideHalfspaceSphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <>
void
PrimitiveShapeAlgorithms::collide(const dynamics::SphereShape& shapeA,
                                  const Eigen::Isometry3d& tfA,
                                  const dynamics::SphereShape& shapeB,
                                  const Eigen::Isometry3d& tfB,
                                  NarrowPhaseCallback* callback)
{
  collideSphereSphere(shapeA, tfA, shapeB, tfB, callback);
}

//==============================================================================
template <>
void
PrimitiveShapeAlgorithms::collide(const dynamics::BoxShape& shapeA,
                                  const Eigen::Isometry3d& tfA,
                                  const dynamics::BoxShape& shapeB,
                                  const Eigen::Isometry3d& tfB,
                                  NarrowPhaseCallback* callback)
{
  collideBoxBox(shapeA, tfA, shapeB, tfB, callback);
}

//==============================================================================
//template <>
//inline std::size_t
//PrimitiveShapeAlgorithms::collide(const Halfspace& shapeA,
//                                  const Eigen::Isometry3d& tfA,
//                                  const dynamics::SphereShape& shapeB,
//                                  const Eigen::Isometry3d& tfB,
//                                  NarrowPhaseCallback* callback)
//{
//  return collideHalfspaceSphere(shapeA, tfA, shapeB, tfB, callback);
//}

//==============================================================================
//template <>
//inline std::size_t
//PrimitiveShapeAlgorithms::collide(const dynamics::SphereShape& shapeA,
//                                  const Eigen::Isometry3d& tfA,
//                                  const Halfspace& shapeB,
//                                  const Eigen::Isometry3d& tfB,
//                                  NarrowPhaseCallback* callback)
//{
//  return collideSphereHalfspace(shapeA, tfA, shapeB, tfB, callback);
//}

} // namespace collision
} // namespace dart
