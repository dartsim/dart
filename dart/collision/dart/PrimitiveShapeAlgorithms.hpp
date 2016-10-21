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

#ifndef DART_COLLISION_DART_PRIMITIVESHAPEALGORITHMS_HPP_
#define DART_COLLISION_DART_PRIMITIVESHAPEALGORITHMS_HPP_

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/NarrowPhaseAlgorithms.hpp"
#include "dart/collision/dart/ConvexityBasedAlgorithms.hpp"
#include "dart/collision/dart/CollideSphereSphere.hpp"
#include "dart/dynamics/BoxShape.hpp"
//#include "dart/dynamics/Halfspace.hpp"
#include "dart/dynamics/SphereShape.hpp"

namespace dart {

namespace dynamics {
class Shape;
} // namespace dynamics

namespace collision {

class PrimitiveShapeAlgorithms
{
public:

//  template <typename ShapeA, typename ShapeB>
//  static void collide(const CollisionObject* objectA,
//                      const CollisionObject* objectB,
//                      NarrowPhaseCallback* callback = nullptr)
//  {
//    collide<ShapeA, ShapeB>(objectA->getShape().get(),
//                            objectA->getTransform(),
//                            objectB->getShape().get(),
//                            objectB->getTransform(),
//                            callback);
//  }

  template <typename ShapeA, typename ShapeB>
  static void collide(const dynamics::Shape* shapeA,
                      const Eigen::Isometry3d& tfA,
                      const dynamics::Shape* shapeB,
                      const Eigen::Isometry3d& tfB,
                      NarrowPhaseCallback* callback = nullptr)
  {
    collide<ShapeA, ShapeB>(*static_cast<const ShapeA*>(shapeA),
                            tfA,
                            *static_cast<const ShapeB*>(shapeB),
                            tfB,
                            callback);
  }
  // TODO(JS): add binary check function

  template <typename ShapeA, typename ShapeB>
  static void collide(const ShapeA& shapeA,
                      const Eigen::Isometry3d& tfA,
                      const ShapeB& shapeB,
                      const Eigen::Isometry3d& tfB,
                      NarrowPhaseCallback* callback = nullptr)
  {
    // Convexity based algorithm is used if a specialized function is
    // provided for a shape pair of ShapeA and ShapeB.
    ConvexityBasedAlgorithms::collide(shapeA, tfA, shapeB, tfB, callback);
  }
};

//============================================================================//
//                                                                            //
//                              Specializations                               //
//                                                                            //
//============================================================================//

// Po: Point
// Li: Line
// Tr: Triangle
// Pl: Plane
// Ha: Halfspace
// Sp: Sphere
// Ci: Cubic
// El: Ellipsoid
// Cu: Cuboid (Box)
// Ca: Capsule
// Cy: Cylinder
// Co: Cone
// Py: Pyramid

//----+----+----+----+----+----+----+----+----+----+----+----+----+----
//    | Po | Li | Tr | Pl | Ha | Sp | Ci | El | Cu | Ca | Cy | Co | Py
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Po |    |    |    |    |    |    |    |    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Li |----|    |    |    |    |    |    |    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Tr |----|----|    |    |    |    |    |    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Pl |----|----|----|    |    |    |    |    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Ha |----|----|----|----|    | O  |    |    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Sp |----|----|----|----|----| O  |    |    | O  |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Ci |----|----|----|----|----|----|    |    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// El |----|----|----|----|----|----|----|    |    |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Cu |----|----|----|----|----|----|----|----| O  |    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Ca |----|----|----|----|----|----|----|----|----|    |    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Cy |----|----|----|----|----|----|----|----|----|----|    |    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Co |----|----|----|----|----|----|----|----|----|----|----|    |
//----+----+----+----+----+----+----+----+----+----+----+----+----+----
// Py |----|----|----|----|----|----|----|----|----|----|----|----|
//----+----+----+----+----+----+----+----+----+----+----+----+----+----

//==============================================================================
template <>
void PrimitiveShapeAlgorithms::collide(const dynamics::SphereShape& shapeA,
                                       const Eigen::Isometry3d& tfA,
                                       const dynamics::SphereShape& shapeB,
                                       const Eigen::Isometry3d& tfB,
                                       NarrowPhaseCallback* callback);

//==============================================================================
template <>
void PrimitiveShapeAlgorithms::collide(const dynamics::SphereShape& shapeA,
                                       const Eigen::Isometry3d& tfA,
                                       const dynamics::BoxShape& shapeB,
                                       const Eigen::Isometry3d& tfB,
                                       NarrowPhaseCallback* callback);

//==============================================================================
template <>
void PrimitiveShapeAlgorithms::collide(const dynamics::BoxShape& shapeA,
                                       const Eigen::Isometry3d& tfA,
                                       const dynamics::SphereShape& shapeB,
                                       const Eigen::Isometry3d& tfB,
                                       NarrowPhaseCallback* callback);

//==============================================================================
template <>
void PrimitiveShapeAlgorithms::collide(const dynamics::BoxShape& shapeA,
                                       const Eigen::Isometry3d& tfA,
                                       const dynamics::BoxShape& shapeB,
                                       const Eigen::Isometry3d& tfB,
                                       NarrowPhaseCallback* callback);

//==============================================================================
//template <>
//std::size_t PrimitiveShapeAlgorithms::collide(const Halfspace& shapeA,
//                                              const Eigen::Isometry3d& tfA,
//                                              const dynamics::SphereShape& shapeB,
//                                              const Eigen::Isometry3d& tfB,
//                                              NarrowPhaseCallback* callback);

//==============================================================================
//template <>
//std::size_t PrimitiveShapeAlgorithms::collide(const dynamics::SphereShape& shapeA,
//                                              const Eigen::Isometry3d& tfA,
//                                              const Halfspace& shapeB,
//                                              const Eigen::Isometry3d& tfB,
//                                              NarrowPhaseCallback* callback);

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_PRIMITIVESHAPEALGORITHMS_HPP_
