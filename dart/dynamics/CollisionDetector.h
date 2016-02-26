/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_COLLISIONDETECTOR_H_
#define DART_DYNAMICS_COLLISIONDETECTOR_H_

#include <memory>

#include "dart/collision/CollisionDetector.h"
#include "dart/collision/Engine.h"
#include "dart/collision/fcl/FCLEngine.h"
#include "dart/collision/CollisionObject.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace dynamics {

class ShapeFrameCollisionObject : public collision::CollisionObject
{
public:

  ShapeFrameCollisionObject(
      const collision::CollisionDetectorPtr& collisionDetector,
      const ShapePtr& shape,
      const BodyNodePtr& bodyNode);
  // TODO(JS): this should be replaced by ShapeNode

  // Documentation inherited
  const Eigen::Isometry3d getTransform() const override;

  /// Return BodyNode pointer associated with this ShapeFrameCollisionObject
  dynamics::BodyNodePtr getBodyNode() const;

protected:

  dynamics::BodyNodePtr mBodyNode;
  // TODO(JS): this should be changed to ShapeNode

};

using ShapeFrameCollisionObjectPtr = std::shared_ptr<ShapeFrameCollisionObject>;

/// Create a ShapeFrameCollisionObject given ShapeNode
ShapeFrameCollisionObjectPtr createShapeFrameCollisionObject(
    const collision::CollisionDetectorPtr& collisionDetector,
    const ShapePtr& shape,
    const BodyNodePtr& bodyNode);

/// Create a ShapeFrameCollisionObjects given Skeleton
std::vector<collision::CollisionObjectPtr> createShapeFrameCollisionObjects(
    const collision::CollisionDetectorPtr& collisionDetector,
    const dynamics::SkeletonPtr& skel);

/// Create a CollisionGroup given Skeleton
collision::CollisionGroupPtr createShapeFrameCollisionGroup(
    const collision::CollisionDetectorPtr& collisionDetector,
    const dynamics::SkeletonPtr& skel);

///// Checks the collisions between two BodyNodes for one time.
//bool detect(const DynamicsCollisionDetectorPtr& collisionDetector,
//            const dynamics::ShapePtr& shape1,
//            const BodyNodePtr& body1,
//            const dynamics::ShapePtr& shape2,
//            const BodyNodePtr& body2,
//            const collision::Option& option,
//            collision::Result& result);

///// Checks the collisions between two Skeletons for one time.
/////
///// This function creates two collision groups internally as local variables
///// of the skeletons for single time collision check. If you want to check
///// more than once, then create two collision groups explicitly using
///// createShapeNodeCollisionGroup() and check the collisions with the groups
///// for better performance.
//bool detect(const DynamicsCollisionDetectorPtr& collisionDetector,
//            const dynamics::SkeletonPtr& skel1,
//            const dynamics::SkeletonPtr& skel2,
//            const collision::Option& option,
//            collision::Result& result);

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/CollisionDetector.h"

#endif // DART_DYNAMICS_COLLISIONDETECTOR_H_
