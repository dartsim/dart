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
#include "dart/collision/CollisionFilter.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/CollisionObject.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace dynamics {

struct BodyNodeCollisionFilter : collision::CollisionFilter
{
  bool needCollision(const collision::CollisionObject* object1,
                     const collision::CollisionObject* object2) const override;

  bool isAdjacentBodies(const BodyNode* bodyNode1,
                        const BodyNode* bodyNode2) const;
};

//class ShapeNodeCollisionObject : public collision::CollisionObject
//{
//public:

//  friend class collision::CollisionDetector;

//  // Documentation inherited
//  const Eigen::Isometry3d getTransform() const override;

//  // Documentation inherited
//  bool isEqual(const CollisionObject* other) const override;

//  /// Return ShapeNode pointer associated with this ShapeNodeCollisionObject
//  dynamics::ShapeNode* getShapeNode();

//  /// Return ShapeNode pointer associated with this ShapeNodeCollisionObject
//  const dynamics::ShapeNode* getShapeNode() const;

//  /// Return BodyNode pointer associated with this ShapeNodeCollisionObject
//  dynamics::BodyNode* getBodyNode();

//  /// Return BodyNode pointer associated with this ShapeNodeCollisionObject
//  const dynamics::BodyNode* getBodyNode() const;

//protected:

//  ShapeNodeCollisionObject(
//      const collision::CollisionDetectorPtr& collisionDetector,
//      const ShapePtr& shape,
//      const ShapeNodePtr& shapeNode);

//  dynamics::ShapeNodePtr mShapeNode;

//};

//using ShapeNodeCollisionObjectPtr = std::shared_ptr<ShapeNodeCollisionObject>;

///// Create a ShapeNodeCollisionObjectPtr given ShapeNode
//ShapeNodeCollisionObjectPtr createShapeNodeCollisionObject(
//    const collision::CollisionDetectorPtr& collisionDetector,
//    const ShapeNodePtr& shapeNode);

///// Create a ShapeNodeCollisionObjectPtr given Skeleton
//std::vector<collision::CollisionObjectPtr> createShapeFrameCollisionObjects(
//    const collision::CollisionDetectorPtr& collisionDetector,
//    const dynamics::SkeletonPtr& skel);

///// Create a CollisionGroup given Skeleton
//collision::CollisionGroupPtr createShapeFrameCollisionGroup(
//    const collision::CollisionDetectorPtr& collisionDetector,
//    const dynamics::SkeletonPtr& skel);

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/CollisionDetector.h"

#endif // DART_DYNAMICS_COLLISIONDETECTOR_H_
