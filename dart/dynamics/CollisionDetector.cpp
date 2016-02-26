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

#include "dart/dynamics/CollisionDetector.h"

#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
ShapeFrameCollisionObject::ShapeFrameCollisionObject(
    const collision::CollisionDetectorPtr& collisionDetector,
    const dynamics::ShapePtr& shape,
    const dynamics::BodyNodePtr& bodyNode)
  : collision::CollisionObject(collisionDetector, shape),
    mBodyNode(bodyNode)
{
  assert(collisionDetector);
  assert(shape);
  assert(bodyNode);

  auto found = false;
  auto numShapes = mBodyNode->getNumCollisionShapes();
  for (auto i = 0u; i < numShapes; ++i)
  {
    auto shapeIt = mBodyNode->getCollisionShape(i);
    if (shape == shapeIt)
    {
      found = true;
      break;
    }
  }

  if (!found)
  {
    dtwarn << "[ShapeFrameCollisionObject::constructor] Attempting to create "
           << "ShapeFrameCollisionObject with invalid pair of Shape and "
           << "BodyNode.\n";
    assert(false);
  }
}

//==============================================================================
const Eigen::Isometry3d ShapeFrameCollisionObject::getTransform() const
{
  return mBodyNode->getWorldTransform() * mShape->getLocalTransform();
}

//==============================================================================
BodyNodePtr ShapeFrameCollisionObject::getBodyNode() const
{
  return mBodyNode;
}

//==============================================================================
ShapeFrameCollisionObjectPtr createShapeFrameCollisionObject(
    const collision::CollisionDetectorPtr& collisionDetector,
    const ShapePtr& shape,
    const BodyNodePtr& bodyNode)
{
  return collisionDetector->createCollisionObject<ShapeFrameCollisionObject>(
        shape, bodyNode);
}

//==============================================================================
std::vector<collision::CollisionObjectPtr> createShapeFrameCollisionObjects(
    const collision::CollisionDetectorPtr& collisionDetector,
    const SkeletonPtr& skel)
{
  std::vector<collision::CollisionObjectPtr> objects;

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i)
  {
    auto bodyNode = skel->getBodyNode(i);
    auto numColShapes = bodyNode->getNumCollisionShapes();

    for (auto j = 0u; j < numColShapes; ++j)
    {
      auto shape = bodyNode->getCollisionShape(j);
      auto collObj =
          createShapeFrameCollisionObject(collisionDetector, shape, bodyNode);

      objects.push_back(
            std::static_pointer_cast<collision::CollisionObject>(collObj));
    }
  }

  return objects;
}

//==============================================================================
collision::CollisionGroupPtr createShapeFrameCollisionGroup(
    const collision::CollisionDetectorPtr& collisionDetector,
    const SkeletonPtr& skel)
{
  auto collObjs = createShapeFrameCollisionObjects(collisionDetector, skel);
  auto group = collisionDetector->createCollisionGroup(collObjs);

  return group;
}

////==============================================================================
//bool detect(const DynamicsCollisionDetectorPtr& collisionDetector,
//            const ShapePtr& shape1, const BodyNodePtr& body1,
//            const ShapePtr& shape2, const BodyNodePtr& body2,
//            const collision::Option& option,
//            collision::Result& result)
//{
//  auto obj1 = collisionDetector->createCollisionObject(shape1, body1);
//  auto obj2 = collisionDetector->createCollisionObject(shape2, body2);

//  return obj1->detect(obj2.get(), option, result);
//}

////==============================================================================
//bool detect(const DynamicsCollisionDetectorPtr& collisionDetector,
//            const SkeletonPtr& skel1,
//            const SkeletonPtr& skel2,
//            const collision::Option& option,
//            collision::Result& result)
//{
//  auto objects1 = createShapeFrameCollisionObjects(collisionDetector, skel1);
//  auto objects2 = createShapeFrameCollisionObjects(collisionDetector, skel2);

//  auto group1 = collisionDetector->createCollisionGroup(objects1);
//  auto group2 = collisionDetector->createCollisionGroup(objects2);

//  return group1->detect(group2.get(), option, result);
//}

} // namespace dynamics
} // namespace dart
