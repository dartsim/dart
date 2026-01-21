/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/collision/fcl/fcl_collision_detector.hpp"
#include "dart/collision/fcl/fcl_collision_object.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/sphere_shape.hpp"

#include <gtest/gtest.h>

using dart::collision::CollisionDetector;
using dart::collision::FCLCollisionDetector;

class ExposedFCLCollisionDetector : public FCLCollisionDetector
{
public:
  using FCLCollisionDetector::refreshCollisionObject;

  std::shared_ptr<dart::collision::CollisionObject> claim(
      const dart::dynamics::ShapeFrame* shapeFrame)
  {
    return CollisionDetector::claimCollisionObject(shapeFrame);
  }
};

TEST(FCLCollisionObject, RefreshRetainsUserData)
{
  ExposedFCLCollisionDetector detector;

  auto skel = dart::dynamics::Skeleton::create("skel");
  auto pair = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);
  auto shapeNode
      = pair.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(
          shape);

  auto obj = detector.claim(shapeNode);
  auto* fclObj = dynamic_cast<dart::collision::FCLCollisionObject*>(obj.get());
  ASSERT_NE(fclObj, nullptr);

  EXPECT_EQ(
      static_cast<void*>(fclObj),
      fclObj->getFCLCollisionObject()->getUserData());

  shape->setRadius(0.6);
  detector.refreshCollisionObject(fclObj);

  EXPECT_EQ(
      static_cast<void*>(fclObj),
      fclObj->getFCLCollisionObject()->getUserData());
}
