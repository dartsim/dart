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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

namespace {

SkeletonPtr createSphereSkeleton()
{
  auto skeleton = Skeleton::create("sphere");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "sphere_link";

  FreeJoint::Properties jointProps;
  jointProps.mName = "world";

  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProps, bodyProps);
  BodyNode* body = pair.second;
  auto shape = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(2.0));
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape);

  return skeleton;
}

} // namespace

// Regression test for https://github.com/dartsim/dart/issues/1534.
TEST(ShapeNodePtrTest, KeepsSkeletonAliveUntilReset)
{
  SkeletonPtr skeleton = createSphereSkeleton();
  ASSERT_TRUE(skeleton);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  ShapeNodePtr shape = body->getShapeNode(0);
  ASSERT_NE(shape, nullptr);

  std::weak_ptr<Skeleton> weakSkeleton = skeleton;
  skeleton.reset();

  EXPECT_FALSE(weakSkeleton.expired());
  EXPECT_NE(shape->getShape().get(), nullptr);
  EXPECT_EQ(shape->getBodyNodePtr().get(), body);
  EXPECT_EQ(shape->getSkeleton().get(), weakSkeleton.lock().get());

  shape = nullptr;
  EXPECT_TRUE(weakSkeleton.expired());
}
