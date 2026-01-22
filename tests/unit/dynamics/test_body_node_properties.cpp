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
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <optional>

using namespace dart::dynamics;

static SkeletonPtr createBodyNodeSkeleton()
{
  auto skeleton = Skeleton::create("body_props");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("body");
  return skeleton;
}

TEST(BodyNodeProperties, MassAndInertia)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->setMass(2.5);
  EXPECT_DOUBLE_EQ(body->getMass(), 2.5);
  EXPECT_DOUBLE_EQ(body->getInertia().getMass(), 2.5);

  body->setMomentOfInertia(1.1, 2.2, 3.3, 0.1, 0.2, 0.3);
  double ixx = 0.0;
  double iyy = 0.0;
  double izz = 0.0;
  double ixy = 0.0;
  double ixz = 0.0;
  double iyz = 0.0;
  body->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  EXPECT_DOUBLE_EQ(ixx, 1.1);
  EXPECT_DOUBLE_EQ(iyy, 2.2);
  EXPECT_DOUBLE_EQ(izz, 3.3);
  EXPECT_DOUBLE_EQ(ixy, 0.1);
  EXPECT_DOUBLE_EQ(ixz, 0.2);
  EXPECT_DOUBLE_EQ(iyz, 0.3);

  const Eigen::Vector3d localCom(0.1, -0.2, 0.3);
  body->setLocalCOM(localCom);
  EXPECT_TRUE(body->getLocalCOM().isApprox(localCom));
}

TEST(BodyNodeProperties, CollisionAndGravityModes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_TRUE(body->getGravityMode());
  EXPECT_TRUE(body->isCollidable());

  body->setGravityMode(false);
  body->setCollidable(false);
  EXPECT_FALSE(body->getGravityMode());
  EXPECT_FALSE(body->isCollidable());

  body->setGravityMode(true);
  body->setCollidable(true);
  EXPECT_TRUE(body->getGravityMode());
  EXPECT_TRUE(body->isCollidable());
}

TEST(BodyNodeProperties, ComputeInertiaFromShapeNodes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  auto sphere = std::make_shared<SphereShape>(0.5);
  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<VisualAspect>(sphere);

  const auto unused = body->computeInertiaFromShapeNodes(
      [](const ShapeNode*) -> std::optional<double> { return std::nullopt; });
  EXPECT_FALSE(unused.has_value());

  const auto inertia = body->computeInertiaFromShapeNodes(
      [&](const ShapeNode* shapeNode) -> std::optional<double> {
        if (shapeNode->getShape().get() == box.get()) {
          return 2.0;
        }
        if (shapeNode->getShape().get() == sphere.get()) {
          return 1.0;
        }
        return std::nullopt;
      });
  ASSERT_TRUE(inertia.has_value());
  EXPECT_DOUBLE_EQ(inertia->getMass(), 3.0);
}
