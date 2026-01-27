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
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/marker.hpp>
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

TEST(BodyNodeProperties, CreateShapeNodeWithVisualAspect)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->getNumShapeNodes(), 0u);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  ShapeNode* shapeNode = body->createShapeNodeWith<VisualAspect>(box);

  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_TRUE(shapeNode->has<VisualAspect>());
  EXPECT_FALSE(shapeNode->has<CollisionAspect>());
  EXPECT_EQ(shapeNode->getShape().get(), box.get());
}

TEST(BodyNodeProperties, CreateShapeNodeWithCollisionAspect)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto sphere = std::make_shared<SphereShape>(0.5);
  ShapeNode* shapeNode = body->createShapeNodeWith<CollisionAspect>(sphere);

  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_TRUE(shapeNode->has<CollisionAspect>());
  EXPECT_FALSE(shapeNode->has<VisualAspect>());
  EXPECT_EQ(shapeNode->getShape().get(), sphere.get());
}

TEST(BodyNodeProperties, CreateShapeNodeWithMultipleAspects)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  ShapeNode* shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_TRUE(shapeNode->has<VisualAspect>());
  EXPECT_TRUE(shapeNode->has<CollisionAspect>());
}

TEST(BodyNodeProperties, ShapeNodeAccessors)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto cylinder = std::make_shared<CylinderShape>(0.3, 0.8);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<CollisionAspect>(sphere);
  body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  EXPECT_EQ(body->getNumShapeNodes(), 3u);

  // Test getShapeNode by index
  EXPECT_EQ(body->getShapeNode(0)->getShape().get(), box.get());
  EXPECT_EQ(body->getShapeNode(1)->getShape().get(), sphere.get());
  EXPECT_EQ(body->getShapeNode(2)->getShape().get(), cylinder.get());

  // Test getNumShapeNodesWith
  EXPECT_EQ(body->getNumShapeNodesWith<VisualAspect>(), 2u);
  EXPECT_EQ(body->getNumShapeNodesWith<CollisionAspect>(), 2u);
}

TEST(BodyNodeProperties, EachShapeNodeIteration)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<VisualAspect>(sphere);

  // Test eachShapeNode with void callback
  std::size_t count = 0;
  body->eachShapeNode([&count](const ShapeNode*) { ++count; });
  EXPECT_EQ(count, 2u);

  // Test eachShapeNodeWith for specific aspect
  count = 0;
  body->eachShapeNodeWith<VisualAspect>(
      [&count](const ShapeNode*) { ++count; });
  EXPECT_EQ(count, 2u);

  // Test early termination with bool callback
  count = 0;
  body->eachShapeNode([&count](const ShapeNode*) -> bool {
    ++count;
    return false; // Stop after first
  });
  EXPECT_EQ(count, 1u);
}

TEST(BodyNodeProperties, RemoveAllShapeNodesWith)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto cylinder = std::make_shared<CylinderShape>(0.3, 0.8);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<CollisionAspect>(sphere);
  body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  EXPECT_EQ(body->getNumShapeNodes(), 3u);
  EXPECT_EQ(body->getNumShapeNodesWith<VisualAspect>(), 2u);
  EXPECT_EQ(body->getNumShapeNodesWith<CollisionAspect>(), 2u);

  body->removeAllShapeNodesWith<VisualAspect>();

  EXPECT_EQ(body->getNumShapeNodesWith<VisualAspect>(), 0u);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_EQ(body->getNumShapeNodesWith<CollisionAspect>(), 1u);
}

TEST(BodyNodeProperties, EndEffectorManagement)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->getNumEndEffectors(), 0u);

  EndEffector* ee1 = body->createEndEffector("ee1");
  ASSERT_NE(ee1, nullptr);
  EXPECT_EQ(body->getNumEndEffectors(), 1u);
  EXPECT_EQ(ee1->getName(), "ee1");

  EndEffector* ee2 = body->createEndEffector("ee2");
  ASSERT_NE(ee2, nullptr);
  EXPECT_EQ(body->getNumEndEffectors(), 2u);

  // Test getEndEffector by index
  EXPECT_EQ(body->getEndEffector(0), ee1);
  EXPECT_EQ(body->getEndEffector(1), ee2);

  // Test iteration
  std::size_t count = 0;
  body->eachEndEffector([&count](const EndEffector*) { ++count; });
  EXPECT_EQ(count, 2u);
}

TEST(BodyNodeProperties, MarkerManagement)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->getNumMarkers(), 0u);

  Marker* m1 = body->createMarker(std::string("marker1"));
  ASSERT_NE(m1, nullptr);
  EXPECT_EQ(body->getNumMarkers(), 1u);
  EXPECT_EQ(m1->getName(), "marker1");

  Marker* m2 = body->createMarker(
      "marker2", Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector4d(1, 0, 0, 1));
  ASSERT_NE(m2, nullptr);
  EXPECT_EQ(body->getNumMarkers(), 2u);

  // Test getMarker by index
  EXPECT_EQ(body->getMarker(0), m1);
  EXPECT_EQ(body->getMarker(1), m2);

  // Test iteration
  std::size_t count = 0;
  body->eachMarker([&count](const Marker*) { ++count; });
  EXPECT_EQ(count, 2u);
}
