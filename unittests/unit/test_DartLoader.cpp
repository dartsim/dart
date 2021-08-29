/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <iostream>

#include <gtest/gtest.h>

#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/utils/urdf/DartLoader.hpp"

using namespace dart;
using dart::common::Uri;
using dart::utils::DartLoader;

//==============================================================================
TEST(DartLoader, parseSkeleton_NonExistantPathReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(
      nullptr,
      loader.parseSkeleton("dart://sample/skel/test/does_not_exist.urdf"));
}

//==============================================================================
TEST(DartLoader, parseSkeleton_InvalidUrdfReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(
      nullptr, loader.parseSkeleton("dart://sample/urdf/test/invalid.urdf)"));
}

//==============================================================================
TEST(DartLoader, parseSkeleton_MissingMeshReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(
      nullptr,
      loader.parseSkeleton("dart://sample/urdf/test/missing_mesh.urdf"));
}

//==============================================================================
TEST(DartLoader, parseSkeleton_InvalidMeshReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(
      nullptr,
      loader.parseSkeleton("dart://sample/urdf/test/invalid_mesh.urdf"));
}

//==============================================================================
TEST(DartLoader, parseSkeleton_MissingPackageReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(
      nullptr,
      loader.parseSkeleton("dart://sample/urdf/test/missing_package.urdf"));
}

//==============================================================================
TEST(DartLoader, parseSkeleton_LoadsPrimitiveGeometry)
{
  DartLoader loader;
  EXPECT_TRUE(
      nullptr
      != loader.parseSkeleton(
          "dart://sample/urdf/test/primitive_geometry.urdf"));
}

//==============================================================================
TEST(DartLoader, parseWorld)
{
  DartLoader loader;
  EXPECT_TRUE(
      nullptr != loader.parseWorld("dart://sample/urdf/test/testWorld.urdf"));
}

//==============================================================================
TEST(DartLoader, parseJointProperties)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="1_to_2" type="continuous">
        <parent link="link_1" />
        <child link="link_2" />
        <limit effort="2.5" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_2">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(
      joint2->getPositionLowerLimit(0), -dart::math::constantsd::inf());
  EXPECT_DOUBLE_EQ(
      joint2->getPositionUpperLimit(0), dart::math::constantsd::inf());
  EXPECT_TRUE(joint2->isCyclic(0));
}

//==============================================================================
TEST(DartLoader, parseUrdfWithoutWorldLink)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0" />
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
      </joint>
      <link name="link_1" />
    </robot>
  )";
  // clang-format on

  DartLoader loader;

  auto robot1 = loader.parseSkeletonString(urdfStr, "", nullptr);
  ASSERT_TRUE(nullptr != robot1);
  EXPECT_EQ(
      robot1->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_EQ(robot1->getRootJoint()->getName(), "rootJoint");

  auto robot2 = loader.parseSkeletonString(
      urdfStr, "", nullptr, DartLoader::Flags::NONE);
  ASSERT_TRUE(nullptr != robot2);
  EXPECT_EQ(
      robot2->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_EQ(robot2->getRootJoint()->getName(), "rootJoint");

  auto robot3 = loader.parseSkeletonString(
      urdfStr, "", nullptr, DartLoader::Flags::FIXED_BASE_LINK);
  ASSERT_TRUE(nullptr != robot3);
  EXPECT_EQ(
      robot3->getRootJoint()->getType(), dynamics::WeldJoint::getStaticType());
  EXPECT_EQ(robot3->getRootJoint()->getName(), "rootJoint");
}

//==============================================================================
TEST(DartLoader, mimicJoint)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="1_to_2" type="continuous">
        <parent link="link_1" />
        <child link="link_2" />
        <limit effort="2.5" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
        <mimic joint="0_to_1" multiplier="2." offset="0.1" />
      </joint>
      <link name="link_2">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(
      joint2->getPositionLowerLimit(0), -dart::math::constantsd::inf());
  EXPECT_DOUBLE_EQ(
      joint2->getPositionUpperLimit(0), dart::math::constantsd::inf());
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() == dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr != joint2->getMimicJoint());
  EXPECT_DOUBLE_EQ(joint2->getMimicMultiplier(), 2.);
  EXPECT_DOUBLE_EQ(joint2->getMimicOffset(), 0.1);
}

//==============================================================================
TEST(DartLoader, badMimicJoint)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="1_to_2" type="continuous">
        <parent link="link_1" />
        <child link="link_2" />
        <limit effort="2.5" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
        <mimic joint="mjoint" multiplier="2." offset="0.1" />
      </joint>
      <link name="link_2">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(
      joint2->getPositionLowerLimit(0), -dart::math::constantsd::inf());
  EXPECT_DOUBLE_EQ(
      joint2->getPositionUpperLimit(0), dart::math::constantsd::inf());
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() != dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr == joint2->getMimicJoint());
}

//==============================================================================
TEST(DartLoader, WorldShouldBeTreatedAsKeyword)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="world" />
      <joint name="world_to_1" type="revolute">
        <parent link="world" />
        <child link="link_0" />
        <axis xyz="0 0 1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_0" />
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_TRUE(robot->getNumBodyNodes() == 1u);
  EXPECT_TRUE(robot->getRootBodyNode()->getName() == "link_0");
}

//==============================================================================
TEST(DartLoader, SingleLinkWithoutJoint)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0" />
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_TRUE(robot->getNumBodyNodes() == 1u);
  EXPECT_TRUE(robot->getRootBodyNode()->getName() == "link_0");
  EXPECT_EQ(
      robot->getRootJoint()->getType(),
      dart::dynamics::FreeJoint::getStaticType());
}

//==============================================================================
TEST(DartLoader, MultiTreeRobot)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="world" />
      <joint name="world_to_0" type="revolute">
        <parent link="world" />
        <child link="link_0" />
        <axis xyz="0 0 1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_0" />
      <joint name="world_to_1" type="revolute">
        <parent link="world" />
        <child link="link_1" />
        <axis xyz="0 0 1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1" />
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_EQ(robot->getNumBodyNodes(), 2u);
  EXPECT_EQ(robot->getNumTrees(), 2u);
}

//==============================================================================
TEST(DartLoader, KR5MeshColor)
{
  DartLoader loader;
  auto robot
      = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_EQ(robot->getNumBodyNodes(), 7u);
  EXPECT_EQ(robot->getNumTrees(), 1u);

  for (auto i = 0u; i < robot->getNumBodyNodes(); ++i)
  {
    auto body = robot->getBodyNode(i);
    for (auto shapeNode : body->getShapeNodesWith<dynamics::VisualAspect>())
    {
      auto shape = shapeNode->getShape();
      if (auto mesh = std::dynamic_pointer_cast<dynamics::MeshShape>(shape))
      {
        EXPECT_EQ(mesh->getColorMode(), dynamics::MeshShape::SHAPE_COLOR);
      }
    }
  }
}

//==============================================================================
TEST(DartLoader, parseVisualCollisionName)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual name="first_visual">
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual name="second_visual">
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto body0 = robot->getBodyNode(0);
  ASSERT_TRUE(nullptr != body0);
  auto shape_nodes0 = body0->getShapeNodes();
  ASSERT_EQ(shape_nodes0.size(), 2);
  EXPECT_EQ(shape_nodes0[0]->getName(), "first_visual");
  EXPECT_EQ(shape_nodes0[1]->getName(), "second_visual");

  auto body1 = robot->getBodyNode(1);
  ASSERT_TRUE(nullptr != body1);
  auto shape_nodes1 = body1->getShapeNodes();
  ASSERT_EQ(shape_nodes1.size(), 2);
  // The ShapeNode naming pattern is infered from
  // BodyNode::createShapeNodeWith(const ShapePtr& shape) sot this test could
  // fail if the naming pattern in the function is changed.
  EXPECT_EQ(shape_nodes1[0]->getName(), body1->getName() + "_ShapeNode_0");
  EXPECT_EQ(shape_nodes1[1]->getName(), body1->getName() + "_ShapeNode_1");
}
