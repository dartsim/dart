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

#include "helpers/gtest_utils.hpp"

#include "dart/config.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/planar_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/io/read.hpp"
#include "dart/simulation/world.hpp"
#include "dart/utils/urdf/urdf_parser.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace dart;
using namespace dart::test;
using dart::common::Uri;
using dart::utils::UrdfParser;

namespace {

std::filesystem::path makeTempDir(const std::string& tag)
{
  const auto stamp
      = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto dir = std::filesystem::temp_directory_path()
                   / (tag + "-" + std::to_string(stamp));
  std::filesystem::create_directories(dir);
  return dir;
}

} // namespace

//==============================================================================
TEST(UrdfParser, parseSkeleton_NonExistantPathReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/skel/test/does_not_exist.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_InvalidUrdfReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr, parser.parseSkeleton("dart://sample/urdf/test/invalid.urdf)"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_MissingMeshReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/urdf/test/missing_mesh.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_InvalidMeshReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/urdf/test/invalid_mesh.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_MissingPackageReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/urdf/test/missing_package.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_LoadsPrimitiveGeometry)
{
  UrdfParser parser;
  EXPECT_TRUE(
      nullptr
      != parser.parseSkeleton(
          "dart://sample/urdf/test/primitive_geometry.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseWorld)
{
  UrdfParser parser;
  EXPECT_TRUE(
      nullptr != parser.parseWorld("dart://sample/urdf/test/testWorld.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseWorld_ResolvesIncludedModel)
{
  UrdfParser parser;

  const auto tempDir = makeTempDir("dart-urdf-world");
  const auto modelPath = tempDir / "model.urdf";

  std::ofstream modelFile(modelPath);
  ASSERT_TRUE(modelFile.is_open());
  modelFile << "<robot name=\"model\"><link name=\"link\"/></robot>";
  modelFile.close();

  const std::string worldXml = R"(
<world name="world">
  <include filename="model.urdf" model_name="model1"/>
  <entity name="entity1" model="model1"/>
</world>
)";

  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  EXPECT_TRUE(parser.parseWorldString(worldXml, baseUri) != nullptr);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, parseWorld_MissingIncludeReturnsNull)
{
  UrdfParser parser;

  const auto tempDir = makeTempDir("dart-urdf-world-missing");

  const std::string worldXml = R"(
<world name="world">
  <include filename="model.urdf" model_name="model1"/>
  <entity name="entity1" model="missing"/>
</world>
)";

  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, parseJointProperties)
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

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::inf);
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::inf);
  EXPECT_TRUE(joint2->isCyclic(0));
}

//==============================================================================
TEST(UrdfParser, parsePlanarJointLimitsAndAxis)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="planar_example">
      <link name="base"/>
      <link name="tip"/>
      <joint name="planar_joint" type="planar">
        <parent link="base"/>
        <child link="tip"/>
        <limit lower="0.1" upper="1.1" velocity="2.5" effort="3.5"/>
        <axis xyz="0 0 1"/>
        <dynamics damping="0.3" friction="0.7"/>
      </joint>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(robot->getJoint("planar_joint"));
  ASSERT_TRUE(joint);

  EXPECT_EQ(joint->getPlaneType(), dynamics::PlanarJoint::PlaneType::XY);
  EXPECT_TRUE(
      joint->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      joint->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(joint->getRotationalAxis().isApprox(Eigen::Vector3d::UnitZ()));

  const Eigen::VectorXd lower
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 0.1);
  const Eigen::VectorXd upper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 1.1);
  const Eigen::VectorXd velocityUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 2.5);
  const Eigen::VectorXd effortUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 3.5);

  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(lower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(upper));
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(-velocityUpper));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velocityUpper));
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(-effortUpper));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(effortUpper));
  EXPECT_TRUE(joint->getRestPositions().isApprox(
      Eigen::VectorXd::Constant(joint->getNumDofs(), 0.6)));
  EXPECT_TRUE(joint->getDampingCoefficients().isConstant(0.3));
  EXPECT_TRUE(joint->getFrictions().isConstant(0.7));
}

//==============================================================================
TEST(UrdfParser, parsePlanarJointArbitraryAxis)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="planar_example">
      <link name="base"/>
      <link name="tip"/>
      <joint name="planar_joint" type="planar">
        <parent link="base"/>
        <child link="tip"/>
        <axis xyz="1 1 1"/>
      </joint>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(robot->getJoint("planar_joint"));
  ASSERT_TRUE(joint);

  const Eigen::Vector3d axis = Eigen::Vector3d::Ones().normalized();
  EXPECT_TRUE(joint->getRotationalAxis().isApprox(axis));
  EXPECT_NEAR(joint->getTranslationalAxis1().dot(axis), 0.0, 1e-12);
  EXPECT_NEAR(joint->getTranslationalAxis2().dot(axis), 0.0, 1e-12);
  EXPECT_NEAR(
      joint->getTranslationalAxis1().dot(joint->getTranslationalAxis2()),
      0.0,
      1e-12);
  EXPECT_NEAR(joint->getTranslationalAxis1().norm(), 1.0, 1e-12);
  EXPECT_NEAR(joint->getTranslationalAxis2().norm(), 1.0, 1e-12);
}

//==============================================================================
TEST(UrdfParser, parseFloatingJointLimits)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="floating_example">
      <link name="base"/>
      <link name="child"/>
      <joint name="floating_joint" type="floating">
        <parent link="base"/>
        <child link="child"/>
        <limit lower="0.1" upper="0.2" velocity="1.1" effort="2.2"/>
        <dynamics damping="0.4" friction="0.5"/>
      </joint>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::FreeJoint*>(robot->getJoint("floating_joint"));
  ASSERT_TRUE(joint);

  const Eigen::VectorXd lower
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 0.1);
  const Eigen::VectorXd upper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 0.2);
  const Eigen::VectorXd velocityUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 1.1);
  const Eigen::VectorXd effortUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 2.2);

  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(lower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(upper));
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(-velocityUpper));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velocityUpper));
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(-effortUpper));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(effortUpper));
  EXPECT_TRUE(joint->getRestPositions().isConstant(0.15));
  EXPECT_TRUE(joint->getDampingCoefficients().isConstant(0.4));
  EXPECT_TRUE(joint->getFrictions().isConstant(0.5));
}

//==============================================================================
TEST(UrdfParser, parseUrdfWithoutWorldLink)
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

  UrdfParser parser;
  UrdfParser::Options options;

  // Default
  auto robot1 = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot1);
  EXPECT_EQ(
      robot1->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_EQ(robot1->getRootJoint()->getName(), "rootJoint");

  // Floating
  options.mDefaultRootJointType = UrdfParser::RootJointType::Floating;
  parser.setOptions(options);
  auto robot2 = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot2);
  EXPECT_EQ(
      robot2->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_EQ(robot2->getRootJoint()->getName(), "rootJoint");

  // Fixed
  options.mDefaultRootJointType = UrdfParser::RootJointType::Fixed;
  parser.setOptions(options);
  auto robot3 = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot3);
  EXPECT_EQ(
      robot3->getRootJoint()->getType(), dynamics::WeldJoint::getStaticType());
  EXPECT_EQ(robot3->getRootJoint()->getName(), "rootJoint");
}

//==============================================================================
TEST(UrdfParser, mimicJoint)
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

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::inf);
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::inf);
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() == dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr != joint2->getMimicJoint());
  EXPECT_DOUBLE_EQ(joint2->getMimicMultiplier(), 2.);
  EXPECT_DOUBLE_EQ(joint2->getMimicOffset(), 0.1);
}

//==============================================================================
TEST(UrdfParser, transmissionsCreateCoupledMimicJoints)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="transmission_robot">
      <link name="base"/>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1"/>
      <joint name="j2" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l2"/>

      <transmission name="tx1">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
      <transmission name="tx2">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>4.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";
  // clang-format on

  UrdfParser loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* refJoint = robot->getJoint("j1");
  auto* follower = robot->getJoint("j2");
  ASSERT_NE(refJoint, nullptr);
  ASSERT_NE(follower, nullptr);

  EXPECT_NE(refJoint->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_EQ(follower->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_EQ(follower->getMimicJoint(), refJoint);
  EXPECT_TRUE(follower->isUsingCouplerConstraint());
  EXPECT_DOUBLE_EQ(follower->getMimicMultiplier(), 0.5);
  EXPECT_DOUBLE_EQ(follower->getMimicOffset(), 0.0);
}

//==============================================================================
TEST(UrdfParser, transmissionDynamicsRespectsGearRatio)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="transmission_robot_dynamic">
      <link name="base">
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1">
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>
      <joint name="j2" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l2">
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>

      <transmission name="tx1">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
      <transmission name="tx2">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>4.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";
  // clang-format on

  dart::utils::UrdfParser::Options options;
  options.mDefaultRootJointType = dart::utils::UrdfParser::RootJointType::Fixed;
  UrdfParser loader(options);
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  world->addSkeleton(robot);

  auto* j1 = robot->getJoint("j1");
  auto* j2 = robot->getJoint("j2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);

  ASSERT_TRUE(j2->isUsingCouplerConstraint());
  ASSERT_EQ(j2->getActuatorType(), dart::dynamics::Joint::MIMIC);
  const double multiplier = j2->getMimicMultiplier();
  ASSERT_DOUBLE_EQ(multiplier, 0.5);

  const double tol = 5e-3;
  const double torque = 1.0;
  for (int i = 0; i < 200; ++i) {
    j1->setForce(0, torque);
    j2->setForce(0, 0.0);
    world->step();

    EXPECT_NEAR(j2->getPosition(0), multiplier * j1->getPosition(0), tol);
    EXPECT_NEAR(j2->getVelocity(0), multiplier * j1->getVelocity(0), tol);
  }
}

//==============================================================================
TEST(UrdfParser, transmissionsSkipInvalidEntries)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="transmission_robot_invalid">
      <link name="base"/>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1"/>
      <joint name="j2" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l2"/>

      <!-- Unsupported type should be ignored -->
      <transmission name="tx_bad_type">
        <type>transmission_interface/FourBarLinkageTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>

      <!-- Zero ratio should be ignored -->
      <transmission name="tx_zero_ratio">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>0.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";
  // clang-format on

  UrdfParser loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* j1 = robot->getJoint("j1");
  auto* j2 = robot->getJoint("j2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);

  // Neither joint should have been converted to a mimic actuator because the
  // only transmissions were invalid.
  EXPECT_NE(j1->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_NE(j2->getActuatorType(), dart::dynamics::Joint::MIMIC);
}

//==============================================================================
TEST(UrdfParser, badMimicJoint)
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

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::inf);
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::inf);
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() != dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr == joint2->getMimicJoint());
}

//==============================================================================
TEST(UrdfParser, WorldShouldBeTreatedAsKeyword)
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

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_TRUE(robot->getNumBodyNodes() == 1u);
  EXPECT_TRUE(robot->getRootBodyNode()->getName() == "link_0");
}

//==============================================================================
TEST(UrdfParser, SingleLinkWithoutJoint)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0" />
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_TRUE(robot->getNumBodyNodes() == 1u);
  EXPECT_TRUE(robot->getRootBodyNode()->getName() == "link_0");
  EXPECT_EQ(
      robot->getRootJoint()->getType(),
      dart::dynamics::FreeJoint::getStaticType());
}

//==============================================================================
TEST(UrdfParser, MultiTreeRobot)
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

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_EQ(robot->getNumBodyNodes(), 2u);
  EXPECT_EQ(robot->getNumTrees(), 2u);
}

//==============================================================================
TEST(UrdfParser, KR5MeshColor)
{
  UrdfParser parser;
  auto robot
      = parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_EQ(robot->getNumBodyNodes(), 7u);
  EXPECT_EQ(robot->getNumTrees(), 1u);

  robot->eachBodyNode([](dynamics::BodyNode* bodyNode) {
    bodyNode->eachShapeNodeWith<dynamics::VisualAspect>(
        [&](dynamics::ShapeNode* shapeNode) {
          auto shape = shapeNode->getShape();
          if (auto mesh = shape->as<dynamics::MeshShape>()) {
            EXPECT_EQ(mesh->getColorMode(), dynamics::MeshShape::SHAPE_COLOR);
          }
        });
  });
}

//==============================================================================
TEST(UrdfParser, parseVisualCollisionName)
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

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto body0 = robot->getBodyNode(0);
  ASSERT_TRUE(nullptr != body0);
  ASSERT_EQ(body0->getNumShapeNodes(), 2);
  EXPECT_EQ(body0->getShapeNode(0)->getName(), "first_visual");
  EXPECT_EQ(body0->getShapeNode(1)->getName(), "second_visual");

  auto body1 = robot->getBodyNode(1);
  ASSERT_TRUE(nullptr != body1);
  ASSERT_EQ(body1->getNumShapeNodes(), 2);
  // The ShapeNode naming pattern is inferred from
  // BodyNode::createShapeNodeWith(const ShapePtr& shape) sot this test could
  // fail if the naming pattern in the function is changed.
  EXPECT_EQ(
      body1->getShapeNode(0)->getName(), body1->getName() + "_ShapeNode_0");
  EXPECT_EQ(
      body1->getShapeNode(1)->getName(), body1->getName() + "_ShapeNode_1");
}

//==============================================================================
TEST(UrdfParser, Options)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
      </link>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  UrdfParser::Options options;

  // Default inertia
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);
  auto link_0 = robot->getBodyNode("link_0");
  ASSERT_TRUE(link_0 != nullptr);
  EXPECT_TRUE(equals(
      link_0->getInertia().getSpatialTensor(),
      dynamics::Inertia().getSpatialTensor()));

  // Custom inertia
  options.mDefaultInertia.setMass(5);
  options.mDefaultInertia.setMoment(1, 2, 3, 4, 5, 6);
  parser.setOptions(options);
  robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);
  link_0 = robot->getBodyNode("link_0");
  ASSERT_TRUE(link_0 != nullptr);
  EXPECT_TRUE(equals(
      link_0->getInertia().getSpatialTensor(),
      options.mDefaultInertia.getSpatialTensor()));
}

#if DART_IO_HAS_URDF
namespace {

template <typename ShapeType>
bool hasShapeType(const dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return false;
  }

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    const auto* body = skeleton->getBodyNode(i);
    if (!body) {
      continue;
    }
    for (std::size_t j = 0; j < body->getNumShapeNodes(); ++j) {
      const auto* shapeNode = body->getShapeNode(j);
      if (!shapeNode) {
        continue;
      }
      const auto shape = shapeNode->getShape();
      if (shape && shape->is<ShapeType>()) {
        return true;
      }
    }
  }

  return false;
}

} // namespace

//==============================================================================
TEST(UrdfParser, WamMeshMaterialAndLimits)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  options.addPackageDirectory("herb_description", config::dataPath("urdf/wam"));

  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/wam/wam.urdf", options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_TRUE(hasShapeType<dynamics::MeshShape>(skeleton));

  const auto joint = skeleton->getJoint("/j1");
  ASSERT_NE(joint, nullptr);
  const auto* revolute = dynamic_cast<dynamics::RevoluteJoint*>(joint);
  ASSERT_NE(revolute, nullptr);
  EXPECT_NEAR(revolute->getPositionLowerLimit(0), -2.6, 1e-6);
  EXPECT_NEAR(revolute->getPositionUpperLimit(0), 2.6, 1e-6);

  const Eigen::Vector4d expectedColor(
      0.792156862745098, 0.819607843137255, 0.933333333333333, 1.0);
  bool foundVisual = false;
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (!body) {
      continue;
    }
    body->eachShapeNodeWith<dynamics::VisualAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (shapeNode->getVisualAspect()->getRGBA().isApprox(expectedColor)) {
            foundVisual = true;
          }
        });
  }
  EXPECT_TRUE(foundVisual);
}

//==============================================================================
TEST(UrdfParser, DrchuboStructure)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  options.addPackageDirectory("drchubo", config::dataPath("urdf/drchubo"));

  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/drchubo/drchubo.urdf", options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_GT(skeleton->getNumBodyNodes(), 20u);
  EXPECT_GT(skeleton->getNumDofs(), 20u);
  EXPECT_TRUE(hasShapeType<dynamics::MeshShape>(skeleton));
}
#endif

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringEmptyReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(parser.parseSkeletonString("", ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringEmptyReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(parser.parseWorldString("", ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringMissingWorldElementReturnsNull)
{
  UrdfParser parser;
  const auto tempDir = makeTempDir("dart-urdf-world-missing-world");
  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  const std::string worldXml = R"(<robot name="not_a_world"/>)";
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);
  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringMissingNameReturnsNull)
{
  UrdfParser parser;
  const auto tempDir = makeTempDir("dart-urdf-world-missing-name");
  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  const std::string worldXml = R"(<world></world>)";
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);
  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringInvalidXmlReturnsNull)
{
  UrdfParser parser;
  const auto tempDir = makeTempDir("dart-urdf-world-invalid-xml");
  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  const std::string worldXml = R"(<world name="world">)";
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);
  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParsePlanarJointAxisXMapsToYZ)
{
  const std::string urdfStr = R"(
    <robot name="planar_example">
      <link name="base"/>
      <link name="tip"/>
      <joint name="planar_joint" type="planar">
        <parent link="base"/>
        <child link="tip"/>
        <axis xyz="1 0 0"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(robot->getJoint("planar_joint"));
  ASSERT_TRUE(joint);
  EXPECT_EQ(joint->getPlaneType(), dynamics::PlanarJoint::PlaneType::YZ);
  EXPECT_TRUE(joint->getRotationalAxis().isApprox(Eigen::Vector3d::UnitX()));
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringPrismaticJointType)
{
  const std::string urdfStr = R"(
    <robot name="prismatic_example">
      <link name="base"/>
      <link name="slider"/>
      <joint name="slide_joint" type="prismatic">
        <parent link="base"/>
        <child link="slider"/>
        <axis xyz="0 1 0"/>
        <limit lower="-0.5" upper="0.5" effort="1.0" velocity="2.0"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint = robot->getJoint("slide_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), "PrismaticJoint");
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringFixedJointType)
{
  const std::string urdfStr = R"(
    <robot name="fixed_example">
      <link name="base"/>
      <link name="fixed_link"/>
      <joint name="fixed_joint" type="fixed">
        <parent link="base"/>
        <child link="fixed_link"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint = robot->getJoint("fixed_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), "WeldJoint");
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringSetsFixedRootTransform)
{
  UrdfParser::Options options;
  options.mDefaultRootJointType = UrdfParser::RootJointType::Fixed;
  UrdfParser parser(options);

  const auto tempDir = makeTempDir("dart-urdf-world-fixed");
  const auto modelPath = tempDir / "model.urdf";

  std::ofstream modelFile(modelPath);
  ASSERT_TRUE(modelFile.is_open());
  modelFile << "<robot name=\"model\"><link name=\"base\"/></robot>";
  modelFile.close();

  const std::string worldXml = R"(
<world name="world">
  <include filename="model.urdf" model_name="model1"/>
  <entity name="entity1" model="model1">
    <origin xyz="1 2 3" rpy="0 0 1.57079632679"/>
  </entity>
</world>
)";

  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  auto world = parser.parseWorldString(worldXml, baseUri);
  ASSERT_TRUE(world != nullptr);

  const auto skeleton = world->getSkeleton("entity1");
  ASSERT_TRUE(skeleton != nullptr);
  auto* joint = skeleton->getRootJoint();
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), "WeldJoint");
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));

  std::filesystem::remove_all(tempDir);
}
