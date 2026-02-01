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

#include "dart/utils/All.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>

#include <cmath>

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;
using namespace dart::test;

//==============================================================================
TEST(SkelParser, DataStructure)
{
  bool valBool = true;
  int valInt = -3;
  unsigned int valUInt = 1;
  float valFloat = -3.140f;
  double valDouble = 1.4576640;
  char valChar = 'd';
  Eigen::Vector2d valVector2d = Eigen::Vector2d::Random();
  Eigen::Vector3d valVector3d = Eigen::Vector3d::Random();
  Eigen::Vector3i valVector3i = Eigen::Vector3i::Random();
  Eigen::Vector6d valVector6d = Eigen::Vector6d::Random();
  Eigen::VectorXd valVectorXd = Eigen::VectorXd::Random(10);
  Eigen::Isometry3d valIsometry3d = Eigen::Isometry3d::Identity();

  std::string strBool = toString(valBool);
  std::string strInt = toString(valInt);
  std::string strUInt = toString(valUInt);
  std::string strFloat = toString(valFloat);
  std::string strDouble = toString(valDouble);
  std::string strChar = toString(valChar);
  std::string strVector2d = toString(valVector2d);
  std::string strVector3d = toString(valVector3d);
  std::string strVector3i = toString(valVector3i);
  std::string strVector6d = toString(valVector6d);
  std::string strVectorXd = toString(valVectorXd);
  std::string strIsometry3d = toString(valIsometry3d);

  EXPECT_EQ(valBool, toBool(strBool));
  EXPECT_EQ(valInt, toInt(strInt));
  EXPECT_EQ(valUInt, toUInt(strUInt));
  EXPECT_EQ(valFloat, toFloat(strFloat));
  EXPECT_EQ(valDouble, toDouble(strDouble));
  EXPECT_EQ(valChar, toChar(strChar));
  EXPECT_VECTOR_DOUBLE_EQ(valVector2d, toVector2d(strVector2d));
  EXPECT_VECTOR_DOUBLE_EQ(valVector3d, toVector3d(strVector3d));
  EXPECT_VECTOR_DOUBLE_EQ(valVector3i, toVector3i(strVector3i));
  EXPECT_VECTOR_DOUBLE_EQ(valVector6d, toVector6d(strVector6d));
  EXPECT_VECTOR_DOUBLE_EQ(valVectorXd, toVectorXd(strVectorXd));
  EXPECT_TRANSFORM_DOUBLE_EQ(valIsometry3d, toIsometry3d(strIsometry3d));
}

//==============================================================================
TEST(SkelParser, WorldParentNameClearsWhenMissingBody)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world 1">
    <skeleton name="skel">
      <body name="link"/>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  WorldPtr world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);

  SkeletonPtr skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  ASSERT_NE(skel->getRootBodyNode(), nullptr);
  EXPECT_EQ(skel->getRootBodyNode()->getName(), "link");
}

//==============================================================================
TEST(SkelParser, EmptyWorld)
{
  WorldPtr world = SkelParser::readWorld("dart://sample/skel/test/empty.skel");

  EXPECT_TRUE(world != nullptr);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), 0);
  EXPECT_EQ(world->getGravity()(2), -9.81);
  EXPECT_EQ(world->getNumSkeletons(), 0);

  EXPECT_EQ(world->getTime(), 0);
  world->step();
  EXPECT_EQ(world->getTime(), world->getTimeStep());
}

//==============================================================================
TEST(SkelParser, SinglePendulum)
{
  WorldPtr world
      = SkelParser::readWorld("dart://sample/skel/test/single_pendulum.skel");

  EXPECT_TRUE(world != nullptr);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), -9.81);
  EXPECT_EQ(world->getGravity()(2), 0);
  EXPECT_EQ(world->getNumSkeletons(), 1);

  SkeletonPtr skel1 = world->getSkeleton("single_pendulum");

  EXPECT_EQ(skel1->getNumBodyNodes(), 1);

  world->step();
}

//==============================================================================
TEST(SkelParser, SerialChain)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/serial_chain_ball_joint.skel");

  EXPECT_TRUE(world != nullptr);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), -9.81);
  EXPECT_EQ(world->getGravity()(2), 0);
  EXPECT_EQ(world->getNumSkeletons(), 1);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");

  EXPECT_EQ(skel1->getNumBodyNodes(), 10);

  world->step();
}

//==============================================================================
TEST(SkelParser, VariousShapes)
{
  // Check if the parser can load various shapes without any problems and the
  // world can simulate it successfully.

  WorldPtr world
      = SkelParser::readWorld("dart://sample/skel/test/test_shapes.skel");

  for (auto i = 0u; i < 100; ++i) {
    world->step();
  }
}

//==============================================================================
TEST(SkelParser, RigidAndSoftBodies)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/test_articulated_bodies.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel1 != nullptr);
  EXPECT_EQ(skel1->getNumBodyNodes(), 2);
  EXPECT_EQ(skel1->getNumRigidBodyNodes(), 1);
  EXPECT_EQ(skel1->getNumSoftBodyNodes(), 1);

  SoftBodyNode* sbn = skel1->getSoftBodyNode(0);
  EXPECT_TRUE(sbn->getNumPointMasses() > 0);

  world->step();
}

//==============================================================================
TEST(SkelParser, InertiaFromShapeNodes)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/inertia_from_shapes.skel");
  ASSERT_NE(world, nullptr);

  SkeletonPtr noShapeSkel = world->getSkeleton("no_shape_skel");
  ASSERT_NE(noShapeSkel, nullptr);
  BodyNode* noShape = noShapeSkel->getBodyNode("no_shape");
  ASSERT_NE(noShape, nullptr);
  EXPECT_EQ(0u, noShape->getNumShapeNodes());
  const auto emptyAggregate = noShape->computeInertiaFromShapeNodes(
      [](const ShapeNode*) -> std::optional<double> { return 1.0; });
  EXPECT_FALSE(emptyAggregate.has_value());

  SkeletonPtr zeroMassSkel = world->getSkeleton("zero_mass_skel");
  ASSERT_NE(zeroMassSkel, nullptr);
  BodyNode* zeroMass = zeroMassSkel->getBodyNode("zero_mass");
  ASSERT_NE(zeroMass, nullptr);
  EXPECT_GT(zeroMass->getNumShapeNodes(), 0u);
  EXPECT_DOUBLE_EQ(0.0, zeroMass->getMass());
  const auto zeroMassAggregate = zeroMass->computeInertiaFromShapeNodes(
      [](const ShapeNode* shapeNode) -> std::optional<double> {
        return shapeNode->getShape()->getVolume();
      });
  ASSERT_TRUE(zeroMassAggregate.has_value());
  EXPECT_GT(zeroMassAggregate->getMass(), 0.0);

  SkeletonPtr weightedSkel = world->getSkeleton("weighted_skel");
  ASSERT_NE(weightedSkel, nullptr);
  BodyNode* weighted = weightedSkel->getBodyNode("weighted");
  ASSERT_NE(weighted, nullptr);
  ASSERT_GE(weighted->getNumShapeNodes(), 2u);

  const Eigen::Vector3d desiredCom(0.03, -0.01, 0.02);
  EXPECT_VECTOR_DOUBLE_EQ(desiredCom, weighted->getInertia().getLocalCOM());

  std::vector<const ShapeNode*> nodes;
  std::vector<double> weights;
  double totalWeight = 0.0;
  for (auto i = 0u; i < weighted->getNumShapeNodes(); ++i) {
    const ShapeNode* node = weighted->getShapeNode(i);
    ASSERT_NE(node, nullptr);
    nodes.push_back(node);
    double weight = node->getShape()->getVolume();
    if (!std::isfinite(weight) || weight <= 0.0) {
      weight = 1.0;
    }
    weights.push_back(weight);
    totalWeight += weight;
  }
  ASSERT_GT(totalWeight, 0.0);

  const double bodyMass = weighted->getMass();
  const auto combined = weighted->computeInertiaFromShapeNodes(
      [&](const ShapeNode* shapeNode) -> std::optional<double> {
        for (std::size_t i = 0; i < nodes.size(); ++i) {
          if (nodes[i] == shapeNode) {
            return bodyMass * (weights[i] / totalWeight);
          }
        }
        return std::nullopt;
      });
  ASSERT_TRUE(combined.has_value());

  Eigen::Matrix3d expectedMoment = combined->getMoment();
  const Eigen::Vector3d delta = combined->getLocalCOM() - desiredCom;
  if (!delta.isZero(1e-12)) {
    expectedMoment = math::parallelAxisTheorem(expectedMoment, delta, bodyMass);
  }

  EXPECT_MATRIX_DOUBLE_EQ(expectedMoment, weighted->getInertia().getMoment());
}

//==============================================================================
TEST(SkelParser, PlanarJoint)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  WorldPtr world
      = SkelParser::readWorld("dart://sample/skel/test/planar_joint.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton1");
  EXPECT_TRUE(skel1 != nullptr);

  BodyNode* body1 = skel1->getBodyNode("link1");
  BodyNode* body2 = skel1->getBodyNode("link2");
  BodyNode* body3 = skel1->getBodyNode("link3");
  BodyNode* body4 = skel1->getBodyNode("link4");
  EXPECT_TRUE(body1 != nullptr);
  EXPECT_TRUE(body2 != nullptr);
  EXPECT_TRUE(body3 != nullptr);
  EXPECT_TRUE(body4 != nullptr);

  PlanarJoint* planarJoint1
      = dynamic_cast<PlanarJoint*>(body1->getParentJoint());
  PlanarJoint* planarJoint2
      = dynamic_cast<PlanarJoint*>(body2->getParentJoint());
  PlanarJoint* planarJoint3
      = dynamic_cast<PlanarJoint*>(body3->getParentJoint());
  PlanarJoint* planarJoint4
      = dynamic_cast<PlanarJoint*>(body4->getParentJoint());
  EXPECT_TRUE(planarJoint1 != nullptr);
  EXPECT_TRUE(planarJoint2 != nullptr);
  EXPECT_TRUE(planarJoint3 != nullptr);
  EXPECT_TRUE(planarJoint4 != nullptr);

  EXPECT_EQ(planarJoint1->getPlaneType(), PlanarJoint::PlaneType::XY);
  EXPECT_EQ(planarJoint2->getPlaneType(), PlanarJoint::PlaneType::YZ);
  EXPECT_EQ(planarJoint3->getPlaneType(), PlanarJoint::PlaneType::ZX);
  EXPECT_EQ(planarJoint4->getPlaneType(), PlanarJoint::PlaneType::ARBITRARY);

  EXPECT_EQ(planarJoint1->getTranslationalAxis1(), Eigen::Vector3d::UnitX());
  EXPECT_EQ(planarJoint2->getTranslationalAxis1(), Eigen::Vector3d::UnitY());
  EXPECT_EQ(planarJoint3->getTranslationalAxis1(), Eigen::Vector3d::UnitZ());
  EXPECT_EQ(planarJoint4->getTranslationalAxis1(), Eigen::Vector3d::UnitX());

  EXPECT_EQ(planarJoint1->getTranslationalAxis2(), Eigen::Vector3d::UnitY());
  EXPECT_EQ(planarJoint2->getTranslationalAxis2(), Eigen::Vector3d::UnitZ());
  EXPECT_EQ(planarJoint3->getTranslationalAxis2(), Eigen::Vector3d::UnitX());
  EXPECT_EQ(planarJoint4->getTranslationalAxis2(), Eigen::Vector3d::UnitY());

  EXPECT_EQ(planarJoint1->getRotationalAxis(), Eigen::Vector3d::UnitZ());
  EXPECT_EQ(planarJoint2->getRotationalAxis(), Eigen::Vector3d::UnitX());
  EXPECT_EQ(planarJoint3->getRotationalAxis(), Eigen::Vector3d::UnitY());
  EXPECT_EQ(planarJoint4->getRotationalAxis(), Eigen::Vector3d::UnitZ());

  EXPECT_EQ(planarJoint1->getPositions(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(planarJoint2->getPositions(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(planarJoint3->getPositions(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(planarJoint4->getPositions(), Eigen::Vector3d(1, 2, 3));

  EXPECT_EQ(planarJoint1->getVelocities(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(planarJoint2->getVelocities(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(planarJoint3->getVelocities(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(planarJoint4->getVelocities(), Eigen::Vector3d(4, 5, 6));

  EXPECT_EQ(planarJoint1->getDampingCoefficient(0), 1);
  EXPECT_EQ(planarJoint2->getDampingCoefficient(0), 1);
  EXPECT_EQ(planarJoint3->getDampingCoefficient(0), 1);
  EXPECT_EQ(planarJoint4->getDampingCoefficient(0), 1);

  EXPECT_EQ(planarJoint1->getDampingCoefficient(1), 2);
  EXPECT_EQ(planarJoint2->getDampingCoefficient(1), 2);
  EXPECT_EQ(planarJoint3->getDampingCoefficient(1), 2);
  EXPECT_EQ(planarJoint4->getDampingCoefficient(1), 2);

  EXPECT_EQ(planarJoint1->getDampingCoefficient(2), 3);
  EXPECT_EQ(planarJoint2->getDampingCoefficient(2), 3);
  EXPECT_EQ(planarJoint3->getDampingCoefficient(2), 3);
  EXPECT_EQ(planarJoint4->getDampingCoefficient(2), 3);

  EXPECT_EQ(planarJoint1->getPositionLowerLimit(0), -1.0);
  EXPECT_EQ(planarJoint2->getPositionLowerLimit(0), -1.0);
  EXPECT_EQ(planarJoint3->getPositionLowerLimit(0), -1.0);
  EXPECT_EQ(planarJoint4->getPositionLowerLimit(0), -1.0);

  EXPECT_EQ(planarJoint1->getPositionUpperLimit(0), +1.0);
  EXPECT_EQ(planarJoint2->getPositionUpperLimit(0), +1.0);
  EXPECT_EQ(planarJoint3->getPositionUpperLimit(0), +1.0);
  EXPECT_EQ(planarJoint4->getPositionUpperLimit(0), +1.0);

  EXPECT_EQ(planarJoint1->getPositionLowerLimit(1), -2.0);
  EXPECT_EQ(planarJoint2->getPositionLowerLimit(1), -2.0);
  EXPECT_EQ(planarJoint3->getPositionLowerLimit(1), -2.0);
  EXPECT_EQ(planarJoint4->getPositionLowerLimit(1), -2.0);

  EXPECT_EQ(planarJoint1->getPositionUpperLimit(1), +2.0);
  EXPECT_EQ(planarJoint2->getPositionUpperLimit(1), +2.0);
  EXPECT_EQ(planarJoint3->getPositionUpperLimit(1), +2.0);
  EXPECT_EQ(planarJoint4->getPositionUpperLimit(1), +2.0);

  EXPECT_EQ(planarJoint1->getPositionLowerLimit(2), -3.0);
  EXPECT_EQ(planarJoint2->getPositionLowerLimit(2), -3.0);
  EXPECT_EQ(planarJoint3->getPositionLowerLimit(2), -3.0);
  EXPECT_EQ(planarJoint4->getPositionLowerLimit(2), -3.0);

  EXPECT_EQ(planarJoint1->getPositionUpperLimit(2), +3.0);
  EXPECT_EQ(planarJoint2->getPositionUpperLimit(2), +3.0);
  EXPECT_EQ(planarJoint3->getPositionUpperLimit(2), +3.0);
  EXPECT_EQ(planarJoint4->getPositionUpperLimit(2), +3.0);

  world->step();
}

//==============================================================================
TEST(SkelParser, JointActuatorType)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/joint_actuator_type_test.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel1 != nullptr);

  // Test for no actuator type attribute being specified
  Joint* joint0 = skel1->getJoint("joint0");
  EXPECT_EQ(joint0->getActuatorType(), Joint::DefaultActuatorType);
  EXPECT_EQ(joint0->getActuatorType(), Joint::FORCE);

  // Test for when actuator type attribute are specified
  Joint* joint1 = skel1->getJoint("joint1");
  EXPECT_EQ(joint1->getActuatorType(), Joint::FORCE);

  // Test for only a dof name being changed
  Joint* joint2 = skel1->getJoint("joint2");
  EXPECT_EQ(joint2->getActuatorType(), Joint::PASSIVE);
  joint2->setActuatorType(Joint::FORCE);
  EXPECT_EQ(joint2->getActuatorType(), Joint::FORCE);

  // Test for when actuator type attribute are specified
  Joint* joint3 = skel1->getJoint("joint3");
  EXPECT_EQ(joint3->getActuatorType(), Joint::ACCELERATION);

  // Test for when actuator type attribute are specified
  Joint* joint4 = skel1->getJoint("joint4");
  EXPECT_EQ(joint4->getActuatorType(), Joint::VELOCITY);
}

//==============================================================================
TEST(SkelParser, DofAttributes)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/dof_attribute_test.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");

  // Test for no dof elements being specified
  Joint* joint0 = skel1->getJoint("joint0");
  EXPECT_EQ(joint0->getDof(0)->getPositionLowerLimit(), -inf);
  EXPECT_EQ(joint0->getDof(0)->getPositionUpperLimit(), inf);
  EXPECT_EQ(joint0->getDof(0)->getPosition(), 0);

  EXPECT_EQ(joint0->getDof(0)->getVelocityLowerLimit(), -inf);
  EXPECT_EQ(joint0->getDof(0)->getVelocityUpperLimit(), inf);
  EXPECT_EQ(joint0->getDof(0)->getVelocity(), 0);

  EXPECT_EQ(joint0->getDof(0)->getName(), joint0->getName());

  // Test for only a dof name being changed
  Joint* joint1 = skel1->getJoint("joint1");
  EXPECT_EQ(joint1->getDof(0)->getPositionLowerLimit(), -1.57);
  EXPECT_EQ(joint1->getDof(0)->getPositionUpperLimit(), 1.57);
  EXPECT_EQ(joint1->getDof(0)->getName(), "customJoint1");

  // Test for when dof attributes (but not a name) are specified
  Joint* joint2 = skel1->getJoint("joint2");
  EXPECT_EQ(joint2->getDof(0)->getName(), joint2->getName());

  EXPECT_EQ(joint2->getDof(0)->getPositionLowerLimit(), -1);
  EXPECT_EQ(joint2->getDof(0)->getPositionUpperLimit(), 1);
  EXPECT_EQ(joint2->getDof(0)->getPosition(), 0);

  EXPECT_EQ(joint2->getDof(0)->getVelocityLowerLimit(), -2);
  EXPECT_EQ(joint2->getDof(0)->getVelocityUpperLimit(), 2);
  EXPECT_EQ(joint2->getDof(0)->getVelocity(), 0);

  EXPECT_EQ(joint2->getDof(0)->getAccelerationLowerLimit(), -3);
  EXPECT_EQ(joint2->getDof(0)->getAccelerationUpperLimit(), 3);
  EXPECT_EQ(joint2->getDof(0)->getAcceleration(), 0);

  EXPECT_EQ(joint2->getDof(0)->getForceLowerLimit(), -4);
  EXPECT_EQ(joint2->getDof(0)->getForceUpperLimit(), 4);
  EXPECT_EQ(joint2->getDof(0)->getForce(), 0);

  // Test for mixture of old method and new method
  // Note: If there is a conflict, the data given in the dof element will win
  Joint* joint3 = skel1->getJoint("joint3");
  EXPECT_EQ(joint3->getDof(0)->getName(), joint3->getName() + "_1");
  EXPECT_EQ(joint3->getDof(0)->getPositionLowerLimit(), -1);
  EXPECT_EQ(joint3->getDof(0)->getPositionUpperLimit(), 1);
  EXPECT_EQ(joint3->getDof(0)->getPosition(), 5);

  EXPECT_EQ(joint3->getDof(1)->getName(), joint3->getName() + "_2");
  EXPECT_EQ(joint3->getDof(1)->getPositionLowerLimit(), -2);
  EXPECT_EQ(joint3->getDof(1)->getPositionUpperLimit(), 2);
  EXPECT_EQ(joint3->getDof(1)->getPosition(), -5);

  // Test for only some of the DOFs being specified
  Joint* joint4 = skel1->getJoint("joint4");
  EXPECT_EQ(joint4->getDof(0)->getName(), "joint4_1");
  EXPECT_EQ(joint4->getDof(0)->getPositionLowerLimit(), -1);
  EXPECT_EQ(joint4->getDof(0)->getPositionUpperLimit(), 1);
  EXPECT_EQ(joint4->getDof(0)->getVelocityLowerLimit(), -10);
  EXPECT_EQ(joint4->getDof(0)->getVelocityUpperLimit(), 10);

  EXPECT_EQ(joint4->getDof(1)->getName(), joint4->getName() + "_y");

  EXPECT_EQ(joint4->getDof(2)->getName(), "joint4_3");
  EXPECT_EQ(joint4->getDof(2)->getPositionLowerLimit(), -2);
  EXPECT_EQ(joint4->getDof(2)->getPositionUpperLimit(), 2);
  EXPECT_EQ(joint4->getDof(2)->getVelocityLowerLimit(), -20);
  EXPECT_EQ(joint4->getDof(2)->getVelocityUpperLimit(), 20);
}

//==============================================================================
TEST(SkelParser, JointDynamicsElements)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/joint_dynamics_elements_test.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");

  Joint* joint0 = skel1->getJoint("joint0");
  EXPECT_EQ(joint0->getDampingCoefficient(0), 1.0);
  EXPECT_EQ(joint0->getCoulombFriction(0), 5.0);
  EXPECT_EQ(joint0->getRestPosition(0), 0.1);
  EXPECT_EQ(joint0->getSpringStiffness(0), 3.0);

  Joint* joint1 = skel1->getJoint("joint1");
  EXPECT_EQ(joint1->getDampingCoefficient(0), 1.0);
  EXPECT_EQ(joint1->getCoulombFriction(0), 5.0);
  EXPECT_EQ(joint1->getRestPosition(0), 0.1);
  EXPECT_EQ(joint1->getSpringStiffness(0), 3.0);

  EXPECT_EQ(joint1->getDampingCoefficient(1), 2.0);
  EXPECT_EQ(joint1->getCoulombFriction(1), 4.0);
  EXPECT_EQ(joint1->getRestPosition(1), 0.2);
  EXPECT_EQ(joint1->getSpringStiffness(1), 2.0);

  EXPECT_EQ(joint1->getDampingCoefficient(2), 3.0);
  EXPECT_EQ(joint1->getCoulombFriction(2), 3.0);
  EXPECT_EQ(joint1->getRestPosition(2), 0.3);
  EXPECT_EQ(joint1->getSpringStiffness(2), 1.0);
}

//==============================================================================
TEST(SkelParser, Shapes)
{
  WorldPtr world
      = SkelParser::readWorld("dart://sample//skel/test/test_shapes.skel");
  EXPECT_NE(world, nullptr);

  const auto numSkels = world->getNumSkeletons();
  EXPECT_EQ(numSkels, 9);

  ShapePtr shape;
  SkeletonPtr skel;

  // Ground (box)
  skel = world->getSkeleton("ground skeleton");
  EXPECT_NE(skel, nullptr);

  // Box
  skel = world->getSkeleton("box skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<BoxShape>());
  auto boxShape = std::static_pointer_cast<BoxShape>(shape);
  EXPECT_EQ(boxShape->getSize(), Eigen::Vector3d(0.1, 0.05, 0.1));

  // Sphere
  skel = world->getSkeleton("sphere skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<SphereShape>());
  auto sphereShape = std::static_pointer_cast<SphereShape>(shape);
  EXPECT_EQ(sphereShape->getRadius(), 0.05);

  // Ellipsoid
  skel = world->getSkeleton("ellipsoid skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<EllipsoidShape>());
  auto ellipsoidShape = std::static_pointer_cast<EllipsoidShape>(shape);
  EXPECT_EQ(ellipsoidShape->getDiameters(), Eigen::Vector3d(0.05, 0.10, 0.15));

  // Cylinder
  skel = world->getSkeleton("cylinder skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<CylinderShape>());
  auto cylinderShape = std::static_pointer_cast<CylinderShape>(shape);
  EXPECT_EQ(cylinderShape->getHeight(), 0.1);
  EXPECT_EQ(cylinderShape->getRadius(), 0.05);

  // Capsule
  skel = world->getSkeleton("capsule skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<CapsuleShape>());
  auto capsuleShape = std::static_pointer_cast<CapsuleShape>(shape);
  EXPECT_EQ(capsuleShape->getHeight(), 0.1);
  EXPECT_EQ(capsuleShape->getRadius(), 0.05);

  // Cone
  skel = world->getSkeleton("cone skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<ConeShape>());
  auto coneShape = std::static_pointer_cast<CapsuleShape>(shape);
  EXPECT_EQ(coneShape->getHeight(), 0.1);
  EXPECT_EQ(coneShape->getRadius(), 0.05);

  // MultiSphere
  skel = world->getSkeleton("multi sphere skeleton");
  EXPECT_NE(skel, nullptr);
  shape = skel->getBodyNode(0)->getShapeNode(0)->getShape();
  EXPECT_TRUE(shape->is<MultiSphereConvexHullShape>());
  auto multiSphereShape
      = std::static_pointer_cast<MultiSphereConvexHullShape>(shape);
  EXPECT_EQ(multiSphereShape->getNumSpheres(), 2u);
  const auto& spheres = multiSphereShape->getSpheres();
  EXPECT_EQ(spheres[0].first, 0.05);
  EXPECT_EQ(spheres[0].second, Eigen::Vector3d(-0.075, 0.0, 0.0));
  EXPECT_EQ(spheres[1].first, 0.075);
  EXPECT_EQ(spheres[1].second, Eigen::Vector3d(+0.075, 0.0, 0.0));

  // Mesh
  skel = world->getSkeleton("mesh skeleton");
  EXPECT_NE(skel, nullptr);
}

//==============================================================================
TEST(SkelParser, AdditionalSampleFilesLoad)
{
  const std::vector<std::string> samples
      = {"dart://sample/skel/test/ball_joints.skel",
         "dart://sample/skel/test/dof_attribute_test.skel",
         "dart://sample/skel/test/joint_dynamics_elements_test.skel",
         "dart://sample/skel/test/joint_friction_test.skel",
         "dart://sample/skel/test/joint_limit_test.skel",
         "dart://sample/skel/test/planar_joint.skel",
         "dart://sample/skel/test/translational_joints.skel",
         "dart://sample/skel/test/simple_tree_structure.skel",
         "dart://sample/skel/test/test_drop_sphere.skel",
         "dart://sample/skel/test/test_single_body.skel",
         "dart://sample/skel/test/test_drop_box.skel",
         "dart://sample/skel/test/test_drop_box_offset.skel",
         "dart://sample/skel/test/test_drop_low_stiffness.skel",
         "dart://sample/skel/test/test_articulated_bodies.skel",
         "dart://sample/skel/test/test_articulated_bodies_10bodies.skel",
         "dart://sample/skel/test/test_adaptive_deformable.skel",
         "dart://sample/skel/test/test_shapes.skel",
         "dart://sample/skel/test/single_pendulum.skel",
         "dart://sample/skel/test/joint_actuator_type_test.skel",
         "dart://sample/skel/test/collision_of_prescribed_joints_test.skel",
         "dart://sample/skel/test/hybrid_dynamics_test.skel",
         "dart://sample/skel/test/free_joints.skel",
         "dart://sample/skel/test/file_info_world_test.skel",
         "dart://sample/skel/test/double_pendulum_euler_joint.skel",
         "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel",
         "dart://sample/skel/test/serial_chain_revolute_joint.skel",
         "dart://sample/skel/test/single_pendulum_ball_joint.skel",
         "dart://sample/skel/test/single_pendulum_euler_joint.skel",
         "dart://sample/skel/test/double_pendulum_ball_joint.skel",
         "dart://sample/skel/test/double_pendulum_with_base.skel",
         "dart://sample/skel/test/tree_structure_ball_joint.skel",
         "dart://sample/skel/test/tree_structure_euler_joint.skel"};

  for (const auto& uri : samples) {
    SCOPED_TRACE(uri);
    WorldPtr world = SkelParser::readWorld(uri);
    ASSERT_NE(world, nullptr);
    EXPECT_GT(world->getNumSkeletons(), 0u);
  }
}

//==============================================================================
TEST(SkelParser, SoftBodiesWorldProperties)
{
  const auto world
      = utils::SkelParser::readWorld("dart://sample/skel/softBodies.skel");
  ASSERT_NE(world, nullptr);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.001);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(0.0, -9.81, 0.0)));

  const auto detector = world->getConstraintSolver()->getCollisionDetector();
  ASSERT_NE(detector, nullptr);
  EXPECT_EQ(detector->getTypeView(), "fcl");

  const auto skel = world->getSkeleton("skeleton 1");
  ASSERT_NE(skel, nullptr);
  EXPECT_GT(skel->getNumSoftBodyNodes(), 0u);
  EXPECT_GT(skel->getNumBodyNodes(), 0u);
  EXPECT_GT(skel->getNumDofs(), 0u);
}

//==============================================================================
TEST(SkelParser, CubesWorldCollisionDetectorAndJointTypes)
{
  const auto world
      = utils::SkelParser::readWorld("dart://sample/skel/cubes.skel");
  ASSERT_NE(world, nullptr);
  EXPECT_GT(world->getNumSkeletons(), 2u);

  const auto detector = world->getConstraintSolver()->getCollisionDetector();
  ASSERT_NE(detector, nullptr);
  EXPECT_EQ(detector->getTypeView(), "dart");

  const auto ground = world->getSkeleton("ground skeleton");
  ASSERT_NE(ground, nullptr);
  ASSERT_NE(ground->getJoint(0), nullptr);
  EXPECT_NE(dynamic_cast<dynamics::WeldJoint*>(ground->getJoint(0)), nullptr);

  const auto boxSkel = world->getSkeleton("box skeleton");
  ASSERT_NE(boxSkel, nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::FreeJoint*>(boxSkel->getRootJoint()), nullptr);
}

//==============================================================================
TEST(SkelParser, FullbodyStructure)
{
  const auto world
      = utils::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  ASSERT_NE(world, nullptr);

  const auto skel = world->getSkeleton("fullbody1");
  ASSERT_NE(skel, nullptr);
  EXPECT_GT(skel->getNumBodyNodes(), 10u);
  EXPECT_GT(skel->getNumJoints(), 10u);
  EXPECT_GT(skel->getNumDofs(), 10u);
  EXPECT_NE(skel->getBodyNode("h_pelvis"), nullptr);
}

//==============================================================================
TEST(SkelParser, ReadWorldXMLRejectsMalformedXml)
{
  const std::string skelXml = "<skel><world></skel>";
  EXPECT_EQ(SkelParser::readWorldXML(skelXml), nullptr);
}

//==============================================================================
TEST(SkelParser, ReadWorldXMLMissingWorldElement)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <skeleton name="missing_world"/>
</skel>
)";

  EXPECT_EQ(SkelParser::readWorldXML(skelXml), nullptr);
}

//==============================================================================
TEST(SkelParser, ReadWorldMissingFileReturnsNull)
{
  const auto missingPath
      = std::filesystem::temp_directory_path() / "dart_missing_world.skel";
  std::error_code ec;
  std::filesystem::remove(missingPath, ec);

  EXPECT_EQ(SkelParser::readWorld(missingPath.string()), nullptr);
}

//==============================================================================
TEST(SkelParser, ReadSkeletonMissingFileReturnsNull)
{
  const auto missingPath
      = std::filesystem::temp_directory_path() / "dart_missing_skeleton.skel";
  std::error_code ec;
  std::filesystem::remove(missingPath, ec);

  EXPECT_EQ(SkelParser::readSkeleton(missingPath.string()), nullptr);
}

//==============================================================================
TEST(SkelParser, ReadSkeletonMissingRootElements)
{
  auto writeTemp = [](std::string_view xml) {
    const auto path = std::filesystem::temp_directory_path()
                      / "dart_skel_missing_root.skel";
    std::ofstream output(path.string(), std::ios::binary);
    output << xml;
    output.close();
    return path;
  };

  const auto noSkelPath = writeTemp("<root></root>");
  EXPECT_EQ(SkelParser::readSkeleton(noSkelPath.string()), nullptr);
  std::error_code ec;
  std::filesystem::remove(noSkelPath, ec);

  const auto noSkeletonPath = writeTemp(R"(
<skel version="1.0">
  <world name="default"/>
</skel>
)");
  EXPECT_EQ(SkelParser::readSkeleton(noSkeletonPath.string()), nullptr);
  std::filesystem::remove(noSkeletonPath, ec);
}

//==============================================================================
TEST(SkelParser, ReadWorldXmlPhysicsAndCollisionDetector)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <physics>
      <time_step>0.01</time_step>
      <gravity>0 0 -1</gravity>
      <collision_detector>fcl</collision_detector>
    </physics>
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.2</iyy>
            <izz>0.3</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.01);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -1.0)));

  const auto detector = world->getConstraintSolver()->getCollisionDetector();
  ASSERT_NE(detector, nullptr);
  EXPECT_EQ(detector->getTypeView(), "fcl");
}

//==============================================================================
TEST(SkelParser, VisualizationAndCollisionShapesFromXml)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>2.0</mass>
          <offset>0.01 0.02 0.03</offset>
        </inertia>
        <visualization_shape>
          <geometry>
            <box>
              <size>0.2 0.4 0.6</size>
            </box>
          </geometry>
          <color>0.1 0.2 0.3 0.4</color>
        </visualization_shape>
        <collision_shape>
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <collidable>0</collidable>
        </collision_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);

  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* body = skel->getBodyNode("link");
  ASSERT_NE(body, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 2u);
  EXPECT_DOUBLE_EQ(body->getInertia().getMass(), 2.0);
  EXPECT_GT(body->getInertia().getMoment().trace(), 0.0);

  ASSERT_GE(body->getNumShapeNodes(), 2u);
  const auto* visualNode = body->getShapeNode(0);
  ASSERT_NE(visualNode, nullptr);
  const auto* visual = visualNode->getVisualAspect();
  ASSERT_NE(visual, nullptr);
  EXPECT_TRUE(visual->getRGBA().isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)));

  const auto* collisionNode = body->getShapeNode(1);
  ASSERT_NE(collisionNode, nullptr);
  const auto* collision = collisionNode->getCollisionAspect();
  ASSERT_NE(collision, nullptr);
  EXPECT_FALSE(collision->isCollidable());
}

//==============================================================================
TEST(SkelParser, MarkerElementsCreateMarkers)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.2</iyy>
            <izz>0.3</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <marker name="marker1">
          <offset>1 2 3</offset>
        </marker>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  const auto* marker = skel->getMarker("marker1");
  ASSERT_NE(marker, nullptr);
  EXPECT_TRUE(marker->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(SkelParser, PlaneShapeDefaultsOffsetWhenMissing)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.2</iyy>
            <izz>0.3</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <plane>
              <normal>0 1 0</normal>
            </plane>
          </geometry>
        </visualization_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* body = skel->getBodyNode("link");
  ASSERT_NE(body, nullptr);
  auto shape = body->getShapeNode(0)->getShape();
  ASSERT_TRUE(shape->is<dynamics::PlaneShape>());
  auto plane = std::dynamic_pointer_cast<dynamics::PlaneShape>(shape);
  ASSERT_TRUE(plane);
  EXPECT_DOUBLE_EQ(plane->getOffset(), 0.0);
}

//==============================================================================
TEST(SkelParser, ReadWorldXmlFclMeshCollisionDetector)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <physics>
      <time_step>0.02</time_step>
      <gravity>0 1 -2</gravity>
      <collision_detector>fcl_mesh</collision_detector>
    </physics>
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.02);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(0.0, 1.0, -2.0)));

  const auto detector = world->getConstraintSolver()->getCollisionDetector();
  ASSERT_NE(detector, nullptr);
  EXPECT_EQ(detector->getTypeView(), "fcl");
}

//==============================================================================
TEST(SkelParser, ReadWorldXmlUnknownCollisionDetectorFallsBack)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <physics>
      <collision_detector>unknown_detector</collision_detector>
    </physics>
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto detector = world->getConstraintSolver()->getCollisionDetector();
  ASSERT_NE(detector, nullptr);
  EXPECT_EQ(detector->getTypeView(), "fcl");
}

//==============================================================================
TEST(SkelParser, ReadWorldXmlPlaneEllipsoidCylinderShapes)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="ground">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <plane>
              <normal>0 1 0</normal>
              <offset>0.2</offset>
            </plane>
          </geometry>
        </visualization_shape>
      </body>
      <body name="ellipsoid">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.2</ixx>
            <iyy>0.2</iyy>
            <izz>0.2</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <ellipsoid>
              <size>0.1 0.2 0.3</size>
            </ellipsoid>
          </geometry>
        </visualization_shape>
      </body>
      <body name="cylinder">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.3</ixx>
            <iyy>0.3</iyy>
            <izz>0.3</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <height>0.5</height>
            </cylinder>
          </geometry>
        </visualization_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>ground</child>
      </joint>
      <joint type="revolute" name="ellipsoid_joint">
        <parent>ground</parent>
        <child>ellipsoid</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint type="prismatic" name="cylinder_joint">
        <parent>ellipsoid</parent>
        <child>cylinder</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);

  auto* ground = skel->getBodyNode("ground");
  ASSERT_NE(ground, nullptr);
  auto planeShape = ground->getShapeNode(0)->getShape();
  ASSERT_TRUE(planeShape->is<dynamics::PlaneShape>());
  auto plane = std::dynamic_pointer_cast<dynamics::PlaneShape>(planeShape);
  ASSERT_TRUE(plane);
  EXPECT_TRUE(plane->getNormal().isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_DOUBLE_EQ(plane->getOffset(), 0.2);

  auto* ellipsoidBody = skel->getBodyNode("ellipsoid");
  ASSERT_NE(ellipsoidBody, nullptr);
  auto ellipsoidShape = ellipsoidBody->getShapeNode(0)->getShape();
  ASSERT_TRUE(ellipsoidShape->is<dynamics::EllipsoidShape>());
  auto ellipsoid
      = std::dynamic_pointer_cast<dynamics::EllipsoidShape>(ellipsoidShape);
  ASSERT_TRUE(ellipsoid);
  EXPECT_TRUE(
      ellipsoid->getDiameters().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));

  auto* cylinderBody = skel->getBodyNode("cylinder");
  ASSERT_NE(cylinderBody, nullptr);
  auto cylinderShape = cylinderBody->getShapeNode(0)->getShape();
  ASSERT_TRUE(cylinderShape->is<dynamics::CylinderShape>());
  auto cylinder
      = std::dynamic_pointer_cast<dynamics::CylinderShape>(cylinderShape);
  ASSERT_TRUE(cylinder);
  EXPECT_DOUBLE_EQ(cylinder->getRadius(), 0.2);
  EXPECT_DOUBLE_EQ(cylinder->getHeight(), 0.5);
}

//==============================================================================
TEST(SkelParser, ReadWorldFromFileMeshShape)
{
  const auto tempDir = std::filesystem::temp_directory_path();
  const auto meshPath = tempDir / "dart_mesh_shape.obj";
  const auto skelPath = tempDir / "dart_mesh_shape.skel";

  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <mesh>
              <file_name>dart_mesh_shape.obj</file_name>
              <scale>1 2 3</scale>
            </mesh>
          </geometry>
        </visualization_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  std::ofstream skelFile(skelPath.string(), std::ios::binary);
  ASSERT_TRUE(skelFile.is_open());
  skelFile << skelXml;
  skelFile.close();

  const auto world = SkelParser::readWorld(
      dart::common::Uri::createFromPath(skelPath.string()));
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* body = skel->getBodyNode("link");
  ASSERT_NE(body, nullptr);

  auto shape = body->getShapeNode(0)->getShape();
  ASSERT_TRUE(shape->is<dynamics::MeshShape>());
  auto mesh = std::dynamic_pointer_cast<const dynamics::MeshShape>(shape);
  ASSERT_TRUE(mesh);
  EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  std::error_code ec;
  std::filesystem::remove(meshPath, ec);
  std::filesystem::remove(skelPath, ec);
}

//==============================================================================
TEST(SkelParser, ReadWorldXmlSoftBodyEllipsoid)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="soft_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <soft_shape>
          <total_mass>1.0</total_mass>
          <geometry>
            <ellipsoid>
              <size>0.2 0.3 0.4</size>
              <num_slices>8</num_slices>
              <num_stacks>6</num_stacks>
            </ellipsoid>
          </geometry>
          <kv>2.0</kv>
          <ke>3.0</ke>
          <damp>0.4</damp>
        </soft_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>soft_link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumSoftBodyNodes(), 1u);
  auto* softBody = skel->getSoftBodyNode(0);
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);
}

//==============================================================================
TEST(SkelParser, MarkerDefaultsToZeroOffset)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.2</iyy>
            <izz>0.3</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <marker name="marker_zero"/>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  const auto* marker = skel->getMarker("marker_zero");
  ASSERT_NE(marker, nullptr);
  EXPECT_TRUE(marker->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d::Zero()));
}

//==============================================================================
TEST(SkelParser, VisualizationColorThreeComponents)
{
  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.2</iyy>
            <izz>0.3</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <color>0.1 0.2 0.3</color>
        </visualization_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
)";

  const auto world = SkelParser::readWorldXML(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  const auto* body = skel->getBodyNode("link");
  ASSERT_NE(body, nullptr);
  const auto* shapeNode = body->getShapeNode(0);
  ASSERT_NE(shapeNode, nullptr);
  const auto* visual = shapeNode->getVisualAspect();
  ASSERT_NE(visual, nullptr);
  EXPECT_TRUE(visual->getRGBA().isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 1.0)));
}

#include <dart/io/read.hpp>

namespace {

simulation::WorldPtr readWorldFromSkelString(std::string_view xml)
{
  static std::size_t counter = 0;
  const auto tempPath
      = std::filesystem::temp_directory_path()
        / ("dart_skel_temp_" + std::to_string(counter++) + ".skel");
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = SkelParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  return world;
}

} // namespace

//==============================================================================
TEST(SkelParser, InertiaFromShapeNodesUsesVolumes)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>2.0</mass>
          <offset>0.01 0.02 0.03</offset>
        </inertia>
        <visualization_shape>
          <geometry>
            <box>
              <size>0.2 0.4 0.6</size>
            </box>
          </geometry>
        </visualization_shape>
        <collision_shape>
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
        </collision_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* body = skel->getBodyNode("link");
  ASSERT_NE(body, nullptr);
  EXPECT_DOUBLE_EQ(body->getMass(), 2.0);
  EXPECT_GT(body->getInertia().getMoment().trace(), 0.0);
}

//==============================================================================
TEST(SkelParser, SoftBodySphereProperties)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="soft_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <soft_shape>
          <total_mass>1.5</total_mass>
          <geometry>
            <sphere>
              <radius>0.2</radius>
              <num_slices>6</num_slices>
              <num_stacks>4</num_stacks>
            </sphere>
          </geometry>
          <kv>2.0</kv>
          <ke>3.0</ke>
          <damp>0.4</damp>
        </soft_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>soft_link</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumSoftBodyNodes(), 1u);
  auto* softBody = skel->getSoftBodyNode(0);
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);
}

//==============================================================================
TEST(SkelParser, WorldXmlParsesJointAndShapeVariants)
{
  static std::size_t counter = 0;
  const auto tempDir = std::filesystem::temp_directory_path()
                       / ("dart_skel_variants_" + std::to_string(counter++));
  std::filesystem::create_directories(tempDir);
  const auto meshPath = tempDir / "mesh.obj";

  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string skelXml = R"(
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <ellipsoid>
              <size>0.2 0.3 0.4</size>
            </ellipsoid>
          </geometry>
        </visualization_shape>
      </body>
      <body name="link1">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <height>0.5</height>
            </cylinder>
          </geometry>
        </visualization_shape>
      </body>
      <body name="link2">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <mesh>
              <file_name>mesh.obj</file_name>
              <scale>1 2 3</scale>
            </mesh>
          </geometry>
        </visualization_shape>
      </body>
      <body name="link3">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link4">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link5">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="prismatic" name="prismatic_joint" actuator="force">
        <parent>base</parent>
        <child>link1</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
      </joint>
      <joint type="universal" name="universal_joint" actuator="passive">
        <parent>link1</parent>
        <child>link2</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
      </joint>
      <joint type="ball" name="ball_joint" actuator="velocity">
        <parent>link2</parent>
        <child>link3</child>
      </joint>
      <joint type="euler" name="euler_joint" actuator="acceleration">
        <parent>link3</parent>
        <child>link4</child>
        <axis_order>xyz</axis_order>
      </joint>
      <joint type="planar" name="planar_joint" actuator="locked">
        <parent>link4</parent>
        <child>link5</child>
        <plane type="arbitrary">
          <translation_axis1>
            <xyz>1 0 0</xyz>
          </translation_axis1>
          <translation_axis2>
            <xyz>0 1 0</xyz>
          </translation_axis2>
        </plane>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto baseUri
      = common::Uri::createFromPath((tempDir / "world.skel").string());
  const auto world = SkelParser::readWorldXML(skelXml, baseUri);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);

  EXPECT_NE(
      dynamic_cast<dynamics::PrismaticJoint*>(
          skel->getJoint("prismatic_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::UniversalJoint*>(
          skel->getJoint("universal_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::BallJoint*>(skel->getJoint("ball_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::EulerJoint*>(skel->getJoint("euler_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint")),
      nullptr);

  EXPECT_EQ(
      skel->getJoint("prismatic_joint")->getActuatorType(),
      dynamics::Joint::FORCE);
  EXPECT_EQ(
      skel->getJoint("universal_joint")->getActuatorType(),
      dynamics::Joint::PASSIVE);
  EXPECT_EQ(
      skel->getJoint("ball_joint")->getActuatorType(),
      dynamics::Joint::VELOCITY);
  EXPECT_EQ(
      skel->getJoint("euler_joint")->getActuatorType(),
      dynamics::Joint::ACCELERATION);
  EXPECT_EQ(
      skel->getJoint("planar_joint")->getActuatorType(),
      dynamics::Joint::LOCKED);

  auto* baseBody = skel->getBodyNode("base");
  auto* link1 = skel->getBodyNode("link1");
  auto* link2 = skel->getBodyNode("link2");
  ASSERT_NE(baseBody, nullptr);
  ASSERT_NE(link1, nullptr);
  ASSERT_NE(link2, nullptr);

  EXPECT_TRUE(
      baseBody->getShapeNode(0)->getShape()->is<dynamics::EllipsoidShape>());
  EXPECT_TRUE(
      link1->getShapeNode(0)->getShape()->is<dynamics::CylinderShape>());
  EXPECT_TRUE(link2->getShapeNode(0)->getShape()->is<dynamics::MeshShape>());

  std::error_code ec;
  std::filesystem::remove_all(tempDir, ec);
}

//==============================================================================
TEST(SkelParser, SoftBodyBoxProperties)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="soft_box">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <soft_shape>
          <total_mass>2.0</total_mass>
          <geometry>
            <box>
              <size>0.3 0.2 0.1</size>
              <frags>2 3 4</frags>
            </box>
          </geometry>
          <kv>1.2</kv>
          <ke>0.8</ke>
          <damp>0.3</damp>
        </soft_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>soft_box</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumSoftBodyNodes(), 1u);
  auto* softBody = skel->getSoftBodyNode(0);
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);
}

//==============================================================================
TEST(SkelParser, SoftBodyCylinderProperties)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="soft_cylinder">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <soft_shape>
          <total_mass>1.8</total_mass>
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <height>0.4</height>
              <num_slices>8</num_slices>
              <num_stacks>5</num_stacks>
              <num_rings>3</num_rings>
            </cylinder>
          </geometry>
          <kv>1.5</kv>
          <ke>0.9</ke>
          <damp>0.25</damp>
        </soft_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>soft_cylinder</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumSoftBodyNodes(), 1u);
  auto* softBody = skel->getSoftBodyNode(0);
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);
}

//==============================================================================
TEST(SkelParser, PyramidAndConeShapes)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="pyramid">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <pyramid>
              <base_width>0.3</base_width>
              <base_depth>0.2</base_depth>
              <height>0.4</height>
            </pyramid>
          </geometry>
        </visualization_shape>
      </body>
      <body name="cone">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <cone>
              <radius>0.15</radius>
              <height>0.3</height>
            </cone>
          </geometry>
        </visualization_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>pyramid</child>
      </joint>
      <joint type="weld" name="cone_joint">
        <parent>pyramid</parent>
        <child>cone</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);

  auto* pyramidBody = skel->getBodyNode("pyramid");
  ASSERT_NE(pyramidBody, nullptr);
  auto pyramidShape = pyramidBody->getShapeNode(0)->getShape();
  EXPECT_TRUE(pyramidShape->is<dynamics::PyramidShape>());

  auto* coneBody = skel->getBodyNode("cone");
  ASSERT_NE(coneBody, nullptr);
  auto coneShape = coneBody->getShapeNode(0)->getShape();
  EXPECT_TRUE(coneShape->is<dynamics::ConeShape>());
}

//==============================================================================
TEST(SkelParser, TranslationalJointInitialValues)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="slider">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="translational" name="slider_joint">
        <parent>base</parent>
        <child>slider</child>
        <init_pos>0.1 0.2 0.3</init_pos>
        <init_vel>-0.4 -0.5 -0.6</init_vel>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint = dynamic_cast<dynamics::TranslationalJoint*>(
      skel->getJoint("slider_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_TRUE(joint->getPositions().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(
      joint->getVelocities().isApprox(Eigen::Vector3d(-0.4, -0.5, -0.6)));
}

//==============================================================================
TEST(SkelParser, EulerJointAxisOrderAndInitials)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="euler" name="euler_joint">
        <parent>base</parent>
        <child>link</child>
        <axis_order>zyx</axis_order>
        <init_pos>0.1 0.2 0.3</init_pos>
        <init_vel>0.4 0.5 0.6</init_vel>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint
      = dynamic_cast<dynamics::EulerJoint*>(skel->getJoint("euler_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getAxisOrder(), dynamics::EulerJoint::AxisOrder::ZYX);
  EXPECT_TRUE(joint->getPositions().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(joint->getVelocities().isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
}

//==============================================================================
TEST(SkelParser, PlanarJointUnknownPlaneDefaultsToXY)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="platform">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="planar" name="planar_joint">
        <parent>base</parent>
        <child>platform</child>
        <plane type="unknown" />
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getPlaneType(), dynamics::PlanarJoint::PlaneType::XY);
}

//==============================================================================
TEST(SkelParser, JointActuatorServoAndLocked)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="revolute" name="servo_joint" actuator="servo">
        <parent>world</parent>
        <child>base</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint type="revolute" name="locked_joint" actuator="locked">
        <parent>base</parent>
        <child>link</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* servoJoint = skel->getJoint("servo_joint");
  auto* lockedJoint = skel->getJoint("locked_joint");
  ASSERT_NE(servoJoint, nullptr);
  ASSERT_NE(lockedJoint, nullptr);
  EXPECT_EQ(servoJoint->getActuatorType(), dynamics::Joint::SERVO);
  EXPECT_EQ(lockedJoint->getActuatorType(), dynamics::Joint::LOCKED);
}

//==============================================================================
TEST(SkelParser, DofPositionLimitsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="revolute" name="revolute_joint">
        <parent>base</parent>
        <child>link</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <dof local_index="0">
          <position lower="-0.5" upper="1.0" initial="0.25" />
        </dof>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint = skel->getJoint("revolute_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -0.5);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 1.0);
  EXPECT_NEAR(joint->getPosition(0), 0.25, 1e-9);
}

//==============================================================================
TEST(SkelParser, ReadWorldStringAppliesPhysics)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <physics>
      <time_step>0.02</time_step>
      <gravity>1 2 3</gravity>
    </physics>
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.02);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(SkelParser, MeshShapeFromSkelString)
{
  const auto tempDir = std::filesystem::temp_directory_path();
  const auto meshPath = tempDir / "dart_mesh_shape_string.obj";
  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string skelXml = std::string(R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <mesh>
              <file_name>)") + meshPath.string()
                              + R"(</file_name>
              <scale>1 2 3</scale>
            </mesh>
          </geometry>
        </visualization_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>link</child>
      </joint>
    </skeleton>
  </world>
</skel>
 )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* body = skel->getBodyNode("link");
  ASSERT_NE(body, nullptr);
  auto shape = body->getShapeNode(0)->getShape();
  ASSERT_TRUE(shape->is<dynamics::MeshShape>());
  auto mesh = std::dynamic_pointer_cast<const dynamics::MeshShape>(shape);
  ASSERT_TRUE(mesh);
  EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  std::error_code ec;
  std::filesystem::remove(meshPath, ec);
}

//==============================================================================
TEST(SkelParser, JointAxisDynamicsAndLimitsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="revolute" name="rev_joint">
        <parent>base</parent>
        <child>link</child>
        <axis>
          <xyz>0 0 1</xyz>
          <dynamics>
            <damping>1.1</damping>
            <friction>0.2</friction>
            <spring_rest_position>0.3</spring_rest_position>
            <spring_stiffness>4.4</spring_stiffness>
          </dynamics>
          <limit>
            <lower>-1.0</lower>
            <upper>2.0</upper>
          </limit>
        </axis>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint = skel->getJoint("rev_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 1.1);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.2);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.3);
  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 4.4);
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -1.0);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 2.0);
}

//==============================================================================
TEST(SkelParser, DofDynamicsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="revolute" name="rev_joint">
        <parent>base</parent>
        <child>link</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
        <dof local_index="0">
          <position lower="-0.2" upper="0.4" initial="0.1" />
          <damping>2.0</damping>
          <friction>0.6</friction>
          <spring_rest_position>0.7</spring_rest_position>
          <spring_stiffness>3.3</spring_stiffness>
        </dof>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint = skel->getJoint("rev_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -0.2);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 0.4);
  EXPECT_NEAR(joint->getPosition(0), 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 2.0);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.6);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.7);
  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 3.3);
}

//==============================================================================
TEST(SkelParser, FreeJointInitialValuesFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
        <init_pos>0.1 0.2 0.3 0.4 0.5 0.6</init_pos>
        <init_vel>-0.1 -0.2 -0.3 -0.4 -0.5 -0.6</init_vel>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);
  auto* joint
      = dynamic_cast<dynamics::FreeJoint*>(skel->getJoint("root_joint"));
  ASSERT_NE(joint, nullptr);
  const Eigen::Vector6d expectedPos(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
  const Eigen::Vector6d expectedVel(-0.1, -0.2, -0.3, -0.4, -0.5, -0.6);
  EXPECT_TRUE(joint->getPositions().isApprox(expectedPos));
  EXPECT_TRUE(joint->getVelocities().isApprox(expectedVel));
}

//==============================================================================
TEST(SkelParser, JointTypesFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="universal_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="ball_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="planar_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="euler_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="universal" name="universal_joint">
        <parent>base</parent>
        <child>universal_link</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
      </joint>
      <joint type="ball" name="ball_joint">
        <parent>universal_link</parent>
        <child>ball_link</child>
      </joint>
      <joint type="planar" name="planar_joint">
        <parent>ball_link</parent>
        <child>planar_link</child>
        <plane type="arbitrary">
          <translation_axis1>
            <xyz>1 0 0</xyz>
          </translation_axis1>
          <translation_axis2>
            <xyz>0 1 0</xyz>
          </translation_axis2>
        </plane>
      </joint>
      <joint type="euler" name="euler_joint">
        <parent>planar_link</parent>
        <child>euler_link</child>
        <axis_order>xyz</axis_order>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);

  EXPECT_NE(
      dynamic_cast<dynamics::UniversalJoint*>(
          skel->getJoint("universal_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::BallJoint*>(skel->getJoint("ball_joint")),
      nullptr);
  auto* planar
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint"));
  ASSERT_NE(planar, nullptr);
  EXPECT_EQ(
      planar->getPlaneType(), dynamics::PlanarJoint::PlaneType::ARBITRARY);
  auto* euler
      = dynamic_cast<dynamics::EulerJoint*>(skel->getJoint("euler_joint"));
  ASSERT_NE(euler, nullptr);
  EXPECT_EQ(euler->getAxisOrder(), dynamics::EulerJoint::AxisOrder::XYZ);
}

#include <tinyxml2.h>

namespace dart {
namespace io {

dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const ReadOptions& options)
{
  return utils::SkelParser::readSkeleton(uri, options.resourceRetriever);
}

} // namespace io
} // namespace dart

namespace {

std::filesystem::path makeTempSkelPath(const std::string& tag)
{
  static std::size_t counter = 0;
  const auto filename
      = "dart_skel_" + tag + "_" + std::to_string(counter++) + ".skel";
  return std::filesystem::temp_directory_path() / filename;
}

bool writeSkelXml(const std::filesystem::path& path, std::string_view xml)
{
  tinyxml2::XMLDocument doc;
  const std::string xmlCopy(xml);
  if (doc.Parse(xmlCopy.c_str()) != tinyxml2::XML_SUCCESS) {
    return false;
  }
  return doc.SaveFile(path.string().c_str()) == tinyxml2::XML_SUCCESS;
}

dynamics::SkeletonPtr readSkeletonFromSkelXml(std::string_view xml)
{
  const auto path = makeTempSkelPath("inline");
  if (!writeSkelXml(path, xml)) {
    return nullptr;
  }
  auto skeleton = dart::io::readSkeleton(path.string());
  std::error_code ec;
  std::filesystem::remove(path, ec);
  return skeleton;
}

} // namespace

//==============================================================================
TEST(SkelParser, ParsesJointTypesLimitsAndDofsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="skel">
    <body name="base">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link1">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link3">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link4">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link5">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link6">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>base</child>
      <init_pos>0.1 0.2 0.3 0.4 0.5 0.6</init_pos>
      <init_vel>-0.1 -0.2 -0.3 -0.4 -0.5 -0.6</init_vel>
    </joint>
    <joint type="prismatic" name="prismatic_joint">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-0.2</lower>
          <upper>0.4</upper>
        </limit>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.2</friction>
          <spring_rest_position>0.3</spring_rest_position>
          <spring_stiffness>4.0</spring_stiffness>
        </dynamics>
      </axis>
      <dof local_index="0">
        <position lower="-0.2" upper="0.4" initial="0.1" />
        <damping>0.15</damping>
        <spring_stiffness>2.5</spring_stiffness>
      </dof>
    </joint>
    <joint type="translational" name="translational_joint">
      <parent>link1</parent>
      <child>link3</child>
      <init_pos>0.1 0.2 0.3</init_pos>
      <init_vel>-0.4 -0.5 -0.6</init_vel>
      <dof local_index="1">
        <position lower="-1.0" upper="1.0" initial="0.2" />
      </dof>
    </joint>
    <joint type="universal" name="universal_joint">
      <parent>link3</parent>
      <child>link4</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-0.4</lower>
          <upper>0.6</upper>
        </limit>
        <dynamics>
          <damping>0.2</damping>
          <spring_stiffness>0.9</spring_stiffness>
        </dynamics>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.7</upper>
        </limit>
        <dynamics>
          <damping>0.3</damping>
          <spring_stiffness>1.1</spring_stiffness>
        </dynamics>
      </axis2>
      <init_pos>0.1 0.2</init_pos>
      <init_vel>0.3 0.4</init_vel>
      <dof local_index="0">
        <position lower="-0.4" upper="0.6" initial="0.1" />
      </dof>
      <dof local_index="1">
        <position lower="-0.5" upper="0.7" initial="0.2" />
      </dof>
    </joint>
    <joint type="ball" name="ball_joint">
      <parent>link4</parent>
      <child>link5</child>
      <init_pos>0.1 0.2 0.3</init_pos>
      <init_vel>-0.1 -0.2 -0.3</init_vel>
      <dof local_index="2">
        <position lower="-0.3" upper="0.3" initial="0.2" />
      </dof>
    </joint>
    <joint type="planar" name="planar_joint">
      <parent>link5</parent>
      <child>link6</child>
      <plane type="arbitrary">
        <translation_axis1>
          <xyz>1 0 0</xyz>
        </translation_axis1>
        <translation_axis2>
          <xyz>0 1 0</xyz>
        </translation_axis2>
      </plane>
      <init_pos>0.2 0.3 0.4</init_pos>
      <init_vel>-0.2 -0.3 -0.4</init_vel>
    </joint>
  </skeleton>
</skel>
  )";

  const auto skel = readSkeletonFromSkelXml(skelXml);
  ASSERT_NE(skel, nullptr);

  auto* prismatic = dynamic_cast<dynamics::PrismaticJoint*>(
      skel->getJoint("prismatic_joint"));
  ASSERT_NE(prismatic, nullptr);
  EXPECT_DOUBLE_EQ(prismatic->getPositionLowerLimit(0), -0.2);
  EXPECT_DOUBLE_EQ(prismatic->getPositionUpperLimit(0), 0.4);
  EXPECT_DOUBLE_EQ(prismatic->getDampingCoefficient(0), 0.15);
  EXPECT_DOUBLE_EQ(prismatic->getSpringStiffness(0), 2.5);

  auto* translational = dynamic_cast<dynamics::TranslationalJoint*>(
      skel->getJoint("translational_joint"));
  ASSERT_NE(translational, nullptr);
  EXPECT_TRUE(
      translational->getPositions().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(translational->getVelocities().isApprox(
      Eigen::Vector3d(-0.4, -0.5, -0.6)));

  auto* universal = dynamic_cast<dynamics::UniversalJoint*>(
      skel->getJoint("universal_joint"));
  ASSERT_NE(universal, nullptr);
  EXPECT_DOUBLE_EQ(universal->getPositionLowerLimit(0), -0.4);
  EXPECT_DOUBLE_EQ(universal->getPositionUpperLimit(0), 0.6);
  EXPECT_DOUBLE_EQ(universal->getDampingCoefficient(0), 0.2);
  EXPECT_DOUBLE_EQ(universal->getSpringStiffness(0), 0.9);
  EXPECT_DOUBLE_EQ(universal->getPositionLowerLimit(1), -0.5);
  EXPECT_DOUBLE_EQ(universal->getPositionUpperLimit(1), 0.7);
  EXPECT_DOUBLE_EQ(universal->getDampingCoefficient(1), 0.3);
  EXPECT_DOUBLE_EQ(universal->getSpringStiffness(1), 1.1);

  auto* ball = dynamic_cast<dynamics::BallJoint*>(skel->getJoint("ball_joint"));
  ASSERT_NE(ball, nullptr);
  EXPECT_EQ(ball->getNumDofs(), 3u);

  auto* planar
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint"));
  ASSERT_NE(planar, nullptr);
  EXPECT_EQ(
      planar->getPlaneType(), dynamics::PlanarJoint::PlaneType::ARBITRARY);
  EXPECT_TRUE(planar->getPositions().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_TRUE(
      planar->getVelocities().isApprox(Eigen::Vector3d(-0.2, -0.3, -0.4)));
}

//==============================================================================
TEST(SkelParser, ParsesRigidShapeVariantsFromXml)
{
  static std::size_t counter = 0;
  const auto tempDir = std::filesystem::temp_directory_path()
                       / ("dart_skel_shapes_" + std::to_string(counter++));
  std::filesystem::create_directories(tempDir);

  const auto meshPath = tempDir / "mesh.obj";
  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const auto skelPath = tempDir / "shapes.skel";
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="skel">
    <body name="box_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <box><size>0.2 0.3 0.4</size></box>
        </geometry>
      </visualization_shape>
    </body>
    <body name="ellipsoid_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <ellipsoid><size>0.1 0.2 0.3</size></ellipsoid>
        </geometry>
      </visualization_shape>
    </body>
    <body name="cylinder_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <cylinder><radius>0.2</radius><height>0.5</height></cylinder>
        </geometry>
      </visualization_shape>
    </body>
    <body name="capsule_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <capsule><radius>0.15</radius><height>0.4</height></capsule>
        </geometry>
      </visualization_shape>
    </body>
    <body name="mesh_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <mesh>
            <file_name>mesh.obj</file_name>
            <scale>2 3 4</scale>
          </mesh>
        </geometry>
      </visualization_shape>
    </body>
    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>box_body</child>
    </joint>
    <joint type="weld" name="ellipsoid_joint">
      <parent>box_body</parent>
      <child>ellipsoid_body</child>
    </joint>
    <joint type="weld" name="cylinder_joint">
      <parent>ellipsoid_body</parent>
      <child>cylinder_body</child>
    </joint>
    <joint type="weld" name="capsule_joint">
      <parent>cylinder_body</parent>
      <child>capsule_body</child>
    </joint>
    <joint type="weld" name="mesh_joint">
      <parent>capsule_body</parent>
      <child>mesh_body</child>
    </joint>
  </skeleton>
</skel>
  )";

  ASSERT_TRUE(writeSkelXml(skelPath, skelXml));
  const auto skel = dart::io::readSkeleton(skelPath.string());
  ASSERT_NE(skel, nullptr);

  auto* boxBody = skel->getBodyNode("box_body");
  ASSERT_NE(boxBody, nullptr);
  EXPECT_TRUE(boxBody->getShapeNode(0)->getShape()->is<dynamics::BoxShape>());

  auto* ellipsoidBody = skel->getBodyNode("ellipsoid_body");
  ASSERT_NE(ellipsoidBody, nullptr);
  EXPECT_TRUE(ellipsoidBody->getShapeNode(0)
                  ->getShape()
                  ->is<dynamics::EllipsoidShape>());

  auto* cylinderBody = skel->getBodyNode("cylinder_body");
  ASSERT_NE(cylinderBody, nullptr);
  EXPECT_TRUE(
      cylinderBody->getShapeNode(0)->getShape()->is<dynamics::CylinderShape>());

  auto* capsuleBody = skel->getBodyNode("capsule_body");
  ASSERT_NE(capsuleBody, nullptr);
  EXPECT_TRUE(
      capsuleBody->getShapeNode(0)->getShape()->is<dynamics::CapsuleShape>());

  auto* meshBody = skel->getBodyNode("mesh_body");
  ASSERT_NE(meshBody, nullptr);
  auto meshShape = std::dynamic_pointer_cast<dynamics::MeshShape>(
      meshBody->getShapeNode(0)->getShape());
  ASSERT_NE(meshShape, nullptr);
  EXPECT_TRUE(meshShape->getScale().isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));

  std::error_code ec;
  std::filesystem::remove_all(tempDir, ec);
}

//==============================================================================
TEST(SkelParser, ParsesSoftBodyVariantsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="soft_skel">
    <body name="soft_sphere">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.0</total_mass>
        <geometry>
          <sphere>
            <radius>0.2</radius>
            <num_slices>6</num_slices>
            <num_stacks>4</num_stacks>
          </sphere>
        </geometry>
        <kv>1.0</kv>
        <ke>2.0</ke>
        <damp>0.3</damp>
      </soft_shape>
    </body>
    <body name="soft_box">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.1</total_mass>
        <geometry>
          <box>
            <size>0.3 0.2 0.1</size>
            <frags>2 3 4</frags>
          </box>
        </geometry>
        <kv>1.2</kv>
        <ke>2.2</ke>
        <damp>0.4</damp>
      </soft_shape>
    </body>
    <body name="soft_ellipsoid">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.2</total_mass>
        <geometry>
          <ellipsoid>
            <size>0.2 0.3 0.4</size>
            <num_slices>8</num_slices>
            <num_stacks>5</num_stacks>
          </ellipsoid>
        </geometry>
        <kv>1.3</kv>
        <ke>2.3</ke>
        <damp>0.5</damp>
      </soft_shape>
    </body>
    <body name="soft_cylinder">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.3</total_mass>
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <height>0.4</height>
            <num_slices>8</num_slices>
            <num_stacks>5</num_stacks>
            <num_rings>3</num_rings>
          </cylinder>
        </geometry>
        <kv>1.4</kv>
        <ke>2.4</ke>
        <damp>0.6</damp>
      </soft_shape>
    </body>
    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>soft_sphere</child>
    </joint>
    <joint type="weld" name="soft_box_joint">
      <parent>soft_sphere</parent>
      <child>soft_box</child>
    </joint>
    <joint type="weld" name="soft_ellipsoid_joint">
      <parent>soft_box</parent>
      <child>soft_ellipsoid</child>
    </joint>
    <joint type="weld" name="soft_cylinder_joint">
      <parent>soft_ellipsoid</parent>
      <child>soft_cylinder</child>
    </joint>
  </skeleton>
</skel>
  )";

  const auto skel = readSkeletonFromSkelXml(skelXml);
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumSoftBodyNodes(), 4u);

  for (std::size_t i = 0; i < skel->getNumSoftBodyNodes(); ++i) {
    auto* softBody = skel->getSoftBodyNode(i);
    ASSERT_NE(softBody, nullptr);
    EXPECT_GT(softBody->getNumPointMasses(), 0u);
  }
}

//==============================================================================
TEST(SkelParser, JointDynamicsAndDofLimitsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="base">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="slider">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="universal_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <body name="planar_link">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint type="prismatic" name="slide_joint">
        <parent>base</parent>
        <child>slider</child>
        <axis>
          <xyz>1 0 0</xyz>
          <damping>0.9</damping>
          <dynamics>
            <friction>0.2</friction>
            <spring_rest_position>0.3</spring_rest_position>
            <spring_stiffness>4.1</spring_stiffness>
          </dynamics>
          <limit>
            <lower>-0.4</lower>
            <upper>0.6</upper>
          </limit>
        </axis>
        <dof local_index="0" name="slide_dof">
          <velocity lower="-1.2" upper="1.3" initial="0.5" />
          <acceleration lower="-2.2" upper="2.3" initial="0.6" />
          <force lower="-3.2" upper="3.3" initial="0.7" />
        </dof>
      </joint>
      <joint type="universal" name="universal_joint">
        <parent>slider</parent>
        <child>universal_link</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-0.7</lower>
            <upper>0.8</upper>
          </limit>
          <dynamics>
            <damping>0.4</damping>
            <spring_stiffness>1.5</spring_stiffness>
          </dynamics>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-0.9</lower>
            <upper>1.1</upper>
          </limit>
          <dynamics>
            <damping>0.6</damping>
            <spring_stiffness>1.7</spring_stiffness>
          </dynamics>
        </axis2>
        <init_pos>0.1 0.2</init_pos>
        <init_vel>-0.1 -0.2</init_vel>
      </joint>
      <joint type="planar" name="planar_joint">
        <parent>universal_link</parent>
        <child>planar_link</child>
        <plane type="arbitrary">
          <translation_axis1>
            <xyz>1 0 0</xyz>
          </translation_axis1>
          <translation_axis2>
            <xyz>0 1 0</xyz>
          </translation_axis2>
        </plane>
        <init_pos>0.2 0.3 0.4</init_pos>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);

  auto* prismatic
      = dynamic_cast<dynamics::PrismaticJoint*>(skel->getJoint("slide_joint"));
  ASSERT_NE(prismatic, nullptr);
  EXPECT_TRUE(prismatic->getAxis().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_DOUBLE_EQ(prismatic->getDampingCoefficient(0), 0.9);
  EXPECT_DOUBLE_EQ(prismatic->getCoulombFriction(0), 0.2);
  EXPECT_DOUBLE_EQ(prismatic->getRestPosition(0), 0.3);
  EXPECT_DOUBLE_EQ(prismatic->getSpringStiffness(0), 4.1);
  EXPECT_DOUBLE_EQ(prismatic->getPositionLowerLimit(0), -0.4);
  EXPECT_DOUBLE_EQ(prismatic->getPositionUpperLimit(0), 0.6);
  auto* prismaticDof = prismatic->getDof(0);
  ASSERT_NE(prismaticDof, nullptr);
  EXPECT_DOUBLE_EQ(prismaticDof->getVelocityLowerLimit(), -1.2);
  EXPECT_DOUBLE_EQ(prismaticDof->getVelocityUpperLimit(), 1.3);
  EXPECT_DOUBLE_EQ(prismaticDof->getAccelerationLowerLimit(), -2.2);
  EXPECT_DOUBLE_EQ(prismaticDof->getAccelerationUpperLimit(), 2.3);
  EXPECT_DOUBLE_EQ(prismaticDof->getForceLowerLimit(), -3.2);
  EXPECT_DOUBLE_EQ(prismaticDof->getForceUpperLimit(), 3.3);
  EXPECT_NEAR(prismatic->getVelocity(0), 0.5, 1e-9);

  auto* universal = dynamic_cast<dynamics::UniversalJoint*>(
      skel->getJoint("universal_joint"));
  ASSERT_NE(universal, nullptr);
  EXPECT_TRUE(universal->getAxis1().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(universal->getAxis2().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_DOUBLE_EQ(universal->getPositionLowerLimit(0), -0.7);
  EXPECT_DOUBLE_EQ(universal->getPositionUpperLimit(0), 0.8);
  EXPECT_DOUBLE_EQ(universal->getDampingCoefficient(0), 0.4);
  EXPECT_DOUBLE_EQ(universal->getSpringStiffness(0), 1.5);
  EXPECT_DOUBLE_EQ(universal->getPositionLowerLimit(1), -0.9);
  EXPECT_DOUBLE_EQ(universal->getPositionUpperLimit(1), 1.1);
  EXPECT_DOUBLE_EQ(universal->getDampingCoefficient(1), 0.6);
  EXPECT_DOUBLE_EQ(universal->getSpringStiffness(1), 1.7);

  auto* planar
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint"));
  ASSERT_NE(planar, nullptr);
  EXPECT_EQ(
      planar->getPlaneType(), dynamics::PlanarJoint::PlaneType::ARBITRARY);
  EXPECT_TRUE(
      planar->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      planar->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(planar->getPositions().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
}

//==============================================================================
TEST(SkelParser, MeshEllipsoidAndSoftBodyParsingFromXml)
{
  const auto tempDir = std::filesystem::temp_directory_path();
  const auto meshPath = tempDir / "dart_skel_mesh_variants.obj";
  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string skelXml = std::string(R"(<?xml version="1.0" ?>
<skel version="1.0">
  <world name="world">
    <skeleton name="skel">
      <body name="ellipsoid_body">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <ellipsoid>
              <size>0.2 0.4 0.6</size>
            </ellipsoid>
          </geometry>
        </visualization_shape>
      </body>
      <body name="mesh_body">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <visualization_shape>
          <geometry>
            <mesh>
              <file_name>)") + meshPath.string()
                              + R"(</file_name>
              <scale>1 2 3</scale>
            </mesh>
          </geometry>
        </visualization_shape>
      </body>
      <body name="soft_box">
        <inertia>
          <mass>1.0</mass>
          <moment_of_inertia>
            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
            <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
          </moment_of_inertia>
        </inertia>
        <soft_shape>
          <total_mass>1.2</total_mass>
          <transformation>0.1 0.2 0.3 0 0 0</transformation>
          <geometry>
            <box>
              <size>0.3 0.2 0.1</size>
              <frags>2 2 2</frags>
            </box>
          </geometry>
          <kv>1.1</kv>
          <ke>2.2</ke>
          <damp>0.4</damp>
        </soft_shape>
      </body>
      <joint type="free" name="root_joint">
        <parent>world</parent>
        <child>ellipsoid_body</child>
      </joint>
      <joint type="weld" name="mesh_joint">
        <parent>ellipsoid_body</parent>
        <child>mesh_body</child>
      </joint>
      <joint type="weld" name="soft_joint">
        <parent>mesh_body</parent>
        <child>soft_box</child>
      </joint>
    </skeleton>
  </world>
</skel>
  )";

  const auto world = readWorldFromSkelString(skelXml);
  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("skel");
  ASSERT_NE(skel, nullptr);

  auto* ellipsoidBody = skel->getBodyNode("ellipsoid_body");
  ASSERT_NE(ellipsoidBody, nullptr);
  auto ellipsoidShape = ellipsoidBody->getShapeNode(0)->getShape();
  ASSERT_TRUE(ellipsoidShape->is<dynamics::EllipsoidShape>());
  auto ellipsoid
      = std::dynamic_pointer_cast<dynamics::EllipsoidShape>(ellipsoidShape);
  ASSERT_NE(ellipsoid, nullptr);
  EXPECT_TRUE(
      ellipsoid->getDiameters().isApprox(Eigen::Vector3d(0.2, 0.4, 0.6)));

  auto* meshBody = skel->getBodyNode("mesh_body");
  ASSERT_NE(meshBody, nullptr);
  auto meshShape = meshBody->getShapeNode(0)->getShape();
  ASSERT_TRUE(meshShape->is<dynamics::MeshShape>());
  auto mesh = std::dynamic_pointer_cast<dynamics::MeshShape>(meshShape);
  ASSERT_NE(mesh, nullptr);
  EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  ASSERT_EQ(skel->getNumSoftBodyNodes(), 1u);
  auto* softBody = skel->getSoftBodyNode(0);
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);

  std::error_code ec;
  std::filesystem::remove(meshPath, ec);
}
