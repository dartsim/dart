/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/utils/Paths.h"
#include "dart/simulation/World.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
TEST(Parser, DataStructure)
{
  bool v1 = true;
  int v2 = -3;
  unsigned int v3 = 1;
  float v4 = -3.140f;
  double v5 = 1.4576640;
  char v6 = 'd';
  Eigen::Vector2d v7 = Eigen::Vector2d::Ones();
  Eigen::Vector3d v8 = Eigen::Vector3d::Ones();
  //Eigen::Vector3d v9 = Eigen::Vector3d::Ones();
  //math::SO3 v10;
  Eigen::Isometry3d v11 = Eigen::Isometry3d::Identity();

  std::string str1 = toString(v1);
  std::string str2 = toString(v2);
  std::string str3 = toString(v3);
  std::string str4 = toString(v4);
  std::string str5 = toString(v5);
  std::string str6 = toString(v6);
  std::string str7 = toString(v7);
  std::string str8 = toString(v8);
  //std::string str9 = toString(v9);
  //std::string str10 = toString(v10);
  std::string str11 = toString(v11);

  bool b = toBool(str1);
  int i = toInt(str2);
  unsigned int ui = toUInt(str3);
  float f = toFloat(str4);
  double d = toDouble(str5);
  char c = toChar(str6);
  Eigen::Vector2d vec2 = toVector2d(str7);
  Eigen::Vector3d vec3 = toVector3d(str8);
  //Eigen::Vector3d valso3 = toVector3d(str9);
  //math::SO3 valSO3 = toSO3(str10);
  Eigen::Isometry3d valSE3 = toIsometry3d(str11);

  EXPECT_EQ(b, v1);
  EXPECT_EQ(i, v2);
  EXPECT_EQ(ui, v3);
  EXPECT_EQ(f, v4);
  EXPECT_EQ(d, v5);
  EXPECT_EQ(c, v6);
  for (int i = 0; i < 2; i++)
    EXPECT_EQ(vec2[i], v7[i]);
  EXPECT_EQ(vec3, v8);
  //EXPECT_EQ(valso3, v9);
  //EXPECT_EQ(valSO3, v10);
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
      EXPECT_EQ(valSE3(i,j), v11(i,j));
  }
}

//==============================================================================
TEST(Parser, EmptyWorld)
{
  World* world = SkelParser::readWorld(DART_DATA_PATH"skel/test/empty.skel");

  EXPECT_TRUE(world != NULL);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), 0);
  EXPECT_EQ(world->getGravity()(2), -9.81);
  EXPECT_EQ(world->getNumSkeletons(), 0);

  EXPECT_EQ(world->getTime(), 0);
  world->step();
  EXPECT_EQ(world->getTime(), world->getTimeStep());

  delete world;
}

//==============================================================================
TEST(Parser, SinglePendulum)
{
  World* world = SkelParser::readWorld(
                   DART_DATA_PATH"skel/test/single_pendulum.skel");

  EXPECT_TRUE(world != NULL);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), -9.81);
  EXPECT_EQ(world->getGravity()(2), 0);
  EXPECT_EQ(world->getNumSkeletons(), 1);

  Skeleton* skel1 = world->getSkeleton("single_pendulum");

  EXPECT_EQ(skel1->getNumBodyNodes(), 1);

  world->step();

  delete world;
}

//==============================================================================
TEST(Parser, SerialChain)
{
  World* world = SkelParser::readWorld(
                   DART_DATA_PATH"skel/test/serial_chain_ball_joint.skel");

  EXPECT_TRUE(world != NULL);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), -9.81);
  EXPECT_EQ(world->getGravity()(2), 0);
  EXPECT_EQ(world->getNumSkeletons(), 1);

  Skeleton* skel1 = world->getSkeleton("skeleton 1");

  EXPECT_EQ(skel1->getNumBodyNodes(), 10);

  world->step();

  delete world;
}

//==============================================================================
TEST(Parser, RigidAndSoftBodies)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  World* world = SkelParser::readWorld(
                   DART_DATA_PATH"skel/test/test_articulated_bodies.skel");
  EXPECT_TRUE(world != NULL);

  Skeleton* skel1 = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel1 != NULL);
  EXPECT_EQ(skel1->getNumBodyNodes(), 2);
  EXPECT_EQ(skel1->getNumRigidBodyNodes(), 1);
  EXPECT_EQ(skel1->getNumSoftBodyNodes(), 1);

  SoftBodyNode* sbn = skel1->getSoftBodyNode(0);
  EXPECT_TRUE(sbn->getNumPointMasses() > 0);

  world->step();

  delete world;
}

//==============================================================================
TEST(Parser, PlanarJoint)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  World* world = SkelParser::readWorld(
                   DART_DATA_PATH"skel/test/planar_joint.skel");
  EXPECT_TRUE(world != NULL);

  Skeleton* skel1 = world->getSkeleton("skeleton1");
  EXPECT_TRUE(skel1 != NULL);

  BodyNode* body1 = skel1->getBodyNode("link1");
  BodyNode* body2 = skel1->getBodyNode("link2");
  BodyNode* body3 = skel1->getBodyNode("link3");
  BodyNode* body4 = skel1->getBodyNode("link4");
  EXPECT_TRUE(body1 != NULL);
  EXPECT_TRUE(body2 != NULL);
  EXPECT_TRUE(body3 != NULL);
  EXPECT_TRUE(body4 != NULL);

  PlanarJoint* planarJoint1
      = dynamic_cast<PlanarJoint*>(body1->getParentJoint());
  PlanarJoint* planarJoint2
      = dynamic_cast<PlanarJoint*>(body2->getParentJoint());
  PlanarJoint* planarJoint3
      = dynamic_cast<PlanarJoint*>(body3->getParentJoint());
  PlanarJoint* planarJoint4
      = dynamic_cast<PlanarJoint*>(body4->getParentJoint());
  EXPECT_TRUE(planarJoint1 != NULL);
  EXPECT_TRUE(planarJoint2 != NULL);
  EXPECT_TRUE(planarJoint3 != NULL);
  EXPECT_TRUE(planarJoint4 != NULL);

  EXPECT_EQ(planarJoint1->getPlaneType(), PlanarJoint::PT_XY);
  EXPECT_EQ(planarJoint2->getPlaneType(), PlanarJoint::PT_YZ);
  EXPECT_EQ(planarJoint3->getPlaneType(), PlanarJoint::PT_ZX);
  EXPECT_EQ(planarJoint4->getPlaneType(), PlanarJoint::PT_ARBITRARY);

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

  delete world;
}

//==============================================================================
TEST(SKEL_PARSER, JointActuatorType)
{
  World* world = SkelParser::readWorld(
                   DART_DATA_PATH"/skel/test/joint_actuator_type_test.skel");
  EXPECT_TRUE(world != NULL);

  Skeleton* skel1 = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel1 != NULL);

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
  EXPECT_EQ(joint4->getActuatorType(), Joint::VELOCITY);}

//==============================================================================
TEST(Parser, DofAttributes)
{
  World* world = SkelParser::readWorld(
                   DART_DATA_PATH"/skel/test/dof_attribute_test.skel");
  EXPECT_TRUE(world != NULL);

  Skeleton* skel1 = world->getSkeleton("skeleton 1");

  // Test for no dof elements being specified
  Joint* joint0 = skel1->getJoint("joint0");
  EXPECT_EQ(joint0->getDof(0)->getPositionLowerLimit(), -DART_DBL_INF);
  EXPECT_EQ(joint0->getDof(0)->getPositionUpperLimit(),  DART_DBL_INF);
  EXPECT_EQ(joint0->getDof(0)->getPosition(), 0);

  EXPECT_EQ(joint0->getDof(0)->getVelocityLowerLimit(), -DART_DBL_INF);
  EXPECT_EQ(joint0->getDof(0)->getVelocityUpperLimit(),  DART_DBL_INF);
  EXPECT_EQ(joint0->getDof(0)->getVelocity(), 0);

  EXPECT_EQ(joint0->getDof(0)->getName(), joint0->getName());

  // Test for only a dof name being changed
  Joint* joint1 = skel1->getJoint("joint1");
  EXPECT_EQ(joint1->getDof(0)->getPositionLowerLimit(), -1.57);
  EXPECT_EQ(joint1->getDof(0)->getPositionUpperLimit(),  1.57);
  EXPECT_EQ(joint1->getDof(0)->getName(), "customJoint1");

  // Test for when dof attributes (but not a name) are specified
  Joint* joint2 = skel1->getJoint("joint2");
  EXPECT_EQ(joint2->getDof(0)->getName(), joint2->getName());

  EXPECT_EQ(joint2->getDof(0)->getPositionLowerLimit(), -1);
  EXPECT_EQ(joint2->getDof(0)->getPositionUpperLimit(),  1);
  EXPECT_EQ(joint2->getDof(0)->getPosition(), 0);

  EXPECT_EQ(joint2->getDof(0)->getVelocityLowerLimit(), -2);
  EXPECT_EQ(joint2->getDof(0)->getVelocityUpperLimit(),  2);
  EXPECT_EQ(joint2->getDof(0)->getVelocity(), 0);

  EXPECT_EQ(joint2->getDof(0)->getAccelerationLowerLimit(), -3);
  EXPECT_EQ(joint2->getDof(0)->getAccelerationUpperLimit(),  3);
  EXPECT_EQ(joint2->getDof(0)->getAcceleration(), 0);

  EXPECT_EQ(joint2->getDof(0)->getForceLowerLimit(), -4);
  EXPECT_EQ(joint2->getDof(0)->getForceUpperLimit(),  4);
  EXPECT_EQ(joint2->getDof(0)->getForce(), 0);

  // Test for mixture of old method and new method
  // Note: If there is a conflict, the data given in the dof element will win
  Joint* joint3 = skel1->getJoint("joint3");
  EXPECT_EQ(joint3->getDof(0)->getName(), joint3->getName() + "_1");
  EXPECT_EQ(joint3->getDof(0)->getPositionLowerLimit(), -1);
  EXPECT_EQ(joint3->getDof(0)->getPositionUpperLimit(),  1);
  EXPECT_EQ(joint3->getDof(0)->getPosition(), 5);

  EXPECT_EQ(joint3->getDof(1)->getName(), joint3->getName() + "_2");
  EXPECT_EQ(joint3->getDof(1)->getPositionLowerLimit(), -2);
  EXPECT_EQ(joint3->getDof(1)->getPositionUpperLimit(),  2);
  EXPECT_EQ(joint3->getDof(1)->getPosition(), -5);

  // Test for only some of the DOFs being specified
  Joint* joint4 = skel1->getJoint("joint4");
  EXPECT_EQ(joint4->getDof(0)->getName(), "joint4_1");
  EXPECT_EQ(joint4->getDof(0)->getPositionLowerLimit(), -1);
  EXPECT_EQ(joint4->getDof(0)->getPositionUpperLimit(),  1);
  EXPECT_EQ(joint4->getDof(0)->getVelocityLowerLimit(), -10);
  EXPECT_EQ(joint4->getDof(0)->getVelocityUpperLimit(),  10);

  EXPECT_EQ(joint4->getDof(1)->getName(), joint4->getName() + "_y");

  EXPECT_EQ(joint4->getDof(2)->getName(), "joint4_3");
  EXPECT_EQ(joint4->getDof(2)->getPositionLowerLimit(), -2);
  EXPECT_EQ(joint4->getDof(2)->getPositionUpperLimit(),  2);
  EXPECT_EQ(joint4->getDof(2)->getVelocityLowerLimit(), -20);
  EXPECT_EQ(joint4->getDof(2)->getVelocityUpperLimit(),  20);
}

//==============================================================================
TEST(Parser, JointDynamicsElements)
{
  World* world
      = SkelParser::readWorld(
          DART_DATA_PATH"/skel/test/joint_dynamics_elements_test.skel");
  EXPECT_TRUE(world != NULL);

  Skeleton* skel1 = world->getSkeleton("skeleton 1");

  Joint* joint0 = skel1->getJoint("joint0");
  EXPECT_EQ(joint0->getDampingCoefficient(0), 1.0);
  EXPECT_EQ(joint0->getCoulombFriction   (0), 5.0);
  EXPECT_EQ(joint0->getRestPosition      (0), 0.1);
  EXPECT_EQ(joint0->getSpringStiffness   (0), 3.0);

  Joint* joint1 = skel1->getJoint("joint1");
  EXPECT_EQ(joint1->getDampingCoefficient(0), 1.0);
  EXPECT_EQ(joint1->getCoulombFriction   (0), 5.0);
  EXPECT_EQ(joint1->getRestPosition      (0), 0.1);
  EXPECT_EQ(joint1->getSpringStiffness   (0), 3.0);

  EXPECT_EQ(joint1->getDampingCoefficient(1), 2.0);
  EXPECT_EQ(joint1->getCoulombFriction   (1), 4.0);
  EXPECT_EQ(joint1->getRestPosition      (1), 0.2);
  EXPECT_EQ(joint1->getSpringStiffness   (1), 2.0);

  EXPECT_EQ(joint1->getDampingCoefficient(2), 3.0);
  EXPECT_EQ(joint1->getCoulombFriction   (2), 3.0);
  EXPECT_EQ(joint1->getRestPosition      (2), 0.3);
  EXPECT_EQ(joint1->getSpringStiffness   (2), 1.0);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
