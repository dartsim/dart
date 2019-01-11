/*
 * Copyright (c) 2011-2019, The DART development contributors
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
#include "TestHelpers.hpp"

#include "dart/dart.hpp"
#include "dart/io/io.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace io;

//==============================================================================
TEST(SkelParser, DataStructure)
{
  bool              valBool       = true;
  int               valInt        = -3;
  unsigned int      valUInt       = 1;
  float             valFloat      = -3.140f;
  double            valDouble     = 1.4576640;
  char              valChar       = 'd';
  Eigen::Vector2d   valVector2d   = Eigen::Vector2d::Random();
  Eigen::Vector3d   valVector3d   = Eigen::Vector3d::Random();
  Eigen::Vector3i   valVector3i   = Eigen::Vector3i::Random();
  Eigen::Vector6d   valVector6d   = Eigen::Vector6d::Random();
  Eigen::VectorXd   valVectorXd   = Eigen::VectorXd::Random(10);
  Eigen::Isometry3d valIsometry3d = Eigen::Isometry3d::Identity();

  std::string strBool       = toString(valBool);
  std::string strInt        = toString(valInt);
  std::string strUInt       = toString(valUInt);
  std::string strFloat      = toString(valFloat);
  std::string strDouble     = toString(valDouble);
  std::string strChar       = toString(valChar);
  std::string strVector2d   = toString(valVector2d);
  std::string strVector3d   = toString(valVector3d);
  std::string strVector3i   = toString(valVector3i);
  std::string strVector6d   = toString(valVector6d);
  std::string strVectorXd   = toString(valVectorXd);
  std::string strIsometry3d = toString(valIsometry3d);

  EXPECT_EQ(valBool,   toBool(strBool));
  EXPECT_EQ(valInt,    toInt(strInt));
  EXPECT_EQ(valUInt,   toUInt(strUInt));
  EXPECT_EQ(valFloat,  toFloat(strFloat));
  EXPECT_EQ(valDouble, toDouble(strDouble));
  EXPECT_EQ(valChar,   toChar(strChar));
  EXPECT_TRUE(equals(valVector2d, toVector2d(strVector2d)));
  EXPECT_TRUE(equals(valVector3d, toVector3d(strVector3d)));
  EXPECT_EQ(valVector3i, toVector3i(strVector3i));
  EXPECT_TRUE(equals(valVector6d, toVector6d(strVector6d)));
  EXPECT_TRUE(equals(valVectorXd, toVectorXd(strVectorXd)));
  EXPECT_TRUE(equals(valIsometry3d.matrix(),
                     toIsometry3d(strIsometry3d).matrix()));
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
  EXPECT_EQ((int)world->getNumSkeletons(), 0);

  EXPECT_EQ(world->getTime(), 0);
  world->step();
  EXPECT_EQ(world->getTime(), world->getTimeStep());
}

//==============================================================================
TEST(SkelParser, SinglePendulum)
{
  WorldPtr world = SkelParser::readWorld(
        "dart://sample/skel/test/single_pendulum.skel");

  EXPECT_TRUE(world != nullptr);
  EXPECT_EQ(world->getTimeStep(), 0.001);
  EXPECT_EQ(world->getGravity()(0), 0);
  EXPECT_EQ(world->getGravity()(1), -9.81);
  EXPECT_EQ(world->getGravity()(2), 0);
  EXPECT_EQ(static_cast<int>(world->getNumSkeletons()), 1);

  SkeletonPtr skel1 = world->getSkeleton("single_pendulum");

  EXPECT_EQ(static_cast<int>(skel1->getNumBodyNodes()), 1);

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
  EXPECT_EQ(static_cast<int>(world->getNumSkeletons()), 1);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");

  EXPECT_EQ(static_cast<int>(skel1->getNumBodyNodes()), 10);

  world->step();
}

//==============================================================================
TEST(SkelParser, VariousShapes)
{
  // Check if the parser can load various shapes without any problems and the
  // world can simulate it successfully.

  WorldPtr world = SkelParser::readWorld(
        "dart://sample/skel/test/test_shapes.skel");

  for (auto i = 0u; i < 100; ++i)
    world->step();
}

//==============================================================================
TEST(SkelParser, RigidAndSoftBodies)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/test_articulated_bodies.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel1 != nullptr);
  EXPECT_EQ(static_cast<int>(skel1->getNumBodyNodes()), 2);
  EXPECT_EQ(static_cast<int>(skel1->getNumRigidBodyNodes()), 1);
  EXPECT_EQ(static_cast<int>(skel1->getNumSoftBodyNodes()), 1);

  SoftBodyNode* sbn = skel1->getSoftBodyNode(0);
  EXPECT_TRUE(static_cast<int>(sbn->getNumPointMasses()) > 0);

  world->step();
}

//==============================================================================
TEST(SkelParser, PlanarJoint)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/planar_joint.skel");
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
TEST(SKEL_PARSER, JointActuatorType)
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
  EXPECT_EQ(joint4->getActuatorType(), Joint::VELOCITY);}

//==============================================================================
TEST(SkelParser, DofAttributes)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/dof_attribute_test.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");

  // Test for no dof elements being specified
  Joint* joint0 = skel1->getJoint("joint0");
  EXPECT_EQ(joint0->getDof(0)->getPositionLowerLimit(), -constantsd::inf());
  EXPECT_EQ(joint0->getDof(0)->getPositionUpperLimit(),  constantsd::inf());
  EXPECT_EQ(joint0->getDof(0)->getPosition(), 0);

  EXPECT_EQ(joint0->getDof(0)->getVelocityLowerLimit(), -constantsd::inf());
  EXPECT_EQ(joint0->getDof(0)->getVelocityUpperLimit(),  constantsd::inf());
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
TEST(SkelParser, JointDynamicsElements)
{
  WorldPtr world
      = SkelParser::readWorld(
        "dart://sample/skel/test/joint_dynamics_elements_test.skel");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");

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
  auto multiSphereShape = std::static_pointer_cast<MultiSphereConvexHullShape>(shape);
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
