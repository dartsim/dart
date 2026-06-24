/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

TEST(Joints, NonFiniteTransformFromParentBodyNodeRejected)
{
  auto skel = Skeleton::create("test");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;

  Eigen::Isometry3d goodTf = Eigen::Isometry3d::Identity();
  goodTf.translation() = Eigen::Vector3d(1, 2, 3);
  joint->setTransformFromParentBodyNode(goodTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));

  Eigen::Isometry3d nanTf = Eigen::Isometry3d::Identity();
  nanTf.translation()[0] = std::numeric_limits<double>::quiet_NaN();
  joint->setTransformFromParentBodyNode(nanTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));

  Eigen::Isometry3d infTf = Eigen::Isometry3d::Identity();
  infTf.translation()[0] = std::numeric_limits<double>::infinity();
  joint->setTransformFromParentBodyNode(infTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));

  Eigen::Isometry3d badRotTf = Eigen::Isometry3d::Identity();
  badRotTf.linear() << 2, 0, 0, 0, 2, 0, 0, 0, 2;
  joint->setTransformFromParentBodyNode(badRotTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));
}

TEST(Joints, NonFiniteTransformFromChildBodyNodeRejected)
{
  auto skel = Skeleton::create("test");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;

  Eigen::Isometry3d nanTf = Eigen::Isometry3d::Identity();
  nanTf.translation()[0] = std::numeric_limits<double>::quiet_NaN();
  joint->setTransformFromChildBodyNode(nanTf);
  EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(
      joint->getTransformFromChildBodyNode(), 1e-10));
}

TEST(Joints, SimulationSurvivesOverflowTransforms)
{
  auto skel = Skeleton::create("overflow");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;
  body->setMass(1.0);

  Eigen::Isometry3d extremeTf = Eigen::Isometry3d::Identity();
  extremeTf.translation() = Eigen::Vector3d(1e308, 1e308, 1e308);
  pair.first->setTransformFromParentBodyNode(extremeTf);

  auto world = World::create();
  world->addSkeleton(skel);

  for (int i = 0; i < 10; ++i) {
    EXPECT_NO_THROW(world->step());
  }
}

TEST(Joints, BulkPassiveForceSetters)
{
  auto skel = Skeleton::create("test");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  joint->setRestPositions(Eigen::VectorXd::Constant(joint->getNumDofs(), 0.25));
  EXPECT_TRUE(joint->getRestPositions().isConstant(0.25));

  joint->setDampingCoefficients(
      Eigen::VectorXd::Constant(joint->getNumDofs(), 0.5));
  EXPECT_TRUE(joint->getDampingCoefficients().isConstant(0.5));

  EXPECT_FALSE(joint->hasCoulombFriction());
  joint->setFrictions(Eigen::VectorXd::Constant(joint->getNumDofs(), 0.75));
  EXPECT_TRUE(joint->getFrictions().isConstant(0.75));
  EXPECT_TRUE(joint->hasCoulombFriction());

  joint->setFrictions(Eigen::VectorXd::Zero(joint->getNumDofs()));
  EXPECT_TRUE(joint->getFrictions().isZero());
  EXPECT_FALSE(joint->hasCoulombFriction());
}

TEST(Joints, TranslationalJoint2DCopyPointerUsesSource)
{
  auto skel = Skeleton::create("translational_2d_copy");
  auto [joint, body] = skel->createJointAndBodyNodePair<TranslationalJoint2D>();
  (void)body;

  joint->setXYPlane(false);
  joint->copy(static_cast<const TranslationalJoint2D*>(nullptr));
  joint->copy(joint);

  auto [otherJoint, otherBody]
      = skel->createJointAndBodyNodePair<TranslationalJoint2D>();
  (void)otherBody;
  otherJoint->setYZPlane(false);

  joint->copy(otherJoint);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::YZ);
}

TEST(Joints, UniversalJointCopyPointerUsesSource)
{
  auto skel = Skeleton::create("universal_copy");
  auto [joint, body] = skel->createJointAndBodyNodePair<UniversalJoint>();
  (void)body;

  UniversalJoint::Properties props;
  props.mAxis[0] = Eigen::Vector3d::UnitX();
  props.mAxis[1] = Eigen::Vector3d::UnitZ();
  joint->setProperties(props);

  joint->copy(static_cast<const UniversalJoint*>(nullptr));
  joint->copy(joint);

  auto [otherJoint, otherBody]
      = skel->createJointAndBodyNodePair<UniversalJoint>();
  (void)otherBody;
  otherJoint->setAxis1(Eigen::Vector3d::UnitY());

  joint->copy(otherJoint);
  EXPECT_TRUE(joint->getAxis1().isApprox(Eigen::Vector3d::UnitY()));
}

TEST(Joints, PerDofActuatorOverridesStayOrderedAndCompact)
{
  auto skel = Skeleton::create("per_dof_actuator_order");
  auto [joint, body] = skel->createJointAndBodyNodePair<BallJoint>();
  (void)body;

  joint->setActuatorType(Joint::SERVO);
  joint->setActuatorType(2, Joint::MIMIC);
  joint->setActuatorType(1, Joint::MIMIC);

  const std::vector<Joint::ActuatorType> expectedWithOverrides{
      Joint::SERVO, Joint::MIMIC, Joint::MIMIC};
  EXPECT_EQ(joint->getActuatorTypes(), expectedWithOverrides);
  EXPECT_EQ(joint->getActuatorType(0), Joint::SERVO);
  EXPECT_EQ(joint->getActuatorType(1), Joint::MIMIC);
  EXPECT_EQ(joint->getActuatorType(2), Joint::MIMIC);

  const auto& properties = joint->getJointProperties();
  ASSERT_EQ(properties.mActuatorTypes.size(), 2u);
  EXPECT_EQ(properties.mActuatorTypes[0].first, 1u);
  EXPECT_EQ(properties.mActuatorTypes[0].second, Joint::MIMIC);
  EXPECT_EQ(properties.mActuatorTypes[1].first, 2u);
  EXPECT_EQ(properties.mActuatorTypes[1].second, Joint::MIMIC);

  joint->setActuatorType(2, Joint::MIMIC);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes.size(), 2u);

  joint->setActuatorType(1, Joint::SERVO);
  const std::vector<Joint::ActuatorType> expectedAfterErase{
      Joint::SERVO, Joint::SERVO, Joint::MIMIC};
  EXPECT_EQ(joint->getActuatorTypes(), expectedAfterErase);
  ASSERT_EQ(joint->getJointProperties().mActuatorTypes.size(), 1u);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes[0].first, 2u);

  joint->setActuatorType(2, Joint::SERVO);
  EXPECT_TRUE(joint->getJointProperties().mActuatorTypes.empty());
}

TEST(Joints, BulkActuatorTypesRoundTripThroughJointProperties)
{
  auto skel = Skeleton::create("bulk_actuator_types");
  auto [joint, body] = skel->createJointAndBodyNodePair<BallJoint>();
  (void)body;

  const std::vector<Joint::ActuatorType> actuatorTypes{
      Joint::SERVO, Joint::MIMIC, Joint::SERVO};
  joint->setActuatorTypes(actuatorTypes);

  EXPECT_EQ(joint->getActuatorType(), Joint::SERVO);
  EXPECT_EQ(joint->getActuatorTypes(), actuatorTypes);
  ASSERT_EQ(joint->getJointProperties().mActuatorTypes.size(), 1u);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes[0].first, 1u);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes[0].second, Joint::MIMIC);

  auto [copiedJoint, copiedBody]
      = skel->createJointAndBodyNodePair<BallJoint>();
  (void)copiedBody;
  Joint* copiedBase = copiedJoint;
  copiedBase->setProperties(joint->getJointProperties());
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);

  copiedJoint->setActuatorTypes(actuatorTypes);
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);

  copiedJoint->setActuatorTypes(
      {Joint::VELOCITY, Joint::MIMIC, Joint::VELOCITY});
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);

  copiedJoint->setActuatorTypes({Joint::SERVO, Joint::PASSIVE, Joint::SERVO});
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);
}
