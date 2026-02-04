/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "dart/constraint/coupler_constraint.hpp"
#include "dart/constraint/mimic_motor_constraint.hpp"
#include "dart/constraint/servo_motor_constraint.hpp"
#include "dart/dynamics/mimic_dof_properties.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;

namespace {

/// Creates a skeleton with two revolute joints for mimic testing
SkeletonPtr createTwoJointSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);

  // First body with revolute joint (reference joint)
  BodyNode::Properties bodyProps1;
  bodyProps1.mName = name + "_body1";
  bodyProps1.mInertia.setMass(1.0);

  RevoluteJoint::Properties jointProps1;
  jointProps1.mName = name + "_joint1";
  jointProps1.mAxis = Eigen::Vector3d::UnitZ();

  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps1, bodyProps1);

  // Second body with revolute joint (dependent joint)
  BodyNode::Properties bodyProps2;
  bodyProps2.mName = name + "_body2";
  bodyProps2.mInertia.setMass(1.0);

  RevoluteJoint::Properties jointProps2;
  jointProps2.mName = name + "_joint2";
  jointProps2.mAxis = Eigen::Vector3d::UnitZ();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(0.0, 0.0, 1.0));
  jointProps2.mT_ParentBodyToJoint = T;

  skel->createJointAndBodyNodePair<RevoluteJoint>(
      pair1.second, jointProps2, bodyProps2);

  return skel;
}

/// Creates a skeleton with a single revolute joint
SkeletonPtr createSingleJointSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);

  BodyNode::Properties bodyProps;
  bodyProps.mName = name + "_body";
  bodyProps.mInertia.setMass(1.0);

  RevoluteJoint::Properties jointProps;
  jointProps.mName = name + "_joint";
  jointProps.mAxis = Eigen::Vector3d::UnitZ();

  skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps, bodyProps);

  return skel;
}

} // namespace

//==============================================================================
// ServoMotorConstraint Tests
//==============================================================================

TEST(ServoMotorConstraint, StaticType)
{
  EXPECT_EQ(ServoMotorConstraint::getStaticType(), "ServoMotorConstraint");
}

//==============================================================================
TEST(ServoMotorConstraint, ConstructWithValidJoint)
{
  auto skel = createSingleJointSkeleton("test");
  auto joint = skel->getJoint(0);
  ASSERT_NE(joint, nullptr);

  auto constraint = std::make_shared<ServoMotorConstraint>(joint);

  EXPECT_EQ(constraint->getType(), ServoMotorConstraint::getStaticType());
}

//==============================================================================
TEST(ServoMotorConstraint, ConstraintForceMixingParameter)
{
  const double originalCfm = ServoMotorConstraint::getConstraintForceMixing();

  ServoMotorConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(ServoMotorConstraint::getConstraintForceMixing(), 1e-4);

  ServoMotorConstraint::setConstraintForceMixing(1e-6);
  EXPECT_DOUBLE_EQ(ServoMotorConstraint::getConstraintForceMixing(), 1e-6);

  // Restore original value
  ServoMotorConstraint::setConstraintForceMixing(originalCfm);
}

//==============================================================================
// MimicMotorConstraint Tests
//==============================================================================

TEST(MimicMotorConstraint, StaticType)
{
  EXPECT_EQ(MimicMotorConstraint::getStaticType(), "MimicMotorConstraint");
}

//==============================================================================
TEST(MimicMotorConstraint, ConstructWithValidJoint)
{
  auto skel = createTwoJointSkeleton("test");
  auto refJoint = skel->getJoint(0);
  auto depJoint = skel->getJoint(1);
  ASSERT_NE(refJoint, nullptr);
  ASSERT_NE(depJoint, nullptr);

  MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = refJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 1.0;
  mimicProps.mOffset = 0.0;
  mimicProps.mConstraintType = MimicConstraintType::Motor;

  std::array<MimicDofProperties, 1> propsArray = {mimicProps};

  auto constraint
      = std::make_shared<MimicMotorConstraint>(depJoint, propsArray);

  EXPECT_EQ(constraint->getType(), MimicMotorConstraint::getStaticType());
}

//==============================================================================
TEST(MimicMotorConstraint, ConstraintForceMixingParameter)
{
  const double originalCfm = MimicMotorConstraint::getConstraintForceMixing();

  MimicMotorConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(MimicMotorConstraint::getConstraintForceMixing(), 1e-4);

  MimicMotorConstraint::setConstraintForceMixing(1e-6);
  EXPECT_DOUBLE_EQ(MimicMotorConstraint::getConstraintForceMixing(), 1e-6);

  // Restore original value
  MimicMotorConstraint::setConstraintForceMixing(originalCfm);
}

//==============================================================================
TEST(MimicMotorConstraint, ErrorReductionParameter)
{
  const double originalErp = MimicMotorConstraint::getErrorReductionParameter();

  MimicMotorConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(MimicMotorConstraint::getErrorReductionParameter(), 0.5);

  MimicMotorConstraint::setErrorReductionParameter(0.2);
  EXPECT_DOUBLE_EQ(MimicMotorConstraint::getErrorReductionParameter(), 0.2);

  // Restore original value
  MimicMotorConstraint::setErrorReductionParameter(originalErp);
}

//==============================================================================
TEST(MimicMotorConstraint, MimicPropertiesMultiplier)
{
  auto skel = createTwoJointSkeleton("test");
  auto refJoint = skel->getJoint(0);
  auto depJoint = skel->getJoint(1);

  // Test with different multipliers
  MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = refJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 2.0; // 2:1 gear ratio
  mimicProps.mOffset = 0.0;
  mimicProps.mConstraintType = MimicConstraintType::Motor;

  std::array<MimicDofProperties, 1> propsArray = {mimicProps};

  auto constraint
      = std::make_shared<MimicMotorConstraint>(depJoint, propsArray);
  EXPECT_EQ(constraint->getType(), MimicMotorConstraint::getStaticType());
}

//==============================================================================
TEST(MimicMotorConstraint, MimicPropertiesOffset)
{
  auto skel = createTwoJointSkeleton("test");
  auto refJoint = skel->getJoint(0);
  auto depJoint = skel->getJoint(1);

  // Test with offset
  MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = refJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 1.0;
  mimicProps.mOffset = 0.5; // 0.5 radian offset
  mimicProps.mConstraintType = MimicConstraintType::Motor;

  std::array<MimicDofProperties, 1> propsArray = {mimicProps};

  auto constraint
      = std::make_shared<MimicMotorConstraint>(depJoint, propsArray);
  EXPECT_EQ(constraint->getType(), MimicMotorConstraint::getStaticType());
}

//==============================================================================
TEST(MimicMotorConstraint, WorldStepTracksMultiplierAndOffset)
{
  auto world = simulation::World::create();
  ASSERT_NE(world, nullptr);
  world->setTimeStep(1e-3);

  auto skel = createTwoJointSkeleton("mimic_world");
  auto* refJoint = skel->getJoint(0);
  auto* depJoint = skel->getJoint(1);
  ASSERT_NE(refJoint, nullptr);
  ASSERT_NE(depJoint, nullptr);

  refJoint->setActuatorType(Joint::LOCKED);
  refJoint->setPosition(0, 0.2);

  depJoint->setActuatorType(Joint::MIMIC);
  depJoint->setMimicConstraintType(MimicConstraintType::Motor);
  depJoint->setMimicJoint(refJoint, 2.0, 0.1);
  depJoint->setPosition(0, 0.0);

  world->addSkeleton(skel);

  for (int i = 0; i < 30; ++i) {
    world->step();
  }
  EXPECT_GT(depJoint->getPosition(0), 0.0);

  refJoint->setPosition(0, -0.2);
  depJoint->setPosition(0, 0.3);
  depJoint->setMimicJoint(refJoint, -1.5, -0.05);
  refJoint->setVelocity(0, 0.0);
  depJoint->setVelocity(0, 0.0);

  const double resetPosition = depJoint->getPosition(0);
  for (int i = 0; i < 30; ++i) {
    world->step();
  }
  EXPECT_LT(depJoint->getPosition(0), resetPosition);
}

//==============================================================================
// CouplerConstraint Tests
//==============================================================================

TEST(CouplerConstraint, StaticType)
{
  EXPECT_EQ(CouplerConstraint::getStaticType(), "CouplerConstraint");
}

//==============================================================================
TEST(CouplerConstraint, ConstructWithValidJoint)
{
  auto skel = createTwoJointSkeleton("test");
  auto refJoint = skel->getJoint(0);
  auto depJoint = skel->getJoint(1);
  ASSERT_NE(refJoint, nullptr);
  ASSERT_NE(depJoint, nullptr);

  MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = refJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 1.0;
  mimicProps.mOffset = 0.0;
  mimicProps.mConstraintType = MimicConstraintType::Coupler;

  std::array<MimicDofProperties, 1> propsArray = {mimicProps};

  auto constraint = std::make_shared<CouplerConstraint>(depJoint, propsArray);

  EXPECT_EQ(constraint->getType(), CouplerConstraint::getStaticType());
}

//==============================================================================
TEST(CouplerConstraint, ConstraintForceMixingParameter)
{
  const double originalCfm = CouplerConstraint::getConstraintForceMixing();

  CouplerConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(CouplerConstraint::getConstraintForceMixing(), 1e-4);

  CouplerConstraint::setConstraintForceMixing(1e-6);
  EXPECT_DOUBLE_EQ(CouplerConstraint::getConstraintForceMixing(), 1e-6);

  // Restore original value
  CouplerConstraint::setConstraintForceMixing(originalCfm);
}

//==============================================================================
TEST(CouplerConstraint, DifferentFromMimicMotor)
{
  // Verify that CouplerConstraint and MimicMotorConstraint are distinct types
  EXPECT_NE(
      CouplerConstraint::getStaticType(),
      MimicMotorConstraint::getStaticType());
}

//==============================================================================
// MimicDofProperties Tests
//==============================================================================

TEST(MimicDofProperties, DefaultValuesWithNullReference)
{
  // Explicitly initialize all fields including mReferenceJoint to avoid UB
  MimicDofProperties props;
  props.mReferenceJoint = nullptr;
  props.mReferenceDofIndex = 0;
  props.mMultiplier = 1.0;
  props.mOffset = 0.0;
  props.mConstraintType = MimicConstraintType::Motor;

  EXPECT_EQ(props.mReferenceJoint, nullptr);
  EXPECT_EQ(props.mReferenceDofIndex, 0u);
  EXPECT_DOUBLE_EQ(props.mMultiplier, 1.0);
  EXPECT_DOUBLE_EQ(props.mOffset, 0.0);
  EXPECT_EQ(props.mConstraintType, MimicConstraintType::Motor);
}

//==============================================================================
TEST(MimicDofProperties, CustomValues)
{
  auto skel = createSingleJointSkeleton("test");
  auto joint = skel->getJoint(0);

  MimicDofProperties props;
  props.mReferenceJoint = joint;
  props.mReferenceDofIndex = 0;
  props.mMultiplier = 2.5;
  props.mOffset = -0.3;
  props.mConstraintType = MimicConstraintType::Coupler;

  EXPECT_EQ(props.mReferenceJoint, joint);
  EXPECT_EQ(props.mReferenceDofIndex, 0u);
  EXPECT_DOUBLE_EQ(props.mMultiplier, 2.5);
  EXPECT_DOUBLE_EQ(props.mOffset, -0.3);
  EXPECT_EQ(props.mConstraintType, MimicConstraintType::Coupler);
}
