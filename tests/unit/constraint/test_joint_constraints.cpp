/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "helpers/dynamics_helpers.hpp"

#include "dart/constraint/ball_joint_constraint.hpp"
#include "dart/constraint/contact_constraint.hpp"
#include "dart/constraint/dynamic_joint_constraint.hpp"
#include "dart/constraint/joint_constraint.hpp"
#include "dart/constraint/joint_coulomb_friction_constraint.hpp"
#include "dart/constraint/joint_limit_constraint.hpp"
#include "dart/constraint/servo_motor_constraint.hpp"
#include "dart/constraint/soft_contact_constraint.hpp"
#include "dart/constraint/weld_joint_constraint.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;

namespace {

SkeletonPtr createSingleBodySkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);

  BodyNode::Properties bodyProps;
  bodyProps.mName = name + "_body";
  bodyProps.mInertia.setMass(1.0);

  FreeJoint::Properties jointProps;
  jointProps.mName = name + "_joint";

  skel->createJointAndBodyNodePair<FreeJoint>(nullptr, jointProps, bodyProps);

  return skel;
}

// Restores a class-wide constraint parameter on destruction so the clamping
// checks below cannot leak state into the other tests in this binary.
class ScopedParameter
{
public:
  ScopedParameter(void (*set)(double), double (*get)())
    : mSet(set), mPrevious(get())
  {
  }

  ~ScopedParameter()
  {
    mSet(mPrevious);
  }

private:
  void (*mSet)(double);
  double mPrevious;
};

// Passes an out-of-range value to a class-wide constraint parameter setter and
// expects the getter to report the bound that the setter's warning names.
void expectClampsToBound(
    void (*set)(double), double (*get)(), double invalid, double bound)
{
  const ScopedParameter guard(set, get);
  set(invalid);
  EXPECT_DOUBLE_EQ(bound, get()) << "argument " << invalid;
}

template <typename Constraint>
void expectParametersClampToBounds(const char* name)
{
  SCOPED_TRACE(name);
  expectClampsToBound(
      &Constraint::setErrorAllowance,
      &Constraint::getErrorAllowance,
      -0.25,
      0.0);
  expectClampsToBound(
      &Constraint::setErrorReductionParameter,
      &Constraint::getErrorReductionParameter,
      -0.25,
      0.0);
  expectClampsToBound(
      &Constraint::setErrorReductionParameter,
      &Constraint::getErrorReductionParameter,
      1.25,
      1.0);
  expectClampsToBound(
      &Constraint::setMaxErrorReductionVelocity,
      &Constraint::getMaxErrorReductionVelocity,
      -0.25,
      0.0);
  expectClampsToBound(
      &Constraint::setConstraintForceMixing,
      &Constraint::getConstraintForceMixing,
      0.0,
      1e-9);
}

} // namespace

//==============================================================================
// BallJointConstraint Tests
//==============================================================================

TEST(BallJointConstraint, ConstructWithValidSingleBody)
{
  auto skel = createSingleBodySkeleton("test");
  auto body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  Eigen::Vector3d jointPos = Eigen::Vector3d::Zero();
  auto constraint = std::make_shared<BallJointConstraint>(body, jointPos);

  EXPECT_EQ(constraint->getType(), BallJointConstraint::getStaticType());
  EXPECT_EQ(constraint->getBodyNode1(), body);
  EXPECT_EQ(constraint->getBodyNode2(), nullptr);
}

//==============================================================================
TEST(BallJointConstraint, ConstructWithTwoValidBodies)
{
  auto skel1 = createSingleBodySkeleton("skel1");
  auto skel2 = createSingleBodySkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);
  ASSERT_NE(body1, nullptr);
  ASSERT_NE(body2, nullptr);

  Eigen::Vector3d jointPos = Eigen::Vector3d::Zero();
  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  EXPECT_EQ(constraint->getType(), BallJointConstraint::getStaticType());
  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

//==============================================================================
TEST(BallJointConstraint, StaticType)
{
  EXPECT_EQ(BallJointConstraint::getStaticType(), "BallJointConstraint");
}

//==============================================================================
// WeldJointConstraint Tests
//==============================================================================

TEST(WeldJointConstraint, ConstructWithValidSingleBody)
{
  auto skel = createSingleBodySkeleton("test");
  auto body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  auto constraint = std::make_shared<WeldJointConstraint>(body);

  EXPECT_EQ(constraint->getType(), WeldJointConstraint::getStaticType());
  EXPECT_EQ(constraint->getBodyNode1(), body);
  EXPECT_EQ(constraint->getBodyNode2(), nullptr);
}

//==============================================================================
TEST(WeldJointConstraint, ConstructWithTwoValidBodies)
{
  auto skel1 = createSingleBodySkeleton("skel1");
  auto skel2 = createSingleBodySkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);
  ASSERT_NE(body1, nullptr);
  ASSERT_NE(body2, nullptr);

  auto constraint = std::make_shared<WeldJointConstraint>(body1, body2);

  EXPECT_EQ(constraint->getType(), WeldJointConstraint::getStaticType());
  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

//==============================================================================
TEST(WeldJointConstraint, RelativeTransform)
{
  auto skel = createSingleBodySkeleton("test");
  auto body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  auto constraint = std::make_shared<WeldJointConstraint>(body);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  constraint->setRelativeTransform(tf);

  const auto& retrieved = constraint->getRelativeTransform();
  EXPECT_TRUE(tf.isApprox(retrieved));
}

//==============================================================================
TEST(WeldJointConstraint, StaticType)
{
  EXPECT_EQ(WeldJointConstraint::getStaticType(), "WeldJointConstraint");
}

//==============================================================================
// DynamicJointConstraint Base Class Tests
//==============================================================================

TEST(DynamicJointConstraint, GetBodyNodes)
{
  auto skel1 = createSingleBodySkeleton("skel1");
  auto skel2 = createSingleBodySkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  Eigen::Vector3d jointPos = Eigen::Vector3d::Zero();
  auto ballConstraint = std::make_shared<BallJointConstraint>(body1, jointPos);
  EXPECT_EQ(ballConstraint->getBodyNode1(), body1);
  EXPECT_EQ(ballConstraint->getBodyNode2(), nullptr);

  auto weldConstraint = std::make_shared<WeldJointConstraint>(body1, body2);
  EXPECT_EQ(weldConstraint->getBodyNode1(), body1);
  EXPECT_EQ(weldConstraint->getBodyNode2(), body2);
}

//==============================================================================
// Constraint Parameter Tests
//==============================================================================

// Regression test for https://github.com/dartsim/dart/issues/3501: the setters
// warned that an out-of-range argument "is set to" a bound, but then stored the
// argument verbatim, so the getter reported the value the warning rejected.
TEST(ConstraintParameters, SettersStoreTheBoundNamedByTheWarning)
{
  expectParametersClampToBounds<DynamicJointConstraint>(
      "DynamicJointConstraint");
  expectParametersClampToBounds<JointConstraint>("JointConstraint");
  expectParametersClampToBounds<JointLimitConstraint>("JointLimitConstraint");
  expectParametersClampToBounds<SoftContactConstraint>("SoftContactConstraint");

  // ContactConstraint already clamps through std::max/std::clamp. It is pinned
  // here so the whole family keeps one contract.
  expectParametersClampToBounds<ContactConstraint>("ContactConstraint");

  {
    SCOPED_TRACE("JointCoulombFrictionConstraint");
    expectClampsToBound(
        &JointCoulombFrictionConstraint::setConstraintForceMixing,
        &JointCoulombFrictionConstraint::getConstraintForceMixing,
        0.0,
        1e-9);
  }

  {
    SCOPED_TRACE("ServoMotorConstraint");
    expectClampsToBound(
        &ServoMotorConstraint::setConstraintForceMixing,
        &ServoMotorConstraint::getConstraintForceMixing,
        0.0,
        1e-9);
  }
}
