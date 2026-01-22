/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "helpers/dynamics_helpers.hpp"

#include "dart/constraint/ball_joint_constraint.hpp"
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
