/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/euler_joint.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/translational_joint.hpp>
#include <dart/dynamics/universal_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

namespace {

template <typename JointType>
SkeletonPtr createSkeletonWithJoint(const std::string& name)
{
  auto skel = Skeleton::create(name);

  BodyNode::Properties bodyProps;
  bodyProps.mName = name + "_body";
  bodyProps.mInertia.setMass(1.0);

  typename JointType::Properties jointProps;
  jointProps.mName = name + "_joint";

  skel->createJointAndBodyNodePair<JointType>(nullptr, jointProps, bodyProps);

  return skel;
}

} // namespace

//==============================================================================
TEST(FreeJointTest, StaticType)
{
  EXPECT_EQ(FreeJoint::getStaticType(), "FreeJoint");
}

//==============================================================================
TEST(FreeJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 6u);
}

//==============================================================================
TEST(FreeJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), FreeJoint::getStaticType());
}

//==============================================================================
TEST(FreeJointTest, ConvertToPositions)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  Eigen::Vector6d positions = FreeJoint::convertToPositions(tf);

  EXPECT_EQ(positions.size(), 6);
}

//==============================================================================
TEST(FreeJointTest, ConvertToTransform)
{
  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions.tail<3>() = Eigen::Vector3d(1.0, 2.0, 3.0);

  Eigen::Isometry3d tf = FreeJoint::convertToTransform(positions);

  EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(BallJointTest, StaticType)
{
  EXPECT_EQ(BallJoint::getStaticType(), "BallJoint");
}

//==============================================================================
TEST(BallJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 3u);
}

//==============================================================================
TEST(BallJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), BallJoint::getStaticType());
}

//==============================================================================
TEST(BallJointTest, IsCyclic)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball");
  auto joint = skel->getJoint(0);

  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    EXPECT_TRUE(joint->isCyclic(i));
  }
}

//==============================================================================
TEST(RevoluteJointTest, StaticType)
{
  EXPECT_EQ(RevoluteJoint::getStaticType(), "RevoluteJoint");
}

//==============================================================================
TEST(RevoluteJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 1u);
}

//==============================================================================
TEST(RevoluteJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), RevoluteJoint::getStaticType());
}

//==============================================================================
TEST(RevoluteJointTest, IsCyclic)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  EXPECT_TRUE(joint->isCyclic(0));
}

//==============================================================================
TEST(PrismaticJointTest, StaticType)
{
  EXPECT_EQ(PrismaticJoint::getStaticType(), "PrismaticJoint");
}

//==============================================================================
TEST(PrismaticJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<PrismaticJoint>("prismatic");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 1u);
}

//==============================================================================
TEST(PrismaticJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<PrismaticJoint>("prismatic");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), PrismaticJoint::getStaticType());
}

//==============================================================================
TEST(PrismaticJointTest, IsNotCyclic)
{
  auto skel = createSkeletonWithJoint<PrismaticJoint>("prismatic");
  auto joint = skel->getJoint(0);

  EXPECT_FALSE(joint->isCyclic(0));
}

//==============================================================================
TEST(UniversalJointTest, StaticType)
{
  EXPECT_EQ(UniversalJoint::getStaticType(), "UniversalJoint");
}

//==============================================================================
TEST(UniversalJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<UniversalJoint>("universal");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 2u);
}

//==============================================================================
TEST(UniversalJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<UniversalJoint>("universal");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), UniversalJoint::getStaticType());
}

//==============================================================================
TEST(EulerJointTest, StaticType)
{
  EXPECT_EQ(EulerJoint::getStaticType(), "EulerJoint");
}

//==============================================================================
TEST(EulerJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<EulerJoint>("euler");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 3u);
}

//==============================================================================
TEST(EulerJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<EulerJoint>("euler");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), EulerJoint::getStaticType());
}

//==============================================================================
TEST(TranslationalJointTest, StaticType)
{
  EXPECT_EQ(TranslationalJoint::getStaticType(), "TranslationalJoint");
}

//==============================================================================
TEST(TranslationalJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<TranslationalJoint>("translational");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 3u);
}

//==============================================================================
TEST(TranslationalJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<TranslationalJoint>("translational");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), TranslationalJoint::getStaticType());
}

//==============================================================================
TEST(PlanarJointTest, StaticType)
{
  EXPECT_EQ(PlanarJoint::getStaticType(), "PlanarJoint");
}

//==============================================================================
TEST(PlanarJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<PlanarJoint>("planar");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 3u);
}

//==============================================================================
TEST(PlanarJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<PlanarJoint>("planar");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), PlanarJoint::getStaticType());
}

//==============================================================================
TEST(WeldJointTest, StaticType)
{
  EXPECT_EQ(WeldJoint::getStaticType(), "WeldJoint");
}

//==============================================================================
TEST(WeldJointTest, NumDofs)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("weld");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getNumDofs(), 0u);
}

//==============================================================================
TEST(WeldJointTest, GetType)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("weld");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getType(), WeldJoint::getStaticType());
}

//==============================================================================
TEST(JointTest, SetPosition)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setPosition(0, 1.5);
  EXPECT_DOUBLE_EQ(joint->getPosition(0), 1.5);
}

//==============================================================================
TEST(JointTest, SetVelocity)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setVelocity(0, 2.0);
  EXPECT_DOUBLE_EQ(joint->getVelocity(0), 2.0);
}

//==============================================================================
TEST(JointTest, SetAcceleration)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setAcceleration(0, 0.5);
  EXPECT_DOUBLE_EQ(joint->getAcceleration(0), 0.5);
}

//==============================================================================
TEST(JointTest, SetForce)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setForce(0, 10.0);
  EXPECT_DOUBLE_EQ(joint->getForce(0), 10.0);
}

//==============================================================================
TEST(JointTest, PositionLimits)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setPositionLowerLimit(0, -1.0);
  joint->setPositionUpperLimit(0, 1.0);

  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -1.0);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 1.0);
}

//==============================================================================
TEST(JointTest, VelocityLimits)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setVelocityLowerLimit(0, -5.0);
  joint->setVelocityUpperLimit(0, 5.0);

  EXPECT_DOUBLE_EQ(joint->getVelocityLowerLimit(0), -5.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityUpperLimit(0), 5.0);
}

//==============================================================================
TEST(JointTest, DampingCoefficient)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setDampingCoefficient(0, 0.1);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 0.1);
}

//==============================================================================
TEST(JointTest, SpringStiffness)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setSpringStiffness(0, 100.0);
  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 100.0);
}

//==============================================================================
TEST(JointTest, RestPosition)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setRestPosition(0, 0.5);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.5);
}

//==============================================================================
TEST(JointTest, TransformFromParentBodyNode)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  joint->setTransformFromParentBodyNode(tf);

  EXPECT_TRUE(joint->getTransformFromParentBodyNode().isApprox(tf));
}

//==============================================================================
TEST(JointTest, TransformFromChildBodyNode)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.5);

  joint->setTransformFromChildBodyNode(tf);

  EXPECT_TRUE(joint->getTransformFromChildBodyNode().isApprox(tf));
}
