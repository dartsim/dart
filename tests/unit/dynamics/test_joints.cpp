/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/all.hpp>

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

class TestNode final : public Node
{
public:
  TestNode(BodyNode* bodyNode, std::string name)
    : Node(bodyNode), mName(std::move(name))
  {
  }

  const std::string& setName(const std::string& newName) override
  {
    mName = registerNameChange(newName);
    return mName;
  }

  const std::string& getName() const override
  {
    return mName;
  }

  void attachToBody()
  {
    attach();
  }

protected:
  Node* cloneNode(BodyNode* bodyNode) const override
  {
    return new TestNode(bodyNode, mName);
  }

private:
  std::string mName;
};

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
TEST(JointTest, TransformAccessorsMultiDof)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball");
  auto joint = skel->getJoint(0);

  Eigen::Isometry3d tfParent = Eigen::Isometry3d::Identity();
  tfParent.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
  tfParent.linear()
      = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  joint->setTransformFromParentBodyNode(tfParent);
  EXPECT_TRUE(
      joint->getTransformFromParentBodyNode().isApprox(tfParent, 1e-10));

  Eigen::Isometry3d tfChild = Eigen::Isometry3d::Identity();
  tfChild.translation() = Eigen::Vector3d(-0.25, -0.25, -0.25);
  joint->setTransformFromChildBodyNode(tfChild);
  EXPECT_TRUE(joint->getTransformFromChildBodyNode().isApprox(tfChild, 1e-10));
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
TEST(BallJointTest, ConvertToPositionsAndRelativeTransform)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball_transform");
  auto* joint = dynamic_cast<BallJoint*>(skel->getJoint(0));
  ASSERT_NE(joint, nullptr);

  const Eigen::Matrix3d rotation
      = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d positions = BallJoint::convertToPositions(rotation);
  joint->setPositions(positions);

  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    EXPECT_NEAR(joint->getPosition(i), positions[static_cast<int>(i)], 1e-12);
  }

  const Eigen::Isometry3d tf = joint->getRelativeTransform();
  EXPECT_TRUE(tf.linear().isApprox(rotation, 1e-10));

  const Eigen::Vector3d roundTrip = BallJoint::convertToPositions(tf.linear());
  EXPECT_TRUE(roundTrip.isApprox(positions, 1e-10));
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
TEST(ZeroDofJointTest, ConstLimitGetters)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("weld_limits");
  const Joint* constJoint = skel->getJoint(0);

  EXPECT_DOUBLE_EQ(constJoint->getPositionLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(constJoint->getPositionUpperLimit(0), 0.0);
  EXPECT_EQ(constJoint->getPositionLowerLimits().size(), 0);
  EXPECT_EQ(constJoint->getPositionUpperLimits().size(), 0);

  EXPECT_DOUBLE_EQ(constJoint->getVelocityLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(constJoint->getVelocityUpperLimit(0), 0.0);
  EXPECT_EQ(constJoint->getVelocityLowerLimits().size(), 0);
  EXPECT_EQ(constJoint->getVelocityUpperLimits().size(), 0);

  EXPECT_DOUBLE_EQ(constJoint->getAccelerationLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(constJoint->getAccelerationUpperLimit(0), 0.0);
  EXPECT_EQ(constJoint->getAccelerationLowerLimits().size(), 0);
  EXPECT_EQ(constJoint->getAccelerationUpperLimits().size(), 0);

  EXPECT_DOUBLE_EQ(constJoint->getForceLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(constJoint->getForceUpperLimit(0), 0.0);
  EXPECT_EQ(constJoint->getForceLowerLimits().size(), 0);
  EXPECT_EQ(constJoint->getForceUpperLimits().size(), 0);
}

//==============================================================================
TEST(ZeroDofJointTest, CommandsPositionsAndVelocities)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("weld_zero_ops");
  Joint* joint = skel->getJoint(0);

  joint->setCommand(0, 5.0);
  joint->setPosition(0, 1.2);
  joint->setVelocity(0, -3.4);

  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getPosition(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getVelocity(0), 0.0);

  EXPECT_EQ(joint->getCommands().size(), 0);
  EXPECT_EQ(joint->getPositions().size(), 0);
  EXPECT_EQ(joint->getVelocities().size(), 0);
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

//==============================================================================
TEST(JointTest, NonFiniteTransformFromParentBodyNodeRejected)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  Eigen::Isometry3d goodTf = Eigen::Isometry3d::Identity();
  goodTf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  joint->setTransformFromParentBodyNode(goodTf);
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().isApprox(goodTf));

  Eigen::Isometry3d nanTf = Eigen::Isometry3d::Identity();
  nanTf.translation()
      = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  joint->setTransformFromParentBodyNode(nanTf);
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().isApprox(goodTf));

  Eigen::Isometry3d infTf = Eigen::Isometry3d::Identity();
  infTf.translation()
      = Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  joint->setTransformFromParentBodyNode(infTf);
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().isApprox(goodTf));

  Eigen::Isometry3d badRotTf = Eigen::Isometry3d::Identity();
  badRotTf.linear() << 2, 0, 0, 0, 1, 0, 0, 0, 1;
  joint->setTransformFromParentBodyNode(badRotTf);
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().isApprox(goodTf));
}

//==============================================================================
TEST(JointTest, NonFiniteTransformFromChildBodyNodeRejected)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  Eigen::Isometry3d goodTf = Eigen::Isometry3d::Identity();
  goodTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.5);
  joint->setTransformFromChildBodyNode(goodTf);
  EXPECT_TRUE(joint->getTransformFromChildBodyNode().isApprox(goodTf));

  Eigen::Isometry3d nanTf = Eigen::Isometry3d::Identity();
  nanTf.translation() = Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN());
  joint->setTransformFromChildBodyNode(nanTf);
  EXPECT_TRUE(joint->getTransformFromChildBodyNode().isApprox(goodTf));
}

//==============================================================================
// gz-physics#861/#862: 1e308 is finite but overflows during dynamics.
TEST(JointTest, SimulationSurvivesOverflowTransforms)
{
  auto skel = Skeleton::create("overflow_model");

  auto [rootJoint, link1] = skel->createJointAndBodyNodePair<FreeJoint>();
  link1->setName("link1");

  WeldJoint::Properties weldProps;
  weldProps.mName = "faulty_joint";
  auto [weldJoint, link2]
      = link1->createChildJointAndBodyNodePair<WeldJoint>(weldProps);
  link2->setName("link2");

  Eigen::Isometry3d overflowTf = Eigen::Isometry3d::Identity();
  overflowTf.translation() = Eigen::Vector3d(1e308, 0.0, 0.0);
  weldJoint->setTransformFromParentBodyNode(overflowTf);

  auto world = simulation::World::create("test_world");
  world->addSkeleton(skel);

  for (int i = 0; i < 10; ++i) {
    EXPECT_NO_THROW(world->step());
  }
}

//==============================================================================
TEST(JointTest, GetSetPositions)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  Eigen::Vector6d positions;
  positions << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;

  joint->setPositions(positions);
  Eigen::VectorXd retrieved = joint->getPositions();

  EXPECT_EQ(retrieved.size(), 6);
  EXPECT_TRUE(retrieved.isApprox(positions));
}

//==============================================================================
TEST(JointTest, GetSetVelocities)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  Eigen::Vector6d velocities;
  velocities << 0.5, 1.0, 1.5, 2.0, 2.5, 3.0;

  joint->setVelocities(velocities);
  Eigen::VectorXd retrieved = joint->getVelocities();

  EXPECT_EQ(retrieved.size(), 6);
  EXPECT_TRUE(retrieved.isApprox(velocities));
}

//==============================================================================
TEST(JointTest, GetSetAccelerations)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  Eigen::Vector6d accelerations;
  accelerations << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  joint->setAccelerations(accelerations);
  Eigen::VectorXd retrieved = joint->getAccelerations();

  EXPECT_EQ(retrieved.size(), 6);
  EXPECT_TRUE(retrieved.isApprox(accelerations));
}

//==============================================================================
TEST(JointTest, GetSetForces)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  Eigen::Vector6d forces;
  forces << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;

  joint->setForces(forces);
  Eigen::VectorXd retrieved = joint->getForces();

  EXPECT_EQ(retrieved.size(), 6);
  EXPECT_TRUE(retrieved.isApprox(forces));
}

//==============================================================================
TEST(JointTest, GetPositionLimits)
{
  auto skel = createSkeletonWithJoint<TranslationalJoint>("translational");
  auto joint = skel->getJoint(0);

  Eigen::Vector3d lowerLimits(-1.0, -2.0, -3.0);
  Eigen::Vector3d upperLimits(1.0, 2.0, 3.0);

  for (std::size_t i = 0; i < 3; ++i) {
    joint->setPositionLowerLimit(i, lowerLimits[i]);
    joint->setPositionUpperLimit(i, upperLimits[i]);
  }

  Eigen::VectorXd retrievedLower = joint->getPositionLowerLimits();
  Eigen::VectorXd retrievedUpper = joint->getPositionUpperLimits();

  EXPECT_EQ(retrievedLower.size(), 3);
  EXPECT_EQ(retrievedUpper.size(), 3);
  EXPECT_TRUE(retrievedLower.isApprox(lowerLimits));
  EXPECT_TRUE(retrievedUpper.isApprox(upperLimits));
}

//==============================================================================
TEST(JointTest, HasPositionLimit)
{
  auto skel = createSkeletonWithJoint<PrismaticJoint>("prismatic");
  auto joint = skel->getJoint(0);

  joint->setPositionLowerLimit(0, -1.0);
  joint->setPositionUpperLimit(0, 1.0);
  EXPECT_TRUE(joint->hasPositionLimit(0));

  joint->setPositionLowerLimit(0, -std::numeric_limits<double>::infinity());
  joint->setPositionUpperLimit(0, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(joint->hasPositionLimit(0));
}

//==============================================================================
// Additional coverage tests for joint.cpp
//==============================================================================

//==============================================================================
TEST(JointTest, SetActuatorType)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  // Default is FORCE
  EXPECT_EQ(joint->getActuatorType(), Joint::FORCE);

  joint->setActuatorType(Joint::PASSIVE);
  EXPECT_EQ(joint->getActuatorType(), Joint::PASSIVE);

  joint->setActuatorType(Joint::SERVO);
  EXPECT_EQ(joint->getActuatorType(), Joint::SERVO);

  joint->setActuatorType(Joint::ACCELERATION);
  EXPECT_EQ(joint->getActuatorType(), Joint::ACCELERATION);

  joint->setActuatorType(Joint::VELOCITY);
  EXPECT_EQ(joint->getActuatorType(), Joint::VELOCITY);

  joint->setActuatorType(Joint::LOCKED);
  EXPECT_EQ(joint->getActuatorType(), Joint::LOCKED);

  // Setting same type again should be a no-op
  joint->setActuatorType(Joint::LOCKED);
  EXPECT_EQ(joint->getActuatorType(), Joint::LOCKED);
}

//==============================================================================
TEST(JointTest, SetActuatorTypePerDof)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  // Set base type to FORCE (dynamic)
  joint->setActuatorType(Joint::FORCE);
  EXPECT_EQ(joint->getActuatorType(), Joint::FORCE);

  // Per-DoF override to MIMIC (also dynamic) should work
  joint->setActuatorType(0, Joint::MIMIC);
  EXPECT_EQ(joint->getActuatorType(0), Joint::MIMIC);
  EXPECT_EQ(joint->getActuatorType(1), Joint::FORCE);

  // Clearing override by setting back to default
  joint->setActuatorType(0, Joint::FORCE);
  EXPECT_EQ(joint->getActuatorType(0), Joint::FORCE);
}

//==============================================================================
TEST(JointTest, GetActuatorTypes)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  joint->setActuatorType(Joint::FORCE);
  auto types = joint->getActuatorTypes();
  EXPECT_EQ(types.size(), 6u);
  for (auto t : types) {
    EXPECT_EQ(t, Joint::FORCE);
  }

  // Set one DoF to MIMIC
  joint->setActuatorType(2, Joint::MIMIC);
  types = joint->getActuatorTypes();
  EXPECT_EQ(types[0], Joint::FORCE);
  EXPECT_EQ(types[2], Joint::MIMIC);
}

//==============================================================================
TEST(JointTest, HasActuatorType)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free");
  auto joint = skel->getJoint(0);

  joint->setActuatorType(Joint::FORCE);
  EXPECT_TRUE(joint->hasActuatorType(Joint::FORCE));
  EXPECT_FALSE(joint->hasActuatorType(Joint::MIMIC));

  joint->setActuatorType(0, Joint::MIMIC);
  EXPECT_TRUE(joint->hasActuatorType(Joint::MIMIC));
  EXPECT_TRUE(joint->hasActuatorType(Joint::FORCE));
}

//==============================================================================
TEST(JointTest, IsKinematicAndIsDynamic)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setActuatorType(Joint::FORCE);
  EXPECT_TRUE(joint->isDynamic());
  EXPECT_FALSE(joint->isKinematic());

  joint->setActuatorType(Joint::VELOCITY);
  EXPECT_TRUE(joint->isKinematic());
  EXPECT_FALSE(joint->isDynamic());

  joint->setActuatorType(Joint::ACCELERATION);
  EXPECT_TRUE(joint->isKinematic());

  joint->setActuatorType(Joint::LOCKED);
  EXPECT_TRUE(joint->isKinematic());

  joint->setActuatorType(Joint::PASSIVE);
  EXPECT_TRUE(joint->isDynamic());

  joint->setActuatorType(Joint::SERVO);
  EXPECT_TRUE(joint->isDynamic());
}

//==============================================================================
TEST(JointTest, IsKinematicActuatorTypeStatic)
{
  EXPECT_TRUE(Joint::isKinematicActuatorType(Joint::ACCELERATION));
  EXPECT_TRUE(Joint::isKinematicActuatorType(Joint::VELOCITY));
  EXPECT_TRUE(Joint::isKinematicActuatorType(Joint::LOCKED));
  EXPECT_FALSE(Joint::isKinematicActuatorType(Joint::FORCE));
  EXPECT_FALSE(Joint::isKinematicActuatorType(Joint::PASSIVE));
  EXPECT_FALSE(Joint::isKinematicActuatorType(Joint::SERVO));
  EXPECT_FALSE(Joint::isKinematicActuatorType(Joint::MIMIC));
}

//==============================================================================
TEST(JointTest, IsDynamicActuatorTypeStatic)
{
  EXPECT_TRUE(Joint::isDynamicActuatorType(Joint::FORCE));
  EXPECT_TRUE(Joint::isDynamicActuatorType(Joint::PASSIVE));
  EXPECT_TRUE(Joint::isDynamicActuatorType(Joint::SERVO));
  EXPECT_TRUE(Joint::isDynamicActuatorType(Joint::MIMIC));
  EXPECT_FALSE(Joint::isDynamicActuatorType(Joint::ACCELERATION));
  EXPECT_FALSE(Joint::isDynamicActuatorType(Joint::VELOCITY));
  EXPECT_FALSE(Joint::isDynamicActuatorType(Joint::LOCKED));
}

//==============================================================================
TEST(JointTest, SetLimitEnforcement)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setLimitEnforcement(true);
  EXPECT_TRUE(joint->areLimitsEnforced());

  joint->setLimitEnforcement(false);
  EXPECT_FALSE(joint->areLimitsEnforced());
}

//==============================================================================
TEST(JointTest, CoulombFriction)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setCoulombFriction(0, 0.5);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.5);

  joint->setCoulombFriction(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.0);
}

//==============================================================================
TEST(JointTest, InitialPositionAndVelocity)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setInitialPosition(0, 0.5);
  EXPECT_DOUBLE_EQ(joint->getInitialPosition(0), 0.5);

  joint->setInitialVelocity(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getInitialVelocity(0), 1.0);
}

//==============================================================================
TEST(JointTest, CommandGetSet)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setCommand(0, 5.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 5.0);

  // Test resetCommands
  joint->resetCommands();
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
}

//==============================================================================
TEST(JointTest, ConstraintImpulse)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setConstraintImpulse(0, 3.0);
  EXPECT_DOUBLE_EQ(joint->getConstraintImpulse(0), 3.0);
}

//==============================================================================
TEST(JointTest, VelocityChange)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  // VelocityChange is typically set internally, but we can test the getter
  // After creation, velocity change should be zero
  EXPECT_DOUBLE_EQ(joint->getVelocityChange(0), 0.0);
}

//==============================================================================
TEST(JointTest, ResetVelocity)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setVelocity(0, 5.0);
  EXPECT_DOUBLE_EQ(joint->getVelocity(0), 5.0);

  joint->resetVelocity(0);
  EXPECT_DOUBLE_EQ(joint->getVelocity(0), joint->getInitialVelocity(0));
}

//==============================================================================
TEST(JointTest, ResetAccelerations)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setAcceleration(0, 3.0);
  EXPECT_DOUBLE_EQ(joint->getAcceleration(0), 3.0);

  joint->resetAccelerations();
  EXPECT_DOUBLE_EQ(joint->getAcceleration(0), 0.0);
}

//==============================================================================
TEST(JointTest, ResetForces)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  joint->setForce(0, 10.0);
  EXPECT_DOUBLE_EQ(joint->getForce(0), 10.0);

  joint->resetForces();
  EXPECT_DOUBLE_EQ(joint->getForce(0), 0.0);
}

//==============================================================================
TEST(JointTest, CheckSanity)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("revolute");
  auto joint = skel->getJoint(0);

  // Default should be sane
  joint->setPositionLowerLimit(0, -1.0);
  joint->setPositionUpperLimit(0, 1.0);
  joint->setVelocityLowerLimit(0, -5.0);
  joint->setVelocityUpperLimit(0, 5.0);
  joint->setInitialPosition(0, 0.0);
  joint->setInitialVelocity(0, 0.0);
  EXPECT_TRUE(joint->checkSanity(true));

  // Position out of limits
  joint->setInitialPosition(0, 2.0);
  EXPECT_FALSE(joint->checkSanity(true));
  EXPECT_FALSE(joint->checkSanity(false));

  // Reset and check velocity out of limits
  joint->setInitialPosition(0, 0.0);
  joint->setInitialVelocity(0, 10.0);
  EXPECT_FALSE(joint->checkSanity(true));
  EXPECT_FALSE(joint->checkSanity(false));
}

//==============================================================================
TEST(JointTest, CopyJoint)
{
  auto skel1 = createSkeletonWithJoint<RevoluteJoint>("source");
  auto skel2 = createSkeletonWithJoint<RevoluteJoint>("dest");
  auto source = skel1->getJoint(0);
  auto dest = skel2->getJoint(0);

  source->setName("source_joint");
  source->setActuatorType(Joint::SERVO);
  source->setLimitEnforcement(true);

  dest->copy(*source);
  EXPECT_EQ(dest->getName(), "source_joint");
  EXPECT_EQ(dest->getActuatorType(), Joint::SERVO);
  EXPECT_TRUE(dest->areLimitsEnforced());

  // Copy from pointer
  source->setName("ptr_copy");
  dest->copy(source);
  EXPECT_EQ(dest->getName(), "ptr_copy");

  // Copy nullptr should be no-op
  dest->copy(static_cast<const Joint*>(nullptr));
  EXPECT_EQ(dest->getName(), "ptr_copy");

  // Self-copy should be no-op
  dest->copy(*dest);
  EXPECT_EQ(dest->getName(), "ptr_copy");
}

//==============================================================================
TEST(JointTest, OperatorAssignment)
{
  auto skel1 = createSkeletonWithJoint<RevoluteJoint>("source");
  auto skel2 = createSkeletonWithJoint<RevoluteJoint>("dest");
  auto source = skel1->getJoint(0);
  auto dest = skel2->getJoint(0);

  source->setName("assigned_joint");
  source->setActuatorType(Joint::PASSIVE);

  *dest = *source;
  EXPECT_EQ(dest->getName(), "assigned_joint");
  EXPECT_EQ(dest->getActuatorType(), Joint::PASSIVE);
}

//==============================================================================
TEST(JointTest, SetMimicJoint)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("mimic_test");
  auto joint = skel->getJoint(0);

  // Create a second joint to be the reference
  BodyNode::Properties bodyProps;
  bodyProps.mName = "body2";
  bodyProps.mInertia.setMass(1.0);
  RevoluteJoint::Properties jointProps;
  jointProps.mName = "ref_joint";
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      skel->getBodyNode(0), jointProps, bodyProps);
  auto refJoint = pair.first;

  joint->setMimicJoint(refJoint, 2.0, 0.5);
  EXPECT_EQ(joint->getMimicJoint(0), refJoint);
  EXPECT_DOUBLE_EQ(joint->getMimicMultiplier(0), 2.0);
  EXPECT_DOUBLE_EQ(joint->getMimicOffset(0), 0.5);
}

//==============================================================================
TEST(JointTest, MimicConstraintType)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("mimic_type");
  auto joint = skel->getJoint(0);

  // Default should be Motor
  EXPECT_EQ(joint->getMimicConstraintType(), MimicConstraintType::Motor);

  joint->setMimicConstraintType(MimicConstraintType::Coupler);
  EXPECT_EQ(joint->getMimicConstraintType(), MimicConstraintType::Coupler);

  // Test coupler constraint helpers
  EXPECT_TRUE(joint->isUsingCouplerConstraint());

  joint->setUseCouplerConstraint(false);
  EXPECT_FALSE(joint->isUsingCouplerConstraint());
  EXPECT_EQ(joint->getMimicConstraintType(), MimicConstraintType::Motor);

  joint->setUseCouplerConstraint(true);
  EXPECT_TRUE(joint->isUsingCouplerConstraint());
}

//==============================================================================
TEST(JointTest, GetChildAndParentBodyNode)
{
  auto skel = Skeleton::create("hierarchy");

  BodyNode::Properties bodyProps1;
  bodyProps1.mName = "body1";
  bodyProps1.mInertia.setMass(1.0);
  RevoluteJoint::Properties jointProps1;
  jointProps1.mName = "joint1";
  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps1, bodyProps1);
  auto body1 = pair1.second;
  auto joint1 = pair1.first;

  BodyNode::Properties bodyProps2;
  bodyProps2.mName = "body2";
  bodyProps2.mInertia.setMass(1.0);
  RevoluteJoint::Properties jointProps2;
  jointProps2.mName = "joint2";
  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      body1, jointProps2, bodyProps2);
  auto joint2 = pair2.first;

  // joint1 has no parent body (root)
  EXPECT_EQ(joint1->getParentBodyNode(), nullptr);
  EXPECT_EQ(joint1->getChildBodyNode(), body1);

  // joint2 has body1 as parent
  EXPECT_EQ(joint2->getParentBodyNode(), body1);
  EXPECT_NE(joint2->getChildBodyNode(), nullptr);
}

//==============================================================================
TEST(JointTest, GetSkeleton)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("skel_test");
  auto joint = skel->getJoint(0);

  EXPECT_EQ(joint->getSkeleton(), skel);

  // Const version
  const Joint* constJoint = joint;
  EXPECT_EQ(constJoint->getSkeleton(), skel);
}

//==============================================================================
TEST(JointTest, JointIndexInSkeleton)
{
  auto skel = Skeleton::create("index_test");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "body1";
  bodyProps.mInertia.setMass(1.0);
  RevoluteJoint::Properties jointProps;
  jointProps.mName = "joint1";
  skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps, bodyProps);

  auto joint = skel->getJoint(0);
  EXPECT_EQ(joint->getJointIndexInSkeleton(), 0u);
  EXPECT_EQ(joint->getJointIndexInTree(), 0u);
  EXPECT_EQ(joint->getTreeIndex(), 0u);
}

//==============================================================================
TEST(JointTest, SetName)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("name_test");
  auto joint = skel->getJoint(0);

  const std::string& name = joint->setName("new_name");
  EXPECT_EQ(name, "new_name");
  EXPECT_EQ(joint->getName(), "new_name");

  // Setting same name should return same name
  const std::string& sameName = joint->setName("new_name");
  EXPECT_EQ(sameName, "new_name");

  // Setting name with rename dofs
  joint->setName("renamed", true);
  EXPECT_EQ(joint->getName(), "renamed");
}

//==============================================================================
TEST(JointTest, IntegratePositions)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("integrate");
  auto joint = skel->getJoint(0);

  Eigen::VectorXd q0(1);
  q0 << 0.0;
  Eigen::VectorXd v(1);
  v << 1.0;
  double dt = 0.01;

  Eigen::VectorXd result = joint->integratePositions(q0, v, dt);
  EXPECT_EQ(result.size(), 1);
  EXPECT_NEAR(result[0], 0.01, 1e-10);
}

//==============================================================================
TEST(JointTest, GetRelativeTransform)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("rel_tf");
  auto joint = skel->getJoint(0);

  joint->setPosition(0, 0.0);
  const Eigen::Isometry3d& tf = joint->getRelativeTransform();
  EXPECT_TRUE(tf.matrix().allFinite());
}

//==============================================================================
TEST(JointTest, GetRelativeSpatialVelocity)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("rel_vel");
  auto joint = skel->getJoint(0);

  joint->setVelocity(0, 1.0);
  const Eigen::Vector6d& vel = joint->getRelativeSpatialVelocity();
  EXPECT_TRUE(vel.allFinite());
}

//==============================================================================
TEST(JointTest, GetRelativeSpatialAcceleration)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("rel_acc");
  auto joint = skel->getJoint(0);

  joint->setAcceleration(0, 1.0);
  const Eigen::Vector6d& acc = joint->getRelativeSpatialAcceleration();
  EXPECT_TRUE(acc.allFinite());
}

//==============================================================================
TEST(JointTest, GetRelativePrimaryAcceleration)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("rel_prim_acc");
  auto joint = skel->getJoint(0);

  const Eigen::Vector6d& acc = joint->getRelativePrimaryAcceleration();
  EXPECT_TRUE(acc.allFinite());
}

//==============================================================================
TEST(JointTest, GetWrenchToChildBodyNode)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("wrench");
  auto joint = skel->getJoint(0);
  auto body = skel->getBodyNode(0);

  // Compute forward dynamics to set up body forces
  skel->computeForwardDynamics();

  // Default (joint frame)
  Eigen::Vector6d wrench = joint->getWrenchToChildBodyNode();
  EXPECT_TRUE(wrench.allFinite());

  // With respect to child body
  wrench = joint->getWrenchToChildBodyNode(body);
  EXPECT_TRUE(wrench.allFinite());

  // With respect to parent body (nullptr for root)
  wrench = joint->getWrenchToChildBodyNode(nullptr);
  EXPECT_TRUE(wrench.allFinite());

  // With respect to world frame
  wrench = joint->getWrenchToChildBodyNode(Frame::World());
  EXPECT_TRUE(wrench.allFinite());
}

//==============================================================================
TEST(JointTest, GetWrenchToParentBodyNode)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("wrench_parent");
  auto joint = skel->getJoint(0);

  skel->computeForwardDynamics();

  Eigen::Vector6d wrench = joint->getWrenchToParentBodyNode();
  EXPECT_TRUE(wrench.allFinite());
}

//==============================================================================
TEST(JointTest, SetActuatorTypes)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free_types");
  auto joint = skel->getJoint(0);

  // Set all to FORCE
  std::vector<Joint::ActuatorType> types(6, Joint::FORCE);
  joint->setActuatorTypes(types);
  auto retrieved = joint->getActuatorTypes();
  for (auto t : retrieved) {
    EXPECT_EQ(t, Joint::FORCE);
  }

  // Set with one MIMIC override
  types[2] = Joint::MIMIC;
  joint->setActuatorTypes(types);
  retrieved = joint->getActuatorTypes();
  EXPECT_EQ(retrieved[0], Joint::FORCE);
  EXPECT_EQ(retrieved[2], Joint::MIMIC);

  // Wrong size should be rejected
  std::vector<Joint::ActuatorType> wrongSize(3, Joint::FORCE);
  joint->setActuatorTypes(wrongSize);
  // Should still have previous types
  retrieved = joint->getActuatorTypes();
  EXPECT_EQ(retrieved.size(), 6u);
}

//==============================================================================
TEST(JointTest, SetProperties)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("props");
  auto joint = skel->getJoint(0);

  Joint::Properties props;
  props.mName = "custom_name";
  props.mIsPositionLimitEnforced = true;
  props.mActuatorType = Joint::SERVO;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  props.mT_ParentBodyToJoint = tf;

  joint->setProperties(props);
  EXPECT_EQ(joint->getName(), "custom_name");
  EXPECT_TRUE(joint->areLimitsEnforced());
  EXPECT_EQ(joint->getActuatorType(), Joint::SERVO);
}

//==============================================================================
TEST(JointTest, GetJointProperties)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("get_props");
  auto joint = skel->getJoint(0);

  joint->setName("test_props");
  joint->setActuatorType(Joint::PASSIVE);
  joint->setLimitEnforcement(true);

  const Joint::Properties& props = joint->getJointProperties();
  EXPECT_EQ(props.mName, "test_props");
  EXPECT_EQ(props.mActuatorType, Joint::PASSIVE);
  EXPECT_TRUE(props.mIsPositionLimitEnforced);
}

//==============================================================================
TEST(JointTest, MimicDofProperties)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("mimic_dof");
  auto joint = skel->getJoint(0);

  auto props = joint->getMimicDofProperties();
  EXPECT_EQ(props.size(), 6u);

  // Set mimic dof via map
  MimicDofProperties prop;
  prop.mMultiplier = 3.0;
  prop.mOffset = 1.0;
  std::map<std::size_t, MimicDofProperties> propMap;
  propMap[0] = prop;
  joint->setMimicJointDofs(propMap);
  EXPECT_DOUBLE_EQ(joint->getMimicMultiplier(0), 3.0);
  EXPECT_DOUBLE_EQ(joint->getMimicOffset(0), 1.0);
}

//==============================================================================
// Additional FreeJoint coverage tests
//==============================================================================

//==============================================================================
TEST(FreeJointTest, ConvertToPositionsWithCharts)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  tf.linear()
      = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // EXP_MAP (default)
  Eigen::Vector6d posExpMap = FreeJoint::convertToPositions(tf);
  EXPECT_TRUE(posExpMap.allFinite());

  // EULER_XYZ
  Eigen::Vector6d posXYZ = FreeJoint::convertToPositions(
      tf, FreeJoint::CoordinateChart::EULER_XYZ);
  EXPECT_TRUE(posXYZ.allFinite());

  // EULER_ZYX
  Eigen::Vector6d posZYX = FreeJoint::convertToPositions(
      tf, FreeJoint::CoordinateChart::EULER_ZYX);
  EXPECT_TRUE(posZYX.allFinite());

  // Roundtrip: positions -> transform -> positions
  Eigen::Isometry3d tfBack = FreeJoint::convertToTransform(posExpMap);
  EXPECT_TRUE(tfBack.translation().isApprox(tf.translation(), 1e-10));
  EXPECT_TRUE(tfBack.linear().isApprox(tf.linear(), 1e-10));
}

//==============================================================================
TEST(FreeJointTest, ConvertToTransformWithCharts)
{
  Eigen::Vector6d positions;
  positions << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;

  // EXP_MAP
  Eigen::Isometry3d tfExpMap = FreeJoint::convertToTransform(positions);
  EXPECT_TRUE(tfExpMap.matrix().allFinite());

  // EULER_XYZ
  Eigen::Isometry3d tfXYZ = FreeJoint::convertToTransform(
      positions, FreeJoint::CoordinateChart::EULER_XYZ);
  EXPECT_TRUE(tfXYZ.matrix().allFinite());

  // EULER_ZYX
  Eigen::Isometry3d tfZYX = FreeJoint::convertToTransform(
      positions, FreeJoint::CoordinateChart::EULER_ZYX);
  EXPECT_TRUE(tfZYX.matrix().allFinite());

  // All should have same translation
  EXPECT_TRUE(tfExpMap.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(tfXYZ.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(tfZYX.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(FreeJointTest, ConvertWithInvalidChart)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.3, -0.4, 0.5);
  tf.linear()
      = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const auto invalidChart = static_cast<FreeJoint::CoordinateChart>(-1);
  Eigen::Vector6d positions = FreeJoint::convertToPositions(tf, invalidChart);
  EXPECT_TRUE(positions.head<3>().isZero(1e-12));
  EXPECT_TRUE(positions.tail<3>().isApprox(tf.translation(), 1e-12));

  Eigen::Vector6d raw;
  raw << 0.9, -0.8, 0.7, -1.0, 2.0, -3.0;
  Eigen::Isometry3d tfInvalid
      = FreeJoint::convertToTransform(raw, invalidChart);
  EXPECT_TRUE(tfInvalid.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(tfInvalid.translation().isApprox(raw.tail<3>(), 1e-12));
}

//==============================================================================
TEST(FreeJointTest, SetCoordinateChart)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("chart_test");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  EXPECT_EQ(
      freeJoint->getCoordinateChart(), FreeJoint::CoordinateChart::EXP_MAP);

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);
  EXPECT_EQ(
      freeJoint->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_XYZ);

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  EXPECT_EQ(
      freeJoint->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_ZYX);

  // Setting same chart should be no-op
  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  EXPECT_EQ(
      freeJoint->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_ZYX);
}

//==============================================================================
TEST(FreeJointTest, SetRelativeTransform)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("rel_tf");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  freeJoint->setRelativeTransform(tf);
  const Eigen::Isometry3d& result = freeJoint->getRelativeTransform();
  EXPECT_TRUE(result.translation().isApprox(tf.translation(), 1e-10));
}

//==============================================================================
TEST(FreeJointTest, SetTransform)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("set_tf");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Isometry3d worldTf = Eigen::Isometry3d::Identity();
  worldTf.translation() = Eigen::Vector3d(5.0, 6.0, 7.0);

  freeJoint->setTransform(worldTf, Frame::World());

  auto body = skel->getBodyNode(0);
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      worldTf.translation(), 1e-10));
}

//==============================================================================
TEST(FreeJointTest, SetTransformWithReferenceFrameAndCharts)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("ref_frame_tf");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  auto refFrame = SimpleFrame::createShared(Frame::World(), "ref_frame");
  refFrame->setTranslation(Eigen::Vector3d(0.2, -0.1, 0.3));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);
  freeJoint->setTransform(tf, refFrame.get());
  EXPECT_TRUE(freeJoint->getPositions().allFinite());

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  freeJoint->setTransform(tf, refFrame.get());
  EXPECT_TRUE(freeJoint->getPositions().allFinite());
}

//==============================================================================
TEST(FreeJointTest, TransformJacobianAndDynamicsCoverage)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free_cov");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Isometry3d worldTf = Eigen::Isometry3d::Identity();
  worldTf.translation() = Eigen::Vector3d(0.5, -0.3, 0.2);
  worldTf.linear()
      = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  freeJoint->setTransform(worldTf, Frame::World());

  auto refFrame = SimpleFrame::createShared(Frame::World(), "free_ref");
  refFrame->setTranslation(Eigen::Vector3d(-0.1, 0.2, 0.3));

  Eigen::Isometry3d localTf = Eigen::Isometry3d::Identity();
  localTf.translation() = Eigen::Vector3d(0.2, 0.1, -0.2);
  localTf.linear()
      = Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitY()).toRotationMatrix();
  freeJoint->setTransform(localTf, refFrame.get());

  Eigen::Vector6d positions;
  positions << 0.1, -0.2, 0.15, 0.4, -0.3, 0.2;
  Eigen::Vector6d velocities;
  velocities << -0.05, 0.1, 0.2, -0.3, 0.25, -0.15;
  freeJoint->setPositions(positions);
  freeJoint->setVelocities(velocities);

  const auto jac = freeJoint->getRelativeJacobian();
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), 6);

  const auto jacDeriv = freeJoint->getRelativeJacobianTimeDeriv();
  EXPECT_EQ(jacDeriv.rows(), 6);
  EXPECT_EQ(jacDeriv.cols(), 6);

  Eigen::VectorXd jacPositions = Eigen::VectorXd::Zero(6);
  jacPositions.tail<3>() = Eigen::Vector3d(0.1, -0.2, 0.05);
  const auto jacAtPositions = freeJoint->getRelativeJacobian(jacPositions);
  EXPECT_EQ(jacAtPositions.rows(), 6);
  EXPECT_EQ(jacAtPositions.cols(), 6);

  skel->computeForwardDynamics();

  Eigen::Vector6d spatialVel;
  spatialVel << 0.2, -0.1, 0.3, 0.4, -0.2, 0.1;
  freeJoint->setSpatialVelocity(spatialVel, Frame::World(), Frame::World());
  EXPECT_TRUE(freeJoint->getVelocities().allFinite());
}

//==============================================================================
TEST(FreeJointTest, SetSpatialMotion)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("spatial_motion");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  Eigen::Vector6d vel = Eigen::Vector6d::Zero();
  vel.tail<3>() = Eigen::Vector3d(0.0, 1.0, 0.0);

  Eigen::Vector6d acc = Eigen::Vector6d::Zero();
  acc.tail<3>() = Eigen::Vector3d(0.0, 0.0, 1.0);

  freeJoint->setSpatialMotion(
      &tf,
      Frame::World(),
      &vel,
      Frame::World(),
      freeJoint->getChildBodyNode(),
      &acc,
      Frame::World(),
      freeJoint->getChildBodyNode());

  // Verify transform was set
  auto body = skel->getBodyNode(0);
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));
}

//==============================================================================
TEST(FreeJointTest, SetSpatialMotionNullArgs)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("spatial_null");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  // All null pointers should be no-op
  freeJoint->setSpatialMotion(
      nullptr,
      Frame::World(),
      nullptr,
      Frame::World(),
      freeJoint->getChildBodyNode(),
      nullptr,
      Frame::World(),
      freeJoint->getChildBodyNode());

  // Should not crash
  EXPECT_TRUE(freeJoint->getRelativeTransform().matrix().allFinite());
}

//==============================================================================
TEST(FreeJointTest, SetLinearVelocity)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("lin_vel");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Vector3d linVel(1.0, 2.0, 3.0);
  freeJoint->setLinearVelocity(linVel, Frame::World(), Frame::World());

  // Velocities should be set
  Eigen::VectorXd vels = freeJoint->getVelocities();
  EXPECT_TRUE(vels.allFinite());
}

//==============================================================================
TEST(FreeJointTest, SetAngularVelocity)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("ang_vel");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Vector3d angVel(0.1, 0.2, 0.3);
  freeJoint->setAngularVelocity(angVel, Frame::World(), Frame::World());

  Eigen::VectorXd vels = freeJoint->getVelocities();
  EXPECT_TRUE(vels.allFinite());
}

//==============================================================================
TEST(FreeJointTest, CopyFreeJoint)
{
  auto skel1 = createSkeletonWithJoint<FreeJoint>("source");
  auto skel2 = createSkeletonWithJoint<FreeJoint>("dest");
  auto* source = static_cast<FreeJoint*>(skel1->getJoint(0));
  auto* dest = static_cast<FreeJoint*>(skel2->getJoint(0));

  source->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);

  dest->copy(*source);
  EXPECT_EQ(dest->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_XYZ);

  // Copy from pointer
  source->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  dest->copy(source);
  EXPECT_EQ(dest->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_ZYX);

  // Copy nullptr should be no-op
  dest->copy(static_cast<const FreeJoint*>(nullptr));
  EXPECT_EQ(dest->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_ZYX);

  // Self-copy should be no-op
  dest->copy(*dest);
  EXPECT_EQ(dest->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_ZYX);
}

//==============================================================================
TEST(FreeJointTest, OperatorAssignment)
{
  auto skel1 = createSkeletonWithJoint<FreeJoint>("source");
  auto skel2 = createSkeletonWithJoint<FreeJoint>("dest");
  auto* source = static_cast<FreeJoint*>(skel1->getJoint(0));
  auto* dest = static_cast<FreeJoint*>(skel2->getJoint(0));

  source->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);
  *dest = *source;
  EXPECT_EQ(dest->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_XYZ);
}

//==============================================================================
TEST(FreeJointTest, IsCyclic)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("cyclic");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  // Rotation DOFs (0-2) are cyclic when no position limits
  EXPECT_TRUE(freeJoint->isCyclic(0));
  EXPECT_TRUE(freeJoint->isCyclic(1));
  EXPECT_TRUE(freeJoint->isCyclic(2));

  // Translation DOFs (3-5) are not cyclic
  EXPECT_FALSE(freeJoint->isCyclic(3));
  EXPECT_FALSE(freeJoint->isCyclic(4));
  EXPECT_FALSE(freeJoint->isCyclic(5));
}

//==============================================================================
TEST(FreeJointTest, SetTransformOfStaticMethods)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("static_tf");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));
  auto body = skel->getBodyNode(0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(3.0, 4.0, 5.0);

  // setTransformOf with Joint*
  FreeJoint::setTransformOf(freeJoint, tf, Frame::World());
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));

  // setTransformOf with BodyNode*
  tf.translation() = Eigen::Vector3d(6.0, 7.0, 8.0);
  FreeJoint::setTransformOf(body, tf, Frame::World());
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));

  // setTransformOf with Skeleton*
  tf.translation() = Eigen::Vector3d(9.0, 10.0, 11.0);
  FreeJoint::setTransformOf(skel.get(), tf, Frame::World());
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));

  // setTransformOf with nullptr should be no-op
  FreeJoint::setTransformOf(static_cast<Joint*>(nullptr), tf, Frame::World());
  FreeJoint::setTransformOf(
      static_cast<BodyNode*>(nullptr), tf, Frame::World());
  FreeJoint::setTransformOf(
      static_cast<Skeleton*>(nullptr), tf, Frame::World());
}

//==============================================================================
TEST(FreeJointTest, GetPositionDifferences)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("pos_diff");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Vector6d q1 = Eigen::Vector6d::Zero();
  q1.tail<3>() = Eigen::Vector3d(1.0, 0.0, 0.0);

  Eigen::Vector6d q2 = Eigen::Vector6d::Zero();
  q2.tail<3>() = Eigen::Vector3d(2.0, 0.0, 0.0);

  Eigen::Vector6d diff = freeJoint->getPositionDifferencesStatic(q2, q1);
  EXPECT_TRUE(diff.allFinite());
  EXPECT_NEAR(diff[3], 1.0, 1e-10);
  EXPECT_NEAR(diff[4], 0.0, 1e-10);
  EXPECT_NEAR(diff[5], 0.0, 1e-10);
}

//==============================================================================
TEST(FreeJointTest, GetRelativeJacobianStatic)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("jac_static");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  Eigen::Matrix6d J = freeJoint->getRelativeJacobianStatic(positions);
  EXPECT_TRUE(J.allFinite());
}

//==============================================================================
TEST(FreeJointTest, GetFreeJointProperties)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("props");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);
  auto props = freeJoint->getFreeJointProperties();
  EXPECT_EQ(props.mCoordinateChart, FreeJoint::CoordinateChart::EULER_XYZ);
}

//==============================================================================
TEST(FreeJointTest, SetTransformOfSkeletonAllRoots)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("all_roots");

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  // Apply to all root bodies
  FreeJoint::setTransformOf(skel.get(), tf, Frame::World(), true);
  auto body = skel->getBodyNode(0);
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));
}

//==============================================================================
TEST(FreeJointTest, SpatialVelocityAndAccelerationFrames)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("frame_motion");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  auto refFrame = SimpleFrame::createShared(Frame::World(), "ref_frame");
  refFrame->setTranslation(Eigen::Vector3d(0.1, -0.2, 0.3));

  Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();
  targetTf.translation() = Eigen::Vector3d(0.2, 0.1, -0.1);
  freeJoint->setTransform(targetTf, refFrame.get());
  EXPECT_TRUE(freeJoint->getPositions().array().isFinite().all());

  Eigen::Vector6d spatialVel;
  spatialVel << 0.1, -0.2, 0.3, 1.0, -2.0, 3.0;
  freeJoint->setSpatialVelocity(spatialVel, refFrame.get(), refFrame.get());
  freeJoint->setSpatialVelocity(spatialVel, Frame::World(), refFrame.get());

  Eigen::Vector6d spatialAcc;
  spatialAcc << -0.1, 0.2, -0.3, 0.5, -0.6, 0.7;
  freeJoint->setSpatialAcceleration(spatialAcc, refFrame.get(), refFrame.get());
  freeJoint->setSpatialAcceleration(spatialAcc, Frame::World(), refFrame.get());

  freeJoint->setVelocities(Eigen::Vector6d::Constant(0.05));
  const auto jacobianDot = freeJoint->getRelativeJacobianTimeDeriv();
  EXPECT_TRUE(jacobianDot.allFinite());
}

//==============================================================================
TEST(ZeroDofJointTest, ConstLimitAccessors)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("const_limits");
  const auto* joint = static_cast<const WeldJoint*>(skel->getJoint(0));

  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityUpperLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getAccelerationLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getAccelerationUpperLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 0.0);
}

//==============================================================================
TEST(JointTest, ConstAccessorsAndIntegratePositions)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("const_joint");
  auto* joint = skel->getJoint(0);
  const Joint* constJoint = joint;

  EXPECT_EQ(constJoint->getChildBodyNode(), skel->getBodyNode(0));
  EXPECT_EQ(constJoint->getParentBodyNode(), nullptr);
  EXPECT_EQ(constJoint->getSkeleton(), skel);

  Eigen::VectorXd q0(1);
  q0 << 0.0;
  Eigen::VectorXd v(1);
  v << 1.0;
  const double dt = 0.2;
  Eigen::VectorXd result = constJoint->integratePositions(q0, v, dt);
  EXPECT_NEAR(result[0], 0.2, 1e-10);
}

//==============================================================================
TEST(FreeJointTest, SetTransformWithNonWorldReference)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("non_world_ref");
  auto* freeJoint = static_cast<FreeJoint*>(skel->getJoint(0));

  auto refFrame = SimpleFrame::createShared(Frame::World(), "non_world_ref");
  Eigen::Isometry3d refTf = Eigen::Isometry3d::Identity();
  refTf.translation() = Eigen::Vector3d(-0.2, 0.1, 0.05);
  refTf.linear()
      = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  refFrame->setTransform(refTf);

  Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();
  targetTf.translation() = Eigen::Vector3d(0.4, -0.3, 0.2);
  targetTf.linear()
      = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);
  freeJoint->setTransform(targetTf, refFrame.get());
  const auto xyzPositions = freeJoint->getPositions();
  EXPECT_EQ(xyzPositions.size(), 6);

  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  freeJoint->setTransform(targetTf, refFrame.get());
  const auto zyxPositions = freeJoint->getPositions();
  EXPECT_EQ(zyxPositions.size(), 6);
}

//==============================================================================
TEST(NodeTest, ConstAccessorsAndRemovalState)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("node_accessors");
  auto* body = skel->getBodyNode(0);
  auto* node = new TestNode(body, "test_node");

  EXPECT_TRUE(node->isRemoved());
  node->attachToBody();
  EXPECT_FALSE(node->isRemoved());

  EXPECT_EQ(node->getBodyNodePtr().get(), body);
  const TestNode* constNode = node;
  EXPECT_EQ(constNode->getBodyNodePtr().get(), body);
  EXPECT_EQ(constNode->getSkeleton(), skel);
}

//==============================================================================
TEST(LinkageTest, EmptyCriteriaReturnsEmpty)
{
  Linkage::Criteria criteria;
  const auto nodes = criteria.satisfy();
  EXPECT_TRUE(nodes.empty());
}

//==============================================================================
TEST(LinkageTest, ChainStopsAtFreeJoint)
{
  auto skel = Skeleton::create("linkage_chain");

  BodyNode::Properties bodyProps;
  bodyProps.mInertia.setMass(1.0);

  RevoluteJoint::Properties rootProps;
  rootProps.mName = "root_joint";
  auto rootPair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, rootProps, bodyProps);

  FreeJoint::Properties freeProps;
  freeProps.mName = "free_joint";
  auto freePair = skel->createJointAndBodyNodePair<FreeJoint>(
      rootPair.second, freeProps, bodyProps);

  RevoluteJoint::Properties leafProps;
  leafProps.mName = "leaf_joint";
  auto leafPair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      freePair.second, leafProps, bodyProps);

  Linkage::Criteria criteria(rootPair.second, leafPair.second, true);
  ASSERT_EQ(criteria.mTargets.size(), 1u);
  criteria.mTargets[0].mChain = true;
  criteria.mStart.mPolicy = Linkage::Criteria::INCLUDE;

  const auto nodes = criteria.satisfy();
  EXPECT_FALSE(nodes.empty());
  EXPECT_EQ(nodes.front(), rootPair.second);

  bool hasFree = false;
  for (auto* node : nodes) {
    if (node == freePair.second) {
      hasFree = true;
      break;
    }
  }
  EXPECT_FALSE(hasFree);
}
