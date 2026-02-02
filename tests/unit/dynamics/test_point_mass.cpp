// Copyright (c) 2011, The DART development contributors

#include <dart/simulation/world.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/point_mass.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dynamics;

TEST(PointMassStateTest, DefaultConstructor)
{
  PointMass::State state;

  EXPECT_TRUE(state.mPositions.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(state.mVelocities.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(state.mAccelerations.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(state.mForces.isApprox(Eigen::Vector3d::Zero()));
}

TEST(PointMassStateTest, ParameterizedConstructor)
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  Eigen::Vector3d vel(0.1, 0.2, 0.3);
  Eigen::Vector3d acc(0.01, 0.02, 0.03);
  Eigen::Vector3d force(10.0, 20.0, 30.0);

  PointMass::State state(pos, vel, acc, force);

  EXPECT_TRUE(state.mPositions.isApprox(pos));
  EXPECT_TRUE(state.mVelocities.isApprox(vel));
  EXPECT_TRUE(state.mAccelerations.isApprox(acc));
  EXPECT_TRUE(state.mForces.isApprox(force));
}

TEST(PointMassStateTest, Equality)
{
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  PointMass::State state1(pos);
  PointMass::State state2(pos);
  PointMass::State state3;

  EXPECT_TRUE(state1 == state2);
  EXPECT_FALSE(state1 == state3);
}

TEST(PointMassPropertiesTest, DefaultConstructor)
{
  PointMass::Properties props;

  EXPECT_TRUE(props.mX0.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_DOUBLE_EQ(props.mMass, 0.0005);
  EXPECT_TRUE(props.mConnectedPointMassIndices.empty());
}

TEST(PointMassPropertiesTest, SetRestingPosition)
{
  PointMass::Properties props;
  Eigen::Vector3d restPos(1.0, 2.0, 3.0);

  props.setRestingPosition(restPos);
  EXPECT_TRUE(props.mX0.isApprox(restPos));
}

TEST(PointMassPropertiesTest, SetMass)
{
  PointMass::Properties props;

  props.setMass(0.5);
  EXPECT_DOUBLE_EQ(props.mMass, 0.5);

  props.setMass(1.0);
  EXPECT_DOUBLE_EQ(props.mMass, 1.0);
}

TEST(PointMassPropertiesTest, Equality)
{
  PointMass::Properties props1;
  PointMass::Properties props2;

  EXPECT_TRUE(props1 == props2);

  props1.setMass(1.0);
  EXPECT_TRUE(props1 != props2);

  props2.setMass(1.0);
  EXPECT_TRUE(props1 == props2);
}

TEST(PointMassPropertiesTest, ParameterizedConstructor)
{
  Eigen::Vector3d x0(0.1, 0.2, 0.3);
  double mass = 0.01;
  std::vector<std::size_t> connections = {1, 2, 3};

  PointMass::Properties props(x0, mass, connections);

  EXPECT_TRUE(props.mX0.isApprox(x0));
  EXPECT_DOUBLE_EQ(props.mMass, mass);
  EXPECT_EQ(props.mConnectedPointMassIndices.size(), 3u);
  EXPECT_EQ(props.mConnectedPointMassIndices[0], 1u);
  EXPECT_EQ(props.mConnectedPointMassIndices[1], 2u);
  EXPECT_EQ(props.mConnectedPointMassIndices[2], 3u);
}

TEST(PointMassPropertiesTest, DefaultLimitsAreInfinite)
{
  PointMass::Properties props;

  EXPECT_DOUBLE_EQ(props.mPositionLowerLimits[0], -math::inf);
  EXPECT_DOUBLE_EQ(props.mPositionUpperLimits[0], math::inf);
  EXPECT_DOUBLE_EQ(props.mVelocityLowerLimits[0], -math::inf);
  EXPECT_DOUBLE_EQ(props.mVelocityUpperLimits[0], math::inf);
}

TEST(PointMassStateTest, StateModification)
{
  PointMass::State state;

  state.mPositions = Eigen::Vector3d(1.0, 1.0, 1.0);
  EXPECT_TRUE(state.mPositions.isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));

  state.mVelocities = Eigen::Vector3d(0.5, 0.5, 0.5);
  EXPECT_TRUE(state.mVelocities.isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  state.mAccelerations = Eigen::Vector3d(0.1, 0.1, 0.1);
  EXPECT_TRUE(state.mAccelerations.isApprox(Eigen::Vector3d(0.1, 0.1, 0.1)));

  state.mForces = Eigen::Vector3d(10.0, 10.0, 10.0);
  EXPECT_TRUE(state.mForces.isApprox(Eigen::Vector3d(10.0, 10.0, 10.0)));
}

TEST(PointMassPropertiesTest, ConnectionIndices)
{
  std::vector<std::size_t> connections = {0, 1, 2, 3, 4};
  PointMass::Properties props(Eigen::Vector3d::Zero(), 0.001, connections);

  EXPECT_EQ(props.mConnectedPointMassIndices.size(), 5u);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(props.mConnectedPointMassIndices[i], i);
  }
}

TEST(PointMassPropertiesTest, CustomLimits)
{
  Eigen::Vector3d posLower(-1.0, -1.0, -1.0);
  Eigen::Vector3d posUpper(1.0, 1.0, 1.0);
  Eigen::Vector3d velLower(-10.0, -10.0, -10.0);
  Eigen::Vector3d velUpper(10.0, 10.0, 10.0);

  PointMass::Properties props(
      Eigen::Vector3d::Zero(),
      0.001,
      {},
      posLower,
      posUpper,
      velLower,
      velUpper);

  EXPECT_TRUE(props.mPositionLowerLimits.isApprox(posLower));
  EXPECT_TRUE(props.mPositionUpperLimits.isApprox(posUpper));
  EXPECT_TRUE(props.mVelocityLowerLimits.isApprox(velLower));
  EXPECT_TRUE(props.mVelocityUpperLimits.isApprox(velUpper));
}

TEST(PointMass, KinematicsAndForces)
{
  auto skeleton = Skeleton::create("point-mass");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(1.0),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  ASSERT_GT(softBody->getNumPointMasses(), 0u);
  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  pm->setPositions(Eigen::Vector3d(0.2, 0.3, 0.4));
  EXPECT_TRUE(pm->getPositions().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  pm->setPosition(1, 0.8);
  EXPECT_NEAR(pm->getPosition(1), 0.8, 1e-12);
  pm->resetPositions();
  EXPECT_TRUE(pm->getPositions().isApprox(Eigen::Vector3d::Zero()));

  pm->setVelocities(Eigen::Vector3d(0.5, 0.6, 0.7));
  EXPECT_TRUE(pm->getVelocities().isApprox(Eigen::Vector3d(0.5, 0.6, 0.7)));
  pm->setVelocity(0, -0.2);
  EXPECT_NEAR(pm->getVelocity(0), -0.2, 1e-12);
  pm->integratePositions(0.5);
  EXPECT_NEAR(pm->getPositions()[0], -0.1, 1e-12);
  pm->resetVelocities();
  EXPECT_TRUE(pm->getVelocities().isApprox(Eigen::Vector3d::Zero()));

  pm->setAccelerations(Eigen::Vector3d(1.0, -2.0, 3.0));
  EXPECT_TRUE(pm->getAccelerations().isApprox(Eigen::Vector3d(1.0, -2.0, 3.0)));
  pm->setAcceleration(2, 0.9);
  EXPECT_NEAR(pm->getAcceleration(2), 0.9, 1e-12);
  pm->integrateVelocities(0.5);
  EXPECT_NEAR(pm->getVelocities()[2], 0.45, 1e-12);
  pm->resetAccelerations();
  EXPECT_TRUE(pm->getAccelerations().isApprox(Eigen::Vector3d::Zero()));

  pm->setForces(Eigen::Vector3d(3.0, 2.0, 1.0));
  EXPECT_TRUE(pm->getForces().isApprox(Eigen::Vector3d(3.0, 2.0, 1.0)));
  pm->setForce(1, -4.0);
  EXPECT_NEAR(pm->getForce(1), -4.0, 1e-12);
  pm->resetForces();
  EXPECT_TRUE(pm->getForces().isApprox(Eigen::Vector3d::Zero()));

  pm->setVelocityChange(0, 0.3);
  EXPECT_NEAR(pm->getVelocityChange(0), 0.3, 1e-12);
  pm->resetVelocityChanges();
  EXPECT_NEAR(pm->getVelocityChange(0), 0.0, 1e-12);

  pm->setConstraintImpulse(0, 1.2);
  EXPECT_NEAR(pm->getConstraintImpulse(0), 1.2, 1e-12);
  pm->addConstraintImpulse(Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_TRUE(
      pm->getConstraintImpulses().isApprox(Eigen::Vector3d(1.3, 0.2, 0.3)));
  pm->clearConstraintImpulse();
  EXPECT_TRUE(pm->getConstraintImpulses().isApprox(Eigen::Vector3d::Zero()));

  pm->addExtForce(Eigen::Vector3d(0.5, 0.0, -0.5));
  pm->addExtForce(Eigen::Vector3d(0.1, 0.2, 0.3), true);
  pm->clearExtForce();

  const auto bodyJac = pm->getBodyJacobian();
  EXPECT_EQ(bodyJac.rows(), 3);
  EXPECT_EQ(bodyJac.cols(), softBody->getNumDependentGenCoords() + 3);

  const auto worldJac = pm->getWorldJacobian();
  EXPECT_EQ(worldJac.rows(), 3);
  EXPECT_EQ(worldJac.cols(), softBody->getNumDependentGenCoords() + 3);

  pm->setVelocities(Eigen::Vector3d(0.1, 0.0, 0.0));
  EXPECT_TRUE(pm->getBodyVelocity().isApprox(Eigen::Vector3d(0.1, 0.0, 0.0)));
  EXPECT_TRUE(pm->getWorldVelocity().isApprox(Eigen::Vector3d(0.1, 0.0, 0.0)));

  pm->setAccelerations(Eigen::Vector3d(0.0, -0.2, 0.0));
  EXPECT_TRUE(
      pm->getBodyAcceleration().isApprox(Eigen::Vector3d(0.0, -0.2, 0.0)));
  EXPECT_TRUE(
      pm->getWorldAcceleration().isApprox(Eigen::Vector3d(0.0, -0.2, 0.0)));
}

TEST(PointMass, WorldStepUpdates)
{
  auto skeleton = Skeleton::create("point-mass-step");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      8.0,
      8.0,
      0.2);

  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(skeleton);

  for (int i = 0; i < 4; ++i) {
    world->step();
  }

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);
  EXPECT_TRUE(pm->getBodyVelocityChange().array().isFinite().all());
  EXPECT_TRUE(pm->getConstraintImpulses().array().isFinite().all());
  EXPECT_TRUE(pm->getWorldVelocity().array().isFinite().all());
  EXPECT_TRUE(pm->getWorldPosition().array().isFinite().all());
}

TEST(PointMass, ConnectedPointMassesAndMass)
{
  auto skeleton = Skeleton::create("point-mass-connect");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(1.0),
      Eigen::Isometry3d::Identity(),
      1.0,
      6.0,
      6.0,
      0.2);

  ASSERT_GE(softBody->getNumPointMasses(), 2u);
  auto* pm0 = softBody->getPointMass(0);
  auto* pm1 = softBody->getPointMass(1);
  ASSERT_NE(pm0, nullptr);
  ASSERT_NE(pm1, nullptr);

  pm0->setMass(0.02);
  EXPECT_NEAR(pm0->getMass(), 0.02, 1e-12);
  EXPECT_EQ(pm0->getIndexInSoftBodyNode(), 0u);

  const auto initialConnections = pm0->getNumConnectedPointMasses();
  pm0->addConnectedPointMass(pm1);
  EXPECT_EQ(pm0->getNumConnectedPointMasses(), initialConnections + 1u);
  EXPECT_EQ(pm0->getConnectedPointMass(initialConnections), pm1);
  EXPECT_EQ(pm0->getParentSoftBodyNode(), softBody);
}

TEST(PointMass, RestingAndWorldPositions)
{
  auto skeleton = Skeleton::create("point-mass-rest");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      4.0,
      4.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  pm->setRestingPosition(Eigen::Vector3d(0.05, -0.02, 0.01));
  pm->setPositions(Eigen::Vector3d(0.1, 0.2, -0.1));

  EXPECT_TRUE(pm->getLocalPosition().allFinite());
  EXPECT_TRUE(pm->getWorldPosition().allFinite());
}

TEST(PointMass, ConstraintImpulseFrames)
{
  auto skeleton = Skeleton::create("point-mass-impulse");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      4.0,
      4.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  pm->setConstraintImpulse(Eigen::Vector3d(0.3, -0.1, 0.2), false);
  EXPECT_TRUE(pm->getConstraintImpulses().allFinite());

  pm->setConstraintImpulse(Eigen::Vector3d(-0.2, 0.4, 0.1), true);
  EXPECT_TRUE(pm->getConstraintImpulses().allFinite());
}

TEST(PointMass, AddExtForceIgnoresInvalidValues)
{
  auto skeleton = Skeleton::create("point-mass-invalid-force");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.6),
      Eigen::Isometry3d::Identity(),
      1.0,
      4.0,
      4.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  const auto initialForces = pm->getForces();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  pm->addExtForce(Eigen::Vector3d(nan, 0.0, 0.0));
  pm->addExtForce(Eigen::Vector3d(0.0, inf, 0.0));
  pm->addExtForce(Eigen::Vector3d(0.0, 0.0, -inf));

  EXPECT_TRUE(pm->getForces().isApprox(initialForces));
}

TEST(PointMass, RestingPositionNoChange)
{
  auto skeleton = Skeleton::create("point-mass-rest-nochange");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.4),
      Eigen::Isometry3d::Identity(),
      1.0,
      3.0,
      3.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  const Eigen::Vector3d restPos(0.02, -0.01, 0.03);
  pm->setRestingPosition(restPos);
  const auto saved = pm->getRestingPosition();
  pm->setRestingPosition(saved);

  EXPECT_TRUE(pm->getRestingPosition().isApprox(saved));
}

TEST(PointMass, CollidingFlag)
{
  auto skeleton = Skeleton::create("point-mass-collide");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.4),
      Eigen::Isometry3d::Identity(),
      1.0,
      3.0,
      3.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  pm->setColliding(true);
  EXPECT_TRUE(pm->isColliding());
  pm->setColliding(false);
  EXPECT_FALSE(pm->isColliding());
}

TEST(PointMass, ExternalForceLocalAndWorld)
{
  auto skeleton = Skeleton::create("point-mass-ext");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      4.0,
      4.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  const auto forces = pm->getForces();
  pm->addExtForce(Eigen::Vector3d(0.2, -0.1, 0.3), true);
  pm->addExtForce(Eigen::Vector3d(-0.1, 0.2, 0.0), false);
  pm->clearExtForce();
  EXPECT_TRUE(pm->getForces().isApprox(forces));
}

TEST(PointMass, ParentAccessorsAndVelocityChange)
{
  auto skeleton = Skeleton::create("point-mass-parent");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.4),
      Eigen::Isometry3d::Identity(),
      1.0,
      3.0,
      3.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  const PointMass* pmConst = pm;
  EXPECT_EQ(pmConst->getParentSoftBodyNode(), softBody);
  EXPECT_TRUE(
      pmConst->getBodyVelocityChange().isApprox(Eigen::Vector3d::Zero()));
}

TEST(PointMass, ConstrainedTermsAndPartialAcceleration)
{
  auto skeleton = Skeleton::create("point-mass-constraints");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.3),
      Eigen::Isometry3d::Identity(),
      1.0,
      6.0,
      6.0,
      0.1);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  pm->setVelocities(Eigen::Vector3d(0.2, -0.1, 0.05));
  const auto partial = pm->getPartialAccelerations();
  EXPECT_TRUE(partial.allFinite());

  pm->setConstraintImpulse(Eigen::Vector3d(0.2, -0.1, 0.3), true);
  EXPECT_TRUE(pm->getConstraintImpulses().allFinite());
  ASSERT_NE(softBody->getNotifier(), nullptr);
  EXPECT_FALSE(softBody->getNotifier()->getName().empty());
}

TEST(PointMass, RestingPositionAndAccelerationSetters)
{
  auto skeleton = Skeleton::create("point-mass-rest-accel");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.4),
      Eigen::Isometry3d::Identity(),
      1.0,
      4.0,
      4.0,
      0.2);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  const Eigen::Vector3d restPos(0.03, -0.02, 0.01);
  pm->setRestingPosition(restPos);
  EXPECT_TRUE(pm->getRestingPosition().isApprox(restPos));

  pm->setAcceleration(1, 0.75);
  EXPECT_NEAR(pm->getAcceleration(1), 0.75, 1e-12);

  const Eigen::Vector3d acc(0.2, -0.1, 0.05);
  pm->setAccelerations(acc);
  EXPECT_TRUE(pm->getAccelerations().isApprox(acc));
}

TEST(PointMass, PartialAccelerationAfterWorldStep)
{
  auto world = simulation::World::create();
  auto skeleton = Skeleton::create("point-mass-step");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      5.0,
      5.0,
      0.1);

  world->addSkeleton(skeleton);
  world->step();

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);
  EXPECT_TRUE(pm->getPartialAccelerations().allFinite());
  EXPECT_TRUE(pm->getBodyVelocityChange().allFinite());
}

TEST(PointMass, ConstAccessorsAndPsiValues)
{
  auto world = simulation::World::create();
  auto skeleton = Skeleton::create("point-mass-const");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.6),
      Eigen::Isometry3d::Identity(),
      1.0,
      5.0,
      5.0,
      0.1);

  ASSERT_GE(softBody->getNumPointMasses(), 2u);
  softBody->connectPointMasses(0, 1);

  world->addSkeleton(skeleton);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->step();

  const PointMass* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  EXPECT_TRUE(std::isfinite(pm->getPsi()));
  EXPECT_TRUE(std::isfinite(pm->getImplicitPsi()));
  EXPECT_TRUE(std::isfinite(pm->getPi()));
  EXPECT_TRUE(std::isfinite(pm->getImplicitPi()));

  EXPECT_TRUE(pm->getWorldPosition().allFinite());
  EXPECT_TRUE(pm->getWorldVelocity().allFinite());
  EXPECT_TRUE(pm->getWorldAcceleration().allFinite());

  const PointMass* connected = pm->getConnectedPointMass(0);
  EXPECT_EQ(connected, softBody->getPointMass(1));
}
