// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>

#include <gtest/gtest.h>

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
