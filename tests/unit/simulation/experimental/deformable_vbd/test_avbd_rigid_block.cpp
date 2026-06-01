/*
 * Copyright (c) 2011, The DART development contributors
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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/rigid_block_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/rigid_world_contact.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include <cmath>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;
namespace sx = dart::simulation::experimental;

//==============================================================================
Eigen::Quaterniond rotationZ(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitZ()));
}

//==============================================================================
std::size_t findEntityIndex(
    const std::vector<entt::entity>& entities, entt::entity entity)
{
  const auto it = std::find(entities.begin(), entities.end(), entity);
  EXPECT_NE(it, entities.end());
  return static_cast<std::size_t>(std::distance(entities.begin(), it));
}

} // namespace

//==============================================================================
TEST(AvbdRigidBlock, RigidStepUpdatesTranslationAndOrientation)
{
  vbd::AvbdRigidBodyState state;

  vbd::Vector6d step = vbd::Vector6d::Zero();
  step.head<3>() = Vec3(1.0, -2.0, 3.0);
  step.tail<3>() = 0.5 * vbd::kAvbdRigidPi * Vec3::UnitZ();

  vbd::applyAvbdRigidBodyStep(state, step);

  EXPECT_DOUBLE_EQ(state.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.position.y(), -2.0);
  EXPECT_DOUBLE_EQ(state.position.z(), 3.0);
  EXPECT_NEAR(
      (state.orientation.toRotationMatrix()
       - rotationZ(0.5 * vbd::kAvbdRigidPi).toRotationMatrix())
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(state.orientation.norm(), 1.0, 1e-15);
}

//==============================================================================
TEST(AvbdRigidBlock, InertiaTermSolvesBackToInertialTarget)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(1.0, -2.0, 0.5);
  state.orientation = rotationZ(0.2);

  vbd::AvbdRigidBodyState target;
  target.position = Vec3::Zero();
  target.orientation = Eigen::Quaterniond::Identity();

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia.diagonal() = Vec3(4.0, 5.0, 6.0);

  vbd::AvbdRigidBodyBlock block;
  vbd::addAvbdRigidBodyInertiaTerm(
      block, /*mass=*/2.0, inertia, /*timeStep=*/0.5, state, target);
  const vbd::Vector6d step = vbd::solveAvbdRigidBodyBlock(block);

  EXPECT_NEAR((step.head<3>() + state.position).norm(), 0.0, 1e-12);
  EXPECT_NEAR(step.tail<3>().x(), 0.0, 1e-12);
  EXPECT_NEAR(step.tail<3>().y(), 0.0, 1e-12);
  EXPECT_NEAR(step.tail<3>().z(), -0.2, 1e-12);

  vbd::applyAvbdRigidBodyStep(state, step);
  EXPECT_NEAR(state.position.norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (state.orientation.toRotationMatrix() - Eigen::Matrix3d::Identity())
          .norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointAttachmentStampsForceTorqueAndPsdHessian)
{
  vbd::AvbdRigidBodyState state;

  vbd::AvbdRigidPointAttachmentRow row;
  row.localPoint = Vec3::UnitY();
  row.target = Vec3(1.0, 1.0, 0.0);
  row.axis = Vec3::UnitX();
  row.state.stiffness = 40.0;

  vbd::AvbdRigidBodyBlock block;
  const double forceMagnitude
      = vbd::addAvbdRigidPointAttachment(block, state, row, /*alpha=*/0.0);

  vbd::Vector6d expectedDirection = vbd::Vector6d::Zero();
  expectedDirection.head<3>() = Vec3::UnitX();
  expectedDirection.tail<3>() = Vec3(0.0, 0.0, -1.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 40.0);
  EXPECT_NEAR((block.force - 40.0 * expectedDirection).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (block.hessian - 40.0 * expectedDirection * expectedDirection.transpose())
          .norm(),
      0.0,
      1e-12);
  EXPECT_GE(
      block.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointAttachmentDualUpdateGrowsInsideBounds)
{
  vbd::AvbdRigidBodyState state;

  vbd::AvbdRigidPointAttachmentRow row;
  row.target = Vec3::UnitX();
  row.axis = Vec3::UnitX();
  row.state.stiffness = 10.0;
  row.state.lambda = 1.0;

  vbd::AvbdRigidPointAttachmentOptions options;
  options.alpha = 0.0;
  options.beta = 3.0;

  const vbd::AvbdScalarRowState updated
      = vbd::updateAvbdRigidPointAttachmentRow(row.state, state, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 11.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 13.0);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairStampsEqualAndOppositeRigidDirections)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3::UnitX();

  vbd::AvbdRigidPointPairRow row;
  row.axis = Vec3::UnitX();
  row.state.stiffness = 25.0;

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPair(
      blockA, blockB, stateA, stateB, row, /*alpha=*/0.0);

  vbd::Vector6d expectedA = vbd::Vector6d::Zero();
  expectedA.head<3>() = Vec3::UnitX();

  vbd::Vector6d expectedB = vbd::Vector6d::Zero();
  expectedB.head<3>() = -Vec3::UnitX();

  EXPECT_DOUBLE_EQ(forceMagnitude, 25.0);
  EXPECT_NEAR((blockA.force - 25.0 * expectedA).norm(), 0.0, 1e-12);
  EXPECT_NEAR((blockB.force - 25.0 * expectedB).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockA.hessian - 25.0 * expectedA * expectedA.transpose()).norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (blockB.hessian - 25.0 * expectedB * expectedB.transpose()).norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairIncludesTorqueDirections)
{
  vbd::AvbdRigidBodyState stateA;
  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.0, 1.0, 0.0);

  vbd::AvbdRigidPointPairRow row;
  row.localPointA = Vec3::UnitY();
  row.localPointB = -Vec3::UnitY();
  row.axis = Vec3::UnitX();

  const vbd::Vector6d firstDirection
      = vbd::avbdRigidPointPairDirectionA(stateA, row);
  const vbd::Vector6d secondDirection
      = vbd::avbdRigidPointPairDirectionB(stateB, row);

  EXPECT_NEAR((firstDirection.head<3>() - Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (firstDirection.tail<3>() - Vec3(0.0, 0.0, -1.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((secondDirection.head<3>() + Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (secondDirection.tail<3>() - Vec3(0.0, 0.0, -1.0)).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairDualUpdateUsesBounds)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3::UnitX();

  vbd::AvbdRigidPointPairRow row;
  row.axis = Vec3::UnitX();
  row.state.stiffness = 10.0;
  row.bounds.lower = 0.0;
  row.bounds.upper = 2.0;

  vbd::AvbdRigidPointAttachmentOptions options;
  options.beta = 4.0;

  const vbd::AvbdScalarRowState updated = vbd::updateAvbdRigidPointPairRow(
      row.state, stateA, stateB, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 2.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 10.0);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactNormalPointPairUsesGapOffsetAndBounds)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.1, 0.0, 0.0);

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 100.0;

  const vbd::AvbdRigidPointPairRow row = vbd::makeAvbdRigidContactNormalRow(
      Vec3::Zero(),
      Vec3::Zero(),
      -Vec3::UnitX(),
      /*targetDistance=*/0.2,
      rowState);

  EXPECT_DOUBLE_EQ(row.bounds.lower, 0.0);
  EXPECT_TRUE(std::isinf(row.bounds.upper));
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(stateA, stateB, row), 0.1, 1e-12);

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPair(
      blockA, blockB, stateA, stateB, row, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 10.0);
  EXPECT_NEAR(
      (blockA.force.head<3>() - Vec3(-10.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockB.force.head<3>() - Vec3(10.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockA.force.tail<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockB.force.tail<3>().norm(), 0.0, 1e-12);
  EXPECT_GE(
      blockA.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);
  EXPECT_GE(
      blockB.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);

  stateB.position = Vec3(0.3, 0.0, 0.0);
  const vbd::AvbdScalarRowState separated = vbd::updateAvbdRigidPointPairRow(
      row.state, stateA, stateB, row, vbd::AvbdRigidPointAttachmentOptions{});
  EXPECT_DOUBLE_EQ(separated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(separated.stiffness, row.state.stiffness);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactFrictionPointPairUsesLaggedRelativeOffset)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.0, 0.4, 0.0);

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 100.0;

  const vbd::AvbdRigidPointPairRow row
      = vbd::makeAvbdRigidContactFrictionTangentRow(
          Vec3::Zero(),
          Vec3::Zero(),
          2.0 * Vec3::UnitY(),
          Vec3(0.0, 0.1, 0.0),
          /*forceLimit=*/5.0,
          rowState);

  EXPECT_NEAR(row.axis.norm(), 1.0, 1e-12);
  EXPECT_NEAR(row.offset, -0.1, 1e-12);
  EXPECT_DOUBLE_EQ(row.bounds.lower, -5.0);
  EXPECT_DOUBLE_EQ(row.bounds.upper, 5.0);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(stateA, stateB, row), 0.3, 1e-12);

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPair(
      blockA, blockB, stateA, stateB, row, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 5.0);
  EXPECT_NEAR(
      (blockA.force.head<3>() - Vec3(0.0, 5.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockB.force.head<3>() - Vec3(0.0, -5.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockA.force.tail<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockB.force.tail<3>().norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactFrictionPointPairProjectsStaticForceToCone)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(1.0, 1.0, 0.0);

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 10.0;

  vbd::AvbdRigidPointPairRow rowX = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitX(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);
  vbd::AvbdRigidPointPairRow rowY = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitY(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);

  vbd::AvbdRigidPointPairFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_TRUE(
      vbd::avbdRigidPointPairFrictionPreviousDualInsideCone(rowX, rowY));
  bool clamped = false;
  const Eigen::Vector2d force = vbd::avbdRigidPointPairFrictionTangentPairForce(
      stateA, stateB, rowX, rowY, options, &clamped);

  EXPECT_TRUE(clamped);
  EXPECT_NEAR(force.norm(), 5.0, 1e-12);
  EXPECT_NEAR(force.x(), force.y(), 1e-12);
  EXPECT_GT(force.x(), 0.0);

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const Eigen::Vector2d stampedForce
      = vbd::addAvbdRigidPointPairFrictionTangentPair(
          blockA, blockB, stateA, stateB, rowX, rowY, options);

  EXPECT_NEAR((stampedForce - force).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockA.force.head<3>() - Vec3(force.x(), force.y(), 0.0)).norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (blockB.force.head<3>() + Vec3(force.x(), force.y(), 0.0)).norm(),
      0.0,
      1e-12);
  EXPECT_GE(
      blockA.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);
  EXPECT_GE(
      blockB.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);

  vbd::updateAvbdRigidPointPairFrictionTangentPair(
      rowX, rowY, stateA, stateB, options);
  EXPECT_NEAR(std::hypot(rowX.state.lambda, rowY.state.lambda), 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactFrictionPointPairSwitchesToDynamicSlipDirection)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.0, 2.0, 0.0);

  vbd::AvbdScalarRowState rowStateX;
  rowStateX.stiffness = 10.0;
  rowStateX.lambda = 5.0;
  vbd::AvbdRigidPointPairRow rowX = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitX(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowStateX);

  vbd::AvbdScalarRowState rowStateY;
  rowStateY.stiffness = 20.0;
  vbd::AvbdRigidPointPairRow rowY = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitY(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowStateY);

  vbd::AvbdRigidPointPairFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_FALSE(
      vbd::avbdRigidPointPairFrictionPreviousDualInsideCone(rowX, rowY));
  const Eigen::Vector2d force = vbd::avbdRigidPointPairFrictionTangentPairForce(
      stateA, stateB, rowX, rowY, options);
  EXPECT_NEAR(force.x(), 0.0, 1e-12);
  EXPECT_NEAR(force.y(), 5.0, 1e-12);

  vbd::updateAvbdRigidPointPairFrictionTangentPair(
      rowX, rowY, stateA, stateB, options);
  EXPECT_NEAR(rowX.state.lambda, 0.0, 1e-12);
  EXPECT_NEAR(rowY.state.lambda, 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 20.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverSeparatesContactPair)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = Vec3(0.5, 0.0, 0.0);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 100.0;

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs(1);
  pointPairs[0].bodyA = 0;
  pointPairs[0].bodyB = 1;
  pointPairs[0].row = vbd::makeAvbdRigidContactNormalRow(
      Vec3::Zero(),
      Vec3::Zero(),
      -Vec3::UnitX(),
      /*targetDistance=*/1.0,
      rowState);
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs;

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_NEAR(states[0].position.norm(), 0.0, 1e-12);
  EXPECT_GT(states[1].position.x(), 0.9);
  EXPECT_GT(pointPairs[0].row.state.lambda, 0.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverAppliesFrictionPair)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = Vec3(0.0, 1.0, 0.0);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 10.0;

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs(1);
  frictionPairs[0].bodyA = 0;
  frictionPairs[0].bodyB = 1;
  frictionPairs[0].first = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitX(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);
  frictionPairs[0].second = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitY(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_NEAR(states[1].position.x(), 0.0, 1e-12);
  EXPECT_LT(states[1].position.y(), 1.0);
  EXPECT_NEAR(frictionPairs[0].first.state.lambda, 0.0, 1e-12);
  EXPECT_NEAR(frictionPairs[0].second.state.lambda, 5.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverHonorsConvergenceDisplacement)
{
  std::vector<vbd::AvbdRigidBodyState> states(1);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0};
  const std::vector<Eigen::Matrix3d> inertias{Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {0u};

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 10.0;

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments(1);
  attachments[0].body = 0;
  attachments[0].row.target = Vec3::Zero();
  attachments[0].row.axis = Vec3::UnitX();
  attachments[0].row.state = rowState;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs;

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.convergenceDisplacement = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_NEAR(states[0].position.norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldBuilderCreatesWarmStartedRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(1);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].endpointA
      = {42,
         vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Vertex, 4)};
  contacts[0].endpointB = {
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 2)};
  contacts[0].point = Vec3::Zero();
  contacts[0].normalFromAtoB = 2.0 * Vec3::UnitX();
  contacts[0].depth = 0.2;
  contacts[0].frictionCoefficient = 0.5;
  contacts[0].startStiffness = 80.0;
  contacts[0].maxStiffness = 400.0;
  contacts[0].row = 3;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), 1u);
  EXPECT_EQ(
      normalInventory[0].descriptor.key.objectA, contacts[0].endpointB.object);
  EXPECT_EQ(
      normalInventory[0].descriptor.key.featureA,
      contacts[0].endpointB.feature);
  EXPECT_EQ(normalInventory[0].descriptor.key.row, 3u);
  EXPECT_DOUBLE_EQ(normalInventory[0].descriptor.startStiffness, 80.0);
  EXPECT_DOUBLE_EQ(normalInventory[0].descriptor.maxStiffness, 400.0);
  ASSERT_EQ(normalRows.size(), 1u);
  EXPECT_EQ(normalRows[0].bodyA, 0u);
  EXPECT_EQ(normalRows[0].bodyB, 1u);
  EXPECT_NEAR((normalRows[0].row.axis + Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(
          states[0], states[1], normalRows[0].row),
      0.2,
      1e-12);

  normalInventory[0].state.lambda = 8.0;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(frictionRows.size(), 1u);
  EXPECT_DOUBLE_EQ(frictionRows[0].first.bounds.lower, -4.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].first.bounds.upper, 4.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].second.bounds.lower, -4.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].second.bounds.upper, 4.0);
  EXPECT_NEAR(frictionRows[0].first.axis.dot(Vec3::UnitX()), 0.0, 1e-12);
  EXPECT_NEAR(frictionRows[0].second.axis.dot(Vec3::UnitX()), 0.0, 1e-12);
  EXPECT_NEAR(
      frictionRows[0].first.axis.dot(frictionRows[0].second.axis), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldBuilderSkipsInactiveRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(2);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].normalFromAtoB = Vec3::UnitX();
  contacts[0].depth = 0.0;
  contacts[1].bodyA = 0;
  contacts[1].bodyB = 1;
  contacts[1].normalFromAtoB = Vec3::Zero();
  contacts[1].depth = 0.1;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows);

  EXPECT_TRUE(normalInventory.empty());
  EXPECT_TRUE(frictionInventory.empty());
  EXPECT_TRUE(normalRows.empty());
  EXPECT_TRUE(frictionRows.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldRowsDriveSeparation)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(1);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  contacts[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  contacts[0].point = Vec3::Zero();
  contacts[0].normalFromAtoB = Vec3::UnitX();
  contacts[0].depth = 0.5;
  contacts[0].startStiffness = 100.0;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;
  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          normalRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_GT(states[1].position.x(), 0.4);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotBuildsManifoldRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));
  ground.setFriction(0.25);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 2.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.45);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(0.64);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  vbd::AvbdRigidWorldContactOptions options;
  options.startStiffness = 90.0;
  options.maxStiffness = 700.0;
  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          world.getRegistry(), contacts, options);

  ASSERT_EQ(snapshot.entities.size(), 2u);
  ASSERT_EQ(snapshot.states.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.masses.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.bodyInertias.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.fixed.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.contacts.size(), contacts.size());

  const std::size_t groundIndex
      = findEntityIndex(snapshot.entities, ground.getEntity());
  const std::size_t sphereIndex
      = findEntityIndex(snapshot.entities, sphere.getEntity());
  EXPECT_EQ(snapshot.fixed[groundIndex], 1u);
  EXPECT_EQ(snapshot.fixed[sphereIndex], 0u);
  EXPECT_DOUBLE_EQ(snapshot.masses[sphereIndex], 2.0);
  EXPECT_NEAR(
      (snapshot.states[sphereIndex].position - sphere.getTranslation()).norm(),
      0.0,
      1e-12);

  for (std::size_t i = 0; i < snapshot.contacts.size(); ++i) {
    const vbd::AvbdRigidContactManifoldPoint& manifoldPoint
        = snapshot.contacts[i];
    const sx::Contact& sourceContact = contacts[i];
    const std::size_t bodyA
        = findEntityIndex(snapshot.entities, sourceContact.bodyA.getEntity());
    const std::size_t bodyB
        = findEntityIndex(snapshot.entities, sourceContact.bodyB.getEntity());

    EXPECT_EQ(manifoldPoint.bodyA, bodyA);
    EXPECT_EQ(manifoldPoint.bodyB, bodyB);
    EXPECT_EQ(
        manifoldPoint.endpointA.object,
        vbd::avbdRigidWorldContactObjectId(sourceContact.bodyA.getEntity()));
    EXPECT_EQ(
        manifoldPoint.endpointB.object,
        vbd::avbdRigidWorldContactObjectId(sourceContact.bodyB.getEntity()));
    EXPECT_EQ(
        vbd::avbdContactFeatureKind(manifoldPoint.endpointA.feature),
        vbd::AvbdContactFeatureKind::Body);
    EXPECT_EQ(
        vbd::avbdContactFeatureKind(manifoldPoint.endpointB.feature),
        vbd::AvbdContactFeatureKind::Body);
    EXPECT_NEAR((manifoldPoint.point - sourceContact.point).norm(), 0.0, 1e-12);
    EXPECT_NEAR(
        (manifoldPoint.normalFromAtoB - sourceContact.normal).norm(),
        0.0,
        1e-12);
    EXPECT_DOUBLE_EQ(manifoldPoint.depth, sourceContact.depth);
    EXPECT_NEAR(manifoldPoint.frictionCoefficient, 0.4, 1e-12);
    EXPECT_DOUBLE_EQ(manifoldPoint.startStiffness, 90.0);
    EXPECT_DOUBLE_EQ(manifoldPoint.maxStiffness, 700.0);
    EXPECT_EQ(manifoldPoint.row, i);
  }

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::buildAvbdRigidContactManifoldRows(
      snapshot.states,
      snapshot.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows);

  ASSERT_EQ(normalRows.size(), snapshot.contacts.size());
  for (std::size_t i = 0; i < normalRows.size(); ++i) {
    const auto& row = normalRows[i];
    const Vec3 pointA = vbd::avbdRigidBodyWorldPoint(
        snapshot.states[row.bodyA], row.row.localPointA);
    const Vec3 pointB = vbd::avbdRigidBodyWorldPoint(
        snapshot.states[row.bodyB], row.row.localPointB);
    EXPECT_NEAR((pointA - snapshot.contacts[i].point).norm(), 0.0, 1e-12);
    EXPECT_NEAR((pointB - snapshot.contacts[i].point).norm(), 0.0, 1e-12);
  }
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotSolveMovesDynamicBody)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  vbd::AvbdRigidWorldContactOptions contactOptions;
  contactOptions.startStiffness = 200.0;
  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          world.getRegistry(), world.collide(), contactOptions);
  ASSERT_FALSE(snapshot.contacts.empty());

  const std::size_t sphereIndex
      = findEntityIndex(snapshot.entities, sphere.getEntity());
  const double initialSphereZ = snapshot.states[sphereIndex].position.z();

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 4;
  solveOptions.descent.convergenceDisplacement = 1e-12;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  const vbd::AvbdRigidWorldContactSolveResult result
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          /*timeStep=*/1.0,
          solveOptions);

  EXPECT_EQ(result.normalRows, snapshot.contacts.size());
  EXPECT_GT(result.stats.bodyUpdates, 0u);
  EXPECT_GT(snapshot.states[sphereIndex].position.z(), initialSphereZ);

  ASSERT_FALSE(normalInventory.empty());
  bool foundPositiveNormalDual = false;
  for (const vbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    foundPositiveNormalDual
        = foundPositiveNormalDual || record.state.lambda > 0.0;
  }
  EXPECT_TRUE(foundPositiveNormalDual);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotApplyWritesDynamicBodyState)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  vbd::AvbdRigidWorldContactOptions contactOptions;
  contactOptions.startStiffness = 200.0;
  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          world.getRegistry(), world.collide(), contactOptions);
  ASSERT_FALSE(snapshot.contacts.empty());

  const std::size_t sphereIndex
      = findEntityIndex(snapshot.entities, sphere.getEntity());
  const double initialSphereZ = sphere.getTransform().translation().z();

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 4;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  const vbd::AvbdRigidWorldContactSolveResult solveResult
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          /*timeStep=*/1.0,
          solveOptions);
  ASSERT_GT(solveResult.stats.bodyUpdates, 0u);
  ASSERT_GT(snapshot.states[sphereIndex].position.z(), initialSphereZ);

  const double writebackTimeStep = 0.5;
  const vbd::AvbdRigidWorldContactApplyResult applyResult
      = vbd::applyAvbdRigidWorldContactSnapshot(
          world.getRegistry(), snapshot, writebackTimeStep);

  EXPECT_EQ(applyResult.bodies, 1u);
  EXPECT_NEAR(
      sphere.getTransform().translation().z(),
      snapshot.states[sphereIndex].position.z(),
      1e-12);
  EXPECT_NEAR(
      sphere.getLinearVelocity().z(),
      (snapshot.states[sphereIndex].position.z() - initialSphereZ)
          / writebackTimeStep,
      1e-12);
  EXPECT_NEAR(ground.getTransform().translation().z(), -0.25, 1e-12);
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Vec3::Zero(), 1e-12));
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotSkipsStaticPairs)
{
  sx::World world;

  sx::RigidBodyOptions optionsA;
  optionsA.isStatic = true;
  auto bodyA = world.addRigidBody("static_a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions optionsB;
  optionsB.isStatic = true;
  optionsB.position = Vec3(0.25, 0.0, 0.0);
  auto bodyB = world.addRigidBody("static_b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(world.getRegistry(), contacts);

  EXPECT_TRUE(snapshot.entities.empty());
  EXPECT_TRUE(snapshot.contacts.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, SolveRejectsIndefiniteHessian)
{
  vbd::AvbdRigidBodyBlock block;
  block.force.setOnes();
  block.hessian.diagonal().array() = -1.0;

  EXPECT_NEAR(vbd::solveAvbdRigidBodyBlock(block).norm(), 0.0, 1e-12);
}
