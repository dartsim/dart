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

#include <dart/simulation/experimental/detail/deformable_vbd/rigid_block_kernel.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
Eigen::Quaterniond rotationZ(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitZ()));
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
TEST(AvbdRigidBlock, SolveRejectsIndefiniteHessian)
{
  vbd::AvbdRigidBodyBlock block;
  block.force.setOnes();
  block.hessian.diagonal().array() = -1.0;

  EXPECT_NEAR(vbd::solveAvbdRigidBodyBlock(block).norm(), 0.0, 1e-12);
}
