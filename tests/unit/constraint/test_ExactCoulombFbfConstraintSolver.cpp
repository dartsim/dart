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

#include <dart/constraint/ConstrainedGroup.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

#include <cmath>

namespace {

class ExposedExactCoulombFbfConstraintSolver final
  : public dart::constraint::ExactCoulombFbfConstraintSolver
{
public:
  using ExactCoulombFbfConstraintSolver::ExactCoulombFbfConstraintSolver;
  using ExactCoulombFbfConstraintSolver::solveConstrainedGroup;
};

class ContactLikeConstraint final : public dart::constraint::ConstraintBase
{
public:
  ContactLikeConstraint(
      Eigen::Index offset,
      const Eigen::MatrixXd& delassus,
      const Eigen::Vector3d& rhs,
      double primaryCoefficient,
      double secondaryCoefficient,
      std::shared_ptr<Eigen::Index> activeRow)
    : mOffset(offset),
      mDelassus(delassus),
      mRhs(rhs),
      mPrimaryCoefficient(primaryCoefficient),
      mSecondaryCoefficient(secondaryCoefficient),
      mActiveRow(std::move(activeRow))
  {
    mDim = 3u;
    mAppliedImpulse.setZero();
  }

  void update() override {}

  void getInformation(dart::constraint::ConstraintInfo* info) override
  {
    info->x[0] = 0.0;
    info->x[1] = 0.0;
    info->x[2] = 0.0;

    info->lo[0] = 0.0;
    info->lo[1] = -mPrimaryCoefficient;
    info->lo[2] = -mSecondaryCoefficient;

    info->hi[0] = std::numeric_limits<double>::infinity();
    info->hi[1] = mPrimaryCoefficient;
    info->hi[2] = mSecondaryCoefficient;

    info->b[0] = mRhs[0];
    info->b[1] = mRhs[1];
    info->b[2] = mRhs[2];

    info->w[0] = 0.0;
    info->w[1] = 0.0;
    info->w[2] = 0.0;

    info->findex[0] = -1;
    info->findex[1] = 0;
    info->findex[2] = 0;
  }

  void applyUnitImpulse(std::size_t index) override
  {
    *mActiveRow = mOffset + static_cast<Eigen::Index>(index);
  }

  void getVelocityChange(double* velocityChange, bool) override
  {
    Eigen::Map<Eigen::Vector3d> velocity(velocityChange);
    velocity.setZero();
    if (*mActiveRow < 0) {
      return;
    }

    velocity = mDelassus.block<1, 3>(*mActiveRow, mOffset).transpose();
  }

  void excite() override {}

  void unexcite() override
  {
    *mActiveRow = -1;
  }

  void applyImpulse(double* lambda) override
  {
    mAppliedImpulse = Eigen::Map<Eigen::Vector3d>(lambda);
  }

  bool isActive() const override
  {
    return true;
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  const Eigen::Vector3d& getAppliedImpulse() const
  {
    return mAppliedImpulse;
  }

private:
  Eigen::Index mOffset;
  Eigen::MatrixXd mDelassus;
  Eigen::Vector3d mRhs;
  double mPrimaryCoefficient;
  double mSecondaryCoefficient;
  std::shared_ptr<Eigen::Index> mActiveRow;
  Eigen::Vector3d mAppliedImpulse;
};

class ScalarConstraint final : public dart::constraint::ConstraintBase
{
public:
  ScalarConstraint()
  {
    mDim = 1u;
  }

  void update() override {}

  void getInformation(dart::constraint::ConstraintInfo* info) override
  {
    info->x[0] = 0.0;
    info->lo[0] = 0.0;
    info->hi[0] = std::numeric_limits<double>::infinity();
    info->b[0] = -1.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override
  {
    mImpulseApplied = true;
  }

  void getVelocityChange(double* velocityChange, bool) override
  {
    velocityChange[0] = mImpulseApplied ? 1.0 : 0.0;
  }

  void excite() override
  {
    mImpulseApplied = false;
  }

  void unexcite() override
  {
    mImpulseApplied = false;
  }

  void applyImpulse(double* lambda) override
  {
    mApplied = true;
    mAppliedImpulse = lambda[0];
  }

  bool isActive() const override
  {
    return true;
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  bool wasApplied() const
  {
    return mApplied;
  }

  double getAppliedImpulse() const
  {
    return mAppliedImpulse;
  }

private:
  bool mImpulseApplied = false;
  bool mApplied = false;
  double mAppliedImpulse = 0.0;
};

dart::constraint::ConstrainedGroup makeGroup(
    const std::shared_ptr<dart::constraint::ConstraintBase>& constraint)
{
  dart::constraint::ConstrainedGroup group;
  group.addConstraint(constraint);
  return group;
}

} // namespace

TEST(ExactCoulombFbfConstraintSolver, SolvesSupportedContactGroup)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.seedNormalImpulseFromDiagonal = false;
  options.maxResidualHistorySamples = 128;
  options.maxResidualHistoryRecords = 4;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombProjectedGradientRetries(), 0u);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_TRUE(std::isfinite(solver.getLastExactCoulombBestResidual()));
  EXPECT_LE(
      solver.getLastExactCoulombBestResidual(),
      solver.getLastExactCoulombResidual());
  EXPECT_LE(
      solver.getLastExactCoulombBestIteration(),
      solver.getLastExactCoulombIterations());
  EXPECT_NEAR(solver.getLastExactCoulombResidualDetails().value, 0.0, 1e-10);
  EXPECT_NEAR(
      solver.getLastExactCoulombResidualDetails().primalFeasibility,
      0.0,
      1e-10);
  EXPECT_NEAR(
      solver.getLastExactCoulombResidualDetails().dualFeasibility, 0.0, 1e-10);
  EXPECT_NEAR(
      solver.getLastExactCoulombResidualDetails().complementarity, 0.0, 1e-10);
  const auto& residualHistory = solver.getLastExactCoulombResidualHistory();
  ASSERT_GE(residualHistory.size(), 2u);
  EXPECT_EQ(residualHistory.front().iteration, 0);
  EXPECT_EQ(
      residualHistory.back().iteration, solver.getLastExactCoulombIterations());
  EXPECT_GT(
      residualHistory.front().residual.value,
      residualHistory.back().residual.value);
  EXPECT_NEAR(
      residualHistory.back().residual.value,
      solver.getLastExactCoulombResidual(),
      1e-10);

  const auto& residualHistoryRecords
      = solver.getExactCoulombResidualHistoryRecords();
  ASSERT_EQ(residualHistoryRecords.size(), 1u);
  EXPECT_EQ(residualHistoryRecords.front().solveIndex, 0u);
  EXPECT_EQ(residualHistoryRecords.front().contactCount, 1u);
  EXPECT_EQ(
      residualHistoryRecords.front().status,
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(
      residualHistoryRecords.front().fbfStatus,
      dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_EQ(
      residualHistoryRecords.front().iterations,
      solver.getLastExactCoulombIterations());
  ASSERT_EQ(
      residualHistoryRecords.front().samples.size(), residualHistory.size());
  EXPECT_NEAR(
      residualHistoryRecords.front().samples.back().residual.value,
      solver.getLastExactCoulombResidual(),
      1e-10);

  solver.clearExactCoulombResidualHistoryRecords();
  EXPECT_TRUE(solver.getExactCoulombResidualHistoryRecords().empty());

  EXPECT_TRUE(std::isfinite(solver.getLastExactCoulombStepSize()));
  EXPECT_TRUE(std::isfinite(solver.getLastExactCoulombSafeStepSize()));
  EXPECT_TRUE(
      std::isfinite(solver.getLastExactCoulombCouplingVariationRatio()));
  EXPECT_GE(solver.getLastExactCoulombShrinkIterations(), 0);
  EXPECT_EQ(
      solver.getMaxExactCoulombIterations(),
      solver.getLastExactCoulombIterations());
  EXPECT_EQ(
      solver.getTotalExactCoulombIterations(),
      static_cast<std::size_t>(solver.getLastExactCoulombIterations()));
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    CanUseMatrixFreeDelassusOperatorForSupportedContactGroup)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.useMatrixFreeDelassusOperator = true;
  options.useMatrixFreeDelassusSeed = true;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombMatrixFreeDelassusOperatorUsed());
  EXPECT_TRUE(solver.getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    RetriesProjectedGradientWhenBlockGaussSeidelFails)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.innerMaxSweeps = 0;
  options.projectedGradientMaxIterations = 200;
  options.projectedGradientTolerance = 1e-14;
  options.seedNormalImpulseFromDiagonal = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombProjectedGradientRetries(), 1u);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_NEAR(solver.getLastExactCoulombResidualDetails().value, 0.0, 1e-10);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    MatrixFreeDelassusSeedDiagnosticYieldsToWarmStart)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.useMatrixFreeDelassusOperator = true;
  options.useMatrixFreeDelassusSeed = true;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());

  options.maxOuterIterations = 0;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 1u);
}

TEST(ExactCoulombFbfConstraintSolver, ReusesWarmStartForSameConstraintSequence)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 0u);

  options.maxOuterIterations = 0;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 1u);
  EXPECT_EQ(solver.getLastExactCoulombIterations(), 0);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 2u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
}

TEST(ExactCoulombFbfConstraintSolver, WarmStartRequiresSameConstraintSequence)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto first = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto firstGroup = makeGroup(first);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  auto second = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto secondGroup = makeGroup(second);

  options.maxOuterIterations = 0;
  options.fallbackToBoxedLcp = false;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableDenseResidualPolish = false;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(secondGroup);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombIterations(),
      solver.getLastExactCoulombIterations());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombStepSize(),
      solver.getLastExactCoulombStepSize());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombSafeStepSize(),
      solver.getLastExactCoulombSafeStepSize());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombCouplingVariationRatio(),
      solver.getLastExactCoulombCouplingVariationRatio());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombShrinkIterations(),
      solver.getLastExactCoulombShrinkIterations());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombResidual(),
      solver.getLastExactCoulombResidual());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombResidualDetails().value,
      solver.getLastExactCoulombResidualDetails().value);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombBestResidual(),
      solver.getLastExactCoulombBestResidual());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombBestIteration(),
      solver.getLastExactCoulombBestIteration());
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_NEAR(second->getAppliedImpulse().norm(), 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, DenseResidualPolishCanRecoverFailedSolve)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 0;
  options.fallbackToBoxedLcp = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = true;
  options.seedNormalImpulseFromDiagonal = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombDenseResidualPolishUsed());
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombDenseResidualPolishes(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(solver.getLastExactCoulombResidual(), options.tolerance);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-8);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, FallsBackForUnsupportedGroup)
{
  auto scalar = std::make_shared<ScalarConstraint>();
  auto group = makeGroup(scalar);

  ExposedExactCoulombFbfConstraintSolver solver;
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          BoxedLcpFallback);
  EXPECT_EQ(
      solver.getLastExactCoulombBuildStatus(),
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::
          UnsupportedDimension);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 1u);
  EXPECT_TRUE(scalar->wasApplied());
  EXPECT_TRUE(std::isfinite(scalar->getAppliedImpulse()));
}

TEST(ExactCoulombFbfConstraintSolver, CanDisableFallback)
{
  auto scalar = std::make_shared<ScalarConstraint>();
  auto group = makeGroup(scalar);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          UnsupportedProblem);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_FALSE(scalar->wasApplied());
}

TEST(ExactCoulombFbfConstraintSolver, RejectsInvalidStepSizeScale)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.stepSizeScale = 0.0;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
  EXPECT_NEAR(contact->getAppliedImpulse().norm(), 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, RejectsInvalidOuterRelaxation)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.outerRelaxation = 0.0;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
  EXPECT_NEAR(contact->getAppliedImpulse().norm(), 0.0, 1e-12);
}
