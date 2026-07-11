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

#include "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"

#include "dart/common/Console.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/detail/ExactCoulombContactRowOperator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <vector>

#include <cmath>

namespace dart {
namespace constraint {
namespace {

bool isValidExactCoulombOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  const bool validInitialStep = std::isnan(options.initialStepSize)
                                || (std::isfinite(options.initialStepSize)
                                    && options.initialStepSize > 0.0);

  return options.maxOuterIterations >= 0 && std::isfinite(options.tolerance)
         && options.tolerance >= 0.0 && validInitialStep
         && std::isfinite(options.stepSizeScale) && options.stepSizeScale > 0.0
         && std::isfinite(options.outerRelaxation)
         && options.outerRelaxation > 0.0
         && std::isfinite(options.couplingVariationTolerance)
         && options.couplingVariationTolerance > 0.0
         && std::isfinite(options.shrinkFactor) && options.shrinkFactor > 0.0
         && options.shrinkFactor < 1.0 && options.maxStepShrinkIterations >= 0
         && options.spectralIterations > 0 && options.innerMaxSweeps >= 0
         && options.innerLocalIterations > 0
         && std::isfinite(options.innerTolerance)
         && options.innerTolerance >= 0.0
         && std::isfinite(options.innerLocalTolerance)
         && options.innerLocalTolerance >= 0.0
         && std::isfinite(options.innerDiagonalRegularization)
         && options.innerDiagonalRegularization >= 0.0
         && options.projectedGradientMaxIterations >= 0
         && std::isfinite(options.projectedGradientTolerance)
         && options.projectedGradientTolerance >= 0.0
         && options.denseResidualPolishIterations >= 0
         && options.denseResidualPolishLineSearchIterations >= 0
         && std::isfinite(options.denseResidualPolishRegularization)
         && options.denseResidualPolishRegularization >= 0.0
         && options.maxResidualHistorySamples >= 0
         && options.maxResidualHistoryRecords >= 0;
}

math::detail::CoulombConeResidual computeExactCoulombDenseResidual(
    const detail::ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const math::detail::CoulombConeResidualScales& scales)
{
  const auto applyDelassus = [&problem](
                                 const Eigen::Ref<const Eigen::VectorXd>& input,
                                 Eigen::Ref<Eigen::VectorXd> output) {
    output.noalias() = problem.delassus * input;
  };

  return math::detail::computeExactCoulombContactResidualNormalFirst(
      problem.contactProblem, reaction, applyDelassus, scales);
}

bool computeExactCoulombDenseDualCorrection(
    const detail::ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    Eigen::Ref<Eigen::VectorXd> correction)
{
  const Eigen::Index dimension = problem.contactProblem.getDimension();
  if (reaction.size() != dimension || correction.size() != dimension
      || !reaction.allFinite()) {
    return false;
  }

  Eigen::VectorXd velocity = problem.delassus * reaction;
  if (!velocity.allFinite()) {
    return false;
  }
  velocity += problem.contactProblem.freeVelocity;

  Eigen::VectorXd augmentedVelocity(dimension);
  if (!math::detail::computeExactCoulombAugmentedVelocityNormalFirst(
          velocity, problem.contactProblem.coefficients, augmentedVelocity)) {
    return false;
  }

  correction.setZero();
  for (Eigen::Index contact = 0;
       contact < problem.contactProblem.getContactCount();
       ++contact) {
    const Eigen::Index offset = 3 * contact;
    const Eigen::Vector3d augmented = augmentedVelocity.segment<3>(offset);
    const Eigen::Vector3d projected
        = math::detail::projectCoulombDualConeNormalFirst(
            augmented, problem.contactProblem.coefficients[contact]);
    correction.segment<3>(offset) = projected - augmented;
  }

  return correction.allFinite();
}

math::detail::ExactCoulombFbfOptions makeFbfOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  math::detail::ExactCoulombFbfOptions fbfOptions;
  fbfOptions.maxOuterIterations = options.maxOuterIterations;
  fbfOptions.tolerance = options.tolerance;
  fbfOptions.initialStepSize = options.initialStepSize;
  fbfOptions.stepSizeScale = options.stepSizeScale;
  fbfOptions.outerRelaxation = options.outerRelaxation;
  fbfOptions.couplingVariationTolerance = options.couplingVariationTolerance;
  fbfOptions.shrinkFactor = options.shrinkFactor;
  fbfOptions.maxStepShrinkIterations = options.maxStepShrinkIterations;
  fbfOptions.spectralIterations = options.spectralIterations;
  fbfOptions.maxResidualHistorySamples = options.maxResidualHistorySamples;
  return fbfOptions;
}

math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions
makeFrozenConeOptions(const ExactCoulombFbfConstraintSolverOptions& options)
{
  math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions frozenOptions;
  frozenOptions.maxSweeps = options.innerMaxSweeps;
  frozenOptions.localIterations = options.innerLocalIterations;
  frozenOptions.tolerance = options.innerTolerance;
  frozenOptions.localTolerance = options.innerLocalTolerance;
  frozenOptions.diagonalRegularization = options.innerDiagonalRegularization;
  return frozenOptions;
}

math::detail::ExactCoulombFrozenConeOptions makeProjectedGradientOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  math::detail::ExactCoulombFrozenConeOptions projectedOptions;
  projectedOptions.maxIterations = options.projectedGradientMaxIterations;
  projectedOptions.tolerance = options.projectedGradientTolerance;
  projectedOptions.spectralIterations = options.spectralIterations;
  return projectedOptions;
}

} // namespace

//==============================================================================
ExactCoulombFbfConstraintSolver::ExactCoulombFbfConstraintSolver() = default;

//==============================================================================
ExactCoulombFbfConstraintSolver::ExactCoulombFbfConstraintSolver(
    const ExactCoulombFbfConstraintSolverOptions& options)
  : mExactCoulombOptions(options)
{
  // Do nothing
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::setExactCoulombOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  if (!isValidExactCoulombOptions(options)) {
    dtwarn << "[ExactCoulombFbfConstraintSolver] Ignoring invalid exact "
           << "Coulomb FBF options.\n";
    return;
  }

  mExactCoulombOptions = options;
  if (!mExactCoulombOptions.enableWarmStart) {
    clearExactCoulombWarmStart();
  }
}

//==============================================================================
const ExactCoulombFbfConstraintSolverOptions&
ExactCoulombFbfConstraintSolver::getExactCoulombOptions() const
{
  return mExactCoulombOptions;
}

//==============================================================================
ExactCoulombFbfConstraintSolverStatus
ExactCoulombFbfConstraintSolver::getLastExactCoulombStatus() const
{
  return mLastExactCoulombStatus;
}

//==============================================================================
detail::ExactCoulombConstraintBuildStatus
ExactCoulombFbfConstraintSolver::getLastExactCoulombBuildStatus() const
{
  return mLastExactCoulombBuildStatus;
}

//==============================================================================
math::detail::ExactCoulombFbfStatus
ExactCoulombFbfConstraintSolver::getLastExactCoulombFbfStatus() const
{
  return mLastExactCoulombFbfStatus;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombResidual() const
{
  return mLastExactCoulombResidual;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombBestResidual() const
{
  return mLastExactCoulombBestResidual;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastExactCoulombBestIteration() const
{
  return mLastExactCoulombBestIteration;
}

//==============================================================================
const math::detail::CoulombConeResidual&
ExactCoulombFbfConstraintSolver::getLastExactCoulombResidualDetails() const
{
  return mLastExactCoulombResidualDetails;
}

const std::vector<math::detail::ExactCoulombFbfResidualSample>&
ExactCoulombFbfConstraintSolver::getLastExactCoulombResidualHistory() const
{
  return mLastExactCoulombResidualHistory;
}

//==============================================================================
const std::vector<ExactCoulombFbfResidualHistoryRecord>&
ExactCoulombFbfConstraintSolver::getExactCoulombResidualHistoryRecords() const
{
  return mExactCoulombResidualHistoryRecords;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::clearExactCoulombResidualHistoryRecords()
{
  mExactCoulombResidualHistoryRecords.clear();
  mResidualHistoryRecordSequence = 0u;
}

//==============================================================================
math::detail::ExactCoulombFbfStatus
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombFbfStatus() const
{
  return mLastFailedExactCoulombFbfStatus;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombResidual()
    const
{
  return mLastFailedExactCoulombResidual;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombBestResidual()
    const
{
  return mLastFailedExactCoulombBestResidual;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombBestIteration()
    const
{
  return mLastFailedExactCoulombBestIteration;
}

//==============================================================================
const math::detail::CoulombConeResidual&
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombResidualDetails()
    const
{
  return mLastFailedExactCoulombResidualDetails;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombIterations() const
{
  return mLastFailedExactCoulombIterations;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastExactCoulombIterations() const
{
  return mLastExactCoulombIterations;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombStepSize() const
{
  return mLastExactCoulombStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombSafeStepSize() const
{
  return mLastExactCoulombSafeStepSize;
}

//==============================================================================
double
ExactCoulombFbfConstraintSolver::getLastExactCoulombCouplingVariationRatio()
    const
{
  return mLastExactCoulombCouplingVariationRatio;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastExactCoulombShrinkIterations() const
{
  return mLastExactCoulombShrinkIterations;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombStepSize()
    const
{
  return mLastFailedExactCoulombStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombSafeStepSize()
    const
{
  return mLastFailedExactCoulombSafeStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::
    getLastFailedExactCoulombCouplingVariationRatio() const
{
  return mLastFailedExactCoulombCouplingVariationRatio;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombShrinkIterations()
    const
{
  return mLastFailedExactCoulombShrinkIterations;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getMaxExactCoulombIterations() const
{
  return mMaxExactCoulombIterations;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getTotalExactCoulombIterations()
    const
{
  return mTotalExactCoulombIterations;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombProjectedGradientRetryUsed() const
{
  return mLastExactCoulombProjectedGradientRetryUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombDenseResidualPolishUsed() const
{
  return mLastExactCoulombDenseResidualPolishUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombMatrixFreeDelassusOperatorUsed() const
{
  return mLastExactCoulombMatrixFreeDelassusOperatorUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombContactRowOperatorUsed() const
{
  return mLastExactCoulombContactRowOperatorUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombMatrixFreeDelassusSeedUsed() const
{
  return mLastExactCoulombMatrixFreeDelassusSeedUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::getLastExactCoulombWarmStartUsed() const
{
  return mLastExactCoulombWarmStartUsed;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getLastExactCoulombWarmStartMatchedContacts()
    const
{
  return mLastExactCoulombWarmStartMatchedContacts;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombSolves() const
{
  return mNumExactCoulombSolves;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumBoxedLcpFallbacks() const
{
  return mNumBoxedLcpFallbacks;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombFailures() const
{
  return mNumExactCoulombFailures;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombWarmStarts()
    const
{
  return mNumExactCoulombWarmStarts;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombProjectedGradientRetries()
    const
{
  return mNumExactCoulombProjectedGradientRetries;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombDenseResidualPolishes() const
{
  return mNumExactCoulombDenseResidualPolishes;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::solveConstrainedGroup(
    ConstrainedGroup& group)
{
  if (trySolveExactCoulombConstrainedGroup(group)) {
    return;
  }

  if (!mExactCoulombOptions.fallbackToBoxedLcp) {
    return;
  }

  ++mNumBoxedLcpFallbacks;
  mLastExactCoulombStatus
      = ExactCoulombFbfConstraintSolverStatus::BoxedLcpFallback;
  BoxedLcpConstraintSolver::solveConstrainedGroup(group);
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::trySolveExactCoulombConstrainedGroup(
    ConstrainedGroup& group)
{
  mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::NotRun;
  mLastExactCoulombBuildStatus
      = detail::ExactCoulombConstraintBuildStatus::EmptyInput;
  mLastExactCoulombFbfStatus
      = math::detail::ExactCoulombFbfStatus::InvalidInput;
  mLastExactCoulombResidual = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombResidualDetails
      = math::detail::makeInvalidCoulombConeResidual();
  mLastExactCoulombBestResidual = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombBestIteration = 0;
  mLastExactCoulombResidualHistory.clear();
  mLastExactCoulombIterations = 0;
  mLastExactCoulombStepSize = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombSafeStepSize = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombCouplingVariationRatio
      = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombShrinkIterations = 0;
  mLastExactCoulombProjectedGradientRetryUsed = false;
  mLastExactCoulombDenseResidualPolishUsed = false;
  mLastExactCoulombMatrixFreeDelassusOperatorUsed = false;
  mLastExactCoulombContactRowOperatorUsed = false;
  mLastExactCoulombMatrixFreeDelassusSeedUsed = false;
  mLastExactCoulombWarmStartUsed = false;
  mLastExactCoulombWarmStartMatchedContacts = 0u;

  if (!isValidExactCoulombOptions(mExactCoulombOptions)) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::InvalidOptions;
    clearExactCoulombWarmStart();
    recordLastFailedExactCoulombAttempt(
        math::detail::ExactCoulombFbfStatus::InvalidInput,
        mLastExactCoulombResidualDetails,
        mLastExactCoulombIterations);
    ++mNumExactCoulombFailures;
    return false;
  }

  if (!std::isfinite(mTimeStep) || mTimeStep <= 0.0) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::InvalidOptions;
    clearExactCoulombWarmStart();
    recordLastFailedExactCoulombAttempt(
        math::detail::ExactCoulombFbfStatus::InvalidInput,
        mLastExactCoulombResidualDetails,
        mLastExactCoulombIterations);
    ++mNumExactCoulombFailures;
    return false;
  }

  std::vector<ConstraintBase*> constraints;
  constraints.reserve(group.getNumConstraints());
  for (std::size_t i = 0u; i < group.getNumConstraints(); ++i) {
    constraints.push_back(group.getConstraint(i).get());
  }

  detail::ExactCoulombConstraintBuildOptions buildOptions;
  buildOptions.invTimeStep = 1.0 / mTimeStep;
  // Honor the solver's split-impulse setting: with split impulse enabled the
  // velocity-phase right-hand side excludes DART's ERP position-correction
  // bias (position recovery runs as a separate pseudo-impulse phase), which
  // matches the paper's formulation where the exact friction solve sees pure
  // dynamics.
  buildOptions.useSplitImpulse = isSplitImpulseEnabled();
  buildOptions.includeConstraintRegularization
      = mExactCoulombOptions.includeConstraintRegularization;
  const bool useMatrixFreeDelassusSeed
      = mExactCoulombOptions.seedNormalImpulseFromDiagonal
        && mExactCoulombOptions.useMatrixFreeDelassusOperator
        && mExactCoulombOptions.useMatrixFreeDelassusSeed;
  buildOptions.seedNormalImpulseFromDiagonal
      = mExactCoulombOptions.seedNormalImpulseFromDiagonal
        && !useMatrixFreeDelassusSeed;

  // The scratch-backed contact-row operator represents the pure reduced
  // operator `J M^-1 J^T`, so CFM/slip-regularized staging keeps the
  // impulse-test assembly route.
  const bool attemptContactRowOperator
      = mExactCoulombOptions.useContactRowDelassusOperator
        && !mExactCoulombOptions.includeConstraintRegularization;
  const bool deferDenseSeed
      = buildOptions.seedNormalImpulseFromDiagonal && attemptContactRowOperator;
  buildOptions.assembleDenseDelassus = !attemptContactRowOperator;
  buildOptions.seedNormalImpulseFromDiagonal
      = buildOptions.seedNormalImpulseFromDiagonal && !deferDenseSeed;

  auto problem
      = detail::buildExactCoulombConstraintProblem(constraints, buildOptions);
  mLastExactCoulombBuildStatus = problem.status;
  if (problem.status != detail::ExactCoulombConstraintBuildStatus::Success) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::UnsupportedProblem;
    invalidateExactCoulombWarmStartPointerCache();
    recordLastFailedExactCoulombAttempt(
        math::detail::ExactCoulombFbfStatus::InvalidInput,
        mLastExactCoulombResidualDetails,
        mLastExactCoulombIterations);
    ++mNumExactCoulombFailures;
    return false;
  }

  detail::ExactCoulombContactRowOperator contactRowOperator;
  if (attemptContactRowOperator) {
    bool rowOperatorReady = contactRowOperator.build(constraints)
                            && contactRowOperator.getDimension()
                                   == problem.contactProblem.getDimension();
    if (rowOperatorReady) {
      contactRowOperator.assembleDense(problem.delassus);
      rowOperatorReady = problem.delassus.allFinite();
    }

    if (!rowOperatorReady) {
      // Unsupported or degenerate row group: restore the general
      // impulse-test snapshot so every downstream path behaves as before.
      contactRowOperator.clear();
      if (!detail::assembleExactCoulombConstraintDelassusByImpulseTests(
              problem, mExactCoulombOptions.includeConstraintRegularization)) {
        mLastExactCoulombBuildStatus
            = detail::ExactCoulombConstraintBuildStatus::NonFiniteData;
        mLastExactCoulombStatus
            = ExactCoulombFbfConstraintSolverStatus::UnsupportedProblem;
        invalidateExactCoulombWarmStartPointerCache();
        recordLastFailedExactCoulombAttempt(
            math::detail::ExactCoulombFbfStatus::InvalidInput,
            mLastExactCoulombResidualDetails,
            mLastExactCoulombIterations);
        ++mNumExactCoulombFailures;
        return false;
      }
    }

    if (deferDenseSeed) {
      detail::seedExactCoulombImpulseFromDelassus(problem);
    }

    mLastExactCoulombContactRowOperatorUsed = rowOperatorReady;
  }

  const bool contactRowOperatorActive = mLastExactCoulombContactRowOperatorUsed;
  mLastExactCoulombMatrixFreeDelassusOperatorUsed
      = mExactCoulombOptions.useMatrixFreeDelassusOperator
        && !contactRowOperatorActive;

  const auto applyDelassus
      = [this, &problem, &contactRowOperator, contactRowOperatorActive](
            const Eigen::Ref<const Eigen::VectorXd>& input,
            Eigen::Ref<Eigen::VectorXd> output) {
          if (contactRowOperatorActive) {
            contactRowOperator.apply(input, output);
            return;
          }

          if (mExactCoulombOptions.useMatrixFreeDelassusOperator) {
            if (!detail::applyExactCoulombConstraintDelassus(
                    problem,
                    input,
                    output,
                    mExactCoulombOptions.includeConstraintRegularization)) {
              output.setConstant(std::numeric_limits<double>::quiet_NaN());
            }
            return;
          }

          output.noalias() = problem.delassus * input;
        };

  if (useMatrixFreeDelassusSeed && !contactRowOperatorActive) {
    mLastExactCoulombMatrixFreeDelassusSeedUsed
        = detail::seedExactCoulombImpulseFromDelassusOperator(
            problem, applyDelassus);
  }

  if (tryApplyExactCoulombWarmStart(constraints, problem)) {
    mLastExactCoulombMatrixFreeDelassusSeedUsed = false;
    mLastExactCoulombWarmStartUsed = true;
    ++mNumExactCoulombWarmStarts;
  }

  // The Delassus operator is fixed for one group solve, so extract the
  // per-contact 3x3 diagonal blocks once for every frozen-cone subproblem
  // instead of once per FBF outer iteration.
  const Eigen::Index frozenConeContactCount
      = problem.contactProblem.getContactCount();
  std::vector<Eigen::Matrix3d> frozenConeDiagonalBlocks;
  bool frozenConeDiagonalBlocksValid = false;
  if (contactRowOperatorActive) {
    frozenConeDiagonalBlocks.resize(
        static_cast<std::size_t>(frozenConeContactCount));
    for (Eigen::Index contact = 0; contact < frozenConeContactCount;
         ++contact) {
      frozenConeDiagonalBlocks[static_cast<std::size_t>(contact)]
          = contactRowOperator.diagonalBlock(contact);
    }
    frozenConeDiagonalBlocksValid = true;
  } else if (mLastExactCoulombMatrixFreeDelassusOperatorUsed) {
    frozenConeDiagonalBlocksValid
        = math::detail::computeExactCoulombDelassusDiagonalBlocksNormalFirst(
            problem.contactProblem, applyDelassus, frozenConeDiagonalBlocks);
  } else {
    frozenConeDiagonalBlocks.resize(
        static_cast<std::size_t>(frozenConeContactCount));
    for (Eigen::Index contact = 0; contact < frozenConeContactCount;
         ++contact) {
      frozenConeDiagonalBlocks[static_cast<std::size_t>(contact)]
          = problem.delassus.block<3, 3>(3 * contact, 3 * contact);
    }
    frozenConeDiagonalBlocksValid = true;
  }

  // Apply `accumulator += W.middleCols(3 * contact, 3) * delta` for the
  // incremental inner block Gauss-Seidel updates. The dense snapshot route
  // reads the columns directly; the opt-in constraint-row route stays on
  // full sparse-delta products until a scratch-backed operator lands.
  Eigen::VectorXd blockColumnBasis
      = Eigen::VectorXd::Zero(problem.contactProblem.getDimension());
  Eigen::VectorXd blockColumnProduct(problem.contactProblem.getDimension());
  const auto accumulateDelassusBlockColumns
      = [this,
         &problem,
         &applyDelassus,
         &contactRowOperator,
         contactRowOperatorActive,
         &blockColumnBasis,
         &blockColumnProduct](
            Eigen::Index contact,
            const Eigen::Vector3d& delta,
            Eigen::Ref<Eigen::VectorXd> accumulator) {
          if (contactRowOperatorActive) {
            contactRowOperator.accumulateBlockColumns(
                contact, delta, accumulator);
            return;
          }

          if (mExactCoulombOptions.useMatrixFreeDelassusOperator) {
            blockColumnBasis.segment<3>(3 * contact) = delta;
            applyDelassus(blockColumnBasis, blockColumnProduct);
            accumulator += blockColumnProduct;
            blockColumnBasis.segment<3>(3 * contact).setZero();
            return;
          }

          accumulator.noalias()
              += problem.delassus.middleCols<3>(3 * contact) * delta;
        };

  auto frozenOptions = makeFrozenConeOptions(mExactCoulombOptions);
  if (frozenConeDiagonalBlocksValid) {
    frozenOptions.cachedDiagonalBlocks = &frozenConeDiagonalBlocks;
  }
  const auto solveFrozenCone
      = [&applyDelassus, &accumulateDelassusBlockColumns, &frozenOptions](
            const auto& innerProblem,
            const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
            const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
            double stepSizeGamma,
            Eigen::Ref<Eigen::VectorXd> output) {
          const auto innerResult
              = math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
                  innerProblem,
                  referenceReaction,
                  frozenCoupling,
                  stepSizeGamma,
                  applyDelassus,
                  accumulateDelassusBlockColumns,
                  frozenOptions);
          output = innerResult.reaction;
          return innerResult.status
                 == math::detail::ExactCoulombFrozenConeStatus::Success;
        };

  auto solution = math::detail::solveExactCoulombFbf(
      problem.contactProblem,
      problem.initialGuess,
      applyDelassus,
      solveFrozenCone,
      makeFbfOptions(mExactCoulombOptions));

  if (solution.status != math::detail::ExactCoulombFbfStatus::Success
      && mExactCoulombOptions.enableProjectedGradientRetry) {
    mLastExactCoulombProjectedGradientRetryUsed = true;
    ++mNumExactCoulombProjectedGradientRetries;
    const auto projectedOptions
        = makeProjectedGradientOptions(mExactCoulombOptions);
    // The Delassus spectral estimate is deterministic and fixed for one group
    // solve, so compute it once for the retry instead of once per projected
    // frozen-cone subproblem.
    double projectedSpectralRadius = std::numeric_limits<double>::quiet_NaN();
    if (std::isnan(projectedOptions.stepSize)) {
      projectedSpectralRadius
          = math::detail::estimateLargestExactCoulombDelassusEigenvalue(
              problem.contactProblem,
              applyDelassus,
              projectedOptions.spectralIterations);
    }
    const auto solveProjectedFrozenCone
        = [&applyDelassus, &projectedOptions, projectedSpectralRadius](
              const auto& innerProblem,
              const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
              const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
              double stepSizeGamma,
              Eigen::Ref<Eigen::VectorXd> output) {
            auto localOptions = projectedOptions;
            if (std::isnan(localOptions.stepSize)
                && std::isfinite(projectedSpectralRadius)
                && std::isfinite(stepSizeGamma) && stepSizeGamma > 0.0) {
              const double lipschitz
                  = projectedSpectralRadius + 1.0 / stepSizeGamma;
              if (std::isfinite(lipschitz) && lipschitz > 0.0) {
                localOptions.stepSize = 1.0 / lipschitz;
              }
            }

            const auto innerResult
                = math::detail::solveExactCoulombFrozenConeProjectedGradient(
                    innerProblem,
                    referenceReaction,
                    frozenCoupling,
                    stepSizeGamma,
                    applyDelassus,
                    localOptions);
            output = innerResult.reaction;
            return innerResult.status
                   == math::detail::ExactCoulombFrozenConeStatus::Success;
          };

    const bool hasBestReaction
        = solution.bestReaction.size() == problem.initialGuess.size()
          && solution.bestReaction.allFinite()
          && std::isfinite(solution.bestResidual.value)
          && (!std::isfinite(solution.residual.value)
              || solution.bestResidual.value <= solution.residual.value);
    const Eigen::VectorXd& retryInitialReaction
        = hasBestReaction
              ? solution.bestReaction
              : (solution.reaction.size() == problem.initialGuess.size()
                         && solution.reaction.allFinite()
                     ? solution.reaction
                     : problem.initialGuess);

    const auto projectedSolution = math::detail::solveExactCoulombFbf(
        problem.contactProblem,
        retryInitialReaction,
        applyDelassus,
        solveProjectedFrozenCone,
        makeFbfOptions(mExactCoulombOptions));
    if (projectedSolution.status == math::detail::ExactCoulombFbfStatus::Success
        || projectedSolution.residual.value <= solution.residual.value) {
      solution = projectedSolution;
    }
  }

  if (solution.status != math::detail::ExactCoulombFbfStatus::Success) {
    tryPolishFailedExactCoulombSolution(problem, solution);
  }

  mLastExactCoulombFbfStatus = solution.status;
  mLastExactCoulombResidual = solution.residual.value;
  mLastExactCoulombResidualDetails = solution.residual;
  mLastExactCoulombBestResidual = solution.bestResidual.value;
  mLastExactCoulombBestIteration = solution.bestIteration;
  mLastExactCoulombResidualHistory = solution.residualHistory;
  mLastExactCoulombIterations = solution.iterations;
  mLastExactCoulombStepSize = solution.stepSize;
  mLastExactCoulombSafeStepSize = solution.safeStepSize;
  mLastExactCoulombCouplingVariationRatio = solution.couplingVariationRatio;
  mLastExactCoulombShrinkIterations = solution.shrinkIterations;
  if (solution.iterations > 0) {
    mMaxExactCoulombIterations
        = std::max(mMaxExactCoulombIterations, solution.iterations);
    mTotalExactCoulombIterations
        += static_cast<std::size_t>(solution.iterations);
  }
  if (solution.status != math::detail::ExactCoulombFbfStatus::Success) {
    mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::FbfFailed;
    recordExactCoulombResidualHistory(
        static_cast<std::size_t>(problem.contactProblem.getContactCount()),
        mLastExactCoulombStatus,
        solution);
    invalidateExactCoulombWarmStartPointerCache();
    recordLastFailedExactCoulombAttempt(
        solution.status,
        solution.residual,
        solution.iterations,
        solution.stepSize,
        solution.safeStepSize,
        solution.couplingVariationRatio,
        solution.shrinkIterations,
        solution.bestResidual.value,
        solution.bestIteration);
    ++mNumExactCoulombFailures;
    return false;
  }

  if (!detail::applyExactCoulombConstraintImpulses(
          problem, solution.reaction)) {
    mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::FbfFailed;
    recordExactCoulombResidualHistory(
        static_cast<std::size_t>(problem.contactProblem.getContactCount()),
        mLastExactCoulombStatus,
        solution);
    invalidateExactCoulombWarmStartPointerCache();
    recordLastFailedExactCoulombAttempt(
        solution.status,
        solution.residual,
        solution.iterations,
        solution.stepSize,
        solution.safeStepSize,
        solution.couplingVariationRatio,
        solution.shrinkIterations,
        solution.bestResidual.value,
        solution.bestIteration);
    ++mNumExactCoulombFailures;
    return false;
  }

  updateExactCoulombWarmStart(constraints, solution.reaction);
  ++mNumExactCoulombSolves;
  mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::Success;
  recordExactCoulombResidualHistory(
      static_cast<std::size_t>(problem.contactProblem.getContactCount()),
      mLastExactCoulombStatus,
      solution);
  return true;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::tryPolishFailedExactCoulombSolution(
    const detail::ExactCoulombConstraintProblem& problem,
    math::detail::ExactCoulombFbfResult& solution)
{
  if (!mExactCoulombOptions.enableDenseResidualPolish
      || mExactCoulombOptions.denseResidualPolishIterations <= 0
      || solution.residual.value <= mExactCoulombOptions.tolerance) {
    return;
  }

  const Eigen::Index dimension = problem.contactProblem.getDimension();
  Eigen::VectorXd reaction;
  if (solution.bestReaction.size() == dimension
      && solution.bestReaction.allFinite()) {
    reaction = solution.bestReaction;
  } else {
    reaction = solution.reaction;
  }
  if (reaction.size() != dimension || !reaction.allFinite()) {
    return;
  }

  if (!math::detail::projectExactCoulombReactionNormalFirst(
          reaction, problem.contactProblem.coefficients, reaction)) {
    return;
  }

  Eigen::MatrixXd symmetricDelassus
      = 0.5 * (problem.delassus + problem.delassus.transpose());
  if (!symmetricDelassus.allFinite()) {
    return;
  }

  const double maxDiagonal = symmetricDelassus.diagonal().cwiseAbs().maxCoeff();
  const double regularization
      = mExactCoulombOptions.denseResidualPolishRegularization
        * (std::max)(1.0, maxDiagonal);
  symmetricDelassus.diagonal().array() += regularization;

  Eigen::LDLT<Eigen::MatrixXd> factorization(symmetricDelassus);
  if (factorization.info() != Eigen::Success) {
    return;
  }

  auto bestResidual = computeExactCoulombDenseResidual(
      problem, reaction, solution.residualScales);
  if (!std::isfinite(bestResidual.value)
      || (std::isfinite(solution.bestResidual.value)
          && solution.bestResidual.value < bestResidual.value)) {
    bestResidual = solution.bestResidual;
  }

  bool attempted = false;
  Eigen::VectorXd dualCorrection(dimension);
  Eigen::VectorXd direction(dimension);
  Eigen::VectorXd candidate(dimension);
  Eigen::VectorXd localDirection = Eigen::VectorXd::Zero(dimension);
  for (int iteration = 0;
       iteration < mExactCoulombOptions.denseResidualPolishIterations;
       ++iteration) {
    if (!computeExactCoulombDenseDualCorrection(
            problem, reaction, dualCorrection)) {
      break;
    }

    if (dualCorrection.norm() == 0.0) {
      break;
    }

    direction = factorization.solve(dualCorrection);
    if (!direction.allFinite()) {
      break;
    }

    attempted = true;
    bool accepted = false;
    double step = 1.0;
    for (int lineSearch = 0;
         lineSearch
         <= mExactCoulombOptions.denseResidualPolishLineSearchIterations;
         ++lineSearch) {
      candidate = reaction + step * direction;
      if (!math::detail::projectExactCoulombReactionNormalFirst(
              candidate, problem.contactProblem.coefficients, candidate)) {
        break;
      }

      const auto candidateResidual = computeExactCoulombDenseResidual(
          problem, candidate, solution.residualScales);
      if (std::isfinite(candidateResidual.value)
          && candidateResidual.value < bestResidual.value) {
        reaction = candidate;
        bestResidual = candidateResidual;
        accepted = true;
        break;
      }

      step *= 0.5;
    }

    if (!accepted || bestResidual.value <= mExactCoulombOptions.tolerance) {
      break;
    }
  }

  for (int iteration = 0;
       iteration < mExactCoulombOptions.denseResidualPolishIterations
       && bestResidual.value > mExactCoulombOptions.tolerance;
       ++iteration) {
    const Eigen::Index contact = bestResidual.worstDualContact;
    if (contact < 0 || contact >= problem.contactProblem.getContactCount()) {
      break;
    }

    if (!computeExactCoulombDenseDualCorrection(
            problem, reaction, dualCorrection)) {
      break;
    }

    const Eigen::Index offset = 3 * contact;
    const Eigen::Vector3d localCorrection = dualCorrection.segment<3>(offset);
    if (localCorrection.norm() == 0.0) {
      break;
    }

    const Eigen::Matrix3d localBlock
        = symmetricDelassus.block<3, 3>(offset, offset);
    Eigen::LDLT<Eigen::Matrix3d> localFactorization(localBlock);
    if (localFactorization.info() != Eigen::Success) {
      break;
    }

    const Eigen::Vector3d localStep = localFactorization.solve(localCorrection);
    if (!localStep.allFinite()) {
      break;
    }

    localDirection.setZero();
    localDirection.segment<3>(offset) = localStep;
    attempted = true;

    bool accepted = false;
    double step = 1.0;
    for (int lineSearch = 0;
         lineSearch
         <= mExactCoulombOptions.denseResidualPolishLineSearchIterations;
         ++lineSearch) {
      candidate = reaction + step * localDirection;
      if (!math::detail::projectExactCoulombReactionNormalFirst(
              candidate, problem.contactProblem.coefficients, candidate)) {
        break;
      }

      const auto candidateResidual = computeExactCoulombDenseResidual(
          problem, candidate, solution.residualScales);
      if (std::isfinite(candidateResidual.value)
          && candidateResidual.value < bestResidual.value) {
        reaction = candidate;
        bestResidual = candidateResidual;
        accepted = true;
        break;
      }

      step *= 0.5;
    }

    if (!accepted) {
      break;
    }
  }

  if (!attempted) {
    return;
  }

  mLastExactCoulombDenseResidualPolishUsed = true;
  ++mNumExactCoulombDenseResidualPolishes;

  if (std::isfinite(bestResidual.value)
      && bestResidual.value < solution.bestResidual.value) {
    solution.bestResidual = bestResidual;
    solution.bestReaction = reaction;
    solution.bestIteration = solution.iterations;
  }

  if (std::isfinite(bestResidual.value)
      && bestResidual.value < solution.residual.value) {
    solution.reaction = reaction;
    solution.residual = bestResidual;
  }

  if (solution.residual.value <= mExactCoulombOptions.tolerance) {
    solution.status = math::detail::ExactCoulombFbfStatus::Success;
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::recordLastFailedExactCoulombAttempt(
    math::detail::ExactCoulombFbfStatus status,
    const math::detail::CoulombConeResidual& residual,
    int iterations,
    double stepSize,
    double safeStepSize,
    double couplingVariationRatio,
    int shrinkIterations,
    double bestResidual,
    int bestIteration)
{
  mLastFailedExactCoulombFbfStatus = status;
  mLastFailedExactCoulombResidual = residual.value;
  mLastFailedExactCoulombResidualDetails = residual;
  mLastFailedExactCoulombBestResidual = bestResidual;
  mLastFailedExactCoulombBestIteration = bestIteration;
  mLastFailedExactCoulombIterations = iterations;
  mLastFailedExactCoulombStepSize = stepSize;
  mLastFailedExactCoulombSafeStepSize = safeStepSize;
  mLastFailedExactCoulombCouplingVariationRatio = couplingVariationRatio;
  mLastFailedExactCoulombShrinkIterations = shrinkIterations;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::tryApplyExactCoulombWarmStart(
    const std::vector<ConstraintBase*>& constraints,
    detail::ExactCoulombConstraintProblem& problem)
{
  if (!mExactCoulombOptions.enableWarmStart || !mHasExactCoulombWarmStart) {
    return false;
  }

  // Fast path: identical constraint pointer sequence within one step.
  const bool pointerSequenceMatches
      = mWarmStartConstraints.size() == constraints.size()
        && mWarmStartReaction.size() == problem.initialGuess.size()
        && mWarmStartReaction.allFinite();
  if (pointerSequenceMatches) {
    bool identical = true;
    for (std::size_t i = 0u; i < constraints.size(); ++i) {
      if (mWarmStartConstraints[i] != constraints[i]) {
        identical = false;
        break;
      }
    }

    if (identical) {
      Eigen::VectorXd projected(problem.initialGuess.size());
      if (math::detail::projectExactCoulombReactionNormalFirst(
              mWarmStartReaction,
              problem.contactProblem.coefficients,
              projected)) {
        problem.initialGuess = projected;
        mLastExactCoulombWarmStartMatchedContacts = constraints.size();
        return true;
      }
    }
  }

  // Manifold path: contact constraints are recreated every `World` step, so
  // match cached contacts by (body pair, nearest world contact point) and
  // re-project the cached world-space reaction onto the current frames.
  const double matchDistance = mExactCoulombOptions.warmStartMatchDistance;
  if (mWarmStartContactRecords.empty() || !std::isfinite(matchDistance)
      || matchDistance <= 0.0) {
    return false;
  }

  std::vector<bool> recordUsed(mWarmStartContactRecords.size(), false);
  std::size_t matchedContacts = 0u;
  for (std::size_t i = 0u; i < constraints.size(); ++i) {
    detail::ExactCoulombContactRowOperator::ContactFrame frame;
    if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraints[i], frame)) {
      continue;
    }

    int bestRecord = -1;
    double bestDistance = matchDistance;
    bool bestFlipped = false;
    for (std::size_t r = 0u; r < mWarmStartContactRecords.size(); ++r) {
      if (recordUsed[r]) {
        continue;
      }

      const auto& record = mWarmStartContactRecords[r];
      const bool sameOrder
          = record.bodyA == frame.bodyA && record.bodyB == frame.bodyB;
      const bool flippedOrder
          = record.bodyA == frame.bodyB && record.bodyB == frame.bodyA;
      if (!sameOrder && !flippedOrder) {
        continue;
      }

      const double distance = (record.point - frame.point).norm();
      if (distance <= bestDistance) {
        bestDistance = distance;
        bestRecord = static_cast<int>(r);
        bestFlipped = !sameOrder;
      }
    }

    if (bestRecord < 0) {
      continue;
    }

    recordUsed[static_cast<std::size_t>(bestRecord)] = true;
    const auto& record
        = mWarmStartContactRecords[static_cast<std::size_t>(bestRecord)];
    const Eigen::Vector3d worldImpulse
        = bestFlipped ? Eigen::Vector3d(-record.worldImpulse)
                      : record.worldImpulse;
    const Eigen::Index offset = static_cast<Eigen::Index>(3u * i);
    Eigen::Vector3d localReaction(
        worldImpulse.dot(frame.normal),
        worldImpulse.dot(frame.tangent1),
        worldImpulse.dot(frame.tangent2));
    if (!localReaction.allFinite()) {
      continue;
    }

    problem.initialGuess.segment<3>(offset)
        = math::detail::projectCoulombConeNormalFirst(
            localReaction,
            problem.contactProblem.coefficients[static_cast<Eigen::Index>(i)]);
    ++matchedContacts;
  }

  if (matchedContacts == 0u) {
    return false;
  }

  mLastExactCoulombWarmStartMatchedContacts = matchedContacts;
  return true;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::updateExactCoulombWarmStart(
    const std::vector<ConstraintBase*>& constraints,
    const Eigen::Ref<const Eigen::VectorXd>& reaction)
{
  if (!mExactCoulombOptions.enableWarmStart || !reaction.allFinite()) {
    clearExactCoulombWarmStart();
    return;
  }

  mWarmStartConstraints = constraints;
  mWarmStartReaction = reaction;
  mHasExactCoulombWarmStart = true;

  // Store the world-space per-contact reactions for cross-step manifold
  // matching. One `World` step can solve several constrained groups
  // (contact islands) through this solver, so the record store is keyed by
  // body pair: a group's update replaces only records for its own pairs and
  // leaves other islands' records intact. Skip the manifold update when any
  // frame is unavailable so the pointer-identity fast path keeps its
  // previous behavior.
  std::vector<ExactCoulombWarmStartContactRecord> groupRecords;
  groupRecords.reserve(constraints.size());
  for (std::size_t i = 0u; i < constraints.size(); ++i) {
    detail::ExactCoulombContactRowOperator::ContactFrame frame;
    if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraints[i], frame)) {
      mWarmStartContactRecords.clear();
      return;
    }

    const Eigen::Index offset = static_cast<Eigen::Index>(3u * i);
    ExactCoulombWarmStartContactRecord record;
    record.bodyA = frame.bodyA;
    record.bodyB = frame.bodyB;
    record.point = frame.point;
    record.worldImpulse = reaction[offset] * frame.normal
                          + reaction[offset + 1] * frame.tangent1
                          + reaction[offset + 2] * frame.tangent2;
    groupRecords.push_back(record);
  }

  const auto groupOwnsPair
      = [&groupRecords](const ExactCoulombWarmStartContactRecord& record) {
          for (const auto& groupRecord : groupRecords) {
            const bool sameOrder = groupRecord.bodyA == record.bodyA
                                   && groupRecord.bodyB == record.bodyB;
            const bool flippedOrder = groupRecord.bodyA == record.bodyB
                                      && groupRecord.bodyB == record.bodyA;
            if (sameOrder || flippedOrder) {
              return true;
            }
          }
          return false;
        };
  mWarmStartContactRecords.erase(
      std::remove_if(
          mWarmStartContactRecords.begin(),
          mWarmStartContactRecords.end(),
          groupOwnsPair),
      mWarmStartContactRecords.end());
  mWarmStartContactRecords.insert(
      mWarmStartContactRecords.end(), groupRecords.begin(), groupRecords.end());

  // Bound the store so pathological scenes cannot grow it without limit.
  constexpr std::size_t kMaxWarmStartContactRecords = 4096u;
  if (mWarmStartContactRecords.size() > kMaxWarmStartContactRecords) {
    mWarmStartContactRecords.erase(
        mWarmStartContactRecords.begin(),
        mWarmStartContactRecords.begin()
            + static_cast<std::ptrdiff_t>(
                mWarmStartContactRecords.size() - kMaxWarmStartContactRecords));
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::clearExactCoulombWarmStart()
{
  mHasExactCoulombWarmStart = false;
  mWarmStartConstraints.clear();
  mWarmStartReaction.resize(0);
  mWarmStartContactRecords.clear();
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    invalidateExactCoulombWarmStartPointerCache()
{
  // One solve failure invalidates only the same-step pointer-identity cache.
  // The pair-keyed manifold records from OTHER islands stay valid, and even
  // the failing group's own last-success records remain safe seeds: warm
  // starts are cone-projected initial guesses, so a stale seed can only cost
  // convergence speed, never correctness.
  mHasExactCoulombWarmStart = false;
  mWarmStartConstraints.clear();
  mWarmStartReaction.resize(0);
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::recordExactCoulombResidualHistory(
    std::size_t contactCount,
    ExactCoulombFbfConstraintSolverStatus status,
    const math::detail::ExactCoulombFbfResult& solution)
{
  if (mExactCoulombOptions.maxResidualHistoryRecords <= 0
      || solution.residualHistory.empty()) {
    return;
  }

  const auto maxRecords = static_cast<std::size_t>(
      mExactCoulombOptions.maxResidualHistoryRecords);
  if (mExactCoulombResidualHistoryRecords.size() >= maxRecords) {
    mExactCoulombResidualHistoryRecords.erase(
        mExactCoulombResidualHistoryRecords.begin());
  }

  ExactCoulombFbfResidualHistoryRecord record;
  record.solveIndex = mResidualHistoryRecordSequence++;
  record.contactCount = contactCount;
  record.status = status;
  record.fbfStatus = solution.status;
  record.iterations = solution.iterations;
  record.samples = solution.residualHistory;
  mExactCoulombResidualHistoryRecords.push_back(record);
}

} // namespace constraint
} // namespace dart
