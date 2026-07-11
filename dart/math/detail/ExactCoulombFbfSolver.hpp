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

#ifndef DART_MATH_DETAIL_EXACTCOULOMBFBFSOLVER_HPP_
#define DART_MATH_DETAIL_EXACTCOULOMBFBFSOLVER_HPP_

#include <dart/math/detail/ExactCoulombContactProblem.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>

namespace dart {
namespace math {
namespace detail {

/// Exit status for the internal exact-Coulomb FBF reference loop.
enum class ExactCoulombFbfStatus
{
  Success,
  MaxIterations,
  InvalidInput,
  InnerSolverFailed,
  StepSizeUnderflow,
};

/// Exit status for the internal frozen cone-QP approximation.
enum class ExactCoulombFrozenConeStatus
{
  Success,
  MaxIterations,
  InvalidInput,
};

/// Tunable policy for the paper's forward-backward-forward outer loop.
struct ExactCoulombFbfOptions
{
  /// Maximum accepted FBF outer iterations.
  int maxOuterIterations = 50;

  /// Stop when the dimensionless exact-Coulomb residual is at or below this.
  double tolerance = 1e-6;

  /// Optional initial gamma. NaN means use the safe spectral estimate.
  double initialStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Multiplier applied to the safe spectral estimate when choosing gamma.
  double stepSizeScale = 1.0;

  /// Relaxation applied to the accepted FBF correction.
  double outerRelaxation = 1.0;

  /// Accept a trial when gamma * ||B(y) - B(x)|| / ||y - x|| <= this value.
  double couplingVariationTolerance = 0.9;

  /// Shrink factor applied to gamma when the local coupling test fails.
  double shrinkFactor = 0.7;

  /// Maximum shrink/retry attempts per outer iteration.
  int maxStepShrinkIterations = 20;

  /// Power iterations used when deriving the safe base step.
  int spectralIterations = 10;

  /// Scales for the dimensionless residual.
  CoulombConeResidualScales residualScales;

  /// Compute the paper's fixed per-solve residual scales from the initial
  /// reaction, free velocity, augmented initial velocity, and Delassus spectral
  /// estimate. Disable this only for unit tests or diagnostics that need
  /// explicit `residualScales`.
  bool useAutomaticResidualScales = true;

  /// Maximum residual-history samples to retain. Zero disables recording.
  int maxResidualHistorySamples = 0;
};

/// Options for the first matrix-free frozen cone-QP approximation.
struct ExactCoulombFrozenConeOptions
{
  /// Maximum projected-gradient iterations.
  int maxIterations = 100;

  /// Relative fixed-point tolerance on successive projected iterates.
  double tolerance = 1e-10;

  /// Optional projected-gradient step. NaN means use a spectral bound.
  double stepSize = std::numeric_limits<double>::quiet_NaN();

  /// Power iterations used to estimate `lambda_max(W)`.
  int spectralIterations = 10;
};

/// Options for a matrix-free block Gauss-Seidel frozen cone-QP solve.
struct ExactCoulombFrozenConeBlockGaussSeidelOptions
{
  /// Maximum contact-block sweeps.
  int maxSweeps = 10;

  /// Maximum local projected-gradient steps per 3D contact block update.
  int localIterations = 8;

  /// Relative sweep-change tolerance.
  double tolerance = 1e-10;

  /// Relative tolerance for the local 3D projected-gradient subproblem.
  double localTolerance = 1e-12;

  /// Optional diagonal padding used only to choose conservative local steps.
  double diagonalRegularization = 0.0;

  /// Optional cached per-contact 3x3 diagonal Delassus blocks.
  ///
  /// The Delassus operator is constant across one exact-Coulomb solve, so a
  /// caller that invokes this subproblem once per FBF outer iteration can
  /// extract the blocks once per solve instead of paying `3 * contactCount`
  /// full Delassus products per inner call. When null, the blocks are
  /// extracted with matrix-free products inside the call. The pointee must
  /// outlive the call and match the problem's contact count.
  const std::vector<Eigen::Matrix3d>* cachedDiagonalBlocks = nullptr;
};

/// Residual sample from one accepted FBF outer iteration.
struct ExactCoulombFbfResidualSample
{
  int iteration = 0;
  int shrinkIterations = 0;
  double stepSize = std::numeric_limits<double>::quiet_NaN();
  double safeStepSize = std::numeric_limits<double>::quiet_NaN();
  double couplingVariationRatio = std::numeric_limits<double>::quiet_NaN();
  CoulombConeResidual residual = makeInvalidCoulombConeResidual();
};

/// Result bundle from the internal exact-Coulomb FBF reference loop.
struct ExactCoulombFbfResult
{
  ExactCoulombFbfStatus status = ExactCoulombFbfStatus::InvalidInput;
  int iterations = 0;
  int shrinkIterations = 0;
  double stepSize = std::numeric_limits<double>::quiet_NaN();
  double safeStepSize = std::numeric_limits<double>::quiet_NaN();
  double couplingVariationRatio = 0.0;
  CoulombConeResidual residual = makeInvalidCoulombConeResidual();
  int bestIteration = 0;
  CoulombConeResidual bestResidual = makeInvalidCoulombConeResidual();
  CoulombConeResidualScales residualScales;
  std::vector<ExactCoulombFbfResidualSample> residualHistory;
  Eigen::VectorXd reaction;
  Eigen::VectorXd bestReaction;
};

/// Result bundle from the internal frozen cone-QP approximation.
struct ExactCoulombFrozenConeResult
{
  ExactCoulombFrozenConeStatus status
      = ExactCoulombFrozenConeStatus::InvalidInput;
  int iterations = 0;
  double stepSize = std::numeric_limits<double>::quiet_NaN();
  double fixedPointError = std::numeric_limits<double>::infinity();
  Eigen::VectorXd reaction;
};

inline bool isValidExactCoulombFbfOptions(const ExactCoulombFbfOptions& options)
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
         && options.spectralIterations > 0
         && options.maxResidualHistorySamples >= 0
         && isValidCoulombConeResidualScale(
             options.residualScales.reactionScale)
         && isValidCoulombConeResidualScale(
             options.residualScales.velocityScale);
}

inline void appendExactCoulombFbfResidualHistorySample(
    ExactCoulombFbfResult& result,
    const ExactCoulombFbfOptions& options,
    int iteration)
{
  if (options.maxResidualHistorySamples <= 0)
    return;

  if (result.residualHistory.size()
      >= static_cast<std::size_t>(options.maxResidualHistorySamples)) {
    return;
  }

  ExactCoulombFbfResidualSample sample;
  sample.iteration = iteration;
  sample.shrinkIterations = result.shrinkIterations;
  sample.stepSize = result.stepSize;
  sample.safeStepSize = result.safeStepSize;
  sample.couplingVariationRatio = result.couplingVariationRatio;
  sample.residual = result.residual;
  result.residualHistory.push_back(sample);
}

inline bool projectExactCoulombReactionNormalFirst(
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const Eigen::Ref<const Eigen::VectorXd>& coefficients,
    Eigen::Ref<Eigen::VectorXd> projectedReaction);

/// Compute the fixed dimensionless residual scales from the paper.
///
/// With `lambda_0` already projected into the primal cone, the paper holds
/// these scales constant over one FBF solve:
/// `s_r = max(||lambda_0||, ||v_f|| / lambda_max(W), eps_0)` and
/// `s_u = max(||v_tilde(lambda_0)||, ||v_f||, eps_0)`.
template <typename DelassusOperator>
CoulombConeResidualScales computeExactCoulombFbfResidualScales(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& initialReaction,
    const DelassusOperator& applyDelassus,
    int spectralIterations = 10)
{
  constexpr double kResidualScaleFloor = 1e-12;
  CoulombConeResidualScales scales{kResidualScaleFloor, kResidualScaleFloor};

  if (!isValidExactCoulombContactProblem(problem)
      || initialReaction.size() != problem.getDimension()
      || !initialReaction.allFinite() || spectralIterations <= 0) {
    DART_ASSERT(false && "Invalid exact-Coulomb residual-scale input.");
    return scales;
  }

  Eigen::VectorXd projectedReaction(problem.getDimension());
  if (!projectExactCoulombReactionNormalFirst(
          initialReaction, problem.coefficients, projectedReaction)) {
    return scales;
  }

  Eigen::VectorXd initialVelocity(problem.getDimension());
  if (!computeExactCoulombContactVelocityNormalFirst(
          problem, projectedReaction, applyDelassus, initialVelocity)) {
    return scales;
  }

  Eigen::VectorXd augmentedInitialVelocity(problem.getDimension());
  if (!computeExactCoulombAugmentedVelocityNormalFirst(
          initialVelocity, problem.coefficients, augmentedInitialVelocity)) {
    return scales;
  }

  scales.reactionScale
      = (std::max)(projectedReaction.norm(), kResidualScaleFloor);
  const double spectralRadius = estimateLargestExactCoulombDelassusEigenvalue(
      problem, applyDelassus, spectralIterations);
  if (std::isfinite(spectralRadius) && spectralRadius > 0.0) {
    scales.reactionScale = (std::max)(
        scales.reactionScale, problem.freeVelocity.norm() / spectralRadius);
  }

  scales.velocityScale = (std::max)(
      {augmentedInitialVelocity.norm(),
       problem.freeVelocity.norm(),
       kResidualScaleFloor});
  return scales;
}

inline bool isValidExactCoulombFrozenConeOptions(
    const ExactCoulombFrozenConeOptions& options)
{
  const bool validStepSize
      = std::isnan(options.stepSize)
        || (std::isfinite(options.stepSize) && options.stepSize > 0.0);

  return options.maxIterations >= 0 && std::isfinite(options.tolerance)
         && options.tolerance >= 0.0 && validStepSize
         && options.spectralIterations > 0;
}

inline bool isValidExactCoulombFrozenConeBlockGaussSeidelOptions(
    const ExactCoulombFrozenConeBlockGaussSeidelOptions& options)
{
  return options.maxSweeps >= 0 && options.localIterations > 0
         && std::isfinite(options.tolerance) && options.tolerance >= 0.0
         && std::isfinite(options.localTolerance)
         && options.localTolerance >= 0.0
         && std::isfinite(options.diagonalRegularization)
         && options.diagonalRegularization >= 0.0;
}

/// Project a product of normal-first contact triples onto their Coulomb cones.
inline bool projectExactCoulombReactionNormalFirst(
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const Eigen::Ref<const Eigen::VectorXd>& coefficients,
    Eigen::Ref<Eigen::VectorXd> projectedReaction)
{
  if (reaction.size() != projectedReaction.size()
      || reaction.size() != 3 * coefficients.size() || !reaction.allFinite()
      || !coefficients.allFinite() || (coefficients.array() < 0.0).any()) {
    DART_ASSERT(false && "Invalid exact-Coulomb reaction projection input.");
    return false;
  }

  for (Eigen::Index i = 0; i < coefficients.size(); ++i) {
    const Eigen::Index offset = 3 * i;
    projectedReaction.segment<3>(offset) = projectCoulombConeNormalFirst(
        reaction.segment<3>(offset), coefficients[i]);
  }
  return true;
}

/// Compute the explicit FBF coupling `B(lambda) = mu * ||v_t|| * e_n`.
inline bool computeExactCoulombFbfCouplingNormalFirst(
    const Eigen::Ref<const Eigen::VectorXd>& velocity,
    const Eigen::Ref<const Eigen::VectorXd>& coefficients,
    Eigen::Ref<Eigen::VectorXd> coupling)
{
  if (velocity.size() != coupling.size()
      || velocity.size() != 3 * coefficients.size() || !velocity.allFinite()
      || !coefficients.allFinite() || (coefficients.array() < 0.0).any()) {
    DART_ASSERT(false && "Invalid exact-Coulomb FBF coupling input.");
    return false;
  }

  coupling.setZero();
  for (Eigen::Index i = 0; i < coefficients.size(); ++i) {
    const Eigen::Index offset = 3 * i;
    coupling[offset] = coefficients[i] * velocity.segment<2>(offset + 1).norm();
  }
  return true;
}

/// Compute gamma_safe = 0.5 / (mu_max * lambda_max(W)).
template <typename DelassusOperator>
double computeExactCoulombFbfSafeStepSize(
    const ExactCoulombContactProblem& problem,
    const DelassusOperator& applyDelassus,
    int spectralIterations = 10)
{
  if (!isValidExactCoulombContactProblem(problem) || spectralIterations <= 0) {
    DART_ASSERT(false && "Invalid exact-Coulomb FBF safe-step input.");
    return std::numeric_limits<double>::infinity();
  }

  if (problem.coefficients.size() == 0) {
    return std::numeric_limits<double>::infinity();
  }

  const double maxCoefficient = problem.coefficients.maxCoeff();
  if (maxCoefficient == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double spectralRadius = estimateLargestExactCoulombDelassusEigenvalue(
      problem, applyDelassus, spectralIterations);
  if (!std::isfinite(spectralRadius)) {
    return std::numeric_limits<double>::infinity();
  }
  if (spectralRadius == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return 0.5 / (maxCoefficient * spectralRadius);
}

inline double computeExactCoulombFbfCouplingVariationRatio(
    const Eigen::Ref<const Eigen::VectorXd>& previousReaction,
    const Eigen::Ref<const Eigen::VectorXd>& trialReaction,
    const Eigen::Ref<const Eigen::VectorXd>& previousCoupling,
    const Eigen::Ref<const Eigen::VectorXd>& trialCoupling,
    double stepSize)
{
  if (previousReaction.size() != trialReaction.size()
      || previousCoupling.size() != trialCoupling.size()
      || previousReaction.size() != previousCoupling.size()
      || !previousReaction.allFinite() || !trialReaction.allFinite()
      || !previousCoupling.allFinite() || !trialCoupling.allFinite()
      || !std::isfinite(stepSize) || stepSize <= 0.0) {
    DART_ASSERT(false && "Invalid exact-Coulomb FBF ratio input.");
    return std::numeric_limits<double>::infinity();
  }

  const double reactionChange = (trialReaction - previousReaction).norm();
  const double couplingChange = (trialCoupling - previousCoupling).norm();
  if (reactionChange == 0.0) {
    return couplingChange == 0.0 ? 0.0
                                 : std::numeric_limits<double>::infinity();
  }
  return stepSize * couplingChange / reactionChange;
}

/// Extract 3x3 diagonal Delassus blocks using only matrix-free products.
template <typename DelassusOperator>
bool computeExactCoulombDelassusDiagonalBlocksNormalFirst(
    const ExactCoulombContactProblem& problem,
    const DelassusOperator& applyDelassus,
    std::vector<Eigen::Matrix3d>& diagonalBlocks)
{
  if (!isValidExactCoulombContactProblem(problem)) {
    DART_ASSERT(false && "Invalid exact-Coulomb diagonal-block input.");
    return false;
  }

  const Eigen::Index contactCount = problem.getContactCount();
  const Eigen::Index dimension = problem.getDimension();
  diagonalBlocks.assign(
      static_cast<std::size_t>(contactCount), Eigen::Matrix3d::Zero());

  Eigen::VectorXd basis = Eigen::VectorXd::Zero(dimension);
  Eigen::VectorXd product(dimension);
  for (Eigen::Index contact = 0; contact < contactCount; ++contact) {
    const Eigen::Index offset = 3 * contact;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      basis.setZero();
      basis[offset + axis] = 1.0;
      applyDelassus(basis, product);
      if (!product.allFinite()) {
        DART_ASSERT(
            false
            && "Exact-Coulomb Delassus operator returned non-finite values.");
        return false;
      }

      diagonalBlocks[static_cast<std::size_t>(contact)].col(axis)
          = product.segment<3>(offset);
    }
  }

  return true;
}

inline double computeExactCoulombLocalBlockLipschitz(
    const Eigen::Matrix3d& hessian, double diagonalRegularization)
{
  if (!hessian.allFinite() || !std::isfinite(diagonalRegularization)
      || diagonalRegularization < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::Matrix3d symmetricHessian = 0.5 * (hessian + hessian.transpose());
  symmetricHessian.diagonal().array() += diagonalRegularization;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetricHessian);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }

  return (std::max)(solver.eigenvalues().maxCoeff(), 0.0);
}

/// Solve the frozen cone QP with a matrix-free projected-gradient iteration.
///
/// This approximates the resolvent subproblem used by the paper's FBF loop:
///
/// `0 in W y + v_free + frozenCoupling + gamma^-1 (y - reference) + N_K(y)`.
///
/// The implementation is intentionally simple and inspectable. It uses the
/// product-cone projection already shared by the FBF correction and a stable
/// spectral projected-gradient step. The block Gauss-Seidel helper below uses
/// the same frozen cone-QP callback shape.
template <typename DelassusOperator>
ExactCoulombFrozenConeResult solveExactCoulombFrozenConeProjectedGradient(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
    const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
    double stepSizeGamma,
    const DelassusOperator& applyDelassus,
    const ExactCoulombFrozenConeOptions& options
    = ExactCoulombFrozenConeOptions())
{
  ExactCoulombFrozenConeResult result;
  result.reaction.resize(problem.getDimension());

  if (!isValidExactCoulombContactProblem(problem)
      || !isValidExactCoulombFrozenConeOptions(options)
      || referenceReaction.size() != problem.getDimension()
      || frozenCoupling.size() != problem.getDimension()
      || !referenceReaction.allFinite() || !frozenCoupling.allFinite()
      || !std::isfinite(stepSizeGamma) || stepSizeGamma <= 0.0) {
    DART_ASSERT(false && "Invalid exact-Coulomb frozen cone-QP input.");
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  if (!projectExactCoulombReactionNormalFirst(
          referenceReaction, problem.coefficients, result.reaction)) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  double projectedGradientStep = options.stepSize;
  if (std::isnan(projectedGradientStep)) {
    const double spectralRadius = estimateLargestExactCoulombDelassusEigenvalue(
        problem, applyDelassus, options.spectralIterations);
    if (!std::isfinite(spectralRadius)) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }

    const double lipschitz = spectralRadius + 1.0 / stepSizeGamma;
    if (!std::isfinite(lipschitz) || lipschitz <= 0.0) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }
    projectedGradientStep = 1.0 / lipschitz;
  }

  if (!std::isfinite(projectedGradientStep) || projectedGradientStep <= 0.0) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }
  result.stepSize = projectedGradientStep;

  const Eigen::Index dimension = problem.getDimension();
  Eigen::VectorXd delassusReaction(dimension);
  Eigen::VectorXd gradient(dimension);
  Eigen::VectorXd candidate(dimension);
  Eigen::VectorXd nextReaction(dimension);

  for (int iteration = 0; iteration < options.maxIterations; ++iteration) {
    applyDelassus(result.reaction, delassusReaction);
    if (!delassusReaction.allFinite()) {
      DART_ASSERT(
          false
          && "Exact-Coulomb Delassus operator returned non-finite values.");
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }

    gradient = delassusReaction + problem.freeVelocity + frozenCoupling
               + (result.reaction - referenceReaction) / stepSizeGamma;
    if (!gradient.allFinite()) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }

    candidate = result.reaction - projectedGradientStep * gradient;
    if (!projectExactCoulombReactionNormalFirst(
            candidate, problem.coefficients, nextReaction)) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }

    result.fixedPointError
        = (nextReaction - result.reaction).norm() / (1.0 + nextReaction.norm());
    result.reaction = nextReaction;
    result.iterations = iteration + 1;
    if (result.fixedPointError <= options.tolerance) {
      result.status = ExactCoulombFrozenConeStatus::Success;
      return result;
    }
  }

  result.status = ExactCoulombFrozenConeStatus::MaxIterations;
  return result;
}

/// Solve the frozen cone QP with contact-block Gauss-Seidel sweeps.
///
/// Each sweep visits one normal-first contact triple at a time, keeps the
/// off-diagonal Delassus response fixed through the current global iterate, and
/// approximately solves the resulting 3D cone subproblem with projected
/// gradient. This mirrors the paper's matrix-free inner-solve structure while
/// staying small enough to validate against the projected-gradient reference.
///
/// `accumulateDelassusBlockColumns(contact, delta, accumulator)` must perform
/// `accumulator += W.middleCols(3 * contact, 3) * delta`. The tracked Delassus
/// response is updated incrementally through those block-column products and
/// refreshed with one full product at a fixed sweep interval, so a sweep costs
/// one block-column product per updated contact instead of one full Delassus
/// product per updated contact.
template <typename DelassusOperator, typename DelassusBlockColumnOperator>
ExactCoulombFrozenConeResult solveExactCoulombFrozenConeBlockGaussSeidel(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
    const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
    double stepSizeGamma,
    const DelassusOperator& applyDelassus,
    const DelassusBlockColumnOperator& accumulateDelassusBlockColumns,
    const ExactCoulombFrozenConeBlockGaussSeidelOptions& options)
{
  ExactCoulombFrozenConeResult result;
  result.reaction.resize(problem.getDimension());

  if (!isValidExactCoulombContactProblem(problem)
      || !isValidExactCoulombFrozenConeBlockGaussSeidelOptions(options)
      || referenceReaction.size() != problem.getDimension()
      || frozenCoupling.size() != problem.getDimension()
      || !referenceReaction.allFinite() || !frozenCoupling.allFinite()
      || !std::isfinite(stepSizeGamma) || stepSizeGamma <= 0.0) {
    DART_ASSERT(false && "Invalid exact-Coulomb block-GS input.");
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  if (!projectExactCoulombReactionNormalFirst(
          referenceReaction, problem.coefficients, result.reaction)) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  const Eigen::Index dimension = problem.getDimension();
  if (dimension == 0) {
    result.status = ExactCoulombFrozenConeStatus::Success;
    result.fixedPointError = 0.0;
    return result;
  }

  const Eigen::Index contactCount = problem.getContactCount();
  std::vector<Eigen::Matrix3d> localDiagonalBlocks;
  const std::vector<Eigen::Matrix3d>* diagonalBlocks
      = options.cachedDiagonalBlocks;
  if (diagonalBlocks == nullptr) {
    if (!computeExactCoulombDelassusDiagonalBlocksNormalFirst(
            problem, applyDelassus, localDiagonalBlocks)) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }
    diagonalBlocks = &localDiagonalBlocks;
  } else if (diagonalBlocks->size() != static_cast<std::size_t>(contactCount)) {
    DART_ASSERT(false && "Cached exact-Coulomb diagonal-block count mismatch.");
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  const double inverseGamma = 1.0 / stepSizeGamma;
  const Eigen::Matrix3d gammaHessian
      = inverseGamma * Eigen::Matrix3d::Identity();

  // The Delassus blocks and gamma are fixed for this call, so the local
  // Hessians and conservative projected-gradient steps are cached once per
  // call instead of once per contact visit.
  std::vector<Eigen::Matrix3d> localHessians(
      static_cast<std::size_t>(contactCount));
  std::vector<double> localSteps(static_cast<std::size_t>(contactCount));
  double minimumLocalStep = std::numeric_limits<double>::infinity();
  for (Eigen::Index contact = 0; contact < contactCount; ++contact) {
    const std::size_t blockIndex = static_cast<std::size_t>(contact);
    localHessians[blockIndex] = (*diagonalBlocks)[blockIndex] + gammaHessian;
    const double localLipschitz = computeExactCoulombLocalBlockLipschitz(
        localHessians[blockIndex], options.diagonalRegularization);
    if (!std::isfinite(localLipschitz) || localLipschitz <= 0.0) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }

    localSteps[blockIndex] = 1.0 / localLipschitz;
    minimumLocalStep = (std::min)(minimumLocalStep, localSteps[blockIndex]);
  }

  Eigen::VectorXd delassusReaction(dimension);
  applyDelassus(result.reaction, delassusReaction);
  if (!delassusReaction.allFinite()) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  // Bound incremental floating-point drift in the tracked Delassus response
  // without paying one full product per contact update.
  constexpr int kDelassusRefreshSweepInterval = 16;

  for (int sweep = 0; sweep < options.maxSweeps; ++sweep) {
    double sweepChangeSquared = 0.0;
    bool sweepUpdated = false;

    for (Eigen::Index contact = 0; contact < contactCount; ++contact) {
      const Eigen::Index offset = 3 * contact;
      const std::size_t blockIndex = static_cast<std::size_t>(contact);
      const Eigen::Matrix3d& localHessian = localHessians[blockIndex];
      const double localStep = localSteps[blockIndex];

      const Eigen::Vector3d previousBlock = result.reaction.segment<3>(offset);
      const Eigen::Vector3d previousGradient
          = delassusReaction.segment<3>(offset)
            + problem.freeVelocity.segment<3>(offset)
            + frozenCoupling.segment<3>(offset)
            + inverseGamma
                  * (previousBlock - referenceReaction.segment<3>(offset));
      if (!previousGradient.allFinite()) {
        result.status = ExactCoulombFrozenConeStatus::InvalidInput;
        return result;
      }

      Eigen::Vector3d block = previousBlock;
      for (int localIteration = 0; localIteration < options.localIterations;
           ++localIteration) {
        const Eigen::Vector3d localGradient
            = previousGradient + localHessian * (block - previousBlock);
        const Eigen::Vector3d candidate = block - localStep * localGradient;
        const Eigen::Vector3d nextBlock = projectCoulombConeNormalFirst(
            candidate, problem.coefficients[contact]);
        const double localChange
            = (nextBlock - block).norm() / (1.0 + nextBlock.norm());
        block = nextBlock;
        if (localChange <= options.localTolerance) {
          break;
        }
      }

      const Eigen::Vector3d delta = block - previousBlock;
      if (delta.norm() == 0.0) {
        continue;
      }

      result.reaction.segment<3>(offset) = block;
      accumulateDelassusBlockColumns(contact, delta, delassusReaction);
      sweepUpdated = true;
      sweepChangeSquared += delta.squaredNorm();
    }

    result.fixedPointError
        = std::sqrt(sweepChangeSquared) / (1.0 + result.reaction.norm());
    result.iterations = sweep + 1;
    result.stepSize = minimumLocalStep;
    if (result.fixedPointError <= options.tolerance) {
      result.status = ExactCoulombFrozenConeStatus::Success;
      return result;
    }

    if (sweepUpdated && (sweep + 1) % kDelassusRefreshSweepInterval == 0) {
      applyDelassus(result.reaction, delassusReaction);
      if (!delassusReaction.allFinite()) {
        result.status = ExactCoulombFrozenConeStatus::InvalidInput;
        return result;
      }
    }
  }

  result.stepSize = minimumLocalStep;
  result.status = ExactCoulombFrozenConeStatus::MaxIterations;
  return result;
}

/// Solve the frozen cone QP with contact-block Gauss-Seidel sweeps using only
/// a full Delassus product operator.
///
/// This forwards to the block-column overload with a generic adapter that
/// applies each block-column update through one full Delassus product of a
/// 3-sparse delta vector. Callers that can apply Delassus block columns
/// directly (for example from a dense snapshot) should use the two-operator
/// overload instead.
template <typename DelassusOperator>
ExactCoulombFrozenConeResult solveExactCoulombFrozenConeBlockGaussSeidel(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
    const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
    double stepSizeGamma,
    const DelassusOperator& applyDelassus,
    const ExactCoulombFrozenConeBlockGaussSeidelOptions& options
    = ExactCoulombFrozenConeBlockGaussSeidelOptions())
{
  const Eigen::Index dimension = problem.getDimension();
  Eigen::VectorXd basisDelta = Eigen::VectorXd::Zero(dimension);
  Eigen::VectorXd columnProduct(dimension);
  const auto accumulateThroughFullProduct
      = [&applyDelassus, &basisDelta, &columnProduct](
            Eigen::Index contact,
            const Eigen::Vector3d& delta,
            Eigen::Ref<Eigen::VectorXd> accumulator) {
          basisDelta.segment<3>(3 * contact) = delta;
          applyDelassus(basisDelta, columnProduct);
          accumulator += columnProduct;
          basisDelta.segment<3>(3 * contact).setZero();
        };

  return solveExactCoulombFrozenConeBlockGaussSeidel(
      problem,
      referenceReaction,
      frozenCoupling,
      stepSizeGamma,
      applyDelassus,
      accumulateThroughFullProduct,
      options);
}

/// Solve with the exact reduced Coulomb FBF structure from Song, Fan, Ascher,
/// and Pai, "A Splitting Architecture for Exact Reduced Coulomb Friction"
/// (SCA 2026), using an injected inner cone solver.
///
/// `solveFrozenConeProblem(problem, reaction, coupling, gamma, output)` must
/// solve the strongly convex cone subproblem with frozen coupling and write the
/// intermediate reaction into `output`.
template <typename DelassusOperator, typename FrozenConeSolver>
ExactCoulombFbfResult solveExactCoulombFbf(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& initialReaction,
    const DelassusOperator& applyDelassus,
    const FrozenConeSolver& solveFrozenConeProblem,
    const ExactCoulombFbfOptions& options = ExactCoulombFbfOptions())
{
  ExactCoulombFbfResult result;
  result.reaction.resize(problem.getDimension());

  if (!isValidExactCoulombContactProblem(problem)
      || !isValidExactCoulombFbfOptions(options)
      || initialReaction.size() != problem.getDimension()
      || !initialReaction.allFinite()) {
    DART_ASSERT(false && "Invalid exact-Coulomb FBF solve input.");
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }

  if (!projectExactCoulombReactionNormalFirst(
          initialReaction, problem.coefficients, result.reaction)) {
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }

  result.safeStepSize = computeExactCoulombFbfSafeStepSize(
      problem, applyDelassus, options.spectralIterations);

  double baseStepSize = options.initialStepSize;
  double maxStepSize = std::numeric_limits<double>::infinity();
  if (std::isfinite(result.safeStepSize)) {
    maxStepSize = result.safeStepSize * options.stepSizeScale;
  }
  if (std::isnan(baseStepSize)) {
    baseStepSize = std::isfinite(maxStepSize) ? maxStepSize : 1.0;
  }
  if (std::isfinite(maxStepSize)) {
    baseStepSize = (std::min)(baseStepSize, maxStepSize);
  }
  if (!std::isfinite(baseStepSize) || baseStepSize <= 0.0) {
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }
  result.stepSize = baseStepSize;

  CoulombConeResidualScales residualScales = options.residualScales;
  if (options.useAutomaticResidualScales) {
    residualScales = computeExactCoulombFbfResidualScales(
        problem, result.reaction, applyDelassus, options.spectralIterations);
  }
  if (!isValidCoulombConeResidualScale(residualScales.reactionScale)
      || !isValidCoulombConeResidualScale(residualScales.velocityScale)) {
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }
  result.residualScales = residualScales;

  result.residual = computeExactCoulombContactResidualNormalFirst(
      problem, result.reaction, applyDelassus, residualScales);
  result.bestIteration = 0;
  result.bestResidual = result.residual;
  result.bestReaction = result.reaction;
  appendExactCoulombFbfResidualHistorySample(result, options, 0);
  if (result.residual.value <= options.tolerance) {
    result.status = ExactCoulombFbfStatus::Success;
    return result;
  }

  const Eigen::Index dimension = problem.getDimension();
  Eigen::VectorXd velocity(dimension);
  Eigen::VectorXd coupling(dimension);
  Eigen::VectorXd trialReaction(dimension);
  Eigen::VectorXd trialVelocity(dimension);
  Eigen::VectorXd trialCoupling(dimension);
  Eigen::VectorXd correctedReaction(dimension);
  Eigen::VectorXd projectedCorrection(dimension);
  Eigen::VectorXd relaxedReaction(dimension);

  for (int iteration = 0; iteration < options.maxOuterIterations; ++iteration) {
    double stepSize = baseStepSize;

    if (!computeExactCoulombContactVelocityNormalFirst(
            problem, result.reaction, applyDelassus, velocity)
        || !computeExactCoulombFbfCouplingNormalFirst(
            velocity, problem.coefficients, coupling)) {
      result.status = ExactCoulombFbfStatus::InvalidInput;
      return result;
    }

    bool accepted = false;
    for (int shrink = 0; shrink <= options.maxStepShrinkIterations; ++shrink) {
      if (!solveFrozenConeProblem(
              problem, result.reaction, coupling, stepSize, trialReaction)
          || !trialReaction.allFinite()) {
        result.status = ExactCoulombFbfStatus::InnerSolverFailed;
        return result;
      }

      if (!computeExactCoulombContactVelocityNormalFirst(
              problem, trialReaction, applyDelassus, trialVelocity)
          || !computeExactCoulombFbfCouplingNormalFirst(
              trialVelocity, problem.coefficients, trialCoupling)) {
        result.status = ExactCoulombFbfStatus::InvalidInput;
        return result;
      }

      result.couplingVariationRatio
          = computeExactCoulombFbfCouplingVariationRatio(
              result.reaction,
              trialReaction,
              coupling,
              trialCoupling,
              stepSize);
      if (result.couplingVariationRatio <= options.couplingVariationTolerance) {
        accepted = true;
        break;
      }

      stepSize *= options.shrinkFactor;
      ++result.shrinkIterations;
      result.stepSize = stepSize;
      if (!std::isfinite(stepSize) || stepSize <= 0.0) {
        result.status = ExactCoulombFbfStatus::StepSizeUnderflow;
        return result;
      }
    }

    if (!accepted) {
      result.status = ExactCoulombFbfStatus::StepSizeUnderflow;
      return result;
    }

    correctedReaction = trialReaction - stepSize * (trialCoupling - coupling);
    if (!projectExactCoulombReactionNormalFirst(
            correctedReaction, problem.coefficients, projectedCorrection)) {
      result.status = ExactCoulombFbfStatus::InvalidInput;
      return result;
    }
    if (options.outerRelaxation == 1.0) {
      result.reaction = projectedCorrection;
    } else {
      relaxedReaction
          = result.reaction
            + options.outerRelaxation * (projectedCorrection - result.reaction);
      if (!projectExactCoulombReactionNormalFirst(
              relaxedReaction, problem.coefficients, result.reaction)) {
        result.status = ExactCoulombFbfStatus::InvalidInput;
        return result;
      }
    }

    result.stepSize = stepSize;
    result.iterations = iteration + 1;
    result.residual = computeExactCoulombContactResidualNormalFirst(
        problem, result.reaction, applyDelassus, residualScales);
    if (std::isfinite(result.residual.value)
        && (!std::isfinite(result.bestResidual.value)
            || result.residual.value < result.bestResidual.value)) {
      result.bestIteration = result.iterations;
      result.bestResidual = result.residual;
      result.bestReaction = result.reaction;
    }
    appendExactCoulombFbfResidualHistorySample(
        result, options, result.iterations);
    if (result.residual.value <= options.tolerance) {
      result.status = ExactCoulombFbfStatus::Success;
      return result;
    }
  }

  result.status = ExactCoulombFbfStatus::MaxIterations;
  return result;
}

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_EXACTCOULOMBFBFSOLVER_HPP_
