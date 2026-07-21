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

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <limits>
#include <type_traits>
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
  Plateau,
  NonFiniteValue,
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

  /// Accepted-iteration interval for residual and termination checks.
  ///
  /// The final configured outer iteration is always checked. One preserves the
  /// established DART behavior of checking every accepted iteration.
  int residualCheckInterval = 1;

  /// Number of sampled natural-map residuals separating a plateau comparison.
  ///
  /// Zero disables plateau termination. With interval five and patience five,
  /// the first possible plateau is at accepted iteration 30, matching the
  /// pinned author implementation.
  int plateauPatience = 0;

  /// Plateau when sampled relative improvement is below this value.
  double plateauRelativeTolerance = 0.01;

  /// Use a strict `< tolerance` convergence comparison.
  ///
  /// Disabled by default to preserve DART's established `<=` comparison.
  bool useStrictToleranceComparison = false;

  /// Use the unscaled natural-map residual for the iteration-zero gate.
  ///
  /// Later sampled convergence checks continue to use the dimensionless
  /// exact-Coulomb residual. This is disabled by default.
  bool useNaturalMapResidualForInitialConvergence = false;

  /// Report nonfinite runtime values through `NonFiniteValue`.
  ///
  /// Disabled by default to preserve the established legacy status ordering.
  /// Source-continuation policy enables the distinct fail-closed diagnostic.
  bool reportNonFiniteValuesSeparately = false;

  /// Optional initial gamma. NaN means use the safe spectral estimate.
  double initialStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Cap an explicitly configured gamma at the safe spectral estimate.
  ///
  /// Keep this enabled for normal adaptive solves. Disable it only for a
  /// diagnostic fixed-gamma experiment where `initialStepSize` must be used
  /// exactly even when it exceeds the conservative estimate.
  bool capInitialStepSizeAtSafeBound = true;

  /// Multiplier applied to the safe spectral estimate when choosing gamma
  /// automatically (`initialStepSize` is NaN). Explicit gamma values are
  /// capped at the unscaled safe estimate when the cap is enabled.
  double stepSizeScale = 1.0;

  /// Multiplier applied to the safe bound for an explicit initial gamma.
  ///
  /// The default preserves the unscaled explicit-gamma cap. An opt-in
  /// cross-step policy can use the same scaled safe family as its automatic
  /// gamma while still honoring a freshly computed bound.
  double explicitStepSizeSafeBoundScale = 1.0;

  /// Optional lower gamma bound. NaN leaves gamma unbounded below.
  double minimumStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Optional upper gamma bound. NaN leaves gamma unbounded above.
  double maximumStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Initial residual below which `initialResidualStepSizeCap` is applied.
  ///
  /// NaN disables the cap. This opt-in protects an already-good warm start
  /// from an aggressive cross-step gamma without changing the safe-step or
  /// line-search policy used by default.
  double initialResidualStepSizeCapThreshold
      = std::numeric_limits<double>::quiet_NaN();

  /// Gamma cap applied when the initial residual is below the threshold.
  ///
  /// NaN disables the cap. This and
  /// `initialResidualStepSizeCapThreshold` must be enabled together.
  double initialResidualStepSizeCap = std::numeric_limits<double>::quiet_NaN();

  /// Use the unscaled natural-map norm for the initial-residual gamma cap.
  ///
  /// Disabled by default. This opt-in matches the metric used by the pinned
  /// author implementation without changing DART's convergence residual.
  bool useNaturalMapResidualForInitialStepSizeCap = false;

  /// Relaxation applied to the accepted FBF correction.
  double outerRelaxation = 1.0;

  /// Project the forward-backward-forward correction back onto the cone.
  ///
  /// Enabled by default to preserve DART's existing exact-Coulomb behavior.
  /// Disable only when reproducing a source solver that accepts the raw FBF
  /// correction before evaluating the next outer iteration.
  bool projectAfterCorrection = true;

  /// Start every frozen-cone solve from the current outer FBF reaction.
  ///
  /// Disabled by default to preserve DART's existing reuse of the previous
  /// accepted cone-QP iterate. Enable only when reproducing a source solver
  /// whose inner solver resets from the current outer reaction on every
  /// outer iteration and every rejected step-size trial.
  bool restartInnerFromCurrentOuterReaction = false;

  /// Accept a trial when gamma * ||B(y) - B(x)|| / ||y - x|| <= this value.
  double couplingVariationTolerance = 0.9;

  /// Accept without the coupling-variation ratio when `||y - x||` is no
  /// larger than this threshold.
  ///
  /// Zero preserves DART's exact-zero special case. The pinned source uses
  /// `1e-10`.
  double couplingVariationSkipThreshold = 0.0;

  /// Shrink factor applied to gamma when the local coupling test fails.
  double shrinkFactor = 0.7;

  /// Maximum shrink/retry attempts per outer iteration.
  int maxStepShrinkIterations = 20;

  /// Accept the final inner result after applying one last unsolved shrink.
  ///
  /// Disabled by default, so a budget of N permits trials at gamma_0 through
  /// gamma_N and fails closed if all N+1 trials are rejected. When enabled, a
  /// budget of N permits trials through gamma_(N-1), then accepts that inner
  /// result while applying the correction with gamma_N. This reproduces the
  /// pinned source's shrink-cap behavior without changing the default path.
  bool acceptStepSizeAfterFinalShrink = false;

  /// Adapt gamma when the local coupling-variation test rejects a trial.
  ///
  /// Disabling this preserves the configured gamma for every outer iteration.
  /// The coupling-variation ratio is still computed and reported, but it does
  /// not reject the trial. This is intended for reproducing fixed-gamma sweep
  /// experiments, not as the default solve policy.
  bool enableAdaptiveStepSize = true;

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

struct ExactCoulombFrozenConeBlockGaussSeidelWorkspace;

/// Per-contact update used inside a frozen cone-QP block-Gauss-Seidel sweep.
enum class ExactCoulombFrozenConeLocalSolver
{
  /// Apply the inverse 3x3 block, then project in the Euclidean cone metric.
  ///
  /// This is an explicit inexpensive approximation. For an anisotropic block
  /// it is not the exact H-metric minimizer of the local quadratic.
  InverseEuclideanProjection,

  /// Solve the local SPD quadratic exactly in the Hessian metric. This is the
  /// correctness-preserving default.
  ExactMetricProjection,

  /// Retain the original scalar-step projected-gradient approximation.
  ProjectedGradient,
};

/// Options for a matrix-free block Gauss-Seidel frozen cone-QP solve.
struct ExactCoulombFrozenConeBlockGaussSeidelOptions
{
  /// Maximum contact-block sweeps.
  int maxSweeps = 10;

  /// Per-contact local update.
  ExactCoulombFrozenConeLocalSolver localSolver
      = ExactCoulombFrozenConeLocalSolver::ExactMetricProjection;

  /// Maximum local steps in `ProjectedGradient` mode.
  int localIterations = 8;

  /// Relative sweep-change tolerance.
  double tolerance = 1e-10;

  /// Run the full sweep budget even when the change tolerance is reached.
  bool runFixedSweeps = false;

  /// Relative local change tolerance in `ProjectedGradient` mode.
  double localTolerance = 1e-12;

  /// Diagonal padding used only to choose the `ProjectedGradient` step.
  ///
  /// It never changes the local quadratic objective in any mode.
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

  /// Optional initial iterate for the cone-QP solve.
  ///
  /// The proximal center remains the `referenceReaction` argument. This only
  /// selects the block Gauss-Seidel starting point. When null, the proximal
  /// center is also the initial iterate. The pointee must outlive the call and
  /// match the problem dimension.
  const Eigen::VectorXd* initialReaction = nullptr;

  /// Project the selected initial iterate onto the product Coulomb cone.
  ///
  /// Enabled by default to preserve DART's established frozen-cone behavior.
  /// Disable only when reproducing a source block Gauss-Seidel solver that
  /// copies the current outer reaction verbatim before its first sweep.
  bool projectInitialReaction = true;

  /// Optional reusable workspace for consecutive solves of the same local
  /// systems.
  ///
  /// The workspace owns its cache keys and never retains pointers to
  /// `cachedDiagonalBlocks`. It automatically rebuilds the local Hessians and
  /// mode-specific 3x3 data when the contact count, a diagonal block, a
  /// friction coefficient, local solver mode, `stepSizeGamma`, or
  /// projected-gradient regularization changes. A workspace must outlive the
  /// call and must not be used by concurrent solves.
  ExactCoulombFrozenConeBlockGaussSeidelWorkspace* workspace = nullptr;
};

/// Residual sample from one accepted FBF outer iteration.
struct ExactCoulombFbfResidualSample
{
  int iteration = 0;
  int shrinkIterations = 0;
  int lineSearchShrinkCapCount = 0;
  double stepSize = std::numeric_limits<double>::quiet_NaN();
  double innerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  double safeStepSize = std::numeric_limits<double>::quiet_NaN();
  double couplingVariationRatio = std::numeric_limits<double>::quiet_NaN();
  double naturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  CoulombConeResidual residual = makeInvalidCoulombConeResidual();
};

/// Result bundle from the internal exact-Coulomb FBF reference loop.
struct ExactCoulombFbfResult
{
  ExactCoulombFbfStatus status = ExactCoulombFbfStatus::InvalidInput;
  int iterations = 0;
  int shrinkIterations = 0;
  int lineSearchShrinkCapCount = 0;
  bool lineSearchShrinkCapReached = false;
  double stepSize = std::numeric_limits<double>::quiet_NaN();
  double lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  double uncappedInitialStepSize = std::numeric_limits<double>::quiet_NaN();
  bool initialStepSizeCapApplied = false;
  double safeStepSize = std::numeric_limits<double>::quiet_NaN();
  double couplingVariationRatio = 0.0;
  double initialNaturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  double naturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  double plateauReferenceNaturalMapResidual
      = std::numeric_limits<double>::quiet_NaN();
  double plateauRelativeImprovement = std::numeric_limits<double>::quiet_NaN();
  CoulombConeResidual initialResidual = makeInvalidCoulombConeResidual();
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
  const bool hasInitialResidualStepSizeCap
      = std::isfinite(options.initialResidualStepSizeCapThreshold)
        && std::isfinite(options.initialResidualStepSizeCap);
  const bool validInitialResidualStepSizeCap
      = (std::isnan(options.initialResidualStepSizeCapThreshold)
         && std::isnan(options.initialResidualStepSizeCap))
        || (hasInitialResidualStepSizeCap
            && options.initialResidualStepSizeCapThreshold >= 0.0
            && options.initialResidualStepSizeCap > 0.0);
  const bool validMinimumStepSize = std::isnan(options.minimumStepSize)
                                    || (std::isfinite(options.minimumStepSize)
                                        && options.minimumStepSize > 0.0);
  const bool validMaximumStepSize = std::isnan(options.maximumStepSize)
                                    || (std::isfinite(options.maximumStepSize)
                                        && options.maximumStepSize > 0.0);
  const bool validStepSizeRange
      = validMinimumStepSize && validMaximumStepSize
        && (std::isnan(options.minimumStepSize)
            || std::isnan(options.maximumStepSize)
            || options.minimumStepSize <= options.maximumStepSize);

  return options.maxOuterIterations >= 0 && std::isfinite(options.tolerance)
         && options.tolerance >= 0.0 && options.residualCheckInterval > 0
         && options.plateauPatience >= 0
         && std::isfinite(options.plateauRelativeTolerance)
         && options.plateauRelativeTolerance >= 0.0 && validInitialStep
         && std::isfinite(options.stepSizeScale) && options.stepSizeScale > 0.0
         && std::isfinite(options.explicitStepSizeSafeBoundScale)
         && options.explicitStepSizeSafeBoundScale > 0.0 && validStepSizeRange
         && validInitialResidualStepSizeCap
         && std::isfinite(options.outerRelaxation)
         && options.outerRelaxation > 0.0
         && std::isfinite(options.couplingVariationTolerance)
         && options.couplingVariationTolerance > 0.0
         && std::isfinite(options.couplingVariationSkipThreshold)
         && options.couplingVariationSkipThreshold >= 0.0
         && std::isfinite(options.shrinkFactor) && options.shrinkFactor > 0.0
         && options.shrinkFactor < 1.0 && options.maxStepShrinkIterations >= 0
         && (!options.acceptStepSizeAfterFinalShrink
             || options.maxStepShrinkIterations > 0)
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
  sample.lineSearchShrinkCapCount = result.lineSearchShrinkCapCount;
  sample.stepSize = result.stepSize;
  sample.innerSolveStepSize = result.lastInnerSolveStepSize;
  sample.safeStepSize = result.safeStepSize;
  sample.couplingVariationRatio = result.couplingVariationRatio;
  sample.naturalMapResidual = result.naturalMapResidual;
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
CoulombConeResidualScales
computeExactCoulombFbfResidualScalesWithSpectralRadius(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& initialReaction,
    const DelassusOperator& applyDelassus,
    double spectralRadius)
{
  constexpr double kResidualScaleFloor = 1e-12;
  CoulombConeResidualScales scales{kResidualScaleFloor, kResidualScaleFloor};

  if (!isValidExactCoulombContactProblem(problem)
      || initialReaction.size() != problem.getDimension()
      || !initialReaction.allFinite() || std::isnan(spectralRadius)
      || spectralRadius < 0.0) {
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

template <typename DelassusOperator>
CoulombConeResidualScales computeExactCoulombFbfResidualScales(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& initialReaction,
    const DelassusOperator& applyDelassus,
    int spectralIterations = 10)
{
  if (spectralIterations <= 0) {
    DART_ASSERT(false && "Invalid exact-Coulomb residual-scale input.");
    return CoulombConeResidualScales{1e-12, 1e-12};
  }

  const double spectralRadius = estimateLargestExactCoulombDelassusEigenvalue(
      problem, applyDelassus, spectralIterations);
  return computeExactCoulombFbfResidualScalesWithSpectralRadius(
      problem, initialReaction, applyDelassus, spectralRadius);
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
  const bool validLocalSolver
      = options.localSolver
            == ExactCoulombFrozenConeLocalSolver::InverseEuclideanProjection
        || options.localSolver
               == ExactCoulombFrozenConeLocalSolver::ExactMetricProjection
        || options.localSolver
               == ExactCoulombFrozenConeLocalSolver::ProjectedGradient;
  const bool validProjectedGradientOptions
      = options.localSolver
            != ExactCoulombFrozenConeLocalSolver::ProjectedGradient
        || (options.localIterations > 0 && std::isfinite(options.localTolerance)
            && options.localTolerance >= 0.0
            && std::isfinite(options.diagonalRegularization)
            && options.diagonalRegularization >= 0.0);
  return validLocalSolver && options.maxSweeps >= 0
         && validProjectedGradientOptions && std::isfinite(options.tolerance)
         && options.tolerance >= 0.0;
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

/// Compute gamma_safe from a retained Delassus spectral estimate.
inline double computeExactCoulombFbfSafeStepSizeFromSpectralRadius(
    const ExactCoulombContactProblem& problem, double spectralRadius)
{
  if (!isValidExactCoulombContactProblem(problem) || std::isnan(spectralRadius)
      || spectralRadius < 0.0) {
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

  if (!std::isfinite(spectralRadius)) {
    return std::numeric_limits<double>::infinity();
  }
  if (spectralRadius == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return 0.5 / (maxCoefficient * spectralRadius);
}

/// Compute gamma_safe = 0.5 / (mu_max * lambda_max(W)).
template <typename DelassusOperator>
double computeExactCoulombFbfSafeStepSize(
    const ExactCoulombContactProblem& problem,
    const DelassusOperator& applyDelassus,
    int spectralIterations = 10)
{
  if (spectralIterations <= 0) {
    DART_ASSERT(false && "Invalid exact-Coulomb FBF safe-step input.");
    return std::numeric_limits<double>::infinity();
  }

  const double spectralRadius = estimateLargestExactCoulombDelassusEigenvalue(
      problem, applyDelassus, spectralIterations);
  return computeExactCoulombFbfSafeStepSizeFromSpectralRadius(
      problem, spectralRadius);
}

inline double computeExactCoulombFbfCouplingVariationRatioFromNorms(
    double reactionChange, double couplingChange, double stepSize)
{
  if (reactionChange < 0.0 || couplingChange < 0.0 || !std::isfinite(stepSize)
      || stepSize <= 0.0) {
    DART_ASSERT(false && "Invalid exact-Coulomb FBF ratio input.");
    return std::numeric_limits<double>::infinity();
  }

  if (reactionChange == 0.0) {
    return couplingChange == 0.0 ? 0.0
                                 : std::numeric_limits<double>::infinity();
  }
  return stepSize * couplingChange / reactionChange;
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
  return computeExactCoulombFbfCouplingVariationRatioFromNorms(
      reactionChange, couplingChange, stepSize);
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

inline bool prepareExactCoulombLocalSpdHessian(
    const Eigen::Matrix3d& hessian, Eigen::Matrix3d& symmetricHessian)
{
  if (!hessian.allFinite()) {
    return false;
  }
  const double matrixScale = hessian.cwiseAbs().maxCoeff();
  if (!std::isfinite(matrixScale) || matrixScale <= 0.0) {
    return false;
  }
  const double symmetryTolerance
      = 256.0 * std::numeric_limits<double>::epsilon() * matrixScale;
  if ((hessian - hessian.transpose()).cwiseAbs().maxCoeff()
      > symmetryTolerance) {
    return false;
  }

  symmetricHessian = 0.5 * (hessian + hessian.transpose());
  Eigen::LLT<Eigen::Matrix3d> factorization(symmetricHessian);
  return factorization.info() == Eigen::Success;
}

inline bool prepareExactCoulombLocalSpdInverse(
    const Eigen::Matrix3d& hessian,
    Eigen::Matrix3d& symmetricHessian,
    Eigen::Matrix3d& inverseHessian)
{
  if (!prepareExactCoulombLocalSpdHessian(hessian, symmetricHessian)) {
    return false;
  }
  Eigen::LLT<Eigen::Matrix3d> factorization(symmetricHessian);
  inverseHessian = factorization.solve(Eigen::Matrix3d::Identity());
  return factorization.info() == Eigen::Success && inverseHessian.allFinite();
}

/// Apply a cached inverse block followed by Euclidean Coulomb-cone projection.
///
/// This is an explicit inexpensive per-contact approximation. For a general
/// anisotropic Hessian it is not the exact H-metric projection and therefore
/// need not satisfy the local quadratic KKT conditions after one update.
inline bool solveExactCoulombLocalConeInverseEuclideanProjection(
    const Eigen::Matrix3d& inverseHessian,
    const Eigen::Vector3d& linearTerm,
    double coefficient,
    Eigen::Vector3d& reaction)
{
  if (!inverseHessian.allFinite() || !linearTerm.allFinite()
      || !isValidCoulombConeCoefficient(coefficient)) {
    return false;
  }
  const Eigen::Vector3d unconstrained = -inverseHessian * linearTerm;
  if (!unconstrained.allFinite()) {
    return false;
  }
  reaction = projectCoulombConeNormalFirst(unconstrained, coefficient);
  return reaction.allFinite();
}

inline double computeExactCoulombLocalBlockLipschitz(
    const Eigen::Matrix3d& hessian, double diagonalRegularization)
{
  if (!hessian.allFinite() || !std::isfinite(diagonalRegularization)
      || diagonalRegularization < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  Eigen::Matrix3d regularizedHessian = hessian;
  regularizedHessian.diagonal().array() += diagonalRegularization;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(regularizedHessian);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  return (std::max)(solver.eigenvalues().maxCoeff(), 0.0);
}

/// Cached factorization for an exact SPD quadratic solve on one Coulomb cone.
///
/// For positive friction, `reaction = D * z` maps the Coulomb cone to the
/// standard Lorentz cone. The transformed Hessian is factored once and reduced
/// to a three-term secular equation. The two dense transforms make each later
/// solve two 3x3 matrix-vector products plus scalar root arithmetic.
struct ExactCoulombLocalConeQuadraticFactorization
{
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d linearToSpectral = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d spectralToReaction = Eigen::Matrix3d::Zero();
  Eigen::Vector2d negativeRatios = Eigen::Vector2d::Zero();
  double coefficient = std::numeric_limits<double>::quiet_NaN();
  bool frictionless = false;
  bool valid = false;
};

/// Prepare an exact local cone-QP factorization without perturbing the Hessian.
inline bool prepareExactCoulombLocalConeQuadraticFactorization(
    const Eigen::Matrix3d& hessian,
    double coefficient,
    ExactCoulombLocalConeQuadraticFactorization& factorization)
{
  factorization = ExactCoulombLocalConeQuadraticFactorization();
  if (!isValidCoulombConeCoefficient(coefficient)) {
    return false;
  }

  Eigen::Matrix3d symmetricHessian;
  if (!prepareExactCoulombLocalSpdHessian(hessian, symmetricHessian)) {
    return false;
  }
  factorization.hessian = symmetricHessian;
  factorization.coefficient = coefficient;
  factorization.frictionless = coefficient == 0.0;
  if (factorization.frictionless) {
    if (!std::isfinite(symmetricHessian(0, 0))
        || symmetricHessian(0, 0) <= 0.0) {
      return false;
    }
    factorization.valid = true;
    return true;
  }

  Eigen::Matrix3d coneScale = Eigen::Matrix3d::Identity();
  coneScale(1, 1) = coefficient;
  coneScale(2, 2) = coefficient;
  const Eigen::Matrix3d transformedHessian
      = coneScale * symmetricHessian * coneScale;
  if (!transformedHessian.allFinite()) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> transformedFactorization(transformedHessian);
  if (transformedFactorization.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Matrix3d upper = transformedFactorization.matrixU();
  const Eigen::Matrix3d inverseUpper
      = upper.triangularView<Eigen::Upper>().solve(Eigen::Matrix3d::Identity());
  if (!inverseUpper.allFinite()) {
    return false;
  }

  Eigen::Matrix3d lorentzSignature = -Eigen::Matrix3d::Identity();
  lorentzSignature(0, 0) = 1.0;
  Eigen::Matrix3d reducedSignature
      = inverseUpper.transpose() * lorentzSignature * inverseUpper;
  reducedSignature = 0.5 * (reducedSignature + reducedSignature.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> signatureSolver(
      reducedSignature);
  if (signatureSolver.info() != Eigen::Success
      || !signatureSolver.eigenvalues().allFinite()
      || !signatureSolver.eigenvectors().allFinite()) {
    return false;
  }

  const Eigen::Vector3d orderedEigenvalues{
      signatureSolver.eigenvalues()[2],
      signatureSolver.eigenvalues()[0],
      signatureSolver.eigenvalues()[1],
  };
  const double eigenvalueScale = orderedEigenvalues.cwiseAbs().maxCoeff();
  const double signTolerance
      = 128.0 * std::numeric_limits<double>::epsilon() * eigenvalueScale;
  if (!std::isfinite(eigenvalueScale) || eigenvalueScale <= 0.0
      || orderedEigenvalues[0] <= signTolerance
      || orderedEigenvalues[1] >= -signTolerance
      || orderedEigenvalues[2] >= -signTolerance) {
    return false;
  }

  Eigen::Matrix3d orderedEigenvectors;
  orderedEigenvectors.col(0) = signatureSolver.eigenvectors().col(2);
  orderedEigenvectors.col(1) = signatureSolver.eigenvectors().col(0);
  orderedEigenvectors.col(2) = signatureSolver.eigenvectors().col(1);

  factorization.negativeRatios
      = -orderedEigenvalues.tail<2>() / orderedEigenvalues[0];
  factorization.linearToSpectral
      = orderedEigenvectors.transpose() * inverseUpper.transpose() * coneScale;
  factorization.spectralToReaction
      = coneScale * inverseUpper * orderedEigenvectors;
  if (!factorization.negativeRatios.allFinite()
      || (factorization.negativeRatios.array() <= 0.0).any()
      || !factorization.linearToSpectral.allFinite()
      || !factorization.spectralToReaction.allFinite()) {
    return false;
  }

  factorization.valid = true;
  return true;
}

template <typename Function, typename Derivative>
inline bool solveExactCoulombMonotoneUnitIntervalRoot(
    const Function& function,
    const Derivative& derivative,
    long double functionAtZero,
    long double& root)
{
  if (!std::isfinite(functionAtZero) || functionAtZero >= 0.0L) {
    return false;
  }

  long double lower = 0.0L;
  long double upper = 1.0L;
  long double iterate = 0.5L;
  constexpr int kMaxIterations = 80;
  const long double intervalTolerance
      = 64.0L * std::numeric_limits<double>::epsilon();
  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    const long double value = function(iterate);
    if (!std::isfinite(value)) {
      upper = iterate;
      iterate = 0.5L * (lower + upper);
      continue;
    }

    if (value < 0.0L) {
      lower = iterate;
    } else {
      upper = iterate;
    }
    if (upper - lower <= intervalTolerance) {
      root = 0.5L * (lower + upper);
      return std::isfinite(root) && root > 0.0L && root < 1.0L;
    }

    const long double slope = derivative(iterate);
    long double candidate = std::numeric_limits<long double>::quiet_NaN();
    if (std::isfinite(slope) && slope > 0.0L) {
      candidate = iterate - value / slope;
    }
    if (!std::isfinite(candidate) || candidate <= lower || candidate >= upper) {
      candidate = 0.5L * (lower + upper);
    }
    iterate = candidate;
  }

  root = 0.5L * (lower + upper);
  return std::isfinite(root) && root > 0.0L && root < 1.0L;
}

inline bool isExactCoulombLocalConeQuadraticKktPoint(
    const ExactCoulombLocalConeQuadraticFactorization& factorization,
    const Eigen::Vector3d& linearTerm,
    const Eigen::Vector3d& reaction)
{
  if (!factorization.valid || !linearTerm.allFinite()
      || !reaction.allFinite()) {
    return false;
  }

  const double coefficient = factorization.coefficient;
  const Eigen::Vector3d gradient
      = factorization.hessian * reaction + linearTerm;
  if (!gradient.allFinite()) {
    return false;
  }

  const double reactionScale = (std::max)(1.0, reaction.norm());
  const double gradientScale = (std::max)(1.0, gradient.norm());
  const double coefficientScale = (std::max)(1.0, coefficient);
  const double coneTolerance
      = 4096.0 * std::numeric_limits<double>::epsilon() * coefficientScale;
  const double reactionTolerance = coneTolerance * reactionScale;
  const double gradientTolerance = coneTolerance * gradientScale;
  const double tangentNorm = std::hypot(reaction[1], reaction[2]);
  const double gradientTangentNorm = std::hypot(gradient[1], gradient[2]);

  const bool primalFeasible
      = coefficient == 0.0 ? reaction[0] >= -reactionTolerance
                                 && tangentNorm <= reactionTolerance
                           : reaction[0] >= -reactionTolerance
                                 && tangentNorm <= coefficient * reaction[0]
                                                       + reactionTolerance;
  if (!primalFeasible) {
    return false;
  }

  // The normal cone is exactly zero at a strictly interior reaction, so
  // stationarity is the complete dual/complementarity certificate there.
  // Requiring reaction.dot(gradient) as well only repeats stationarity after
  // multiplying by a potentially large reaction and can reject a solution
  // because of cancellation in hessian * reaction + linearTerm.
  const bool strictlyInterior
      = coefficient > 0.0 && reaction[0] > reactionTolerance
        && tangentNorm + reactionTolerance < coefficient * reaction[0];
  if (strictlyInterior) {
    return gradient.norm() <= gradientTolerance;
  }

  const bool dualFeasible
      = gradient[0] >= coefficient * gradientTangentNorm - gradientTolerance;
  const double gapTolerance = 8192.0 * std::numeric_limits<double>::epsilon()
                              * (1.0 + reaction.norm() * gradient.norm());
  return dualFeasible && std::abs(reaction.dot(gradient)) <= gapTolerance;
}

/// Recheck a rejected boundary candidate with fused long-double normal-cone
/// stationarity (higher precision where the platform provides it).
///
/// This is an independent approximate KKT certificate, not a relaxed
/// recomputation of the scalar complementarity gap. In exact arithmetic the
/// two boundary certificates are equivalent. In floating-point arithmetic,
/// normal-cone stationarity avoids multiplying component-level gradient error
/// by a potentially large reaction, so its tolerance decision need not match
/// the scalar-gap decision. The ordinary primal and dual tolerances are kept.
inline bool isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
    const ExactCoulombLocalConeQuadraticFactorization& factorization,
    const Eigen::Vector3d& linearTerm,
    const Eigen::Vector3d& reaction)
{
  if (!factorization.valid || factorization.frictionless
      || !linearTerm.allFinite() || !reaction.allFinite()) {
    return false;
  }

  const double coefficient = factorization.coefficient;
  const Eigen::Vector3d gradient
      = factorization.hessian * reaction + linearTerm;
  if (!gradient.allFinite()) {
    return false;
  }

  const double reactionScale = (std::max)(1.0, reaction.norm());
  const double gradientScale = (std::max)(1.0, gradient.norm());
  const double coefficientScale = (std::max)(1.0, coefficient);
  const double coneTolerance
      = 4096.0 * std::numeric_limits<double>::epsilon() * coefficientScale;
  const double reactionTolerance = coneTolerance * reactionScale;
  const double gradientTolerance = coneTolerance * gradientScale;
  const double tangentNorm = std::hypot(reaction[1], reaction[2]);
  const double gradientTangentNorm = std::hypot(gradient[1], gradient[2]);
  const bool primalFeasible
      = coefficient == 0.0 ? reaction[0] >= -reactionTolerance
                                 && tangentNorm <= reactionTolerance
                           : reaction[0] >= -reactionTolerance
                                 && tangentNorm <= coefficient * reaction[0]
                                                       + reactionTolerance;
  if (!primalFeasible) {
    return false;
  }

  const bool strictlyInterior
      = coefficient > 0.0 && reaction[0] > reactionTolerance
        && tangentNorm + reactionTolerance < coefficient * reaction[0];
  if (strictlyInterior) {
    return false;
  }
  const bool dualFeasible
      = gradient[0] >= coefficient * gradientTangentNorm - gradientTolerance;
  if (!dualFeasible) {
    return false;
  }

  using LongVector3 = Eigen::Matrix<long double, 3, 1>;
  LongVector3 refinedGradient;
  for (Eigen::Index row = 0; row < 3; ++row) {
    long double value = static_cast<long double>(linearTerm[row]);
    for (Eigen::Index column = 0; column < 3; ++column) {
      value = std::fma(
          static_cast<long double>(factorization.hessian(row, column)),
          static_cast<long double>(reaction[column]),
          value);
    }
    refinedGradient[row] = value;
  }
  const LongVector3 refinedReaction = reaction.cast<long double>();
  LongVector3 signature;
  const long double refinedCoefficient = static_cast<long double>(coefficient);
  signature << -refinedCoefficient * refinedCoefficient, 1.0L, 1.0L;
  const LongVector3 constraintGradient
      = signature.cwiseProduct(refinedReaction);
  const long double constraintGradientSquared
      = constraintGradient.squaredNorm();
  if (!refinedGradient.allFinite() || !constraintGradient.allFinite()
      || !std::isfinite(constraintGradientSquared)
      || constraintGradientSquared <= 0.0L) {
    return false;
  }
  const long double multiplier
      = -constraintGradient.dot(refinedGradient) / constraintGradientSquared;
  if (!std::isfinite(multiplier) || multiplier < 0.0L) {
    return false;
  }
  const LongVector3 stationarityResidual
      = refinedGradient + multiplier * constraintGradient;
  return stationarityResidual.allFinite()
         && stationarityResidual.norm()
                <= static_cast<long double>(gradientTolerance);
}

/// Try the closed-form spectral solve for one SPD quadratic contact block.
inline bool trySolveExactCoulombLocalConeQuadraticAnalytically(
    const ExactCoulombLocalConeQuadraticFactorization& factorization,
    const Eigen::Vector3d& linearTerm,
    Eigen::Vector3d& reaction)
{
  if (!factorization.valid || !linearTerm.allFinite()) {
    return false;
  }

  const double coefficient = factorization.coefficient;
  if (factorization.frictionless) {
    reaction = Eigen::Vector3d::Zero();
    reaction[0] = (std::max)(0.0, -linearTerm[0] / factorization.hessian(0, 0));
    return reaction.allFinite()
           && isExactCoulombLocalConeQuadraticKktPoint(
               factorization, linearTerm, reaction);
  }

  const Eigen::Vector3d transformedLinearTerm{
      linearTerm[0], coefficient * linearTerm[1], coefficient * linearTerm[2]};
  if (!transformedLinearTerm.allFinite()) {
    return false;
  }

  if (transformedLinearTerm[0]
      >= std::hypot(transformedLinearTerm[1], transformedLinearTerm[2])) {
    reaction.setZero();
    return isExactCoulombLocalConeQuadraticKktPoint(
        factorization, linearTerm, reaction);
  }

  const Eigen::Vector3d spectralLinearTerm
      = factorization.linearToSpectral * linearTerm;
  if (!spectralLinearTerm.allFinite()) {
    return false;
  }

  const Eigen::Vector3d unconstrainedReaction
      = factorization.spectralToReaction * (-spectralLinearTerm);
  if (!unconstrainedReaction.allFinite()) {
    return false;
  }
  if (unconstrainedReaction[0] >= 0.0
      && std::hypot(unconstrainedReaction[1], unconstrainedReaction[2])
             <= coefficient * unconstrainedReaction[0]) {
    reaction = unconstrainedReaction;
    return isExactCoulombLocalConeQuadraticKktPoint(
        factorization, linearTerm, reaction);
  }

  const double spectralScale = spectralLinearTerm.cwiseAbs().maxCoeff();
  if (!std::isfinite(spectralScale) || spectralScale <= 0.0) {
    return false;
  }
  const Eigen::Vector3d scaledLinearTerm = spectralLinearTerm / spectralScale;
  const long double positiveCoefficient
      = std::abs(static_cast<long double>(scaledLinearTerm[0]));
  const long double scaledNorm
      = static_cast<long double>(scaledLinearTerm.norm());
  const long double poleTolerance
      = 128.0L * std::numeric_limits<double>::epsilon() * scaledNorm;
  const Eigen::Vector2d& ratios = factorization.negativeRatios;

  const auto acceptSpectralCandidate = [&](const Eigen::Vector3d& spectral) {
    const Eigen::Vector3d candidate
        = factorization.spectralToReaction * spectral;
    if (!isExactCoulombLocalConeQuadraticKktPoint(
            factorization, linearTerm, candidate)
        && !isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
            factorization, linearTerm, candidate)) {
      return false;
    }
    reaction = candidate;
    return true;
  };

  // If the linear term lies in the range of the singular shifted system, the
  // unique cone solution sits exactly at the positive generalized pole.
  if (positiveCoefficient <= poleTolerance) {
    Eigen::Vector3d poleSpectral = Eigen::Vector3d::Zero();
    for (Eigen::Index index = 0; index < 2; ++index) {
      poleSpectral[index + 1]
          = -spectralLinearTerm[index + 1] / (1.0 + ratios[index]);
    }
    const double positiveMagnitude = std::sqrt(
        (ratios.array() * poleSpectral.tail<2>().array().square()).sum());
    poleSpectral[0] = positiveMagnitude;
    if (acceptSpectralCandidate(poleSpectral)) {
      return true;
    }
    poleSpectral[0] = -positiveMagnitude;
    if (acceptSpectralCandidate(poleSpectral)) {
      return true;
    }
  }

  const long double firstNegative
      = static_cast<long double>(scaledLinearTerm[1]);
  const long double secondNegative
      = static_cast<long double>(scaledLinearTerm[2]);
  const long double firstRatio = static_cast<long double>(ratios[0]);
  const long double secondRatio = static_cast<long double>(ratios[1]);

  const auto leftFunction = [&](long double value) {
    const long double firstDenominator = 1.0L + firstRatio * value;
    const long double secondDenominator = 1.0L + secondRatio * value;
    const long double negativeSquared
        = firstRatio * firstNegative * firstNegative
              / (firstDenominator * firstDenominator)
          + secondRatio * secondNegative * secondNegative
                / (secondDenominator * secondDenominator);
    return positiveCoefficient / (1.0L - value)
           - std::sqrt((std::max)(0.0L, negativeSquared));
  };
  const auto leftDerivative = [&](long double value) {
    const long double firstDenominator = 1.0L + firstRatio * value;
    const long double secondDenominator = 1.0L + secondRatio * value;
    const long double negativeSquared
        = firstRatio * firstNegative * firstNegative
              / (firstDenominator * firstDenominator)
          + secondRatio * secondNegative * secondNegative
                / (secondDenominator * secondDenominator);
    const long double negativeNorm
        = std::sqrt((std::max)(0.0L, negativeSquared));
    if (negativeNorm == 0.0L) {
      return std::numeric_limits<long double>::infinity();
    }
    const long double negativeDerivative
        = firstRatio * firstRatio * firstNegative * firstNegative
              / (firstDenominator * firstDenominator * firstDenominator)
          + secondRatio * secondRatio * secondNegative * secondNegative
                / (secondDenominator * secondDenominator * secondDenominator);
    return positiveCoefficient / ((1.0L - value) * (1.0L - value))
           + negativeDerivative / negativeNorm;
  };

  long double root = std::numeric_limits<long double>::quiet_NaN();
  const long double leftAtZero = leftFunction(0.0L);
  if (solveExactCoulombMonotoneUnitIntervalRoot(
          leftFunction, leftDerivative, leftAtZero, root)) {
    const double leftRoot = static_cast<double>(root);
    Eigen::Vector3d spectral;
    spectral[0] = -spectralLinearTerm[0] / (1.0 - leftRoot);
    spectral[1] = -spectralLinearTerm[1] / (1.0 + ratios[0] * leftRoot);
    spectral[2] = -spectralLinearTerm[2] / (1.0 + ratios[1] * leftRoot);
    if (spectral.allFinite() && acceptSpectralCandidate(spectral)) {
      return true;
    }
  }

  const auto rightFunction = [&](long double value) {
    const long double firstDenominator = value + firstRatio;
    const long double secondDenominator = value + secondRatio;
    const long double negativeSquared
        = firstRatio * firstNegative * firstNegative
              / (firstDenominator * firstDenominator)
          + secondRatio * secondNegative * secondNegative
                / (secondDenominator * secondDenominator);
    return positiveCoefficient / (1.0L - value)
           - std::sqrt((std::max)(0.0L, negativeSquared));
  };
  const auto rightDerivative = [&](long double value) {
    const long double firstDenominator = value + firstRatio;
    const long double secondDenominator = value + secondRatio;
    const long double negativeSquared
        = firstRatio * firstNegative * firstNegative
              / (firstDenominator * firstDenominator)
          + secondRatio * secondNegative * secondNegative
                / (secondDenominator * secondDenominator);
    const long double negativeNorm
        = std::sqrt((std::max)(0.0L, negativeSquared));
    if (negativeNorm == 0.0L) {
      return std::numeric_limits<long double>::infinity();
    }
    const long double negativeDerivative
        = firstRatio * firstNegative * firstNegative
              / (firstDenominator * firstDenominator * firstDenominator)
          + secondRatio * secondNegative * secondNegative
                / (secondDenominator * secondDenominator * secondDenominator);
    return positiveCoefficient / ((1.0L - value) * (1.0L - value))
           + negativeDerivative / negativeNorm;
  };

  const long double rightAtZero = rightFunction(0.0L);
  if (solveExactCoulombMonotoneUnitIntervalRoot(
          rightFunction, rightDerivative, rightAtZero, root)) {
    const double rightRoot = static_cast<double>(root);
    Eigen::Vector3d spectral;
    spectral[0] = -spectralLinearTerm[0] * rightRoot / (rightRoot - 1.0);
    spectral[1] = -spectralLinearTerm[1] * rightRoot / (rightRoot + ratios[0]);
    spectral[2] = -spectralLinearTerm[2] * rightRoot / (rightRoot + ratios[1]);
    if (spectral.allFinite() && acceptSpectralCandidate(spectral)) {
      return true;
    }
  }

  return false;
}

/// Solve a local SPD cone QP with a contractive projected-gradient map.
///
/// This is the rare numerical fallback for a closed-form spectral solve whose
/// candidate cannot pass the KKT certificate, most commonly when the solution
/// lies extremely close to a generalized pole. For eigenvalues `m` and `L`,
/// the step `2 / (m + L)` makes the affine gradient map contractive with
/// factor `(L - m) / (L + m)`. Euclidean projection is non-expansive, so the
/// complete fixed-point iteration is contractive as well. A candidate is
/// returned only after it passes the same primal, dual, and complementarity
/// KKT certificate as the analytical path.
inline bool solveExactCoulombLocalConeQuadraticProjectedGradientFallback(
    const ExactCoulombLocalConeQuadraticFactorization& factorization,
    const Eigen::Vector3d& linearTerm,
    Eigen::Vector3d& reaction)
{
  if (!factorization.valid || !linearTerm.allFinite()) {
    return false;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
      factorization.hessian, Eigen::EigenvaluesOnly);
  if (eigenSolver.info() != Eigen::Success
      || !eigenSolver.eigenvalues().allFinite()) {
    return false;
  }
  const double minimumEigenvalue = eigenSolver.eigenvalues().minCoeff();
  const double maximumEigenvalue = eigenSolver.eigenvalues().maxCoeff();
  if (!std::isfinite(minimumEigenvalue) || !std::isfinite(maximumEigenvalue)
      || minimumEigenvalue <= 0.0 || maximumEigenvalue < minimumEigenvalue) {
    return false;
  }

  // Form the optimal constant step without overflowing m + L.
  const double stepSize = (2.0 / maximumEigenvalue)
                          / (1.0 + minimumEigenvalue / maximumEigenvalue);
  if (!std::isfinite(stepSize) || stepSize <= 0.0) {
    return false;
  }

  reaction = projectCoulombConeNormalFirst(
      -stepSize * linearTerm, factorization.coefficient);
  if (!reaction.allFinite()) {
    return false;
  }

  constexpr int kMaximumIterations = 4096;
  for (int iteration = 0; iteration < kMaximumIterations; ++iteration) {
    if (isExactCoulombLocalConeQuadraticKktPoint(
            factorization, linearTerm, reaction)) {
      return true;
    }
    const Eigen::Vector3d gradient
        = factorization.hessian * reaction + linearTerm;
    if (!gradient.allFinite()) {
      return false;
    }
    const Eigen::Vector3d nextReaction = projectCoulombConeNormalFirst(
        reaction - stepSize * gradient, factorization.coefficient);
    if (!nextReaction.allFinite()) {
      return false;
    }
    reaction = nextReaction;
  }

  if (isExactCoulombLocalConeQuadraticKktPoint(
          factorization, linearTerm, reaction)) {
    return true;
  }
  return isExactCoulombLocalConeQuadraticBoundaryNormalConeKktPoint(
      factorization, linearTerm, reaction);
}

/// Solve one SPD quadratic contact block exactly over its Coulomb cone.
inline bool solveExactCoulombLocalConeQuadratic(
    const ExactCoulombLocalConeQuadraticFactorization& factorization,
    const Eigen::Vector3d& linearTerm,
    Eigen::Vector3d& reaction)
{
  if (trySolveExactCoulombLocalConeQuadraticAnalytically(
          factorization, linearTerm, reaction)) {
    return true;
  }
  return solveExactCoulombLocalConeQuadraticProjectedGradientFallback(
      factorization, linearTerm, reaction);
}

/// Reusable storage for consecutive block Gauss-Seidel cone-QP solves.
///
/// Local Hessians and mode-specific inverse, exact-metric, or scalar-step data
/// are keyed by value, not by the address of caller-owned data. This makes
/// cache reuse explicit while ensuring resized or mutated inputs cannot leave
/// stale pointers in the workspace.
struct ExactCoulombFrozenConeBlockGaussSeidelWorkspace
{
  /// Drop cached local systems and reusable scratch storage.
  void clear()
  {
    mDiagonalBlocks.clear();
    mCoefficients.resize(0);
    mLocalHessians.clear();
    mLocalInverses.clear();
    mLocalConeFactorizations.clear();
    mLocalSteps.clear();
    mComputedDiagonalBlocksScratch.clear();
    mDelassusReactionScratch.resize(0);
    mBasisDeltaScratch.resize(0);
    mColumnProductScratch.resize(0);
    mStepSizeGamma = std::numeric_limits<double>::quiet_NaN();
    mDiagonalRegularization = std::numeric_limits<double>::quiet_NaN();
    mMinimumLocalStep = std::numeric_limits<double>::infinity();
    mLocalSystemsValid = false;
    mLocalSystemBuildCount = 0;
  }

  /// Number of successful local-system cache builds since construction or
  /// `clear()`. Intended for focused diagnostics and regression tests.
  std::size_t getLocalSystemBuildCount() const
  {
    return mLocalSystemBuildCount;
  }

  bool prepareLocalSystems(
      const std::vector<Eigen::Matrix3d>& diagonalBlocks,
      const Eigen::Ref<const Eigen::VectorXd>& coefficients,
      double stepSizeGamma,
      ExactCoulombFrozenConeLocalSolver localSolver,
      double diagonalRegularization)
  {
    const double effectiveDiagonalRegularization
        = localSolver == ExactCoulombFrozenConeLocalSolver::ProjectedGradient
              ? diagonalRegularization
              : 0.0;
    bool cacheMatches
        = mLocalSystemsValid && mStepSizeGamma == stepSizeGamma
          && mLocalSolver == localSolver
          && mDiagonalRegularization == effectiveDiagonalRegularization
          && mDiagonalBlocks.size() == diagonalBlocks.size()
          && mCoefficients.size() == coefficients.size()
          && (mCoefficients.array() == coefficients.array()).all();
    if (cacheMatches) {
      for (std::size_t block = 0; block < diagonalBlocks.size(); ++block) {
        if (!(mDiagonalBlocks[block].array() == diagonalBlocks[block].array())
                 .all()) {
          cacheMatches = false;
          break;
        }
      }
    }
    if (cacheMatches) {
      return true;
    }

    mLocalSystemsValid = false;
    if (!std::isfinite(stepSizeGamma) || stepSizeGamma <= 0.0
        || !std::isfinite(effectiveDiagonalRegularization)
        || effectiveDiagonalRegularization < 0.0
        || coefficients.size()
               != static_cast<Eigen::Index>(diagonalBlocks.size())
        || !coefficients.allFinite() || (coefficients.array() < 0.0).any()) {
      return false;
    }

    mDiagonalBlocks = diagonalBlocks;
    mCoefficients = coefficients;
    mLocalHessians.resize(diagonalBlocks.size());
    mLocalInverses.clear();
    mLocalConeFactorizations.clear();
    mLocalSteps.clear();
    if (localSolver
        == ExactCoulombFrozenConeLocalSolver::InverseEuclideanProjection) {
      mLocalInverses.resize(diagonalBlocks.size());
    } else if (
        localSolver
        == ExactCoulombFrozenConeLocalSolver::ExactMetricProjection) {
      mLocalConeFactorizations.resize(diagonalBlocks.size());
    } else if (
        localSolver == ExactCoulombFrozenConeLocalSolver::ProjectedGradient) {
      mLocalSteps.resize(diagonalBlocks.size());
    } else {
      return false;
    }
    const double inverseGamma = 1.0 / stepSizeGamma;
    if (!std::isfinite(inverseGamma)) {
      return false;
    }
    const Eigen::Matrix3d gammaHessian
        = inverseGamma * Eigen::Matrix3d::Identity();
    double minimumLocalStep = std::numeric_limits<double>::infinity();
    for (std::size_t block = 0; block < diagonalBlocks.size(); ++block) {
      const Eigen::Matrix3d requestedHessian
          = diagonalBlocks[block] + gammaHessian;
      if (localSolver
          == ExactCoulombFrozenConeLocalSolver::InverseEuclideanProjection) {
        if (!prepareExactCoulombLocalSpdInverse(
                requestedHessian,
                mLocalHessians[block],
                mLocalInverses[block])) {
          return false;
        }
      } else if (
          localSolver
          == ExactCoulombFrozenConeLocalSolver::ExactMetricProjection) {
        if (!prepareExactCoulombLocalConeQuadraticFactorization(
                requestedHessian,
                coefficients[static_cast<Eigen::Index>(block)],
                mLocalConeFactorizations[block])) {
          return false;
        }
        mLocalHessians[block] = mLocalConeFactorizations[block].hessian;
      } else {
        if (!prepareExactCoulombLocalSpdHessian(
                requestedHessian, mLocalHessians[block])) {
          return false;
        }
        const double localLipschitz = computeExactCoulombLocalBlockLipschitz(
            mLocalHessians[block], effectiveDiagonalRegularization);
        if (!std::isfinite(localLipschitz) || localLipschitz <= 0.0) {
          return false;
        }
        mLocalSteps[block] = 1.0 / localLipschitz;
        minimumLocalStep = (std::min)(minimumLocalStep, mLocalSteps[block]);
      }
    }

    mStepSizeGamma = stepSizeGamma;
    mLocalSolver = localSolver;
    mDiagonalRegularization = effectiveDiagonalRegularization;
    mMinimumLocalStep = minimumLocalStep;
    mLocalSystemsValid = true;
    ++mLocalSystemBuildCount;
    return true;
  }

  const std::vector<Eigen::Matrix3d>& getLocalHessians() const
  {
    return mLocalHessians;
  }

  const std::vector<ExactCoulombLocalConeQuadraticFactorization>&
  getLocalConeFactorizations() const
  {
    return mLocalConeFactorizations;
  }

  const std::vector<Eigen::Matrix3d>& getLocalInverses() const
  {
    return mLocalInverses;
  }

  const std::vector<double>& getLocalSteps() const
  {
    return mLocalSteps;
  }

  double getMinimumLocalStep() const
  {
    return mMinimumLocalStep;
  }

  std::vector<Eigen::Matrix3d>& getComputedDiagonalBlocksScratch()
  {
    return mComputedDiagonalBlocksScratch;
  }

  Eigen::VectorXd& getDelassusReactionScratch(Eigen::Index dimension)
  {
    mDelassusReactionScratch.resize(dimension);
    return mDelassusReactionScratch;
  }

  Eigen::VectorXd& getBasisDeltaScratch(Eigen::Index dimension)
  {
    mBasisDeltaScratch.setZero(dimension);
    return mBasisDeltaScratch;
  }

  Eigen::VectorXd& getColumnProductScratch(Eigen::Index dimension)
  {
    mColumnProductScratch.resize(dimension);
    return mColumnProductScratch;
  }

private:
  std::vector<Eigen::Matrix3d> mDiagonalBlocks;
  Eigen::VectorXd mCoefficients;
  std::vector<Eigen::Matrix3d> mLocalHessians;
  std::vector<Eigen::Matrix3d> mLocalInverses;
  std::vector<ExactCoulombLocalConeQuadraticFactorization>
      mLocalConeFactorizations;
  std::vector<double> mLocalSteps;
  std::vector<Eigen::Matrix3d> mComputedDiagonalBlocksScratch;
  Eigen::VectorXd mDelassusReactionScratch;
  Eigen::VectorXd mBasisDeltaScratch;
  Eigen::VectorXd mColumnProductScratch;
  double mStepSizeGamma = std::numeric_limits<double>::quiet_NaN();
  double mDiagonalRegularization = std::numeric_limits<double>::quiet_NaN();
  double mMinimumLocalStep = std::numeric_limits<double>::infinity();
  ExactCoulombFrozenConeLocalSolver mLocalSolver
      = ExactCoulombFrozenConeLocalSolver::ExactMetricProjection;
  bool mLocalSystemsValid = false;
  std::size_t mLocalSystemBuildCount = 0;
};

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
    // Invalid caller-controlled solver input is part of this helper's status
    // contract.  Do not assert here: assert-enabled builds must return the
    // same fail-closed result as release builds.
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
/// applies the selected per-contact inverse/projection, exact metric solve, or
/// scalar projected-gradient update using cached 3x3 local data.
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
    // Invalid caller-controlled solver input is part of this helper's status
    // contract. Do not assert here: assert-enabled builds must return the same
    // fail-closed result as release builds.
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  const bool hasInitialReaction = options.initialReaction != nullptr;
  if (hasInitialReaction
      && (options.initialReaction->size() != problem.getDimension()
          || !options.initialReaction->allFinite())) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }
  if (options.projectInitialReaction) {
    const bool projected
        = hasInitialReaction ? projectExactCoulombReactionNormalFirst(
              *options.initialReaction, problem.coefficients, result.reaction)
                             : projectExactCoulombReactionNormalFirst(
                                 referenceReaction,
                                 problem.coefficients,
                                 result.reaction);
    if (!projected) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }
  } else {
    if (hasInitialReaction)
      result.reaction = *options.initialReaction;
    else
      result.reaction = referenceReaction;
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
    std::vector<Eigen::Matrix3d>* computedDiagonalBlocks = &localDiagonalBlocks;
    if (options.workspace != nullptr) {
      computedDiagonalBlocks
          = &options.workspace->getComputedDiagonalBlocksScratch();
    }
    if (!computeExactCoulombDelassusDiagonalBlocksNormalFirst(
            problem, applyDelassus, *computedDiagonalBlocks)) {
      result.status = ExactCoulombFrozenConeStatus::InvalidInput;
      return result;
    }
    diagonalBlocks = computedDiagonalBlocks;
  } else if (diagonalBlocks->size() != static_cast<std::size_t>(contactCount)) {
    // The optional cache is supplied by the caller.  Treat a stale/mismatched
    // cache as invalid input in assert-enabled and release builds alike.
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  const double inverseGamma = 1.0 / stepSizeGamma;
  if (!std::isfinite(inverseGamma)) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }

  // The Delassus blocks, coefficients, mode, and gamma are fixed for this call,
  // so all mode-specific local data is prepared once per solve.
  ExactCoulombFrozenConeBlockGaussSeidelWorkspace localSystemWorkspace;
  ExactCoulombFrozenConeBlockGaussSeidelWorkspace* systemWorkspace
      = options.workspace == nullptr ? &localSystemWorkspace
                                     : options.workspace;
  if (!systemWorkspace->prepareLocalSystems(
          *diagonalBlocks,
          problem.coefficients,
          stepSizeGamma,
          options.localSolver,
          options.diagonalRegularization)) {
    result.status = ExactCoulombFrozenConeStatus::InvalidInput;
    return result;
  }
  const auto& preparedLocalHessians = systemWorkspace->getLocalHessians();
  const auto& preparedLocalInverses = systemWorkspace->getLocalInverses();
  const auto& preparedLocalConeFactorizations
      = systemWorkspace->getLocalConeFactorizations();
  const auto& preparedLocalSteps = systemWorkspace->getLocalSteps();
  if (options.localSolver
      == ExactCoulombFrozenConeLocalSolver::ProjectedGradient) {
    result.stepSize = systemWorkspace->getMinimumLocalStep();
  }

  Eigen::VectorXd localDelassusReaction;
  if (options.workspace == nullptr) {
    localDelassusReaction.resize(dimension);
  }
  Eigen::VectorXd& delassusReaction
      = options.workspace == nullptr
            ? localDelassusReaction
            : options.workspace->getDelassusReactionScratch(dimension);
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
      const Eigen::Matrix3d& localHessian = preparedLocalHessians[blockIndex];

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

      const Eigen::Vector3d localLinearTerm
          = previousGradient - localHessian * previousBlock;
      Eigen::Vector3d block;
      bool localSolved = false;
      if (options.localSolver
          == ExactCoulombFrozenConeLocalSolver::InverseEuclideanProjection) {
        localSolved = solveExactCoulombLocalConeInverseEuclideanProjection(
            preparedLocalInverses[blockIndex],
            localLinearTerm,
            problem.coefficients[contact],
            block);
      } else if (
          options.localSolver
          == ExactCoulombFrozenConeLocalSolver::ExactMetricProjection) {
        localSolved = solveExactCoulombLocalConeQuadratic(
            preparedLocalConeFactorizations[blockIndex],
            localLinearTerm,
            block);
      } else {
        block = previousBlock;
        const double localStep = preparedLocalSteps[blockIndex];
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
        localSolved = block.allFinite();
      }
      if (!localSolved) {
        result.status = ExactCoulombFrozenConeStatus::InvalidInput;
        return result;
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
    if (!options.runFixedSweeps
        && result.fixedPointError <= options.tolerance) {
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
  Eigen::VectorXd localBasisDelta;
  Eigen::VectorXd localColumnProduct;
  Eigen::VectorXd* basisDelta = &localBasisDelta;
  Eigen::VectorXd* columnProduct = &localColumnProduct;
  if (options.workspace == nullptr) {
    localBasisDelta.setZero(dimension);
    localColumnProduct.resize(dimension);
  } else {
    basisDelta = &options.workspace->getBasisDeltaScratch(dimension);
    columnProduct = &options.workspace->getColumnProductScratch(dimension);
  }
  const auto accumulateThroughFullProduct
      = [&applyDelassus, basisDelta, columnProduct](
            Eigen::Index contact,
            const Eigen::Vector3d& delta,
            Eigen::Ref<Eigen::VectorXd> accumulator) {
          basisDelta->segment<3>(3 * contact) = delta;
          applyDelassus(*basisDelta, *columnProduct);
          accumulator += *columnProduct;
          basisDelta->segment<3>(3 * contact).setZero();
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
/// `solveFrozenConeProblem(problem, reaction, coupling, gamma, initial,
/// output)` must solve the strongly convex cone subproblem with frozen coupling
/// and write the intermediate reaction into `output`. By default, `initial` is
/// the previous accepted cone-QP solution and remains fixed across rejected
/// step-size trials. The source-reproduction option instead supplies the
/// current outer reaction on every trial. A legacy five-argument callback
/// without `initial` is also accepted.
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
    // Invalid caller-controlled solver input is part of this helper's status
    // contract. Do not assert here: assert-enabled builds must return the same
    // fail-closed result as release builds.
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }

  if (!projectExactCoulombReactionNormalFirst(
          initialReaction, problem.coefficients, result.reaction)) {
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }

  // Equation (17)'s power-iteration estimate is also used by the fixed
  // residual scale in equation (21), so compute it once per FBF solve.
  const double spectralRadius = estimateLargestExactCoulombDelassusEigenvalue(
      problem, applyDelassus, options.spectralIterations);
  result.safeStepSize = computeExactCoulombFbfSafeStepSizeFromSpectralRadius(
      problem, spectralRadius);

  double baseStepSize = options.initialStepSize;
  double automaticStepSize = std::numeric_limits<double>::infinity();
  if (std::isfinite(result.safeStepSize)) {
    automaticStepSize = result.safeStepSize * options.stepSizeScale;
  }
  if (std::isnan(baseStepSize)) {
    baseStepSize = std::isfinite(automaticStepSize) ? automaticStepSize : 1.0;
  }
  if (options.capInitialStepSizeAtSafeBound
      && std::isfinite(options.initialStepSize)
      && std::isfinite(result.safeStepSize)) {
    // Explicit gamma values, including a constraint solver's persisted gamma,
    // are capped at the fresh unscaled spectral bound. `stepSizeScale` applies
    // only when this solve chooses gamma automatically from a NaN request.
    const double explicitSafeBound
        = options.explicitStepSizeSafeBoundScale == 1.0
              ? result.safeStepSize
              : result.safeStepSize * options.explicitStepSizeSafeBoundScale;
    baseStepSize = (std::min)(baseStepSize, explicitSafeBound);
  }
  if (std::isfinite(options.minimumStepSize))
    baseStepSize = (std::max)(baseStepSize, options.minimumStepSize);
  if (std::isfinite(options.maximumStepSize))
    baseStepSize = (std::min)(baseStepSize, options.maximumStepSize);
  if (!std::isfinite(baseStepSize) || baseStepSize <= 0.0) {
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }
  result.uncappedInitialStepSize = baseStepSize;
  result.stepSize = baseStepSize;

  CoulombConeResidualScales residualScales = options.residualScales;
  if (options.useAutomaticResidualScales) {
    residualScales = computeExactCoulombFbfResidualScalesWithSpectralRadius(
        problem, result.reaction, applyDelassus, spectralRadius);
  }
  if (!isValidCoulombConeResidualScale(residualScales.reactionScale)
      || !isValidCoulombConeResidualScale(residualScales.velocityScale)) {
    result.status = ExactCoulombFbfStatus::InvalidInput;
    return result;
  }
  result.residualScales = residualScales;

  result.residual = computeExactCoulombContactResidualNormalFirst(
      problem, result.reaction, applyDelassus, residualScales);
  result.initialResidual = result.residual;
  if (options.reportNonFiniteValuesSeparately
      && !isFiniteCoulombConeResidual(result.residual)) {
    result.status = ExactCoulombFbfStatus::NonFiniteValue;
    return result;
  }
  const bool needsInitialNaturalMapResidual
      = options.useNaturalMapResidualForInitialStepSizeCap
        || options.useNaturalMapResidualForInitialConvergence
        || options.plateauPatience > 0;
  if (needsInitialNaturalMapResidual) {
    result.initialNaturalMapResidual
        = computeExactCoulombNaturalMapResidualNormalFirst(
            problem, result.reaction, applyDelassus);
    result.naturalMapResidual = result.initialNaturalMapResidual;
    if (!std::isfinite(result.initialNaturalMapResidual)) {
      result.status = options.reportNonFiniteValuesSeparately
                          ? ExactCoulombFbfStatus::NonFiniteValue
                          : ExactCoulombFbfStatus::InvalidInput;
      return result;
    }
  }
  const double initialStepSizeCapResidual
      = options.useNaturalMapResidualForInitialStepSizeCap
            ? result.initialNaturalMapResidual
            : result.initialResidual.value;
  if (std::isfinite(options.initialResidualStepSizeCapThreshold)
      && std::isfinite(options.initialResidualStepSizeCap)
      && std::isfinite(initialStepSizeCapResidual)
      && initialStepSizeCapResidual
             < options.initialResidualStepSizeCapThreshold
      && baseStepSize > options.initialResidualStepSizeCap) {
    baseStepSize = options.initialResidualStepSizeCap;
    if (std::isfinite(options.minimumStepSize))
      baseStepSize = (std::max)(baseStepSize, options.minimumStepSize);
    result.stepSize = baseStepSize;
    result.initialStepSizeCapApplied = true;
  }
  result.bestIteration = 0;
  result.bestResidual = result.residual;
  result.bestReaction = result.reaction;
  appendExactCoulombFbfResidualHistorySample(result, options, 0);
  const auto satisfiesTolerance = [&options](double residual) {
    return options.useStrictToleranceComparison ? residual < options.tolerance
                                                : residual <= options.tolerance;
  };
  const double initialConvergenceResidual
      = options.useNaturalMapResidualForInitialConvergence
            ? result.initialNaturalMapResidual
            : result.residual.value;
  if (satisfiesTolerance(initialConvergenceResidual)) {
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
  Eigen::VectorXd previousInnerReaction = result.reaction;
  const bool sampleNaturalMapResidual
      = options.plateauPatience > 0
        || options.useNaturalMapResidualForInitialConvergence;
  std::vector<double> plateauResidualHistory;
  if (options.plateauPatience > 0) {
    plateauResidualHistory.reserve(
        static_cast<std::size_t>(options.plateauPatience + 1));
  }
  const auto setFailureStatus = [&](ExactCoulombFbfStatus status) {
    result.status = status;
    if (result.iterations <= 0
        || result.iterations % options.residualCheckInterval == 0
        || !result.reaction.allFinite()) {
      return;
    }

    // A failure can follow one or more accepted iterations that were not
    // residual-sampling points. Refresh the returned telemetry so it describes
    // the current reaction instead of the last sampled (possibly initial)
    // iterate. This runs only on a terminating failure path.
    result.residual = computeExactCoulombContactResidualNormalFirst(
        problem, result.reaction, applyDelassus, residualScales);
    if (sampleNaturalMapResidual) {
      result.naturalMapResidual
          = computeExactCoulombNaturalMapResidualNormalFirst(
              problem, result.reaction, applyDelassus);
    }
  };

  for (int iteration = 0; iteration < options.maxOuterIterations; ++iteration) {
    double stepSize = baseStepSize;

    if (!computeExactCoulombContactVelocityNormalFirst(
            problem, result.reaction, applyDelassus, velocity)
        || !computeExactCoulombFbfCouplingNormalFirst(
            velocity, problem.coefficients, coupling)) {
      setFailureStatus(
          options.reportNonFiniteValuesSeparately
              ? ExactCoulombFbfStatus::NonFiniteValue
              : ExactCoulombFbfStatus::InvalidInput);
      return result;
    }

    bool accepted = false;
    const int maxShrinkIterations
        = options.enableAdaptiveStepSize ? options.maxStepShrinkIterations : 0;
    for (int shrink = 0; shrink <= maxShrinkIterations; ++shrink) {
      bool innerSolved = false;
      result.lastInnerSolveStepSize = stepSize;
      if constexpr (std::is_invocable_r_v<
                        bool,
                        const FrozenConeSolver&,
                        const ExactCoulombContactProblem&,
                        const Eigen::VectorXd&,
                        const Eigen::VectorXd&,
                        double,
                        const Eigen::VectorXd&,
                        Eigen::VectorXd&>) {
        const Eigen::VectorXd& innerInitialReaction
            = options.restartInnerFromCurrentOuterReaction
                  ? result.reaction
                  : previousInnerReaction;
        innerSolved = solveFrozenConeProblem(
            problem,
            result.reaction,
            coupling,
            stepSize,
            innerInitialReaction,
            trialReaction);
      } else {
        innerSolved = solveFrozenConeProblem(
            problem, result.reaction, coupling, stepSize, trialReaction);
      }
      if (!innerSolved
          || (!options.reportNonFiniteValuesSeparately
              && !trialReaction.allFinite())) {
        setFailureStatus(ExactCoulombFbfStatus::InnerSolverFailed);
        return result;
      }
      if (options.reportNonFiniteValuesSeparately
          && !trialReaction.allFinite()) {
        setFailureStatus(ExactCoulombFbfStatus::NonFiniteValue);
        return result;
      }

      if (!computeExactCoulombContactVelocityNormalFirst(
              problem, trialReaction, applyDelassus, trialVelocity)
          || !computeExactCoulombFbfCouplingNormalFirst(
              trialVelocity, problem.coefficients, trialCoupling)) {
        setFailureStatus(
            options.reportNonFiniteValuesSeparately
                ? ExactCoulombFbfStatus::NonFiniteValue
                : ExactCoulombFbfStatus::InvalidInput);
        return result;
      }

      double reactionChange = std::numeric_limits<double>::quiet_NaN();
      double couplingChange = std::numeric_limits<double>::quiet_NaN();
      if (options.reportNonFiniteValuesSeparately
          || options.couplingVariationSkipThreshold > 0.0) {
        reactionChange = (trialReaction - result.reaction).norm();
        couplingChange = (trialCoupling - coupling).norm();
        if (options.reportNonFiniteValuesSeparately) {
          if (!std::isfinite(reactionChange)
              || !std::isfinite(couplingChange)) {
            setFailureStatus(ExactCoulombFbfStatus::NonFiniteValue);
            return result;
          }
        }
        if (options.couplingVariationSkipThreshold > 0.0
            && reactionChange <= options.couplingVariationSkipThreshold) {
          result.couplingVariationRatio = -1.0;
          accepted = true;
          break;
        }
      }

      result.couplingVariationRatio
          = std::isnan(reactionChange)
                ? computeExactCoulombFbfCouplingVariationRatio(
                    result.reaction,
                    trialReaction,
                    coupling,
                    trialCoupling,
                    stepSize)
                : computeExactCoulombFbfCouplingVariationRatioFromNorms(
                    reactionChange, couplingChange, stepSize);
      if (options.reportNonFiniteValuesSeparately
          && (std::isnan(result.couplingVariationRatio)
              || (std::isinf(result.couplingVariationRatio)
                  && reactionChange != 0.0))) {
        setFailureStatus(ExactCoulombFbfStatus::NonFiniteValue);
        return result;
      }
      if (!options.enableAdaptiveStepSize
          || result.couplingVariationRatio
                 <= options.couplingVariationTolerance) {
        accepted = true;
        break;
      }

      if (options.acceptStepSizeAfterFinalShrink) {
        double shrunkenStepSize = stepSize * options.shrinkFactor;
        if (std::isfinite(options.minimumStepSize))
          shrunkenStepSize
              = (std::max)(shrunkenStepSize, options.minimumStepSize);
        if (!std::isfinite(shrunkenStepSize) || shrunkenStepSize <= 0.0) {
          setFailureStatus(ExactCoulombFbfStatus::StepSizeUnderflow);
          return result;
        }
        const bool sourceShrinkCapReached
            = shrunkenStepSize == stepSize || shrink + 1 == maxShrinkIterations;
        if (sourceShrinkCapReached) {
          stepSize = shrunkenStepSize;
          ++result.shrinkIterations;
          result.stepSize = stepSize;
          ++result.lineSearchShrinkCapCount;
          result.lineSearchShrinkCapReached = true;
          accepted = true;
          break;
        }
      }

      // The current gamma was the final permitted trial. Do not report a
      // smaller value that was never passed to the inner solver.
      if (shrink == maxShrinkIterations)
        break;

      stepSize *= options.shrinkFactor;
      if (std::isfinite(options.minimumStepSize))
        stepSize = (std::max)(stepSize, options.minimumStepSize);
      ++result.shrinkIterations;
      result.stepSize = stepSize;
      if (!std::isfinite(stepSize) || stepSize <= 0.0) {
        setFailureStatus(ExactCoulombFbfStatus::StepSizeUnderflow);
        return result;
      }
    }

    if (!accepted) {
      setFailureStatus(ExactCoulombFbfStatus::StepSizeUnderflow);
      return result;
    }

    // The paper's asymmetric policy never grows gamma again within one FBF
    // solve after a rejected trial. A new simulation substep may start from
    // the safe bound again, but later outer iterations in this solve inherit
    // the accepted shrunken value.
    baseStepSize = stepSize;
    previousInnerReaction = trialReaction;

    correctedReaction = trialReaction - stepSize * (trialCoupling - coupling);
    if (options.reportNonFiniteValuesSeparately
        && !correctedReaction.allFinite()) {
      setFailureStatus(ExactCoulombFbfStatus::NonFiniteValue);
      return result;
    }
    if (options.projectAfterCorrection) {
      if (!projectExactCoulombReactionNormalFirst(
              correctedReaction, problem.coefficients, projectedCorrection)) {
        setFailureStatus(
            options.reportNonFiniteValuesSeparately
                ? ExactCoulombFbfStatus::NonFiniteValue
                : ExactCoulombFbfStatus::InvalidInput);
        return result;
      }
      if (options.outerRelaxation == 1.0) {
        result.reaction = projectedCorrection;
      } else {
        relaxedReaction = result.reaction
                          + options.outerRelaxation
                                * (projectedCorrection - result.reaction);
        if (!projectExactCoulombReactionNormalFirst(
                relaxedReaction, problem.coefficients, result.reaction)) {
          setFailureStatus(
              options.reportNonFiniteValuesSeparately
                  ? ExactCoulombFbfStatus::NonFiniteValue
                  : ExactCoulombFbfStatus::InvalidInput);
          return result;
        }
      }
    } else if (options.outerRelaxation == 1.0) {
      result.reaction = correctedReaction;
    } else {
      result.reaction
          += options.outerRelaxation * (correctedReaction - result.reaction);
    }
    if (options.reportNonFiniteValuesSeparately
        && !result.reaction.allFinite()) {
      setFailureStatus(ExactCoulombFbfStatus::NonFiniteValue);
      return result;
    }

    result.stepSize = stepSize;
    result.iterations = iteration + 1;
    const bool sampleResidual
        = result.iterations % options.residualCheckInterval == 0
          || result.iterations == options.maxOuterIterations;
    if (sampleResidual) {
      result.residual = computeExactCoulombContactResidualNormalFirst(
          problem, result.reaction, applyDelassus, residualScales);
      if (options.reportNonFiniteValuesSeparately
          && !isFiniteCoulombConeResidual(result.residual)) {
        result.status = ExactCoulombFbfStatus::NonFiniteValue;
        return result;
      }
      if (std::isfinite(result.residual.value)
          && (!std::isfinite(result.bestResidual.value)
              || result.residual.value < result.bestResidual.value)) {
        result.bestIteration = result.iterations;
        result.bestResidual = result.residual;
        result.bestReaction = result.reaction;
      }

      if (sampleNaturalMapResidual) {
        result.naturalMapResidual
            = computeExactCoulombNaturalMapResidualNormalFirst(
                problem, result.reaction, applyDelassus);
        if (!std::isfinite(result.naturalMapResidual)) {
          result.status = ExactCoulombFbfStatus::NonFiniteValue;
          return result;
        }
      }
      appendExactCoulombFbfResidualHistorySample(
          result, options, result.iterations);
      if (satisfiesTolerance(result.residual.value)) {
        result.status = ExactCoulombFbfStatus::Success;
        return result;
      }

      if (options.plateauPatience > 0) {
        plateauResidualHistory.push_back(result.naturalMapResidual);
        const std::size_t patience
            = static_cast<std::size_t>(options.plateauPatience);
        if (plateauResidualHistory.size() > patience) {
          const double oldResidual = plateauResidualHistory
              [plateauResidualHistory.size() - 1u - patience];
          if (oldResidual > 0.0) {
            const double relativeImprovement
                = (oldResidual - result.naturalMapResidual) / oldResidual;
            if (relativeImprovement < options.plateauRelativeTolerance) {
              result.plateauReferenceNaturalMapResidual = oldResidual;
              result.plateauRelativeImprovement = relativeImprovement;
              result.status = ExactCoulombFbfStatus::Plateau;
              return result;
            }
          }
        }
      }
    }
  }

  result.status = ExactCoulombFbfStatus::MaxIterations;
  return result;
}

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_EXACTCOULOMBFBFSOLVER_HPP_
