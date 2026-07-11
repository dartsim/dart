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

#ifndef DART_CONSTRAINT_EXACTCOULOMBFBFCONSTRAINTSOLVER_HPP_
#define DART_CONSTRAINT_EXACTCOULOMBFBFCONSTRAINTSOLVER_HPP_

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/detail/ExactCoulombConstraintAdapter.hpp>

#include <dart/math/detail/ExactCoulombFbfSolver.hpp>

#include <Eigen/Core>

#include <limits>
#include <vector>

#include <cstddef>

namespace dart {

namespace dynamics {
class BodyNode;
} // namespace dynamics

namespace constraint {

/// Options for the opt-in exact-Coulomb FBF constraint solver.
struct ExactCoulombFbfConstraintSolverOptions
{
  /// Fall back to BoxedLcpConstraintSolver for unsupported or failed groups.
  bool fallbackToBoxedLcp = true;

  /// Include DART CFM/slip row regularization in the assembled Delassus matrix.
  bool includeConstraintRegularization = false;

  /// Use constraint impulse tests for exact-FBF Delassus products.
  ///
  /// The dense snapshot is still assembled for the current DART 6 staging
  /// bridge and prototype dense polish path. This option routes FBF `W*x`
  /// products through the retained constraints instead of multiplying by that
  /// snapshot, which stages the paper-style matrix-free operator without
  /// changing the default route.
  bool useMatrixFreeDelassusOperator = false;

  /// Use the scratch-backed contact-row Delassus operator when supported.
  ///
  /// For groups of exactly-typed non-self-collision three-row contact
  /// constraints between single free rigid bodies, this builds per-row body
  /// Jacobians and unit velocity changes once per solve, then serves `W*x`
  /// products, inner block-column updates, per-contact diagonal blocks, and
  /// the dense snapshot from that row data instead of DART impulse tests.
  /// This is the paper-style scatter/inverse-mass/gather product route.
  /// Unsupported groups automatically keep the impulse-test assembly, and
  /// `includeConstraintRegularization` disables this route because the row
  /// operator represents the pure reduced operator `J M^-1 J^T`.
  bool useContactRowDelassusOperator = true;

  /// Reuse the previous successful reaction for the same constraint sequence.
  bool enableWarmStart = true;

  /// Maximum world-space contact-point distance for manifold warm starts.
  ///
  /// DART recreates contact constraints every step, so pointer-sequence
  /// identity alone can never warm start consecutive `World` steps. When the
  /// pointer sequence does not match, contacts are matched across steps by
  /// (body pair, nearest world contact point within this distance) and the
  /// cached world-space reaction is re-projected onto the current contact
  /// frames and Coulomb cones.
  double warmStartMatchDistance = 0.025;

  /// Seed zero contact-row guesses from local/global Delassus estimates.
  bool seedNormalImpulseFromDiagonal = true;

  /// When using matrix-free Delassus products, seed from operator-extracted
  /// local Delassus diagonal blocks instead of the dense snapshot seed.
  ///
  /// This is a staging and benchmark option. It avoids the dense full-Delassus
  /// global seed, but the current DART 6 adapter still assembles a dense
  /// snapshot for compatibility and dense-polish diagnostics.
  bool useMatrixFreeDelassusSeed = false;

  /// Retry a failed exact solve with the matrix-free projected-gradient inner
  /// solver. This is slower than the block Gauss-Seidel path, but is useful as
  /// a robust reference for tightly coupled small contact sets.
  bool enableProjectedGradientRetry = true;

  /// Try a dense residual-polish step before boxed-LCP fallback.
  ///
  /// This is a prototype-only continuation aid for the current DART 6 dense
  /// Delassus route. It is not the paper-style matrix-free path.
  bool enableDenseResidualPolish = true;

  /// Maximum accepted FBF outer iterations.
  int maxOuterIterations = 50;

  /// Stop when the dimensionless exact-Coulomb residual is at or below this.
  double tolerance = 1e-6;

  /// Optional initial gamma. NaN means use the safe spectral estimate.
  double initialStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Multiplier applied to the safe spectral estimate when choosing gamma.
  ///
  /// The default preserves the conservative paper-derived safe estimate. Values
  /// above one are useful for benchmark-only continuation experiments when the
  /// local FBF coupling-variation line search still accepts the larger step.
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

  /// Maximum contact-block Gauss-Seidel sweeps in the frozen cone solve.
  int innerMaxSweeps = 10;

  /// Maximum local projected-gradient steps per 3D contact block update.
  int innerLocalIterations = 8;

  /// Relative sweep-change tolerance for the frozen cone solve.
  double innerTolerance = 1e-10;

  /// Relative tolerance for the local 3D projected-gradient subproblem.
  double innerLocalTolerance = 1e-12;

  /// Optional diagonal padding used only to choose conservative local steps.
  double innerDiagonalRegularization = 0.0;

  /// Maximum projected-gradient iterations in the retry frozen cone solve.
  int projectedGradientMaxIterations = 400;

  /// Relative fixed-point tolerance for the retry frozen cone solve.
  double projectedGradientTolerance = 1e-12;

  /// Maximum dense residual-polish iterations after an FBF failure.
  int denseResidualPolishIterations = 8;

  /// Backtracking attempts per dense residual-polish iteration.
  int denseResidualPolishLineSearchIterations = 8;

  /// Relative diagonal regularization for the dense residual-polish solve.
  double denseResidualPolishRegularization = 1e-9;

  /// Maximum residual-history samples to retain from the last exact solve.
  ///
  /// Zero disables recording. This is intended for tests, examples, and
  /// benchmark/report tooling that needs paper-style convergence traces.
  int maxResidualHistorySamples = 0;

  /// Maximum per-solve residual-history records to retain.
  ///
  /// Zero disables multi-solve retention. This bounds contact-rich reporting
  /// where one `World::step()` may solve several exact-Coulomb groups.
  int maxResidualHistoryRecords = 0;
};

/// Last attempted route for ExactCoulombFbfConstraintSolver.
enum class ExactCoulombFbfConstraintSolverStatus
{
  NotRun,
  Success,
  InvalidOptions,
  UnsupportedProblem,
  FbfFailed,
  BoxedLcpFallback,
};

/// Retained residual-history diagnostics for one exact-Coulomb solve attempt.
struct ExactCoulombFbfResidualHistoryRecord
{
  std::size_t solveIndex = 0u;
  std::size_t contactCount = 0u;
  ExactCoulombFbfConstraintSolverStatus status
      = ExactCoulombFbfConstraintSolverStatus::NotRun;
  math::detail::ExactCoulombFbfStatus fbfStatus
      = math::detail::ExactCoulombFbfStatus::InvalidInput;
  int iterations = 0;
  std::vector<math::detail::ExactCoulombFbfResidualSample> samples;
};

/// Opt-in DART 6 constraint solver for exact reduced Coulomb friction.
///
/// The exact-Coulomb FBF route follows the splitting architecture introduced
/// by Hongcheng Song, Ye Fan, Uri M. Ascher, and Dinesh K. Pai in "A Splitting
/// Architecture for Exact Reduced Coulomb Friction" (SCA 2026).
///
/// This solver preserves the existing boxed-LCP path as a fallback. Only
/// supported all-contact groups with isotropic three-row contact constraints
/// are solved through the exact-Coulomb FBF path.
class ExactCoulombFbfConstraintSolver : public BoxedLcpConstraintSolver
{
public:
  /// Constructs with default FBF options and default boxed-LCP fallback.
  ExactCoulombFbfConstraintSolver();

  /// Constructs with explicit FBF options and default boxed-LCP fallback.
  explicit ExactCoulombFbfConstraintSolver(
      const ExactCoulombFbfConstraintSolverOptions& options);

  /// Set exact-Coulomb FBF options.
  void setExactCoulombOptions(
      const ExactCoulombFbfConstraintSolverOptions& options);

  /// Get exact-Coulomb FBF options.
  const ExactCoulombFbfConstraintSolverOptions& getExactCoulombOptions() const;

  /// Status from the last constrained group solve attempt.
  ExactCoulombFbfConstraintSolverStatus getLastExactCoulombStatus() const;

  /// Adapter status from the last constrained group solve attempt.
  detail::ExactCoulombConstraintBuildStatus getLastExactCoulombBuildStatus()
      const;

  /// FBF status from the last exact-Coulomb solve attempt.
  math::detail::ExactCoulombFbfStatus getLastExactCoulombFbfStatus() const;

  /// Residual from the last exact-Coulomb solve attempt.
  double getLastExactCoulombResidual() const;

  /// Best residual encountered during the last exact-Coulomb solve attempt.
  double getLastExactCoulombBestResidual() const;

  /// Iteration where the last exact-Coulomb solve saw its best residual.
  int getLastExactCoulombBestIteration() const;

  /// Full residual terms from the last exact-Coulomb solve attempt.
  const math::detail::CoulombConeResidual& getLastExactCoulombResidualDetails()
      const;

  /// Residual history retained from the last exact-Coulomb solve attempt.
  const std::vector<math::detail::ExactCoulombFbfResidualSample>&
  getLastExactCoulombResidualHistory() const;

  /// Residual-history records retained from recent exact-Coulomb attempts.
  const std::vector<ExactCoulombFbfResidualHistoryRecord>&
  getExactCoulombResidualHistoryRecords() const;

  /// Clear retained multi-solve residual-history records.
  void clearExactCoulombResidualHistoryRecords();

  /// FBF status from the most recent failed exact-Coulomb solve attempt.
  math::detail::ExactCoulombFbfStatus getLastFailedExactCoulombFbfStatus()
      const;

  /// Residual from the most recent failed exact-Coulomb solve attempt.
  double getLastFailedExactCoulombResidual() const;

  /// Best residual from the most recent failed exact-Coulomb solve attempt.
  double getLastFailedExactCoulombBestResidual() const;

  /// Iteration where the most recent failed exact solve saw its best residual.
  int getLastFailedExactCoulombBestIteration() const;

  /// Full residual terms from the most recent failed exact-Coulomb solve.
  const math::detail::CoulombConeResidual&
  getLastFailedExactCoulombResidualDetails() const;

  /// Outer FBF iterations from the most recent failed exact-Coulomb solve.
  int getLastFailedExactCoulombIterations() const;

  /// Outer FBF iterations from the last exact-Coulomb solve attempt.
  int getLastExactCoulombIterations() const;

  /// Final FBF step size from the last exact-Coulomb solve attempt.
  double getLastExactCoulombStepSize() const;

  /// Safe spectral FBF step size from the last exact-Coulomb solve attempt.
  double getLastExactCoulombSafeStepSize() const;

  /// Last accepted coupling-variation ratio from the last exact solve attempt.
  double getLastExactCoulombCouplingVariationRatio() const;

  /// Total line-search shrink count from the last exact solve attempt.
  int getLastExactCoulombShrinkIterations() const;

  /// Final FBF step size from the most recent failed exact-Coulomb solve.
  double getLastFailedExactCoulombStepSize() const;

  /// Safe spectral FBF step size from the most recent failed exact solve.
  double getLastFailedExactCoulombSafeStepSize() const;

  /// Last accepted coupling-variation ratio from the most recent failure.
  double getLastFailedExactCoulombCouplingVariationRatio() const;

  /// Total line-search shrink count from the most recent failed exact solve.
  int getLastFailedExactCoulombShrinkIterations() const;

  /// Maximum outer FBF iterations used by any exact-Coulomb attempt so far.
  int getMaxExactCoulombIterations() const;

  /// Total outer FBF iterations used by exact-Coulomb attempts so far.
  std::size_t getTotalExactCoulombIterations() const;

  /// Whether the projected-gradient retry was attempted in the last solve.
  bool getLastExactCoulombProjectedGradientRetryUsed() const;

  /// Whether dense residual polishing was attempted in the last solve.
  bool getLastExactCoulombDenseResidualPolishUsed() const;

  /// Whether the last solve used constraint-row Delassus products.
  bool getLastExactCoulombMatrixFreeDelassusOperatorUsed() const;

  /// Whether the last solve used the scratch-backed contact-row operator.
  bool getLastExactCoulombContactRowOperatorUsed() const;

  /// Whether the last solve seeded from matrix-free Delassus diagonal blocks.
  bool getLastExactCoulombMatrixFreeDelassusSeedUsed() const;

  /// Whether the last exact-Coulomb solve used the warm-start cache.
  bool getLastExactCoulombWarmStartUsed() const;

  /// Number of contacts warm-started by manifold matching in the last solve.
  std::size_t getLastExactCoulombWarmStartMatchedContacts() const;

  /// Number of groups solved through the exact-Coulomb path.
  std::size_t getNumExactCoulombSolves() const;

  /// Number of groups routed through boxed-LCP fallback.
  std::size_t getNumBoxedLcpFallbacks() const;

  /// Number of exact-Coulomb group solve attempts that failed before fallback.
  std::size_t getNumExactCoulombFailures() const;

  /// Number of exact-Coulomb solves that reused a cached warm start.
  std::size_t getNumExactCoulombWarmStarts() const;

  /// Number of groups that attempted the projected-gradient retry path.
  std::size_t getNumExactCoulombProjectedGradientRetries() const;

  /// Number of groups that attempted the dense residual-polish path.
  std::size_t getNumExactCoulombDenseResidualPolishes() const;

protected:
  // Documentation inherited.
  void solveConstrainedGroup(ConstrainedGroup& group) override;

private:
  bool trySolveExactCoulombConstrainedGroup(ConstrainedGroup& group);
  void tryPolishFailedExactCoulombSolution(
      const detail::ExactCoulombConstraintProblem& problem,
      math::detail::ExactCoulombFbfResult& solution);
  void recordLastFailedExactCoulombAttempt(
      math::detail::ExactCoulombFbfStatus status,
      const math::detail::CoulombConeResidual& residual,
      int iterations,
      double stepSize = std::numeric_limits<double>::quiet_NaN(),
      double safeStepSize = std::numeric_limits<double>::quiet_NaN(),
      double couplingVariationRatio = std::numeric_limits<double>::quiet_NaN(),
      int shrinkIterations = 0,
      double bestResidual = std::numeric_limits<double>::quiet_NaN(),
      int bestIteration = 0);
  bool tryApplyExactCoulombWarmStart(
      const std::vector<ConstraintBase*>& constraints,
      detail::ExactCoulombConstraintProblem& problem);
  void updateExactCoulombWarmStart(
      const std::vector<ConstraintBase*>& constraints,
      const Eigen::Ref<const Eigen::VectorXd>& reaction);
  void clearExactCoulombWarmStart();

  /// Invalidates only the same-step pointer-identity warm-start cache,
  /// keeping the pair-keyed cross-step manifold records of other islands.
  void invalidateExactCoulombWarmStartPointerCache();
  void recordExactCoulombResidualHistory(
      std::size_t contactCount,
      ExactCoulombFbfConstraintSolverStatus status,
      const math::detail::ExactCoulombFbfResult& solution);

  ExactCoulombFbfConstraintSolverOptions mExactCoulombOptions;
  ExactCoulombFbfConstraintSolverStatus mLastExactCoulombStatus
      = ExactCoulombFbfConstraintSolverStatus::NotRun;
  detail::ExactCoulombConstraintBuildStatus mLastExactCoulombBuildStatus
      = detail::ExactCoulombConstraintBuildStatus::EmptyInput;
  math::detail::ExactCoulombFbfStatus mLastExactCoulombFbfStatus
      = math::detail::ExactCoulombFbfStatus::InvalidInput;
  double mLastExactCoulombResidual = std::numeric_limits<double>::quiet_NaN();
  math::detail::CoulombConeResidual mLastExactCoulombResidualDetails
      = math::detail::makeInvalidCoulombConeResidual();
  double mLastExactCoulombBestResidual
      = std::numeric_limits<double>::quiet_NaN();
  int mLastExactCoulombBestIteration = 0;
  std::vector<math::detail::ExactCoulombFbfResidualSample>
      mLastExactCoulombResidualHistory;
  std::vector<ExactCoulombFbfResidualHistoryRecord>
      mExactCoulombResidualHistoryRecords;
  std::size_t mResidualHistoryRecordSequence = 0u;
  math::detail::ExactCoulombFbfStatus mLastFailedExactCoulombFbfStatus
      = math::detail::ExactCoulombFbfStatus::InvalidInput;
  double mLastFailedExactCoulombResidual
      = std::numeric_limits<double>::quiet_NaN();
  math::detail::CoulombConeResidual mLastFailedExactCoulombResidualDetails
      = math::detail::makeInvalidCoulombConeResidual();
  double mLastFailedExactCoulombBestResidual
      = std::numeric_limits<double>::quiet_NaN();
  int mLastFailedExactCoulombBestIteration = 0;
  int mLastFailedExactCoulombIterations = 0;
  int mLastExactCoulombIterations = 0;
  double mLastExactCoulombStepSize = std::numeric_limits<double>::quiet_NaN();
  double mLastExactCoulombSafeStepSize
      = std::numeric_limits<double>::quiet_NaN();
  double mLastExactCoulombCouplingVariationRatio
      = std::numeric_limits<double>::quiet_NaN();
  int mLastExactCoulombShrinkIterations = 0;
  double mLastFailedExactCoulombStepSize
      = std::numeric_limits<double>::quiet_NaN();
  double mLastFailedExactCoulombSafeStepSize
      = std::numeric_limits<double>::quiet_NaN();
  double mLastFailedExactCoulombCouplingVariationRatio
      = std::numeric_limits<double>::quiet_NaN();
  int mLastFailedExactCoulombShrinkIterations = 0;
  int mMaxExactCoulombIterations = 0;
  std::size_t mTotalExactCoulombIterations = 0u;
  bool mLastExactCoulombProjectedGradientRetryUsed = false;
  bool mLastExactCoulombDenseResidualPolishUsed = false;
  bool mLastExactCoulombMatrixFreeDelassusOperatorUsed = false;
  bool mLastExactCoulombContactRowOperatorUsed = false;
  bool mLastExactCoulombMatrixFreeDelassusSeedUsed = false;
  bool mLastExactCoulombWarmStartUsed = false;
  std::size_t mNumExactCoulombSolves = 0u;
  std::size_t mNumBoxedLcpFallbacks = 0u;
  std::size_t mNumExactCoulombFailures = 0u;
  std::size_t mNumExactCoulombWarmStarts = 0u;
  std::size_t mNumExactCoulombProjectedGradientRetries = 0u;
  std::size_t mNumExactCoulombDenseResidualPolishes = 0u;
  bool mHasExactCoulombWarmStart = false;
  std::vector<ConstraintBase*> mWarmStartConstraints;
  Eigen::VectorXd mWarmStartReaction;

  /// World-space per-contact record for manifold warm-start matching.
  struct ExactCoulombWarmStartContactRecord
  {
    const dynamics::BodyNode* bodyA = nullptr;
    const dynamics::BodyNode* bodyB = nullptr;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d worldImpulse = Eigen::Vector3d::Zero();
  };
  std::vector<ExactCoulombWarmStartContactRecord> mWarmStartContactRecords;
  std::size_t mLastExactCoulombWarmStartMatchedContacts = 0u;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_EXACTCOULOMBFBFCONSTRAINTSOLVER_HPP_
