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

/// Per-contact update used by the exact-Coulomb frozen cone block solver.
enum class ExactCoulombFbfLocalBlockSolver
{
  /// Apply the inverse 3x3 block, then project in the Euclidean cone metric.
  ///
  /// This is an explicit inexpensive approximation and is not generally the
  /// exact local cone-QP minimizer for an anisotropic block.
  InverseEuclideanProjection,

  /// Solve each local SPD cone quadratic exactly in its Hessian metric. This
  /// is the correctness-preserving default.
  ExactMetricProjection,

  /// Use the original scalar-step projected-gradient approximation.
  ProjectedGradient,
};

/// Cross-step contact identity used by exact-Coulomb warm starts.
enum class ExactCoulombFbfWarmStartMatchMode
{
  /// Match either body-local feature and allow reversed body-pair order.
  ///
  /// This preserves the existing DART exact-Coulomb warm-start policy.
  EitherBodyLocalFeature,

  /// Match only the body-B-local feature of an ordered body pair.
  ///
  /// This is an opt-in policy inspired by the pinned SCA 2026 author code.
  OrderedBodyBLocalFeature,
};

/// Additive cross-step policy for exact-Coulomb warm starts and gamma reuse.
///
/// This value type is stored outside the solver object so the DART 6 class
/// layout remains unchanged. Its defaults preserve the existing behavior.
struct ExactCoulombFbfCrossStepPolicyOptions
{
  /// Contact identity rule used for reaction and persisted-gamma matching.
  ExactCoulombFbfWarmStartMatchMode warmStartMatchMode
      = ExactCoulombFbfWarmStartMatchMode::EitherBodyLocalFeature;

  /// Minimum cosine between cached and current body-local normals.
  double warmStartNormalCosine = 0.9;

  /// Require distance to be strictly below `warmStartMatchDistance`.
  ///
  /// The default retains DART's inclusive distance boundary.
  bool useStrictWarmStartMatchDistance = false;

  /// Maximum consecutive unsuccessful cache updates before eviction.
  ///
  /// A negative value disables age-based eviction, preserving the existing
  /// DART behavior.
  int warmStartMaxAge = -1;

  /// Scale applied to the fresh safe bound for a persisted gamma request.
  double persistentStepSizeSafeBoundScale = 1.0;

  /// Optional lower gamma bound. NaN disables the bound.
  double minimumStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Optional upper gamma bound. NaN disables the bound.
  double maximumStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Initial residual below which `warmStartStepSizeCap` is applied.
  ///
  /// NaN disables the cap. The threshold and cap must be enabled together.
  double warmStartResidualThreshold = std::numeric_limits<double>::quiet_NaN();

  /// Gamma cap used to protect an already-good warm start.
  double warmStartStepSizeCap = std::numeric_limits<double>::quiet_NaN();

  /// Persist the pre-cap gamma when the warm-start gamma cap is used.
  bool persistUncappedStepSizeOnWarmStartCap = false;

  /// Save an accepted unconverged reaction only when it improves residual.
  ///
  /// Converged reactions are always saved. The applied finite reaction is not
  /// changed; this controls only the next step's cache contents.
  bool requireResidualImprovementForUnconvergedCacheSave = false;
};

/// Options for the opt-in exact-Coulomb FBF constraint solver.
struct ExactCoulombFbfConstraintSolverOptions
{
  /// Fall back to BoxedLcpConstraintSolver for unsupported or failed groups.
  ///
  /// Setting this false leaves every failed or unsupported constrained group
  /// unsolved. That mode is only for diagnostics/evidence; callers must inspect
  /// the status after each step and stop rather than advancing a partial world.
  bool fallbackToBoxedLcp = true;

  /// Include DART CFM/slip row regularization in the assembled Delassus matrix.
  bool includeConstraintRegularization = false;

  /// Use constraint impulse tests for exact-FBF Delassus products.
  ///
  /// The dense snapshot is still assembled for the current DART 6 staging
  /// bridge unless `assembleDenseContactRowSnapshot` is disabled. This option
  /// routes FBF `W*x` products through the retained constraints instead of
  /// multiplying by that snapshot.
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

  /// Assemble a dense snapshot even when the contact-row operator is active.
  ///
  /// The compatibility/default path retains this snapshot for the dense global
  /// cold-start seed and optional failed-solve polish. Disable it for the
  /// paper-style matrix-free path, which performs only per-body and
  /// per-contact operations during a successful solve. If dense polish is
  /// enabled, a failed solve assembles the snapshot lazily before polishing.
  bool assembleDenseContactRowSnapshot = true;

  /// Reuse the previous successful reaction for the same constraint sequence.
  bool enableWarmStart = true;

  /// Reuse and cautiously restore an accepted adaptive gamma across steps.
  ///
  /// The state is matched by the same constraint sequence or by the
  /// body-local contact manifold used for reaction warm starts. A solve that
  /// rejects any gamma stores the accepted shrunken value and suppresses
  /// growth for the next matching solve. Consecutive rejection-free solves
  /// multiply that value by `stepSizeRecoveryGrowthFactor`; the FBF solver's
  /// newly computed safe bound remains the cap. Explicit `initialStepSize`
  /// values and fixed-gamma solves never use this state.
  bool enableStepSizePersistence = true;

  /// Cross-step growth factor for a stable persisted gamma.
  double stepSizeRecoveryGrowthFactor = 1.05;

  /// Maximum body-local contact-point distance for manifold warm starts.
  ///
  /// DART may pool and reset contact constraints between steps, so
  /// pointer-sequence identity never establishes contact identity across
  /// `World` steps. Physical contacts are matched across steps by body pair,
  /// a stable body-local contact feature within this distance, and a
  /// consistent local normal. World-space matching is retained only when a
  /// contact side cannot expose a body-local feature. The cached world-space
  /// reaction is re-projected onto the current contact frames and cones.
  double warmStartMatchDistance = 0.025;

  /// Seed zero contact-row guesses from local/global Delassus estimates.
  bool seedNormalImpulseFromDiagonal = true;

  /// Seed from operator-extracted local Delassus diagonal blocks instead of
  /// the dense global snapshot seed.
  ///
  /// This is a staging and benchmark option. It avoids the dense full-Delassus
  /// global seed. It reuses the per-contact blocks already required by the
  /// frozen-cone solver, including on the contact-row route.
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

  /// Apply a finite FBF iterate when the configured outer budget is exhausted.
  ///
  /// The paper advances capped substeps and reports their residuals rather than
  /// replacing them with a different contact law. Enabling this option accepts
  /// only `MaxIterations`; invalid input, inner-solver failure, and step-size
  /// underflow remain failures. The distinct solver status keeps a capped
  /// substep from being mistaken for residual convergence.
  bool acceptOuterMaxIterations = false;

  /// Stop when the dimensionless exact-Coulomb residual is at or below this.
  double tolerance = 1e-6;

  /// Optional initial gamma. NaN means use the safe spectral estimate.
  double initialStepSize = std::numeric_limits<double>::quiet_NaN();

  /// Cap an explicitly configured gamma at the safe spectral estimate.
  ///
  /// Disable only for fixed-gamma diagnostics that must use
  /// `initialStepSize` exactly. Normal solver configurations should retain the
  /// conservative cap.
  bool capInitialStepSizeAtSafeBound = true;

  /// Multiplier applied to the safe spectral estimate when choosing gamma
  /// automatically (`initialStepSize` is NaN). Explicit and persisted gamma
  /// values are capped at the unscaled fresh safe estimate.
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

  /// Adapt gamma when the local coupling-variation test rejects a trial.
  ///
  /// Disable only for fixed-gamma diagnostics. The variation ratio remains
  /// available through the solver diagnostics even when it does not reject a
  /// trial.
  bool enableAdaptiveStepSize = true;

  /// Power iterations used when deriving the safe base step.
  int spectralIterations = 10;

  /// Maximum contact-block Gauss-Seidel sweeps in the frozen cone solve.
  int innerMaxSweeps = 10;

  /// Per-contact update used during each block Gauss-Seidel sweep.
  ExactCoulombFbfLocalBlockSolver innerLocalSolver
      = ExactCoulombFbfLocalBlockSolver::ExactMetricProjection;

  /// Run every configured inner sweep instead of stopping on sweep change.
  ///
  /// Enable this to reproduce the paper's fixed 10-sweep budget (30 on the
  /// masonry arch). The outer exact-Coulomb residual remains the convergence
  /// gate.
  bool runFixedInnerSweeps = false;

  /// Accept the finite iterate produced by the fixed inner sweep budget.
  ///
  /// The paper runs a fixed 10 block Gauss-Seidel sweeps per FBF outer
  /// iteration (30 for the masonry arch). Reaching that budget is therefore
  /// not an inner-solver failure: the outer exact-Coulomb residual remains the
  /// convergence gate. Disable this only for diagnostics that require every
  /// frozen cone subproblem to meet `innerTolerance` independently.
  bool acceptInnerMaxIterations = true;

  /// Maximum local steps in `ProjectedGradient` mode.
  int innerLocalIterations = 8;

  /// Relative sweep-change tolerance for the frozen cone solve.
  double innerTolerance = 1e-10;

  /// Relative local change tolerance in `ProjectedGradient` mode.
  double innerLocalTolerance = 1e-12;

  /// Diagonal padding used only to choose the `ProjectedGradient` step.
  ///
  /// This never changes the local quadratic objective in any mode.
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
  MaxIterationsAccepted,
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

  /// Destructor.
  ~ExactCoulombFbfConstraintSolver() override;

  /// Set exact-Coulomb FBF options.
  void setExactCoulombOptions(
      const ExactCoulombFbfConstraintSolverOptions& options);

  /// Get exact-Coulomb FBF options.
  const ExactCoulombFbfConstraintSolverOptions& getExactCoulombOptions() const;

  /// Set the additive cross-step warm-start and gamma policy.
  void setExactCoulombCrossStepPolicyOptions(
      const ExactCoulombFbfCrossStepPolicyOptions& options);

  /// Get the additive cross-step warm-start and gamma policy.
  ExactCoulombFbfCrossStepPolicyOptions getExactCoulombCrossStepPolicyOptions()
      const;

  // Documentation inherited.
  void setFromOtherConstraintSolver(const ConstraintSolver& other) override;

  /// Enable deterministic manifold-colored inner block Gauss-Seidel.
  ///
  /// This compatibility-branch opt-in is retained in implementation-side
  /// state and therefore does not change this class's object layout. When the
  /// contact-row operator can build a color with multiple independent
  /// manifolds, the colored ordering is used for any requested thread count;
  /// one thread provides the apples-to-apples colored baseline. Disabled is
  /// the default and preserves the legacy serial contact ordering.
  void setExactCoulombColoredBlockGaussSeidelEnabled(bool enabled);

  /// Whether deterministic manifold-colored inner BGS is requested.
  bool getExactCoulombColoredBlockGaussSeidelEnabled() const;

  /// Enable evidence-only participant affinity for colored inner BGS.
  ///
  /// Disabled by default. On Linux, when the calling process has an explicit
  /// allowed-CPU mask whose size exactly matches the requested participant
  /// count, each persistent-dispatch participant is temporarily pinned to the
  /// corresponding sorted CPU for that FBF attempt. Other platforms and
  /// broader masks retain normal scheduler behavior.
  void setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
      bool enabled);

  /// Whether evidence-only colored participant affinity is requested.
  bool getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled() const;

  /// Whether the last exact attempt tried the colored inner BGS path.
  ///
  /// This reports attempted work, not whether that attempt supplied the final
  /// accepted solution after any configured retry or polish path.
  bool getLastExactCoulombColoredBlockGaussSeidelUsed() const;

  /// Number of attempted colored frozen-cone solves in the last exact attempt.
  std::size_t getLastExactCoulombColoredBlockGaussSeidelSolves() const;

  /// Number of persistent-pool dispatches in the last exact attempt.
  ///
  /// A multi-threaded FBF attempt dispatches its worker indices once and keeps
  /// them resident across all colored frozen-cone solves. A one-thread
  /// colored baseline uses no pool dispatch. A dispatch can occur before the
  /// math layer accepts an iteration-zero warm start, in which case this count
  /// is positive while the colored-solve count is zero.
  std::size_t getLastExactCoulombColoredBlockGaussSeidelDispatches() const;

  /// Maximum worker participants in a colored dispatch in the last attempt.
  std::size_t getLastExactCoulombColoredBlockGaussSeidelParticipants() const;

  /// Number of contact manifolds in the selected colored schedule.
  std::size_t getLastExactCoulombColoredBlockGaussSeidelManifolds() const;

  /// Number of sequential colors in the selected colored schedule.
  std::size_t getLastExactCoulombColoredBlockGaussSeidelColors() const;

  /// Largest number of independent manifolds in one selected color.
  std::size_t getLastExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor()
      const;

  /// Logical CPU IDs observed at colored persistent-dispatch entry.
  ///
  /// These sorted IDs are runtime residency observations, not a claim that
  /// every worker owned useful manifold work or ran simultaneously.
  std::vector<int> getLastExactCoulombColoredBlockGaussSeidelLogicalCpuIds()
      const;

  /// Largest one-dispatch logical-CPU set in the last exact attempt.
  std::vector<int>
  getLastExactCoulombColoredBlockGaussSeidelMaxPhaseLogicalCpuIds() const;

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

  /// Number of contact-row Delassus products in the last exact attempt.
  std::size_t getLastExactCoulombContactRowDelassusProducts() const;

  /// Number of those products that actually used multiple participants.
  std::size_t getLastExactCoulombParallelContactRowDelassusProducts() const;

  /// Maximum participant count used by one contact-row Delassus product.
  ///
  /// Zero means the last exact attempt executed no contact-row product. One
  /// means contact-row products ran serially. Values above one prove that the
  /// exact solver dispatched work through multiple pool participants.
  std::size_t getLastExactCoulombMaxContactRowParticipants() const;

  /// Cumulative number of contact-row Delassus products.
  std::size_t getNumExactCoulombContactRowDelassusProducts() const;

  /// Cumulative number of products that used multiple participants.
  std::size_t getNumExactCoulombParallelContactRowDelassusProducts() const;

  /// Maximum contact-row participant count observed by this solver.
  std::size_t getMaxExactCoulombContactRowParticipants() const;

  /// Logical CPU IDs observed while executing the last exact attempt's
  /// parallel contact-row products.
  ///
  /// The sorted IDs report runtime residency observations, not simultaneous
  /// execution. The vector is empty when no parallel product ran or when the
  /// platform cannot observe the current logical CPU.
  std::vector<int> getLastExactCoulombContactRowLogicalCpuIds() const;

  /// Sorted union of logical CPU IDs observed by this solver so far.
  ///
  /// This lifetime-to-date diagnostic is empty when no supported runtime CPU
  /// observation has occurred. It adds no state to the solver object.
  std::vector<int> getExactCoulombContactRowLogicalCpuIds() const;

  /// Largest logical-CPU set observed within one parallel contact-row phase
  /// of the last exact attempt.
  ///
  /// Each candidate set comes from one scatter or gather phase and therefore
  /// contains at most that phase's participant count. Ties keep the
  /// lexicographically smallest sorted set. The IDs are runtime residency
  /// observations and do not assert simultaneous execution.
  std::vector<int> getLastExactCoulombMaxPhaseContactRowLogicalCpuIds() const;

  /// Best one-phase logical-CPU set observed by this solver so far.
  ///
  /// This is the lifetime-to-date counterpart of
  /// getLastExactCoulombMaxPhaseContactRowLogicalCpuIds().
  std::vector<int> getMaxExactCoulombPhaseContactRowLogicalCpuIds() const;

  /// Whether the last solve assembled a dense contact-row snapshot.
  bool getLastExactCoulombDenseContactRowSnapshotAssembled() const;

  /// Whether the last solve seeded from matrix-free Delassus diagonal blocks.
  bool getLastExactCoulombMatrixFreeDelassusSeedUsed() const;

  /// Whether the last exact-Coulomb solve used the warm-start cache.
  bool getLastExactCoulombWarmStartUsed() const;

  /// Whether the last solve started from matched cross-step gamma state.
  bool getLastExactCoulombPersistentStepSizeUsed() const;

  /// Persisted gamma requested before the current solve's fresh safe cap.
  double getLastExactCoulombPersistentStepSizeRequest() const;

  /// Number of contacts warm-started by manifold matching in the last solve.
  std::size_t getLastExactCoulombWarmStartMatchedContacts() const;

  /// Initial residual from the last exact-Coulomb FBF attempt.
  double getLastExactCoulombInitialResidual() const;

  /// Initial unscaled natural-map residual from the last exact attempt.
  double getLastExactCoulombInitialNaturalMapResidual() const;

  /// Final unscaled natural-map residual from the last exact attempt.
  double getLastExactCoulombNaturalMapResidual() const;

  /// Initial gamma before the opt-in warm-start residual cap.
  double getLastExactCoulombUncappedInitialStepSize() const;

  /// Whether the last exact attempt applied the warm-start residual gamma cap.
  bool getLastExactCoulombWarmStartStepSizeCapApplied() const;

  /// Number of exact attempts that applied the warm-start residual gamma cap.
  std::size_t getNumExactCoulombWarmStartStepSizeCaps() const;

  /// Number of accepted unconverged reactions omitted from the warm cache.
  std::size_t getNumExactCoulombUnconvergedCacheSkips() const;

  /// Number of groups solved through the exact-Coulomb path.
  std::size_t getNumExactCoulombSolves() const;

  /// Number of constrained groups that attempted the exact-Coulomb path.
  ///
  /// This cumulative diagnostic includes successful, capped, failed, and
  /// unsupported groups. It is intended for evidence/audit tooling that must
  /// not infer trajectory coverage from only the last group status.
  std::size_t getNumExactCoulombAttempts() const;

  /// Number of exact groups whose finite iterate was accepted at the cap.
  std::size_t getNumExactCoulombMaxIterationsAccepted() const;

  /// Worst finite final residual observed by any exact group attempt.
  ///
  /// Returns NaN until an attempt produces a finite final residual.
  double getWorstExactCoulombResidual() const;

  /// Number of groups routed through boxed-LCP fallback.
  std::size_t getNumBoxedLcpFallbacks() const;

  /// Number of exact-Coulomb group solve attempts that failed before fallback.
  std::size_t getNumExactCoulombFailures() const;

  /// Number of exact-Coulomb solves that reused a cached warm start.
  std::size_t getNumExactCoulombWarmStarts() const;

  /// Number of groups that attempted the projected-gradient retry path.
  std::size_t getNumExactCoulombProjectedGradientRetries() const;

  /// Number of failed persisted-gamma attempts retried with automatic gamma.
  std::size_t getNumExactCoulombPersistentStepSizeRetries() const;

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
  bool tryGetExactCoulombPersistentStepSize(
      const std::vector<ConstraintBase*>& constraints,
      double& stepSize,
      std::size_t& consecutiveNoRejectionSolves) const;
  void updateExactCoulombWarmStart(
      const std::vector<ConstraintBase*>& constraints,
      const Eigen::Ref<const Eigen::VectorXd>& reaction,
      double stepSize,
      std::size_t consecutiveNoRejectionSolves,
      bool updateReactionCache = true);
  void clearExactCoulombWarmStart();
  void clearExactCoulombPersistentStepSize();

  /// Invalidates only the non-contact adapter pointer warm-start cache,
  /// keeping body-local cross-step manifold records of other islands.
  void invalidateExactCoulombWarmStartPointerCache();

  /// Invalidates persisted gamma tokens belonging to a failed contact group.
  /// World-space reaction records and unrelated islands remain available.
  void invalidateExactCoulombPersistentStepSizeForGroup(
      const std::vector<ConstraintBase*>& constraints);
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
  bool mLastExactCoulombDenseContactRowSnapshotAssembled = false;
  bool mLastExactCoulombMatrixFreeDelassusSeedUsed = false;
  bool mLastExactCoulombWarmStartUsed = false;
  bool mLastExactCoulombPersistentStepSizeUsed = false;
  double mLastExactCoulombPersistentStepSizeRequest
      = std::numeric_limits<double>::quiet_NaN();
  std::size_t mNumExactCoulombSolves = 0u;
  std::size_t mNumExactCoulombAttempts = 0u;
  std::size_t mNumExactCoulombMaxIterationsAccepted = 0u;
  double mWorstExactCoulombResidual = std::numeric_limits<double>::quiet_NaN();
  std::size_t mNumBoxedLcpFallbacks = 0u;
  std::size_t mNumExactCoulombFailures = 0u;
  std::size_t mNumExactCoulombWarmStarts = 0u;
  std::size_t mNumExactCoulombProjectedGradientRetries = 0u;
  std::size_t mNumExactCoulombPersistentStepSizeRetries = 0u;
  std::size_t mNumExactCoulombDenseResidualPolishes = 0u;
  bool mHasExactCoulombWarmStart = false;
  std::vector<ConstraintBase*> mWarmStartConstraints;
  Eigen::VectorXd mWarmStartReaction;
  double mWarmStartStepSize = std::numeric_limits<double>::quiet_NaN();
  std::size_t mWarmStartConsecutiveNoRejectionSolves = 0u;

  /// Per-contact record for body-local manifold warm-start matching.
  struct ExactCoulombWarmStartContactRecord
  {
    const dynamics::BodyNode* bodyA = nullptr;
    const dynamics::BodyNode* bodyB = nullptr;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d worldNormal = Eigen::Vector3d::Zero();
    Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
    Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
    Eigen::Vector3d localNormalA = Eigen::Vector3d::Zero();
    Eigen::Vector3d localNormalB = Eigen::Vector3d::Zero();
    bool hasLocalFeatureA = false;
    bool hasLocalFeatureB = false;
    Eigen::Vector3d worldImpulse = Eigen::Vector3d::Zero();
    Eigen::Vector3d localImpulseB = Eigen::Vector3d::Zero();
    bool hasLocalImpulseB = false;
    bool reactionValid = true;
    std::size_t stepSizeGroupId = 0u;
    double stepSize = std::numeric_limits<double>::quiet_NaN();
    std::size_t consecutiveNoRejectionSolves = 0u;
    int age = 0;
  };
  std::vector<ExactCoulombWarmStartContactRecord> mWarmStartContactRecords;
  std::size_t mNextExactCoulombStepSizeGroupId = 1u;
  std::size_t mLastExactCoulombWarmStartMatchedContacts = 0u;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_EXACTCOULOMBFBFCONSTRAINTSOLVER_HPP_
