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

#pragma once

#include <dart/simulation/compute/compute_stage_metadata.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <dart/common/stl_allocator.hpp>

#include <array>
#include <memory>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::common {
class MemoryAllocator;
class MemoryManager;
} // namespace dart::common

namespace dart::simulation::compute {

class WorldKinematicsGraph;

/// Summary of the most recent deformable solver execution.
struct DeformableSolverStats
{
  std::size_t bodyCount = 0;
  std::size_t nodeCount = 0;
  std::size_t edgeCount = 0;
  std::size_t activeDirichletNodeCount = 0;
  std::size_t activeNeumannNodeCount = 0;
  std::size_t objectiveEvaluations = 0;
  std::size_t solverIterations = 0;
  std::size_t lineSearchTrials = 0;
  std::size_t rejectedLineSearchCandidates = 0;
  std::size_t acceptedLineSearchSteps = 0;
  std::size_t initialProjectionCount = 0;
  std::size_t staticGroundBarrierCount = 0;
  std::size_t staticGroundBarrierCcdNodeChecks = 0;
  std::size_t staticGroundBarrierCcdSampleChecks = 0;
  std::size_t staticGroundBarrierCcdHits = 0;
  std::size_t staticGroundBarrierCcdLimitedSteps = 0;
  std::size_t staticGroundBarrierCcdZeroStepCount = 0;
  std::size_t surfaceContactCandidateBuilds = 0;
  std::size_t surfaceContactCandidatePairCapacity = 0;
  std::size_t surfaceContactCandidateRejectedPairs = 0;
  std::size_t surfaceContactPointTriangleCandidates = 0;
  std::size_t surfaceContactEdgeEdgeCandidates = 0;
  std::size_t surfaceContactCcdPointTriangleChecks = 0;
  std::size_t surfaceContactCcdEdgeEdgeChecks = 0;
  std::size_t surfaceContactCcdHits = 0;
  std::size_t surfaceContactCcdMisses = 0;
  std::size_t surfaceContactCcdIndeterminateCount = 0;
  std::size_t surfaceContactCcdLimitedSteps = 0;
  std::size_t surfaceContactCcdZeroStepCount = 0;
  std::size_t interBodySurfaceContactCandidateBuilds = 0;
  std::size_t interBodySurfaceContactCandidatePairCapacity = 0;
  std::size_t interBodySurfaceContactCandidateRejectedPairs = 0;
  std::size_t interBodySurfaceContactPointTriangleCandidates = 0;
  std::size_t interBodySurfaceContactEdgeEdgeCandidates = 0;
  std::size_t interBodySurfaceContactCcdPointTriangleChecks = 0;
  std::size_t interBodySurfaceContactCcdEdgeEdgeChecks = 0;
  std::size_t interBodySurfaceContactCcdHits = 0;
  std::size_t interBodySurfaceContactCcdMisses = 0;
  std::size_t interBodySurfaceContactCcdIndeterminateCount = 0;
  std::size_t interBodySurfaceContactCcdLimitedSteps = 0;
  std::size_t interBodySurfaceContactCcdZeroStepCount = 0;
  std::size_t staticRigidSurfaceCcdSnapshotBuilds = 0;
  std::size_t staticRigidSurfaceCcdBoxCount = 0;
  std::size_t staticRigidSurfaceCcdSphereCount = 0;
  std::size_t staticRigidSurfaceCcdTriangleCount = 0;
  std::size_t staticRigidSurfaceCcdEdgeCount = 0;
  std::size_t staticRigidSurfaceCcdCandidateBuilds = 0;
  std::size_t staticRigidSurfaceCcdCandidatePairCapacity = 0;
  std::size_t staticRigidSurfaceCcdCandidateRejectedPairs = 0;
  std::size_t staticRigidSurfaceCcdPointTriangleCandidates = 0;
  std::size_t staticRigidSurfaceCcdEdgeEdgeCandidates = 0;
  std::size_t staticRigidSurfaceCcdPointTriangleChecks = 0;
  std::size_t staticRigidSurfaceCcdEdgeEdgeChecks = 0;
  std::size_t staticRigidSurfaceCcdHits = 0;
  std::size_t staticRigidSurfaceCcdMisses = 0;
  std::size_t staticRigidSurfaceCcdIndeterminateCount = 0;
  std::size_t staticRigidSurfaceCcdLimitedSteps = 0;
  std::size_t staticRigidSurfaceCcdZeroStepCount = 0;
  std::size_t movingRigidSurfaceCcdSnapshotBuilds = 0;
  std::size_t movingRigidSurfaceCcdBoxCount = 0;
  std::size_t movingRigidSurfaceCcdSampleCount = 0;
  std::size_t movingRigidSurfaceCcdInflatedBoxCount = 0;
  std::size_t movingRigidSurfaceCcdTriangleCount = 0;
  std::size_t movingRigidSurfaceCcdEdgeCount = 0;
  std::size_t movingRigidSurfaceCcdCandidateBuilds = 0;
  std::size_t movingRigidSurfaceCcdCandidatePairCapacity = 0;
  std::size_t movingRigidSurfaceCcdCandidateRejectedPairs = 0;
  std::size_t movingRigidSurfaceCcdPointTriangleCandidates = 0;
  std::size_t movingRigidSurfaceCcdEdgeEdgeCandidates = 0;
  std::size_t movingRigidSurfaceCcdPointTriangleChecks = 0;
  std::size_t movingRigidSurfaceCcdEdgeEdgeChecks = 0;
  std::size_t movingRigidSurfaceCcdHits = 0;
  std::size_t movingRigidSurfaceCcdMisses = 0;
  std::size_t movingRigidSurfaceCcdIndeterminateCount = 0;
  std::size_t movingRigidSurfaceCcdLimitedSteps = 0;
  std::size_t movingRigidSurfaceCcdZeroStepCount = 0;
  std::size_t selfContactBarrierCandidateBuilds = 0;
  // Cumulative count of active barrier contacts summed over the outer solver
  // iterations of a step (one increment per active candidate per gradient
  // evaluation), matching the cumulative semantics of solverIterations; it is
  // not the active-set size of a single iteration.
  std::size_t selfContactBarrierActiveContacts = 0;
  std::size_t projectedNewtonSteps = 0;
  std::size_t projectedNewtonFallbacks = 0;
  // Sparse-direct factorization accounting retained for diagnostics
  // compatibility. Built-in DART 7 World steps route systems above the retained
  // dense-direct cap to iterative CG, so these counters stay zero for the
  // allocation-safe default path.
  std::size_t projectedNewtonSymbolicFactorizations = 0;
  std::size_t projectedNewtonNumericFactorizations = 0;
  // Maximum compressed sparse Hessian footprint assembled for a
  // projected-Newton linear solve in this step. These are matrix-storage
  // diagnostics (not full process peak memory): nonzeros counts Eigen's
  // compressed sparse entries, and storage bytes estimates the value, inner
  // index, and outer pointer arrays for that matrix. Small systems solved with
  // retained dense direct scratch and matrix-free solves report zero because
  // they do not assemble an Eigen sparse matrix.
  std::size_t projectedNewtonHessianNonZeros = 0;
  std::size_t projectedNewtonHessianStorageBytes = 0;
  // Iterative (conjugate-gradient) linear solves. Counts Newton iterations that
  // took the matrix-light CG path instead of direct factorization -- either
  // because the system exceeds the retained dense-direct cap or because the
  // body opted in via DeformableMaterial.useIterativeLinearSolver. CG never
  // factorizes, so a nonzero value here with zero sparse numeric factorizations
  // means the whole solve ran iteratively.
  std::size_t projectedNewtonIterativeSolves = 0;
  // Iterative solves that used the matrix-free Hessian-vector product path
  // instead of an assembled sparse Hessian. This is a subset of
  // projectedNewtonIterativeSolves.
  std::size_t projectedNewtonMatrixFreeSolves = 0;
  // Total conjugate-gradient iterations consumed by successful iterative
  // projected-Newton linear solves. This is the solve-effort axis for the M7
  // scaling benchmarks; zero when every Newton step used the direct solver.
  std::size_t projectedNewtonIterativeIterations = 0;
  // Maximum Eigen-reported relative residual estimate across successful
  // iterative projected-Newton linear solves in this step.
  double projectedNewtonIterativeMaxError = 0.0;
  // Convergence diagnostic: the largest L2 gradient norm at solve termination
  // across the step's deformable bodies (the projected-Newton residual). Near
  // the gradient tolerance means the solve converged; a large value means a
  // body hit the iteration cap or stalled. Feeds the paper's benchmark
  // statistics (Fig. 23 / Table 1).
  double finalGradientResidualNorm = 0.0;
  // Converged-ness diagnostic: the largest last-accepted step infinity norm
  // (per-node position change) across the step's deformable bodies. Unlike the
  // gradient residual, this shrinks toward zero at equilibrium even for stiff
  // barrier contacts (whose near-singular Hessian keeps the gradient norm
  // large), so a small value here with a large finalGradientResidualNorm
  // indicates a feasible settled configuration rather than a failed solve.
  double finalStepInfinityNorm = 0.0;
  // Friction diagnostics at the converged iterate, summed over the step's
  // bodies and over both static-ground and self-contact friction. The
  // dissipation is the IPC Coulomb work mu * lambda * f1(y) * y (force times
  // tangential slip, smoothly ramped to zero at rest by the f0/f1 mollifier);
  // the active count is the number of friction contacts carrying a nonzero
  // lagged normal force. Both are zero when friction is disabled
  // (frictionCoefficient == 0). Feed the paper's friction benchmark statistics.
  double frictionDissipation = 0.0;
  std::size_t activeFrictionContacts = 0;
  std::size_t vbdBodyCount = 0;
  std::size_t vbdSweeps = 0;
  std::size_t vbdVertexUpdates = 0;
  std::size_t vbdAvbdContactNormalRows = 0;
  std::size_t vbdAvbdSelfContactNormalRows = 0;
  std::size_t vbdAvbdFrictionTangentRows = 0;
  std::size_t vbdAvbdAttachmentRows = 0;
  std::size_t vbdAvbdFiniteStiffnessRows = 0;
  std::size_t vbdAvbdFiniteStiffnessTetRows = 0;
  double vbdResidualNormSquared = 0.0;
  // Contact closest-approach diagnostic at the converged iterate, folded across
  // the step's deformable bodies. minActiveContactDistance is the smallest
  // point-triangle / edge-edge distance among the active self-contact barrier
  // set (candidates within the activation band d_hat) at solve termination --
  // the IPC intersection-free "minimum distance" statistic (Fig. 23 / Table 1).
  // It is meaningful only when convergedActiveContactCount > 0 (otherwise 0).
  // convergedActiveContactCount is the size of that active set at termination:
  // a single-iteration snapshot, unlike the cumulative
  // selfContactBarrierActiveContacts (which sums the active set over every
  // outer iteration). Both are zero for bodies without self-contact (point-mass
  // bodies, or surfaces that stay outside d_hat). Read once after the outer
  // loop, not on the line-search hot path.
  double minActiveContactDistance = 0.0;
  std::size_t convergedActiveContactCount = 0;

  void reset() noexcept
  {
    *this = {};
  }
};

/// Experimental contract for one stage in the World step pipeline.
class DART_SIMULATION_API WorldStepStage
{
public:
  virtual ~WorldStepStage() = default;

  WorldStepStage() = default;
  WorldStepStage(const WorldStepStage&) = delete;
  WorldStepStage& operator=(const WorldStepStage&) = delete;
  WorldStepStage(WorldStepStage&&) noexcept = default;
  WorldStepStage& operator=(WorldStepStage&&) noexcept = default;

  [[nodiscard]] virtual std::string_view getName() const noexcept = 0;
  [[nodiscard]] virtual ComputeStageMetadata getMetadata() const noexcept;
  virtual void execute(World& world, ComputeExecutor& executor) = 0;

  /// Optional pre-step preparation hook. The default is a no-op; a stage that
  /// caches per-step state overrides this. The built-in schedule calls
  /// `prepare()` on every scheduled stage, so a stateless stage needs no entry
  /// in any parallel "needs preparation" table.
  virtual void prepare(World& world);
};

/// Non-owning, ordered collection of stages for one DART 7 World step.
///
/// The pipeline is domain-neutral: rigid-body, articulated-body, deformable,
/// fluid, collision, constraint, control, sensor, and rendering-prep stages can
/// all use the same executor boundary and profiling surface.
class DART_SIMULATION_API WorldStepPipeline
{
public:
  WorldStepPipeline();
  explicit WorldStepPipeline(common::MemoryAllocator& allocator);
  ~WorldStepPipeline() = default;

  WorldStepPipeline(const WorldStepPipeline&) = delete;
  WorldStepPipeline& operator=(const WorldStepPipeline&) = delete;
  WorldStepPipeline(WorldStepPipeline&&) noexcept = default;
  WorldStepPipeline& operator=(WorldStepPipeline&&) noexcept = default;

  /// Number of stage pointers stored inline before custom pipelines spill over.
  static constexpr std::size_t kInlineStageCount = 8;

  WorldStepPipeline& addStage(WorldStepStage& stage);
  void clear() noexcept;

  [[nodiscard]] std::size_t getStageCount() const noexcept;
  [[nodiscard]] bool isEmpty() const noexcept;
  [[nodiscard]] WorldStepStage& getStage(std::size_t index) const;

  void execute(World& world, ComputeExecutor& executor);
  [[nodiscard]] WorldStepProfile executeProfiled(
      World& world, ComputeExecutor& executor);
  void executeProfiled(
      World& world, ComputeExecutor& executor, WorldStepProfile& profile);

private:
  std::array<WorldStepStage*, kInlineStageCount> m_stages{};
  std::vector<WorldStepStage*, common::StlAllocator<WorldStepStage*>>
      m_overflowStages;
  std::size_t m_stageCount = 0;
};

/// Default kinematics/cache update stage for the DART 7 World.
class DART_SIMULATION_API KinematicsStage final : public WorldStepStage
{
public:
  KinematicsStage();
  explicit KinematicsStage(common::MemoryManager* memoryManager);
  ~KinematicsStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void prepare(World& world) override;
  void execute(World& world, ComputeExecutor& executor) override;

private:
  struct CachedGraphDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(WorldKinematicsGraph* graph) const noexcept;
  };

  World* m_cachedWorld = nullptr;
  common::MemoryManager* m_memoryManager = nullptr;
  std::unique_ptr<WorldKinematicsGraph, CachedGraphDeleter> m_cachedGraph;
};

/// Per-entity unconstrained rigid-body integration stage for the experimental
/// World.
class DART_SIMULATION_API RigidBodyIntegrationStage final
  : public WorldStepStage
{
public:
  explicit RigidBodyIntegrationStage(std::size_t batchSize = 64);
  explicit RigidBodyIntegrationStage(common::MemoryManager* memoryManager);
  RigidBodyIntegrationStage(
      std::size_t batchSize, common::MemoryManager* memoryManager);
  ~RigidBodyIntegrationStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getBatchSize() const noexcept;

private:
  struct Scratch;
  struct ScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(Scratch* scratch) const noexcept;
  };

  using ScratchPtr = std::unique_ptr<Scratch, ScratchDeleter>;

  static Scratch* createScratch(common::MemoryManager* memoryManager);

  std::size_t m_batchSize;
  common::MemoryManager* m_memoryManager = nullptr;
  ScratchPtr m_scratch;
};

/// Unconstrained rigid-body integration stage driven by the batched
/// structure-of-arrays path.
///
/// Instead of per-entity component access, this stage extracts a
/// `RigidBodyStateBatch` (plus the immutable `RigidBodyModelBatch`), runs the
/// scalar-generic SoA integrator, and applies the result back to the World. The
/// world-space dynamics are frame-independent, so the integration runs in flat
/// SoA order; frame-coupled rigid bodies only require parent-before-child
/// local-transform bookkeeping after state write-back. This is the
/// canonical-direction seed for a live World stage; later SIMD and device batch
/// paths should keep the same Model/State separation without exposing backend
/// residency details.
class DART_SIMULATION_API BatchedRigidBodyIntegrationStage final
  : public WorldStepStage
{
public:
  BatchedRigidBodyIntegrationStage();
  explicit BatchedRigidBodyIntegrationStage(
      common::MemoryManager* memoryManager);
  ~BatchedRigidBodyIntegrationStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

private:
  struct Scratch;
  struct ScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(Scratch* scratch) const noexcept;
  };

  using ScratchPtr = std::unique_ptr<Scratch, ScratchDeleter>;

  static Scratch* createScratch(common::MemoryManager* memoryManager);

  common::MemoryManager* m_memoryManager = nullptr;
  ScratchPtr m_scratch;
};

/// Updates free rigid-body velocities from the assembled transient force buffer
/// (persistent applied force/torque plus gravity) without advancing positions.
/// Pairs with RigidBodyPositionStage so a contact-resolution stage can run at
/// the velocity level in between.
class DART_SIMULATION_API RigidBodyVelocityStage final : public WorldStepStage
{
public:
  RigidBodyVelocityStage();
  explicit RigidBodyVelocityStage(common::MemoryManager* memoryManager);
  ~RigidBodyVelocityStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void prepare(World& world) override;
  void execute(World& world, ComputeExecutor& executor) override;

private:
  struct Scratch;
  struct ScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(Scratch* scratch) const noexcept;
  };

  using ScratchPtr = std::unique_ptr<Scratch, ScratchDeleter>;

  static Scratch* createScratch(common::MemoryManager* memoryManager);

  common::MemoryManager* m_memoryManager = nullptr;
  ScratchPtr m_scratch;
};

/// Advances free rigid-body poses from their current velocities and refreshes
/// frame caches. Run after velocity and contact resolution.
class DART_SIMULATION_API RigidBodyPositionStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Resolves contacts between free rigid bodies. Static bodies (non-positive
/// mass) act as immovable.
///
/// Two public solver paths are available, selected per-World via
/// `WorldOptions::contactSolverMethod`:
///   - `SequentialImpulse` (default): the long-standing sequential normal +
///     friction impulse solve with positional correction.
///   - `BoxedLcp`: an opt-in boxed-LCP normal/friction solve via the pivoting
///     Dantzig solver. Articulated-link contacts are handled by the unified
///     constraint/contact path, not by this free-rigid stage.
///
/// Internal PLAN-104 AVBD work can also opt specific rigid bodies into the
/// private `RigidAvbdContactConfig` row projection without exposing AVBD row
/// storage or solver registries through the facade.
class DART_SIMULATION_API RigidBodyContactStage final : public WorldStepStage
{
public:
  explicit RigidBodyContactStage(std::size_t iterations = 8);
  RigidBodyContactStage(
      std::size_t iterations, common::MemoryManager* memoryManager);
  ~RigidBodyContactStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void prepare(World& world) override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getIterations() const noexcept;

private:
  struct AvbdScratch;
  struct ContactScratch;
  struct AvbdScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(AvbdScratch* scratch) const noexcept;
  };
  struct ContactScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(ContactScratch* scratch) const noexcept;
  };

  using AvbdScratchPtr = std::unique_ptr<AvbdScratch, AvbdScratchDeleter>;
  using ContactScratchPtr
      = std::unique_ptr<ContactScratch, ContactScratchDeleter>;

  static AvbdScratch* createAvbdScratch(common::MemoryManager* memoryManager);
  static ContactScratch* createContactScratch(
      common::MemoryManager* memoryManager);

  std::size_t m_iterations;
  common::MemoryManager* m_memoryManager = nullptr;
  AvbdScratchPtr m_avbdScratch;
  ContactScratchPtr m_contactScratch;
};

/// Terminal state of the most recent opt-in rigid IPC solver execution.
enum class RigidIpcSolveStatus
{
  NoDofs,
  Converged,
  MaxIterations,
  LineSearchBlocked,
  FactorizationFailed,
};

/// Time-integration rule used by the opt-in rigid IPC stage.
enum class RigidIpcTimeIntegration
{
  SemiImplicit,
  Bdf2,
};

/// Summary of the most recent opt-in rigid IPC solver execution.
struct RigidIpcSolverStats
{
  RigidIpcSolveStatus status = RigidIpcSolveStatus::NoDofs;
  std::size_t bodyCount = 0;
  std::size_t dynamicBodyCount = 0;
  std::size_t surfaceCount = 0;
  std::size_t skippedUnsupportedShapeCount = 0;
  std::size_t activeConstraints = 0;
  std::size_t activeFrictionConstraints = 0;
  std::size_t activeArticulationConstraints = 0;
  std::size_t activeDynamicsTerms = 0;
  std::size_t bdf2RestartedDynamicsTerms = 0;
  std::size_t bdf2SecondOrderDynamicsTerms = 0;
  std::size_t mixedDomainSurfaceCount = 0;
  std::size_t mixedDomainDeformableSurfaceCount = 0;
  std::size_t mixedDomainCandidateCount = 0;
  std::size_t mixedDomainActiveBarrierCount = 0;
  double mixedDomainBarrierValue = 0.0;
  std::size_t solverIterations = 0;
  std::size_t frictionIterations = 0;
  std::size_t acceptedSteps = 0;
  std::size_t lineSearchLimitedSteps = 0;
  double initialValue = 0.0;
  double finalValue = 0.0;
  double initialGradientNorm = 0.0;
  double finalGradientNorm = 0.0;
  double initialEqualityResidualNorm = 0.0;
  double finalEqualityResidualNorm = 0.0;
  double finalMomentumBalance = 0.0;
  double lastStepNorm = 0.0;
  double barrierStiffness = 0.0;
  std::size_t barrierStiffnessIncreases = 0;
  double lastLineSearchStepBound = 1.0;
  bool lastLineSearchIndeterminate = false;
  std::size_t lineSearchPointPointChecks = 0;
  std::size_t lineSearchPointEdgeChecks = 0;
  std::size_t lineSearchEdgeEdgeChecks = 0;
  std::size_t lineSearchPointTriangleChecks = 0;
  std::size_t lineSearchHits = 0;
  std::size_t lineSearchMisses = 0;
  std::size_t lineSearchIndeterminateCount = 0;
  std::size_t lineSearchZeroStepCount = 0;
  std::size_t sufficientDecreaseChecks = 0;
  std::size_t sufficientDecreaseBacktracks = 0;
  bool converged = false;
  bool failed = false;
  bool resultApplied = false;
  bool nonConvergedResultSkipped = false;

  void reset() noexcept
  {
    *this = {};
  }
};

/// Configuration for the opt-in rigid IPC world-step stage.
struct RigidIpcContactStageOptions
{
  /// Maximum projected-Newton iterations per stage execution. Zero is valid
  /// for tests that need to exercise the non-converged application policy.
  std::size_t maxIterations = 8;

  /// IPC barrier activation distance (`dHat`) in world units. Non-finite or
  /// non-positive values fall back to the default distance.
  double activationDistance = 1e-2;

  /// Number of lagged friction outer passes. Zero disables lagged friction.
  std::size_t frictionIterations = 1;

  /// Static-friction speed bound (`epsv`) in world units per second. The stage
  /// converts this to the solver's per-step displacement threshold using the
  /// world timestep. Zero disables lagged friction.
  double staticFrictionSpeedBound = 1e-3;

  /// Absolute momentum-balance tolerance for stopping lagged friction passes.
  /// Zero requires the configured number of friction passes.
  double frictionConvergenceTolerance = 0.0;

  /// Inertial target rule for dynamic rigid bodies. The default preserves the
  /// existing semi-implicit Euler target; BDF-2 is opt-in and restarts as a
  /// first-order step until each body has one accepted previous pose.
  RigidIpcTimeIntegration timeIntegration
      = RigidIpcTimeIntegration::SemiImplicit;
};

/// Opt-in rigid IPC world-step stage for free rigid bodies.
///
/// The default `World::step()` path remains the established sequential-impulse
/// rigid solver. Selecting `RigidBodySolver::Ipc` routes the built-in schedule
/// through this stage for supported free rigid bodies, and custom pipelines may
/// still instantiate the stage directly for focused tests or experiments.
class DART_SIMULATION_API RigidIpcContactStage final : public WorldStepStage
{
public:
  explicit RigidIpcContactStage(std::size_t maxIterations = 8);
  RigidIpcContactStage(
      std::size_t maxIterations, common::MemoryManager* memoryManager);
  explicit RigidIpcContactStage(RigidIpcContactStageOptions options);
  RigidIpcContactStage(
      RigidIpcContactStageOptions options,
      common::MemoryManager* memoryManager);
  ~RigidIpcContactStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void prepare(World& world) override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getMaxIterations() const noexcept;
  [[nodiscard]] double getActivationDistance() const noexcept;
  [[nodiscard]] std::size_t getFrictionIterations() const noexcept;
  [[nodiscard]] double getStaticFrictionSpeedBound() const noexcept;
  [[nodiscard]] double getFrictionConvergenceTolerance() const noexcept;
  [[nodiscard]] RigidIpcContactStageOptions getOptions() const noexcept;
  [[nodiscard]] const RigidIpcSolverStats& getLastStats() const noexcept;

private:
  struct Scratch;
  struct ScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(Scratch* scratch) const noexcept;
  };

  using ScratchPtr = std::unique_ptr<Scratch, ScratchDeleter>;

  RigidIpcContactStageOptions m_options;
  RigidIpcSolverStats m_lastStats;
  common::MemoryManager* m_memoryManager = nullptr;
  ScratchPtr m_scratch;
};

/// Default deformable-body dynamics stage.
///
/// This stage advances point-mass deformable bodies through the default
/// DART 7 World pipeline. The public stage name is intentionally
/// algorithm-neutral; contact/barrier details remain implementation internals.
class DART_SIMULATION_API DeformableDynamicsStage final : public WorldStepStage
{
public:
  DeformableDynamicsStage();
  explicit DeformableDynamicsStage(common::MemoryManager* memoryManager);
  ~DeformableDynamicsStage() override;

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  /// Pre-reserve per-body deformable solver scratch before baked steps.
  void prepare(World& world) override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] const DeformableSolverStats& getLastStats() const noexcept;

private:
  struct Scratch;
  struct ScratchDeleter
  {
    common::MemoryManager* memoryManager = nullptr;
    void operator()(Scratch* scratch) const noexcept;
  };

  DeformableSolverStats m_lastStats;
  common::MemoryManager* m_memoryManager = nullptr;
  std::unique_ptr<Scratch, ScratchDeleter> m_scratch;
};

} // namespace dart::simulation::compute
