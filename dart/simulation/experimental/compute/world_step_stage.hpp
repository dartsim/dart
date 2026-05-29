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

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::compute {

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
  std::size_t staticRigidSurfaceCcdTriangleCount = 0;
  std::size_t staticRigidSurfaceCcdEdgeCount = 0;
  std::size_t staticRigidSurfaceCcdCandidateBuilds = 0;
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
  // Sparse-solve factorization accounting. The symbolic factorization
  // (fill-reducing ordering) is reused whenever the Hessian sparsity pattern is
  // unchanged across iterations/steps, so only the numeric factorization
  // repeats; symbolic < numeric indicates the analysis was amortized.
  std::size_t projectedNewtonSymbolicFactorizations = 0;
  std::size_t projectedNewtonNumericFactorizations = 0;
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

  void reset() noexcept
  {
    *this = {};
  }
};

/// Experimental contract for one stage in the World step pipeline.
class DART_EXPERIMENTAL_API WorldStepStage
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
};

/// Non-owning, ordered collection of stages for one experimental World step.
///
/// The pipeline is domain-neutral: rigid-body, articulated-body, deformable,
/// fluid, collision, constraint, control, sensor, and rendering-prep stages can
/// all use the same executor boundary and profiling surface.
class DART_EXPERIMENTAL_API WorldStepPipeline
{
public:
  WorldStepPipeline() = default;
  ~WorldStepPipeline() = default;

  WorldStepPipeline(const WorldStepPipeline&) = delete;
  WorldStepPipeline& operator=(const WorldStepPipeline&) = delete;
  WorldStepPipeline(WorldStepPipeline&&) noexcept = default;
  WorldStepPipeline& operator=(WorldStepPipeline&&) noexcept = default;

  WorldStepPipeline& addStage(WorldStepStage& stage);
  void clear() noexcept;

  [[nodiscard]] std::size_t getStageCount() const noexcept;
  [[nodiscard]] bool isEmpty() const noexcept;
  [[nodiscard]] WorldStepStage& getStage(std::size_t index) const;

  void execute(World& world, ComputeExecutor& executor);

private:
  std::vector<WorldStepStage*> m_stages;
};

/// Default kinematics/cache update stage for the experimental World.
class DART_EXPERIMENTAL_API KinematicsStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Per-entity unconstrained rigid-body integration stage for the experimental
/// World.
class DART_EXPERIMENTAL_API RigidBodyIntegrationStage final
  : public WorldStepStage
{
public:
  explicit RigidBodyIntegrationStage(std::size_t batchSize = 64);

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getBatchSize() const noexcept;

private:
  std::size_t m_batchSize;
};

/// Unconstrained rigid-body integration stage driven by the batched
/// structure-of-arrays path.
///
/// Instead of per-entity component access, this stage extracts a
/// `RigidBodyStateBatch` (plus the immutable `RigidBodyModelBatch`), runs the
/// scalar-generic SoA integrator, and applies the result back to the World. The
/// world-space dynamics are frame-independent, so the integration runs in flat
/// SoA order; frame-coupled rigid bodies only require parent-before-child
/// local-transform bookkeeping after state write-back. This is the experimental
/// seam through which the later SIMD and device batch paths drive a live World
/// step.
class DART_EXPERIMENTAL_API BatchedRigidBodyIntegrationStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Updates free rigid-body velocities from the assembled transient force buffer
/// (persistent applied force/torque plus gravity) without advancing positions.
/// Pairs with RigidBodyPositionStage so a contact-resolution stage can run at
/// the velocity level in between.
class DART_EXPERIMENTAL_API RigidBodyVelocityStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Advances free rigid-body poses from their current velocities and refreshes
/// frame caches. Run after velocity and contact resolution.
class DART_EXPERIMENTAL_API RigidBodyPositionStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Resolves contacts between free rigid bodies with sequential normal impulses
/// (frictionless, fully inelastic). Static bodies (non-positive mass) act as
/// immovable. This is the first contact-solver slice; friction, restitution
/// tuning, joints/links, and an LCP formulation are future work.
class DART_EXPERIMENTAL_API RigidBodyContactStage final : public WorldStepStage
{
public:
  explicit RigidBodyContactStage(std::size_t iterations = 8);

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getIterations() const noexcept;

private:
  std::size_t m_iterations;
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
  std::size_t activeDynamicsTerms = 0;
  std::size_t solverIterations = 0;
  std::size_t frictionIterations = 0;
  std::size_t acceptedSteps = 0;
  std::size_t lineSearchLimitedSteps = 0;
  double initialValue = 0.0;
  double finalValue = 0.0;
  double initialGradientNorm = 0.0;
  double finalGradientNorm = 0.0;
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
  bool converged = false;
  bool failed = false;
  bool resultApplied = false;
  bool nonConvergedResultSkipped = false;

  void reset() noexcept
  {
    *this = {};
  }
};

/// Opt-in rigid IPC world-step stage for free rigid bodies.
///
/// This stage is intentionally not part of the default `World::step()`
/// pipeline yet. It lets tests and custom pipelines exercise the internal IPC
/// barrier/Newton path on mesh-like rigid bodies while the default rigid solver
/// remains the established sequential-impulse stage.
class DART_EXPERIMENTAL_API RigidIpcContactStage final : public WorldStepStage
{
public:
  explicit RigidIpcContactStage(std::size_t maxIterations = 8);

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getMaxIterations() const noexcept;
  [[nodiscard]] const RigidIpcSolverStats& getLastStats() const noexcept;

private:
  std::size_t m_maxIterations;
  RigidIpcSolverStats m_lastStats;
};

/// Default deformable-body dynamics stage.
///
/// This stage advances point-mass deformable bodies through the default
/// experimental World pipeline. The public stage name is intentionally
/// algorithm-neutral; contact/barrier details remain implementation internals.
class DART_EXPERIMENTAL_API DeformableDynamicsStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] const DeformableSolverStats& getLastStats() const noexcept;

private:
  DeformableSolverStats m_lastStats;
};

} // namespace dart::simulation::experimental::compute
