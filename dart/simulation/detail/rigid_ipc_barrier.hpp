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
 *     copyright notice, this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/line_search.hpp>
#include <dart/simulation/detail/rigid_ipc_ccd.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <algorithm>
#include <array>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace dart::simulation::detail {

struct RigidIpcBarrierOptions
{
  double squaredActivationDistance = 0.0;
  double stiffness = 1.0;
  bool projectReducedHessianToPsd = true;
};

using RigidIpcPrimitiveBarrierResult = newton_barrier::PrimitiveBarrierResult;
using RigidIpcVector6d = Eigen::Matrix<double, 6, 1>;
using RigidIpcVector12d = Eigen::Matrix<double, 12, 1>;
using RigidIpcMatrix12d = Eigen::Matrix<double, 12, 12>;

struct RigidIpcReducedBarrierResult
{
  double value = 0.0;
  RigidIpcVector12d gradient = RigidIpcVector12d::Zero();
  RigidIpcMatrix12d hessian = RigidIpcMatrix12d::Zero();
  RigidIpcPrimitiveBarrierResult primitive;
  bool active = false;
};

struct RigidIpcFrictionOptions
{
  double coefficient = 0.0;
  double laggedNormalForce = 0.0;
  double staticFrictionDisplacement = 0.0;
  bool projectReducedHessianToPsd = true;
};

struct RigidIpcFrictionPotentialResult
{
  double value = 0.0;
  double work = 0.0;
  RigidIpcVector12d gradient = RigidIpcVector12d::Zero();
  RigidIpcMatrix12d hessian = RigidIpcMatrix12d::Zero();
  Eigen::Vector2d tangentialDisplacement = Eigen::Vector2d::Zero();
  double tangentialDisplacementNorm = 0.0;
  double weight = 0.0;
  bool active = false;
  bool dynamicBranch = false;
};

struct RigidIpcReducedFrictionResult
{
  double value = 0.0;
  RigidIpcVector12d gradient = RigidIpcVector12d::Zero();
  RigidIpcMatrix12d hessian = RigidIpcMatrix12d::Zero();
  RigidIpcFrictionPotentialResult potential;
  bool active = false;
};

enum class RigidIpcBarrierPrimitive
{
  VertexVertex,
  EdgeVertex,
  EdgeEdge,
  FaceVertex,
};

enum class RigidIpcArticulationConstraintType
{
  PointConnection,
  HingeAxis,
};

struct RigidIpcBarrierSurface
{
  using VertexAllocator = dart::common::StlAllocator<Eigen::Vector3d>;
  using TriangleAllocator = dart::common::StlAllocator<Eigen::Vector3i>;
  using VertexVector = std::vector<Eigen::Vector3d, VertexAllocator>;
  using TriangleVector = std::vector<Eigen::Vector3i, TriangleAllocator>;

  RigidIpcBarrierSurface() = default;

  explicit RigidIpcBarrierSurface(dart::common::MemoryAllocator& allocator)
    : vertices(VertexAllocator{allocator}),
      triangles(TriangleAllocator{allocator})
  {
  }

  RigidIpcBarrierSurface(const RigidIpcBarrierSurface&) = default;
  RigidIpcBarrierSurface(RigidIpcBarrierSurface&&) noexcept = default;
  RigidIpcBarrierSurface& operator=(RigidIpcBarrierSurface&&) noexcept
      = default;

  RigidIpcBarrierSurface& operator=(const RigidIpcBarrierSurface& other)
  {
    if (this == &other) {
      return *this;
    }

    body = other.body;
    pose = other.pose;
    vertices.assign(other.vertices.begin(), other.vertices.end());
    triangles.assign(other.triangles.begin(), other.triangles.end());
    frictionCoefficient = other.frictionCoefficient;
    dynamic = other.dynamic;
    kinematic = other.kinematic;
    kinematicStartPose = other.kinematicStartPose;
    return *this;
  }

  std::size_t body = 0;
  RigidIpcPose pose;
  VertexVector vertices;
  TriangleVector triangles;
  double frictionCoefficient = 1.0;
  bool dynamic = true;
  // Kinematic (prescribed-motion) obstacle: holds no solver DOFs (dynamic ==
  // false), but its pose advances from `kinematicStartPose` to `pose` over the
  // step. The barrier and dynamics see it at the end pose (`pose`); friction
  // and the conservative CCD line search use the start->end motion so it both
  // drags contacting dynamic bodies and stays swept-collision safe. Ignored
  // unless `kinematic` is set, so non-kinematic scenes are bit-for-bit
  // unchanged.
  bool kinematic = false;
  RigidIpcPose kinematicStartPose;
};

struct RigidIpcBarrierConstraint
{
  RigidIpcBarrierPrimitive primitive = RigidIpcBarrierPrimitive::VertexVertex;
  std::size_t bodyA = 0;
  std::size_t bodyB = 0;
  std::array<std::size_t, 4> vertices{
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max()};
  RigidIpcReducedBarrierResult reduced;
};

struct RigidIpcFrictionConstraint
{
  RigidIpcBarrierPrimitive primitive = RigidIpcBarrierPrimitive::VertexVertex;
  std::size_t bodyA = 0;
  std::size_t bodyB = 0;
  std::array<std::size_t, 4> vertices{
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max()};
  RigidIpcReducedFrictionResult reduced;
  double coefficient = 0.0;
  double laggedNormalForce = 0.0;
};

struct RigidIpcArticulationConstraintInput
{
  bool active = true;
  RigidIpcArticulationConstraintType type
      = RigidIpcArticulationConstraintType::PointConnection;
  std::size_t bodyA = 0;
  std::size_t bodyB = 0;
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAxisA = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d localAxisB = Eigen::Vector3d::UnitZ();
};

struct RigidIpcArticulationConstraint
{
  RigidIpcArticulationConstraintType type
      = RigidIpcArticulationConstraintType::PointConnection;
  std::size_t bodyA = 0;
  std::size_t bodyB = 0;
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
};

struct RigidIpcBarrierAssembly
{
  static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();

  using BodyDofOffsetAllocator = dart::common::StlAllocator<std::size_t>;
  using ConstraintAllocator
      = dart::common::StlAllocator<RigidIpcBarrierConstraint>;
  using FrictionConstraintAllocator
      = dart::common::StlAllocator<RigidIpcFrictionConstraint>;
  using ArticulationConstraintAllocator
      = dart::common::StlAllocator<RigidIpcArticulationConstraint>;
  using BodyDofOffsetVector = std::vector<std::size_t, BodyDofOffsetAllocator>;
  using ConstraintVector
      = std::vector<RigidIpcBarrierConstraint, ConstraintAllocator>;
  using FrictionConstraintVector
      = std::vector<RigidIpcFrictionConstraint, FrictionConstraintAllocator>;
  using ArticulationConstraintVector = std::
      vector<RigidIpcArticulationConstraint, ArticulationConstraintAllocator>;

  RigidIpcBarrierAssembly() = default;

  explicit RigidIpcBarrierAssembly(dart::common::MemoryAllocator& allocator)
    : bodyDofOffsets(BodyDofOffsetAllocator{allocator}),
      activeConstraints(ConstraintAllocator{allocator}),
      activeFrictionConstraints(FrictionConstraintAllocator{allocator}),
      activeArticulationConstraints(ArticulationConstraintAllocator{allocator})
  {
  }

  RigidIpcBarrierAssembly(const RigidIpcBarrierAssembly&) = default;
  RigidIpcBarrierAssembly(RigidIpcBarrierAssembly&&) noexcept = default;

  RigidIpcBarrierAssembly& operator=(const RigidIpcBarrierAssembly& other)
  {
    if (this == &other) {
      return *this;
    }

    value = other.value;
    gradient = other.gradient;
    hessian = other.hessian;
    equalityResidual = other.equalityResidual;
    equalityJacobian = other.equalityJacobian;
    bodyDofOffsets.assign(
        other.bodyDofOffsets.begin(), other.bodyDofOffsets.end());
    activeConstraints.assign(
        other.activeConstraints.begin(), other.activeConstraints.end());
    activeFrictionConstraints.assign(
        other.activeFrictionConstraints.begin(),
        other.activeFrictionConstraints.end());
    activeArticulationConstraints.assign(
        other.activeArticulationConstraints.begin(),
        other.activeArticulationConstraints.end());
    activeDynamicsTerms = other.activeDynamicsTerms;
    return *this;
  }

  RigidIpcBarrierAssembly& operator=(RigidIpcBarrierAssembly&& other)
  {
    if (this == &other) {
      return *this;
    }

    value = other.value;
    gradient = std::move(other.gradient);
    hessian = std::move(other.hessian);
    equalityResidual = std::move(other.equalityResidual);
    equalityJacobian = std::move(other.equalityJacobian);
    bodyDofOffsets.assign(
        other.bodyDofOffsets.begin(), other.bodyDofOffsets.end());
    activeConstraints.assign(
        other.activeConstraints.begin(), other.activeConstraints.end());
    activeFrictionConstraints.assign(
        other.activeFrictionConstraints.begin(),
        other.activeFrictionConstraints.end());
    activeArticulationConstraints.assign(
        other.activeArticulationConstraints.begin(),
        other.activeArticulationConstraints.end());
    activeDynamicsTerms = other.activeDynamicsTerms;
    return *this;
  }

  double value = 0.0;
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd equalityResidual;
  Eigen::SparseMatrix<double> equalityJacobian;
  BodyDofOffsetVector bodyDofOffsets;
  ConstraintVector activeConstraints;
  FrictionConstraintVector activeFrictionConstraints;
  ArticulationConstraintVector activeArticulationConstraints;
  std::size_t activeDynamicsTerms = 0;
};

struct RigidIpcBodyDynamicsTerm
{
  bool active = false;
  RigidIpcPose targetPose;
  RigidIpcVector6d diagonalWeights = RigidIpcVector6d::Zero();
  RigidIpcVector6d generalizedForce = RigidIpcVector6d::Zero();
};

/// Physical rigid-body state used to build one internal dynamics objective row.
///
/// The generalized vectors use the same order as the global rigid IPC assembly:
/// linear translation/velocity/force followed by rotation-vector
/// angular-velocity/torque. The current internal dynamics model uses diagonal
/// rotational weights, so only the inertia diagonal contributes in this slice.
struct RigidIpcBodyDynamicsState
{
  bool active = true;
  RigidIpcPose pose;
  RigidIpcVector6d velocity = RigidIpcVector6d::Zero();
  double mass = 1.0;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  RigidIpcVector6d generalizedForce = RigidIpcVector6d::Zero();
};

using RigidIpcLineSearchOptions = newton_barrier::LineSearchOptions;
using RigidIpcLineSearchStats = newton_barrier::LineSearchStats;

struct RigidIpcLineSearchResult
{
  bool limited = false;
  bool indeterminate = false;
  double stepBound = 1.0;
  RigidIpcBarrierPrimitive limitingPrimitive
      = RigidIpcBarrierPrimitive::VertexVertex;
  std::size_t bodyA = RigidIpcBarrierAssembly::npos;
  std::size_t bodyB = RigidIpcBarrierAssembly::npos;
  std::array<std::size_t, 4> vertices{
      RigidIpcBarrierAssembly::npos,
      RigidIpcBarrierAssembly::npos,
      RigidIpcBarrierAssembly::npos,
      RigidIpcBarrierAssembly::npos};
  RigidIpcLineSearchStats stats;

  [[nodiscard]] bool allowsPositiveStep() const noexcept
  {
    return newton_barrier::allowsPositiveLineSearchStep(
        stepBound, indeterminate);
  }

  [[nodiscard]] bool allowsFullStep() const noexcept
  {
    return newton_barrier::allowsFullLineSearchStep(
        stepBound, limited, indeterminate);
  }
};

enum class RigidIpcProjectedNewtonStatus
{
  NoDofs,
  Converged,
  DescentStep,
  LineSearchBlocked,
  FactorizationFailed,
};

enum class RigidIpcProjectedNewtonSolveStatus
{
  NoDofs,
  Converged,
  MaxIterations,
  LineSearchBlocked,
  FactorizationFailed,
};

struct RigidIpcProjectedNewtonOptions
{
  double gradientTolerance = 1e-8;
  /// Relative gradient convergence floor. The effective gradient tolerance is
  /// `max(gradientTolerance, relativeGradientTolerance * initialGradientNorm)`.
  /// A stiff adaptive barrier makes the absolute tolerance unreachable, so a
  /// small relative floor lets a near-stationary resting contact converge
  /// instead of exhausting the iteration budget. Default 0 keeps the absolute
  /// criterion only (no behavior change for existing callers).
  double relativeGradientTolerance = 0.0;
  double hessianRegularization = 1e-8;
  double maxStepNorm = std::numeric_limits<double>::infinity();
  double lineSearchSafetyScale = 0.99;
  /// When enabled, the solve loop backtracks feasible Newton steps until the
  /// assembled objective satisfies an Armijo sufficient-decrease check. If the
  /// finite backtracking budget misses Armijo but finds a decreasing feasible
  /// candidate, the solve accepts that best decreasing candidate rather than
  /// treating the step as an unsafe CCD block.
  bool useSufficientDecreaseLineSearch = true;
  double sufficientDecreaseFactor
      = newton_barrier::kDefaultSufficientDecreaseFactor;
  double backtrackingScale = newton_barrier::kDefaultBacktrackingScale;
  std::size_t maxBacktrackingIterations = 16;
};

struct RigidIpcProjectedNewtonStats
{
  std::size_t dofs = 0;
  double gradientNorm = 0.0;
  double rawStepNorm = 0.0;
  double stepNorm = 0.0;
  double stepScale = 1.0;
  double hessianRegularization = 0.0;
  double gradientDotStep = 0.0;
  bool usedLineSearch = false;
  bool lineSearchLimited = false;
};

class RigidIpcProjectedNewtonDelta
{
public:
  using Allocator = dart::common::StlAllocator<double>;
  using Vector = std::vector<double, Allocator>;
  using EigenMap = Eigen::Map<Eigen::VectorXd>;
  using ConstEigenMap = Eigen::Map<const Eigen::VectorXd>;

  RigidIpcProjectedNewtonDelta() = default;

  explicit RigidIpcProjectedNewtonDelta(
      dart::common::MemoryAllocator& allocator)
    : m_values(Allocator{allocator})
  {
  }

  [[nodiscard]] Eigen::Index size() const noexcept
  {
    return static_cast<Eigen::Index>(m_values.size());
  }

  void resize(Eigen::Index size)
  {
    m_values.resize(static_cast<std::size_t>(std::max<Eigen::Index>(0, size)));
  }

  void setZero()
  {
    std::fill(m_values.begin(), m_values.end(), 0.0);
  }

  [[nodiscard]] double norm() const
  {
    return asEigen().norm();
  }

  [[nodiscard]] double& operator[](Eigen::Index index) noexcept
  {
    return m_values[static_cast<std::size_t>(index)];
  }

  [[nodiscard]] double operator[](Eigen::Index index) const noexcept
  {
    return m_values[static_cast<std::size_t>(index)];
  }

  [[nodiscard]] EigenMap asEigen() noexcept
  {
    return EigenMap(m_values.data(), size());
  }

  [[nodiscard]] ConstEigenMap asEigen() const noexcept
  {
    return ConstEigenMap(m_values.data(), size());
  }

  template <typename Derived>
  RigidIpcProjectedNewtonDelta& operator=(
      const Eigen::MatrixBase<Derived>& values)
  {
    resize(values.size());
    asEigen() = values;
    return *this;
  }

  RigidIpcProjectedNewtonDelta& operator*=(double scale)
  {
    asEigen() *= scale;
    return *this;
  }

private:
  Vector m_values;
};

struct RigidIpcProjectedNewtonStep
{
  RigidIpcProjectedNewtonStep() = default;

  explicit RigidIpcProjectedNewtonStep(dart::common::MemoryAllocator& allocator)
    : delta(allocator)
  {
  }

  RigidIpcProjectedNewtonStatus status
      = RigidIpcProjectedNewtonStatus::FactorizationFailed;
  bool success = false;
  bool converged = false;
  bool lineSearchBlocked = false;
  RigidIpcProjectedNewtonDelta delta;
  RigidIpcProjectedNewtonStats stats;

  [[nodiscard]] bool hasDescentDirection() const noexcept
  {
    return success && delta.size() > 0 && stats.gradientDotStep < 0.0;
  }
};

/// Adaptive barrier-stiffness controls for the projected-Newton solve.
///
/// When `enabled`, the solve replaces the fixed `barrier.stiffness` with the
/// IPC adaptive-kappa scheme (Li et al. 2020; the same
/// `initial_barrier_stiffness` / `update_barrier_stiffness` algorithm used by
/// the `ipc-sim/rigid-ipc` reference): an initial kappa that balances the
/// barrier gradient against the inertial energy gradient and is clamped to
/// `[kappa_min, 100*kappa_min]`, then per-iteration doubling whenever the
/// closest pair keeps approaching inside the `dhatEpsilonScale` band. This
/// prevents the body from creeping into penetration under a too-soft barrier
/// (which otherwise traps the conservative line search at a zero step).
/// Defaults mirror the reference
/// (`min_barrier_stiffness_scale = 1e11`, `dhat_epsilon = 1e-9`).
struct RigidIpcAdaptiveStiffnessOptions
{
  bool enabled = false;
  double averageMass = 1.0;
  double bboxDiagonal = 1.0;
  double minStiffnessScale = 1e11;
  double dhatEpsilonScale = 1e-9;
};

struct RigidIpcProjectedNewtonSolveOptions
{
  using DynamicsTermAllocator
      = dart::common::StlAllocator<RigidIpcBodyDynamicsTerm>;
  using ArticulationConstraintInputAllocator
      = dart::common::StlAllocator<RigidIpcArticulationConstraintInput>;

  RigidIpcProjectedNewtonSolveOptions() = default;

  explicit RigidIpcProjectedNewtonSolveOptions(
      dart::common::MemoryAllocator& allocator)
    : dynamicsTerms(DynamicsTermAllocator{allocator}),
      articulationConstraints(ArticulationConstraintInputAllocator{allocator})
  {
  }

  RigidIpcBarrierOptions barrier;
  RigidIpcFrictionOptions friction;
  RigidIpcLineSearchOptions lineSearch;
  RigidIpcProjectedNewtonOptions newton;
  RigidIpcAdaptiveStiffnessOptions adaptiveStiffness;
  std::vector<RigidIpcBodyDynamicsTerm, DynamicsTermAllocator> dynamicsTerms;
  std::vector<
      RigidIpcArticulationConstraintInput,
      ArticulationConstraintInputAllocator>
      articulationConstraints;
  std::size_t maxIterations = 16;
  std::size_t frictionIterations = 1;
  double stepTolerance = 1e-10;
  double equalityTolerance = 1e-10;
  double frictionConvergenceTolerance = 0.0;
  bool useLineSearch = true;
};

struct RigidIpcProjectedNewtonSolveStats
{
  std::size_t iterations = 0;
  std::size_t acceptedSteps = 0;
  std::size_t lineSearchLimitedSteps = 0;
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
  std::size_t activeArticulationConstraints = 0;
  std::size_t activeFrictionConstraints = 0;
  std::size_t frictionIterations = 0;
  std::size_t barrierStiffnessIncreases = 0;
  double initialValue = 0.0;
  double finalValue = 0.0;
  double initialGradientNorm = 0.0;
  double finalGradientNorm = 0.0;
  double initialEqualityResidualNorm = 0.0;
  double finalEqualityResidualNorm = 0.0;
  double finalMomentumBalance = 0.0;
  double lastStepNorm = 0.0;
  double barrierStiffness = 0.0;
};

struct RigidIpcProjectedNewtonSolveResult
{
  using SurfaceAllocator = dart::common::StlAllocator<RigidIpcBarrierSurface>;

  RigidIpcProjectedNewtonSolveResult() = default;

  explicit RigidIpcProjectedNewtonSolveResult(
      dart::common::MemoryAllocator& allocator)
    : surfaces(SurfaceAllocator{allocator}),
      assembly(allocator),
      lastStep(allocator)
  {
  }

  RigidIpcProjectedNewtonSolveStatus status
      = RigidIpcProjectedNewtonSolveStatus::MaxIterations;
  bool converged = false;
  bool failed = false;
  std::vector<RigidIpcBarrierSurface, SurfaceAllocator> surfaces;
  RigidIpcBarrierAssembly assembly;
  RigidIpcLineSearchResult lineSearch;
  RigidIpcProjectedNewtonStep lastStep;
  RigidIpcProjectedNewtonSolveStats stats;

  [[nodiscard]] bool madeProgress() const noexcept
  {
    return stats.acceptedSteps > 0;
  }
};

struct RigidIpcProjectedNewtonSolveScratchWorkspace;

struct RigidIpcProjectedNewtonSolveScratch
{
  using SurfaceAllocator = dart::common::StlAllocator<RigidIpcBarrierSurface>;

  RigidIpcProjectedNewtonSolveScratch() = default;
  explicit RigidIpcProjectedNewtonSolveScratch(
      dart::common::MemoryAllocator& allocator);
  ~RigidIpcProjectedNewtonSolveScratch();

  RigidIpcProjectedNewtonSolveScratch(
      const RigidIpcProjectedNewtonSolveScratch&) = delete;
  RigidIpcProjectedNewtonSolveScratch& operator=(
      const RigidIpcProjectedNewtonSolveScratch&) = delete;
  RigidIpcProjectedNewtonSolveScratch(
      RigidIpcProjectedNewtonSolveScratch&& other) noexcept;
  RigidIpcProjectedNewtonSolveScratch& operator=(
      RigidIpcProjectedNewtonSolveScratch&& other) noexcept;

  dart::common::MemoryAllocator* memoryAllocator = nullptr;
  std::vector<RigidIpcBarrierSurface, SurfaceAllocator> laggedSurfaces;
  std::vector<RigidIpcBarrierSurface, SurfaceAllocator> lineSearchStartSurfaces;
  std::vector<RigidIpcBarrierSurface, SurfaceAllocator> candidateSurfaces;
  std::vector<RigidIpcBarrierSurface, SurfaceAllocator> acceptedSurfaces;
  std::vector<RigidIpcBarrierSurface, SurfaceAllocator> bestDecreasingSurfaces;
  RigidIpcProjectedNewtonStep step;
  RigidIpcProjectedNewtonSolveScratchWorkspace* workspace = nullptr;
};

/// Evaluates the IPC point-triangle barrier after rigid pose interpolation.
///
/// The returned derivatives are with respect to the transformed world-space
/// primitive coordinates. Generalized rigid-body derivatives are intentionally
/// left to the solver assembly layer.
[[nodiscard]] DART_SIMULATION_API RigidIpcPrimitiveBarrierResult
rigidIpcPointTriangleBarrierAtTime(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    double time,
    const RigidIpcBarrierOptions& options);

/// Evaluates the IPC point-edge barrier after rigid pose interpolation.
///
/// The returned derivatives are with respect to the transformed world-space
/// primitive coordinates. Generalized rigid-body derivatives are intentionally
/// left to the solver assembly layer.
[[nodiscard]] DART_SIMULATION_API RigidIpcPrimitiveBarrierResult
rigidIpcPointEdgeBarrierAtTime(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    double time,
    const RigidIpcBarrierOptions& options);

/// Evaluates the IPC edge-edge barrier after rigid pose interpolation.
///
/// The returned derivatives are with respect to the transformed world-space
/// primitive coordinates. Generalized rigid-body derivatives are intentionally
/// left to the solver assembly layer.
[[nodiscard]] DART_SIMULATION_API RigidIpcPrimitiveBarrierResult
rigidIpcEdgeEdgeBarrierAtTime(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    double time,
    const RigidIpcBarrierOptions& options);

/// Evaluates the IPC point-point barrier after rigid pose interpolation.
///
/// The returned derivatives are with respect to the transformed world-space
/// primitive coordinates. Generalized rigid-body derivatives are intentionally
/// left to the solver assembly layer.
[[nodiscard]] DART_SIMULATION_API RigidIpcPrimitiveBarrierResult
rigidIpcPointPointBarrierAtTime(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& pointAPoseStart,
    const RigidIpcPose& pointAPoseEnd,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& pointBPoseStart,
    const RigidIpcPose& pointBPoseEnd,
    double time,
    const RigidIpcBarrierOptions& options);

/// Evaluates a face-vertex barrier in reduced two-rigid-body coordinates.
///
/// The reduced coordinate order is point-body translation, point-body rotation
/// vector, triangle-body translation, triangle-body rotation vector. The
/// primitive derivative is chained through each local vertex transform,
/// matching the rigid IPC reduced-coordinate assembly shape before global
/// sparse insertion.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedBarrierResult
rigidIpcPointTriangleReducedBarrier(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePose,
    const RigidIpcBarrierOptions& options);

/// Evaluates an edge-vertex barrier in reduced two-rigid-body coordinates.
///
/// The reduced coordinate order is point-body translation, point-body rotation
/// vector, edge-body translation, edge-body rotation vector. The primitive
/// derivative is chained through each local vertex transform, matching the
/// rigid IPC reduced-coordinate assembly shape before global sparse insertion.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedBarrierResult
rigidIpcPointEdgeReducedBarrier(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePose,
    const RigidIpcBarrierOptions& options);

/// Evaluates an edge-edge barrier in reduced two-rigid-body coordinates.
///
/// The reduced coordinate order is edge-A-body translation, edge-A-body
/// rotation vector, edge-B-body translation, edge-B-body rotation vector. The
/// primitive derivative is chained through each local vertex transform,
/// matching the rigid IPC reduced-coordinate assembly shape before global
/// sparse insertion.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedBarrierResult
rigidIpcEdgeEdgeReducedBarrier(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPose,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPose,
    const RigidIpcBarrierOptions& options);

/// Evaluates a vertex-vertex barrier in reduced two-rigid-body coordinates.
///
/// The reduced coordinate order is point-A-body translation, point-A-body
/// rotation vector, point-B-body translation, point-B-body rotation vector. The
/// primitive derivative is chained through each local vertex transform,
/// matching the rigid IPC reduced-coordinate assembly shape before global
/// sparse insertion.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedBarrierResult
rigidIpcPointPointReducedBarrier(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& pointAPose,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& pointBPose,
    const RigidIpcBarrierOptions& options);

/// Evaluates the lagged smoothed Coulomb friction potential for a point-point
/// contact in world coordinates.
///
/// `laggedPointA` and `laggedPointB` define the tangent basis and static
/// contact anchor. `pointA` and `pointB` are the current world points.
/// Derivatives are with respect to stacked current world coordinates [pointA,
/// pointB] in the leading six entries of the returned 12D derivative storage.
[[nodiscard]] DART_SIMULATION_API RigidIpcFrictionPotentialResult
rigidIpcPointPointFrictionPotential(
    const Eigen::Vector3d& laggedPointA,
    const Eigen::Vector3d& laggedPointB,
    const Eigen::Vector3d& pointA,
    const Eigen::Vector3d& pointB,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged smoothed Coulomb friction potential for a point-edge
/// contact in world coordinates.
///
/// Derivatives are with respect to stacked current world coordinates [point,
/// edgeA, edgeB] in the leading nine entries of the returned 12D derivative
/// storage.
[[nodiscard]] DART_SIMULATION_API RigidIpcFrictionPotentialResult
rigidIpcPointEdgeFrictionPotential(
    const Eigen::Vector3d& laggedPoint,
    const Eigen::Vector3d& laggedEdgeA,
    const Eigen::Vector3d& laggedEdgeB,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged smoothed Coulomb friction potential for an edge-edge
/// contact in world coordinates.
///
/// Derivatives are with respect to stacked current world coordinates [edgeA0,
/// edgeA1, edgeB0, edgeB1].
[[nodiscard]] DART_SIMULATION_API RigidIpcFrictionPotentialResult
rigidIpcEdgeEdgeFrictionPotential(
    const Eigen::Vector3d& laggedEdgeA0,
    const Eigen::Vector3d& laggedEdgeA1,
    const Eigen::Vector3d& laggedEdgeB0,
    const Eigen::Vector3d& laggedEdgeB1,
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged smoothed Coulomb friction potential for a face-vertex
/// contact in world coordinates.
///
/// Derivatives are with respect to stacked current world coordinates [point,
/// triangleA, triangleB, triangleC].
[[nodiscard]] DART_SIMULATION_API RigidIpcFrictionPotentialResult
rigidIpcPointTriangleFrictionPotential(
    const Eigen::Vector3d& laggedPoint,
    const Eigen::Vector3d& laggedTriangleA,
    const Eigen::Vector3d& laggedTriangleB,
    const Eigen::Vector3d& laggedTriangleC,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged point-point friction potential and maps derivatives to
/// two-body 6-DOF rigid pose coordinates.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedFrictionResult
rigidIpcPointPointReducedFrictionPotential(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& laggedPointAPose,
    const RigidIpcPose& pointAPose,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& laggedPointBPose,
    const RigidIpcPose& pointBPose,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged point-edge friction potential and maps derivatives to
/// two-body 6-DOF rigid pose coordinates.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedFrictionResult
rigidIpcPointEdgeReducedFrictionPotential(
    const Eigen::Vector3d& point,
    const RigidIpcPose& laggedPointPose,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& laggedEdgePose,
    const RigidIpcPose& edgePose,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged edge-edge friction potential and maps derivatives to
/// two-body 6-DOF rigid pose coordinates.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedFrictionResult
rigidIpcEdgeEdgeReducedFrictionPotential(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& laggedEdgeAPose,
    const RigidIpcPose& edgeAPose,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& laggedEdgeBPose,
    const RigidIpcPose& edgeBPose,
    const RigidIpcFrictionOptions& options);

/// Evaluates the lagged face-vertex friction potential and maps derivatives to
/// two-body 6-DOF rigid pose coordinates.
[[nodiscard]] DART_SIMULATION_API RigidIpcReducedFrictionResult
rigidIpcPointTriangleReducedFrictionPotential(
    const Eigen::Vector3d& point,
    const RigidIpcPose& laggedPointPose,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& laggedTrianglePose,
    const RigidIpcPose& trianglePose,
    const RigidIpcFrictionOptions& options);

/// Assemble cross-body rigid IPC distance barrier rows for surface meshes.
///
/// This first scene-level assembly layer owns active primitive rows and
/// scatters their two-body reduced derivatives into a global
/// 6-DOF-per-dynamic-body vector/matrix. Static bodies contribute primitive
/// geometry but receive no global DOFs.
[[nodiscard]] DART_SIMULATION_API RigidIpcBarrierAssembly
assembleRigidIpcBarrierSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcBarrierOptions& options);

/// Assemble rigid IPC barrier rows plus per-body dynamics objective terms.
///
/// Dynamics terms use the same 6-DOF order as global assembly: translation,
/// then rotation vector. Their diagonal weights represent the positive
/// quadratic inertia/fixed-coordinate contribution for this internal slice, and
/// `generalizedForce` contributes the matching linear force/torque term.
[[nodiscard]] DART_SIMULATION_API RigidIpcBarrierAssembly
assembleRigidIpcObjectiveSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    const RigidIpcBarrierOptions& options);

/// Assemble rigid IPC barrier, lagged friction, and per-body dynamics terms.
///
/// Friction rows use active barrier rows from `laggedSurfaces` to define
/// tangent stencils and normal-force weights, then evaluate the smoothed
/// Coulomb potential against the current `surfaces`. Empty `laggedSurfaces` or
/// non-positive friction options leave the objective frictionless.
[[nodiscard]] DART_SIMULATION_API RigidIpcBarrierAssembly
assembleRigidIpcObjectiveSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions);

/// Assemble rigid IPC barrier, lagged friction, per-body dynamics, and
/// articulation equality rows. Articulation rows are stored in
/// `equalityResidual`/`equalityJacobian` and enforced by the projected-Newton
/// solve's private equality change-of-variable step.
[[nodiscard]] DART_SIMULATION_API RigidIpcBarrierAssembly
assembleRigidIpcObjectiveSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions);

/// Build a diagonal generalized dynamics objective term from physical state.
///
/// The target pose is explicit Euler prediction `pose + velocity * timeStep`;
/// the weights are `mass / timeStep^2` for translation and diagonal inertia
/// over `timeStep^2` for rotation. Invalid, static, or non-positive-mass
/// inputs produce an inactive term.
[[nodiscard]] DART_SIMULATION_API RigidIpcBodyDynamicsTerm
makeRigidIpcBodyDynamicsTerm(
    const RigidIpcBodyDynamicsState& state, double timeStep);

/// Compute a conservative rigid IPC line-search step bound over surface pairs.
///
/// The start and end surfaces must have matching body order and topology. The
/// returned step bound is in [0, 1] and is shortened by curved rigid CCD over
/// edge-vertex, edge-edge, and face-vertex primitives. Vertex-vertex CCD is not
/// yet represented by the rigid curved primitive set, so vertex-vertex remains
/// a barrier-only row in this slice.
[[nodiscard]] DART_SIMULATION_API RigidIpcLineSearchResult
computeRigidIpcLineSearchStepBound(
    std::span<const RigidIpcBarrierSurface> startSurfaces,
    std::span<const RigidIpcBarrierSurface> endSurfaces,
    const RigidIpcLineSearchOptions& options = {});

/// Compute one internal projected-Newton step from a rigid IPC barrier system.
///
/// This helper consumes the assembled barrier gradient and PSD-projected sparse
/// Hessian, adds configurable diagonal regularization for singular active
/// sets, and optionally scales the step by a conservative line-search result.
/// It does not mutate world state or select a runtime solver method.
[[nodiscard]] DART_SIMULATION_API RigidIpcProjectedNewtonStep
computeRigidIpcProjectedNewtonStep(
    const RigidIpcBarrierAssembly& assembly,
    const RigidIpcProjectedNewtonOptions& options = {});

/// Compute one internal projected-Newton step with a line-search feasibility
/// bound from the same candidate displacement.
[[nodiscard]] DART_SIMULATION_API RigidIpcProjectedNewtonStep
computeRigidIpcProjectedNewtonStep(
    const RigidIpcBarrierAssembly& assembly,
    const RigidIpcLineSearchResult& lineSearch,
    const RigidIpcProjectedNewtonOptions& options = {});

/// Run a small internal projected-Newton loop over rigid IPC barrier rows.
///
/// The loop updates only the copied surface poses in the returned result. It is
/// a barrier-solver scaffold for tests and later runtime integration, not a
/// `World` stepping policy.
[[nodiscard]] DART_SIMULATION_API RigidIpcProjectedNewtonSolveResult
solveRigidIpcProjectedNewtonBarrierSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcProjectedNewtonSolveOptions& options = {});

/// Run the projected-Newton solve into caller-owned result and scratch storage.
///
/// This is the hot-path form for simulation stages that repeatedly solve the
/// same scene shape. The return-by-value overload remains the convenience API
/// for tests and one-off callers.
DART_SIMULATION_API void solveRigidIpcProjectedNewtonBarrierSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcProjectedNewtonSolveOptions& options,
    RigidIpcProjectedNewtonSolveResult& result,
    RigidIpcProjectedNewtonSolveScratch& scratch);

/// Prewarm solve-local projected-Newton assembly scratch for a same-shape
/// solve. This performs assembly work but does not mutate input surfaces or
/// solver results; allocator-backed storage owned by @p scratch remains
/// reserved for later solves.
DART_SIMULATION_API void prewarmRigidIpcProjectedNewtonAssemblyScratch(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions,
    RigidIpcProjectedNewtonSolveScratch& scratch);

/// Compute the initial IPC adaptive barrier stiffness (kappa) for a solve.
///
/// Ports `ipc::initial_barrier_stiffness` (with `dmin = 0`) using DART's
/// squared-distance clamped-log barrier for the `kappa_min` bound: the
/// suggested kappa best cancels the inertial energy gradient with the unit
/// barrier gradient, clamped to `[kappa_min, 100*kappa_min]`, where
/// `kappa_min = minStiffnessScale * averageMass / (4*d0^2*b''(d0^2, dhat^2))`
/// and `d0 = 1e-8 * bboxDiagonal`. `maxStiffness` is set to the clamp ceiling.
/// DART's objective applies kappa directly (it is not normalized by mass like
/// the reference), so the returned kappa is used as `barrier.stiffness`.
[[nodiscard]] DART_SIMULATION_API double computeInitialRigidIpcBarrierStiffness(
    double bboxDiagonal,
    double squaredActivationDistance,
    double averageMass,
    const Eigen::VectorXd& gradEnergy,
    const Eigen::VectorXd& gradBarrier,
    double minStiffnessScale,
    double& maxStiffness);

/// Adaptive kappa update: ports `ipc::update_barrier_stiffness` (with
/// `dmin = 0`). Doubles `currentStiffness` (capped at `maxStiffness`) when the
/// closest active pair stays inside the `dhatEpsilonScale * bboxDiagonal`
/// squared-distance band and keeps approaching, otherwise returns it unchanged.
/// Distances are squared to match DART's squared-distance barrier.
[[nodiscard]] DART_SIMULATION_API double updateRigidIpcBarrierStiffness(
    double prevMinSquaredDistance,
    double minSquaredDistance,
    double maxStiffness,
    double currentStiffness,
    double bboxDiagonal,
    double dhatEpsilonScale);

} // namespace dart::simulation::detail
