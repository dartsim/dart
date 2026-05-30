/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#pragma once

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/comps/component_category.hpp>
#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::comps {
struct MultibodyStructure;
} // namespace dart::simulation::experimental::comps

namespace dart::simulation::experimental::compute {

/// Persistent two-step discrete-mechanics history for one multibody, indexed by
/// link construction order (parent-before-child).
///
/// The linear-time variational integrator is a two-step method: evaluating the
/// forced discrete Euler-Lagrange residual at step k needs the relative
/// configuration displacement and the discrete spatial momentum carried in from
/// step k-1. This is dynamic State, owned by the integrator and hidden from the
/// public facade.
///
/// **Internal Implementation Detail** - Not exposed in public API. Serialized
/// (State category) so a saved/loaded trajectory resumes without
/// re-bootstrapping the two-step history. It holds no entity references, so no
/// remapping pass is needed.
struct MultibodyVariationalState
{
  DART_EXPERIMENTAL_STATE_COMPONENT(MultibodyVariationalState);

  /// Whether `previousDeltaTransform`/`previousMomentum` have been seeded from
  /// a consistent prior step. The first integration bootstraps them from the
  /// current generalized velocity.
  bool bootstrapped = false;

  /// Per-link relative configuration displacement dT from the previous step
  /// (the SE(3) transform whose log gives the previous average velocity).
  std::vector<Eigen::Isometry3d> previousDeltaTransform;

  /// Per-link discrete spatial momentum from the previous step (link frame).
  std::vector<Eigen::Matrix<double, 6, 1>> previousMomentum;
};

/// Diagnostics from one RIQN (recursive impulse-based quasi-Newton) solve.
///
/// **Internal Implementation Detail** - Not exposed in public API.
struct VariationalSolveReport
{
  std::size_t iterations = 0; ///< RIQN iterations performed.
  double residualNorm = 0.0;  ///< Final forced-DEL residual L2 norm.
  bool converged = false;     ///< Whether the residual fell below tolerance.
};

/// A holonomic loop-closure constraint between two endpoint frames fixed on two
/// links of the same multibody (or an endpoint on a link and a fixed world
/// anchor when `linkB`/`linkA` is null). Point closures constrain the 3D offset
/// to zero (3 rows); distance closures constrain the separation to `length`
/// (1 row); rigid closures additionally constrain the relative orientation
/// (6 rows: 3 position + 3 world-frame rotation residual).
///
/// **Internal Implementation Detail** - Not exposed in public API.
struct VariationalLoopConstraint
{
  entt::entity linkA = entt::null; ///< null => `pointA` is a world anchor.
  Eigen::Vector3d pointA
      = Eigen::Vector3d::Zero();   ///< local on linkA (or world).
  entt::entity linkB = entt::null; ///< null => `pointB` is a world anchor.
  Eigen::Vector3d pointB
      = Eigen::Vector3d::Zero(); ///< local on linkB (or world).
  bool distance = false;         ///< false: point (3 rows); true: distance (1).
  double length = 0.0;           ///< target separation when `distance`.
  bool rigid
      = false; ///< true: also constrain relative orientation (3 extra rows).
  Eigen::Matrix3d rotationA
      = Eigen::Matrix3d::Identity(); ///< endpoint-A offset
                                     ///< rotation (rigid).
  Eigen::Matrix3d rotationB
      = Eigen::Matrix3d::Identity(); ///< endpoint-B offset
                                     ///< rotation (rigid).
};

/// Advance one multibody by one step with the linear-time variational
/// integrator (Lee, Liu, Park, Srinivasa, WAFR 2016 / arXiv:1609.02898).
///
/// Each step solves the forced discrete Euler-Lagrange equation
/// `D2 Ld(q^{k-1}, q^k) + D1 Ld(q^k, q^{k+1}) + F^k = 0` for the next
/// configuration by RIQN: the residual is evaluated in O(n) by a discrete
/// recursive Newton-Euler sweep (DRNEA), and the quasi-Newton update applies
/// the approximate inverse Jacobian `dt * M(q^k)^{-1}` via an O(n)
/// articulated-body-inertia (ABA) inverse-mass solve, so the whole step is
/// linear-time in the degree-of-freedom count. Gravity enters as a forcing-side
/// spatial impulse (not a Lagrangian potential).
///
/// Scope (Phase A1): fixed-base open chains with fixed, revolute, and prismatic
/// joints; fixed time step. Other joint types and floating bases are rejected.
///
/// Updates the multibody's joint positions, velocities, and accelerations in
/// the registry, and advances `state`. Returns solve diagnostics. `tolerance`
/// is a per-coordinate accuracy; convergence tests the residual L2 norm against
/// `tolerance * sqrt(dofCount)` so the accuracy is uniform across chain
/// lengths. Throws InvalidOperationException if RIQN fails to converge within
/// `maxIterations` (non-convergence is a hard error, not a silent best-effort
/// step).
///
/// When `constraints` is non-empty, holonomic loop closures are enforced after
/// the unconstrained step by an impulse-based position projection onto the
/// constraint manifold `g(q) = 0` (the paper's Sec. 5 extension), reusing the
/// O(n) inverse-mass apply: `lambda = (J M^{-1} J^T)^{-1} (-g)`,
/// `dq = M^{-1} J^T lambda`. This keeps closed loops satisfied each step.
///
/// The `dt * M^{-1}` quasi-Newton preconditioner is only an approximate inverse
/// Jacobian, so the plain iteration's convergence rate degrades for long
/// chains. When the generalized coordinates form a vector space (every movable
/// joint is revolute/prismatic) the RIQN fixed-point iteration is accelerated
/// with depth-m Anderson mixing, which keeps the iteration count bounded for
/// large degree-of-freedom counts; spherical/floating coordinates live on a
/// manifold where linear mixing is invalid and use the plain step.
DART_EXPERIMENTAL_API VariationalSolveReport integrateMultibodyVariational(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    int maxIterations = 100,
    double tolerance = 1e-10,
    const std::vector<VariationalLoopConstraint>& constraints = {});

/// Total mechanical energy (kinetic + gravitational potential) of one multibody
/// at its current configuration and velocity.
///
/// Used to verify the variational integrator's near-energy-conserving behavior
/// on passive systems. Kinetic energy is `0.5 * sum_i V_i^T G_i V_i` over link
/// spatial velocities; potential energy is `- sum_i m_i * g . r_i^com` (zero
/// reference at the world origin). Fixed-base open chains with fixed/revolute/
/// prismatic joints.
[[nodiscard]] DART_EXPERIMENTAL_API double computeMultibodyMechanicalEnergy(
    const entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity);

/// O(n) product `M(q)^{-1} * impulse` for one multibody at its current
/// configuration, via the articulated-body algorithm (zero velocity/gravity).
/// This is the linear-time inverse-mass apply that powers the variational
/// integrator's RIQN step; exposed for testing against the dense mass-matrix
/// solve. Throws InvalidArgumentException if `impulse` size does not equal the
/// movable degree-of-freedom count.
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::VectorXd
computeMultibodyInverseMassProduct(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& impulse);

/// Holonomic loop-closure residual `g(q)` and its Jacobian `J = dg/dq`
/// evaluated at the multibody's current configuration, for the given closures.
/// The Jacobian columns are in the integrator's generalized-coordinate order
/// (the same ordering as `computeMultibodyInverseMassProduct`). Exposed for
/// finite-difference verification of the constraint Jacobian used by the
/// constrained variational step (the impulse-based loop-closure projection).
struct VariationalConstraintLinearization
{
  Eigen::VectorXd residual; ///< `g(q)`, stacked per constraint.
  Eigen::MatrixXd jacobian; ///< `dg/dq` (rows = constraint dim, cols = DOF).
};

[[nodiscard]] DART_EXPERIMENTAL_API VariationalConstraintLinearization
computeVariationalConstraintLinearization(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const std::vector<VariationalLoopConstraint>& constraints);

/// Classification of one public loop closure (`comps::LoopClosure`) for the
/// variational loop-closure solver. Point closures whose endpoints are links of
/// a single multibody and/or a fixed world anchor are `Supported`; everything
/// the variational stage cannot honestly enforce yet (the Rigid and Distance
/// families, rigid-body endpoints, or a closure spanning two multibodies) is
/// `Unsupported` with a human-readable `reason`.
struct VariationalLoopClosureBinding
{
  enum class Status
  {
    Ignored,     ///< Disabled, or not requesting dynamic solving.
    Supported,   ///< Translatable; `structure` and `constraint` are set.
    Unsupported, ///< Requests solving but this stage cannot enforce it.
  };

  Status status = Status::Ignored;
  entt::entity structure = entt::null; ///< Owning multibody (when Supported).
  VariationalLoopConstraint
      constraint;          ///< Translated closure (when Supported).
  std::string_view reason; ///< Why rejected (when Unsupported).
};

/// Classify a loop closure (the `comps::LoopClosure` on `closureEntity`) for
/// the variational loop-closure solver. The World's dynamics-policy validation
/// uses it to accept/reject a `Solve` closure under the variational method, and
/// the variational stage uses it to gather the constraints to enforce, so the
/// two can never disagree. Returns `Ignored` when there is no such component,
/// the closure is disabled, or it does not request `Solve`.
[[nodiscard]] DART_EXPERIMENTAL_API VariationalLoopClosureBinding
bindVariationalLoopClosure(
    const entt::registry& registry, entt::entity closureEntity);

/// Variational-integrator multibody stage (a peer of
/// `MultibodyForwardDynamicsStage`, selected by the `variational integrator`
/// integration-family method name).
///
/// For each multibody, advances the dynamics with
/// `integrateMultibodyVariational`, persisting the two-step discrete-mechanics
/// history as a `MultibodyVariationalState` component on the multibody entity.
///
/// Scope and assumptions match `integrateMultibodyVariational` (Phase A1:
/// fixed-base open chains; fixed/revolute/prismatic joints; no contact).
class DART_EXPERIMENTAL_API MultibodyVariationalIntegrationStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

} // namespace dart::simulation::experimental::compute
