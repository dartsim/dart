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

#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/fwd.hpp>

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
/// **Internal Implementation Detail** - Not exposed in public API.
struct MultibodyVariationalState
{
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
/// the registry, and advances `state`. Returns solve diagnostics.
DART_EXPERIMENTAL_API VariationalSolveReport integrateMultibodyVariational(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    int maxIterations = 50,
    double tolerance = 1e-10);

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
