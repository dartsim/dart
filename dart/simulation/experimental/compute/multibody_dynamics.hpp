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
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/fwd.hpp>

#include <string_view>

namespace dart::simulation::experimental::comps {
struct MultibodyStructure;
} // namespace dart::simulation::experimental::comps

namespace dart::simulation::experimental::compute {

/// Generalized-coordinate dynamics terms for one multibody at its current
/// configuration and velocity, ordered by joint construction order.
///
/// The terms satisfy the joint-space equation of motion
/// `massMatrix * qddot + coriolisForces + gravityForces = tau`, where `tau` is
/// the applied generalized force. This matches the legacy DART decomposition of
/// `Skeleton::getMassMatrix`, `getCoriolisForces`, and `getGravityForces`.
struct MultibodyDynamicsTerms
{
  Eigen::MatrixXd massMatrix;     ///< M(q), size dof x dof
  Eigen::VectorXd coriolisForces; ///< C(q, qdot) qdot, size dof
  Eigen::VectorXd gravityForces;  ///< g(q), size dof
};

/// Compute the joint-space mass matrix and bias (Coriolis/centrifugal and
/// gravity) generalized forces for a single multibody.
///
/// Uses the same recursive Newton-Euler formulation as the forward-dynamics
/// stage, evaluated at the multibody's current joint positions and velocities.
/// Fixed-base trees with fixed/revolute/prismatic joints are supported; other
/// joint types are rejected. For a multibody with no movable degrees of freedom
/// the returned matrix and vectors are empty.
[[nodiscard]] DART_EXPERIMENTAL_API MultibodyDynamicsTerms
computeMultibodyDynamicsTerms(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity);

/// Compute the generalized joint forces required to produce a desired
/// generalized acceleration at the multibody's current configuration and
/// velocity (inverse dynamics): `tau = M(q) qddot + C(q, qdot) qdot + g(q)`,
/// including any joint armature on the mass-matrix diagonal.
///
/// Uses the recursive Newton-Euler algorithm directly. `desiredAcceleration`
/// must have size equal to the multibody's movable degree-of-freedom count,
/// ordered by joint construction order; the result has the same ordering. For a
/// multibody with no movable degrees of freedom the result is empty.
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::VectorXd
computeMultibodyInverseDynamics(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& desiredAcceleration);

/// Compute the body-frame spatial Jacobian of one link of a multibody.
///
/// The returned 6 x DOF matrix maps the multibody's generalized velocity to the
/// link's spatial velocity `[angular; linear]` expressed in the link's own
/// frame, with columns ordered by joint construction order. It depends only on
/// the current joint configuration (not on world transforms), so it is robust
/// to stale frame caches. Columns of joints that do not move the link are zero.
///
/// @throws InvalidArgumentException if `linkEntity` is not part of `structure`.
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::MatrixXd
computeMultibodyLinkJacobian(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity);

/// Compute the world-frame geometric Jacobian of one link of a multibody.
///
/// The returned 6 x DOF matrix maps the multibody's generalized velocity to the
/// link's spatial velocity `[angular; linear]` expressed in world axes, with
/// the link-frame origin as the linear reference point. It is the body Jacobian
/// with both blocks rotated into world axes. The link world transform is
/// derived from the joint configuration and the base world transform.
///
/// @throws InvalidArgumentException if `linkEntity` is not part of `structure`.
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::MatrixXd
computeMultibodyLinkWorldJacobian(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity);

/// Fixed-base articulated-body forward-dynamics stage.
///
/// For each multibody, this stage computes generalized joint accelerations from
/// joint efforts, gravity, and velocity-dependent (Coriolis/centrifugal) terms,
/// then integrates joint velocities and positions with semi-implicit Euler.
///
/// The dynamics use a recursive Newton-Euler formulation to build the
/// joint-space mass matrix and bias forces, then solve `M qddot = tau - bias`.
/// This matches the legacy DART rigid-body articulated-dynamics results for
/// open chains while keeping the implementation backend-neutral.
///
/// Scope and assumptions:
/// - Fixed-base trees only: the root link is fixed to the world (it has no
///   parent joint). Floating-base (free root joint) dynamics are not yet
///   implemented.
/// - Supported joints: fixed, revolute, and prismatic. Other joint types have
///   kinematics but their dynamics are rejected until implemented.
/// - Each link's center of mass is at the link frame origin, and the link
///   inertia tensor is expressed about that origin.
class DART_EXPERIMENTAL_API MultibodyForwardDynamicsStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Computes unconstrained articulated-body velocities without advancing
/// positions.
///
/// This is the velocity half of the semi-implicit multibody step. It leaves the
/// solved generalized velocities in a transient internal staging component so
/// contact/constraint stages can operate before positions are written back.
class DART_EXPERIMENTAL_API MultibodyVelocityStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Resolves current link-vs-rigid-body contacts against staged multibody
/// velocities.
///
/// This stage preserves the existing link-contact solve while making it an
/// explicit velocity-level stage between unconstrained velocity integration and
/// position write-back.
class DART_EXPERIMENTAL_API MultibodyContactStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Applies multibody velocity limits and advances joint positions from the
/// staged semi-implicit velocities.
class DART_EXPERIMENTAL_API MultibodyPositionStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

} // namespace dart::simulation::experimental::compute
