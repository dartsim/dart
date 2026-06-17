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

// @internal
//
// Implementation-detail header for opt-in differentiable simulation (PLAN-110
// WS1). It is NOT a public facade header: it takes ECS/registry and component
// types, which must never appear in a public signature. It lives under
// `detail/` so Doxygen and the API-boundary checks treat it as internal, and it
// is not part of the DART 7 public-facade promotion target. In-tree tests may
// include it directly.

#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entity/entity.hpp>

#include <vector>

namespace dart::simulation::comps {
struct MultibodyStructure;
} // namespace dart::simulation::comps

namespace dart::simulation::detail {

/// One generalized coordinate used by the contact-free differentiable
/// multibody Jacobian path.
struct ContactFreeStepCoordinate
{
  entt::entity joint = entt::null;
  Eigen::Index local = 0;
};

using ContactFreeStepCoordinateAllocator
    = common::StlAllocator<ContactFreeStepCoordinate>;
using ContactFreeStepCoordinateScratch = std::
    vector<ContactFreeStepCoordinate, ContactFreeStepCoordinateAllocator>;

/// Reusable dynamics-term storage for contact-free differentiable Jacobians.
///
/// The contact-free derivative path evaluates base dynamics terms and, for
/// manifold/fallback joints, perturbed dynamics terms. This bundle lets the
/// World keep the DART-owned tree/RNEA scratch and the caller-owned Eigen
/// result capacity across same-shape Jacobian calls.
struct ContactFreeStepDynamicsTermsScratch
{
  explicit ContactFreeStepDynamicsTermsScratch(
      common::MemoryAllocator& allocator
      = common::MemoryAllocator::GetDefault())
    : dynamicsTermsScratch(allocator)
  {
    // Empty.
  }

  compute::MultibodyDynamicsTermsScratch dynamicsTermsScratch;
  compute::MultibodyDynamicsTerms baseTerms;
  compute::MultibodyDynamicsTerms plusTerms;
  compute::MultibodyDynamicsTerms minusTerms;
};

/// Reusable Eigen storage for contact-free differentiable Jacobians.
///
/// Eigen's dynamic matrices allocate through the raw malloc family, so they are
/// invisible to DART's allocator hooks. This scratch keeps same-shape
/// derivative intermediates and decompositions live across warmed
/// differentiable steps.
struct ContactFreeStepDerivativeScratch
{
  Eigen::VectorXd biasForces;
  Eigen::VectorXd qdot;
  Eigen::VectorXd rhs;
  Eigen::VectorXd qddot;
  Eigen::VectorXd nextVelocity;
  Eigen::VectorXd dBias;
  Eigen::VectorXd tempVector;
  Eigen::VectorXd tempVector2;

  Eigen::MatrixXd inverseMass;
  Eigen::MatrixXd dVelNext_dq;
  Eigen::MatrixXd dVelNext_dqdot;
  Eigen::MatrixXd dVelNext_dtau;
  Eigen::MatrixXd posPartialPos;
  Eigen::MatrixXd posPartialVel;
  Eigen::MatrixXd dPosNext_dq;
  Eigen::MatrixXd dPosNext_dqdot;
  Eigen::MatrixXd dPosNext_dtau;
  Eigen::MatrixXd tempMatrix;
  Eigen::MatrixXd tempMatrix2;

  Eigen::LDLT<Eigen::MatrixXd> massMatrixLdlt;
  compute::InverseDynamicsDerivatives inverseDynamicsDerivatives;
};

/// Analytic contact-free single-step Jacobian for ONE multibody, evaluated at
/// its current (pre-step) joint state. Supports fixed, revolute, prismatic,
/// screw, universal, planar, ball (Spherical, SO(3)), and free (Floating,
/// SE(3)) joints.
///
/// The forward step is semi-implicit Euler without contact:
///
///   c     = C(q, q̇) q̇ + g(q)
///   qddot = M⁻¹ (τ − c)
///   q̇'    = q̇ + Δt · qddot
///   q'    = Φ(q, q̇')           (joint-type-keyed manifold integration)
///
/// The velocity-update partials are joint-type independent:
///
///   ∂q̇'/∂q   = Δt · ( −M⁻¹ (∂M/∂q · qddot) − M⁻¹ (∂c/∂q) )
///   ∂q̇'/∂q̇  = I + Δt · ( −M⁻¹ (∂c/∂q̇) )
///   ∂q̇'/∂τ   = Δt · M⁻¹
///
/// The position-update partials are joint-type-keyed. With ∂Φ/∂q and ∂Φ/∂q̇'
/// block diagonal over joints, the chain rule gives
///
///   ∂q'/∂q   = ∂Φ/∂q + ∂Φ/∂q̇' · ∂q̇'/∂q
///   ∂q'/∂q̇  =          ∂Φ/∂q̇' · ∂q̇'/∂q̇
///   ∂q'/∂τ   =          ∂Φ/∂q̇' · ∂q̇'/∂τ
///
/// For Euclidean coordinates Φ = q + Δt · q̇', so ∂Φ/∂q = I and ∂Φ/∂q̇' = Δt · I
/// (recovering the classic blocks). For Spherical joints the rotation vector
/// integrates as q' = Log(Exp(q) · Exp(q̇' Δt)), giving the SO(3) right-Jacobian
/// blocks (dexp/dlog); for Floating joints the same SO(3) blocks apply to the
/// orientation coordinates plus a translation-vs-orientation coupling block
/// (the parent-frame translation update R(orientation) · q̇'_lin Δt). These
/// differentiate the EXACT manifold integration used by `simulateMultibody`.
///
/// The articulated derivatives `∂M/∂q_k`, `∂c/∂q_k`, and `∂c/∂q̇_k` are obtained
/// by CENTRAL finite differencing `computeMultibodyDynamicsTerms` with respect
/// to the registry's joint positions and velocities. The joint state is
/// perturbed in place, the terms recomputed, and the state restored exactly,
/// leaving the registry unchanged on return. This finite-difference-of-terms is
/// distinct from the finite-difference-of-step used by the correctness checker.
///
/// @param registry  The world registry owning the multibody (mutated and
///                   restored during finite differencing; unchanged on return).
/// @param structure The multibody structure.
/// @param gravity   Uniform gravitational acceleration.
/// @param timeStep  Integration time step (> 0).
/// @param tau       Applied generalized effort, size ndof, ordered by joint
///                  construction order.
/// @return The assembled state and control Jacobians. Both matrices are empty
///         when the multibody has no movable degrees of freedom.
[[nodiscard]] DART_SIMULATION_API StepDerivatives contactFreeStepDerivatives(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::Ref<const Eigen::VectorXd>& tau,
    ContactFreeStepCoordinateScratch* coordinateScratch = nullptr,
    compute::MultibodyInverseDynamicsScratch* inverseDynamicsScratch = nullptr,
    ContactFreeStepDynamicsTermsScratch* dynamicsTermsScratch = nullptr,
    ContactFreeStepDerivativeScratch* derivativeScratch = nullptr);

/// Compute contact-free single-step Jacobians into caller-owned output storage.
///
/// `derivatives` is resized and overwritten. Same-shape callers that retain
/// `derivatives` and `derivativeScratch` avoid raw malloc traffic from Eigen
/// dynamic temporaries on warmed differentiable steps.
DART_SIMULATION_API void contactFreeStepDerivativesInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::Ref<const Eigen::VectorXd>& tau,
    StepDerivatives& derivatives,
    ContactFreeStepCoordinateScratch* coordinateScratch = nullptr,
    compute::MultibodyInverseDynamicsScratch* inverseDynamicsScratch = nullptr,
    ContactFreeStepDynamicsTermsScratch* dynamicsTermsScratch = nullptr,
    ContactFreeStepDerivativeScratch* derivativeScratch = nullptr);

} // namespace dart::simulation::detail
