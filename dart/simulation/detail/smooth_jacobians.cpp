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

#include "dart/simulation/detail/smooth_jacobians.hpp"

#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/link.hpp"
#include "dart/simulation/comps/multibody.hpp"
#include "dart/simulation/compute/multibody_dynamics.hpp"

#include <entt/entt.hpp>

#include <cmath>

namespace dart::simulation::detail {

namespace {

//==============================================================================
Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

//==============================================================================
// SO(3) exponential map: a rotation vector (axis * angle) to a rotation matrix.
// Matches `rotationExp` in multibody_dynamics.cpp so the analytic position
// Jacobian differentiates the exact forward integration step.
Eigen::Matrix3d rotationExp(const Eigen::Vector3d& rotationVector)
{
  const double angle = rotationVector.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
}

//==============================================================================
// SO(3) logarithm map: a rotation matrix to a rotation vector (axis * angle).
Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd angleAxis(rotation);
  return angleAxis.angle() * angleAxis.axis();
}

//==============================================================================
// Right Jacobian of SO(3): for the exponential map Exp(phi), it satisfies
// Exp(phi + dphi) ≈ Exp(phi) · Exp(J_r(phi) · dphi). A series-stable
// small-angle branch keeps it well conditioned as ‖phi‖ → 0.
Eigen::Matrix3d rotationRightJacobian(const Eigen::Vector3d& phi)
{
  const double angle = phi.norm();
  const Eigen::Matrix3d cross = skew(phi);
  if (angle < 1e-8) {
    return Eigen::Matrix3d::Identity() - 0.5 * cross;
  }
  const double a2 = angle * angle;
  const double a3 = a2 * angle;
  return Eigen::Matrix3d::Identity() - (1.0 - std::cos(angle)) / a2 * cross
         + (angle - std::sin(angle)) / a3 * (cross * cross);
}

//==============================================================================
// Inverse of the SO(3) right Jacobian, with a small-angle branch matching
// rotationRightJacobian.
Eigen::Matrix3d rotationRightJacobianInverse(const Eigen::Vector3d& phi)
{
  const double angle = phi.norm();
  const Eigen::Matrix3d cross = skew(phi);
  if (angle < 1e-8) {
    return Eigen::Matrix3d::Identity() + 0.5 * cross;
  }
  const double a2 = angle * angle;
  const double coefficient
      = 1.0 / a2 - (1.0 + std::cos(angle)) / (2.0 * angle * std::sin(angle));
  return Eigen::Matrix3d::Identity() + 0.5 * cross
         + coefficient * (cross * cross);
}

//==============================================================================
void collectCoordinatesInto(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    ContactFreeStepCoordinateScratch& coordinates)
{
  coordinates.clear();
  coordinates.reserve(structure.links.size());
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::Link>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.parentJoint);
    const auto dof = static_cast<Eigen::Index>(joint.getDOF());
    for (Eigen::Index d = 0; d < dof; ++d) {
      coordinates.push_back({link.parentJoint, d});
    }
  }
}

//==============================================================================
// Central finite difference of the dynamics terms with respect to one
// generalized coordinate. `applyPerturbation` writes the perturbed value into
// the registry and is followed by a recompute; the caller restores the original
// value afterward. Returns (terms(+h) - terms(-h)) / (2h) for M, c, where
// c = coriolisForces + gravityForces.
struct TermDerivative
{
  Eigen::MatrixXd massMatrix; // dM/d(coordinate)
  Eigen::VectorXd biasForces; // dc/d(coordinate),  c = C q̇ + g
};

} // namespace

//==============================================================================
StepDerivatives contactFreeStepDerivatives(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::Ref<const Eigen::VectorXd>& tau,
    ContactFreeStepCoordinateScratch* coordinateScratch,
    compute::MultibodyInverseDynamicsScratch* inverseDynamicsScratch)
{
  StepDerivatives derivatives;

  ContactFreeStepCoordinateScratch localCoordinates;
  auto& coordinates
      = coordinateScratch != nullptr ? *coordinateScratch : localCoordinates;
  collectCoordinatesInto(registry, structure, coordinates);
  const auto ndof = static_cast<Eigen::Index>(coordinates.size());
  if (ndof == 0) {
    return derivatives;
  }

  // Base-point dynamics terms and current generalized velocity.
  const auto baseTerms
      = compute::computeMultibodyDynamicsTerms(registry, structure, gravity);
  const Eigen::MatrixXd massMatrix = baseTerms.massMatrix;
  const Eigen::VectorXd biasForces
      = baseTerms.coriolisForces + baseTerms.gravityForces; // c = C q̇ + g

  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(ndof);
  for (Eigen::Index k = 0; k < ndof; ++k) {
    const auto& coordinate = coordinates[static_cast<std::size_t>(k)];
    const auto& joint = registry.get<comps::Joint>(coordinate.joint);
    qdot[k] = joint.velocity[coordinate.local];
  }

  const Eigen::MatrixXd inverseMass = massMatrix.inverse();
  const Eigen::VectorXd qddot = inverseMass * (tau - biasForces);

  // Central finite differencing of the dynamics terms. The relative step keeps
  // the perturbation well scaled regardless of the coordinate magnitude.
  const auto evalTerms = [&]() {
    const auto terms
        = compute::computeMultibodyDynamicsTerms(registry, structure, gravity);
    TermDerivative snapshot;
    snapshot.massMatrix = terms.massMatrix;
    snapshot.biasForces = terms.coriolisForces + terms.gravityForces;
    return snapshot;
  };

  // Velocity-block Jacobians of the semi-implicit-Euler velocity update
  //   q̇' = q̇ + Δt · M⁻¹ (τ − c),  with c = C(q,q̇) q̇ + g(q).
  // Writing inverse dynamics as τ_ID = M(q) q̈ + c evaluated at the solved q̈,
  // we have ∂(M q̈ + c)/∂q = ∂τ_ID/∂q and ∂c/∂q̇ = ∂τ_ID/∂q̇, so
  //   ∂q̇'/∂q  = -Δt · M⁻¹ ∂τ_ID/∂q ,   ∂q̇'/∂q̇ = I - Δt · M⁻¹ ∂τ_ID/∂q̇ .
  // The analytic O(dof²) RNEA derivatives are used when the tree supports them
  // (constant unit-twist joints: revolute/prismatic/screw/fixed); otherwise the
  // code falls back to O(dof³) central finite differencing of the dynamics
  // terms (manifold / configuration-dependent joints), the original path.
  Eigen::MatrixXd dVelNext_dq = Eigen::MatrixXd::Zero(ndof, ndof);
  Eigen::MatrixXd dVelNext_dqdot = Eigen::MatrixXd::Identity(ndof, ndof);

  compute::InverseDynamicsDerivatives idDerivatives;
  if (inverseDynamicsScratch != nullptr) {
    compute::computeMultibodyInverseDynamicsDerivativesInto(
        *inverseDynamicsScratch,
        registry,
        structure,
        gravity,
        qddot,
        idDerivatives);
  } else {
    idDerivatives = compute::computeMultibodyInverseDynamicsDerivatives(
        registry, structure, gravity, qddot);
  }

  if (idDerivatives.valid) {
    dVelNext_dq.noalias() = -timeStep * (inverseMass * idDerivatives.dTau_dq);
    dVelNext_dqdot.noalias()
        = Eigen::MatrixXd::Identity(ndof, ndof)
          - timeStep * (inverseMass * idDerivatives.dTau_dqdot);
  } else {
    for (Eigen::Index k = 0; k < ndof; ++k) {
      const auto& coordinate = coordinates[static_cast<std::size_t>(k)];
      auto& joint = registry.get<comps::Joint>(coordinate.joint);

      // --- Position perturbation: dM/dq_k and dc/dq_k. ---
      {
        const double original = joint.position[coordinate.local];
        const double h = 1e-6 * (1.0 + std::abs(original));

        joint.position[coordinate.local] = original + h;
        const TermDerivative plus = evalTerms();
        joint.position[coordinate.local] = original - h;
        const TermDerivative minus = evalTerms();
        joint.position[coordinate.local] = original; // restore exactly

        const Eigen::MatrixXd dMass
            = (plus.massMatrix - minus.massMatrix) / (2.0 * h);
        const Eigen::VectorXd dBias
            = (plus.biasForces - minus.biasForces) / (2.0 * h);

        dVelNext_dq.col(k)
            = timeStep * (-inverseMass * (dMass * qddot) - inverseMass * dBias);
      }

      // --- Velocity perturbation: dc/dq̇_k (the Coriolis velocity derivative).
      // ---
      {
        const double original = joint.velocity[coordinate.local];
        const double h = 1e-6 * (1.0 + std::abs(original));

        joint.velocity[coordinate.local] = original + h;
        const TermDerivative plus = evalTerms();
        joint.velocity[coordinate.local] = original - h;
        const TermDerivative minus = evalTerms();
        joint.velocity[coordinate.local] = original; // restore exactly

        const Eigen::VectorXd dBias
            = (plus.biasForces - minus.biasForces) / (2.0 * h);

        dVelNext_dqdot.col(k) += timeStep * (-inverseMass * dBias);
      }
    }
  }

  // ∂q̇'/∂τ = Δt * Minv.
  const Eigen::MatrixXd dVelNext_dtau = timeStep * inverseMass;

  // Joint-type-keyed position map q' = Φ(q, q̇'). The next velocity q̇' is the
  // semi-implicit-Euler result already assembled above, so by the chain rule
  //   ∂q'/∂q   = ∂Φ/∂q + ∂Φ/∂q̇' · ∂q̇'/∂q
  //   ∂q'/∂q̇  =          ∂Φ/∂q̇' · ∂q̇'/∂q̇
  //   ∂q'/∂τ   =          ∂Φ/∂q̇' · ∂q̇'/∂τ
  // where ∂Φ/∂q (`posPartialPos`) and ∂Φ/∂q̇' (`posPartialVel`) are block
  // diagonal over joints. Euclidean coordinates contribute the classic
  // I / Δt·I blocks; Spherical (SO(3)) and Floating (SE(3)) joints contribute
  // the right/left exponential-map Jacobians (dexp/dlog) and, for free joints,
  // a translation-vs-orientation coupling block. These differentiate the EXACT
  // manifold integration in `simulateMultibody` (multibody_dynamics.cpp).
  Eigen::MatrixXd posPartialPos = Eigen::MatrixXd::Identity(ndof, ndof);
  Eigen::MatrixXd posPartialVel
      = timeStep * Eigen::MatrixXd::Identity(ndof, ndof);

  // The next generalized velocity at the base point, ordered by global DOF.
  const Eigen::VectorXd nextVelocity = qdot + timeStep * qddot;

  // Walk joints in DOF order, filling the manifold blocks for ball/free joints
  // and leaving the Euclidean default in place for the rest.
  Eigen::Index offset = 0;
  while (offset < ndof) {
    const auto& coordinate = coordinates[static_cast<std::size_t>(offset)];
    const auto& joint = registry.get<comps::Joint>(coordinate.joint);
    // Movable joints in `coordinates` always have a positive DOF count, so this
    // advances by at least one each iteration.
    const auto jointDof = static_cast<Eigen::Index>(joint.getDOF());

    if (joint.type == comps::JointType::Spherical) {
      // q_local is the rotation vector; q̇'_local is the body angular velocity.
      const Eigen::Vector3d phi = joint.position.head<3>();
      const Eigen::Vector3d omegaNext = nextVelocity.segment<3>(offset);
      const Eigen::Vector3d twist = omegaNext * timeStep;
      const Eigen::Vector3d phiNext
          = rotationLog(rotationExp(phi) * rotationExp(twist));

      const Eigen::Matrix3d jrInvNext = rotationRightJacobianInverse(phiNext);
      posPartialPos.block<3, 3>(offset, offset)
          = jrInvNext * rotationExp(-twist) * rotationRightJacobian(phi);
      posPartialVel.block<3, 3>(offset, offset)
          = jrInvNext * rotationRightJacobian(twist) * timeStep;
    } else if (joint.type == comps::JointType::Floating) {
      // q_local = [translation; rotation vector]; q̇'_local = [linear; angular]
      // body twist. Translation advances in the parent frame (R · v), rotation
      // integrates on SO(3).
      const Eigen::Vector3d theta = joint.position.tail<3>();
      const Eigen::Matrix3d rotation = rotationExp(theta);
      const Eigen::Vector3d linearNext = nextVelocity.segment<3>(offset);
      const Eigen::Vector3d angularNext = nextVelocity.segment<3>(offset + 3);
      const Eigen::Vector3d twist = angularNext * timeStep;
      const Eigen::Vector3d thetaNext
          = rotationLog(rotation * rotationExp(twist));

      const Eigen::Matrix3d jrTheta = rotationRightJacobian(theta);
      const Eigen::Matrix3d jrInvNext = rotationRightJacobianInverse(thetaNext);

      // ∂Φ/∂q: translation rows depend on translation (I) and on orientation
      // (the coupling block); orientation rows are the SO(3) dexp/dlog block.
      posPartialPos.block<3, 3>(offset, offset).setIdentity();
      posPartialPos.block<3, 3>(offset, offset + 3)
          = -rotation * skew(linearNext) * jrTheta * timeStep;
      posPartialPos.block<3, 3>(offset + 3, offset).setZero();
      posPartialPos.block<3, 3>(offset + 3, offset + 3)
          = jrInvNext * rotationExp(-twist) * jrTheta;

      // ∂Φ/∂q̇': translation advances by R·Δt; orientation by the SO(3) block.
      posPartialVel.block<3, 3>(offset, offset) = rotation * timeStep;
      posPartialVel.block<3, 3>(offset, offset + 3).setZero();
      posPartialVel.block<3, 3>(offset + 3, offset).setZero();
      posPartialVel.block<3, 3>(offset + 3, offset + 3)
          = jrInvNext * rotationRightJacobian(twist) * timeStep;
    }

    offset += jointDof;
  }

  const Eigen::MatrixXd dPosNext_dq
      = posPartialPos + posPartialVel * dVelNext_dq;
  const Eigen::MatrixXd dPosNext_dqdot = posPartialVel * dVelNext_dqdot;
  const Eigen::MatrixXd dPosNext_dtau = posPartialVel * dVelNext_dtau;

  // Assemble the state Jacobian [[∂q'/∂q, ∂q'/∂q̇], [∂q̇'/∂q, ∂q̇'/∂q̇]].
  derivatives.stateJacobian = Eigen::MatrixXd::Zero(2 * ndof, 2 * ndof);
  derivatives.stateJacobian.topLeftCorner(ndof, ndof) = dPosNext_dq;
  derivatives.stateJacobian.topRightCorner(ndof, ndof) = dPosNext_dqdot;
  derivatives.stateJacobian.bottomLeftCorner(ndof, ndof) = dVelNext_dq;
  derivatives.stateJacobian.bottomRightCorner(ndof, ndof) = dVelNext_dqdot;

  // Assemble the control Jacobian [[∂q'/∂τ], [∂q̇'/∂τ]].
  derivatives.controlJacobian = Eigen::MatrixXd::Zero(2 * ndof, ndof);
  derivatives.controlJacobian.topRows(ndof) = dPosNext_dtau;
  derivatives.controlJacobian.bottomRows(ndof) = dVelNext_dtau;

  return derivatives;
}

} // namespace dart::simulation::detail
