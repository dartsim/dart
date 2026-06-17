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
    const auto& link = registry.get<comps::LinkModel>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }
    const auto& jointModel = registry.get<comps::JointModel>(link.parentJoint);
    const auto dof = static_cast<Eigen::Index>(jointModel.getDOF());
    for (Eigen::Index d = 0; d < dof; ++d) {
      coordinates.push_back({link.parentJoint, d});
    }
  }
}

} // namespace

//==============================================================================
StepDerivatives contactFreeStepDerivatives(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::Ref<const Eigen::VectorXd>& tau,
    ContactFreeStepCoordinateScratch* coordinateScratch,
    compute::MultibodyInverseDynamicsScratch* inverseDynamicsScratch,
    ContactFreeStepDynamicsTermsScratch* dynamicsTermsScratch,
    ContactFreeStepDerivativeScratch* derivativeScratch)
{
  StepDerivatives derivatives;
  contactFreeStepDerivativesInto(
      registry,
      structure,
      gravity,
      timeStep,
      tau,
      derivatives,
      coordinateScratch,
      inverseDynamicsScratch,
      dynamicsTermsScratch,
      derivativeScratch);
  return derivatives;
}

//==============================================================================
void contactFreeStepDerivativesInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::Ref<const Eigen::VectorXd>& tau,
    StepDerivatives& derivatives,
    ContactFreeStepCoordinateScratch* coordinateScratch,
    compute::MultibodyInverseDynamicsScratch* inverseDynamicsScratch,
    ContactFreeStepDynamicsTermsScratch* dynamicsTermsScratch,
    ContactFreeStepDerivativeScratch* derivativeScratch)
{
  ContactFreeStepDerivativeScratch localDerivativeScratch;
  auto& scratch = derivativeScratch != nullptr ? *derivativeScratch
                                               : localDerivativeScratch;
  derivatives.parameterJacobian.resize(0, 0);

  ContactFreeStepCoordinateScratch localCoordinates;
  auto& coordinates
      = coordinateScratch != nullptr ? *coordinateScratch : localCoordinates;
  collectCoordinatesInto(registry, structure, coordinates);
  const auto ndof = static_cast<Eigen::Index>(coordinates.size());
  if (ndof == 0) {
    derivatives.stateJacobian.resize(0, 0);
    derivatives.controlJacobian.resize(0, 0);
    return;
  }

  // Base-point dynamics terms and current generalized velocity.
  compute::MultibodyDynamicsTerms localBaseTerms;
  compute::MultibodyDynamicsTerms localPlusTerms;
  compute::MultibodyDynamicsTerms localMinusTerms;
  compute::MultibodyDynamicsTermsScratch localDynamicsTermsScratch;
  auto& dynamicsScratch = dynamicsTermsScratch != nullptr
                              ? dynamicsTermsScratch->dynamicsTermsScratch
                              : localDynamicsTermsScratch;
  auto& baseTerms = dynamicsTermsScratch != nullptr
                        ? dynamicsTermsScratch->baseTerms
                        : localBaseTerms;
  auto& plusTerms = dynamicsTermsScratch != nullptr
                        ? dynamicsTermsScratch->plusTerms
                        : localPlusTerms;
  auto& minusTerms = dynamicsTermsScratch != nullptr
                         ? dynamicsTermsScratch->minusTerms
                         : localMinusTerms;

  compute::computeMultibodyDynamicsTermsInto(
      dynamicsScratch, registry, structure, gravity, baseTerms);
  const Eigen::MatrixXd& massMatrix = baseTerms.massMatrix;
  scratch.biasForces.resize(ndof);
  scratch.biasForces = baseTerms.coriolisForces;
  scratch.biasForces += baseTerms.gravityForces; // c = C q̇ + g

  scratch.qdot.resize(ndof);
  scratch.qdot.setZero();
  for (Eigen::Index k = 0; k < ndof; ++k) {
    const auto& coordinate = coordinates[static_cast<std::size_t>(k)];
    const auto& jointState = registry.get<comps::JointState>(coordinate.joint);
    scratch.qdot[k] = jointState.velocity[coordinate.local];
  }

  scratch.massMatrixLdlt.compute(massMatrix);
  scratch.inverseMass.resize(ndof, ndof);
  scratch.inverseMass.setIdentity();
  scratch.massMatrixLdlt.solveInPlace(scratch.inverseMass);
  scratch.rhs.resize(ndof);
  scratch.rhs = tau;
  scratch.rhs -= scratch.biasForces;
  scratch.qddot = scratch.rhs;
  scratch.massMatrixLdlt.solveInPlace(scratch.qddot);

  // Central finite differencing of the dynamics terms. The relative step keeps
  // the perturbation well scaled regardless of the coordinate magnitude.
  const auto evalTermsInto = [&](compute::MultibodyDynamicsTerms& terms)
      -> const compute::MultibodyDynamicsTerms& {
    compute::computeMultibodyDynamicsTermsInto(
        dynamicsScratch, registry, structure, gravity, terms);
    return terms;
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
  scratch.dVelNext_dq.resize(ndof, ndof);
  scratch.dVelNext_dq.setZero();
  scratch.dVelNext_dqdot.resize(ndof, ndof);
  scratch.dVelNext_dqdot.setIdentity();

  compute::InverseDynamicsDerivatives localIdDerivatives;
  auto& idDerivatives = derivativeScratch != nullptr
                            ? scratch.inverseDynamicsDerivatives
                            : localIdDerivatives;
  if (inverseDynamicsScratch != nullptr) {
    compute::computeMultibodyInverseDynamicsDerivativesInto(
        *inverseDynamicsScratch,
        registry,
        structure,
        gravity,
        scratch.qddot,
        idDerivatives);
  } else {
    idDerivatives = compute::computeMultibodyInverseDynamicsDerivatives(
        registry, structure, gravity, scratch.qddot);
  }

  if (idDerivatives.valid) {
    scratch.tempMatrix.resize(ndof, ndof);
    scratch.tempMatrix.noalias() = scratch.inverseMass * idDerivatives.dTau_dq;
    scratch.dVelNext_dq.noalias() = -timeStep * scratch.tempMatrix;
    scratch.tempMatrix.noalias()
        = scratch.inverseMass * idDerivatives.dTau_dqdot;
    scratch.dVelNext_dqdot.setIdentity();
    scratch.dVelNext_dqdot.noalias() -= timeStep * scratch.tempMatrix;
  } else {
    for (Eigen::Index k = 0; k < ndof; ++k) {
      const auto& coordinate = coordinates[static_cast<std::size_t>(k)];
      auto& jointState = registry.get<comps::JointState>(coordinate.joint);

      // --- Position perturbation: dM/dq_k and dc/dq_k. ---
      {
        const double original = jointState.position[coordinate.local];
        const double h = 1e-6 * (1.0 + std::abs(original));

        jointState.position[coordinate.local] = original + h;
        const auto& plus = evalTermsInto(plusTerms);
        jointState.position[coordinate.local] = original - h;
        const auto& minus = evalTermsInto(minusTerms);
        jointState.position[coordinate.local] = original; // restore exactly

        scratch.tempMatrix.resize(ndof, ndof);
        scratch.tempMatrix = plus.massMatrix;
        scratch.tempMatrix -= minus.massMatrix;
        scratch.tempMatrix /= 2.0 * h;

        scratch.dBias.resize(ndof);
        scratch.dBias = plus.coriolisForces;
        scratch.dBias += plus.gravityForces;
        scratch.dBias -= minus.coriolisForces;
        scratch.dBias -= minus.gravityForces;
        scratch.dBias /= 2.0 * h;

        scratch.tempVector.resize(ndof);
        scratch.tempVector.noalias() = scratch.tempMatrix * scratch.qddot;
        scratch.tempVector += scratch.dBias;
        scratch.tempVector2.resize(ndof);
        scratch.tempVector2.noalias()
            = scratch.inverseMass * scratch.tempVector;
        scratch.dVelNext_dq.col(k).noalias() = -timeStep * scratch.tempVector2;
      }

      // --- Velocity perturbation: dc/dq̇_k (the Coriolis velocity derivative).
      // ---
      {
        const double original = jointState.velocity[coordinate.local];
        const double h = 1e-6 * (1.0 + std::abs(original));

        jointState.velocity[coordinate.local] = original + h;
        const auto& plus = evalTermsInto(plusTerms);
        jointState.velocity[coordinate.local] = original - h;
        const auto& minus = evalTermsInto(minusTerms);
        jointState.velocity[coordinate.local] = original; // restore exactly

        scratch.dBias.resize(ndof);
        scratch.dBias = plus.coriolisForces;
        scratch.dBias += plus.gravityForces;
        scratch.dBias -= minus.coriolisForces;
        scratch.dBias -= minus.gravityForces;
        scratch.dBias /= 2.0 * h;

        scratch.tempVector.resize(ndof);
        scratch.tempVector.noalias() = scratch.inverseMass * scratch.dBias;
        scratch.dVelNext_dqdot.col(k).noalias()
            -= timeStep * scratch.tempVector;
      }
    }
  }

  // ∂q̇'/∂τ = Δt * Minv.
  scratch.dVelNext_dtau.resize(ndof, ndof);
  scratch.dVelNext_dtau.noalias() = timeStep * scratch.inverseMass;

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
  scratch.posPartialPos.resize(ndof, ndof);
  scratch.posPartialPos.setIdentity();
  scratch.posPartialVel.resize(ndof, ndof);
  scratch.posPartialVel.setZero();
  scratch.posPartialVel.diagonal().setConstant(timeStep);

  // The next generalized velocity at the base point, ordered by global DOF.
  scratch.nextVelocity = scratch.qdot;
  scratch.nextVelocity.noalias() += timeStep * scratch.qddot;

  // Walk joints in DOF order, filling the manifold blocks for ball/free joints
  // and leaving the Euclidean default in place for the rest.
  Eigen::Index offset = 0;
  while (offset < ndof) {
    const auto& coordinate = coordinates[static_cast<std::size_t>(offset)];
    const auto& jointModel = registry.get<comps::JointModel>(coordinate.joint);
    const auto& jointState = registry.get<comps::JointState>(coordinate.joint);
    // Movable joints in `coordinates` always have a positive DOF count, so this
    // advances by at least one each iteration.
    const auto jointDof = static_cast<Eigen::Index>(jointModel.getDOF());

    if (jointModel.type == comps::JointType::Spherical) {
      // q_local is the rotation vector; q̇'_local is the body angular velocity.
      const Eigen::Vector3d phi = jointState.position.head<3>();
      const Eigen::Vector3d omegaNext = scratch.nextVelocity.segment<3>(offset);
      const Eigen::Vector3d twist = omegaNext * timeStep;
      const Eigen::Vector3d phiNext
          = rotationLog(rotationExp(phi) * rotationExp(twist));

      const Eigen::Matrix3d jrInvNext = rotationRightJacobianInverse(phiNext);
      scratch.posPartialPos.block<3, 3>(offset, offset)
          = jrInvNext * rotationExp(-twist) * rotationRightJacobian(phi);
      scratch.posPartialVel.block<3, 3>(offset, offset)
          = jrInvNext * rotationRightJacobian(twist) * timeStep;
    } else if (jointModel.type == comps::JointType::Floating) {
      // q_local = [translation; rotation vector]; q̇'_local = [linear; angular]
      // body twist. Translation advances in the parent frame (R · v), rotation
      // integrates on SO(3).
      const Eigen::Vector3d theta = jointState.position.tail<3>();
      const Eigen::Matrix3d rotation = rotationExp(theta);
      const Eigen::Vector3d linearNext
          = scratch.nextVelocity.segment<3>(offset);
      const Eigen::Vector3d angularNext
          = scratch.nextVelocity.segment<3>(offset + 3);
      const Eigen::Vector3d twist = angularNext * timeStep;
      const Eigen::Vector3d thetaNext
          = rotationLog(rotation * rotationExp(twist));

      const Eigen::Matrix3d jrTheta = rotationRightJacobian(theta);
      const Eigen::Matrix3d jrInvNext = rotationRightJacobianInverse(thetaNext);

      // ∂Φ/∂q: translation rows depend on translation (I) and on orientation
      // (the coupling block); orientation rows are the SO(3) dexp/dlog block.
      scratch.posPartialPos.block<3, 3>(offset, offset).setIdentity();
      scratch.posPartialPos.block<3, 3>(offset, offset + 3)
          = -rotation * skew(linearNext) * jrTheta * timeStep;
      scratch.posPartialPos.block<3, 3>(offset + 3, offset).setZero();
      scratch.posPartialPos.block<3, 3>(offset + 3, offset + 3)
          = jrInvNext * rotationExp(-twist) * jrTheta;

      // ∂Φ/∂q̇': translation advances by R·Δt; orientation by the SO(3) block.
      scratch.posPartialVel.block<3, 3>(offset, offset) = rotation * timeStep;
      scratch.posPartialVel.block<3, 3>(offset, offset + 3).setZero();
      scratch.posPartialVel.block<3, 3>(offset + 3, offset).setZero();
      scratch.posPartialVel.block<3, 3>(offset + 3, offset + 3)
          = jrInvNext * rotationRightJacobian(twist) * timeStep;
    }

    offset += jointDof;
  }

  scratch.tempMatrix.resize(ndof, ndof);
  scratch.tempMatrix.noalias() = scratch.posPartialVel * scratch.dVelNext_dq;
  scratch.dPosNext_dq.resize(ndof, ndof);
  scratch.dPosNext_dq = scratch.posPartialPos;
  scratch.dPosNext_dq += scratch.tempMatrix;
  scratch.dPosNext_dqdot.resize(ndof, ndof);
  scratch.dPosNext_dqdot.noalias()
      = scratch.posPartialVel * scratch.dVelNext_dqdot;
  scratch.dPosNext_dtau.resize(ndof, ndof);
  scratch.dPosNext_dtau.noalias()
      = scratch.posPartialVel * scratch.dVelNext_dtau;

  // Assemble the state Jacobian [[∂q'/∂q, ∂q'/∂q̇], [∂q̇'/∂q, ∂q̇'/∂q̇]].
  derivatives.stateJacobian.resize(2 * ndof, 2 * ndof);
  derivatives.stateJacobian.setZero();
  derivatives.stateJacobian.topLeftCorner(ndof, ndof) = scratch.dPosNext_dq;
  derivatives.stateJacobian.topRightCorner(ndof, ndof) = scratch.dPosNext_dqdot;
  derivatives.stateJacobian.bottomLeftCorner(ndof, ndof) = scratch.dVelNext_dq;
  derivatives.stateJacobian.bottomRightCorner(ndof, ndof)
      = scratch.dVelNext_dqdot;

  // Assemble the control Jacobian [[∂q'/∂τ], [∂q̇'/∂τ]].
  derivatives.controlJacobian.resize(2 * ndof, ndof);
  derivatives.controlJacobian.topRows(ndof) = scratch.dPosNext_dtau;
  derivatives.controlJacobian.bottomRows(ndof) = scratch.dVelNext_dtau;
}

} // namespace dart::simulation::detail
