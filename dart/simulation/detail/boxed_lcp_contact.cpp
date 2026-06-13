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

#include "dart/simulation/detail/boxed_lcp_contact.hpp"

#include "dart/simulation/body/collision_body.hpp"
#include "dart/simulation/body/contact.hpp"
#include "dart/simulation/comps/contact_material.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"

#include <dart/math/lcp/lcp_solver.hpp>
#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/lcp_utils.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

#include <cmath>

namespace dart::simulation::detail {

namespace {

//==============================================================================
double inverseMassOf(const comps::MassProperties& mass)
{
  return (mass.mass > 0.0 && std::isfinite(mass.mass)) ? 1.0 / mass.mass : 0.0;
}

//==============================================================================
Eigen::Quaterniond normalizeOrIdentity(const Eigen::Quaterniond& orientation)
{
  const double norm = orientation.norm();
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(orientation.coeffs() / norm);
}

//==============================================================================
Eigen::Matrix3d inverseWorldInertiaOf(
    const comps::MassProperties& mass, const comps::Transform& transform)
{
  if (!(mass.mass > 0.0) || !std::isfinite(mass.mass)) {
    return Eigen::Matrix3d::Zero();
  }
  const Eigen::Matrix3d rotation
      = normalizeOrIdentity(transform.orientation).toRotationMatrix();
  const Eigen::Matrix3d worldInertia
      = rotation * mass.inertia * rotation.transpose();
  Eigen::LDLT<Eigen::Matrix3d> solver(worldInertia);
  if (solver.info() != Eigen::Success || !solver.isPositive()) {
    return Eigen::Matrix3d::Zero();
  }
  return solver.solve(Eigen::Matrix3d::Identity());
}

//==============================================================================
double restitutionOf(const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->restitution;
  }
  return 0.0;
}

//==============================================================================
double frictionOf(const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->friction;
  }
  return 1.0;
}

//==============================================================================
bool hasPrescribedRigidBodyContactResponse(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

} // namespace

//==============================================================================
void BoxedLcpContactScratch::reserve(
    std::size_t contactCapacity, std::size_t bodyCapacity)
{
  normals.reserve(contactCapacity);
  bodyColumn.reserve(bodyCapacity);

  const Eigen::Index rows = static_cast<Eigen::Index>(3u * contactCapacity);
  const Eigen::Index dofs = static_cast<Eigen::Index>(6u * bodyCapacity);
  if (rows <= 0) {
    return;
  }

  snapshot.A.resize(rows, rows);
  snapshot.b.resize(rows);
  snapshot.lo.resize(rows);
  snapshot.hi.resize(rows);
  snapshot.findex.resize(rows);
  snapshot.f.resize(rows);
  snapshot.J.resize(rows, dofs);
  Minv.resize(dofs, dofs);
  vFree.resize(dofs);
  JMinv.resize(rows, dofs);
  jtImpulse.resize(dofs);
  deltaV.resize(dofs);

  const auto vectorSize = static_cast<std::size_t>(rows);
  const auto nSkip
      = static_cast<std::size_t>(math::padding(static_cast<int>(rows)));
  const auto matrixSize = vectorSize * nSkip;
  dantzig.Adata.reserve(matrixSize);
  dantzig.xdata.reserve(vectorSize);
  dantzig.wdata.reserve(vectorSize);
  dantzig.bdata.reserve(vectorSize);
  dantzig.loData.reserve(vectorSize);
  dantzig.hiData.reserve(vectorSize);
  dantzig.findexData.reserve(vectorSize);
  dantzig.w.resize(rows);
  dantzig.loEff.resize(rows);
  dantzig.hiEff.resize(rows);
  dantzig.lcp.L.reserve(matrixSize);
  dantzig.lcp.d.reserve(vectorSize);
  dantzig.lcp.w.reserve(vectorSize);
  dantzig.lcp.deltaW.reserve(vectorSize);
  dantzig.lcp.deltaX.reserve(vectorSize);
  dantzig.lcp.dell.reserve(vectorSize);
  dantzig.lcp.ell.reserve(vectorSize);
  dantzig.lcp.p.reserve(vectorSize);
  dantzig.lcp.C.reserve(vectorSize);
  dantzig.lcp.rowPointers.reserve(vectorSize);
  dantzig.lcp.reserveState(vectorSize);
}

//==============================================================================
void BoxedLcpContactScratch::clearProblem()
{
  normals.clear();
  bodyColumn.clear();
  snapshot.A.resize(0, 0);
  snapshot.b.resize(0);
  snapshot.lo.resize(0);
  snapshot.hi.resize(0);
  snapshot.findex.resize(0);
  snapshot.f.resize(0);
  snapshot.J.resize(0, 0);
  snapshot.bodyCount = 0;
  snapshot.contactCount = 0;
}

//==============================================================================
void reserveBoxedLcpContactScratch(
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    BoxedLcpContactScratch& scratch)
{
  if (contacts.empty()) {
    return;
  }

  scratch.normals.reserve(contacts.size());
  scratch.bodyColumn.clear();
  scratch.bodyColumn.reserve(2u * contacts.size());
  const auto registerBody = [&](entt::entity entity, bool isStatic) {
    if (!isStatic) {
      scratch.bodyColumn.emplace(entity, scratch.bodyColumn.size());
    }
  };

  std::size_t activeContacts = 0;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const bool staticA
        = hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = hasPrescribedRigidBodyContactResponse(registry, entityB);
    if (staticA && staticB) {
      continue;
    }

    ++activeContacts;
    registerBody(entityA, staticA);
    registerBody(entityB, staticB);
  }

  scratch.reserve(activeContacts, scratch.bodyColumn.size());
  scratch.bodyColumn.clear();
}

//==============================================================================
BoxedLcpContactSnapshot solveBoxedLcpContacts(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    double timeStep)
{
  BoxedLcpContactScratch scratch;
  BoxedLcpContactSnapshot& snapshot
      = solveBoxedLcpContacts(registry, contacts, timeStep, scratch);
  return std::move(snapshot);
}

//==============================================================================
BoxedLcpContactSnapshot& solveBoxedLcpContacts(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    double timeStep,
    BoxedLcpContactScratch& scratch)
{
  if (contacts.empty()) {
    scratch.clearProblem();
    return scratch.snapshot;
  }

  // Velocity-level bias combines restitution with a Baumgarte-style
  // penetration recovery for slow resting contacts. High-speed inelastic
  // impacts stay restitution-only so penetration bias does not inject extra
  // rebound velocity. The positional projection step still removes any
  // residual penetration after the impulse solve; the slow-contact velocity
  // bias prevents taller coupled stacks from accumulating downward drift
  // between projections.
  constexpr double restitutionThreshold = 1e-3;
  constexpr double penetrationSlop = 1e-4;
  constexpr double baumgarteFactor = 0.2;
  constexpr double baumgarteApproachThreshold = -0.25;

  // Collect rigid-body/rigid-body normal contacts and the dynamic bodies they
  // touch. Each dynamic body owns a 6-column block in J ([v; ω]).
  auto& normals = scratch.normals;
  auto& bodyColumn = scratch.bodyColumn;
  normals.clear();
  normals.reserve(contacts.size());
  bodyColumn.clear();
  bodyColumn.reserve(2u * contacts.size());

  const auto registerBody = [&](entt::entity entity, bool isStatic) {
    if (isStatic) {
      return;
    }
    bodyColumn.emplace(entity, bodyColumn.size());
  };

  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

    // Rigid-body pairs only; articulated-link contacts are out of scope for
    // this slice (handled by the multibody solve).
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const bool kinematicA = registry.all_of<comps::KinematicBodyTag>(entityA);
    const bool kinematicB = registry.all_of<comps::KinematicBodyTag>(entityB);
    const bool staticA
        = hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = hasPrescribedRigidBodyContactResponse(registry, entityB);
    if (staticA && staticB) {
      continue;
    }

    const auto& transformA = registry.get<comps::Transform>(entityA);
    const auto& transformB = registry.get<comps::Transform>(entityB);

    BoxedLcpContactNormal normal;
    normal.bodyA = entityA;
    normal.bodyB = entityB;
    normal.staticA = staticA;
    normal.staticB = staticB;
    normal.normal = contact.normal;
    // Orthonormal tangent basis spanning the contact plane, matching the
    // sequential-impulse stage's convention so the box Coulomb model is shared.
    normal.tangent1 = contact.normal.unitOrthogonal();
    normal.tangent2 = contact.normal.cross(normal.tangent1);
    normal.friction = std::sqrt(
        frictionOf(registry, entityA) * frictionOf(registry, entityB));
    normal.armA = contact.point - transformA.position;
    normal.armB = contact.point - transformB.position;

    // Restitution target from the pre-solve approach velocity, combining the
    // two materials by the larger bounce (parity with the existing stage).
    const auto& velocityA = registry.get<comps::Velocity>(entityA);
    const auto& velocityB = registry.get<comps::Velocity>(entityB);
    Eigen::Vector3d pointVelocityA = Eigen::Vector3d::Zero();
    if (!staticA) {
      pointVelocityA = velocityA.linear + velocityA.angular.cross(normal.armA);
    }
    Eigen::Vector3d pointVelocityB = Eigen::Vector3d::Zero();
    if (!staticB) {
      pointVelocityB = velocityB.linear + velocityB.angular.cross(normal.armB);
    }
    const double approach
        = (pointVelocityB - pointVelocityA).dot(normal.normal);

    const double restitution = std::max(
        restitutionOf(registry, entityA), restitutionOf(registry, entityB));
    const double restitutionVelocity
        = (restitution > 0.0 && approach < -restitutionThreshold)
              ? -restitution * approach
              : 0.0;
    // Kinematic bodies follow the sequential-impulse compatibility path here:
    // their prescribed motion is ignored by the LCP contact solve, and initial
    // penetration does not inject a velocity-level push from the kinematic
    // body.
    const double baumgarteVelocity
        = (timeStep > 0.0 && !kinematicA && !kinematicB
           && approach > baumgarteApproachThreshold)
              ? baumgarteFactor * std::max(0.0, contact.depth - penetrationSlop)
                    / timeStep
              : 0.0;

    normal.bias = std::max(restitutionVelocity, baumgarteVelocity);

    registerBody(entityA, staticA);
    registerBody(entityB, staticB);
    normals.push_back(normal);
  }

  const Eigen::Index n = static_cast<Eigen::Index>(normals.size());
  if (n == 0) {
    scratch.clearProblem();
    return scratch.snapshot;
  }

  const std::size_t bodyCount = bodyColumn.size();
  const Eigen::Index dofs = static_cast<Eigen::Index>(6 * bodyCount);
  scratch.reserve(normals.size(), bodyCount);
  auto& snapshot = scratch.snapshot;

  // Stacked inverse mass operator M⁻¹ (block-diagonal, 6 dofs per body) and the
  // free velocity v_free at solve time.
  auto& Minv = scratch.Minv;
  auto& vFree = scratch.vFree;
  Minv.resize(dofs, dofs);
  Minv.setZero();
  vFree.resize(dofs);
  vFree.setZero();
  for (const auto& [entity, column] : bodyColumn) {
    const auto& mass = registry.get<comps::MassProperties>(entity);
    const auto& transform = registry.get<comps::Transform>(entity);
    const auto& velocity = registry.get<comps::Velocity>(entity);
    const Eigen::Index base = static_cast<Eigen::Index>(6 * column);

    const double invMass = inverseMassOf(mass);
    Minv.block<3, 3>(base, base) = invMass * Eigen::Matrix3d::Identity();
    Minv.block<3, 3>(base + 3, base + 3)
        = inverseWorldInertiaOf(mass, transform);
    vFree.segment<3>(base) = velocity.linear;
    vFree.segment<3>(base + 3) = velocity.angular;
  }

  // Row layout: the first n rows are normal rows; the following 2*n rows are
  // friction rows (two tangents per contact). Friction row 2*i references
  // normal row i through findex, so the boxed solver applies the ±mu·f_normal
  // Coulomb box automatically (per the lcp_validation findex convention: the
  // effective bound of a friction row is ±|hi| * |x[findex]|).
  const Eigen::Index rows = n + 2 * n;

  // Stacked contact Jacobian J. A normal row maps the stacked body twists to
  // the normal-direction relative velocity; a friction row maps them to the
  // corresponding tangential relative velocity:
  //   J_row v = (v_B + ω_B × armB - v_A - ω_A × armA) · d   (d =
  //   normal/tangent).
  auto& J = snapshot.J;
  J.resize(rows, dofs);
  J.setZero();
  const auto fillRow = [&](Eigen::Index row,
                           const BoxedLcpContactNormal& contact,
                           const Eigen::Vector3d& direction) {
    if (!contact.staticB) {
      const Eigen::Index base
          = static_cast<Eigen::Index>(6 * bodyColumn.at(contact.bodyB));
      J.block<1, 3>(row, base) += direction.transpose();
      J.block<1, 3>(row, base + 3) += contact.armB.cross(direction).transpose();
    }
    if (!contact.staticA) {
      const Eigen::Index base
          = static_cast<Eigen::Index>(6 * bodyColumn.at(contact.bodyA));
      J.block<1, 3>(row, base) -= direction.transpose();
      J.block<1, 3>(row, base + 3) -= contact.armA.cross(direction).transpose();
    }
  };
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto& normal = normals[static_cast<std::size_t>(i)];
    fillRow(i, normal, normal.normal);
    fillRow(n + 2 * i, normal, normal.tangent1);
    fillRow(n + 2 * i + 1, normal, normal.tangent2);
  }

  // Delassus system: A = J M⁻¹ Jᵀ.
  // LCP convention: w = A f - b, with the boxed bounds/findex coupling and f ⟂
  // w. For a non-penetrating normal contact post-impulse normal velocity must
  // equal the bias target (restitution ≥ 0). Setting b = -(J v_free) + bias
  // gives w = A f + J v_free - bias, and at an active normal row (f > 0) w = 0
  // → post-approach = bias. Friction rows target zero tangential relative
  // velocity (b = -(J v_free), no bias) and are coupled to their normal row
  // through findex with lo = -mu, hi = +mu.
  auto& JMinv = scratch.JMinv;
  JMinv.resize(rows, dofs);
  JMinv.noalias() = J * Minv;
  auto& A = snapshot.A;
  A.resize(rows, rows);
  A.noalias() = JMinv * J.transpose();
  auto& b = snapshot.b;
  b.resize(rows);
  b.noalias() = J * vFree;
  b = -b;
  for (Eigen::Index i = 0; i < n; ++i) {
    b[i] += normals[static_cast<std::size_t>(i)].bias;
  }

  // Constraint-force-mixing regularization on the FRICTION-row diagonal,
  // matching the legacy contact-constraint path (DART_CFM = 1e-5,
  // A(i,i) += A(i,i)*cfm). A box resting flat on a plane yields rank-deficient
  // friction rows (identical tangent effective masses, contact arm along the
  // normal), which makes the pivoting Dantzig solver hit a degenerate (s <= 0)
  // pivot. The CFM keeps the friction block away from singular without
  // perceptibly altering the physics. It is applied only to friction rows so
  // the frictionless normal-only behavior stays bit-for-bit identical to the
  // pre-friction path.
  constexpr double kConstraintForceMixing = 1e-5;
  for (Eigen::Index row = n; row < rows; ++row) {
    A(row, row) += A(row, row) * kConstraintForceMixing;
  }

  // Normal rows: lo = 0, hi = +∞, findex = -1 (push-only, no coupling).
  // Friction rows: lo = -mu, hi = +mu, findex = owning normal row.
  auto& lo = snapshot.lo;
  auto& hi = snapshot.hi;
  auto& findex = snapshot.findex;
  lo.resize(rows);
  lo.setZero();
  hi.resize(rows);
  hi.setConstant(std::numeric_limits<double>::infinity());
  findex.resize(rows);
  findex.setConstant(-1);
  for (Eigen::Index i = 0; i < n; ++i) {
    const double mu = normals[static_cast<std::size_t>(i)].friction;
    for (Eigen::Index t = 0; t < 2; ++t) {
      const Eigen::Index row = n + 2 * i + t;
      lo[row] = -mu;
      hi[row] = mu;
      findex[row] = static_cast<int>(i);
    }
  }

  // Solve with the pivoting Dantzig boxed-LCP solver.
  math::DantzigSolver solver;
  math::LcpOptions options;
  options.warmStart = false;
  options.validateSolution = false;
  auto& f = snapshot.f;
  f.resize(rows);
  f.setZero();
  solver.solve(A, b, lo, hi, findex, f, scratch.dantzig, options);
  for (Eigen::Index i = 0; i < n; ++i) {
    // Normal impulses are push-only; sanitize non-finite/negative values.
    if (!std::isfinite(f[i]) || f[i] < 0.0) {
      f[i] = 0.0;
    }
  }
  for (Eigen::Index row = n; row < rows; ++row) {
    // Friction impulses are signed; only sanitize non-finite values.
    if (!std::isfinite(f[row])) {
      f[row] = 0.0;
    }
  }

  // Apply Δv = M⁻¹ Jᵀ f to the dynamic body velocities.
  auto& jtImpulse = scratch.jtImpulse;
  auto& deltaV = scratch.deltaV;
  jtImpulse.resize(dofs);
  jtImpulse.noalias() = J.transpose() * f;
  deltaV.resize(dofs);
  deltaV.noalias() = Minv * jtImpulse;
  for (const auto& [entity, column] : bodyColumn) {
    const Eigen::Index base = static_cast<Eigen::Index>(6 * column);
    auto& velocity = registry.get<comps::Velocity>(entity);
    velocity.linear += deltaV.segment<3>(base);
    velocity.angular += deltaV.segment<3>(base + 3);
  }

  snapshot.bodyCount = bodyCount;
  snapshot.contactCount = static_cast<std::size_t>(n);
  return snapshot;
}

} // namespace dart::simulation::detail
