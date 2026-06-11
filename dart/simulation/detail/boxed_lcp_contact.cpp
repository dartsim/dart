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
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <limits>
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

//==============================================================================
// Per-contact constraint, mirroring the sequential-impulse stage so the two
// paths assemble the same physics: a normal row plus two tangential friction
// rows spanning the contact plane (box Coulomb model).
struct NormalContact
{
  entt::entity bodyA{entt::null};
  entt::entity bodyB{entt::null};
  bool staticA = false;
  bool staticB = false;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d tangent1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d tangent2 = Eigen::Vector3d::UnitY();
  Eigen::Vector3d armA = Eigen::Vector3d::Zero();
  Eigen::Vector3d armB = Eigen::Vector3d::Zero();
  double bias = 0.0; // Baumgarte/restitution bias as a target normal velocity.
  double friction = 0.0; // Combined Coulomb friction coefficient mu.
};

} // namespace

//==============================================================================
BoxedLcpContactSnapshot solveBoxedLcpContacts(
    detail::WorldRegistry& registry,
    const std::vector<Contact>& contacts,
    double timeStep)
{
  BoxedLcpContactSnapshot snapshot;
  if (contacts.empty()) {
    return snapshot;
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
  std::vector<NormalContact> normals;
  normals.reserve(contacts.size());
  std::unordered_map<entt::entity, std::size_t> bodyColumn;

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

    NormalContact normal;
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
    return snapshot;
  }

  const std::size_t bodyCount = bodyColumn.size();
  const Eigen::Index dofs = static_cast<Eigen::Index>(6 * bodyCount);

  // Stacked inverse mass operator M⁻¹ (block-diagonal, 6 dofs per body) and the
  // free velocity v_free at solve time.
  Eigen::MatrixXd Minv = Eigen::MatrixXd::Zero(dofs, dofs);
  Eigen::VectorXd vFree = Eigen::VectorXd::Zero(dofs);
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
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(rows, dofs);
  const auto fillRow = [&](Eigen::Index row,
                           const NormalContact& contact,
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
  const Eigen::MatrixXd JMinv = J * Minv;
  Eigen::MatrixXd A = JMinv * J.transpose();
  Eigen::VectorXd b = -(J * vFree);
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
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(
      rows, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(rows, -1);
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
  // Degenerate flat contact stacks can hit a non-positive pivot; return early
  // and let the bounded iterative fallback below handle the contact packet.
  options.earlyTermination = true;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(rows);
  const math::LcpProblem problem(A, b, lo, hi, findex);
  const math::LcpResult result = solver.solve(problem, f, options);
  if (!result.succeeded() || !f.allFinite()) {
    math::PgsSolver fallback;
    math::LcpOptions fallbackOptions = math::LcpOptions::realTime();
    fallbackOptions.maxIterations = 120;
    fallbackOptions.relativeTolerance = 1e-6;
    fallbackOptions.validateSolution = false;
    fallbackOptions.warmStart = f.allFinite();
    fallback.solve(problem, f, fallbackOptions);
  }
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
  const Eigen::VectorXd deltaV = Minv * (J.transpose() * f);
  for (const auto& [entity, column] : bodyColumn) {
    const Eigen::Index base = static_cast<Eigen::Index>(6 * column);
    auto& velocity = registry.get<comps::Velocity>(entity);
    velocity.linear += deltaV.segment<3>(base);
    velocity.angular += deltaV.segment<3>(base + 3);
  }

  snapshot.A = std::move(A);
  snapshot.b = std::move(b);
  snapshot.lo = std::move(lo);
  snapshot.hi = std::move(hi);
  snapshot.findex = std::move(findex);
  snapshot.f = std::move(f);
  snapshot.J = std::move(J);
  snapshot.bodyCount = bodyCount;
  snapshot.contactCount = static_cast<std::size_t>(n);
  return snapshot;
}

} // namespace dart::simulation::detail
