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

#include "dart/simulation/compute/rigid_body_constraint.hpp"

#include "dart/simulation/comps/contact_material.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/rigid_contact/rigid_contact_assembly.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>

#include <cmath>

namespace dart::simulation::compute {
namespace {

//==============================================================================
struct ContactEnd
{
  entt::entity entity;
  double sign;
  double invMass;
  const Eigen::Matrix3d* invInertia;
  const Eigen::Vector3d* arm;
};

//==============================================================================
std::array<ContactEnd, 2> endsOf(const RigidBodyContactConstraint& c)
{
  return std::array<ContactEnd, 2>{
      ContactEnd{c.bodyA, -1.0, c.invMassA, &c.invInertiaA, &c.armA},
      ContactEnd{c.bodyB, +1.0, c.invMassB, &c.invInertiaB, &c.armB}};
}

//==============================================================================
const Eigen::Vector3d& directionOf(const RigidBodyContactConstraint& c, int row)
{
  return row == 0 ? c.normal : (row == 1 ? c.tangent1 : c.tangent2);
}

//==============================================================================
Eigen::Index contactRowIndex(
    RigidBodyContactRowLayout layout,
    std::size_t contactIndex,
    int row,
    std::size_t contactCount)
{
  const Eigen::Index i = static_cast<Eigen::Index>(contactIndex);
  switch (layout) {
    case RigidBodyContactRowLayout::ContactMajor:
      return i * RigidBodyContactProblem::kRowsPerContact + row;
    case RigidBodyContactRowLayout::NormalThenTangents:
      if (row == 0) {
        return i;
      }
      return static_cast<Eigen::Index>(contactCount) + 2 * i + (row - 1);
  }
  return i * RigidBodyContactProblem::kRowsPerContact + row;
}

//==============================================================================
std::optional<std::size_t> dynamicBodyColumn(
    const std::vector<entt::entity, RigidBodyContactProblem::EntityAllocator>&
        dynamicBodies,
    entt::entity entity)
{
  const auto it = std::find(dynamicBodies.begin(), dynamicBodies.end(), entity);
  if (it == dynamicBodies.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::distance(dynamicBodies.begin(), it));
}

//==============================================================================
void registerDynamicBody(
    std::vector<entt::entity, RigidBodyContactProblem::EntityAllocator>&
        dynamicBodies,
    entt::entity entity,
    bool isStatic)
{
  if (isStatic || dynamicBodyColumn(dynamicBodies, entity).has_value()) {
    return;
  }
  dynamicBodies.push_back(entity);
}

//==============================================================================
double delassusEntry(
    const std::array<ContactEnd, 2>& endsI,
    const Eigen::Vector3d& dirI,
    const std::array<ContactEnd, 2>& endsJ,
    const Eigen::Vector3d& dirJ)
{
  double entry = 0.0;
  for (const auto& ei : endsI) {
    if (ei.invMass == 0.0) {
      continue; // Static side: no contribution.
    }
    for (const auto& ej : endsJ) {
      if (ei.entity != ej.entity) {
        continue; // Bodies not shared between the two contacts.
      }
      entry += ei.sign * ej.sign
               * (ei.invMass * dirI.dot(dirJ)
                  + dirI.dot(
                      ((*ei.invInertia) * ej.arm->cross(dirJ)).cross(*ei.arm)));
    }
  }
  return entry;
}

} // namespace

//==============================================================================
Eigen::Vector3d computeRigidBodyContactPointVelocity(
    const comps::Velocity& velocity, const Eigen::Vector3d& arm, bool isStatic)
{
  if (isStatic) {
    return Eigen::Vector3d::Zero();
  }
  return velocity.linear + velocity.angular.cross(arm);
}

//==============================================================================
RigidBodyContactProblem assembleRigidBodyContactProblem(
    const detail::WorldRegistry& registry, std::span<const Contact> contacts)
{
  return assembleRigidBodyContactProblem(
      registry, contacts, RigidBodyContactAssemblyOptions{});
}

//==============================================================================
RigidBodyContactProblem assembleRigidBodyContactProblem(
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const RigidBodyContactAssemblyOptions& options)
{
  RigidBodyContactProblem problem;
  assembleRigidBodyContactProblemInto(problem, registry, contacts, options);
  return problem;
}

//==============================================================================
void assembleRigidBodyContactProblemInto(
    RigidBodyContactProblem& problem,
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts)
{
  assembleRigidBodyContactProblemInto(
      problem, registry, contacts, RigidBodyContactAssemblyOptions{});
}

//==============================================================================
void assembleRigidBodyContactProblemInto(
    RigidBodyContactProblem& problem,
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const RigidBodyContactAssemblyOptions& options)
{
  problem.constraints.clear();
  problem.dynamicBodies.clear();

  std::size_t rigidContactCount = 0;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    if (registry.all_of<comps::RigidBodyTag>(entityA)
        && registry.all_of<comps::RigidBodyTag>(entityB)) {
      ++rigidContactCount;
    }
  }
  problem.constraints.reserve(rigidContactCount);

  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

    // This assembly handles rigid-body pairs only; contacts involving
    // multibody links are resolved by the articulated contact solve for now.
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const auto& transformA = registry.get<comps::Transform>(entityA);
    const auto& transformB = registry.get<comps::Transform>(entityB);
    const auto& massA = registry.get<comps::MassProperties>(entityA);
    const auto& massB = registry.get<comps::MassProperties>(entityB);

    const bool prescribedA
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool prescribedB
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityB);

    RigidBodyContactConstraint constraint;
    constraint.bodyA = entityA;
    constraint.bodyB = entityB;
    constraint.normal = contact.normal;
    constraint.depth = contact.depth;
    constraint.armA = contact.point - transformA.position;
    constraint.armB = contact.point - transformB.position;
    constraint.staticA = prescribedA;
    constraint.staticB = prescribedB;
    constraint.invMassA = prescribedA ? 0.0 : detail::inverseMassOf(massA);
    constraint.invMassB = prescribedB ? 0.0 : detail::inverseMassOf(massB);
    constraint.invInertiaA
        = prescribedA ? Eigen::Matrix3d::Zero()
                      : detail::inverseWorldInertiaOf(massA, transformA);
    constraint.invInertiaB
        = prescribedB ? Eigen::Matrix3d::Zero()
                      : detail::inverseWorldInertiaOf(massB, transformB);

    const Eigen::Vector3d crossA = constraint.armA.cross(constraint.normal);
    const Eigen::Vector3d crossB = constraint.armB.cross(constraint.normal);
    const double angular
        = constraint.normal.dot(
              (constraint.invInertiaA * crossA).cross(constraint.armA))
          + constraint.normal.dot(
              (constraint.invInertiaB * crossB).cross(constraint.armB));
    constraint.effectiveMass
        = constraint.invMassA + constraint.invMassB + angular;
    if (constraint.effectiveMass <= 0.0) {
      continue; // Both bodies are static.
    }

    // Restitution target from the pre-solve approach velocity. Combine the two
    // materials by taking the larger bounce.
    constraint.restitution = std::max(
        detail::restitutionOf(registry, entityA),
        detail::restitutionOf(registry, entityB));
    const auto& velocityA = registry.get<comps::Velocity>(entityA);
    const auto& velocityB = registry.get<comps::Velocity>(entityB);
    const double initialApproach
        = (computeRigidBodyContactPointVelocity(
               velocityB, constraint.armB, constraint.staticB)
           - computeRigidBodyContactPointVelocity(
               velocityA, constraint.armA, constraint.staticA))
              .dot(constraint.normal);
    constexpr double restitutionThreshold
        = detail::kRigidContactRestitutionThreshold;
    constraint.restitutionVelocity
        = (constraint.restitution > 0.0
           && initialApproach < -restitutionThreshold)
              ? -constraint.restitution * initialApproach
              : 0.0;

    // Two tangent directions spanning the contact plane, plus their effective
    // masses, for a friction-pyramid (box) Coulomb model.
    constraint.tangent1 = constraint.normal.unitOrthogonal();
    constraint.tangent2 = constraint.normal.cross(constraint.tangent1);
    const auto tangentMass = [&](const Eigen::Vector3d& tangent) {
      const Eigen::Vector3d crossTangentA = constraint.armA.cross(tangent);
      const Eigen::Vector3d crossTangentB = constraint.armB.cross(tangent);
      return constraint.invMassA + constraint.invMassB
             + tangent.dot((constraint.invInertiaA * crossTangentA)
                               .cross(constraint.armA))
             + tangent.dot((constraint.invInertiaB * crossTangentB)
                               .cross(constraint.armB));
    };
    constraint.tangentMass1 = tangentMass(constraint.tangent1);
    constraint.tangentMass2 = tangentMass(constraint.tangent2);
    constraint.friction = std::sqrt(
        detail::frictionOf(registry, entityA)
        * detail::frictionOf(registry, entityB));
    constraint.normalVelocityBias = constraint.restitutionVelocity;
    if (options.includeBaumgarteBias && options.timeStep > 0.0
        && initialApproach > detail::kRigidContactBaumgarteApproachThreshold
        && !registry.all_of<comps::KinematicBodyTag>(entityA)
        && !registry.all_of<comps::KinematicBodyTag>(entityB)) {
      const double penetration = std::max(
          0.0, contact.depth - detail::kRigidContactPositionAllowance);
      const double baumgarteVelocity
          = detail::kRigidContactPositionCorrectionFactor * penetration
            / options.timeStep;
      constraint.normalVelocityBias
          = std::max(constraint.normalVelocityBias, baumgarteVelocity);
    }

    problem.constraints.push_back(constraint);
    registerDynamicBody(problem.dynamicBodies, entityA, constraint.staticA);
    registerDynamicBody(problem.dynamicBodies, entityB, constraint.staticB);
  }

  const std::size_t n = problem.constraints.size();
  const auto size
      = static_cast<Eigen::Index>(n * RigidBodyContactProblem::kRowsPerContact);

  // Each optional output is handled independently: cleared when its flag is
  // disabled (so a caller that reuses the problem and toggles only one flag can
  // never observe stale shape/content from a previous assembly), and resized in
  // place when enabled (so a stable contact shape reuses the existing buffers --
  // Eigen reallocates only when the requested size differs). resize(0, ...) is a
  // no-op when already empty, so repeated same-shape steps stay malloc-free.
  if (options.populateSystem) {
    // The Delassus block is fully overwritten below, but it is zeroed in place
    // to preserve the previous Eigen::MatrixXd::Zero(size, size) init exactly.
    problem.delassus.resize(size, size);
    problem.delassus.setZero();
    problem.rhs.resize(size);
    problem.lo.resize(size);
    problem.hi.resize(size);
    problem.findex.resize(size);
  } else {
    problem.delassus.resize(0, 0);
    problem.rhs.resize(0);
    problem.lo.resize(0);
    problem.hi.resize(0);
    problem.findex.resize(0);
  }
  if (options.populateJacobian) {
    // The Jacobian rows are accumulated with +=, so it must be zeroed; resize
    // in place first to keep a stable shape on the allocator-backed buffer.
    problem.jacobian.resize(
        size, static_cast<Eigen::Index>(6 * problem.dynamicBodies.size()));
    problem.jacobian.setZero();
  } else {
    problem.jacobian.resize(0, 0);
  }

  if (!options.populateSystem && !options.populateJacobian) {
    return;
  }

  for (std::size_t i = 0; i < n; ++i) {
    const auto& ci = problem.constraints[i];
    const auto endsI = endsOf(ci);
    const Eigen::Index normalRow = contactRowIndex(options.rowLayout, i, 0, n);

    // Relative contact velocity before any impulse is applied this step.
    const auto& velocityA = registry.get<comps::Velocity>(ci.bodyA);
    const auto& velocityB = registry.get<comps::Velocity>(ci.bodyB);
    const Eigen::Vector3d relativeVelocity
        = computeRigidBodyContactPointVelocity(velocityB, ci.armB, ci.staticB)
          - computeRigidBodyContactPointVelocity(
              velocityA, ci.armA, ci.staticA);

    // Normal row: drive the approach velocity to the restitution target.
    if (options.populateSystem) {
      problem.rhs[normalRow]
          = ci.normalVelocityBias - relativeVelocity.dot(ci.normal);
      problem.lo[normalRow] = 0.0;
      problem.hi[normalRow] = std::numeric_limits<double>::infinity();
      problem.findex[normalRow] = -1;
    }

    // Friction rows: drive each tangential velocity to zero, bounded by the
    // Coulomb cone. `hi` carries the friction coefficient mu; the solver scales
    // it by the solved normal impulse referenced through `findex`.
    for (int t = 1; t < RigidBodyContactProblem::kRowsPerContact; ++t) {
      if (options.populateSystem) {
        const Eigen::Index row = contactRowIndex(options.rowLayout, i, t, n);
        problem.rhs[row] = -relativeVelocity.dot(directionOf(ci, t));
        problem.lo[row] = -ci.friction;
        problem.hi[row] = ci.friction;
        problem.findex[row] = static_cast<int>(normalRow);
      }
    }

    if (options.populateJacobian) {
      const auto fillJacobianRow = [&](Eigen::Index row,
                                       const Eigen::Vector3d& direction) {
        if (!ci.staticB) {
          const auto column
              = dynamicBodyColumn(problem.dynamicBodies, ci.bodyB);
          if (column.has_value()) {
            const Eigen::Index base = static_cast<Eigen::Index>(6 * *column);
            problem.jacobian.block<1, 3>(row, base) += direction.transpose();
            problem.jacobian.block<1, 3>(row, base + 3)
                += ci.armB.cross(direction).transpose();
          }
        }
        if (!ci.staticA) {
          const auto column
              = dynamicBodyColumn(problem.dynamicBodies, ci.bodyA);
          if (column.has_value()) {
            const Eigen::Index base = static_cast<Eigen::Index>(6 * *column);
            problem.jacobian.block<1, 3>(row, base) -= direction.transpose();
            problem.jacobian.block<1, 3>(row, base + 3)
                -= ci.armA.cross(direction).transpose();
          }
        }
      };
      for (int a = 0; a < RigidBodyContactProblem::kRowsPerContact; ++a) {
        fillJacobianRow(
            contactRowIndex(options.rowLayout, i, a, n), directionOf(ci, a));
      }
    }

    if (options.populateSystem) {
      // Fill the 3x3 Delassus block of contact i against every contact j.
      for (std::size_t j = 0; j < n; ++j) {
        const auto& cj = problem.constraints[j];
        const auto endsJ = endsOf(cj);
        for (int a = 0; a < RigidBodyContactProblem::kRowsPerContact; ++a) {
          const Eigen::Index row = contactRowIndex(options.rowLayout, i, a, n);
          for (int b = 0; b < RigidBodyContactProblem::kRowsPerContact; ++b) {
            const Eigen::Index col
                = contactRowIndex(options.rowLayout, j, b, n);
            problem.delassus(row, col) = delassusEntry(
                endsI, directionOf(ci, a), endsJ, directionOf(cj, b));
          }
        }
      }
    }
  }

  if (options.populateSystem && options.regularizeFrictionRows) {
    for (std::size_t i = 0; i < n; ++i) {
      for (int t = 1; t < RigidBodyContactProblem::kRowsPerContact; ++t) {
        const Eigen::Index row = contactRowIndex(options.rowLayout, i, t, n);
        problem.delassus(row, row)
            += problem.delassus(row, row) * detail::kRigidContactFrictionCfm;
      }
    }
  }
}

//==============================================================================
void applyRigidBodyContactImpulse(
    detail::WorldRegistry& registry,
    const RigidBodyContactConstraint& constraint,
    const Eigen::Vector3d& impulse)
{
  auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
  auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
  velocityB.linear += constraint.invMassB * impulse;
  velocityB.angular += constraint.invInertiaB * constraint.armB.cross(impulse);
  velocityA.linear -= constraint.invMassA * impulse;
  velocityA.angular -= constraint.invInertiaA * constraint.armA.cross(impulse);
}

} // namespace dart::simulation::compute
