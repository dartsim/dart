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

#include <dart/simulation/comps/contact_material.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <cmath>

namespace dart::simulation::detail {

// PLAN-091 WP-091.13 slice A: the rigid-rigid contact-assembly helpers and
// constants shared by every contact path -- the sequential-impulse stage, the
// boxed-LCP path, the canonical constraint assembly, and the differentiable
// contact capture. These were duplicated verbatim across those translation
// units; single-sourcing them keeps the paths assembling identical physics and
// gives the convergence onto one assembly producer (the rest of WP-091.13) a
// shared base to build on.

/// Restitution is applied only when the closing speed exceeds this threshold
/// (m/s), so resting contacts do not bounce.
inline constexpr double kRigidContactRestitutionThreshold = 1e-3;

/// Constraint-force-mixing added to the friction-row Delassus diagonal so a
/// flat resting contact's rank-deficient friction block stays solvable; the
/// differentiable capture mirrors it to match the forward boxed-LCP step.
inline constexpr double kRigidContactFrictionCfm = 1e-5;

/// Penetration depth below this allowance is left unprojected (meters).
inline constexpr double kRigidContactPositionAllowance = 1e-4;

/// Fraction of penetration beyond the allowance removed by position projection.
inline constexpr double kRigidContactPositionCorrectionFactor = 0.2;

/// Slow-contact velocity threshold above which Baumgarte penetration bias is
/// allowed to assist the boxed-LCP solve (m/s).
inline constexpr double kRigidContactBaumgarteApproachThreshold = -0.25;

/// Reciprocal of a finite, positive body mass; zero for static/degenerate mass.
inline double inverseMassOf(const comps::MassProperties& mass)
{
  return (mass.mass > 0.0 && std::isfinite(mass.mass)) ? 1.0 / mass.mass : 0.0;
}

/// A unit quaternion, falling back to identity for a zero or non-finite input.
inline Eigen::Quaterniond normalizeOrIdentity(
    const Eigen::Quaterniond& orientation)
{
  const double norm = orientation.norm();
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(orientation.coeffs() / norm);
}

/// World-frame inverse inertia; zero for static/degenerate or non-PD inertia.
inline Eigen::Matrix3d inverseWorldInertiaOf(
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

/// Restitution coefficient for a body; 0 when it carries no contact material.
inline double restitutionOf(const WorldRegistry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->restitution;
  }
  return 0.0;
}

/// Friction coefficient for a body; 1 when it carries no contact material.
inline double frictionOf(const WorldRegistry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->friction;
  }
  return 1.0;
}

/// Whether a body's contact response is prescribed (static or kinematic), so
/// the contact solve must not push it.
inline bool hasPrescribedRigidBodyContactResponse(
    const WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

} // namespace dart::simulation::detail
