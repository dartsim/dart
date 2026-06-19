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

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/comps/dynamics.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Dense>
#include <entt/fwd.hpp>

#include <span>
#include <vector>

namespace dart::simulation::compute {

/// One rigid-rigid contact after material and inverse-mass precomputation.
///
/// The row convention is the relative contact velocity `v_B - v_A`. Body A
/// therefore contributes with sign -1 and body B with sign +1 in the Delassus
/// assembly and impulse application.
struct RigidBodyContactConstraint
{
  entt::entity bodyA{entt::null};
  entt::entity bodyB{entt::null};
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d armA = Eigen::Vector3d::Zero();
  Eigen::Vector3d armB = Eigen::Vector3d::Zero();
  bool staticA = false;
  bool staticB = false;
  double invMassA = 0.0;
  double invMassB = 0.0;
  Eigen::Matrix3d invInertiaA = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d invInertiaB = Eigen::Matrix3d::Zero();
  double effectiveMass = 0.0;
  double depth = 0.0;
  double restitution = 0.0;
  double restitutionVelocity = 0.0;
  double normalVelocityBias = 0.0;
  double normalImpulse = 0.0;
  Eigen::Vector3d tangent1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d tangent2 = Eigen::Vector3d::UnitY();
  double tangentMass1 = 0.0;
  double tangentMass2 = 0.0;
  double tangentImpulse1 = 0.0;
  double tangentImpulse2 = 0.0;
  double friction = 1.0;
};

enum class RigidBodyContactRowLayout
{
  /// Rows are grouped by contact: normal, tangent 1, tangent 2.
  ContactMajor,
  /// All normal rows first, then each contact's two tangent rows.
  NormalThenTangents,
};

struct RigidBodyContactAssemblyOptions
{
  bool populateSystem = true;
  bool populateJacobian = false;
  RigidBodyContactRowLayout rowLayout = RigidBodyContactRowLayout::ContactMajor;
  bool includeBaumgarteBias = false;
  bool regularizeFrictionRows = false;
  double timeStep = 0.0;
};

/// Canonical rigid-body contact problem.
///
/// The accepted contacts and dynamic-body order are always populated. Dense
/// system rows and Jacobian rows are optional because some callers consume only
/// the precomputed per-contact constraints.
struct RigidBodyContactProblem
{
  static constexpr int kRowsPerContact = 3;

  using ConstraintAllocator = common::StlAllocator<RigidBodyContactConstraint>;
  using EntityAllocator = common::StlAllocator<entt::entity>;

  RigidBodyContactProblem() = default;

  explicit RigidBodyContactProblem(common::MemoryAllocator& allocator)
    : constraints(ConstraintAllocator{allocator}),
      dynamicBodies(EntityAllocator{allocator})
  {
  }

  std::vector<RigidBodyContactConstraint, ConstraintAllocator> constraints;
  std::vector<entt::entity, EntityAllocator> dynamicBodies;
  Eigen::MatrixXd delassus;
  Eigen::VectorXd rhs;
  Eigen::VectorXd lo;
  Eigen::VectorXd hi;
  Eigen::VectorXi findex;
  Eigen::MatrixXd jacobian;
};

/// Velocity of a rigid body's contact point in world coordinates.
DART_SIMULATION_API Eigen::Vector3d computeRigidBodyContactPointVelocity(
    const comps::Velocity& velocity, const Eigen::Vector3d& arm, bool isStatic);

/// Assemble the rigid-body-only contact problem.
///
/// Contacts involving multibody links are ignored here; they are handled by
/// articulated contact stages until the unified constraint stage lands. The
/// default output row order is three rows per accepted rigid contact: normal,
/// first tangent, second tangent.
DART_SIMULATION_API RigidBodyContactProblem assembleRigidBodyContactProblem(
    const detail::WorldRegistry& registry, std::span<const Contact> contacts);

DART_SIMULATION_API RigidBodyContactProblem assembleRigidBodyContactProblem(
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const RigidBodyContactAssemblyOptions& options);

/// Assemble the rigid-body-only contact problem into caller-owned
/// storage, reusing any existing vector/Eigen capacity for same-shape solves.
DART_SIMULATION_API void assembleRigidBodyContactProblemInto(
    RigidBodyContactProblem& problem,
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts);

DART_SIMULATION_API void assembleRigidBodyContactProblemInto(
    RigidBodyContactProblem& problem,
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const RigidBodyContactAssemblyOptions& options);

/// Apply one world-space contact impulse to both rigid-body ends.
DART_SIMULATION_API void applyRigidBodyContactImpulse(
    detail::WorldRegistry& registry,
    const RigidBodyContactConstraint& constraint,
    const Eigen::Vector3d& impulse);

} // namespace dart::simulation::compute
