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

#include <dart/simulation/body/contact_force.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/detail/memory_diagnostics.hpp>
#include <dart/simulation/detail/smooth_jacobians.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/diff/physical_parameter.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/stl_allocator.hpp>

#include <optional>
#include <set>
#include <utility>
#include <vector>

#include <cstdint>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::simulation::detail {

struct BakedMultibodyModel
{
  entt::entity entity = entt::null;
  std::size_t linkOffset = 0;
  std::size_t linkCount = 0;
  std::size_t jointOffset = 0;
  std::size_t jointCount = 0;
  std::size_t dofOffset = 0;
  std::size_t dofCount = 0;
};

/// Immutable dense-index Model artifact baked at the simulation boundary.
///
/// Entity vectors define the canonical creation-ordered dense indices used by
/// state/control facades and batch extraction. Scalar arrays are Model data
/// aligned with those indices and are rebuilt only when topology or model
/// parameters change.
struct BakedWorldModel
{
  using EntityAllocator = dart::common::StlAllocator<entt::entity>;
  using EntityVector = std::vector<entt::entity, EntityAllocator>;
  using IndexAllocator = dart::common::StlAllocator<std::size_t>;
  using IndexVector = std::vector<std::size_t, IndexAllocator>;
  using ScalarAllocator = dart::common::StlAllocator<double>;
  using ScalarVector = std::vector<double, ScalarAllocator>;
  using ByteAllocator = dart::common::StlAllocator<std::uint8_t>;
  using ByteVector = std::vector<std::uint8_t, ByteAllocator>;
  using MultibodyAllocator = dart::common::StlAllocator<BakedMultibodyModel>;
  using MultibodyVector = std::vector<BakedMultibodyModel, MultibodyAllocator>;

  explicit BakedWorldModel(dart::common::MemoryAllocator& allocator);

  void clear() noexcept;

  bool valid = false;
  std::uint64_t rigidBodyModelBuildCount = 0;

  EntityVector rigidBodyEntities;
  EntityVector dynamicRigidBodyEntities;
  ByteVector rigidBodyIsDynamic;
  ScalarVector rigidBodyInverseMass;
  ScalarVector rigidBodyInertia;

  MultibodyVector multibodies;
  EntityVector multibodyLinkEntities;
  EntityVector multibodyJointEntities;
  IndexVector multibodyLinkDofOffsets;
  IndexVector multibodyLinkDofs;
  ScalarVector multibodyLinkMass;
  ScalarVector multibodyLinkInertia;
};

/// Opaque, ECS-typed storage owned by `World` via a `std::unique_ptr`.
///
/// This is the internal-only home for the EnTT-typed `World` state. Keeping it
/// out of `world.hpp` (forward-declared there) lets the promoted public header
/// name no EnTT symbols and include no `<entt/entt.hpp>`. This header is
/// implementation-internal: it MUST NOT be included by any promoted public
/// header. Only `world.cpp` (and other internal `.cpp` files that need the
/// registry) include it.
struct WorldStorage
{
  using DifferentiableParameter = std::pair<entt::entity, PhysicalParameter>;
  using DifferentiableParameterAllocator
      = dart::common::StlAllocator<DifferentiableParameter>;
  using DifferentiableTorqueAllocator = dart::common::StlAllocator<double>;
  using CollisionPairKey = std::pair<entt::entity, entt::entity>;
  using CollisionPairAllocator = dart::common::StlAllocator<CollisionPairKey>;
  using IgnoredCollisionPairSet = std::set<
      CollisionPairKey,
      std::less<CollisionPairKey>,
      CollisionPairAllocator>;

  explicit WorldStorage(dart::common::MemoryAllocator& allocator);

  /// Free allocator backing World-owned storage and internal scratch helpers.
  dart::common::MemoryAllocator& memoryAllocator;

  /// The ECS registry holding every entity and component owned by the World.
  WorldRegistry registry;

  /// Cached frame-scratch counters and opt-in ECS diagnostics collector.
  MemoryDiagnosticsTracker memoryDiagnostics;

  /// Registered differentiable physical parameters, in registration order. Each
  /// entry pairs the owning rigid-body entity with the parameter to
  /// differentiate. Columns of `StepDerivatives::parameterJacobian` follow this
  /// order. Stored in both build configs (the public registration API exists
  /// either way); only consumed when differentiable support is compiled.
  std::vector<DifferentiableParameter, DifferentiableParameterAllocator>
      differentiableParameters;

  /// Reusable torque collection scratch for differentiable multibody
  /// derivatives. This avoids rebuilding a default-allocator std::vector on
  /// every differentiable step.
  std::vector<double, DifferentiableTorqueAllocator>
      differentiableTorqueScratch;

  /// Reusable coordinate collection scratch for the differentiable contact-free
  /// multibody Jacobian path.
  ContactFreeStepCoordinateScratch differentiableCoordinateScratch;

  /// Reusable inverse-dynamics derivative scratch for the differentiable
  /// contact-free multibody Jacobian path.
  compute::MultibodyInverseDynamicsScratch differentiableInverseDynamicsScratch;

  /// Reusable dynamics-term scratch for the differentiable contact-free
  /// multibody Jacobian path.
  ContactFreeStepDynamicsTermsScratch differentiableDynamicsTermsScratch;

  /// Reusable Eigen matrix/vector scratch for the differentiable contact-free
  /// multibody Jacobian path.
  ContactFreeStepDerivativeScratch differentiableDerivativeScratch;

  /// Cached explicit Jacobians of the most recent differentiable step.
  /// Populated only when the World opted into differentiable simulation and
  /// differentiable support is compiled (`DART_HAS_DIFF`); always empty
  /// otherwise.
  std::optional<StepDerivatives> stepDerivatives;

  /// True only after a real simulation step captured `stepDerivatives`.
  ///
  /// The bake boundary may resize this storage to make the first post-bake
  /// differentiable step allocation-free, but callers still need `step()` to
  /// populate a semantically valid derivative snapshot.
  bool stepDerivativesValid = false;

  /// Last-step contact and solver diagnostics populated by mutating step
  /// stages. `World::computeStepMetrics()` folds these into its read-only
  /// physical snapshot without running a narrow-phase query itself.
  compute::StepMetrics lastStepDiagnostics;

  /// Per-contact reaction forces recovered from the most recent rigid-body
  /// contact solve (`force = impulse / timeStep`), exposed through
  /// `World::getLastContactForces()`. Populated by the rigid contact stage for
  /// the sequential-impulse and boxed-LCP paths and cleared alongside
  /// `lastStepDiagnostics`; empty otherwise.
  std::vector<ContactForce> lastContactForces;

  /// Persistent pair-level collision-query exclusions, stored with canonical
  /// endpoint ordering. This scene-level filter is applied after broad-phase
  /// candidate generation and before narrow-phase contact generation.
  IgnoredCollisionPairSet ignoredCollisionPairs;

  /// Baked dense-index Model artifact for simulation-mode steady-state paths.
  BakedWorldModel bakedModel;
};

[[nodiscard]] DART_SIMULATION_API const BakedWorldModel&
ensureBakedWorldModelCurrent(const World& world);

} // namespace dart::simulation::detail
