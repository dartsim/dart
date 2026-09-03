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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp>
#include <dart/simulation/detail/rigid_avbd/projected_velocity_record.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <entt/entt.hpp>

#include <span>
#include <vector>

namespace dart::simulation::compute::avbd_replay {

namespace dvbd = dart::simulation::detail::deformable_vbd;

/// Replay snapshot of the AVBD deformable warm-start continuation state.
///
/// The live scratch component remains private to the deformable dynamics stage;
/// replay preserves both scalar row state and the per-friction-pair tangent
/// anchors whose transported reference geometry affects the next AVBD solve.
struct DeformableAvbdWarmStartReplayState
{
  using RowAllocator = dart::common::StlAllocator<dvbd::AvbdScalarRowRecord>;
  using RowVector = std::vector<dvbd::AvbdScalarRowRecord, RowAllocator>;
  using HalfSpaceAnchorAllocator
      = dart::common::StlAllocator<dvbd::AvbdHalfSpaceTangentAnchorState>;
  using HalfSpaceAnchorVector = std::
      vector<dvbd::AvbdHalfSpaceTangentAnchorState, HalfSpaceAnchorAllocator>;
  using SelfContactAnchorAllocator
      = dart::common::StlAllocator<dvbd::AvbdSelfContactTangentAnchorState>;
  using SelfContactAnchorVector = std::vector<
      dvbd::AvbdSelfContactTangentAnchorState,
      SelfContactAnchorAllocator>;

  DeformableAvbdWarmStartReplayState() = default;

  explicit DeformableAvbdWarmStartReplayState(
      dart::common::MemoryAllocator& allocator)
    : contactRows(RowAllocator{allocator}),
      frictionRows(RowAllocator{allocator}),
      frictionAnchors(HalfSpaceAnchorAllocator{allocator}),
      selfContactRows(RowAllocator{allocator}),
      selfContactFrictionRows(RowAllocator{allocator}),
      selfContactFrictionAnchors(SelfContactAnchorAllocator{allocator}),
      attachmentRows(RowAllocator{allocator}),
      springRows(RowAllocator{allocator})
  {
    // Empty.
  }

  entt::entity entity = entt::null;
  RowVector contactRows;
  RowVector frictionRows;
  HalfSpaceAnchorVector frictionAnchors;
  RowVector selfContactRows;
  RowVector selfContactFrictionRows;
  SelfContactAnchorVector selfContactFrictionAnchors;
  RowVector attachmentRows;
  RowVector springRows;
};

/// Replay snapshot of the rigid AVBD warm-start continuation state.
///
/// The live inventories belong to `RigidBodyContactStage`; replay preserves
/// the scalar row records plus contact-only material identities and tangent
/// anchors whose payload affects the next solve.
struct RigidAvbdWarmStartReplayState
{
  using RowAllocator = dart::common::StlAllocator<dvbd::AvbdScalarRowRecord>;
  using RowVector = std::vector<dvbd::AvbdScalarRowRecord, RowAllocator>;
  using ContactIdentityAllocator
      = dart::common::StlAllocator<dvbd::AvbdRigidContactIdentityState>;
  using ContactIdentityVector = std::
      vector<dvbd::AvbdRigidContactIdentityState, ContactIdentityAllocator>;
  using ContactAnchorAllocator
      = dart::common::StlAllocator<dvbd::AvbdContactTangentAnchorState>;
  using ContactAnchorVector = std::
      vector<dvbd::AvbdContactTangentAnchorState, ContactAnchorAllocator>;
  using ProjectedVelocityAllocator
      = dart::common::StlAllocator<dvbd::AvbdRigidProjectedVelocityRecord>;
  using ProjectedVelocityVector = std::vector<
      dvbd::AvbdRigidProjectedVelocityRecord,
      ProjectedVelocityAllocator>;

  RigidAvbdWarmStartReplayState() = default;

  explicit RigidAvbdWarmStartReplayState(
      dart::common::MemoryAllocator& allocator)
    : normalRows(RowAllocator{allocator}),
      frictionRows(RowAllocator{allocator}),
      contactIdentities(ContactIdentityAllocator{allocator}),
      contactTangentAnchors(ContactAnchorAllocator{allocator}),
      jointLinearRows(RowAllocator{allocator}),
      jointAngularRows(RowAllocator{allocator}),
      motorRows(RowAllocator{allocator}),
      distanceSpringRows(RowAllocator{allocator}),
      projectedVelocities(ProjectedVelocityAllocator{allocator})
  {
    // Empty.
  }

  RowVector normalRows;
  RowVector frictionRows;
  ContactIdentityVector contactIdentities;
  ContactAnchorVector contactTangentAnchors;
  RowVector jointLinearRows;
  RowVector jointAngularRows;
  RowVector motorRows;
  RowVector distanceSpringRows;
  /// Projected linear velocities of the last two owned steps that feed the
  /// adaptive initial guess (AVBD Algorithm 1 line 4).
  ProjectedVelocityVector projectedVelocities;
};

using DeformableAvbdWarmStartReplayStates
    = std::vector<DeformableAvbdWarmStartReplayState>;
using AllocatedDeformableAvbdWarmStartReplayStates = std::vector<
    DeformableAvbdWarmStartReplayState,
    dart::common::StlAllocator<DeformableAvbdWarmStartReplayState>>;

/// Whether any private deformable AVBD row inventory currently carries
/// continuation state that binary serialization does not encode.
///
/// This query is allocation-free and is used to fail closed before writing a
/// mid-trajectory binary checkpoint that could not resume exactly.
[[nodiscard]] DART_SIMULATION_API bool
hasDeformableAvbdWarmStartContinuationState(
    const detail::WorldRegistry& registry) noexcept;

[[nodiscard]] DART_SIMULATION_API DeformableAvbdWarmStartReplayStates
captureDeformableAvbdWarmStartReplayState(
    const detail::WorldRegistry& registry);

[[nodiscard]] DART_SIMULATION_API AllocatedDeformableAvbdWarmStartReplayStates
captureDeformableAvbdWarmStartReplayState(
    const detail::WorldRegistry& registry,
    dart::common::MemoryAllocator& allocator);

DART_SIMULATION_API void restoreDeformableAvbdWarmStartReplayState(
    detail::WorldRegistry& registry,
    std::span<const DeformableAvbdWarmStartReplayState> replayStates);

DART_SIMULATION_API void restoreDeformableAvbdWarmStartReplayState(
    detail::WorldRegistry& registry,
    std::span<const DeformableAvbdWarmStartReplayState> replayStates,
    dart::common::MemoryAllocator& allocator);

/// Consume a previously captured live-state snapshot during replay rollback.
///
/// Every referenced scratch component existed when the snapshot was captured,
/// and normal replay restoration never removes those components. Moving the
/// allocator-backed row payloads back into that retained storage therefore
/// restores continuation state without consulting the allocator. This is the
/// failure path used after a replay restore has already mutated live state, so
/// it must remain allocation-free and non-throwing.
DART_SIMULATION_API void restoreDeformableAvbdWarmStartReplayStateNoAlloc(
    detail::WorldRegistry& registry,
    std::span<DeformableAvbdWarmStartReplayState> replayStates) noexcept;

} // namespace dart::simulation::compute::avbd_replay
