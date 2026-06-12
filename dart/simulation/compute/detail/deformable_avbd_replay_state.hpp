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
/// replay only needs the persistent row inventories whose lambda/stiffness
/// values affect the next AVBD solve.
struct DeformableAvbdWarmStartReplayState
{
  using RowAllocator = dart::common::StlAllocator<dvbd::AvbdScalarRowRecord>;
  using RowVector = std::vector<dvbd::AvbdScalarRowRecord, RowAllocator>;

  DeformableAvbdWarmStartReplayState() = default;

  explicit DeformableAvbdWarmStartReplayState(
      dart::common::MemoryAllocator& allocator)
    : contactRows(RowAllocator{allocator}),
      frictionRows(RowAllocator{allocator}),
      selfContactRows(RowAllocator{allocator}),
      selfContactFrictionRows(RowAllocator{allocator}),
      attachmentRows(RowAllocator{allocator}),
      springRows(RowAllocator{allocator}),
      tetRows(RowAllocator{allocator})
  {
    // Empty.
  }

  entt::entity entity = entt::null;
  RowVector contactRows;
  RowVector frictionRows;
  RowVector selfContactRows;
  RowVector selfContactFrictionRows;
  RowVector attachmentRows;
  RowVector springRows;
  RowVector tetRows;
};

using DeformableAvbdWarmStartReplayStates
    = std::vector<DeformableAvbdWarmStartReplayState>;
using AllocatedDeformableAvbdWarmStartReplayStates = std::vector<
    DeformableAvbdWarmStartReplayState,
    dart::common::StlAllocator<DeformableAvbdWarmStartReplayState>>;

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

} // namespace dart::simulation::compute::avbd_replay
