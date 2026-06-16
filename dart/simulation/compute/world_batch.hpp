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

#include <dart/simulation/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/export.hpp>

#include <vector>

#include <cstddef>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::simulation::compute {

class ComputeExecutor;

/// Advance a homogeneous batch of independent worlds in parallel.
///
/// Each world becomes an independent node in a single compute graph and is
/// stepped @p stepCount times through the default (sequential) per-world path;
/// @p executor then runs the nodes, so a parallel executor advances the whole
/// batch concurrently. This is the heterogeneous-fallback seed for batched
/// execution: it preserves exact per-World semantics by scheduling independent
/// Worlds through the existing compute-graph executor seam. The canonical
/// homogeneous direction is a baked immutable Model plus SoA State blocks with
/// a leading world dimension.
///
/// The worlds must be independent (no shared state); their per-world results
/// are identical to stepping each world on its own. Throws if any pointer is
/// null.
DART_SIMULATION_API void stepWorldsBatched(
    const std::vector<World*>& worlds,
    std::size_t stepCount,
    ComputeExecutor& executor);

/// Roll out a homogeneous batch of worlds from a shared initial state.
///
/// Applies @p initialState to @p worlds, advances them @p stepCount times with
/// @c stepWorldsBatched, and returns the resulting batched state. This is the
/// rollout entry point built on the batch executor and the SoA state batch; it
/// keeps device/stream types out of the API. Control-sequence inputs are a
/// later addition once a control owner type exists.
[[nodiscard]] DART_SIMULATION_API RigidBodyStateBatch rolloutWorldsBatched(
    const std::vector<World*>& worlds,
    const RigidBodyStateBatch& initialState,
    std::size_t stepCount,
    ComputeExecutor& executor);

} // namespace dart::simulation::compute
