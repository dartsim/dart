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

#include "dart/simulation/compute/world_batch.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/compute_stage_metadata.hpp"
#include "dart/simulation/world.hpp"

#include <string>
#include <unordered_set>

namespace dart::simulation::compute {

//==============================================================================
void stepWorldsBatched(
    const std::vector<World*>& worlds,
    std::size_t stepCount,
    ComputeExecutor& executor)
{
  ComputeGraph graph;
  std::unordered_set<const World*> seen;
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_SIMULATION_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "stepWorldsBatched received a null world at lane {}",
        w);
    // The nodes have no edges and run concurrently, so the same World must not
    // appear twice -- that would step one object from multiple threads at once
    // (a data race) and advance it more than intended.
    DART_SIMULATION_THROW_T_IF(
        !seen.insert(worlds[w]).second,
        InvalidArgumentException,
        "stepWorldsBatched received a duplicate world at lane {}; each world "
        "must be unique",
        w);

    auto* world = worlds[w];
    graph.addNode(
        "world_" + std::to_string(w),
        [world, stepCount]() { world->step(stepCount); },
        {ComputeStageDomain::Simulation,
         toMask(ComputeStageAcceleration::TaskParallel)});
  }

  executor.execute(graph);
}

//==============================================================================
RigidBodyStateBatch rolloutWorldsBatched(
    const std::vector<World*>& worlds,
    const RigidBodyStateBatch& initialState,
    std::size_t stepCount,
    ComputeExecutor& executor)
{
  applyRigidBodyStateBatch(worlds, initialState);
  stepWorldsBatched(worlds, stepCount, executor);

  const std::vector<const World*> constWorlds(worlds.begin(), worlds.end());
  return extractRigidBodyStateBatch(constWorlds);
}

} // namespace dart::simulation::compute
