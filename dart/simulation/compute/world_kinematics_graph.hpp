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

#include <dart/simulation/compute/compute_graph.hpp>
#include <dart/simulation/export.hpp>

#include <dart/common/stl_allocator.hpp>

#include <entt/entt.hpp>

#include <string>
#include <vector>

#include <cstdint>

namespace dart::simulation {
class World;
}

namespace dart::simulation::compute {

class ComputeExecutor;

/// Builds a graph for the DART 7 World's kinematic cache update stage.
class DART_SIMULATION_API WorldKinematicsGraph
{
public:
  explicit WorldKinematicsGraph(World& world);
  WorldKinematicsGraph(World& world, dart::common::MemoryAllocator& allocator);

  void rebuild();
  [[nodiscard]] bool isTopologyCurrent() const noexcept;
  void execute(ComputeExecutor& executor);

  [[nodiscard]] const ComputeGraph& getGraph() const noexcept
  {
    return m_graph;
  }

private:
  struct EntityNode
  {
    entt::entity entity;
    ComputeNode* node;
  };
  using EntityNodeAllocator = dart::common::StlAllocator<EntityNode>;

  [[nodiscard]] ComputeNode* findNode(entt::entity entity) const;

  World& m_world;
  dart::common::MemoryAllocator* m_allocator = nullptr;
  ComputeGraph m_graph;
  std::vector<EntityNode, EntityNodeAllocator> m_entityNodes;
  std::uint64_t m_frameTopologyRevision = 0;
};

} // namespace dart::simulation::compute
