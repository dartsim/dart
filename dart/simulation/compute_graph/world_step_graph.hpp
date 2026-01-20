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

#include <dart/simulation/compute_graph/compute_graph.hpp>
#include <dart/simulation/compute_graph/graph_executor.hpp>

#include <vector>

namespace dart::simulation {
class World;
class IntegratePositionsNode;
class BatchIntegratePositionsNode;
} // namespace dart::simulation

namespace dart::simulation {

/// Builds and executes a compute graph equivalent to World::step().
/// Wrapper that integrates the compute graph with the main World class.
class DART_API WorldStepGraph
{
public:
  explicit WorldStepGraph(World& world, const ExecutorConfig& config = {});

  void step(bool resetCommand = true);

  [[nodiscard]] ComputeGraph& getGraph()
  {
    return *mGraph;
  }

  [[nodiscard]] const ComputeGraph& getGraph() const
  {
    return *mGraph;
  }

  [[nodiscard]] GraphExecutor& getExecutor()
  {
    return *mExecutor;
  }

  void rebuild();

  [[nodiscard]] std::size_t getNumWorkers() const
  {
    return mExecutor->getNumWorkers();
  }

private:
  void buildGraph();
  [[nodiscard]] std::size_t resolveBatchSize(std::size_t skeletonCount) const;

  World& mWorld;
  ComputeGraphPtr mGraph;
  GraphExecutorPtr mExecutor;
  ExecutorConfig mConfig;
  std::vector<IntegratePositionsNode*> mIntegrateNodes;
  std::vector<BatchIntegratePositionsNode*> mBatchIntegrateNodes;
};

} // namespace dart::simulation
