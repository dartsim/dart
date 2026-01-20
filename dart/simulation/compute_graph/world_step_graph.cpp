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

#include "dart/simulation/compute_graph/world_step_graph.hpp"

#include "dart/simulation/World.hpp"
#include "dart/simulation/compute_graph/physics_nodes.hpp"

namespace dart::simulation {

WorldStepGraph::WorldStepGraph(World& world, const ExecutorConfig& config)
  : mWorld(world),
    mGraph(std::make_shared<ComputeGraph>()),
    mExecutor(GraphExecutor::create(config))
{
  buildGraph();
}

void WorldStepGraph::step(bool resetCommand)
{
  mResetCommand = resetCommand;

  ExecutionContext ctx;
  ctx.timeStep = mWorld.getTimeStep();
  ctx.currentTime = mWorld.getTime();
  ctx.frameNumber = static_cast<std::uint64_t>(mWorld.getSimFrames());
  ctx.resetCommand = resetCommand;
  ctx.worldPtr = &mWorld;

  mExecutor->execute(*mGraph, ctx);
}

void WorldStepGraph::rebuild()
{
  mGraph->clear();
  buildGraph();
}

void WorldStepGraph::buildGraph()
{
  std::vector<NodeId> forwardDynamicsNodes;
  std::vector<NodeId> integrateNodes;

  for (std::size_t i = 0; i < mWorld.getNumSkeletons(); ++i) {
    auto skel = mWorld.getSkeleton(i);
    if (!skel || !skel->isMobile()) {
      continue;
    }

    auto fdNode = std::make_shared<ForwardDynamicsNode>(skel);
    NodeId fdId = mGraph->addNode(fdNode);
    forwardDynamicsNodes.push_back(fdId);

    auto intNode
        = std::make_shared<IntegratePositionsNode>(skel, mResetCommand);
    NodeId intId = mGraph->addNode(intNode);
    integrateNodes.push_back(intId);
  }

  auto constraintNode
      = std::make_shared<ConstraintSolveNode>(mWorld.getConstraintSolver());
  NodeId constraintId = mGraph->addNode(constraintNode);

  for (NodeId fdId : forwardDynamicsNodes) {
    mGraph->addEdge(fdId, constraintId);
  }

  for (NodeId intId : integrateNodes) {
    mGraph->addEdge(constraintId, intId);
  }

  auto timeNode = std::make_shared<TimeAdvanceNode>(
      nullptr, nullptr, mWorld.getTimeStep());
  NodeId timeId = mGraph->addNode(timeNode);

  for (NodeId intId : integrateNodes) {
    mGraph->addEdge(intId, timeId);
  }

  if (integrateNodes.empty()) {
    mGraph->addEdge(constraintId, timeId);
  }

  mGraph->finalize();
}

} // namespace dart::simulation
