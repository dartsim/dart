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

#include <algorithm>

namespace dart::simulation {

WorldStepGraph::WorldStepGraph(World& world, const ExecutorConfig& config)
  : mWorld(world),
    mGraph(std::make_shared<ComputeGraph>()),
    mExecutor(GraphExecutor::create(config)),
    mConfig(config)
{
  buildGraph();
}

void WorldStepGraph::step(bool resetCommand)
{
  for (auto* node : mIntegrateNodes) {
    node->setResetCommand(resetCommand);
  }
  for (auto* node : mBatchIntegrateNodes) {
    node->setResetCommand(resetCommand);
  }

  ExecutionContext ctx;
  ctx.timeStep = mWorld.getTimeStep();
  ctx.currentTime = mWorld.getTime();
  ctx.frameNumber = static_cast<std::uint64_t>(mWorld.getSimFrames());
  ctx.resetCommand = resetCommand;
  ctx.worldPtr = &mWorld;

  mExecutor->execute(*mGraph, ctx);
  mWorld.getSensorManager().updateSensors(mWorld);
}

void WorldStepGraph::rebuild()
{
  mGraph->clear();
  mIntegrateNodes.clear();
  mBatchIntegrateNodes.clear();
  buildGraph();
}

void WorldStepGraph::buildGraph()
{
  std::vector<NodeId> forwardDynamicsNodes;
  std::vector<NodeId> integrateNodes;
  std::vector<dynamics::SkeletonPtr> mobileSkeletons;

  mobileSkeletons.reserve(mWorld.getNumSkeletons());

  for (std::size_t i = 0; i < mWorld.getNumSkeletons(); ++i) {
    auto skel = mWorld.getSkeleton(i);
    if (!skel || !skel->isMobile()) {
      continue;
    }

    mobileSkeletons.push_back(skel);
  }

  const std::size_t batchSize = resolveBatchSize(mobileSkeletons.size());

  if (batchSize <= 1 || mobileSkeletons.size() <= 1) {
    for (const auto& skel : mobileSkeletons) {
      auto fdNode = std::make_shared<ForwardDynamicsNode>(skel);
      NodeId fdId = mGraph->addNode(fdNode);
      forwardDynamicsNodes.push_back(fdId);

      auto intNode = std::make_shared<IntegratePositionsNode>(skel);
      mIntegrateNodes.push_back(intNode.get());
      NodeId intId = mGraph->addNode(intNode);
      integrateNodes.push_back(intId);
    }
  } else {
    for (std::size_t start = 0; start < mobileSkeletons.size();
         start += batchSize) {
      const std::size_t end
          = std::min(start + batchSize, mobileSkeletons.size());
      std::vector<dynamics::SkeletonPtr> batch;
      batch.reserve(end - start);
      for (std::size_t i = start; i < end; ++i) {
        batch.push_back(mobileSkeletons[i]);
      }

      auto fdNode = std::make_shared<BatchForwardDynamicsNode>(batch);
      NodeId fdId = mGraph->addNode(fdNode);
      forwardDynamicsNodes.push_back(fdId);

      auto intNode = std::make_shared<BatchIntegratePositionsNode>(batch);
      mBatchIntegrateNodes.push_back(intNode.get());
      NodeId intId = mGraph->addNode(intNode);
      integrateNodes.push_back(intId);
    }
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
      &mWorld.mTime, &mWorld.mFrame, mWorld.getTimeStep());
  NodeId timeId = mGraph->addNode(timeNode);

  for (NodeId intId : integrateNodes) {
    mGraph->addEdge(intId, timeId);
  }

  if (integrateNodes.empty()) {
    mGraph->addEdge(constraintId, timeId);
  }

  mGraph->finalize();
}

std::size_t WorldStepGraph::resolveBatchSize(std::size_t skeletonCount) const
{
  if (skeletonCount == 0) {
    return 1;
  }

  if (mConfig.batchSize != 0) {
    return mConfig.batchSize;
  }

  std::size_t numWorkers = mExecutor ? mExecutor->getNumWorkers() : 1;
  if (numWorkers == 0) {
    numWorkers = 1;
  }

  const std::size_t targetTasks = numWorkers * 2;
  std::size_t batchSize = (skeletonCount + targetTasks - 1) / targetTasks;
  if (batchSize == 0) {
    batchSize = 1;
  }

  return batchSize;
}

} // namespace dart::simulation
