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

#include "dart/simulation/compute_graph/physics_nodes.hpp"

namespace dart::simulation {

ForwardDynamicsNode::ForwardDynamicsNode(dynamics::SkeletonPtr skeleton)
  : ComputeNode("forward_dynamics"), mSkeleton(std::move(skeleton))
{
}

void ForwardDynamicsNode::execute(const ExecutionContext& ctx)
{
  if (!mSkeleton || !mSkeleton->isMobile()) {
    return;
  }
  mSkeleton->computeForwardDynamics();
  mSkeleton->integrateVelocities(ctx.timeStep);
}

ConstraintSolveNode::ConstraintSolveNode(constraint::ConstraintSolver* solver)
  : ComputeNode("constraint_solve"), mSolver(solver)
{
}

void ConstraintSolveNode::execute(const ExecutionContext& /*ctx*/)
{
  if (mSolver) {
    mSolver->solve();
  }
}

IntegratePositionsNode::IntegratePositionsNode(
    dynamics::SkeletonPtr skeleton, bool resetCommand)
  : ComputeNode("integrate_positions"),
    mSkeleton(std::move(skeleton)),
    mResetCommand(resetCommand)
{
}

void IntegratePositionsNode::execute(const ExecutionContext& ctx)
{
  if (!mSkeleton || !mSkeleton->isMobile()) {
    return;
  }

  if (mSkeleton->isImpulseApplied()) {
    mSkeleton->computeImpulseForwardDynamics();
    mSkeleton->setImpulseApplied(false);
  }

  if (mSkeleton->isPositionImpulseApplied()) {
    mSkeleton->integratePositions(
        ctx.timeStep, mSkeleton->getPositionVelocityChanges());
    mSkeleton->setPositionImpulseApplied(false);
    mSkeleton->clearPositionVelocityChanges();
  } else {
    mSkeleton->integratePositions(ctx.timeStep);
  }

  if (mResetCommand) {
    mSkeleton->clearInternalForces();
    mSkeleton->clearExternalForces();
    mSkeleton->resetCommands();
  }
}

TimeAdvanceNode::TimeAdvanceNode(
    double* timePtr, int* framePtr, double timeStep)
  : ComputeNode("time_advance"),
    mTimePtr(timePtr),
    mFramePtr(framePtr),
    mTimeStep(timeStep)
{
}

void TimeAdvanceNode::execute(const ExecutionContext& /*ctx*/)
{
  if (mTimePtr) {
    *mTimePtr += mTimeStep;
  }
  if (mFramePtr) {
    ++(*mFramePtr);
  }
}

BatchForwardDynamicsNode::BatchForwardDynamicsNode(
    std::vector<dynamics::SkeletonPtr> skeletons)
  : ComputeNode("batch_forward_dynamics"), mSkeletons(std::move(skeletons))
{
}

void BatchForwardDynamicsNode::execute(const ExecutionContext& ctx)
{
  for (auto& skel : mSkeletons) {
    if (!skel || !skel->isMobile()) {
      continue;
    }
    skel->computeForwardDynamics();
    skel->integrateVelocities(ctx.timeStep);
  }
}

void BatchForwardDynamicsNode::addSkeleton(dynamics::SkeletonPtr skeleton)
{
  mSkeletons.push_back(std::move(skeleton));
}

BatchIntegratePositionsNode::BatchIntegratePositionsNode(
    std::vector<dynamics::SkeletonPtr> skeletons, bool resetCommand)
  : ComputeNode("batch_integrate_positions"),
    mSkeletons(std::move(skeletons)),
    mResetCommand(resetCommand)
{
}

void BatchIntegratePositionsNode::execute(const ExecutionContext& ctx)
{
  for (auto& skel : mSkeletons) {
    if (!skel || !skel->isMobile()) {
      continue;
    }

    if (skel->isImpulseApplied()) {
      skel->computeImpulseForwardDynamics();
      skel->setImpulseApplied(false);
    }

    if (skel->isPositionImpulseApplied()) {
      skel->integratePositions(
          ctx.timeStep, skel->getPositionVelocityChanges());
      skel->setPositionImpulseApplied(false);
      skel->clearPositionVelocityChanges();
    } else {
      skel->integratePositions(ctx.timeStep);
    }

    if (mResetCommand) {
      skel->clearInternalForces();
      skel->clearExternalForces();
      skel->resetCommands();
    }
  }
}

} // namespace dart::simulation
