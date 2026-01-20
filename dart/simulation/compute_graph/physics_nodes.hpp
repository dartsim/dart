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

#include <dart/simulation/compute_graph/compute_node.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/dynamics/Skeleton.hpp>

#include <memory>
#include <vector>

namespace dart::simulation {
class World;
}

namespace dart::simulation {

/// Computes forward dynamics and integrates velocities for a skeleton.
/// Phase 1 of World::step(): skel->computeForwardDynamics() +
/// integrateVelocities()
class DART_API ForwardDynamicsNode : public ComputeNode
{
public:
  explicit ForwardDynamicsNode(dynamics::SkeletonPtr skeleton);

  void execute(const ExecutionContext& ctx) override;

  [[nodiscard]] dynamics::SkeletonPtr getSkeleton() const
  {
    return mSkeleton;
  }

private:
  dynamics::SkeletonPtr mSkeleton;
};

/// Solves all constraints (contacts, joint limits, etc.).
/// Phase 2 of World::step(): constraintSolver->solve()
/// NOT parallel-safe - acts as synchronization barrier.
class DART_API ConstraintSolveNode : public ComputeNode
{
public:
  explicit ConstraintSolveNode(constraint::ConstraintSolver* solver);

  void execute(const ExecutionContext& ctx) override;

  [[nodiscard]] bool isParallelSafe() const override
  {
    return false;
  }

private:
  constraint::ConstraintSolver* mSolver;
};

/// Applies impulses and integrates positions for a skeleton.
/// Phase 3 of World::step(): impulse forward dynamics + position integration
class DART_API IntegratePositionsNode : public ComputeNode
{
public:
  explicit IntegratePositionsNode(
      dynamics::SkeletonPtr skeleton, bool resetCommand = true);

  void execute(const ExecutionContext& ctx) override;

  void setResetCommand(bool reset)
  {
    mResetCommand = reset;
  }

private:
  dynamics::SkeletonPtr mSkeleton;
  bool mResetCommand;
};

/// Updates simulation time and frame counter.
/// Final phase - NOT parallel-safe.
class DART_API TimeAdvanceNode : public ComputeNode
{
public:
  TimeAdvanceNode(double* timePtr, int* framePtr, double timeStep);

  void execute(const ExecutionContext& ctx) override;

  [[nodiscard]] bool isParallelSafe() const override
  {
    return false;
  }

private:
  double* mTimePtr;
  int* mFramePtr;
  double mTimeStep;
};

/// Batch forward dynamics for multiple skeletons.
/// Reduces scheduling overhead when many small skeletons exist.
class DART_API BatchForwardDynamicsNode : public ComputeNode
{
public:
  explicit BatchForwardDynamicsNode(
      std::vector<dynamics::SkeletonPtr> skeletons);

  void execute(const ExecutionContext& ctx) override;

  void addSkeleton(dynamics::SkeletonPtr skeleton);

  [[nodiscard]] std::size_t getSkeletonCount() const
  {
    return mSkeletons.size();
  }

private:
  std::vector<dynamics::SkeletonPtr> mSkeletons;
};

/// Batch position integration for multiple skeletons.
class DART_API BatchIntegratePositionsNode : public ComputeNode
{
public:
  explicit BatchIntegratePositionsNode(
      std::vector<dynamics::SkeletonPtr> skeletons, bool resetCommand = true);

  void execute(const ExecutionContext& ctx) override;

  void setResetCommand(bool reset)
  {
    mResetCommand = reset;
  }

private:
  std::vector<dynamics::SkeletonPtr> mSkeletons;
  bool mResetCommand;
};

} // namespace dart::simulation
