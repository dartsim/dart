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

#include "demo_models/BoxedLcpSolver.hpp"

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

namespace {

class NonBoxedDemoConstraintSolver : public dart::constraint::ConstraintSolver
{
protected:
  void solveConstrainedGroup(dart::constraint::ConstrainedGroup&) override {}
};

} // namespace

//==============================================================================
TEST(BoxedLcpSolverTest, SwitchesBackendAndPreservesWorld)
{
  const auto world = dart::simulation::World::create();
  const auto skeleton = dart::dynamics::Skeleton::create("box");
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  world->addSkeleton(skeleton);
  world->setTimeStep(0.002);
  world->setNumSimulationThreads(2);
  const auto* originalSolver = world->getConstraintSolver();

  for (const std::string kind : {"dantzig", "pgs", "dantzig"}) {
    skeleton->setResting(true);
    ASSERT_TRUE(skeleton->isResting());
    dart_demos::setBoxedLcpSolver(world, kind);
    ASSERT_EQ(world->getConstraintSolver(), originalSolver);

    const auto* solver
        = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
            world->getConstraintSolver());
    ASSERT_NE(solver, nullptr);
    EXPECT_EQ(
        solver->getBoxedLcpSolver()->getType(),
        kind == "dantzig"
            ? dart::constraint::DantzigBoxedLcpSolver::getStaticType()
            : dart::constraint::PgsBoxedLcpSolver::getStaticType());
    EXPECT_EQ(solver->getTimeStep(), 0.002);
    EXPECT_EQ(solver->getNumSimulationThreads(), 2u);
    EXPECT_EQ(world->getNumSkeletons(), 1u);
    EXPECT_EQ(world->getSkeleton(0), skeleton);
    EXPECT_FALSE(skeleton->isResting());
    world->step();
    EXPECT_TRUE(skeleton->getPositions().allFinite());
  }
}

//==============================================================================
TEST(BoxedLcpSolverTest, RejectsNonBoxedConstraintSolverWithoutReplacingIt)
{
  const auto world = dart::simulation::World::create();
  world->setConstraintSolver(std::make_unique<NonBoxedDemoConstraintSolver>());
  const auto* solver = world->getConstraintSolver();
  for (const std::string kind : {"dantzig", "pgs"}) {
    EXPECT_THROW(
        dart_demos::setBoxedLcpSolver(world, kind), std::invalid_argument);
    EXPECT_EQ(world->getConstraintSolver(), solver);
  }
}

//==============================================================================
TEST(BoxedLcpSolverTest, RejectsInvalidInputsBeforeChangingWorld)
{
  const auto world = dart::simulation::World::create();
  const auto* solver = world->getConstraintSolver();
  for (const std::string kind : {"", "Dantzig", "other"}) {
    EXPECT_THROW(
        dart_demos::setBoxedLcpSolver(world, kind), std::invalid_argument);
    EXPECT_EQ(world->getConstraintSolver(), solver);
  }
  EXPECT_THROW(
      dart_demos::setBoxedLcpSolver(nullptr, "dantzig"), std::invalid_argument);
}
