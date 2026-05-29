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

#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/comps/deformable_body.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace sx = dart::simulation::experimental;
namespace compute = dart::simulation::experimental::compute;

namespace {

// A spring chain hanging along -z from a pinned top node.
sx::DeformableBodyOptions makeChainOptions(int count, double spacing)
{
  sx::DeformableBodyOptions options;
  for (int i = 0; i < count; ++i) {
    options.positions.emplace_back(0.0, 0.0, -spacing * static_cast<double>(i));
    options.masses.push_back(1.0);
  }
  options.fixedNodes = {0};
  for (int i = 0; i + 1 < count; ++i) {
    options.edges.push_back(
        sx::DeformableEdge{
            static_cast<std::size_t>(i),
            static_cast<std::size_t>(i + 1),
            -1.0});
  }
  options.edgeStiffness = 500.0;
  return options;
}

// Opt every deformable body in the world into the internal VBD inner solver.
void enableVbd(sx::World& world, std::size_t iterations)
{
  auto& registry = world.getRegistry();
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(
        entity, sx::comps::DeformableVbdConfig{true, iterations});
  }
}

void stepOnce(sx::World& world, compute::DeformableDynamicsStage& stage)
{
  compute::SequentialExecutor executor;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);
}

} // namespace

//==============================================================================
TEST(VbdWorldSolver, RunsVbdPathWhenEnabled)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("chain", makeChainOptions(6, 0.5));
  enableVbd(world, 30);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.vbdSweeps, 0u);
  EXPECT_GT(stats.vbdVertexUpdates, 0u);
}

//==============================================================================
TEST(VbdWorldSolver, DefaultSolverRunsWhenNotOptedIn)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("chain", makeChainOptions(6, 0.5));

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 0u);
  EXPECT_GT(stats.solverIterations, 0u); // gradient-descent path ran
}

//==============================================================================
// VBD and the default gradient-descent solver minimize the same per-step
// variational objective, so on a contact-free mass-spring scene their stepped
// trajectories must agree.
TEST(VbdWorldSolver, MatchesDefaultSolverOnContactFreeScene)
{
  sx::World vbdWorld;
  sx::World defaultWorld;
  for (sx::World* world : {&vbdWorld, &defaultWorld}) {
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->setTimeStep(0.01);
    world->addDeformableBody("chain", makeChainOptions(7, 0.5));
  }
  enableVbd(vbdWorld, 200);

  compute::DeformableDynamicsStage vbdStage;
  compute::DeformableDynamicsStage defaultStage;
  for (int step = 0; step < 5; ++step) {
    stepOnce(vbdWorld, vbdStage);
    stepOnce(defaultWorld, defaultStage);
  }

  const auto vbdBody = vbdWorld.getDeformableBody("chain");
  const auto defaultBody = defaultWorld.getDeformableBody("chain");
  ASSERT_TRUE(vbdBody.has_value());
  ASSERT_TRUE(defaultBody.has_value());
  ASSERT_EQ(vbdBody->getNodeCount(), defaultBody->getNodeCount());
  for (std::size_t i = 0; i < vbdBody->getNodeCount(); ++i) {
    EXPECT_NEAR(
        (vbdBody->getPosition(i) - defaultBody->getPosition(i)).norm(),
        0.0,
        1e-4)
        << "node " << i;
  }
}

//==============================================================================
TEST(VbdWorldSolver, HangingChainStretchesAndStaysStable)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("chain", makeChainOptions(8, 0.5));
  enableVbd(world, 30);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 200; ++step) {
    stepOnce(world, stage);
    const auto body = world.getDeformableBody("chain");
    ASSERT_TRUE(body.has_value());
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      const Eigen::Vector3d p = body->getPosition(i);
      ASSERT_TRUE(p.allFinite()) << "blew up at step " << step;
      ASSERT_LT(p.norm(), 1e3) << "diverged at step " << step;
    }
  }

  const auto body = world.getDeformableBody("chain");
  ASSERT_TRUE(body.has_value());
  // Top node pinned at the origin; gravity pulls the bottom node further down.
  EXPECT_NEAR(body->getPosition(0).norm(), 0.0, 1e-9);
  EXPECT_LT(body->getPosition(body->getNodeCount() - 1).z(), -0.5 * 7.0);
}
