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

// A single tetrahedron with its z=0 base face (nodes 0,1,2) pinned and its apex
// (node 3) free above it, so gravity compresses the tet and the Neo-Hookean
// energy resists. The material stiffness is configurable so tests can assert
// that a stiffer body deforms less.
sx::DeformableBodyOptions makeTetOptions(double youngsModulus)
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)};
  options.masses = {1.0, 1.0, 1.0, 1.0};
  options.tetrahedra = {sx::DeformableTetrahedron{0, 1, 2, 3}};
  options.fixedNodes = {0, 1, 2};
  options.material.youngsModulus = youngsModulus;
  options.material.poissonRatio = 0.3;
  options.damping = 2.0;
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

// Opt every deformable body into VBD with an explicit config (acceleration and
// damping knobs).
void enableVbdConfig(
    sx::World& world, const sx::comps::DeformableVbdConfig& cfg)
{
  auto& registry = world.getRegistry();
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(entity, cfg);
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

//==============================================================================
// A contact-free tetrahedral body opts into VBD too: the World path now
// assembles Stable Neo-Hookean tetrahedra, which the default gradient solver
// does not model.
TEST(VbdWorldSolver, RunsVbdPathForTetrahedralBody)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("tet", makeTetOptions(1.0e5));
  enableVbd(world, 30);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.vbdSweeps, 0u);
  EXPECT_GT(stats.vbdVertexUpdates, 0u);
}

//==============================================================================
TEST(VbdWorldSolver, TetrahedralBodyIsStableUnderGravity)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("tet", makeTetOptions(1.0e5));
  enableVbd(world, 30);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 200; ++step) {
    stepOnce(world, stage);
    const auto body = world.getDeformableBody("tet");
    ASSERT_TRUE(body.has_value());
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      const Eigen::Vector3d p = body->getPosition(i);
      ASSERT_TRUE(p.allFinite()) << "blew up at step " << step;
      ASSERT_LT(p.norm(), 1e3) << "diverged at step " << step;
    }
  }

  const auto body = world.getDeformableBody("tet");
  ASSERT_TRUE(body.has_value());
  // Apex stays above the pinned base (the Neo-Hookean energy resists
  // inversion).
  EXPECT_GT(body->getPosition(3).z(), 0.0);
}

//==============================================================================
// The load-bearing volumetric check: a stiffer Neo-Hookean material resists
// gravity more, so its apex settles closer to the rest height. This is only
// possible because the World VBD path now models material-dependent tet
// elasticity (the default solver, which ignores Young's modulus, cannot).
TEST(VbdWorldSolver, StifferTetrahedralBodyDeformsLess)
{
  const auto settledApexHeight = [](double youngsModulus) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("tet", makeTetOptions(youngsModulus));
    enableVbd(world, 60);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 400; ++step) {
      stepOnce(world, stage);
    }
    const auto body = world.getDeformableBody("tet");
    return body->getPosition(3).z();
  };

  const double softApex = settledApexHeight(1.0e4);
  const double stiffApex = settledApexHeight(1.0e6);

  // Both compress downward from the rest apex height of 1.0 ...
  EXPECT_LT(softApex, 1.0);
  EXPECT_LT(stiffApex, 1.0);
  // ... but the stiffer body compresses measurably less.
  EXPECT_GT(stiffApex, softApex + 1e-4);
}

//==============================================================================
// Chebyshev over-relaxation and Rayleigh damping flow from the config through
// the World VBD path; the body stays stable and the accelerated solve agrees
// with the unaccelerated one (Chebyshev preserves the fixed point).
TEST(VbdWorldSolver, ChebyshevConfigMatchesPlainAndStaysStable)
{
  sx::World plainWorld;
  sx::World chebyshevWorld;
  for (sx::World* world : {&plainWorld, &chebyshevWorld}) {
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->setTimeStep(0.01);
    world->addDeformableBody("chain", makeChainOptions(7, 0.5));
  }
  enableVbd(plainWorld, 200);
  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 200;
  cfg.useChebyshev = true;
  cfg.chebyshevRho = 0.9;
  enableVbdConfig(chebyshevWorld, cfg);

  compute::DeformableDynamicsStage plainStage;
  compute::DeformableDynamicsStage chebyshevStage;
  for (int step = 0; step < 5; ++step) {
    stepOnce(plainWorld, plainStage);
    stepOnce(chebyshevWorld, chebyshevStage);
  }

  const auto plainBody = plainWorld.getDeformableBody("chain");
  const auto chebyshevBody = chebyshevWorld.getDeformableBody("chain");
  ASSERT_TRUE(plainBody.has_value());
  ASSERT_TRUE(chebyshevBody.has_value());
  for (std::size_t i = 0; i < plainBody->getNodeCount(); ++i) {
    EXPECT_NEAR(
        (plainBody->getPosition(i) - chebyshevBody->getPosition(i)).norm(),
        0.0,
        1e-4)
        << "node " << i;
  }
}

//==============================================================================
// Rayleigh damping dissipates motion: after release under gravity, the damped
// chain carries less kinetic energy than the undamped one.
TEST(VbdWorldSolver, RayleighDampingReducesKineticEnergy)
{
  const auto kineticEnergyAfter = [](double rayleigh) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("chain", makeChainOptions(8, 0.5));
    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 40;
    cfg.rayleighDamping = rayleigh;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 30; ++step) {
      stepOnce(world, stage);
    }
    const auto body = world.getDeformableBody("chain");
    double energy = 0.0;
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      energy += body->getVelocity(i).squaredNorm();
    }
    return energy;
  };

  const double undamped = kineticEnergyAfter(0.0);
  const double damped = kineticEnergyAfter(0.2);
  EXPECT_GT(undamped, 0.0);
  EXPECT_LT(damped, undamped);
}

//==============================================================================
// The multithreaded VBD sweep is deterministic: a body solved with several
// worker threads tracks the single-threaded result step for step.
TEST(VbdWorldSolver, MultithreadedSolveMatchesSingleThreaded)
{
  sx::World serialWorld;
  sx::World parallelWorld;
  for (sx::World* world : {&serialWorld, &parallelWorld}) {
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->setTimeStep(0.01);
    world->addDeformableBody("chain", makeChainOptions(12, 0.5));
  }
  sx::comps::DeformableVbdConfig serialCfg;
  serialCfg.enabled = true;
  serialCfg.iterations = 40;
  enableVbdConfig(serialWorld, serialCfg);
  sx::comps::DeformableVbdConfig parallelCfg = serialCfg;
  parallelCfg.workerThreads = 4;
  enableVbdConfig(parallelWorld, parallelCfg);

  compute::DeformableDynamicsStage serialStage;
  compute::DeformableDynamicsStage parallelStage;
  for (int step = 0; step < 10; ++step) {
    stepOnce(serialWorld, serialStage);
    stepOnce(parallelWorld, parallelStage);
  }

  const auto serialBody = serialWorld.getDeformableBody("chain");
  const auto parallelBody = parallelWorld.getDeformableBody("chain");
  ASSERT_TRUE(serialBody.has_value());
  ASSERT_TRUE(parallelBody.has_value());
  for (std::size_t i = 0; i < serialBody->getNodeCount(); ++i) {
    EXPECT_NEAR(
        (serialBody->getPosition(i) - parallelBody->getPosition(i)).norm(),
        0.0,
        1e-12)
        << "node " << i;
  }
}
