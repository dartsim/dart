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

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/compute/detail/world_step_stages.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/world.hpp>

#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace sx = dart::simulation;
namespace compute = dart::simulation::compute;

namespace {

void setGroundBarrierPolicy(sx::RigidBody& body)
{
  auto policy = body.getDeformableObstaclePolicy();
  policy.groundBarrier = true;
  body.setDeformableObstaclePolicy(policy);
}

void setSurfaceObstaclePolicy(sx::RigidBody& body)
{
  auto policy = body.getDeformableObstaclePolicy();
  policy.surfaceObstacle = true;
  body.setDeformableObstaclePolicy(policy);
}

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

sx::DeformableBodyOptions makeTetSpringOptions(double youngsModulus)
{
  sx::DeformableBodyOptions options = makeTetOptions(youngsModulus);
  options.edges.push_back(sx::DeformableEdge{0, 3, -1.0});
  options.edgeStiffness = 500.0;
  return options;
}

// Opt every deformable body in the world into the internal VBD inner solver.
void enableVbd(sx::World& world, std::size_t iterations)
{
  auto& registry = dart::simulation::detail::registryOf(world);
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
  auto& registry = dart::simulation::detail::registryOf(world);
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(entity, cfg);
  }
}

void stepOnce(
    sx::World& world,
    compute::DeformableDynamicsStage& stage,
    compute::ComputeExecutor& executor)
{
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);
}

void stepOnce(sx::World& world, compute::DeformableDynamicsStage& stage)
{
  compute::SequentialExecutor executor;
  stepOnce(world, stage, executor);
}

void expectNoAvbdRows(const compute::DeformableSolverStats& stats)
{
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_EQ(stats.vbdAvbdSelfContactNormalRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);
}

// Add a static box ground barrier whose top face is at z = 0.
void addGroundBarrier(sx::World& world)
{
  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", options);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 4.0, 0.5)));
  setGroundBarrierPolicy(ground);
}

// A small free (un-pinned) spring patch in the xy plane at height z0, with an
// optional initial horizontal velocity, dropped onto the ground.
sx::DeformableBodyOptions makeFallingPatchOptions(double z0, double velocityX)
{
  constexpr int side = 3;
  constexpr double spacing = 0.1;
  sx::DeformableBodyOptions options;
  const auto index = [](int r, int c) {
    return static_cast<std::size_t>(r * side + c);
  };
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      options.positions.emplace_back(
          spacing * c, spacing * r, z0 + 0.002 * ((r + c) % 2));
      options.velocities.emplace_back(velocityX, 0.0, 0.0);
      options.masses.push_back(0.1);
    }
  }
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      if (c + 1 < side) {
        options.edges.push_back({index(r, c), index(r, c + 1), -1.0});
      }
      if (r + 1 < side) {
        options.edges.push_back({index(r, c), index(r + 1, c), -1.0});
      }
    }
  }
  options.edgeStiffness = 200.0;
  options.damping = 2.0;
  return options;
}

// A free point on a deformable body moving toward another body's triangle
// surface. The body's own triangle is fixed far above the crossing point; the
// unreferenced point still participates in inter-body CCD for surface-only
// bodies.
sx::DeformableBodyOptions makeInterBodyMovingPointOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 3.0),
         Eigen::Vector3d(1.0, -1.0, 3.0),
         Eigen::Vector3d(0.0, 1.0, 3.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, -20.0)};
  options.masses.assign(options.positions.size(), 1.0);
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

sx::DeformableBodyOptions makeInterBodyTriangleObstacleOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 1.0);
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

// Two horizontal triangles in a single deformable body: a wide triangle pinned
// in the z = 0 plane and a smaller free triangle just above it (held rigid by
// edge springs). Under gravity the top triangle falls toward the bottom; with
// self-contact the two surfaces of the same body must not interpenetrate.
sx::DeformableBodyOptions makeSelfFoldingBody()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),   // 0 bottom (pinned)
         Eigen::Vector3d(0.5, -0.5, 0.0),    // 1
         Eigen::Vector3d(0.0, 0.6, 0.0),     // 2
         Eigen::Vector3d(-0.2, -0.15, 0.12), // 3 top (free)
         Eigen::Vector3d(0.2, -0.15, 0.12),  // 4
         Eigen::Vector3d(0.0, 0.2, 0.12)};   // 5
  options.masses = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {{0, 1, 2}, {3, 4, 5}};
  options.edges = {{3, 4, -1.0}, {4, 5, -1.0}, {5, 3, -1.0}};
  options.edgeStiffness = 500.0;
  options.damping = 1.0;
  return options;
}

sx::DeformableBodyOptions makeNearSelfContactSpringBody()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(0.0, 0.6, 0.0),
         Eigen::Vector3d(-0.2, -0.15, 0.01),
         Eigen::Vector3d(0.2, -0.15, 0.01),
         Eigen::Vector3d(0.0, 0.2, 0.01)};
  options.masses = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {{0, 1, 2}, {3, 4, 5}};
  options.edges = {{3, 4, -1.0}, {4, 5, -1.0}, {5, 3, -1.0}};
  options.edgeStiffness = 500.0;
  options.damping = 1.0;
  return options;
}

sx::DeformableBodyOptions makeNearSelfContactSlidingSpringBody(double velocityX)
{
  sx::DeformableBodyOptions options = makeNearSelfContactSpringBody();
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  for (std::size_t i = 3; i < 6; ++i) {
    options.velocities[i] = Eigen::Vector3d(velocityX, 0.0, 0.0);
  }
  return options;
}

sx::DeformableBodyOptions makeNearSelfContactTetBody()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(0.0, 0.6, 0.0),
         Eigen::Vector3d(0.0, 0.0, -0.2),
         Eigen::Vector3d(-0.2, -0.15, 0.01),
         Eigen::Vector3d(0.2, -0.15, 0.01),
         Eigen::Vector3d(0.0, 0.2, 0.01),
         Eigen::Vector3d(0.0, 0.0, 0.21)};
  options.masses = {1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1};
  options.fixedNodes = {0, 1, 2, 3};
  options.surfaceTriangles = {{0, 1, 2}, {4, 5, 6}};
  options.tetrahedra = {{0, 1, 2, 3}, {4, 5, 6, 7}};
  options.material.youngsModulus = 1.0e5;
  options.material.poissonRatio = 0.3;
  options.damping = 1.0;
  return options;
}

// Two same-body surface triangles where the top triangle would move completely
// through the pinned bottom triangle in one step if VBD did not reuse the
// self-surface CCD limiter after its block solve.
sx::DeformableBodyOptions makeSelfCrossingTriangleOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(-0.2, -0.2, 1.0),
         Eigen::Vector3d(0.2, -0.2, 1.0),
         Eigen::Vector3d(0.0, 0.2, 1.0)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, -20.0),
         Eigen::Vector3d(0.0, 0.0, -20.0),
         Eigen::Vector3d(0.0, 0.0, -20.0)};
  options.masses.assign(options.positions.size(), 1.0);
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {{0, 1, 2}, {3, 4, 5}};
  return options;
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
// The AVBD attachment-row World slice converts pinned nodes into warm-started
// hard point-attachment rows inside the supported CPU mass-spring path, instead
// of exposing row storage through the public body API.
TEST(VbdWorldSolver, AvbdAttachmentRowsHoldPinnedNode)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.02);
  world.addDeformableBody("chain", makeChainOptions(8, 0.5));

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 50;
  cfg.useAvbdAttachmentRows = true;
  cfg.avbdAttachmentStiffness = 100.0;
  cfg.avbdAlpha = 0.0;
  cfg.avbdBeta = 5000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 120; ++step) {
    stepOnce(world, stage);
    const auto& stats = stage.getLastStats();
    ASSERT_EQ(stats.vbdBodyCount, 1u);
    ASSERT_EQ(stats.vbdAvbdAttachmentRows, 3u);
    ASSERT_EQ(stats.vbdAvbdContactNormalRows, 0u);
    ASSERT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
  }

  const auto body = world.getDeformableBody("chain");
  ASSERT_TRUE(body.has_value());
  EXPECT_LT(body->getPosition(0).norm(), 0.1);
  EXPECT_LT(body->getPosition(body->getNodeCount() - 1).z(), -0.5 * 7.0);
}

//==============================================================================
// Progressive finite-stiffness rows start each spring below its material
// stiffness, then persistently harden through the World row inventory. Compared
// against a constant-soft chain, the AVBD finite-stiffness chain should stretch
// less while reporting the internal spring rows that drove the solve.
TEST(VbdWorldSolver, AvbdFiniteStiffnessRowsHardenSpringChain)
{
  const auto bottomZ = [](double edgeStiffness, bool useFiniteRows) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.02);
    sx::DeformableBodyOptions options = makeChainOptions(8, 0.5);
    options.edgeStiffness = edgeStiffness;
    world.addDeformableBody("chain", options);

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 40;
    cfg.useAvbdFiniteStiffnessRows = useFiniteRows;
    cfg.avbdFiniteStiffnessStart = 20.0;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = edgeStiffness;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 120; ++step) {
      stepOnce(world, stage);
      const auto& stats = stage.getLastStats();
      EXPECT_EQ(stats.vbdBodyCount, 1u);
      EXPECT_EQ(
          stats.vbdAvbdFiniteStiffnessRows,
          useFiniteRows ? static_cast<std::size_t>(7) : 0u);
      EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
      EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
      EXPECT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
    }

    const auto body = world.getDeformableBody("chain");
    if (!body.has_value()) {
      ADD_FAILURE() << "missing chain body";
      return -1e9;
    }
    return body->getPosition(body->getNodeCount() - 1).z();
  };

  const double softBottom = bottomZ(20.0, false);
  const double finiteBottom = bottomZ(500.0, true);
  EXPECT_GT(finiteBottom, softBottom + 0.1);
}

//==============================================================================
// The tetrahedral material finite-stiffness World slice uses dimensionless
// per-tet rows that ramp from a soft material scale toward the full Lamé
// parameters. Compared against a permanently soft tet, the AVBD tet-material
// body should settle closer to its rest apex height while reporting the active
// tet row.
TEST(VbdWorldSolver, AvbdFiniteStiffnessRowsHardenTetrahedralMaterial)
{
  const auto apexZ = [](double youngsModulus, bool useFiniteRows) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("tet", makeTetOptions(youngsModulus));

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 50;
    cfg.useAvbdFiniteStiffnessRows = useFiniteRows;
    cfg.avbdFiniteStiffnessStart = 0.02;
    cfg.avbdBeta = 30.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 1.0;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 160; ++step) {
      stepOnce(world, stage);
      const auto& stats = stage.getLastStats();
      EXPECT_EQ(stats.vbdBodyCount, 1u);
      EXPECT_EQ(
          stats.vbdAvbdFiniteStiffnessRows,
          useFiniteRows ? static_cast<std::size_t>(1) : 0u);
      EXPECT_EQ(
          stats.vbdAvbdFiniteStiffnessTetRows,
          useFiniteRows ? static_cast<std::size_t>(1) : 0u);
      EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
      EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
      EXPECT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
    }

    const auto body = world.getDeformableBody("tet");
    if (!body.has_value()) {
      ADD_FAILURE() << "missing tet body";
      return -1e9;
    }
    return body->getPosition(3).z();
  };

  const double softApex = apexZ(1.0e4, false);
  const double finiteApex = apexZ(1.0e6, true);
  EXPECT_GT(finiteApex, softApex + 1e-4);
}

//==============================================================================
// A material friction coefficient is inert when no static-contact or
// self-contact source is active. It should not force finite-stiffness-only AVBD
// mass-spring scenes back to the legacy VBD path.
TEST(VbdWorldSolver, AvbdFiniteStiffnessRowsIgnoreUnusedFrictionCoefficient)
{
  const auto baseConfig = [] {
    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 8;
    cfg.useAvbdFiniteStiffnessRows = true;
    cfg.avbdFiniteStiffnessStart = 20.0;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 500.0;
    return cfg;
  };
  const auto run = [](sx::DeformableBodyOptions options,
                      sx::comps::DeformableVbdConfig cfg) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("body", options);
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    stepOnce(world, stage);
    return stage.getLastStats();
  };

  {
    auto options = makeChainOptions(8, 0.5);
    options.material.frictionCoefficient = 0.4;
    const auto stats = run(options, baseConfig());
    EXPECT_EQ(stats.vbdBodyCount, 1u);
    EXPECT_GT(stats.vbdAvbdFiniteStiffnessRows, 0u);
    EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);
    EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
    EXPECT_EQ(stats.vbdAvbdSelfContactNormalRows, 0u);
    EXPECT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
    EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  }
}

//==============================================================================
// Optional AVBD row family toggles are often enabled globally. A body should
// still take the supported AVBD mass-spring row path when one requested family
// has active rows and another requested family is empty for that body.
TEST(VbdWorldSolver, AvbdMassSpringRowsAllowEmptyRequestedFamilies)
{
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    addGroundBarrier(world);

    sx::DeformableBodyOptions options;
    options.positions = {Eigen::Vector3d(0.0, 0.0, -0.01)};
    options.masses = {1.0};
    world.addDeformableBody("point", options);

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 8;
    cfg.contactStiffness = 100.0;
    cfg.useAvbdContactNormalRows = true;
    cfg.useAvbdFiniteStiffnessRows = true;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 500.0;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    stepOnce(world, stage);

    const auto& stats = stage.getLastStats();
    EXPECT_EQ(stats.vbdBodyCount, 1u);
    EXPECT_GT(stats.vbdAvbdContactNormalRows, 0u);
    EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
    EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  }

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    sx::DeformableBodyOptions options = makeChainOptions(4, 0.25);
    options.fixedNodes.clear();
    world.addDeformableBody("chain", options);

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 8;
    cfg.useAvbdAttachmentRows = true;
    cfg.useAvbdFiniteStiffnessRows = true;
    cfg.avbdFiniteStiffnessStart = 20.0;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 500.0;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    stepOnce(world, stage);

    const auto& stats = stage.getLastStats();
    EXPECT_EQ(stats.vbdBodyCount, 1u);
    EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
    EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
    EXPECT_GT(stats.vbdAvbdFiniteStiffnessRows, 0u);
  }
}

//==============================================================================
// Ground-barrier AVBD rows should only exist inside the static-contact
// activation band. Nodes high above the ground must keep the contact-free VBD
// step instead of carrying an inactive hard-row stiffness block.
TEST(VbdWorldSolver, AvbdContactNormalRowsSkipInactiveGroundRows)
{
  const auto stepPoint
      = [](bool withGround, compute::DeformableSolverStats* statsOut) {
          sx::World world;
          world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
          world.setTimeStep(0.01);
          if (withGround) {
            addGroundBarrier(world);
          }

          sx::DeformableBodyOptions options;
          options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
          options.masses = {1.0};
          world.addDeformableBody("point", options);

          sx::comps::DeformableVbdConfig cfg;
          cfg.enabled = true;
          cfg.iterations = 8;
          cfg.contactStiffness = 100.0;
          cfg.useAvbdContactNormalRows = true;
          cfg.avbdBeta = 2000.0;
          cfg.avbdMaxStiffness = 500.0;
          enableVbdConfig(world, cfg);

          compute::DeformableDynamicsStage stage;
          stepOnce(world, stage);
          if (statsOut != nullptr) {
            *statsOut = stage.getLastStats();
          }

          const auto body = world.getDeformableBody("point");
          if (!body.has_value()) {
            ADD_FAILURE() << "missing point body";
            return 0.0;
          }
          return body->getPosition(0).z();
        };

  compute::DeformableSolverStats groundStats;
  const double contactFreeZ = stepPoint(false, nullptr);
  const double groundZ = stepPoint(true, &groundStats);

  EXPECT_EQ(groundStats.vbdBodyCount, 1u);
  EXPECT_EQ(groundStats.vbdAvbdContactNormalRows, 0u);
  EXPECT_NEAR(groundZ, contactFreeZ, 1e-12);
}

//==============================================================================
// The current finite-stiffness AVBD World slices are intentionally narrow. The
// mass-spring row path supports static contact/friction rows and explicit
// self-contact row opt-ins, while the pure tet material row path can coexist
// with the existing lagged VBD self-contact penalty. Mixed spring-plus-tet
// topology, Chebyshev, Rayleigh damping, parallel execution, and unsupported
// requested row families must keep using the existing VBD path and report no
// AVBD rows.
TEST(VbdWorldSolver, AvbdFiniteStiffnessRowsFallbackForUnsupportedEnvelopes)
{
  const auto baseConfig = [] {
    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 8;
    cfg.useAvbdFiniteStiffnessRows = true;
    cfg.avbdFiniteStiffnessStart = 20.0;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 500.0;
    return cfg;
  };
  const auto run = [](sx::DeformableBodyOptions options,
                      sx::comps::DeformableVbdConfig cfg) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("body", options);
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    stepOnce(world, stage);
    return stage.getLastStats();
  };

  {
    const auto stats = run(makeTetSpringOptions(1.0e5), baseConfig());
    expectNoAvbdRows(stats);
  }
  {
    const auto stats = run(makeNearSelfContactSpringBody(), baseConfig());
    expectNoAvbdRows(stats);
  }
  {
    auto cfg = baseConfig();
    cfg.useChebyshev = true;
    const auto stats = run(makeChainOptions(8, 0.5), cfg);
    expectNoAvbdRows(stats);
  }
  {
    auto cfg = baseConfig();
    cfg.rayleighDamping = 0.1;
    const auto stats = run(makeChainOptions(8, 0.5), cfg);
    expectNoAvbdRows(stats);
  }
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("body", makeChainOptions(8, 0.5));
    enableVbdConfig(world, baseConfig());

    compute::DeformableDynamicsStage stage;
    compute::ParallelExecutor executor(2);
    stepOnce(world, stage, executor);
    const auto stats = stage.getLastStats();
    expectNoAvbdRows(stats);
  }
}

//==============================================================================
// A supported serial mass-spring self-contact solve can now use AVBD hard
// normal rows for lagged point-triangle / edge-edge candidates. The row counter
// proves World generated the AVBD self-contact rows instead of falling back to
// the existing VBD penalty-only self-contact path.
TEST(VbdWorldSolver, AvbdSelfContactNormalRowsPushSupportedSurfaceApart)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  world.addDeformableBody("fold", makeNearSelfContactSpringBody());

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 8;
  cfg.useAvbdSelfContactNormalRows = true;
  cfg.avbdAlpha = 0.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  const auto topMinZ = [&]() {
    const auto body = world.getDeformableBody("fold");
    double minimum = 1e9;
    for (std::size_t i = 3; i < 6; ++i) {
      minimum = std::min(minimum, body->getPosition(i).z());
    }
    return minimum;
  };

  const double before = topMinZ();
  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_GT(stats.vbdAvbdSelfContactNormalRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);
  EXPECT_GT(topMinZ(), before + 1e-4);
}

//==============================================================================
// Supported serial mass-spring self-contact now generates matching AVBD
// tangent friction rows from the same lagged point-triangle / edge-edge
// primitives when the deformable material carries Coulomb friction.
TEST(VbdWorldSolver, AvbdSelfContactNormalRowsIncludeFrictionTangentRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  sx::DeformableBodyOptions options = makeNearSelfContactSpringBody();
  options.material.frictionCoefficient = 0.8;
  world.addDeformableBody("fold", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 8;
  cfg.useAvbdSelfContactNormalRows = true;
  cfg.avbdAlpha = 0.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_GT(stats.vbdAvbdSelfContactNormalRows, 0u);
  EXPECT_EQ(
      stats.vbdAvbdFrictionTangentRows,
      2u * stats.vbdAvbdSelfContactNormalRows);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);
}

//==============================================================================
// World-generated AVBD self-contact friction rows must affect motion, not only
// row accounting: a near-contact upper triangle sliding over a pinned lower
// triangle should move less tangentially when material friction is positive.
TEST(VbdWorldSolver, AvbdSelfContactFrictionRowsReduceTangentialMotion)
{
  struct Result
  {
    double travelledX = 0.0;
    std::size_t maxSelfContactNormalRows = 0;
    std::size_t maxFrictionRows = 0;
  };

  const auto run = [](double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.01);
    sx::DeformableBodyOptions options
        = makeNearSelfContactSlidingSpringBody(/*velocityX=*/0.5);
    options.material.frictionCoefficient = friction;
    world.addDeformableBody("fold", options);

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 20;
    cfg.useAvbdSelfContactNormalRows = true;
    cfg.avbdAlpha = 0.0;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 1.0e6;
    enableVbdConfig(world, cfg);

    const auto topCentroidX = [&]() {
      const auto body = world.getDeformableBody("fold");
      double sum = 0.0;
      for (std::size_t i = 3; i < 6; ++i) {
        sum += body->getPosition(i).x();
      }
      return sum / 3.0;
    };

    const double startX = topCentroidX();
    Result result;
    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 80; ++step) {
      stepOnce(world, stage);
      const auto& stats = stage.getLastStats();
      EXPECT_EQ(stats.vbdBodyCount, 1u);
      result.maxSelfContactNormalRows = std::max(
          result.maxSelfContactNormalRows, stats.vbdAvbdSelfContactNormalRows);
      result.maxFrictionRows
          = std::max(result.maxFrictionRows, stats.vbdAvbdFrictionTangentRows);
      if (stats.vbdAvbdSelfContactNormalRows > 0u) {
        EXPECT_EQ(
            stats.vbdAvbdFrictionTangentRows,
            friction > 0.0 ? 2u * stats.vbdAvbdSelfContactNormalRows : 0u);
      }
    }
    result.travelledX = topCentroidX() - startX;
    return result;
  };

  const Result frictionless = run(0.0);
  const Result frictional = run(1.0);
  EXPECT_GT(frictionless.maxSelfContactNormalRows, 0u);
  EXPECT_GT(frictional.maxSelfContactNormalRows, 0u);
  EXPECT_EQ(frictionless.maxFrictionRows, 0u);
  EXPECT_GT(frictional.maxFrictionRows, 0u);
  EXPECT_GT(frictionless.travelledX, 0.05);
  EXPECT_LT(frictional.travelledX, 0.75 * frictionless.travelledX);
}

//==============================================================================
// Static contact and self-contact friction rows should coexist in the same
// serial AVBD mass-spring solve. This protects the combined-row envelope from
// accidentally treating friction as a single-source row family.
TEST(VbdWorldSolver, AvbdContactAndSelfContactFrictionRowsCombine)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  sx::DeformableBodyOptions options = makeNearSelfContactSpringBody();
  options.material.frictionCoefficient = 0.8;
  world.addDeformableBody("fold", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 12;
  cfg.contactStiffness = 100.0;
  cfg.useAvbdContactNormalRows = true;
  cfg.useAvbdSelfContactNormalRows = true;
  cfg.avbdAlpha = 0.0;
  cfg.avbdBeta = 2000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_GT(stats.vbdAvbdSelfContactNormalRows, 0u);
  EXPECT_EQ(
      stats.vbdAvbdFrictionTangentRows,
      2u
          * (stats.vbdAvbdContactNormalRows
             + stats.vbdAvbdSelfContactNormalRows));
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);
}

//==============================================================================
// Pure tetrahedral scenes can now use AVBD finite-stiffness material rows and
// AVBD self-contact normal/friction rows together, rather than falling back to
// the older VBD self-contact penalty when hard self-contact rows are requested.
TEST(VbdWorldSolver, AvbdTetRowsCombineSelfContactFrictionRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  sx::DeformableBodyOptions options = makeNearSelfContactTetBody();
  options.material.frictionCoefficient = 0.8;
  world.addDeformableBody("tet-fold", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 12;
  cfg.useAvbdFiniteStiffnessRows = true;
  cfg.useAvbdSelfContactNormalRows = true;
  cfg.avbdAlpha = 0.0;
  cfg.avbdBeta = 2000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_GT(stats.vbdAvbdSelfContactNormalRows, 0u);
  EXPECT_EQ(
      stats.vbdAvbdFrictionTangentRows,
      2u * stats.vbdAvbdSelfContactNormalRows);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 2u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 2u);

  const auto body = world.getDeformableBody("tet-fold");
  ASSERT_TRUE(body.has_value());
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    EXPECT_TRUE(body->getPosition(i).allFinite()) << "node " << i;
  }
}

//==============================================================================
// A supported serial mass-spring ground-contact solve now keeps AVBD's friction
// tangents in the same row inventory as the contact-normal rows. Each active
// contact contributes two bounded tangent rows against a lagged Coulomb limit.
TEST(VbdWorldSolver, AvbdContactNormalRowsIncludeFrictionTangentRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  sx::DeformableBodyOptions options = makeFallingPatchOptions(0.0, 0.6);
  options.material.frictionCoefficient = 0.8;
  world.addDeformableBody("patch", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 20;
  cfg.contactStiffness = 100.0;
  cfg.useAvbdContactNormalRows = true;
  cfg.avbdBeta = 2000.0;
  cfg.avbdMaxStiffness = 5.0e4;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);
  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_EQ(
      stats.vbdAvbdFrictionTangentRows, 2u * stats.vbdAvbdContactNormalRows);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);
}

//==============================================================================
// World-generated AVBD friction rows must do more than report row counters:
// over repeated steps, a frictional patch should slide less than the same
// contact-normal AVBD solve with zero Coulomb bound.
TEST(VbdWorldSolver, AvbdFrictionTangentRowsDecelerateSlidingBody)
{
  const auto travelledX = [](double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    addGroundBarrier(world);
    sx::DeformableBodyOptions options = makeFallingPatchOptions(0.0, 0.6);
    options.material.frictionCoefficient = friction;
    world.addDeformableBody("patch", options);

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 40;
    cfg.contactStiffness = 5.0e3;
    cfg.useAvbdContactNormalRows = true;
    cfg.avbdBeta = 2000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 5.0e4;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    const auto start = world.getDeformableBody("patch");
    double startX = 0.0;
    for (std::size_t i = 0; i < start->getNodeCount(); ++i) {
      startX += start->getPosition(i).x();
    }
    startX /= static_cast<double>(start->getNodeCount());

    for (int step = 0; step < 150; ++step) {
      stepOnce(world, stage);
      const auto& stats = stage.getLastStats();
      EXPECT_EQ(stats.vbdBodyCount, 1u);
      EXPECT_GT(stats.vbdAvbdContactNormalRows, 0u);
      EXPECT_EQ(
          stats.vbdAvbdFrictionTangentRows,
          friction > 0.0 ? 2u * stats.vbdAvbdContactNormalRows : 0u);
    }

    const auto body = world.getDeformableBody("patch");
    double endX = 0.0;
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      endX += body->getPosition(i).x();
    }
    endX /= static_cast<double>(body->getNodeCount());
    return endX - startX;
  };

  const double frictionless = travelledX(0.0);
  const double frictional = travelledX(0.8);
  EXPECT_GT(frictionless, 0.05);
  EXPECT_LT(frictional, frictionless);
}

//==============================================================================
// The supported World AVBD envelope now combines the currently implemented
// deformable row families in one serial mass-spring solve. This catches the
// regression where a multi-family request used to partially apply only the
// first matching branch.
TEST(VbdWorldSolver, AvbdRowsCombineContactAttachmentAndFiniteStiffness)
{
  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 20;
  cfg.contactStiffness = 100.0;
  cfg.useAvbdContactNormalRows = true;
  cfg.useAvbdAttachmentRows = true;
  cfg.useAvbdFiniteStiffnessRows = true;
  cfg.avbdAttachmentStiffness = 100.0;
  cfg.avbdFiniteStiffnessStart = 20.0;
  cfg.avbdBeta = 2000.0;
  cfg.avbdMaxStiffness = 5.0e4;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  sx::DeformableBodyOptions options = makeFallingPatchOptions(0.0, 0.0);
  options.fixedNodes = {0};
  options.material.frictionCoefficient = 0.0;
  world.addDeformableBody("patch", options);
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);
  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFrictionTangentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 3u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 12u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessTetRows, 0u);

  const auto body = world.getDeformableBody("patch");
  ASSERT_TRUE(body.has_value());
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    EXPECT_TRUE(body->getPosition(i).allFinite()) << "node " << i;
  }
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
// The fixed-corotational material selector is meaningful only when
// finite-element elasticity is explicitly enabled. With FEM opt-in off, VBD
// must ignore the FCR flag just like the default solver/material contract does.
TEST(VbdWorldSolver, FixedCorotationalFlagRequiresFemOptIn)
{
  const auto runWithFixedCorotationalFlag = [](bool fixedCorotational) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    sx::DeformableBodyOptions options = makeTetOptions(1.0e4);
    options.material.useFiniteElementElasticity = false;
    options.material.useFixedCorotationalElasticity = fixedCorotational;
    world.addDeformableBody("tet", options);
    enableVbd(world, 80);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 40; ++step) {
      stepOnce(world, stage);
    }

    const auto body = world.getDeformableBody("tet");
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(body->getNodeCount());
    for (std::size_t node = 0; node < body->getNodeCount(); ++node) {
      positions.push_back(body->getPosition(node));
    }
    return positions;
  };

  const auto disabled = runWithFixedCorotationalFlag(false);
  const auto ignored = runWithFixedCorotationalFlag(true);
  ASSERT_EQ(disabled.size(), ignored.size());
  for (std::size_t node = 0; node < disabled.size(); ++node) {
    EXPECT_NEAR((disabled[node] - ignored[node]).norm(), 0.0, 1e-12)
        << "node " << node;
  }
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
  enableVbdConfig(parallelWorld, parallelCfg);

  compute::DeformableDynamicsStage serialStage;
  compute::DeformableDynamicsStage parallelStage;
  compute::ParallelExecutor parallelExecutor(4);
  for (int step = 0; step < 10; ++step) {
    stepOnce(serialWorld, serialStage);
    stepOnce(parallelWorld, parallelStage, parallelExecutor);
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

//==============================================================================
// With ground contact enabled (contactStiffness > 0) the World VBD path keeps a
// free spring patch resting on a static ground barrier instead of falling
// through it.
TEST(VbdWorldSolver, VbdBodyRestsOnGround)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  world.addDeformableBody("patch", makeFallingPatchOptions(0.3, 0.0));
  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 40;
  cfg.contactStiffness = 5.0e3;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 300; ++step) {
    stepOnce(world, stage);
    const auto body = world.getDeformableBody("patch");
    ASSERT_TRUE(body.has_value());
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      ASSERT_TRUE(body->getPosition(i).allFinite()) << "blew up at " << step;
    }
  }

  // The VBD path (not the default solver) handled the body.
  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);

  const auto body = world.getDeformableBody("patch");
  ASSERT_TRUE(body.has_value());
  double minZ = 1e9;
  double maxZ = -1e9;
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    const double z = body->getPosition(i).z();
    minZ = std::min(minZ, z);
    maxZ = std::max(maxZ, z);
  }
  // Settled on the ground (z = 0): it came down to the barrier without falling
  // through, and the whole patch rests near the surface.
  EXPECT_GT(minZ, -0.05);
  EXPECT_LT(minZ, 0.05);
  EXPECT_LT(maxZ, 0.1);
}

//==============================================================================
// The first AVBD World integration slice routes static half-space
// contact-normal rows through the persistent row inventory for supported CPU
// mass-spring scenes. With a deliberately soft starting contact stiffness, the
// augmented rows should harden enough to keep the patch higher than the fixed
// penalty contact path under the same step budget.
TEST(VbdWorldSolver, AvbdContactNormalRowsHardenGroundContact)
{
  const auto settledMinZ = [](bool useAvbdRows) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    addGroundBarrier(world);
    sx::DeformableBodyOptions options = makeFallingPatchOptions(0.08, 0.0);
    options.material.frictionCoefficient = 0.0;
    world.addDeformableBody("patch", options);

    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 30;
    cfg.contactStiffness = 50.0;
    cfg.useAvbdContactNormalRows = useAvbdRows;
    cfg.avbdAlpha = 0.0;
    cfg.avbdBeta = 5000.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 5.0e4;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 160; ++step) {
      stepOnce(world, stage);
      EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
    }

    const auto body = world.getDeformableBody("patch");
    if (!body.has_value()) {
      ADD_FAILURE() << "missing patch body";
      return -1e9;
    }
    double minZ = 1e9;
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      const Eigen::Vector3d position = body->getPosition(i);
      EXPECT_TRUE(position.allFinite());
      minZ = std::min(minZ, position.z());
    }
    return minZ;
  };

  const double penaltyMinZ = settledMinZ(false);
  const double avbdMinZ = settledMinZ(true);

  EXPECT_GT(avbdMinZ, penaltyMinZ + 0.01);
  EXPECT_GT(avbdMinZ, -0.03);
}

//==============================================================================
// Coulomb friction against the ground barrier decelerates a sliding patch: a
// frictional body travels less far than a frictionless one over the same steps.
TEST(VbdWorldSolver, VbdFrictionDeceleratesSlidingBody)
{
  const auto travelledX = [](double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    addGroundBarrier(world);
    sx::DeformableBodyOptions options = makeFallingPatchOptions(0.0, 0.6);
    options.material.frictionCoefficient = friction;
    world.addDeformableBody("patch", options);
    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 40;
    cfg.contactStiffness = 5.0e3;
    enableVbdConfig(world, cfg);

    compute::DeformableDynamicsStage stage;
    const auto start = world.getDeformableBody("patch");
    double startX = 0.0;
    for (std::size_t i = 0; i < start->getNodeCount(); ++i) {
      startX += start->getPosition(i).x();
    }
    startX /= static_cast<double>(start->getNodeCount());

    for (int step = 0; step < 150; ++step) {
      stepOnce(world, stage);
    }
    const auto body = world.getDeformableBody("patch");
    double endX = 0.0;
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      endX += body->getPosition(i).x();
    }
    endX /= static_cast<double>(body->getNodeCount());
    return endX - startX;
  };

  const double frictionless = travelledX(0.0);
  const double frictional = travelledX(0.8);
  EXPECT_GT(frictionless, 0.05);       // it slid forward
  EXPECT_LT(frictional, frictionless); // friction shortened the slide
}

//==============================================================================
// The public, solver-agnostic World::configureDeformableSolver opts a body into
// the VBD inner solver without exposing any solver-internal vocabulary.
TEST(VbdWorldSolver, PublicConfigureSelectsVbdSolver)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("chain", makeChainOptions(6, 0.5));

  sx::DeformableSolverOptions options;
  options.iterations = 30;
  options.useAcceleration = false;
  world.configureDeformableSolver("chain", options);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.vbdSweeps, 0u);
}

//==============================================================================
TEST(VbdWorldSolver, PublicConfigureThrowsForUnknownBody)
{
  sx::World world;
  sx::DeformableSolverOptions options;
  EXPECT_ANY_THROW(world.configureDeformableSolver("missing", options));
}

//==============================================================================
// Ground contact configured through the public API keeps a patch on the ground.
TEST(VbdWorldSolver, PublicConfigureGroundContactRests)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  world.addDeformableBody("patch", makeFallingPatchOptions(0.3, 0.0));

  sx::DeformableSolverOptions options;
  options.iterations = 40;
  options.groundContactStiffness = 5.0e3;
  world.configureDeformableSolver("patch", options);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 300; ++step) {
    stepOnce(world, stage);
  }
  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);

  const auto body = world.getDeformableBody("patch");
  ASSERT_TRUE(body.has_value());
  double minZ = 1e9;
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    minZ = std::min(minZ, body->getPosition(i).z());
  }
  EXPECT_GT(minZ, -0.05);
  EXPECT_LT(minZ, 0.05);
}

//==============================================================================
// Option C: with contactStiffness > 0 a VBD body keeps running on the VBD path
// when a static sphere obstacle is present (any obstacle previously forced the
// body off the VBD path onto the default solver) and settles on the obstacle
// surface without penetrating it.
TEST(VbdWorldSolver, VbdBodyRestsOnSphereObstacle)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  const Eigen::Vector3d sphereCenter(0.1, 0.1, -0.3);
  const double sphereRadius = 0.5; // top of the sphere at z = 0.2
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  sphereOptions.position = sphereCenter;
  auto sphere = world.addRigidBody("obstacle_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(sphereRadius));
  setSurfaceObstaclePolicy(sphere);

  // A free spring patch released just above the top of the sphere.
  world.addDeformableBody("patch", makeFallingPatchOptions(0.28, 0.0));
  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 50;
  cfg.contactStiffness = 1.0e4;
  enableVbdConfig(world, cfg);

  const auto minSurfaceDistance = [&]() {
    const auto body = world.getDeformableBody("patch");
    double minimum = 1e9;
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      minimum = std::min(
          minimum, (body->getPosition(i) - sphereCenter).norm() - sphereRadius);
    }
    return minimum;
  };
  ASSERT_GT(minSurfaceDistance(), 0.02);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 400; ++step) {
    stepOnce(world, stage);
    const auto body = world.getDeformableBody("patch");
    ASSERT_TRUE(body.has_value());
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      ASSERT_TRUE(body->getPosition(i).allFinite()) << "blew up at " << step;
    }
  }

  // The VBD path (not the default solver) handled the body despite the
  // obstacle.
  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
  // The patch fell well below its release height onto the sphere ...
  const auto body = world.getDeformableBody("patch");
  double minZ = 1e9;
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    minZ = std::min(minZ, body->getPosition(i).z());
  }
  EXPECT_LT(minZ, 0.26);
  // ... and no node sinks through the sphere surface (soft-penalty tolerance).
  EXPECT_GT(minSurfaceDistance(), -3.0e-2);
}

//==============================================================================
// A VBD node exactly at a static sphere obstacle center has no radial direction
// to normalize, so it still gets a deterministic upward contact plane instead
// of being skipped as unconstrained.
TEST(VbdWorldSolver, VbdSphereObstacleRepelsCenterEmbeddedNode)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const double sphereRadius = 0.5;
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  auto sphere = world.addRigidBody("obstacle_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(sphereRadius));
  setSurfaceObstaclePolicy(sphere);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.velocities = {Eigen::Vector3d::Zero()};
  options.masses = {0.1};
  auto body = world.addDeformableBody("embedded_node", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 30;
  cfg.contactStiffness = 1.0e4;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
  const auto position = body.getPosition(0);
  EXPECT_TRUE(position.allFinite());
  EXPECT_NEAR(position.x(), 0.0, 1e-12);
  EXPECT_NEAR(position.y(), 0.0, 1e-12);
  EXPECT_GT(position.z(), 0.0);
}

//==============================================================================
// Option C: the same holds for a static box obstacle -- VBD keeps the patch on
// the box's top face without penetrating it.
TEST(VbdWorldSolver, VbdBodyRestsOnBoxObstacle)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  const Eigen::Vector3d boxCenter(0.1, 0.1, 0.0);
  const Eigen::Vector3d boxHalf(0.3, 0.3, 0.3); // top face at z = 0.3
  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  boxOptions.position = boxCenter;
  auto box = world.addRigidBody("obstacle_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox(boxHalf));
  setSurfaceObstaclePolicy(box);

  world.addDeformableBody("patch", makeFallingPatchOptions(0.38, 0.0));
  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 50;
  cfg.contactStiffness = 1.0e4;
  enableVbdConfig(world, cfg);

  const auto minBoxSurfaceDistance = [&]() {
    const auto body = world.getDeformableBody("patch");
    double minimum = 1e9;
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      const Eigen::Vector3d local = body->getPosition(i) - boxCenter;
      const Eigen::Vector3d signedOffset = local.cwiseAbs() - boxHalf;
      const Eigen::Vector3d outside
          = signedOffset.cwiseMax(Eigen::Vector3d::Zero());
      const double distance = outside.squaredNorm() > 0.0
                                  ? outside.norm()
                                  : signedOffset.maxCoeff();
      minimum = std::min(minimum, distance);
    }
    return minimum;
  };
  ASSERT_GT(minBoxSurfaceDistance(), 0.02);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 400; ++step) {
    stepOnce(world, stage);
    const auto body = world.getDeformableBody("patch");
    ASSERT_TRUE(body.has_value());
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      ASSERT_TRUE(body->getPosition(i).allFinite()) << "blew up at " << step;
    }
  }

  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
  // The patch fell onto the box top face (z = 0.3) ...
  const auto body = world.getDeformableBody("patch");
  double minZ = 1e9;
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    minZ = std::min(minZ, body->getPosition(i).z());
  }
  EXPECT_LT(minZ, 0.36);
  // ... without sinking through it (soft-penalty tolerance).
  EXPECT_GT(minBoxSurfaceDistance(), -3.0e-2);
}

//==============================================================================
// A VBD body that starts inside a static box obstacle still gets a lagged
// half-space plane from the nearest exit face, so the first solve moves it out
// instead of leaving it embedded until an outside-only barrier can arm.
TEST(VbdWorldSolver, VbdBoxObstacleRepelsInitiallyEmbeddedNode)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const Eigen::Vector3d boxHalf(0.5, 0.5, 0.5);
  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  auto box = world.addRigidBody("obstacle_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox(boxHalf));
  setSurfaceObstaclePolicy(box);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.45, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d::Zero()};
  options.masses = {0.1};
  auto body = world.addDeformableBody("embedded_node", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 30;
  cfg.contactStiffness = 1.0e4;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
  const auto position = body.getPosition(0);
  EXPECT_TRUE(position.allFinite());
  EXPECT_GT(position.x(), 0.49);
  EXPECT_NEAR(position.y(), 0.0, 1e-12);
  EXPECT_NEAR(position.z(), 0.0, 1e-12);
}

//==============================================================================
// The VBD static-obstacle barrier is lagged at the warm-start position, so fast
// nodes still need the shared static rigid-surface CCD limiter to prevent a
// one-step crossing that starts and ends outside the narrow contact band.
TEST(VbdWorldSolver, VbdStaticRigidSurfaceCcdLimitsFastCrossing)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  auto box = world.addRigidBody("static_box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
  setSurfaceObstaclePolicy(box);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(-1.0, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d(20.0, 0.0, 0.0)};
  options.masses = {1.0};
  auto body = world.addDeformableBody("fast_node", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 10;
  cfg.contactStiffness = 1.0e4;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
  EXPECT_LT(body.getPosition(0).x(), -0.05);
  EXPECT_GT(body.getPosition(0).x(), -1.0);
}

//==============================================================================
// VBD bodies with surface topology still need the shared inter-body CCD limiter
// after the block solve; otherwise two deformable surfaces can pass through
// each other in one large step.
TEST(VbdWorldSolver, VbdInterBodySurfaceCcdLimitsFastCrossing)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto moving
      = world.addDeformableBody("moving", makeInterBodyMovingPointOptions());
  world.addDeformableBody("obstacle", makeInterBodyTriangleObstacleOptions());

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 10;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 2u);
  EXPECT_GT(stats.interBodySurfaceContactCcdHits, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdLimitedSteps, 0u);
  EXPECT_GT(moving.getPosition(3).z(), 0.0);
  EXPECT_LT(moving.getPosition(3).z(), 1.0);
}

//==============================================================================
// VBD self-contact barriers are lagged, so fast same-body surface crossings
// still need the shared self-surface CCD limiter before write-back.
TEST(VbdWorldSolver, VbdSelfSurfaceCcdLimitsFastCrossing)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "self_crossing", makeSelfCrossingTriangleOptions());

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 10;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.vbdBodyCount, 1u);
  EXPECT_GT(stats.surfaceContactCcdHits, 0u);
  EXPECT_GT(stats.surfaceContactCcdLimitedSteps, 0u);
  for (std::size_t node = 3; node < 6; ++node) {
    EXPECT_GT(body.getPosition(node).z(), -1e-9) << "node " << node;
    EXPECT_LT(body.getPosition(node).z(), 1.0) << "node " << node;
  }
}

//==============================================================================
// VBD self-contact uses the same surface-node point mask as the default solver:
// volumetric nodes that are not referenced by the surface mesh must not receive
// point-triangle barrier forces from their own shell.
TEST(VbdWorldSolver, VbdSelfContactIgnoresInteriorTetNodes)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  const Eigen::Vector3d interiorStart(0.25, 0.25, 0.005);
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 1.0),
         interiorStart};
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 1.0);
  options.fixedNodes = {0, 1, 2, 3};
  options.tetrahedra = {sx::DeformableTetrahedron{0, 1, 2, 3}};
  options.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{0, 2, 3}};
  auto body = world.addDeformableBody("tet_with_interior_node", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 20;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  stepOnce(world, stage);

  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
  const Eigen::Vector3d interiorEnd = body.getPosition(4);
  EXPECT_NEAR((interiorEnd - interiorStart).norm(), 0.0, 1e-12);
}

//==============================================================================
// Option A: the World VBD path resists surface self-collision. A body's free
// top triangle falls toward its own pinned bottom triangle; the IPC point-
// triangle / edge-edge barrier (entered per-vertex during the colored sweeps)
// keeps the surfaces from interpenetrating.
TEST(VbdWorldSolver, VbdSelfContactPreventsInterpenetration)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);
  world.addDeformableBody("fold", makeSelfFoldingBody());
  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 60;
  enableVbdConfig(world, cfg);

  const auto topMinZ = [&]() {
    const auto body = world.getDeformableBody("fold");
    double minimum = 1e9;
    for (std::size_t i = 3; i < 6; ++i) {
      minimum = std::min(minimum, body->getPosition(i).z());
    }
    return minimum;
  };
  ASSERT_GT(topMinZ(), 0.05);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 500; ++step) {
    stepOnce(world, stage);
    const auto body = world.getDeformableBody("fold");
    ASSERT_TRUE(body.has_value());
    for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
      ASSERT_TRUE(body->getPosition(i).allFinite()) << "blew up at " << step;
    }
  }

  EXPECT_EQ(stage.getLastStats().vbdBodyCount, 1u);
  // The top triangle fell toward the bottom ...
  EXPECT_LT(topMinZ(), 0.10);
  // ... but self-contact kept it from passing through the bottom surface (z=0).
  EXPECT_GT(topMinZ(), -0.02);
}
