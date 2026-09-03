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
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/comps/name.hpp>
#include <dart/simulation/compute/detail/deformable_avbd_replay_state.hpp>
#include <dart/simulation/compute/detail/world_step_stages.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/frame/free_frame.hpp>
#include <dart/simulation/world.hpp>

#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace sx = dart::simulation;
namespace compute = dart::simulation::compute;

namespace {

class NoOpWorldStepStage final : public compute::WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "no-op";
  }

  void execute(sx::World&, compute::ComputeExecutor&) override
  {
    // Intentionally empty.
  }
};

class MutatingWorldStepStage final : public compute::WorldStepStage
{
public:
  explicit MutatingWorldStepStage(std::size_t& executionCount)
    : m_executionCount(executionCount)
  {
    // Empty.
  }

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "mutating";
  }

  void execute(sx::World& world, compute::ComputeExecutor&) override
  {
    ++m_executionCount;
    world.setTime(world.getTime() + 1.0);
  }

private:
  std::size_t& m_executionCount;
};

void expectDiagnosticsEqual(
    const sx::DeformableSolverDiagnostics& expected,
    const sx::DeformableSolverDiagnostics& actual)
{
#define DART_EXPECT_DIAGNOSTIC_EQ(field) EXPECT_EQ(expected.field, actual.field)
  DART_EXPECT_DIAGNOSTIC_EQ(bodyCount);
  DART_EXPECT_DIAGNOSTIC_EQ(nodeCount);
  DART_EXPECT_DIAGNOSTIC_EQ(edgeCount);
  DART_EXPECT_DIAGNOSTIC_EQ(vbdBodyCount);
  DART_EXPECT_DIAGNOSTIC_EQ(vbdSweeps);
  DART_EXPECT_DIAGNOSTIC_EQ(vbdVertexUpdates);
  DART_EXPECT_DIAGNOSTIC_EQ(solverIterations);
  DART_EXPECT_DIAGNOSTIC_EQ(objectiveEvaluations);
  DART_EXPECT_DIAGNOSTIC_EQ(lineSearchTrials);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonSteps);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonFallbacks);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonHessianNonZeros);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonHessianStorageBytes);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonIterativeSolves);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonMatrixFreeSolves);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonIterativeIterations);
  DART_EXPECT_DIAGNOSTIC_EQ(projectedNewtonIterativeMaxError);
  DART_EXPECT_DIAGNOSTIC_EQ(selfContactBarrierActiveContacts);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCandidateCapacityRequested);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCandidateCapacityResolved);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCandidateCountPeak);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCandidateBuilds);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCandidatePairCapacity);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCandidateRejectedPairs);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactPointTriangleCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactEdgeEdgeCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCcdPointTriangleChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCcdEdgeEdgeChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCcdHits);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCcdLimitedSteps);
  DART_EXPECT_DIAGNOSTIC_EQ(surfaceContactCcdZeroStepCount);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCandidateBuilds);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCandidatePairCapacity);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCandidateRejectedPairs);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactPointTriangleCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactEdgeEdgeCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCcdPointTriangleChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCcdEdgeEdgeChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCcdHits);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCcdLimitedSteps);
  DART_EXPECT_DIAGNOSTIC_EQ(interBodySurfaceContactCcdZeroStepCount);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdSnapshotBuilds);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdBoxCount);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdSphereCount);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdTriangleCount);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdEdgeCount);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdCandidateBuilds);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdCandidatePairCapacity);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdCandidateRejectedPairs);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdPointTriangleCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdEdgeEdgeCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdPointTriangleChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdEdgeEdgeChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdHits);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdLimitedSteps);
  DART_EXPECT_DIAGNOSTIC_EQ(staticRigidSurfaceCcdZeroStepCount);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdSnapshotBuilds);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdBoxCount);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdSampleCount);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdInflatedBoxCount);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdTriangleCount);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdEdgeCount);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdCandidateBuilds);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdCandidatePairCapacity);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdCandidateRejectedPairs);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdPointTriangleCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdEdgeEdgeCandidates);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdPointTriangleChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdEdgeEdgeChecks);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdHits);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdLimitedSteps);
  DART_EXPECT_DIAGNOSTIC_EQ(movingRigidSurfaceCcdZeroStepCount);
  DART_EXPECT_DIAGNOSTIC_EQ(frictionDissipation);
  DART_EXPECT_DIAGNOSTIC_EQ(minActiveContactDistance);
  DART_EXPECT_DIAGNOSTIC_EQ(convergedActiveContactCount);
  DART_EXPECT_DIAGNOSTIC_EQ(maxActiveContactCount);
#undef DART_EXPECT_DIAGNOSTIC_EQ
}

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
  options.material.useFiniteElementElasticity = true;
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
  options.material.useFiniteElementElasticity = true;
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

sx::DeformableBodyOptions makeDelayedSelfCrossingPointOptions(
    std::size_t movingPointCount,
    std::size_t candidateCapacity,
    double pointHeight = 1.0)
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  options.velocities.assign(3u, Eigen::Vector3d::Zero());
  options.masses.assign(3u, 1.0);
  options.fixedNodes = {0u, 1u, 2u};
  options.surfaceTriangles = {{0u, 1u, 2u}};
  for (std::size_t point = 0u; point < movingPointCount; ++point) {
    options.positions.emplace_back(
        0.1 * static_cast<double>(point), 0.0, pointHeight);
    options.velocities.emplace_back(0.0, 0.0, -20.0);
    options.masses.push_back(1.0);
  }
  options.surfaceContactCandidateCapacity = candidateCapacity;
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
// An explicit position edit relocates a model-fixed node's attachment anchor.
// Reusing the creation-time rest position would pull the node back to the
// origin as soon as AVBD attachment rows are enabled.
TEST(VbdWorldSolver, AvbdAttachmentTargetFollowsExplicitPositionEdit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  auto body = world.addDeformableBody("chain", makeChainOptions(3, 0.5));
  const Eigen::Vector3d relocated(1.25, -0.5, 0.75);
  body.setPosition(0, relocated);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  // The relocated node still carries the chain springs stretched by the edit,
  // so the augmented-Lagrangian attachment row has to drive a non-trivial
  // constraint force to zero. Its dual/penalty update converges geometrically
  // in the per-step sweep count, and 40 sweeps stop three orders of magnitude
  // short of the tolerance asserted below. Measured residual after 40 steps at
  // this scene's start stiffness (100) and beta (5000): 40 sweeps 2.7e-4,
  // 80 sweeps 2.2e-6, 100 sweeps 1.9e-7, 120 sweeps 1.7e-8. 120 keeps roughly
  // a decade of headroom under the 1e-6 bound on every step of the loop.
  cfg.iterations = 120;
  cfg.useAvbdAttachmentRows = true;
  cfg.avbdAttachmentStiffness = 100.0;
  cfg.avbdBeta = 5000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 40; ++step) {
    stepOnce(world, stage);
    ASSERT_EQ(stage.getLastStats().vbdAvbdAttachmentRows, 3u);
  }
  EXPECT_LT((body.getPosition(0) - relocated).norm(), 1.0e-6);
}

//==============================================================================
// For a moving Dirichlet target, Equation 18's previous constraint must pair
// x^t with T^t, not the next target T^(t+h). Otherwise regularization creates
// an artificial force opposite the scripted motion even when the node exactly
// follows its prescribed trajectory.
TEST(VbdWorldSolver, AvbdAttachmentUsesPreviousScriptedTarget)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  sx::DeformableDirichletBoundaryCondition boundary;
  boundary.nodes = {0};
  boundary.linearVelocity = Eigen::Vector3d(0.5, -0.25, 0.125);
  options.dirichletBoundaryConditions.push_back(boundary);
  const auto body = world.addDeformableBody("scripted", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 8;
  cfg.useAvbdAttachmentRows = true;
  cfg.avbdAttachmentStiffness = 100.0;
  cfg.avbdBeta = 1000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  for (int step = 1; step <= 50; ++step) {
    stepOnce(world, stage);
    ASSERT_EQ(stage.getLastStats().vbdAvbdAttachmentRows, 3u);
    const Eigen::Vector3d expected = static_cast<double>(step)
                                     * world.getTimeStep()
                                     * boundary.linearVelocity;
    EXPECT_LT((body.getPosition(0) - expected).norm(), 1.0e-12)
        << "step " << step;
  }
}

//==============================================================================
// A model-fixed vertex promoted to a compliant attachment remains eligible for
// static contact and post-solve CCD. Collecting planes with the original fixed
// mask would omit the normal row on exactly the node AVBD later unlocks.
TEST(VbdWorldSolver, AvbdAttachedVertexAlsoReceivesStaticContactRow)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  addGroundBarrier(world);

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)};
  options.masses = {1.0, 1.0};
  options.fixedNodes = {0};
  options.edges = {sx::DeformableEdge{0, 1, 0.2}};
  options.edgeStiffness = 2.0e4;
  const auto body = world.addDeformableBody("attached", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 80;
  cfg.contactStiffness = 2.0e4;
  cfg.useAvbdContactNormalRows = true;
  cfg.useAvbdAttachmentRows = true;
  cfg.avbdAttachmentStiffness = 2.0e4;
  cfg.avbdBeta = 5000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e7;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 20; ++step) {
    stepOnce(world, stage);
    ASSERT_EQ(stage.getLastStats().vbdAvbdAttachmentRows, 3u);
    ASSERT_GE(stage.getLastStats().vbdAvbdContactNormalRows, 1u);
    EXPECT_GE(body.getPosition(0).z(), -1.0e-6) << "step " << step;
  }
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
// The AVBD paper defines finite-stiffness updates only for force rows already
// expressible as E=0.5*k*C^2. DART's supported Neo-Hookean tetrahedral
// materials have no implemented independently rampable Equation-16
// formulation, so requesting that combination must fail closed instead of
// silently changing the material.
TEST(VbdWorldSolver, AvbdFiniteStiffnessRowsRejectTetrahedralMaterials)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.1);
  world.addDeformableBody("tet", makeTetOptions(1.0e6));

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 4;
  cfg.useAvbdFiniteStiffnessRows = true;
  cfg.avbdFiniteStiffnessStart = 0.02;
  cfg.avbdBeta = 1.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  EXPECT_THROW(stepOnce(world, stage), sx::InvalidArgumentException);
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
// Persistent AVBD contact rows identify the actual rigid obstacle and its
// collision-geometry revision. Family constants or collection-vector indices
// would alias different obstacles after insertion/removal, while omitting the
// revision would carry a warm start across a runtime shape edit.
TEST(VbdWorldSolver, AvbdContactRowsUseStableObstacleGeometryIdentity)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0e-4);

  for (int index = 0; index < 4; ++index) {
    sx::RigidBodyOptions options;
    options.isStatic = true;
    options.position = Eigen::Vector3d(10.0 + index, 0.0, 0.0);
    auto obstacle
        = world.addRigidBody("distractor_" + std::to_string(index), options);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
    setSurfaceObstaclePolicy(obstacle);
  }

  sx::RigidBodyOptions targetOptions;
  targetOptions.isStatic = true;
  auto target = world.addRigidBody("target", targetOptions);
  target.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  setSurfaceObstaclePolicy(target);

  sx::DeformableBodyOptions pointOptions;
  pointOptions.positions = {Eigen::Vector3d::Zero()};
  pointOptions.velocities = {Eigen::Vector3d::Zero()};
  pointOptions.masses = {1.0e12};
  world.addDeformableBody("point", pointOptions);

  sx::comps::DeformableVbdConfig config;
  config.enabled = true;
  config.iterations = 1;
  config.contactStiffness = 100.0;
  config.useAvbdContactNormalRows = true;
  config.avbdBeta = 2000.0;
  config.avbdGamma = 1.0;
  config.avbdMaxStiffness = 1.0e6;
  enableVbdConfig(world, config);

  world.enterSimulationMode();
  world.step();
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto expectedObjectId = static_cast<std::uint64_t>(entt::to_integral(
      dart::simulation::detail::toRegistryEntity(target.getEntity())));
  const auto before
      = compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          registry);
  ASSERT_EQ(before.size(), 1u);
  ASSERT_EQ(before[0].contactRows.size(), 1u);
  EXPECT_EQ(before[0].contactRows[0].descriptor.key.objectB, expectedObjectId);
  const std::uint64_t featureBefore
      = before[0].contactRows[0].descriptor.key.featureB;

  target.setCollisionShape(sx::CollisionShape::makeSphere(0.6));
  world.step();
  const auto after
      = compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          registry);
  ASSERT_EQ(after.size(), 1u);
  ASSERT_EQ(after[0].contactRows.size(), 1u);
  EXPECT_EQ(after[0].contactRows[0].descriptor.key.objectB, expectedObjectId);
  EXPECT_NE(after[0].contactRows[0].descriptor.key.featureB, featureBefore);
}

//==============================================================================
// A dual-policy obstacle can constrain a node either as a z-up ground height or
// as a full radial/oriented surface. Those are different constraints even when
// they come from the same entity, geometry revision, and primitive feature.
TEST(VbdWorldSolver, AvbdContactRowsDistinguishDualObstaclePolicyRoles)
{
  const auto contactFeatureAt = [](const Eigen::Vector3d& point) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(1.0e-4);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.isStatic = true;
    auto sphere = world.addRigidBody("dual_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    auto policy = sphere.getDeformableObstaclePolicy();
    policy.groundBarrier = true;
    policy.surfaceObstacle = true;
    sphere.setDeformableObstaclePolicy(policy);

    sx::DeformableBodyOptions pointOptions;
    pointOptions.positions = {point};
    pointOptions.velocities = {Eigen::Vector3d::Zero()};
    pointOptions.masses = {1.0e12};
    world.addDeformableBody("point", pointOptions);

    sx::comps::DeformableVbdConfig config;
    config.enabled = true;
    config.iterations = 1;
    config.contactStiffness = 100.0;
    config.useAvbdContactNormalRows = true;
    config.avbdBeta = 2000.0;
    config.avbdGamma = 1.0;
    config.avbdMaxStiffness = 1.0e6;
    enableVbdConfig(world, config);

    world.enterSimulationMode();
    world.step();
    const auto states
        = compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
            dart::simulation::detail::registryOf(world));
    if (states.size() != 1u || states[0].contactRows.size() != 1u) {
      ADD_FAILURE() << "expected one active AVBD contact row";
      return std::uint64_t{0};
    }
    return states[0].contactRows[0].descriptor.key.featureB;
  };

  const std::uint64_t groundRole = contactFeatureAt({0.0, 0.0, 0.49});
  const std::uint64_t surfaceRole = contactFeatureAt({0.51, 0.0, 0.0});
  EXPECT_NE(groundRole, surfaceRole);
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
// The finite-stiffness AVBD World slice is intentionally paper-grounded:
// tetrahedral materials fail closed because their known energy constraints
// cannot be ramped independently without breaking rest equilibrium, while
// unsupported mass-spring accelerators keep using the
// existing VBD path and report no AVBD rows.
TEST(VbdWorldSolver, AvbdFiniteStiffnessRowsRejectTetsAndFallbackForSprings)
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
    EXPECT_THROW(
        run(makeTetSpringOptions(1.0e5), baseConfig()),
        sx::InvalidArgumentException);
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
}

//==============================================================================
// Pure tetrahedral scenes retain their complete Neo-Hookean material while
// AVBD handles paper-defined self-contact normal/friction rows.
TEST(VbdWorldSolver, TetrahedralMaterialCombinesAvbdSelfContactFrictionRows)
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
  // Empty optional families must not disable the tetrahedral self-contact
  // AVBD route merely because their global toggles are enabled.
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
  EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
  EXPECT_GT(stats.vbdAvbdSelfContactNormalRows, 0u);
  EXPECT_EQ(
      stats.vbdAvbdFrictionTangentRows,
      2u * stats.vbdAvbdSelfContactNormalRows);
  EXPECT_EQ(stats.vbdAvbdAttachmentRows, 0u);
  EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);

  const auto body = world.getDeformableBody("tet-fold");
  ASSERT_TRUE(body.has_value());
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    EXPECT_TRUE(body->getPosition(i).allFinite()) << "node " << i;
  }
}

//==============================================================================
// A supported serial mass-spring ground-contact solve now keeps AVBD's friction
// tangents in the same row inventory as the contact-normal rows. Each active
// contact contributes two bounded tangent rows whose shared Coulomb limit is
// refreshed from the matching current normal force during every sweep.
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

  const auto body = world.getDeformableBody("patch");
  ASSERT_TRUE(body.has_value());
  for (std::size_t i = 0; i < body->getNodeCount(); ++i) {
    EXPECT_TRUE(body->getPosition(i).allFinite()) << "node " << i;
  }
}

//==============================================================================
// A partial AVBD dispatch must not erase a contact family that remains on the
// legacy VBD penalty path. When static contact-normal rows are not enabled, an
// otherwise active finite-stiffness request falls back as a whole.
TEST(VbdWorldSolver, AvbdPartialRowRequestPreservesLegacyStaticContact)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  const auto body
      = world.addDeformableBody("patch", makeFallingPatchOptions(0.0, 0.0));

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 40;
  cfg.contactStiffness = 5.0e3;
  cfg.useAvbdContactNormalRows = false;
  cfg.useAvbdFiniteStiffnessRows = true;
  cfg.avbdFiniteStiffnessStart = 20.0;
  cfg.avbdBeta = 2000.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 5.0e4;
  enableVbdConfig(world, cfg);

  compute::DeformableDynamicsStage stage;
  for (int step = 0; step < 80; ++step) {
    stepOnce(world, stage);
    const auto& stats = stage.getLastStats();
    ASSERT_EQ(stats.vbdBodyCount, 1u);
    EXPECT_EQ(stats.vbdAvbdContactNormalRows, 0u);
    EXPECT_EQ(stats.vbdAvbdFiniteStiffnessRows, 0u);
  }

  double minZ = std::numeric_limits<double>::infinity();
  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    minZ = std::min(minZ, body.getPosition(node).z());
  }
  EXPECT_GE(minZ, -1.0e-3);
}

//==============================================================================
// A contact-free tetrahedral body opts into VBD too: this exercises the
// block-coordinate Stable Neo-Hookean path independently of the default
// projected-Newton FEM path.
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
// gravity more, so its apex settles closer to the rest height on the VBD path.
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
// The default projected-Newton solver owns the same explicit FEM material
// contract. Keep a separate oracle so VBD coverage cannot mask a regression in
// the non-VBD path or imply that the default solver ignores Young's modulus.
TEST(VbdWorldSolver, DefaultProjectedNewtonHonorsTetrahedralMaterialStiffness)
{
  const auto settledApexHeight = [](double youngsModulus) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    world.addDeformableBody("tet", makeTetOptions(youngsModulus));

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 400; ++step) {
      stepOnce(world, stage);
    }
    EXPECT_EQ(stage.getLastStats().vbdBodyCount, 0u);
    const auto body = world.getDeformableBody("tet");
    return body->getPosition(3).z();
  };

  const double softApex = settledApexHeight(1.0e4);
  const double stiffApex = settledApexHeight(1.0e6);

  EXPECT_LT(softApex, 1.0);
  EXPECT_LT(stiffApex, 1.0);
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
// The public material contract keeps tetrahedral FEM opt-in. Selecting VBD
// must not silently turn a topology-only tetrahedron into an elastic element,
// nor make an otherwise force-free trajectory depend on Young's modulus.
TEST(VbdWorldSolver, FemOptOutDisablesTetrahedralElasticity)
{
  const auto run = [](double youngsModulus, bool keepTetTopology) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    sx::DeformableBodyOptions options = makeTetOptions(youngsModulus);
    options.material.useFiniteElementElasticity = false;
    if (!keepTetTopology) {
      options.tetrahedra.clear();
    }
    world.addDeformableBody("tet", options);
    enableVbd(world, 40);

    compute::DeformableDynamicsStage stage;
    for (int step = 0; step < 20; ++step) {
      stepOnce(world, stage);
    }
    return world.getDeformableBody("tet")->getPosition(3);
  };

  const Eigen::Vector3d soft = run(1.0e2, true);
  const Eigen::Vector3d stiff = run(1.0e8, true);
  const Eigen::Vector3d topologyFree = run(1.0e2, false);
  EXPECT_NEAR((soft - stiff).norm(), 0.0, 1e-12);
  EXPECT_NEAR((soft - topologyFree).norm(), 0.0, 1e-12);
  EXPECT_LT(soft.z(), 1.0);
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

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.vbdBodyCount, 1u);
  EXPECT_GT(diagnostics.vbdSweeps, 0u);
}

//==============================================================================
TEST(VbdWorldSolver, PublicConfigureThrowsForUnknownBody)
{
  sx::World world;
  sx::DeformableSolverOptions options;
  EXPECT_ANY_THROW(world.configureDeformableSolver("missing", options));
}

//==============================================================================
// Every documented public option range is enforced before the internal solver
// component is attached, so invalid input cannot silently disable or poison a
// VBD solve.
TEST(VbdWorldSolver, PublicConfigureRejectsInvalidOptionsWithoutMutation)
{
  sx::World world;
  world.addDeformableBody("chain", makeChainOptions(4, 0.5));
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto entity = *registry.view<sx::comps::DeformableBodyTag>().begin();

  const auto expectRejected = [&](const sx::DeformableSolverOptions& options) {
    EXPECT_THROW(
        world.configureDeformableSolver("chain", options),
        sx::InvalidArgumentException);
    EXPECT_FALSE(registry.all_of<sx::comps::DeformableVbdConfig>(entity));
  };

  sx::DeformableSolverOptions options;
  options.iterations = 0u;
  expectRejected(options);

  for (const double invalid :
       {-1.0,
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
    options = {};
    options.convergenceTolerance = invalid;
    expectRejected(options);

    options = {};
    options.stiffnessDamping = invalid;
    expectRejected(options);

    options = {};
    options.groundContactStiffness = invalid;
    expectRejected(options);
  }

  for (const double invalid :
       {0.0,
        1.0,
        -1.0,
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
    options = {};
    options.accelerationSpectralRadius = invalid;
    expectRejected(options);
  }

  options = {};
  options.iterations = 7u;
  world.configureDeformableSolver("chain", options);
  const auto& config = registry.get<sx::comps::DeformableVbdConfig>(entity);
  EXPECT_TRUE(config.enabled);
  EXPECT_TRUE(config.requireVbdExecution);
  EXPECT_EQ(config.iterations, 7u);
}

//==============================================================================
// Solver selection is a design-time operation. Rejecting late configuration
// prevents an unprepared first step and a stale resolved-configuration record.
TEST(VbdWorldSolver, PublicConfigureRejectsChangesAfterSimulationMode)
{
  sx::World world;
  world.addDeformableBody("chain", makeChainOptions(4, 0.5));
  world.enterSimulationMode();

  sx::DeformableSolverOptions options;
  EXPECT_THROW(
      world.configureDeformableSolver("chain", options),
      sx::InvalidOperationException);
}

//==============================================================================
// A required public solve must see the complete executable deformable layout.
// Registry corruption is rejected before simulation mode, time, or frame state
// changes instead of allowing a missing component to skip VBD or fail later in
// a stage after partial mutation.
TEST(VbdWorldSolver, PublicConfigureRejectsMissingExecutionComponentsAtEntry)
{
  enum class MissingComponent
  {
    NodeState,
    NodeModel,
    SpringModel,
    MeshTopology,
    Material,
  };
  struct MissingCase
  {
    std::string_view label;
    MissingComponent component;
  };
  const std::vector<MissingCase> cases{
      {"DeformableNodeState", MissingComponent::NodeState},
      {"DeformableNodeModel", MissingComponent::NodeModel},
      {"DeformableSpringModel", MissingComponent::SpringModel},
      {"DeformableMeshTopology", MissingComponent::MeshTopology},
      {"DeformableMaterial", MissingComponent::Material},
  };

  for (const auto& missingCase : cases) {
    SCOPED_TRACE(missingCase.label);
    sx::World world;
    world.addDeformableBody("chain", makeChainOptions(4, 0.5));
    sx::DeformableSolverOptions options;
    options.iterations = 8u;
    options.groundContactStiffness = 0.0;
    world.configureDeformableSolver("chain", options);
    auto& registry = dart::simulation::detail::registryOf(world);
    const auto entity = *registry.view<sx::comps::DeformableBodyTag>().begin();
    switch (missingCase.component) {
      case MissingComponent::NodeState:
        registry.remove<sx::comps::DeformableNodeState>(entity);
        break;
      case MissingComponent::NodeModel:
        registry.remove<sx::comps::DeformableNodeModel>(entity);
        break;
      case MissingComponent::SpringModel:
        registry.remove<sx::comps::DeformableSpringModel>(entity);
        break;
      case MissingComponent::MeshTopology:
        registry.remove<sx::comps::DeformableMeshTopology>(entity);
        break;
      case MissingComponent::Material:
        registry.remove<sx::comps::DeformableMaterial>(entity);
        break;
    }
    const double beforeTime = world.getTime();
    const std::size_t beforeFrame = world.getFrame();

    EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);

    EXPECT_FALSE(world.isSimulationMode());
    EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
    EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
    EXPECT_EQ(world.getFrame(), beforeFrame);
  }
}

//==============================================================================
// Required public VBD also validates cross-component cardinality and topology
// indices. These malformed layouts are structurally present, so they guard
// against shallow has-component-only preflight checks.
TEST(VbdWorldSolver, PublicConfigureRejectsCorruptExecutionLayoutAtEntry)
{
  enum class Corruption
  {
    StateCardinality,
    ModelCardinality,
    RestPositionCardinality,
    SpringIndex,
    TetrahedronIndex,
  };
  struct CorruptCase
  {
    std::string_view label;
    Corruption corruption;
  };
  const std::vector<CorruptCase> cases{
      {"state cardinality", Corruption::StateCardinality},
      {"model cardinality", Corruption::ModelCardinality},
      {"rest-position cardinality", Corruption::RestPositionCardinality},
      {"spring node index", Corruption::SpringIndex},
      {"tetrahedron node index", Corruption::TetrahedronIndex},
  };

  for (const auto& corruptCase : cases) {
    SCOPED_TRACE(corruptCase.label);
    sx::World world;
    if (corruptCase.corruption == Corruption::TetrahedronIndex) {
      world.addDeformableBody("body", makeTetOptions(1.0e5));
    } else {
      world.addDeformableBody("body", makeChainOptions(4, 0.5));
    }
    sx::DeformableSolverOptions options;
    options.iterations = 8u;
    options.groundContactStiffness = 0.0;
    world.configureDeformableSolver("body", options);
    auto& registry = dart::simulation::detail::registryOf(world);
    const auto entity = *registry.view<sx::comps::DeformableBodyTag>().begin();
    auto& state = registry.get<sx::comps::DeformableNodeState>(entity);
    auto& model = registry.get<sx::comps::DeformableNodeModel>(entity);
    auto& springs = registry.get<sx::comps::DeformableSpringModel>(entity);
    auto& topology = registry.get<sx::comps::DeformableMeshTopology>(entity);
    switch (corruptCase.corruption) {
      case Corruption::StateCardinality:
        state.velocities.pop_back();
        break;
      case Corruption::ModelCardinality:
        model.fixed.pop_back();
        break;
      case Corruption::RestPositionCardinality:
        topology.restPositions.pop_back();
        break;
      case Corruption::SpringIndex:
        ASSERT_FALSE(springs.edges.empty());
        springs.edges.front().nodeB = state.positions.size();
        break;
      case Corruption::TetrahedronIndex:
        ASSERT_FALSE(topology.tetrahedra.empty());
        topology.tetrahedra.front().nodeD = state.positions.size();
        break;
    }
    const double beforeTime = world.getTime();
    const std::size_t beforeFrame = world.getFrame();

    EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);

    EXPECT_FALSE(world.isSimulationMode());
    EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
    EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
    EXPECT_EQ(world.getFrame(), beforeFrame);
  }
}

//==============================================================================
// After using replay to return to a design boundary, end that immutable replay
// recording session before editing construction. Re-entering a required public
// VBD solve after a same-count spring edit must refresh the cached payload, not
// mistake equal vector cardinality for equal topology.
TEST(VbdWorldSolver, PublicRequiredVbdRefreshesSameCountSpringCacheAfterReplay)
{
  enum class Mutation
  {
    RestLength,
    Connectivity,
  };
  struct MutationCase
  {
    std::string_view label;
    Mutation mutation;
  };
  const std::vector<MutationCase> cases{
      {"rest length", Mutation::RestLength},
      {"connectivity", Mutation::Connectivity},
  };

  const auto configure = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);
    sx::DeformableBodyOptions bodyOptions;
    bodyOptions.positions
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 2.0, 0.0)};
    bodyOptions.masses = {1.0, 1.0, 1.0};
    bodyOptions.fixedNodes = {0};
    bodyOptions.edges = {{0, 1, 1.0}};
    bodyOptions.edgeStiffness = 500.0;
    world.addDeformableBody("body", bodyOptions);

    sx::DeformableSolverOptions solverOptions;
    solverOptions.iterations = 40u;
    solverOptions.useAcceleration = false;
    solverOptions.groundContactStiffness = 0.0;
    world.configureDeformableSolver("body", solverOptions);
  };

  const auto mutate = [](sx::World& world, Mutation mutation) {
    auto& registry = dart::simulation::detail::registryOf(world);
    const auto entity = *registry.view<sx::comps::DeformableBodyTag>().begin();
    auto& edge
        = registry.get<sx::comps::DeformableSpringModel>(entity).edges.front();
    if (mutation == Mutation::RestLength) {
      edge.restLength = 0.2;
    } else {
      edge.nodeA = 0u;
      edge.nodeB = 2u;
      edge.restLength = 0.5;
    }
  };

  const auto positions = [](sx::World& world) {
    std::vector<Eigen::Vector3d> result;
    const auto body = world.getDeformableBody("body");
    if (!body.has_value()) {
      return result;
    }
    result.reserve(body->getNodeCount());
    for (std::size_t node = 0; node < body->getNodeCount(); ++node) {
      result.push_back(body->getPosition(node));
    }
    return result;
  };

  for (const auto& mutationCase : cases) {
    SCOPED_TRACE(mutationCase.label);

    sx::World expectedWorld;
    configure(expectedWorld);
    mutate(expectedWorld, mutationCase.mutation);
    expectedWorld.step();
    const auto expected = positions(expectedWorld);
    ASSERT_EQ(expected.size(), 3u);
    EXPECT_EQ(
        expectedWorld.getLastDeformableSolverDiagnostics().vbdBodyCount, 1u);

    sx::World replayWorld;
    configure(replayWorld);
    replayWorld.setReplayRecordingEnabled(true);
    ASSERT_EQ(replayWorld.getReplayFrameCount(), 1u);
    replayWorld.step();
    ASSERT_EQ(replayWorld.getReplayFrameCount(), 2u);
    const auto unmutated = positions(replayWorld);
    ASSERT_EQ(unmutated.size(), expected.size());

    replayWorld.restoreReplayFrame(0u);
    ASSERT_FALSE(replayWorld.isSimulationMode());
    ASSERT_TRUE(replayWorld.getResolvedConfiguration().isEmpty());
    replayWorld.setReplayRecordingEnabled(false);
    mutate(replayWorld, mutationCase.mutation);
    replayWorld.step();
    const auto actual = positions(replayWorld);
    ASSERT_EQ(actual.size(), expected.size());
    EXPECT_EQ(
        replayWorld.getLastDeformableSolverDiagnostics().vbdBodyCount, 1u);

    double mutationEffect = 0.0;
    for (std::size_t node = 0; node < expected.size(); ++node) {
      EXPECT_TRUE(actual[node].isApprox(expected[node], 1e-12))
          << "node " << node;
      mutationEffect
          = std::max(mutationEffect, (expected[node] - unmutated[node]).norm());
    }
    EXPECT_GT(mutationEffect, 1e-3);
  }
}

//==============================================================================
// Rest-shape coordinates and tetrahedron indices are both semantic cache keys.
// Keep their vector sizes fixed, restore the same replay boundary, end the
// immutable recording session, and compare the warmed world against a fresh
// world whose first bake sees the edit.
TEST(VbdWorldSolver, PublicRequiredVbdRefreshesSameCountFemCacheAfterReplay)
{
  enum class Mutation
  {
    RestPositions,
    Connectivity,
  };
  struct MutationCase
  {
    std::string_view label;
    Mutation mutation;
  };
  const std::vector<MutationCase> cases{
      {"rest positions", Mutation::RestPositions},
      {"tet connectivity", Mutation::Connectivity},
  };

  const auto configure = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    sx::DeformableBodyOptions bodyOptions;
    bodyOptions.positions
        = {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0),
           Eigen::Vector3d(0.1, 0.1, 1.0),
           Eigen::Vector3d(0.2, 0.2, 1.4)};
    bodyOptions.masses = {1.0, 1.0, 1.0, 1.0, 1.0};
    bodyOptions.fixedNodes = {0, 1, 2};
    bodyOptions.tetrahedra = {{0, 1, 2, 3}};
    bodyOptions.material.youngsModulus = 5.0e4;
    bodyOptions.material.poissonRatio = 0.3;
    bodyOptions.material.useFiniteElementElasticity = true;
    world.addDeformableBody("body", bodyOptions);

    sx::DeformableSolverOptions solverOptions;
    solverOptions.iterations = 60u;
    solverOptions.useAcceleration = false;
    solverOptions.groundContactStiffness = 0.0;
    world.configureDeformableSolver("body", solverOptions);
  };

  const auto mutate = [](sx::World& world, Mutation mutation) {
    auto& registry = dart::simulation::detail::registryOf(world);
    const auto entity = *registry.view<sx::comps::DeformableBodyTag>().begin();
    auto& topology = registry.get<sx::comps::DeformableMeshTopology>(entity);
    auto& tet = topology.tetrahedra.front();
    if (mutation == Mutation::RestPositions) {
      topology.restPositions[3].z() = 0.5;
    } else {
      tet.nodeD = 4u;
    }
    const auto& a = topology.restPositions[tet.nodeA];
    const auto& b = topology.restPositions[tet.nodeB];
    const auto& c = topology.restPositions[tet.nodeC];
    const auto& d = topology.restPositions[tet.nodeD];
    topology.tetrahedronRestVolumes.front()
        = (b - a).cross(c - a).dot(d - a) / 6.0;
  };

  const auto positions = [](sx::World& world) {
    std::vector<Eigen::Vector3d> result;
    const auto body = world.getDeformableBody("body");
    if (!body.has_value()) {
      return result;
    }
    result.reserve(body->getNodeCount());
    for (std::size_t node = 0; node < body->getNodeCount(); ++node) {
      result.push_back(body->getPosition(node));
    }
    return result;
  };

  for (const auto& mutationCase : cases) {
    SCOPED_TRACE(mutationCase.label);

    sx::World expectedWorld;
    configure(expectedWorld);
    mutate(expectedWorld, mutationCase.mutation);
    expectedWorld.step();
    const auto expected = positions(expectedWorld);
    ASSERT_EQ(expected.size(), 5u);
    EXPECT_EQ(
        expectedWorld.getLastDeformableSolverDiagnostics().vbdBodyCount, 1u);

    sx::World replayWorld;
    configure(replayWorld);
    replayWorld.setReplayRecordingEnabled(true);
    ASSERT_EQ(replayWorld.getReplayFrameCount(), 1u);
    replayWorld.step();
    ASSERT_EQ(replayWorld.getReplayFrameCount(), 2u);
    const auto unmutated = positions(replayWorld);
    ASSERT_EQ(unmutated.size(), expected.size());

    replayWorld.restoreReplayFrame(0u);
    ASSERT_FALSE(replayWorld.isSimulationMode());
    ASSERT_TRUE(replayWorld.getResolvedConfiguration().isEmpty());
    replayWorld.setReplayRecordingEnabled(false);
    mutate(replayWorld, mutationCase.mutation);
    replayWorld.step();
    const auto actual = positions(replayWorld);
    ASSERT_EQ(actual.size(), expected.size());
    EXPECT_EQ(
        replayWorld.getLastDeformableSolverDiagnostics().vbdBodyCount, 1u);

    double mutationEffect = 0.0;
    for (std::size_t node = 0; node < expected.size(); ++node) {
      EXPECT_TRUE(actual[node].isApprox(expected[node], 1e-12))
          << "node " << node;
      mutationEffect
          = std::max(mutationEffect, (expected[node] - unmutated[node]).norm());
    }
    EXPECT_GT(mutationEffect, 1e-5);
  }
}

//==============================================================================
// Zero contact stiffness means what the public option says: the selected VBD
// solver remains selected and the deformable body falls through barriers
// instead of being silently substituted onto projected Newton.
TEST(VbdWorldSolver, PublicZeroContactStiffnessKeepsVbdAndIgnoresBarrier)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  addGroundBarrier(world);
  sx::RigidBodyOptions capsuleOptions;
  capsuleOptions.isStatic = true;
  capsuleOptions.position = Eigen::Vector3d(0.0, 0.0, -10.0);
  auto capsule = world.addRigidBody("ignored-capsule", capsuleOptions);
  capsule.setCollisionShape(
      sx::CollisionShape::makeCapsule(/*radius=*/0.3, /*halfHeight=*/0.4));
  setSurfaceObstaclePolicy(capsule);
  world.addDeformableBody("patch", makeFallingPatchOptions(0.05, 0.0));

  sx::DeformableSolverOptions options;
  options.iterations = 10u;
  options.groundContactStiffness = 0.0;
  world.configureDeformableSolver("patch", options);
  world.enterSimulationMode();
  for (int step = 0; step < 40; ++step) {
    world.step();
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.vbdBodyCount, 1u);
  EXPECT_GT(diagnostics.vbdSweeps, 0u);
  EXPECT_EQ(diagnostics.solverIterations, 0u);
  const auto body = world.getDeformableBody("patch");
  ASSERT_TRUE(body.has_value());
  double maximumZ = -std::numeric_limits<double>::infinity();
  for (std::size_t node = 0; node < body->getNodeCount(); ++node) {
    maximumZ = std::max(maximumZ, body->getPosition(node).z());
  }
  EXPECT_LT(maximumZ, -0.1);
}

//==============================================================================
// Zero public contact stiffness disables the entire tagged rigid-obstacle
// envelope, not only the currently supported sphere/box collectors. Every
// shape and motion variant below overlaps the falling patch initially. Its
// complete deformable trajectory and public diagnostics must remain identical
// to an obstacle-free baseline under both executor implementations. The rigid
// bodies still follow the selected rigid-family schedule: free bodies advance,
// while sequential-impulse kinematic bodies remain prescribed and immobile.
TEST(VbdWorldSolver, PublicZeroContactIgnoresCompleteRigidObstacleEnvelope)
{
  enum class Shape
  {
    Sphere,
    Box,
    Capsule,
    Cylinder,
    Plane,
    Mesh,
    Compound,
  };
  enum class Motion
  {
    Static,
    Free,
    Kinematic,
  };
  struct ObstacleCase
  {
    std::string_view label;
    Shape shape;
    Motion motion;
    bool groundBarrier;
  };
  const std::vector<ObstacleCase> cases{
      {"static sphere", Shape::Sphere, Motion::Static, true},
      {"static box", Shape::Box, Motion::Static, false},
      {"static capsule", Shape::Capsule, Motion::Static, true},
      {"static cylinder", Shape::Cylinder, Motion::Static, false},
      {"static plane", Shape::Plane, Motion::Static, true},
      {"static mesh", Shape::Mesh, Motion::Static, false},
      {"static compound", Shape::Compound, Motion::Static, true},
      {"free sphere", Shape::Sphere, Motion::Free, false},
      {"free capsule", Shape::Capsule, Motion::Free, true},
      {"kinematic box", Shape::Box, Motion::Kinematic, true},
      {"kinematic mesh", Shape::Mesh, Motion::Kinematic, false},
  };

  struct RunResult
  {
    std::vector<Eigen::Vector3d> trajectory;
    std::vector<sx::DeformableSolverDiagnostics> diagnostics;
    Eigen::Vector3d obstacleStart = Eigen::Vector3d::Zero();
    Eigen::Vector3d obstacleEnd = Eigen::Vector3d::Zero();
    Eigen::Vector3d obstacleVelocityStart = Eigen::Vector3d::Zero();
    Eigen::Vector3d obstacleVelocityEnd = Eigen::Vector3d::Zero();
  };

  const auto run = [](const ObstacleCase* obstacleCase, bool useParallel) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);

    if (obstacleCase != nullptr) {
      sx::RigidBodyOptions obstacleOptions;
      obstacleOptions.isStatic = obstacleCase->motion == Motion::Static;
      obstacleOptions.position = Eigen::Vector3d(0.1, 0.1, 0.04);
      obstacleOptions.linearVelocity = Eigen::Vector3d(0.25, 0.0, 0.0);
      auto obstacle = world.addRigidBody("ignored-obstacle", obstacleOptions);
      switch (obstacleCase->shape) {
        case Shape::Sphere:
          obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.3));
          break;
        case Shape::Box:
          obstacle.setCollisionShape(
              sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.3)));
          break;
        case Shape::Capsule:
          obstacle.setCollisionShape(
              sx::CollisionShape::makeCapsule(0.3, 0.35));
          break;
        case Shape::Cylinder:
          obstacle.setCollisionShape(
              sx::CollisionShape::makeCylinder(0.3, 0.35));
          break;
        case Shape::Plane:
          obstacle.setCollisionShape(
              sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));
          break;
        case Shape::Mesh:
          obstacle.setCollisionShape(
              sx::CollisionShape::makeMesh(
                  {Eigen::Vector3d(-0.5, -0.5, 0.0),
                   Eigen::Vector3d(0.5, -0.5, 0.0),
                   Eigen::Vector3d(0.0, 0.5, 0.0)},
                  {Eigen::Vector3i(0, 1, 2)}));
          break;
        case Shape::Compound:
          obstacle.setCollisionShape(
              sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.25)));
          obstacle.addCollisionShape(sx::CollisionShape::makeSphere(0.3));
          break;
      }
      if (obstacleCase->motion == Motion::Kinematic) {
        obstacle.setKinematic(true);
      }
      if (obstacleCase->groundBarrier) {
        setGroundBarrierPolicy(obstacle);
      } else {
        setSurfaceObstaclePolicy(obstacle);
      }
    }

    world.addDeformableBody("patch", makeFallingPatchOptions(0.05, 0.0));
    sx::DeformableSolverOptions options;
    options.iterations = 8u;
    options.useAcceleration = false;
    options.groundContactStiffness = 0.0;
    world.configureDeformableSolver("patch", options);
    world.enterSimulationMode();

    RunResult result;
    if (obstacleCase != nullptr) {
      result.obstacleStart = world.getRigidBody("ignored-obstacle")
                                 ->getTransform()
                                 .translation();
      result.obstacleVelocityStart
          = world.getRigidBody("ignored-obstacle")->getLinearVelocity();
    }
    compute::SequentialExecutor sequentialExecutor;
    compute::ParallelExecutor parallelExecutor(4u);
    compute::ComputeExecutor* executor
        = useParallel
              ? static_cast<compute::ComputeExecutor*>(&parallelExecutor)
              : static_cast<compute::ComputeExecutor*>(&sequentialExecutor);
    constexpr std::size_t stepCount = 8u;
    const auto body = world.getDeformableBody("patch");
    EXPECT_TRUE(body.has_value());
    if (!body.has_value()) {
      return result;
    }
    result.trajectory.reserve(stepCount * body->getNodeCount());
    result.diagnostics.reserve(stepCount);
    for (std::size_t step = 0; step < stepCount; ++step) {
      world.step(*executor);
      for (std::size_t node = 0; node < body->getNodeCount(); ++node) {
        result.trajectory.push_back(body->getPosition(node));
      }
      result.diagnostics.push_back(world.getLastDeformableSolverDiagnostics());
    }
    if (obstacleCase != nullptr) {
      result.obstacleEnd = world.getRigidBody("ignored-obstacle")
                               ->getTransform()
                               .translation();
      result.obstacleVelocityEnd
          = world.getRigidBody("ignored-obstacle")->getLinearVelocity();
    }
    return result;
  };

  for (const bool useParallel : {false, true}) {
    const RunResult baseline = run(nullptr, useParallel);
    for (const auto& obstacleCase : cases) {
      SCOPED_TRACE(
          ::testing::Message() << (useParallel ? "parallel: " : "sequential: ")
                               << obstacleCase.label);
      const RunResult actual = run(&obstacleCase, useParallel);
      ASSERT_EQ(actual.trajectory.size(), baseline.trajectory.size());
      for (std::size_t i = 0; i < baseline.trajectory.size(); ++i) {
        EXPECT_TRUE(
            actual.trajectory[i].isApprox(baseline.trajectory[i], 1e-12));
      }
      ASSERT_EQ(actual.diagnostics.size(), baseline.diagnostics.size());
      for (std::size_t i = 0; i < baseline.diagnostics.size(); ++i) {
        SCOPED_TRACE(::testing::Message() << "step " << i);
        expectDiagnosticsEqual(baseline.diagnostics[i], actual.diagnostics[i]);
        EXPECT_EQ(actual.diagnostics[i].vbdBodyCount, 1u);
        EXPECT_GT(actual.diagnostics[i].vbdSweeps, 0u);
        EXPECT_EQ(actual.diagnostics[i].solverIterations, 0u);
        EXPECT_EQ(
            actual.diagnostics[i].staticRigidSurfaceCcdSnapshotBuilds, 0u);
        EXPECT_EQ(
            actual.diagnostics[i].movingRigidSurfaceCcdSnapshotBuilds, 0u);
      }

      EXPECT_DOUBLE_EQ(
          actual.obstacleVelocityEnd.x(), actual.obstacleVelocityStart.x());
      if (obstacleCase.motion != Motion::Free) {
        EXPECT_EQ(actual.obstacleEnd, actual.obstacleStart);
        EXPECT_EQ(actual.obstacleVelocityEnd, actual.obstacleVelocityStart);
      } else {
        EXPECT_GT(actual.obstacleEnd.x(), actual.obstacleStart.x());
      }
    }
  }
}

//==============================================================================
// The zero-contact collector fast path is valid only when every executable
// deformable body has the public ignore-obstacles contract. In a mixed world,
// collectors remain active for a positive-contact peer while the zero-contact
// body's trajectory still matches an obstacle-free world exactly.
TEST(VbdWorldSolver, PublicZeroContactMixedWorldRetainsRequiredCollectors)
{
  sx::World mixedWorld;
  mixedWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  mixedWorld.setTimeStep(0.005);
  const Eigen::Vector3d sphereCenter(0.1, 0.1, -0.3);
  constexpr double sphereRadius = 0.5;
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  sphereOptions.position = sphereCenter;
  auto sphere = mixedWorld.addRigidBody("obstacle", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(sphereRadius));
  setSurfaceObstaclePolicy(sphere);
  mixedWorld.addDeformableBody(
      "zero-contact", makeFallingPatchOptions(0.28, 0.0));
  mixedWorld.addDeformableBody(
      "positive-contact", makeFallingPatchOptions(0.28, 0.0));
  sx::DeformableSolverOptions zeroOptions;
  zeroOptions.iterations = 20u;
  zeroOptions.useAcceleration = false;
  zeroOptions.groundContactStiffness = 0.0;
  mixedWorld.configureDeformableSolver("zero-contact", zeroOptions);
  sx::DeformableSolverOptions positiveOptions = zeroOptions;
  positiveOptions.iterations = 40u;
  positiveOptions.groundContactStiffness = 1.0e4;
  mixedWorld.configureDeformableSolver("positive-contact", positiveOptions);

  sx::World baselineWorld;
  baselineWorld.setGravity(mixedWorld.getGravity());
  baselineWorld.setTimeStep(mixedWorld.getTimeStep());
  baselineWorld.addDeformableBody(
      "zero-contact", makeFallingPatchOptions(0.28, 0.0));
  baselineWorld.configureDeformableSolver("zero-contact", zeroOptions);

  mixedWorld.enterSimulationMode();
  baselineWorld.enterSimulationMode();
  const auto mixedZero = mixedWorld.getDeformableBody("zero-contact");
  const auto mixedPositive = mixedWorld.getDeformableBody("positive-contact");
  const auto baselineZero = baselineWorld.getDeformableBody("zero-contact");
  ASSERT_TRUE(mixedZero.has_value());
  ASSERT_TRUE(mixedPositive.has_value());
  ASSERT_TRUE(baselineZero.has_value());
  ASSERT_EQ(mixedZero->getNodeCount(), baselineZero->getNodeCount());

  constexpr std::size_t stepCount = 120u;
  for (std::size_t step = 0; step < stepCount; ++step) {
    mixedWorld.step();
    baselineWorld.step();
    SCOPED_TRACE(::testing::Message() << "step " << step);
    for (std::size_t node = 0; node < mixedZero->getNodeCount(); ++node) {
      EXPECT_TRUE(mixedZero->getPosition(node).isApprox(
          baselineZero->getPosition(node), 1e-12));
    }
    const auto& diagnostics = mixedWorld.getLastDeformableSolverDiagnostics();
    EXPECT_EQ(diagnostics.vbdBodyCount, 2u);
    EXPECT_GT(diagnostics.staticRigidSurfaceCcdSnapshotBuilds, 0u);
  }

  double zeroMaximumZ = -std::numeric_limits<double>::infinity();
  double positiveMinimumDistance = std::numeric_limits<double>::infinity();
  for (std::size_t node = 0; node < mixedZero->getNodeCount(); ++node) {
    zeroMaximumZ = std::max(zeroMaximumZ, mixedZero->getPosition(node).z());
    positiveMinimumDistance = std::min(
        positiveMinimumDistance,
        (mixedPositive->getPosition(node) - sphereCenter).norm()
            - sphereRadius);
  }
  EXPECT_LT(zeroMaximumZ, -0.5);
  EXPECT_GT(positiveMinimumDistance, -3.0e-2);
}

//==============================================================================
// A zero-contact public VBD body deliberately ignores tagged rigid obstacles,
// but those bodies must still follow the rigid-domain schedule. Runtime
// static-to-dynamic/kinematic edits invalidate and rebuild that schedule before
// the next sequential or parallel step.
TEST(VbdWorldSolver, RuntimeIgnoredObstacleMutationRefreshesStepSchedule)
{
  const auto configureWorld = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.01);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = Eigen::Vector3d(5.0, 0.0, 0.0);
    obstacleOptions.linearVelocity = Eigen::Vector3d::UnitX();
    auto obstacle = world.addRigidBody("ignored-obstacle", obstacleOptions);
    obstacle.setCollisionShape(
        sx::CollisionShape::makeCapsule(/*radius=*/0.3, /*halfHeight=*/0.4));
    setSurfaceObstaclePolicy(obstacle);

    world.addDeformableBody("chain", makeChainOptions(2, 0.5));
    sx::DeformableSolverOptions options;
    options.iterations = 8u;
    options.useAcceleration = false;
    options.groundContactStiffness = 0.0;
    world.configureDeformableSolver("chain", options);
  };

  enum class StepOverload
  {
    Executor,
    CountExecutor,
    FinalStage,
    CountFinalStage,
  };
  struct StepCase
  {
    std::string_view label;
    StepOverload overload;
  };
  const std::vector<StepCase> stepCases{
      {"executor", StepOverload::Executor},
      {"count + executor", StepOverload::CountExecutor},
      {"executor + final stage", StepOverload::FinalStage},
      {"count + executor + final stage", StepOverload::CountFinalStage},
  };

  for (const bool useParallel : {false, true}) {
    for (const bool makeKinematic : {false, true}) {
      for (const auto& stepCase : stepCases) {
        SCOPED_TRACE(
            ::testing::Message()
            << (useParallel ? "parallel, " : "sequential, ")
            << (makeKinematic ? "kinematic, " : "dynamic, ") << stepCase.label);
        sx::World world;
        configureWorld(world);
        world.enterSimulationMode();
        auto obstacle = world.getRigidBody("ignored-obstacle");
        ASSERT_TRUE(obstacle.has_value());
        ASSERT_FALSE(world.getResolvedConfiguration().isEmpty());
        if (makeKinematic) {
          obstacle->setKinematic(true);
        } else {
          obstacle->setStatic(false);
        }
        EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
        const double beforeX = obstacle->getTransform().translation().x();
        const double beforeTime = world.getTime();
        const std::size_t beforeFrame = world.getFrame();

        compute::SequentialExecutor sequentialExecutor;
        compute::ParallelExecutor parallelExecutor(4u);
        compute::ComputeExecutor* executor
            = useParallel
                  ? static_cast<compute::ComputeExecutor*>(&parallelExecutor)
                  : static_cast<compute::ComputeExecutor*>(&sequentialExecutor);
        std::size_t finalStageExecutionCount = 0u;
        MutatingWorldStepStage finalStage(finalStageExecutionCount);
        const bool usesCallerOwnedFinalStage
            = stepCase.overload == StepOverload::FinalStage
              || stepCase.overload == StepOverload::CountFinalStage;
        switch (stepCase.overload) {
          case StepOverload::Executor:
            world.step(*executor);
            break;
          case StepOverload::CountExecutor:
            world.step(1u, *executor);
            break;
          case StepOverload::FinalStage:
            EXPECT_THROW(
                world.step(*executor, finalStage),
                sx::InvalidOperationException);
            break;
          case StepOverload::CountFinalStage:
            EXPECT_THROW(
                world.step(1u, *executor, finalStage),
                sx::InvalidOperationException);
            break;
        }

        if (usesCallerOwnedFinalStage) {
          // Required public deformable VBD owns the whole step schedule, so
          // the caller-owned final-stage overloads are rejected by design
          // rather than refreshing the schedule. The rejection is atomic: the
          // stage never runs, the runtime-mutated rigid body never advances,
          // and the resolved configuration stays unresolved because no rebake
          // happened.
          EXPECT_EQ(finalStageExecutionCount, 0u);
          EXPECT_EQ(obstacle->getTransform().translation().x(), beforeX);
          EXPECT_EQ(world.getFrame(), beforeFrame);
          EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
          EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
          continue;
        }

        const double positionDelta
            = obstacle->getTransform().translation().x() - beforeX;
        if (makeKinematic) {
          EXPECT_NEAR(positionDelta, 0.0, 1e-12);
          EXPECT_EQ(obstacle->getLinearVelocity(), Eigen::Vector3d::UnitX());
        } else {
          EXPECT_GT(positionDelta, 0.0);
          EXPECT_NEAR(
              positionDelta,
              world.getTimeStep() * obstacle->getLinearVelocity().x(),
              1e-12);
        }
        EXPECT_EQ(world.getFrame(), beforeFrame + 1u);
        EXPECT_DOUBLE_EQ(world.getTime(), beforeTime + world.getTimeStep());
        EXPECT_FALSE(world.getResolvedConfiguration().isEmpty());
        const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
        EXPECT_EQ(diagnostics.vbdBodyCount, 1u);
        EXPECT_GT(diagnostics.vbdSweeps, 0u);
        EXPECT_EQ(diagnostics.solverIterations, 0u);
      }
    }
  }

  // Required public VBD deliberately does not permit arbitrary caller-owned
  // pipelines. After a runtime model mutation, both pipeline overloads still
  // reject before rebaking, advancing the rigid body, or executing any stage.
  for (const bool useParallel : {false, true}) {
    for (const bool makeKinematic : {false, true}) {
      for (const bool useCountOverload : {false, true}) {
        SCOPED_TRACE(
            ::testing::Message()
            << (useParallel ? "parallel, " : "sequential, ")
            << (makeKinematic ? "kinematic, " : "dynamic, ")
            << (useCountOverload ? "count + caller pipeline"
                                 : "caller pipeline"));
        sx::World world;
        configureWorld(world);
        world.enterSimulationMode();
        auto obstacle = world.getRigidBody("ignored-obstacle");
        auto chain = world.getDeformableBody("chain");
        ASSERT_TRUE(obstacle.has_value());
        ASSERT_TRUE(chain.has_value());
        if (makeKinematic) {
          obstacle->setKinematic(true);
        } else {
          obstacle->setStatic(false);
        }
        ASSERT_TRUE(world.getResolvedConfiguration().isEmpty());
        const Eigen::Vector3d obstaclePosition
            = obstacle->getTransform().translation();
        const Eigen::Vector3d chainPosition = chain->getPosition(1);
        const double beforeTime = world.getTime();
        const std::size_t beforeFrame = world.getFrame();

        std::size_t executionCount = 0u;
        MutatingWorldStepStage mutatingStage(executionCount);
        compute::DeformableDynamicsStage deformable;
        compute::WorldStepPipeline pipeline;
        pipeline.addStage(mutatingStage);
        pipeline.addStage(deformable);
        compute::SequentialExecutor sequentialExecutor;
        compute::ParallelExecutor parallelExecutor(4u);
        compute::ComputeExecutor* executor
            = useParallel
                  ? static_cast<compute::ComputeExecutor*>(&parallelExecutor)
                  : static_cast<compute::ComputeExecutor*>(&sequentialExecutor);

        if (useCountOverload) {
          EXPECT_THROW(
              world.step(1u, *executor, pipeline),
              sx::InvalidOperationException);
        } else {
          EXPECT_THROW(
              world.step(*executor, pipeline), sx::InvalidOperationException);
        }

        EXPECT_EQ(executionCount, 0u);
        EXPECT_EQ(obstacle->getTransform().translation(), obstaclePosition);
        EXPECT_EQ(chain->getPosition(1), chainPosition);
        EXPECT_EQ(world.getFrame(), beforeFrame);
        EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
        EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
      }
    }
  }
}

//==============================================================================
// A public required-solver selection cannot be bypassed by any arbitrary
// caller-owned pipeline. Reject both pipeline overloads before entering
// simulation mode or executing a stage, regardless of whether the pipeline is
// empty, unrelated, appears to contain a valid deformable solve, duplicates
// that solve, or places a mutator before it.
TEST(VbdWorldSolver, PublicRequiredVbdRejectsEveryCallerPipelineAtomically)
{
  enum class PipelineCase
  {
    Empty,
    NoOpOnly,
    SingleDeformable,
    DuplicateDeformable,
    MutatorBeforeDeformable,
  };
  struct InvalidCase
  {
    std::string_view label;
    PipelineCase pipelineCase;
  };
  const std::vector<InvalidCase> cases{
      {"empty", PipelineCase::Empty},
      {"no-op only", PipelineCase::NoOpOnly},
      {"single deformable stage", PipelineCase::SingleDeformable},
      {"duplicate deformable stages", PipelineCase::DuplicateDeformable},
      {"mutator before deformable", PipelineCase::MutatorBeforeDeformable},
  };

  for (const auto& invalidCase : cases) {
    for (const bool useCountOverload : {false, true}) {
      SCOPED_TRACE(
          ::testing::Message()
          << invalidCase.label
          << (useCountOverload ? ", count overload" : ", one step"));
      sx::World world;
      world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
      world.setTimeStep(0.01);
      auto chain = world.addDeformableBody("chain", makeChainOptions(4, 0.5));
      sx::DeformableSolverOptions options;
      options.iterations = 8u;
      options.useAcceleration = false;
      options.groundContactStiffness = 0.0;
      world.configureDeformableSolver("chain", options);

      std::vector<Eigen::Vector3d> positions;
      positions.reserve(chain.getNodeCount());
      for (std::size_t node = 0; node < chain.getNodeCount(); ++node) {
        positions.push_back(chain.getPosition(node));
      }
      const double beforeTime = world.getTime();
      const std::size_t beforeFrame = world.getFrame();

      std::size_t executionCount = 0u;
      NoOpWorldStepStage noOp;
      MutatingWorldStepStage mutatingStage(executionCount);
      compute::DeformableDynamicsStage deformableA;
      compute::DeformableDynamicsStage deformableB;
      compute::WorldStepPipeline pipeline;
      switch (invalidCase.pipelineCase) {
        case PipelineCase::Empty:
          break;
        case PipelineCase::NoOpOnly:
          pipeline.addStage(noOp);
          break;
        case PipelineCase::SingleDeformable:
          pipeline.addStage(deformableA);
          break;
        case PipelineCase::DuplicateDeformable:
          pipeline.addStage(deformableA);
          pipeline.addStage(deformableB);
          break;
        case PipelineCase::MutatorBeforeDeformable:
          pipeline.addStage(mutatingStage);
          pipeline.addStage(deformableA);
          break;
      }
      const std::size_t stageCount = pipeline.getStageCount();
      compute::SequentialExecutor executor;

      if (useCountOverload) {
        EXPECT_THROW(
            world.step(1u, executor, pipeline), sx::InvalidOperationException);
      } else {
        EXPECT_THROW(
            world.step(executor, pipeline), sx::InvalidOperationException);
      }

      EXPECT_EQ(executionCount, 0u);
      EXPECT_FALSE(world.isSimulationMode());
      EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
      EXPECT_EQ(world.getFrame(), beforeFrame);
      EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
      EXPECT_EQ(pipeline.getStageCount(), stageCount);
      for (std::size_t node = 0; node < chain.getNodeCount(); ++node) {
        EXPECT_EQ(chain.getPosition(node), positions[node]);
      }
    }
  }
}

//==============================================================================
// Positive-contact public VBD supports only rigid obstacles attached directly
// to the world frame. A parented obstacle is rejected before entry and without
// changing either hierarchy/physical state or simulation counters.
TEST(VbdWorldSolver, PublicConfigureRejectsParentedObstacleAtEntryAtomically)
{
  sx::World world;
  auto mount = world.addFreeFrame("mount");
  Eigen::Isometry3d mountTransform = Eigen::Isometry3d::Identity();
  mountTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  mount.setLocalTransform(mountTransform);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.5)));
  setSurfaceObstaclePolicy(obstacle);
  obstacle.setParentFrame(mount);

  auto patch
      = world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);

  const Eigen::Isometry3d mountState = mount.getTransform();
  const Eigen::Isometry3d obstacleTransform = obstacle.getTransform();
  const Eigen::Isometry3d obstacleLocalTransform = obstacle.getLocalTransform();
  const auto parentEntity = obstacle.getParentFrame().getEntity();
  std::vector<Eigen::Vector3d> patchPositions;
  patchPositions.reserve(patch.getNodeCount());
  for (std::size_t node = 0; node < patch.getNodeCount(); ++node) {
    patchPositions.push_back(patch.getPosition(node));
  }
  const double beforeTime = world.getTime();
  const std::size_t beforeFrame = world.getFrame();

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
  EXPECT_EQ(world.getFrame(), beforeFrame);
  EXPECT_EQ(obstacle.getParentFrame().getEntity(), parentEntity);
  EXPECT_TRUE(mount.getTransform().isApprox(mountState, 0.0));
  EXPECT_TRUE(obstacle.getTransform().isApprox(obstacleTransform, 0.0));
  EXPECT_TRUE(
      obstacle.getLocalTransform().isApprox(obstacleLocalTransform, 0.0));
  for (std::size_t node = 0; node < patch.getNodeCount(); ++node) {
    EXPECT_EQ(patch.getPosition(node), patchPositions[node]);
  }
}

//==============================================================================
// Reparenting a previously valid positive-contact obstacle invalidates the
// baked solver identity. The next step must reject the now-unsupported
// hierarchy before any rigid/deformable stage or clock mutation.
TEST(VbdWorldSolver, PublicConfigureRejectsRuntimeObstacleReparentAtomically)
{
  sx::World world;
  auto mount = world.addFreeFrame("mount");
  Eigen::Isometry3d mountTransform = Eigen::Isometry3d::Identity();
  mountTransform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  mount.setLocalTransform(mountTransform);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.5)));
  setSurfaceObstaclePolicy(obstacle);
  auto patch
      = world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);
  world.enterSimulationMode();
  ASSERT_FALSE(world.getResolvedConfiguration().isEmpty());

  obstacle.setParentFrame(mount);
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  const Eigen::Isometry3d mountState = mount.getTransform();
  const Eigen::Isometry3d obstacleTransform = obstacle.getTransform();
  const Eigen::Isometry3d obstacleLocalTransform = obstacle.getLocalTransform();
  const auto parentEntity = obstacle.getParentFrame().getEntity();
  std::vector<Eigen::Vector3d> patchPositions;
  patchPositions.reserve(patch.getNodeCount());
  for (std::size_t node = 0; node < patch.getNodeCount(); ++node) {
    patchPositions.push_back(patch.getPosition(node));
  }
  const double beforeTime = world.getTime();
  const std::size_t beforeFrame = world.getFrame();

  EXPECT_THROW(world.step(), sx::InvalidArgumentException);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
  EXPECT_EQ(world.getFrame(), beforeFrame);
  EXPECT_EQ(obstacle.getParentFrame().getEntity(), parentEntity);
  EXPECT_TRUE(mount.getTransform().isApprox(mountState, 0.0));
  EXPECT_TRUE(obstacle.getTransform().isApprox(obstacleTransform, 0.0));
  EXPECT_TRUE(
      obstacle.getLocalTransform().isApprox(obstacleLocalTransform, 0.0));
  for (std::size_t node = 0; node < patch.getNodeCount(); ++node) {
    EXPECT_EQ(patch.getPosition(node), patchPositions[node]);
  }
}

//==============================================================================
// A static obstacle's pose is part of the positive-contact VBD bake. Once the
// public solver enters simulation, a non-idempotent teleport is rejected by
// the mutator itself and cannot invalidate or partially change the live solve.
TEST(VbdWorldSolver, PublicConfigureRejectsRuntimeObstacleTeleportAtomically)
{
  sx::World world;
  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.5)));
  setSurfaceObstaclePolicy(obstacle);
  auto patch
      = world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);
  world.enterSimulationMode();
  ASSERT_FALSE(world.getResolvedConfiguration().isEmpty());

  const Eigen::Isometry3d obstacleTransform = obstacle.getTransform();
  const Eigen::Isometry3d obstacleLocalTransform = obstacle.getLocalTransform();
  std::vector<Eigen::Vector3d> patchPositions;
  patchPositions.reserve(patch.getNodeCount());
  for (std::size_t node = 0; node < patch.getNodeCount(); ++node) {
    patchPositions.push_back(patch.getPosition(node));
  }
  const double beforeTime = world.getTime();
  const std::size_t beforeFrame = world.getFrame();

  EXPECT_NO_THROW(obstacle.setTransform(obstacleTransform));
  Eigen::Isometry3d teleportedTransform = obstacleTransform;
  teleportedTransform.translation().x() += 1.0;
  EXPECT_THROW(
      obstacle.setTransform(teleportedTransform),
      sx::InvalidOperationException);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_FALSE(world.getResolvedConfiguration().isEmpty());
  EXPECT_DOUBLE_EQ(world.getTime(), beforeTime);
  EXPECT_EQ(world.getFrame(), beforeFrame);
  EXPECT_TRUE(obstacle.getTransform().isApprox(obstacleTransform, 0.0));
  EXPECT_TRUE(
      obstacle.getLocalTransform().isApprox(obstacleLocalTransform, 0.0));
  for (std::size_t node = 0; node < patch.getNodeCount(); ++node) {
    EXPECT_EQ(patch.getPosition(node), patchPositions[node]);
  }
}

//==============================================================================
// Public solver selection is fail-closed for obstacle families the VBD path
// cannot yet represent. It must not silently substitute the default solver.
TEST(VbdWorldSolver, PublicConfigureRejectsMovingRigidSurfaceAtEntryAtomically)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.linearVelocity = Eigen::Vector3d::UnitX();
  auto obstacle = world.addRigidBody("moving-obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.5)));
  obstacle.setKinematic(true);
  setSurfaceObstaclePolicy(obstacle);

  auto patch
      = world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);

  const Eigen::Isometry3d obstacleTransform = obstacle.getTransform();
  const Eigen::Vector3d obstacleLinearVelocity = obstacle.getLinearVelocity();
  std::vector<Eigen::Vector3d> patchPositions;
  patchPositions.reserve(patch.getNodeCount());
  for (std::size_t i = 0; i < patch.getNodeCount(); ++i) {
    patchPositions.push_back(patch.getPosition(i));
  }
  const double time = world.getTime();
  const std::size_t frame = world.getFrame();

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_EQ(world.getTime(), time);
  EXPECT_EQ(world.getFrame(), frame);
  EXPECT_TRUE(obstacle.getTransform().matrix().isApprox(
      obstacleTransform.matrix(), 0.0));
  EXPECT_EQ(obstacle.getLinearVelocity(), obstacleLinearVelocity);
  for (std::size_t i = 0; i < patch.getNodeCount(); ++i) {
    EXPECT_EQ(patch.getPosition(i), patchPositions[i]);
  }
}

//==============================================================================
TEST(VbdWorldSolver, PublicConfigureRejectsCapsuleObstacleAtEntry)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  auto obstacle = world.addRigidBody("capsule-obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeCapsule(/*radius=*/0.3, /*halfHeight=*/0.4));
  setSurfaceObstaclePolicy(obstacle);

  world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);
  EXPECT_FALSE(world.isSimulationMode());
}

//==============================================================================
// Positive-contact public VBD is fail-closed over the complete public rigid
// obstacle surface. Every tagged obstacle must be one valid static sphere or
// box; unsupported families, missing geometry, and compound secondary shapes
// cannot be silently discarded by the primary-shape collectors.
TEST(VbdWorldSolver, PublicConfigureRejectsUnsupportedObstacleEnvelopeAtEntry)
{
  const auto expectRejected = [](std::string_view label,
                                 const sx::CollisionShape* shape,
                                 bool groundBarrier) {
    SCOPED_TRACE(label);
    sx::World world;
    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    auto obstacle = world.addRigidBody("unsupported-obstacle", obstacleOptions);
    if (shape != nullptr) {
      obstacle.setCollisionShape(*shape);
    }
    if (groundBarrier) {
      setGroundBarrierPolicy(obstacle);
    } else {
      setSurfaceObstaclePolicy(obstacle);
    }

    world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
    sx::DeformableSolverOptions options;
    options.groundContactStiffness = 1.0e4;
    world.configureDeformableSolver("patch", options);

    EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);
    EXPECT_FALSE(world.isSimulationMode());
  };

  const auto capsule
      = sx::CollisionShape::makeCapsule(/*radius=*/0.3, /*halfHeight=*/0.4);
  const auto cylinder
      = sx::CollisionShape::makeCylinder(/*radius=*/0.3, /*halfHeight=*/0.4);
  const auto plane
      = sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0);
  const auto mesh = sx::CollisionShape::makeMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {Eigen::Vector3i(0, 1, 2)});
  expectRejected("ground capsule", &capsule, true);
  expectRejected("surface cylinder", &cylinder, false);
  expectRejected("surface plane", &plane, false);
  expectRejected("surface mesh", &mesh, false);
  expectRejected("missing collision geometry", nullptr, false);

  sx::World compoundWorld;
  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  auto compound
      = compoundWorld.addRigidBody("compound-obstacle", obstacleOptions);
  compound.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.5)));
  compound.addCollisionShape(sx::CollisionShape::makeSphere(0.25));
  setSurfaceObstaclePolicy(compound);
  compoundWorld.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  compoundWorld.configureDeformableSolver("patch", options);
  EXPECT_THROW(
      compoundWorld.enterSimulationMode(), sx::InvalidArgumentException);
  EXPECT_FALSE(compoundWorld.isSimulationMode());
}

//==============================================================================
TEST(VbdWorldSolver, PublicConfigureRejectsFreeSphereObstacleAtEntry)
{
  sx::World world;
  auto obstacle = world.addRigidBody("free-sphere");
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  setSurfaceObstaclePolicy(obstacle);
  world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);
  EXPECT_FALSE(world.isSimulationMode());
}

//==============================================================================
// Runtime obstacle mutations invalidate the bake and must be rejected before
// any subsequent step stage mutates physical state.
TEST(VbdWorldSolver, PublicConfigureRejectsRuntimeObstacleMutationBeforeStep)
{
  sx::World world;
  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d::Constant(0.5)));
  setSurfaceObstaclePolicy(obstacle);
  auto patch
      = world.addDeformableBody("patch", makeFallingPatchOptions(0.6, 0.0));
  sx::DeformableSolverOptions options;
  options.groundContactStiffness = 1.0e4;
  world.configureDeformableSolver("patch", options);
  world.enterSimulationMode();
  ASSERT_FALSE(world.getResolvedConfiguration().isEmpty());

  obstacle.setKinematic(true);
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  const Eigen::Isometry3d obstacleTransform = obstacle.getTransform();
  const Eigen::Vector3d patchPosition = patch.getPosition(0);
  const double time = world.getTime();
  const std::size_t frame = world.getFrame();
  EXPECT_THROW(world.step(), sx::InvalidArgumentException);
  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_EQ(world.getTime(), time);
  EXPECT_EQ(world.getFrame(), frame);
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_TRUE(obstacle.getTransform().matrix().isApprox(
      obstacleTransform.matrix(), 0.0));
  EXPECT_EQ(patch.getPosition(0), patchPosition);
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

  for (int step = 0; step < 300; ++step) {
    world.step();
  }
  EXPECT_EQ(world.getLastDeformableSolverDiagnostics().vbdBodyCount, 1u);

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

//==============================================================================
// A strict cap accepts an active set exactly at the boundary on the first
// delayed crossing. Diagnostics expose the construction policy, baked cap,
// and largest combined PT/EE set requested by a successful build.
TEST(VbdWorldSolver, SurfaceCandidateCapacityAcceptsExactDelayedActiveSet)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "one_crossing",
      makeDelayedSelfCrossingPointOptions(
          /*movingPointCount=*/1u, /*candidateCapacity=*/1u));

  EXPECT_EQ(body.getSurfaceContactCandidateCapacity(), 1u);
  world.enterSimulationMode();
  EXPECT_NO_THROW(world.step());

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityRequested, 1u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityResolved, 1u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCountPeak, 1u);
  EXPECT_GT(body.getPosition(3u).z(), 0.0);
  EXPECT_LT(body.getPosition(3u).z(), 1.0);
}

//==============================================================================
// A body whose baked active set already exceeds an explicit construction
// contract is rejected before simulation mode can commit.
TEST(VbdWorldSolver, SurfaceCandidateCapacityRejectsBakedCapPlusOne)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  world.addDeformableBody(
      "two_active",
      makeDelayedSelfCrossingPointOptions(
          /*movingPointCount=*/2u,
          /*candidateCapacity=*/1u,
          /*pointHeight=*/0.0));
  const double timeBefore = world.getTime();
  const std::size_t frameBefore = world.getFrame();

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidOperationException);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), timeBefore);
  EXPECT_EQ(world.getFrame(), frameBefore);
}

//==============================================================================
// Runtime overflow is discovered only after both crossing points enter the
// swept active set. All deformable solves remain in scratch until the complete
// stage succeeds, so neither this body nor an independent earlier/later body
// can publish a partial step.
TEST(VbdWorldSolver, SurfaceCandidateOverflowRollsBackAllDeformableBodies)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::DeformableBodyOptions freeOptions;
  freeOptions.positions = {Eigen::Vector3d(-2.0, 0.0, 0.0)};
  freeOptions.velocities = {Eigen::Vector3d(1.0, 0.0, 0.0)};
  freeOptions.masses = {1.0};
  auto freeBody = world.addDeformableBody("free", freeOptions);
  auto overflowingBody = world.addDeformableBody(
      "two_crossings",
      makeDelayedSelfCrossingPointOptions(
          /*movingPointCount=*/2u, /*candidateCapacity=*/1u));
  world.enterSimulationMode();

  const Eigen::Vector3d freePositionBefore = freeBody.getPosition(0u);
  const Eigen::Vector3d freeVelocityBefore = freeBody.getVelocity(0u);
  std::vector<Eigen::Vector3d> overflowPositionsBefore;
  std::vector<Eigen::Vector3d> overflowVelocitiesBefore;
  for (std::size_t node = 0u; node < overflowingBody.getNodeCount(); ++node) {
    overflowPositionsBefore.push_back(overflowingBody.getPosition(node));
    overflowVelocitiesBefore.push_back(overflowingBody.getVelocity(node));
  }
  const double timeBefore = world.getTime();
  const std::size_t frameBefore = world.getFrame();

  EXPECT_THROW(world.step(), sx::InvalidOperationException);

  EXPECT_TRUE(freeBody.getPosition(0u).isApprox(freePositionBefore, 0.0));
  EXPECT_TRUE(freeBody.getVelocity(0u).isApprox(freeVelocityBefore, 0.0));
  for (std::size_t node = 0u; node < overflowingBody.getNodeCount(); ++node) {
    EXPECT_TRUE(overflowingBody.getPosition(node).isApprox(
        overflowPositionsBefore[node], 0.0));
    EXPECT_TRUE(overflowingBody.getVelocity(node).isApprox(
        overflowVelocitiesBefore[node], 0.0));
  }
  EXPECT_DOUBLE_EQ(world.getTime(), timeBefore);
  EXPECT_EQ(world.getFrame(), frameBefore);
}

//==============================================================================
// A malformed per-body contact policy is rejected by the all-deformable
// preflight. Even an otherwise valid body that precedes it in registry order
// remains bitwise unchanged because no deformable solve has started.
TEST(VbdWorldSolver, MissingContactConfigRollsBackAllDeformableBodies)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.velocities = {Eigen::Vector3d(1.0, 0.0, 0.0)};
  options.masses = {1.0};
  auto first = world.addDeformableBody("first", options);
  auto malformed = world.addDeformableBody("malformed", options);
  world.enterSimulationMode();

  const Eigen::Vector3d firstPositionBefore = first.getPosition(0u);
  const Eigen::Vector3d firstVelocityBefore = first.getVelocity(0u);
  const Eigen::Vector3d malformedPositionBefore = malformed.getPosition(0u);
  const Eigen::Vector3d malformedVelocityBefore = malformed.getVelocity(0u);
  const double timeBefore = world.getTime();
  const std::size_t frameBefore = world.getFrame();

  auto& registry = dart::simulation::detail::registryOf(world);
  entt::entity malformedEntity = entt::null;
  auto deformableNames
      = registry.view<sx::comps::DeformableBodyTag, sx::comps::Name>();
  for (const entt::entity entity : deformableNames) {
    if (deformableNames.get<sx::comps::Name>(entity).name == "malformed") {
      malformedEntity = entity;
      break;
    }
  }
  ASSERT_NE(malformedEntity, entt::null);
  registry.remove<sx::comps::DeformableContactConfig>(malformedEntity);

  EXPECT_THROW(world.step(), sx::InvalidOperationException);
  EXPECT_TRUE(first.getPosition(0u).isApprox(firstPositionBefore, 0.0));
  EXPECT_TRUE(first.getVelocity(0u).isApprox(firstVelocityBefore, 0.0));
  EXPECT_TRUE(malformed.getPosition(0u).isApprox(malformedPositionBefore, 0.0));
  EXPECT_TRUE(malformed.getVelocity(0u).isApprox(malformedVelocityBefore, 0.0));
  EXPECT_DOUBLE_EQ(world.getTime(), timeBefore);
  EXPECT_EQ(world.getFrame(), frameBefore);
}

//==============================================================================
// Bake reserves the explicit cap into PN candidate/block/triplet storage. A
// later active-set growth from zero to the full cap reuses that retained World
// allocation instead of growing on the contact step.
TEST(VbdWorldSolver, SurfaceCandidateCapacityReusesWarmedWorldAllocation)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto options = makeDelayedSelfCrossingPointOptions(
      /*movingPointCount=*/2u, /*candidateCapacity=*/2u);
  options.positions[3u].z() = 0.3;
  options.positions[4u].z() = 0.5;
  options.velocities[3u].z() = -1.0;
  options.velocities[4u].z() = -2.0;
  world.addDeformableBody("delayed_two", options);
  world.enterSimulationMode();

  world.step();
  world.step();
  auto& freeList = world.getMemoryManager().getFreeListAllocator();
  const std::size_t allocatedBeforeContact = freeList.getAllocatedSize();
  const std::size_t peakBeforeContact = freeList.getPeakAllocatedSize();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityResolved, 2u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCountPeak, 2u);
  EXPECT_EQ(freeList.getAllocatedSize(), allocatedBeforeContact);
  EXPECT_EQ(freeList.getPeakAllocatedSize(), peakBeforeContact);
}

//==============================================================================
// Five disconnected triangles have 15 points, 5 faces, and 15 unique edges.
// Their exact valid self-pair bound is 60 PT + 90 EE = 150, while the topology
// heuristic is 4 * (15 + 5 + 15) = 140. With no baked active pairs, automatic
// capacity deterministically resolves to min(150, max(0, 140)) = 140.
TEST(VbdWorldSolver, AutomaticSurfaceCandidateCapacityUsesTopologyFormula)
{
  sx::World world;
  sx::DeformableBodyOptions options;
  for (std::size_t triangle = 0u; triangle < 5u; ++triangle) {
    const double x = 10.0 * static_cast<double>(triangle);
    const std::uint32_t node = static_cast<std::uint32_t>(3u * triangle);
    options.positions.emplace_back(x, 0.0, 0.0);
    options.positions.emplace_back(x + 1.0, 0.0, 0.0);
    options.positions.emplace_back(x, 1.0, 0.0);
    options.surfaceTriangles.push_back({node, node + 1u, node + 2u});
  }
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 1.0);
  options.fixedNodes.resize(options.positions.size());
  for (std::size_t node = 0u; node < options.positions.size(); ++node) {
    options.fixedNodes[node] = static_cast<std::uint32_t>(node);
  }

  auto body = world.addDeformableBody("disconnected_triangles", options);
  EXPECT_EQ(body.getSurfaceContactCandidateCapacity(), 0u);
  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityRequested, 0u);
  // Exact valid-pair bound: 15 points x 5 triangles - 3 incident pairs per
  // triangle = 60 point-triangle pairs, plus C(15, 2) - 15 vertex-adjacent
  // pairs = 90 edge-edge pairs.
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityResolved, 150u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCountPeak, 0u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateOverflowCount, 0u);
}

//==============================================================================
// A frozen topology whose exact valid-pair bound exceeds the automatic reserve
// budget resolves to the budget and is allowed to grow instead of failing a
// valid scene mid-step. A quiet step never exceeds the reserved storage.
TEST(VbdWorldSolver, AutomaticSurfaceCandidateCapacityCapsAtReserveBudget)
{
  sx::World world;
  sx::DeformableBodyOptions options;
  constexpr std::size_t kSide = 20u;
  for (std::size_t row = 0u; row < kSide; ++row) {
    for (std::size_t column = 0u; column < kSide; ++column) {
      options.positions.emplace_back(
          static_cast<double>(column), static_cast<double>(row), 0.0);
    }
  }
  const auto node = [](std::size_t row, std::size_t column) {
    return static_cast<std::uint32_t>(row * kSide + column);
  };
  for (std::size_t row = 0u; row + 1u < kSide; ++row) {
    for (std::size_t column = 0u; column + 1u < kSide; ++column) {
      options.surfaceTriangles.push_back(
          {node(row, column), node(row, column + 1u), node(row + 1u, column)});
      options.surfaceTriangles.push_back(
          {node(row, column + 1u),
           node(row + 1u, column + 1u),
           node(row + 1u, column)});
    }
  }
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 1.0);
  options.fixedNodes.resize(options.positions.size());
  for (std::size_t index = 0u; index < options.positions.size(); ++index) {
    options.fixedNodes[index] = static_cast<std::uint32_t>(index);
  }

  // 400 points x 722 triangles - 3 x 722 incident pairs alone exceeds the
  // 65,536-candidate reserve budget before any edge-edge pair is counted.
  auto body = world.addDeformableBody("large_cloth", options);
  EXPECT_EQ(body.getSurfaceContactCandidateCapacity(), 0u);
  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityRequested, 0u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityResolved, 65536u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateOverflowCount, 0u);
  EXPECT_LE(diagnostics.surfaceContactCandidateCountPeak, 65536u);
}

//==============================================================================
// Zero remains the serialized construction policy while bake resolves a
// deterministic nonzero floor for topology that has no valid surface pair.
TEST(VbdWorldSolver, AutomaticSurfaceCandidateCapacityHasDocumentedFloor)
{
  sx::World world;
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.velocities = {Eigen::Vector3d::Zero()};
  options.masses = {1.0};
  auto body = world.addDeformableBody("point", options);
  EXPECT_EQ(body.getSurfaceContactCandidateCapacity(), 0u);

  world.step();
  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityRequested, 0u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCapacityResolved, 1u);
  EXPECT_EQ(diagnostics.surfaceContactCandidateCountPeak, 0u);
}

//==============================================================================
// The construction policy, not only the resolved scratch value, is replay
// layout. Mutating it during an immutable recording session must fail before
// stepping or extending history.
TEST(VbdWorldSolver, ReplayFreezesSurfaceCandidateCapacityPolicy)
{
  sx::World world;
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.velocities = {Eigen::Vector3d::Zero()};
  options.masses = {1.0};
  options.surfaceContactCandidateCapacity = 4u;
  world.addDeformableBody("point", options);
  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity entity
      = *registry.view<sx::comps::DeformableBodyTag>().begin();
  registry.get<sx::comps::DeformableContactConfig>(entity)
      .surfaceCandidateCapacity = 5u;
  const auto cursorBefore = world.getReplayCursor();

  EXPECT_THROW(world.step(), sx::InvalidOperationException);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
  EXPECT_EQ(world.getReplayFrameCount(), 1u);
  EXPECT_EQ(world.getReplayCursor(), cursorBefore);
}
