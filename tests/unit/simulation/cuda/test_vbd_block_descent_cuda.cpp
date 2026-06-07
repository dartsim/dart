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

#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/detail/deformable_vbd/stepper.hpp>

#include <dart/simulation/compute/cuda/vbd_block_descent_cuda.cuh>
#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace cuda = dart::simulation::compute::cuda;

namespace {

using Vec3 = Eigen::Vector3d;

struct Scene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::SpringElement> springs;
  double stiffness = 500.0;
  double timeStep = 0.01;
};

// A pinned square spring grid (structural + shear springs), top row fixed.
Scene makeGrid(int side)
{
  Scene scene;
  const auto index = [side](int r, int c) {
    return static_cast<std::uint32_t>(r * side + c);
  };
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      scene.positions.emplace_back(c, -r, 0.0);
      scene.masses.push_back(1.0);
      scene.fixed.push_back(r == 0 ? 1u : 0u);
    }
  }
  scene.inertialTargets = scene.positions;
  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    if (scene.fixed[i] == 0u) {
      scene.inertialTargets[i] += Vec3(0.0, -0.05, 0.01);
    }
  }
  const double rest = 0.9;
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      if (c + 1 < side) {
        scene.springs.push_back({index(r, c), index(r, c + 1), rest});
      }
      if (r + 1 < side) {
        scene.springs.push_back({index(r, c), index(r + 1, c), rest});
      }
      if (r + 1 < side && c + 1 < side) {
        scene.springs.push_back({index(r, c), index(r + 1, c + 1), rest * 1.4});
      }
    }
  }
  return scene;
}

cuda::VbdCudaMassSpringProblem makeProblem(
    const Scene& scene,
    const std::vector<Vec3>& positions,
    const vbd::VertexColoring& coloring,
    const vbd::SpringAdjacency& adjacency,
    std::size_t iterations)
{
  cuda::VbdCudaMassSpringProblem problem;
  problem.nodeCount = positions.size();
  problem.stiffness = scene.stiffness;
  problem.timeStep = scene.timeStep;
  problem.iterations = iterations;

  for (std::size_t i = 0; i < positions.size(); ++i) {
    problem.positions.push_back(positions[i].x());
    problem.positions.push_back(positions[i].y());
    problem.positions.push_back(positions[i].z());
    problem.inertialTargets.push_back(scene.inertialTargets[i].x());
    problem.inertialTargets.push_back(scene.inertialTargets[i].y());
    problem.inertialTargets.push_back(scene.inertialTargets[i].z());
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;

  for (const auto& spring : scene.springs) {
    problem.springA.push_back(spring.a);
    problem.springB.push_back(spring.b);
    problem.springRest.push_back(spring.restLength);
  }

  problem.incidentOffsets.push_back(0);
  for (std::size_t v = 0; v < positions.size(); ++v) {
    for (const std::uint32_t s : adjacency.incidentSprings[v]) {
      problem.incidentSprings.push_back(s);
    }
    problem.incidentOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentSprings.size()));
  }

  problem.colorOffsets.push_back(0);
  for (const auto& group : coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  return problem;
}

} // namespace

//==============================================================================
TEST(CudaVbdBlockDescent, MatchesCpuBlockDescent)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const Scene scene = makeGrid(6);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  constexpr std::size_t iterations = 300;

  // CPU reference (warm-started at the inertial target, as the GPU path is).
  std::vector<Vec3> cpu = scene.inertialTargets;
  for (std::size_t i = 0; i < cpu.size(); ++i) {
    if (scene.fixed[i] != 0u) {
      cpu[i] = scene.positions[i];
    }
  }
  vbd::BlockDescentOptions options;
  options.iterations = iterations;
  vbd::blockDescentMassSpring(
      cpu,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      coloring,
      adjacency,
      options);

  // GPU: warm-start at the inertial target too.
  std::vector<Vec3> warmStart = scene.inertialTargets;
  for (std::size_t i = 0; i < warmStart.size(); ++i) {
    if (scene.fixed[i] != 0u) {
      warmStart[i] = scene.positions[i];
    }
  }
  cuda::VbdCudaMassSpringProblem problem
      = makeProblem(scene, warmStart, coloring, adjacency, iterations);
  cuda::vbdStepMassSpringCuda(problem);

  ASSERT_EQ(problem.positions.size(), cpu.size() * 3);
  for (std::size_t i = 0; i < cpu.size(); ++i) {
    const Vec3 gpu(
        problem.positions[3 * i],
        problem.positions[3 * i + 1],
        problem.positions[3 * i + 2]);
    EXPECT_NEAR((gpu - cpu[i]).norm(), 0.0, 1e-6) << "vertex " << i;
  }
}

//==============================================================================
TEST(CudaVbdBlockDescent, FixedVerticesDoNotMoveOnGpu)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const Scene scene = makeGrid(5);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);

  cuda::VbdCudaMassSpringProblem problem
      = makeProblem(scene, scene.positions, coloring, adjacency, 20);
  cuda::vbdStepMassSpringCuda(problem);

  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    if (scene.fixed[i] != 0u) {
      const Vec3 gpu(
          problem.positions[3 * i],
          problem.positions[3 * i + 1],
          problem.positions[3 * i + 2]);
      EXPECT_NEAR((gpu - scene.positions[i]).norm(), 0.0, 1e-12)
          << "fixed vertex " << i;
    }
  }
}

//==============================================================================
// The device-resident GPU rollout (full per-step pipeline on the GPU for many
// steps) matches the CPU stepper run for the same number of steps with the same
// non-accelerated warm start.
TEST(CudaVbdBlockDescent, RolloutMatchesCpuStepper)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const Scene scene = makeGrid(6);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  const Vec3 gravity(0.0, -9.81, 0.0);
  constexpr std::size_t steps = 20;
  constexpr std::size_t iterations = 40;

  // CPU stepper reference (no adaptive init / Chebyshev, matching the GPU
  // rollout's plain warm start).
  std::vector<Vec3> cpuPos = scene.positions;
  std::vector<Vec3> cpuVel(cpuPos.size(), Vec3::Zero());
  std::vector<Vec3> cpuPrevVel(cpuPos.size(), Vec3::Zero());
  vbd::VbdStepOptions stepOptions;
  stepOptions.iterations = iterations;
  stepOptions.useAdaptiveInit = false;
  stepOptions.useChebyshev = false;
  for (std::size_t s = 0; s < steps; ++s) {
    vbd::vbdStepMassSpring(
        cpuPos,
        cpuVel,
        cpuPrevVel,
        scene.masses,
        scene.fixed,
        scene.springs,
        scene.stiffness,
        gravity,
        scene.timeStep,
        coloring,
        adjacency,
        stepOptions,
        s > 0);
  }

  // GPU device-resident rollout.
  cuda::VbdCudaRolloutProblem problem;
  problem.nodeCount = scene.positions.size();
  problem.stiffness = scene.stiffness;
  problem.timeStep = scene.timeStep;
  problem.iterations = iterations;
  problem.stepCount = steps;
  problem.gravity[0] = gravity.x();
  problem.gravity[1] = gravity.y();
  problem.gravity[2] = gravity.z();
  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    problem.positions.push_back(scene.positions[i].x());
    problem.positions.push_back(scene.positions[i].y());
    problem.positions.push_back(scene.positions[i].z());
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& spring : scene.springs) {
    problem.springA.push_back(spring.a);
    problem.springB.push_back(spring.b);
    problem.springRest.push_back(spring.restLength);
  }
  problem.incidentOffsets.push_back(0);
  for (std::size_t v = 0; v < scene.positions.size(); ++v) {
    for (const std::uint32_t s : adjacency.incidentSprings[v]) {
      problem.incidentSprings.push_back(s);
    }
    problem.incidentOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentSprings.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  cuda::vbdRolloutMassSpringCuda(problem);

  for (std::size_t i = 0; i < cpuPos.size(); ++i) {
    const Vec3 gpu(
        problem.positions[3 * i],
        problem.positions[3 * i + 1],
        problem.positions[3 * i + 2]);
    EXPECT_NEAR((gpu - cpuPos[i]).norm(), 0.0, 1e-5) << "vertex " << i;
  }
}

namespace {

struct TetScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::TetMeshElement> tets;
  double mu = 3000.0;
  double lambda = 6000.0;
  double timeStep = 0.01;
};

// A tetrahedral bar of `cubes` stacked unit cubes (six Kuhn tets each), bottom
// ring pinned, inertial targets perturbed.
TetScene makeTetBar(int cubes)
{
  TetScene scene;
  const auto index = [](int i, int j, int k) {
    return static_cast<std::uint32_t>(((k * 2) + j) * 2 + i);
  };
  for (int k = 0; k <= cubes; ++k) {
    for (int j = 0; j < 2; ++j) {
      for (int i = 0; i < 2; ++i) {
        scene.positions.emplace_back(i, j, k);
        scene.masses.push_back(1.0);
        scene.fixed.push_back(k == 0 ? 1u : 0u);
      }
    }
  }
  scene.inertialTargets = scene.positions;
  for (std::size_t v = 0; v < scene.positions.size(); ++v) {
    if (scene.fixed[v] == 0u) {
      scene.inertialTargets[v] += Vec3(0.02, -0.03, 0.01);
    }
  }
  static const int kuhn[6][4]
      = {{0, 1, 3, 7},
         {0, 3, 2, 7},
         {0, 2, 6, 7},
         {0, 6, 4, 7},
         {0, 4, 5, 7},
         {0, 5, 1, 7}};
  for (int c = 0; c < cubes; ++c) {
    std::array<std::uint32_t, 8> corner{};
    for (int n = 0; n < 8; ++n) {
      corner[static_cast<std::size_t>(n)]
          = index(n & 1, (n >> 1) & 1, c + ((n >> 2) & 1));
    }
    for (const auto& t : kuhn) {
      const std::array<std::uint32_t, 4> v
          = {corner[t[0]], corner[t[1]], corner[t[2]], corner[t[3]]};
      const vbd::TetRestShape rest = vbd::makeTetRestShape(
          {scene.positions[v[0]],
           scene.positions[v[1]],
           scene.positions[v[2]],
           scene.positions[v[3]]});
      scene.tets.push_back({v, rest});
    }
  }
  return scene;
}

} // namespace

//==============================================================================
// The GPU tetrahedral Neo-Hookean solve matches the CPU tet block descent.
TEST(CudaVbdBlockDescent, TetMeshMatchesCpu)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const TetScene scene = makeTetBar(4);
  const std::size_t n = scene.positions.size();
  const auto coloring = vbd::colorTetMesh(n, scene.tets);
  const auto adjacency = vbd::TetAdjacency::build(n, scene.tets);
  constexpr std::size_t iterations = 100;

  // CPU reference, warm-started at the inertial target.
  std::vector<Vec3> cpu = scene.inertialTargets;
  for (std::size_t i = 0; i < n; ++i) {
    if (scene.fixed[i] != 0u) {
      cpu[i] = scene.positions[i];
    }
  }
  vbd::BlockDescentOptions options;
  options.iterations = iterations;
  vbd::blockDescentTetMesh(
      cpu,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.tets,
      scene.mu,
      scene.lambda,
      scene.timeStep,
      coloring,
      adjacency,
      options);

  // GPU problem from the same data and warm start.
  cuda::VbdCudaTetProblem problem;
  problem.nodeCount = n;
  problem.mu = scene.mu;
  problem.lambda = scene.lambda;
  problem.timeStep = scene.timeStep;
  problem.iterations = iterations;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec3 warm
        = scene.fixed[i] != 0u ? scene.positions[i] : scene.inertialTargets[i];
    problem.positions.push_back(warm.x());
    problem.positions.push_back(warm.y());
    problem.positions.push_back(warm.z());
    problem.inertialTargets.push_back(scene.inertialTargets[i].x());
    problem.inertialTargets.push_back(scene.inertialTargets[i].y());
    problem.inertialTargets.push_back(scene.inertialTargets[i].z());
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& tet : scene.tets) {
    for (int k = 0; k < 4; ++k) {
      problem.tetVertices.push_back(tet.vertices[static_cast<std::size_t>(k)]);
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        problem.tetRestShapeInverse.push_back(tet.rest.restShapeInverse(r, c));
      }
    }
    problem.tetRestVolume.push_back(tet.rest.restVolume);
  }
  problem.incidentTetOffsets.push_back(0);
  for (std::size_t v = 0; v < n; ++v) {
    for (const auto& [tetIndex, local] : adjacency.incidentTets[v]) {
      problem.incidentTetIndex.push_back(tetIndex);
      problem.incidentLocalVertex.push_back(local);
    }
    problem.incidentTetOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentTetIndex.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  cuda::vbdStepTetMeshCuda(problem);

  for (std::size_t i = 0; i < n; ++i) {
    const Vec3 gpu(
        problem.positions[3 * i],
        problem.positions[3 * i + 1],
        problem.positions[3 * i + 2]);
    EXPECT_NEAR((gpu - cpu[i]).norm(), 0.0, 1e-6) << "vertex " << i;
  }
}

namespace {

// Flatten a TetScene into the GPU rollout problem (topology + CSR adjacency +
// coloring), warm-started from the scene's rest positions with zero velocity.
cuda::VbdCudaTetRolloutProblem makeTetRolloutProblem(
    const TetScene& scene,
    const vbd::VertexColoring& coloring,
    const vbd::TetAdjacency& adjacency,
    const Vec3& gravity,
    std::size_t iterations,
    std::size_t steps,
    bool useCudaGraph)
{
  const std::size_t n = scene.positions.size();
  cuda::VbdCudaTetRolloutProblem problem;
  problem.nodeCount = n;
  problem.mu = scene.mu;
  problem.lambda = scene.lambda;
  problem.timeStep = scene.timeStep;
  problem.iterations = iterations;
  problem.stepCount = steps;
  problem.useCudaGraph = useCudaGraph;
  problem.gravity[0] = gravity.x();
  problem.gravity[1] = gravity.y();
  problem.gravity[2] = gravity.z();
  for (std::size_t i = 0; i < n; ++i) {
    problem.positions.push_back(scene.positions[i].x());
    problem.positions.push_back(scene.positions[i].y());
    problem.positions.push_back(scene.positions[i].z());
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& tet : scene.tets) {
    for (int k = 0; k < 4; ++k) {
      problem.tetVertices.push_back(tet.vertices[static_cast<std::size_t>(k)]);
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        problem.tetRestShapeInverse.push_back(tet.rest.restShapeInverse(r, c));
      }
    }
    problem.tetRestVolume.push_back(tet.rest.restVolume);
  }
  problem.incidentTetOffsets.push_back(0);
  for (std::size_t v = 0; v < n; ++v) {
    for (const auto& [tetIndex, local] : adjacency.incidentTets[v]) {
      problem.incidentTetIndex.push_back(tetIndex);
      problem.incidentLocalVertex.push_back(local);
    }
    problem.incidentTetOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentTetIndex.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  return problem;
}

// CPU reference for the tet rollout: replicate the GPU per-step pipeline
// (inertial-target warm start at y, tet block descent, backward-Euler velocity
// update) exactly, so the two trajectories must agree.
std::vector<Vec3> cpuTetRollout(
    const TetScene& scene,
    const vbd::VertexColoring& coloring,
    const vbd::TetAdjacency& adjacency,
    const Vec3& gravity,
    std::size_t iterations,
    std::size_t steps)
{
  const std::size_t n = scene.positions.size();
  const double h = scene.timeStep;
  std::vector<Vec3> pos = scene.positions;
  std::vector<Vec3> vel(n, Vec3::Zero());
  vbd::BlockDescentOptions options;
  options.iterations = iterations;
  for (std::size_t s = 0; s < steps; ++s) {
    std::vector<Vec3> stepStart = pos;
    std::vector<Vec3> inertial = pos;
    std::vector<Vec3> next = pos;
    for (std::size_t i = 0; i < n; ++i) {
      if (scene.fixed[i] == 0u) {
        inertial[i] = pos[i] + h * vel[i] + h * h * gravity;
        next[i] = inertial[i];
      }
    }
    vbd::blockDescentTetMesh(
        next,
        scene.masses,
        scene.fixed,
        inertial,
        scene.tets,
        scene.mu,
        scene.lambda,
        h,
        coloring,
        adjacency,
        options);
    for (std::size_t i = 0; i < n; ++i) {
      if (scene.fixed[i] == 0u) {
        vel[i] = (next[i] - stepStart[i]) / h;
      }
    }
    pos = next;
  }
  return pos;
}

} // namespace

//==============================================================================
// The device-resident tetrahedral rollout matches the equivalent CPU per-step
// pipeline over many steps.
TEST(CudaVbdBlockDescent, TetRolloutMatchesCpu)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const TetScene scene = makeTetBar(4);
  const std::size_t n = scene.positions.size();
  const auto coloring = vbd::colorTetMesh(n, scene.tets);
  const auto adjacency = vbd::TetAdjacency::build(n, scene.tets);
  const Vec3 gravity(0.0, -9.81, 0.0);
  constexpr std::size_t steps = 20;
  constexpr std::size_t iterations = 40;

  const std::vector<Vec3> cpu
      = cpuTetRollout(scene, coloring, adjacency, gravity, iterations, steps);

  cuda::VbdCudaTetRolloutProblem problem = makeTetRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, false);
  cuda::vbdRolloutTetMeshCuda(problem);

  for (std::size_t i = 0; i < n; ++i) {
    const Vec3 gpu(
        problem.positions[3 * i],
        problem.positions[3 * i + 1],
        problem.positions[3 * i + 2]);
    EXPECT_NEAR((gpu - cpu[i]).norm(), 0.0, 1e-5) << "vertex " << i;
  }
}

//==============================================================================
// CUDA-graph capture replays one captured step for every step, so it must give
// the same trajectory as launching each step directly.
TEST(CudaVbdBlockDescent, GraphRolloutMatchesPlainRollout)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const TetScene scene = makeTetBar(4);
  const std::size_t n = scene.positions.size();
  const auto coloring = vbd::colorTetMesh(n, scene.tets);
  const auto adjacency = vbd::TetAdjacency::build(n, scene.tets);
  const Vec3 gravity(0.0, -9.81, 0.0);
  constexpr std::size_t steps = 20;
  constexpr std::size_t iterations = 40;

  cuda::VbdCudaTetRolloutProblem plain = makeTetRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, false);
  cuda::vbdRolloutTetMeshCuda(plain);

  cuda::VbdCudaTetRolloutProblem graph = makeTetRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, true);
  cuda::vbdRolloutTetMeshCuda(graph);

  ASSERT_EQ(plain.positions.size(), graph.positions.size());
  for (std::size_t i = 0; i < plain.positions.size(); ++i) {
    EXPECT_NEAR(plain.positions[i], graph.positions[i], 1e-12)
        << "component " << i;
  }
}

namespace {

cuda::VbdCudaRolloutProblem makeMassSpringRolloutProblem(
    const Scene& scene,
    const vbd::VertexColoring& coloring,
    const vbd::SpringAdjacency& adjacency,
    const Vec3& gravity,
    std::size_t iterations,
    std::size_t steps,
    bool useSinglePrecision)
{
  cuda::VbdCudaRolloutProblem problem;
  problem.nodeCount = scene.positions.size();
  problem.stiffness = scene.stiffness;
  problem.timeStep = scene.timeStep;
  problem.iterations = iterations;
  problem.stepCount = steps;
  problem.useSinglePrecision = useSinglePrecision;
  problem.gravity[0] = gravity.x();
  problem.gravity[1] = gravity.y();
  problem.gravity[2] = gravity.z();
  for (const auto& p : scene.positions) {
    problem.positions.push_back(p.x());
    problem.positions.push_back(p.y());
    problem.positions.push_back(p.z());
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& spring : scene.springs) {
    problem.springA.push_back(spring.a);
    problem.springB.push_back(spring.b);
    problem.springRest.push_back(spring.restLength);
  }
  problem.incidentOffsets.push_back(0);
  for (std::size_t v = 0; v < scene.positions.size(); ++v) {
    for (const std::uint32_t s : adjacency.incidentSprings[v]) {
      problem.incidentSprings.push_back(s);
    }
    problem.incidentOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentSprings.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  return problem;
}

} // namespace

//==============================================================================
// The single-precision (mixed-precision) rollout tracks the double rollout to
// float accuracy: the same fixed point, just computed in float.
TEST(CudaVbdBlockDescent, MixedPrecisionRolloutMatchesDouble)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const Scene scene = makeGrid(8);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  const Vec3 gravity(0.0, -9.81, 0.0);
  constexpr std::size_t steps = 20;
  constexpr std::size_t iterations = 40;

  cuda::VbdCudaRolloutProblem doubleProblem = makeMassSpringRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, false);
  cuda::vbdRolloutMassSpringCuda(doubleProblem);

  cuda::VbdCudaRolloutProblem floatProblem = makeMassSpringRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, true);
  cuda::vbdRolloutMassSpringCuda(floatProblem);

  ASSERT_EQ(doubleProblem.positions.size(), floatProblem.positions.size());
  for (std::size_t i = 0; i < doubleProblem.positions.size(); ++i) {
    EXPECT_NEAR(doubleProblem.positions[i], floatProblem.positions[i], 5e-3)
        << "component " << i;
  }
}

//==============================================================================
// The single-precision tetrahedral rollout tracks the double tet rollout to
// float accuracy.
TEST(CudaVbdBlockDescent, MixedPrecisionTetRolloutMatchesDouble)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "no CUDA device available";
  }

  const TetScene scene = makeTetBar(4);
  const std::size_t n = scene.positions.size();
  const auto coloring = vbd::colorTetMesh(n, scene.tets);
  const auto adjacency = vbd::TetAdjacency::build(n, scene.tets);
  const Vec3 gravity(0.0, -9.81, 0.0);
  constexpr std::size_t steps = 20;
  constexpr std::size_t iterations = 40;

  cuda::VbdCudaTetRolloutProblem doubleProblem = makeTetRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, false);
  cuda::vbdRolloutTetMeshCuda(doubleProblem);

  cuda::VbdCudaTetRolloutProblem floatProblem = makeTetRolloutProblem(
      scene, coloring, adjacency, gravity, iterations, steps, false);
  floatProblem.useSinglePrecision = true;
  cuda::vbdRolloutTetMeshCuda(floatProblem);

  ASSERT_EQ(doubleProblem.positions.size(), floatProblem.positions.size());
  for (std::size_t i = 0; i < doubleProblem.positions.size(); ++i) {
    EXPECT_NEAR(doubleProblem.positions[i], floatProblem.positions[i], 1e-2)
        << "component " << i;
  }
}
