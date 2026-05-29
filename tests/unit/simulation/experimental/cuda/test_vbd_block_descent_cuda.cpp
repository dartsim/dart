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

#include <dart/simulation/experimental/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/stepper.hpp>

#include <dart/simulation/experimental/compute/cuda/vbd_block_descent_cuda.cuh>
#include <gtest/gtest.h>

#include <vector>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;
namespace cuda = dart::simulation::experimental::compute::cuda;

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
