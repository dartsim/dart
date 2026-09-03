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

#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/detail/deformable_vbd/parallel_block_descent.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace dc = dart::simulation::detail::deformable_contact;
namespace sim = dart::simulation;
namespace compute = dart::simulation::compute;

namespace {

using Vec3 = Eigen::Vector3d;

struct GridScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::SpringElement> springs;
  vbd::VertexColoring coloring;
  vbd::SpringAdjacency adjacency;
  double stiffness = 500.0;
  double timeStep = 0.02;
};

GridScene makeGrid(int side)
{
  GridScene scene;
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
  scene.coloring = vbd::colorSprings(scene.positions.size(), scene.springs);
  scene.adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  return scene;
}

std::vector<Vec3> runSerial(const GridScene& scene, std::size_t iterations)
{
  std::vector<Vec3> positions = scene.inertialTargets;
  vbd::BlockDescentOptions options;
  options.iterations = iterations;
  vbd::blockDescentMassSpring(
      positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      scene.coloring,
      scene.adjacency,
      options);
  return positions;
}

std::vector<Vec3> runParallel(
    const GridScene& scene, std::size_t iterations, unsigned int threads)
{
  std::vector<Vec3> positions = scene.inertialTargets;
  compute::ParallelExecutor executor(threads);
  vbd::BlockDescentOptions options;
  options.iterations = iterations;
  vbd::parallelBlockDescentMassSpring(
      positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      scene.coloring,
      scene.adjacency,
      options,
      executor);
  return positions;
}

double maxFreeDisplacement(
    const std::vector<Vec3>& from,
    const std::vector<Vec3>& to,
    const std::vector<std::uint8_t>& fixed)
{
  double maximum = 0.0;
  for (std::size_t vertex = 0; vertex < from.size(); ++vertex) {
    if (fixed[vertex] == 0u) {
      maximum = std::max(maximum, (to[vertex] - from[vertex]).norm());
    }
  }
  return maximum;
}

} // namespace

//==============================================================================
// The parallel sweep is deterministic and identical to the serial sweep, since
// same-color vertices are independent and colors stay ordered by the barrier.
TEST(VbdParallelBlockDescent, MatchesSerialExactly)
{
  const GridScene scene = makeGrid(10);
  const std::vector<Vec3> serial = runSerial(scene, 40);
  for (const unsigned int threads : {2u, 4u, 8u}) {
    const std::vector<Vec3> parallel = runParallel(scene, 40, threads);
    ASSERT_EQ(serial.size(), parallel.size());
    for (std::size_t i = 0; i < serial.size(); ++i) {
      EXPECT_NEAR((serial[i] - parallel[i]).norm(), 0.0, 1e-12)
          << "threads=" << threads << " vertex=" << i;
    }
  }
}

//==============================================================================
TEST(VbdParallelBlockDescent, SingleThreadFallsBackToSerial)
{
  const GridScene scene = makeGrid(6);
  const std::vector<Vec3> serial = runSerial(scene, 20);
  const std::vector<Vec3> oneThread = runParallel(scene, 20, 1);
  for (std::size_t i = 0; i < serial.size(); ++i) {
    EXPECT_NEAR((serial[i] - oneThread[i]).norm(), 0.0, 1e-12)
        << "vertex " << i;
  }
}

//==============================================================================
TEST(VbdParallelBlockDescent, ConvergesToLowResidual)
{
  const GridScene scene = makeGrid(8);
  std::vector<Vec3> positions = scene.inertialTargets;
  vbd::BlockDescentOptions options;
  options.iterations = 200;
  compute::ParallelExecutor executor(4);
  const vbd::BlockDescentStats stats = vbd::parallelBlockDescentMassSpring(
      positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      scene.coloring,
      scene.adjacency,
      options,
      executor);
  EXPECT_LT(stats.finalResidualNormSquared, 1e-12);
}

//==============================================================================
// convergenceDisplacement is a public solve control, so the parallel
// mass-spring driver must not silently replace it with a full-budget solve.
TEST(VbdParallelBlockDescent, MassSpringHonorsConvergenceControl)
{
  const GridScene scene = makeGrid(6);
  const std::size_t freeVertexCount = static_cast<std::size_t>(
      std::count(scene.fixed.begin(), scene.fixed.end(), std::uint8_t{0u}));
  vbd::BlockDescentOptions options;
  options.iterations = 9;
  options.convergenceDisplacement = 1e100;

  for (const unsigned int threads : {1u, 2u, 4u}) {
    std::vector<Vec3> positions = scene.inertialTargets;
    compute::ParallelExecutor executor(threads);
    const vbd::BlockDescentStats stats = vbd::parallelBlockDescentMassSpring(
        positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.stiffness,
        scene.timeStep,
        scene.coloring,
        scene.adjacency,
        options,
        executor);
    EXPECT_EQ(stats.iterations, 1u) << "threads=" << threads;
    EXPECT_EQ(stats.vertexUpdates, freeVertexCount) << "threads=" << threads;
  }
}

//==============================================================================
// The combined springs + tetrahedra parallel sweep is likewise deterministic
// and bit-identical to the serial blockDescentDeformable, since same-color
// vertices share neither a spring nor a tet.
TEST(VbdParallelBlockDescent, DeformableMatchesSerialExactly)
{
  const std::vector<Vec3> rest
      = {Vec3(0, 0, 0),
         Vec3(1, 0, 0),
         Vec3(0, 1, 0),
         Vec3(0, 0, 1),
         Vec3(1, 1, 1),
         Vec3(2, 0, 0),
         Vec3(2, 1, 0)};
  std::vector<double> masses(rest.size(), 1.0);
  std::vector<std::uint8_t> fixed(rest.size(), 0u);
  fixed[0] = 1u;

  const auto restLength = [&](std::uint32_t a, std::uint32_t b) {
    return (rest[a] - rest[b]).norm();
  };
  const std::vector<vbd::SpringElement> springs
      = {{1, 5, restLength(1, 5)},
         {5, 6, restLength(5, 6)},
         {4, 6, restLength(4, 6)}};

  const auto makeRest = [&](std::array<std::uint32_t, 4> v) {
    return vbd::makeTetRestShape(
        {rest[v[0]], rest[v[1]], rest[v[2]], rest[v[3]]});
  };
  const std::vector<vbd::TetMeshElement> tets
      = {{{0, 1, 2, 3}, makeRest({0, 1, 2, 3})},
         {{1, 2, 3, 4}, makeRest({1, 2, 3, 4})}};

  std::vector<Vec3> inertialTargets = rest;
  for (std::size_t i = 0; i < inertialTargets.size(); ++i) {
    if (fixed[i] == 0u) {
      inertialTargets[i] += Vec3(0.0, -0.03, 0.01);
    }
  }

  const double mu = 3000.0;
  const double lambda = 6000.0;
  const double springStiffness = 500.0;
  const double timeStep = 0.01;
  const auto coloring = vbd::colorDeformable(rest.size(), springs, tets);
  const auto springAdjacency
      = vbd::SpringAdjacency::build(rest.size(), springs);
  const auto tetAdjacency = vbd::TetAdjacency::build(rest.size(), tets);

  vbd::BlockDescentOptions options;
  options.iterations = 50;

  std::vector<Vec3> serial = inertialTargets;
  vbd::blockDescentDeformable(
      serial,
      masses,
      fixed,
      inertialTargets,
      springs,
      springStiffness,
      springAdjacency,
      tets,
      mu,
      lambda,
      tetAdjacency,
      timeStep,
      coloring,
      options);

  for (const unsigned int threads : {2u, 4u}) {
    std::vector<Vec3> parallel = inertialTargets;
    compute::ParallelExecutor executor(threads);
    vbd::parallelBlockDescentDeformable(
        parallel,
        masses,
        fixed,
        inertialTargets,
        springs,
        springStiffness,
        springAdjacency,
        tets,
        mu,
        lambda,
        tetAdjacency,
        timeStep,
        coloring,
        options,
        executor);
    for (std::size_t i = 0; i < serial.size(); ++i) {
      EXPECT_NEAR((serial[i] - parallel[i]).norm(), 0.0, 1e-12)
          << "threads=" << threads << " vertex=" << i;
    }
  }
}

//==============================================================================
// The public convergence threshold must select the same stopping sweep for
// serial and executor-parallel deformable solves.
TEST(VbdParallelBlockDescent, DeformableHonorsConvergenceControl)
{
  const GridScene scene = makeGrid(6);
  const std::vector<vbd::TetMeshElement> tets;
  const vbd::TetAdjacency tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), tets);
  const vbd::VertexColoring coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, tets);
  vbd::BlockDescentOptions options;
  options.iterations = 9;
  options.convergenceDisplacement = 1e100;

  std::vector<Vec3> serial = scene.inertialTargets;
  const vbd::BlockDescentStats serialStats = vbd::blockDescentDeformable(
      serial,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.adjacency,
      tets,
      /*mu=*/0.0,
      /*lambda=*/0.0,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);
  ASSERT_EQ(serialStats.iterations, 1u);

  for (const unsigned int threads : {2u, 4u}) {
    std::vector<Vec3> parallel = scene.inertialTargets;
    compute::ParallelExecutor executor(threads);
    const vbd::BlockDescentStats parallelStats
        = vbd::parallelBlockDescentDeformable(
            parallel,
            scene.masses,
            scene.fixed,
            scene.inertialTargets,
            scene.springs,
            scene.stiffness,
            scene.adjacency,
            tets,
            /*mu=*/0.0,
            /*lambda=*/0.0,
            tetAdjacency,
            scene.timeStep,
            coloring,
            options,
            executor);
    EXPECT_EQ(parallelStats.iterations, serialStats.iterations)
        << "threads=" << threads;
    EXPECT_EQ(parallelStats.vertexUpdates, serialStats.vertexUpdates)
        << "threads=" << threads;
    for (std::size_t vertex = 0; vertex < serial.size(); ++vertex) {
      EXPECT_NEAR((parallel[vertex] - serial[vertex]).norm(), 0.0, 1e-12)
          << "threads=" << threads << " vertex=" << vertex;
    }
  }
}

//==============================================================================
// Chebyshev is part of BlockDescentOptions, so executor dispatch must preserve
// both its numerical effect and exact sweep accounting.
TEST(VbdParallelBlockDescent, DeformableHonorsChebyshevControl)
{
  const GridScene scene = makeGrid(6);
  const std::vector<vbd::TetMeshElement> tets;
  const vbd::TetAdjacency tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), tets);
  const vbd::VertexColoring coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, tets);
  vbd::BlockDescentOptions options;
  options.iterations = 4;
  options.useChebyshev = true;
  options.chebyshevRho = 0.8;

  const auto runParallelDeformable = [&](unsigned int threads, bool chebyshev) {
    std::vector<Vec3> positions = scene.inertialTargets;
    compute::ParallelExecutor executor(threads);
    vbd::BlockDescentOptions runOptions = options;
    runOptions.useChebyshev = chebyshev;
    const vbd::BlockDescentStats stats = vbd::parallelBlockDescentDeformable(
        positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.stiffness,
        scene.adjacency,
        tets,
        /*mu=*/0.0,
        /*lambda=*/0.0,
        tetAdjacency,
        scene.timeStep,
        coloring,
        runOptions,
        executor);
    return std::pair{positions, stats};
  };

  std::vector<Vec3> serial = scene.inertialTargets;
  const vbd::BlockDescentStats serialStats = vbd::blockDescentDeformable(
      serial,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.adjacency,
      tets,
      /*mu=*/0.0,
      /*lambda=*/0.0,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);

  for (const unsigned int threads : {2u, 4u}) {
    const auto [parallel, parallelStats] = runParallelDeformable(threads, true);
    EXPECT_EQ(parallelStats.iterations, serialStats.iterations)
        << "threads=" << threads;
    EXPECT_EQ(parallelStats.vertexUpdates, serialStats.vertexUpdates)
        << "threads=" << threads;
    for (std::size_t vertex = 0; vertex < serial.size(); ++vertex) {
      EXPECT_NEAR((parallel[vertex] - serial[vertex]).norm(), 0.0, 1e-12)
          << "threads=" << threads << " vertex=" << vertex;
    }
  }

  const auto [plain, plainStats] = runParallelDeformable(4u, false);
  EXPECT_EQ(plainStats.iterations, options.iterations);
  EXPECT_GT(maxFreeDisplacement(plain, serial, scene.fixed), 1e-8);
}

//==============================================================================
// Chebyshev can amplify a small raw block update into a larger final per-node
// sweep displacement. Convergence must be evaluated after that extrapolation.
TEST(VbdParallelBlockDescent, SerialConvergenceUsesPostChebyshevDisplacement)
{
  const GridScene scene = makeGrid(6);
  const std::vector<vbd::TetMeshElement> tets;
  const vbd::TetAdjacency tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), tets);
  const vbd::VertexColoring coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, tets);

  const auto onePlainSweep = [&](const std::vector<Vec3>& initial) {
    std::vector<Vec3> positions = initial;
    vbd::BlockDescentOptions options;
    options.iterations = 1;
    vbd::blockDescentDeformable(
        positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.stiffness,
        scene.adjacency,
        tets,
        /*mu=*/0.0,
        /*lambda=*/0.0,
        tetAdjacency,
        scene.timeStep,
        coloring,
        options);
    return positions;
  };

  const std::vector<Vec3> initial = scene.inertialTargets;
  const std::vector<Vec3> first = onePlainSweep(initial);
  const std::vector<Vec3> secondSwept = onePlainSweep(first);
  std::vector<Vec3> secondAccelerated = secondSwept;
  const double rho = 0.8;
  const double omega = vbd::chebyshevOmega(2u, rho, 1.0);
  for (std::size_t vertex = 0; vertex < secondAccelerated.size(); ++vertex) {
    if (scene.fixed[vertex] == 0u) {
      secondAccelerated[vertex]
          = vbd::applyChebyshev(omega, secondSwept[vertex], initial[vertex]);
    }
  }

  const double firstDisplacement
      = maxFreeDisplacement(initial, first, scene.fixed);
  const double rawSecondDisplacement
      = maxFreeDisplacement(first, secondSwept, scene.fixed);
  const double finalSecondDisplacement
      = maxFreeDisplacement(first, secondAccelerated, scene.fixed);
  const double continuingUpperBound
      = std::min(firstDisplacement, finalSecondDisplacement);
  ASSERT_LT(rawSecondDisplacement, continuingUpperBound);
  const double convergenceDisplacement
      = 0.5 * (rawSecondDisplacement + continuingUpperBound);

  std::vector<Vec3> positions = initial;
  vbd::BlockDescentOptions options;
  options.iterations = 3;
  options.convergenceDisplacement = convergenceDisplacement;
  options.useChebyshev = true;
  options.chebyshevRho = rho;
  const vbd::BlockDescentStats stats = vbd::blockDescentDeformable(
      positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.adjacency,
      tets,
      /*mu=*/0.0,
      /*lambda=*/0.0,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);

  EXPECT_EQ(stats.iterations, 3u);
}

//==============================================================================
// Self-contact stencils are step-lagged and not represented in the cached
// spring/tet coloring. The parallel deformable driver must therefore fall back
// to the serial path while self-contact is active instead of updating one
// stencil vertex while another same-color thread reads it.
TEST(VbdParallelBlockDescent, ActiveSelfContactFallsBackToSerial)
{
  const std::vector<Vec3> rest
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<double> masses(rest.size(), 1.0);
  const std::vector<std::uint8_t> fixed(rest.size(), 0u);
  const std::vector<Vec3> inertialTargets = rest;
  const std::vector<vbd::SpringElement> springs;
  const std::vector<vbd::TetMeshElement> tets;
  const vbd::SpringAdjacency springAdjacency
      = vbd::SpringAdjacency::build(rest.size(), springs);
  const vbd::TetAdjacency tetAdjacency
      = vbd::TetAdjacency::build(rest.size(), tets);
  const vbd::VertexColoring coloring
      = vbd::colorDeformable(rest.size(), springs, tets);

  dc::ContactCandidateSet candidates;
  candidates.pointTriangleCandidates.push_back(
      {/*point=*/0, /*triangle=*/0, 0.0});
  const std::vector<sim::DeformableSurfaceTriangle> triangles = {{1, 2, 3}};
  const vbd::SelfContactAdjacency selfContact
      = vbd::SelfContactAdjacency::build(
          rest.size(),
          candidates,
          triangles,
          /*squaredActivationDistance=*/4e-4,
          1e5);
  ASSERT_TRUE(selfContact.active());

  vbd::BlockDescentOptions options;
  options.iterations = 5;
  // The serial deformable driver honors this early-termination threshold and
  // exits after one sweep; the true parallel path ignores it and would report
  // the full iteration budget.
  options.convergenceDisplacement = 1e100;

  std::vector<Vec3> serial = rest;
  const vbd::BlockDescentStats serialStats = vbd::blockDescentDeformable(
      serial,
      masses,
      fixed,
      inertialTargets,
      springs,
      /*springStiffness=*/0.0,
      springAdjacency,
      tets,
      /*mu=*/0.0,
      /*lambda=*/0.0,
      tetAdjacency,
      /*timeStep=*/0.01,
      coloring,
      options,
      {},
      {},
      0.0,
      &selfContact);

  std::vector<Vec3> parallel = rest;
  compute::ParallelExecutor executor(4);
  const vbd::BlockDescentStats parallelStats
      = vbd::parallelBlockDescentDeformable(
          parallel,
          masses,
          fixed,
          inertialTargets,
          springs,
          /*springStiffness=*/0.0,
          springAdjacency,
          tets,
          /*mu=*/0.0,
          /*lambda=*/0.0,
          tetAdjacency,
          /*timeStep=*/0.01,
          coloring,
          options,
          executor,
          {},
          {},
          0.0,
          &selfContact);

  EXPECT_EQ(serialStats.iterations, 1u);
  EXPECT_EQ(parallelStats.iterations, serialStats.iterations);
  ASSERT_EQ(serial.size(), parallel.size());
  for (std::size_t i = 0; i < serial.size(); ++i) {
    EXPECT_NEAR((serial[i] - parallel[i]).norm(), 0.0, 1e-12) << "vertex=" << i;
  }
}
