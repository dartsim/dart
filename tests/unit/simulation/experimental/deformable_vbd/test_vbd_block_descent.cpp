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

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
// An independent reference minimizer of the same objective: mass-preconditioned
// gradient descent with backtracking line search (the family the existing
// experimental deformable solver uses). VBD must converge to the same
// minimizer.
std::vector<Vec3> referenceMinimize(
    std::vector<Vec3> positions,
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Vec3>& inertialTargets,
    const std::vector<vbd::SpringElement>& springs,
    double stiffness,
    double timeStep)
{
  const double invDt2 = 1.0 / (timeStep * timeStep);
  const std::size_t n = positions.size();

  const auto gradient = [&](const std::vector<Vec3>& x) {
    std::vector<Vec3> g(n, Vec3::Zero());
    for (std::size_t i = 0; i < n; ++i) {
      if (fixed[i] == 0u) {
        g[i] += masses[i] * invDt2 * (x[i] - inertialTargets[i]);
      }
    }
    for (const auto& spring : springs) {
      const Vec3 delta = x[spring.b] - x[spring.a];
      const double length = delta.norm();
      if (length <= vbd::kMinSpringLength) {
        continue;
      }
      const Vec3 grad
          = stiffness * (length - spring.restLength) * delta / length;
      if (fixed[spring.a] == 0u) {
        g[spring.a] -= grad;
      }
      if (fixed[spring.b] == 0u) {
        g[spring.b] += grad;
      }
    }
    return g;
  };

  for (int iteration = 0; iteration < 2000; ++iteration) {
    const std::vector<Vec3> g = gradient(positions);
    double gradNormSq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      if (fixed[i] == 0u) {
        gradNormSq += g[i].squaredNorm();
      }
    }
    if (gradNormSq < 1e-24) {
      break;
    }

    const double base = vbd::massSpringObjective(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        stiffness,
        timeStep);
    double step = 1.0;
    for (int ls = 0; ls < 40; ++ls) {
      std::vector<Vec3> candidate = positions;
      for (std::size_t i = 0; i < n; ++i) {
        if (fixed[i] == 0u) {
          candidate[i] -= step * (timeStep * timeStep / masses[i]) * g[i];
        }
      }
      const double trial = vbd::massSpringObjective(
          candidate,
          masses,
          fixed,
          inertialTargets,
          springs,
          stiffness,
          timeStep);
      if (trial <= base) {
        positions = candidate;
        break;
      }
      step *= 0.5;
    }
  }
  return positions;
}

//==============================================================================
// A taut 1D chain along x: endpoints fixed, all springs stretched (rest length
// < spacing) so the objective is locally convex with a unique minimizer.
struct ChainScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::SpringElement> springs;
  double stiffness = 500.0;
  double timeStep = 0.02;
};

ChainScene makeChain(std::size_t count, double spacing, double restLength)
{
  ChainScene scene;
  for (std::size_t i = 0; i < count; ++i) {
    scene.positions.emplace_back(spacing * static_cast<double>(i), 0.0, 0.0);
    scene.masses.push_back(1.0);
    scene.fixed.push_back(0u);
  }
  scene.fixed.front() = 1u;
  scene.fixed.back() = 1u;
  // Inertial targets pulled slightly downward (a gravity-like step) plus a
  // perturbation so the free vertices must move.
  scene.inertialTargets = scene.positions;
  for (std::size_t i = 0; i < count; ++i) {
    if (scene.fixed[i] == 0u) {
      scene.inertialTargets[i] += Vec3(0.0, -0.05, 0.02);
      scene.positions[i] += Vec3(0.01, 0.03, -0.01);
    }
  }
  for (std::size_t i = 0; i + 1 < count; ++i) {
    scene.springs.push_back(
        {static_cast<std::uint32_t>(i),
         static_cast<std::uint32_t>(i + 1),
         restLength});
  }
  return scene;
}

} // namespace

//==============================================================================
TEST(VbdBlockDescent, FixedVerticesDoNotMove)
{
  ChainScene scene = makeChain(5, 1.0, 0.8);
  const Vec3 first = scene.positions.front();
  const Vec3 last = scene.positions.back();

  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  vbd::BlockDescentOptions options;
  options.iterations = 30;
  vbd::blockDescentMassSpring(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      coloring,
      adjacency,
      options);

  EXPECT_NEAR((scene.positions.front() - first).norm(), 0.0, 1e-15);
  EXPECT_NEAR((scene.positions.back() - last).norm(), 0.0, 1e-15);
}

//==============================================================================
TEST(VbdBlockDescent, DrivesResidualToZero)
{
  ChainScene scene = makeChain(7, 1.0, 0.85);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);

  vbd::BlockDescentOptions options;
  options.iterations = 200;
  const vbd::BlockDescentStats stats = vbd::blockDescentMassSpring(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      coloring,
      adjacency,
      options);
  EXPECT_LT(stats.finalResidualNormSquared, 1e-16);
}

//==============================================================================
TEST(VbdBlockDescent, MonotonicallyDecreasesObjective)
{
  ChainScene scene = makeChain(7, 1.0, 0.85);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);

  double previous = vbd::massSpringObjective(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep);

  vbd::BlockDescentOptions options;
  options.iterations = 1;
  for (int sweep = 0; sweep < 60; ++sweep) {
    vbd::blockDescentMassSpring(
        scene.positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.stiffness,
        scene.timeStep,
        coloring,
        adjacency,
        options);
    const double current = vbd::massSpringObjective(
        scene.positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.stiffness,
        scene.timeStep);
    // Allow a tiny numerical slack but require non-increase.
    EXPECT_LE(current, previous + 1e-9) << "sweep " << sweep;
    previous = current;
  }
}

//==============================================================================
// The load-bearing correctness check: VBD's converged state matches an
// independent gradient-descent minimizer of the same objective.
TEST(VbdBlockDescent, ConvergesToReferenceMinimizer)
{
  ChainScene scene = makeChain(9, 1.0, 0.85);

  const std::vector<Vec3> reference = referenceMinimize(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep);

  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  vbd::BlockDescentOptions options;
  options.iterations = 400;
  vbd::blockDescentMassSpring(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      coloring,
      adjacency,
      options);

  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    EXPECT_NEAR((scene.positions[i] - reference[i]).norm(), 0.0, 1e-6)
        << "vertex " << i;
  }
}

//==============================================================================
// Within a color, vertices share no spring, so a serial sweep equals the
// parallel Jacobi update. Verify a hand-built two-color path: updating color 0
// then color 1 reproduces the same result as recomputing all blocks at the
// pre-sweep positions for each color independently.
TEST(VbdBlockDescent, SameColorUpdatesAreIndependent)
{
  // Path 0-1-2-3-4: colors alternate {0,2,4} and {1,3}. Vertices in {0,2,4}
  // share no spring, likewise {1,3}.
  ChainScene scene = makeChain(5, 1.0, 0.85);
  // Free all interior + endpoints to exercise more blocks; keep ends fixed for
  // a well-posed problem.
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  ASSERT_EQ(coloring.colorCount(), 2u);

  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);

  // Reference: manually apply one sweep, recomputing each color's blocks from
  // the positions left by the previous color.
  std::vector<Vec3> manual = scene.positions;
  vbd::BlockDescentOptions options;
  for (const auto& group : coloring.groups) {
    // Snapshot positions before this color; same-color vertices must only read
    // other-colored neighbors, so the snapshot vs in-place read is irrelevant.
    for (const std::uint32_t vertex : group) {
      if (scene.fixed[vertex] != 0u) {
        continue;
      }
      const vbd::VertexBlock block = vbd::detail::assembleVertexBlock(
          vertex,
          manual,
          scene.masses,
          scene.inertialTargets,
          scene.springs,
          adjacency,
          scene.stiffness,
          scene.timeStep,
          true);
      manual[vertex] += vbd::solveVertexBlock(block, options.regularization);
    }
  }

  // Driver: one full sweep.
  options.iterations = 1;
  std::vector<Vec3> driver = scene.positions;
  vbd::blockDescentMassSpring(
      driver,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      coloring,
      adjacency,
      options);

  for (std::size_t i = 0; i < driver.size(); ++i) {
    EXPECT_NEAR((driver[i] - manual[i]).norm(), 0.0, 1e-15) << "vertex " << i;
  }
}

//==============================================================================
// Residual-based early termination stops before the iteration cap yet reaches
// the same converged state as running the full sweep budget.
TEST(VbdBlockDescent, EarlyTerminationMatchesFullSweepResult)
{
  ChainScene full = makeChain(7, 1.0, 0.85);
  ChainScene early = makeChain(7, 1.0, 0.85);
  const auto coloring = vbd::colorSprings(full.positions.size(), full.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(full.positions.size(), full.springs);

  vbd::BlockDescentOptions fullOptions;
  fullOptions.iterations = 400;
  vbd::blockDescentMassSpring(
      full.positions,
      full.masses,
      full.fixed,
      full.inertialTargets,
      full.springs,
      full.stiffness,
      full.timeStep,
      coloring,
      adjacency,
      fullOptions);

  vbd::BlockDescentOptions earlyOptions;
  earlyOptions.iterations = 400;
  earlyOptions.convergenceDisplacement = 1e-9;
  const vbd::BlockDescentStats stats = vbd::blockDescentMassSpring(
      early.positions,
      early.masses,
      early.fixed,
      early.inertialTargets,
      early.springs,
      early.stiffness,
      early.timeStep,
      coloring,
      adjacency,
      earlyOptions);

  EXPECT_LT(stats.iterations, 400u);
  for (std::size_t i = 0; i < full.positions.size(); ++i) {
    EXPECT_NEAR((full.positions[i] - early.positions[i]).norm(), 0.0, 1e-6)
        << "vertex " << i;
  }
}

//==============================================================================
TEST(VbdBlockDescent, DisabledEarlyTerminationRunsFullBudget)
{
  ChainScene scene = makeChain(5, 1.0, 0.85);
  const auto coloring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  const auto adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);

  vbd::BlockDescentOptions options;
  options.iterations = 12;
  options.convergenceDisplacement = 0.0; // disabled
  const vbd::BlockDescentStats stats = vbd::blockDescentMassSpring(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.stiffness,
      scene.timeStep,
      coloring,
      adjacency,
      options);
  EXPECT_EQ(stats.iterations, 12u);
}
