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

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
// A two-tetrahedron "bar" sharing the face (1,2,3); both tets have positive
// rest volume.
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

TetScene makeTwoTetBar()
{
  TetScene scene;
  scene.positions
      = {Vec3(0, 0, 0),
         Vec3(1, 0, 0),
         Vec3(0, 1, 0),
         Vec3(0, 0, 1),
         Vec3(1, 1, 1)};
  scene.masses.assign(scene.positions.size(), 1.0);
  scene.fixed.assign(scene.positions.size(), 0u);

  const auto restShape = [&](std::array<std::uint32_t, 4> v) {
    return vbd::makeTetRestShape(
        {scene.positions[v[0]],
         scene.positions[v[1]],
         scene.positions[v[2]],
         scene.positions[v[3]]});
  };
  scene.tets.push_back({{0, 1, 2, 3}, restShape({0, 1, 2, 3})});
  scene.tets.push_back({{1, 2, 3, 4}, restShape({1, 2, 3, 4})});

  // Inertial targets perturbed (a gravity-like step) so the body must deform.
  scene.inertialTargets = scene.positions;
  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    scene.inertialTargets[i] += Vec3(0.0, -0.02, 0.0);
    scene.positions[i] += Vec3(0.005, 0.0, -0.005);
  }
  return scene;
}

//==============================================================================
// Independent reference minimizer: global mass-preconditioned gradient descent
// with backtracking on the tet-mesh objective. The per-vertex gradient is the
// negated block force (which includes the inertia term), so this exercises a
// different algorithm (global simultaneous descent) than the per-vertex block
// coordinate descent under test.
std::vector<Vec3> referenceMinimizeTet(const TetScene& scene)
{
  std::vector<Vec3> positions = scene.positions;
  const std::size_t n = positions.size();
  const auto adjacency = vbd::TetAdjacency::build(n, scene.tets);
  const double h2 = scene.timeStep * scene.timeStep;

  for (int iteration = 0; iteration < 8000; ++iteration) {
    std::vector<Vec3> gradient(n, Vec3::Zero());
    double gradNormSq = 0.0;
    for (std::uint32_t i = 0; i < n; ++i) {
      if (scene.fixed[i] != 0u) {
        continue;
      }
      const vbd::VertexBlock block = vbd::detail::assembleTetVertexBlock(
          i,
          positions,
          scene.masses,
          scene.inertialTargets,
          scene.tets,
          adjacency,
          scene.mu,
          scene.lambda,
          scene.timeStep);
      gradient[i] = -block.force; // dG/dx_i
      gradNormSq += gradient[i].squaredNorm();
    }
    if (gradNormSq < 1e-26) {
      break;
    }

    const double base = vbd::tetMeshObjective(
        positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.tets,
        scene.mu,
        scene.lambda,
        scene.timeStep);
    double step = 1.0;
    for (int ls = 0; ls < 50; ++ls) {
      std::vector<Vec3> candidate = positions;
      for (std::uint32_t i = 0; i < n; ++i) {
        if (scene.fixed[i] == 0u) {
          candidate[i] -= step * (h2 / scene.masses[i]) * gradient[i];
        }
      }
      const double trial = vbd::tetMeshObjective(
          candidate,
          scene.masses,
          scene.fixed,
          scene.inertialTargets,
          scene.tets,
          scene.mu,
          scene.lambda,
          scene.timeStep);
      if (trial <= base) {
        positions = candidate;
        break;
      }
      step *= 0.5;
    }
  }
  return positions;
}

} // namespace

//==============================================================================
TEST(VbdTetMeshDescent, ColoringIsProperForTheMesh)
{
  const TetScene scene = makeTwoTetBar();
  vbd::VertexAdjacency adjacency(scene.positions.size());
  for (const auto& tet : scene.tets) {
    adjacency.addTetrahedron(
        tet.vertices[0], tet.vertices[1], tet.vertices[2], tet.vertices[3]);
  }
  const vbd::VertexColoring coloring
      = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
}

//==============================================================================
TEST(VbdTetMeshDescent, RestConfigurationStaysAtRest)
{
  TetScene scene = makeTwoTetBar();
  // Override: start exactly at rest with inertial targets at rest (no motion).
  scene.positions
      = {Vec3(0, 0, 0),
         Vec3(1, 0, 0),
         Vec3(0, 1, 0),
         Vec3(0, 0, 1),
         Vec3(1, 1, 1)};
  scene.inertialTargets = scene.positions;
  const std::vector<Vec3> initial = scene.positions;

  const auto coloring = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  const auto adjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  vbd::BlockDescentOptions options;
  options.iterations = 20;
  const vbd::BlockDescentStats stats = vbd::blockDescentTetMesh(
      scene.positions,
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

  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    EXPECT_NEAR((scene.positions[i] - initial[i]).norm(), 0.0, 1e-9)
        << "vertex " << i;
  }
  EXPECT_LT(stats.finalResidualNormSquared, 1e-12);
}

//==============================================================================
TEST(VbdTetMeshDescent, DrivesResidualToZero)
{
  TetScene scene = makeTwoTetBar();
  const auto coloring = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  const auto adjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  vbd::BlockDescentOptions options;
  options.iterations = 300;
  const vbd::BlockDescentStats stats = vbd::blockDescentTetMesh(
      scene.positions,
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
  EXPECT_LT(stats.finalResidualNormSquared, 1e-14);
}

//==============================================================================
TEST(VbdTetMeshDescent, MonotonicallyDecreasesObjective)
{
  TetScene scene = makeTwoTetBar();
  const auto coloring = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  const auto adjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);

  double previous = vbd::tetMeshObjective(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.tets,
      scene.mu,
      scene.lambda,
      scene.timeStep);

  vbd::BlockDescentOptions options;
  options.iterations = 1;
  for (int sweep = 0; sweep < 80; ++sweep) {
    vbd::blockDescentTetMesh(
        scene.positions,
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
    const double current = vbd::tetMeshObjective(
        scene.positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.tets,
        scene.mu,
        scene.lambda,
        scene.timeStep);
    EXPECT_LE(current, previous + 1e-9) << "sweep " << sweep;
    previous = current;
  }
}

//==============================================================================
// The load-bearing volumetric correctness check: VBD's converged state matches
// an independent global gradient-descent minimizer of the same objective.
TEST(VbdTetMeshDescent, ConvergesToReferenceMinimizer)
{
  TetScene scene = makeTwoTetBar();
  const std::vector<Vec3> reference = referenceMinimizeTet(scene);

  const auto coloring = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  const auto adjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  vbd::BlockDescentOptions options;
  options.iterations = 500;
  vbd::blockDescentTetMesh(
      scene.positions,
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

  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    EXPECT_NEAR((scene.positions[i] - reference[i]).norm(), 0.0, 1e-6)
        << "vertex " << i;
  }
}
