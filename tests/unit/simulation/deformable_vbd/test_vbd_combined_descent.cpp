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
#include <dart/simulation/detail/deformable_elasticity/fem_tet_element.hpp>
#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include <cmath>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace common = dart::common;
namespace compute = dart::simulation::compute;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
// A two-tetrahedron bar sharing face (1,2,3), with extra distance springs along
// some of its edges, so a single body carries both spring and Neo-Hookean tet
// energy at once.
struct CombinedScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::SpringElement> springs;
  std::vector<vbd::TetMeshElement> tets;
  double springStiffness = 500.0;
  double mu = 3000.0;
  double lambda = 6000.0;
  double timeStep = 0.01;
};

CombinedScene makeSpringTetBar()
{
  CombinedScene scene;
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

  const auto restLength = [&](std::uint32_t a, std::uint32_t b) {
    return (scene.positions[a] - scene.positions[b]).norm();
  };
  // A couple of distance springs across the bar.
  scene.springs.push_back({0, 4, restLength(0, 4)});
  scene.springs.push_back({0, 1, restLength(0, 1)});

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
// with backtracking on the combined springs + tets objective. The per-vertex
// gradient is the negated combined block force, so this exercises a different
// algorithm (global simultaneous descent) than the per-vertex block coordinate
// descent under test.
std::vector<Vec3> referenceMinimize(const CombinedScene& scene)
{
  std::vector<Vec3> positions = scene.positions;
  const std::size_t n = positions.size();
  const auto springAdjacency = vbd::SpringAdjacency::build(n, scene.springs);
  const auto tetAdjacency = vbd::TetAdjacency::build(n, scene.tets);
  const double h2 = scene.timeStep * scene.timeStep;

  const auto objective = [&](const std::vector<Vec3>& p) {
    return vbd::deformableObjective(
        p,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.springStiffness,
        scene.tets,
        scene.mu,
        scene.lambda,
        scene.timeStep);
  };

  for (int iteration = 0; iteration < 8000; ++iteration) {
    std::vector<Vec3> gradient(n, Vec3::Zero());
    double gradNormSq = 0.0;
    for (std::uint32_t i = 0; i < n; ++i) {
      if (scene.fixed[i] != 0u) {
        continue;
      }
      const vbd::VertexBlock block = vbd::detail::assembleDeformableVertexBlock(
          i,
          positions,
          scene.masses,
          scene.inertialTargets,
          scene.springs,
          springAdjacency,
          scene.springStiffness,
          /*clampSpringHessian=*/true,
          scene.tets,
          tetAdjacency,
          scene.mu,
          scene.lambda,
          scene.timeStep);
      gradient[i] = -block.force; // dG/dx_i
      gradNormSq += gradient[i].squaredNorm();
    }
    if (gradNormSq < 1e-26) {
      break;
    }

    const double base = objective(positions);
    double step = 1.0;
    for (int ls = 0; ls < 50; ++ls) {
      std::vector<Vec3> candidate = positions;
      for (std::uint32_t i = 0; i < n; ++i) {
        if (scene.fixed[i] == 0u) {
          candidate[i] -= step * (h2 / scene.masses[i]) * gradient[i];
        }
      }
      if (objective(candidate) <= base) {
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
TEST(VbdCombinedDescent, ColoringIsProperForSpringsAndTets)
{
  const CombinedScene scene = makeSpringTetBar();
  vbd::VertexAdjacency adjacency(scene.positions.size());
  for (const auto& spring : scene.springs) {
    adjacency.addEdge(spring.a, spring.b);
  }
  for (const auto& tet : scene.tets) {
    adjacency.addTetrahedron(
        tet.vertices[0], tet.vertices[1], tet.vertices[2], tet.vertices[3]);
  }
  const vbd::VertexColoring coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, scene.tets);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
}

//==============================================================================
TEST(VbdCombinedDescent, TopologyAdjacencyUsesProvidedAllocator)
{
  const CombinedScene scene = makeSpringTetBar();
  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeListAllocator();
  const auto allocationsBefore = allocator.getAllocationCount();

  const vbd::SpringAdjacency springAdjacency = vbd::SpringAdjacency::build(
      scene.positions.size(), scene.springs, allocator);
  const vbd::TetAdjacency tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets, allocator);

  EXPECT_GT(allocator.getAllocationCount(), allocationsBefore);
  ASSERT_FALSE(springAdjacency.incidentSprings.empty());
  ASSERT_FALSE(tetAdjacency.incidentTets.empty());
  EXPECT_EQ(
      springAdjacency.incidentSprings.get_allocator(),
      common::StlAllocator<vbd::SpringAdjacency::IncidentSpringVector>{
          allocator});
  EXPECT_EQ(
      springAdjacency.incidentSprings[0].get_allocator(),
      common::StlAllocator<std::uint32_t>{allocator});
  EXPECT_EQ(
      tetAdjacency.incidentTets.get_allocator(),
      common::StlAllocator<vbd::TetAdjacency::IncidentTetVector>{allocator});
  EXPECT_EQ(
      tetAdjacency.incidentTets[0].get_allocator(),
      common::StlAllocator<vbd::TetAdjacency::IncidentTet>{allocator});
}

//==============================================================================
// With no tetrahedra, the combined driver must reproduce blockDescentMassSpring
// bit-for-bit (same assembly, same sweep order).
TEST(VbdCombinedDescent, ReducesToMassSpringWithoutTets)
{
  CombinedScene scene = makeSpringTetBar();
  scene.tets.clear();

  const std::vector<vbd::TetMeshElement> noTets;
  const auto springAdjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  const auto tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), noTets);
  const auto coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, noTets);

  vbd::BlockDescentOptions options;
  options.iterations = 50;

  std::vector<Vec3> combined = scene.positions;
  vbd::blockDescentDeformable(
      combined,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.springStiffness,
      springAdjacency,
      noTets,
      scene.mu,
      scene.lambda,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);

  std::vector<Vec3> massSpring = scene.positions;
  const auto springColoring
      = vbd::colorSprings(scene.positions.size(), scene.springs);
  vbd::blockDescentMassSpring(
      massSpring,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.springStiffness,
      scene.timeStep,
      springColoring,
      springAdjacency,
      options);

  for (std::size_t i = 0; i < combined.size(); ++i) {
    EXPECT_NEAR((combined[i] - massSpring[i]).norm(), 0.0, 1e-12)
        << "vertex " << i;
  }
}

//==============================================================================
// With no springs, the combined driver must reproduce blockDescentTetMesh.
TEST(VbdCombinedDescent, ReducesToTetMeshWithoutSprings)
{
  CombinedScene scene = makeSpringTetBar();
  scene.springs.clear();

  const std::vector<vbd::SpringElement> noSprings;
  const auto springAdjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), noSprings);
  const auto tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  const auto coloring
      = vbd::colorDeformable(scene.positions.size(), noSprings, scene.tets);

  vbd::BlockDescentOptions options;
  options.iterations = 50;

  std::vector<Vec3> combined = scene.positions;
  vbd::blockDescentDeformable(
      combined,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      noSprings,
      scene.springStiffness,
      springAdjacency,
      scene.tets,
      scene.mu,
      scene.lambda,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);

  std::vector<Vec3> tetOnly = scene.positions;
  const auto tetColoring
      = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  vbd::blockDescentTetMesh(
      tetOnly,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.tets,
      scene.mu,
      scene.lambda,
      scene.timeStep,
      tetColoring,
      tetAdjacency,
      options);

  for (std::size_t i = 0; i < combined.size(); ++i) {
    EXPECT_NEAR((combined[i] - tetOnly[i]).norm(), 0.0, 1e-12)
        << "vertex " << i;
  }
}

//==============================================================================
TEST(VbdCombinedDescent, DrivesResidualToZero)
{
  CombinedScene scene = makeSpringTetBar();
  const auto springAdjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  const auto tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  const auto coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, scene.tets);

  vbd::BlockDescentOptions options;
  options.iterations = 400;
  const vbd::BlockDescentStats stats = vbd::blockDescentDeformable(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.springStiffness,
      springAdjacency,
      scene.tets,
      scene.mu,
      scene.lambda,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);
  EXPECT_LT(stats.finalResidualNormSquared, 1e-12);
}

//==============================================================================
// The load-bearing correctness check: VBD's converged combined-energy state
// matches an independent global gradient-descent minimizer of the same
// combined objective.
TEST(VbdCombinedDescent, ConvergesToReferenceMinimizer)
{
  CombinedScene scene = makeSpringTetBar();
  const std::vector<Vec3> reference = referenceMinimize(scene);

  const auto springAdjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  const auto tetAdjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  const auto coloring
      = vbd::colorDeformable(scene.positions.size(), scene.springs, scene.tets);

  vbd::BlockDescentOptions options;
  options.iterations = 600;
  vbd::blockDescentDeformable(
      scene.positions,
      scene.masses,
      scene.fixed,
      scene.inertialTargets,
      scene.springs,
      scene.springStiffness,
      springAdjacency,
      scene.tets,
      scene.mu,
      scene.lambda,
      tetAdjacency,
      scene.timeStep,
      coloring,
      options);

  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    EXPECT_NEAR((scene.positions[i] - reference[i]).norm(), 0.0, 1e-6)
        << "vertex " << i;
  }
}

//==============================================================================
// Chebyshev over-relaxation accelerates convergence but must not move the fixed
// point: with a generous budget the accelerated solve lands on the same
// minimizer as the unaccelerated one.
TEST(VbdCombinedDescent, ChebyshevConvergesToSameMinimizer)
{
  const CombinedScene base = makeSpringTetBar();
  const auto springAdjacency
      = vbd::SpringAdjacency::build(base.positions.size(), base.springs);
  const auto tetAdjacency
      = vbd::TetAdjacency::build(base.positions.size(), base.tets);
  const auto coloring
      = vbd::colorDeformable(base.positions.size(), base.springs, base.tets);

  const auto solve = [&](bool useChebyshev) {
    CombinedScene scene = base;
    vbd::BlockDescentOptions options;
    options.iterations = 600;
    options.useChebyshev = useChebyshev;
    options.chebyshevRho = 0.9;
    vbd::blockDescentDeformable(
        scene.positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.springStiffness,
        springAdjacency,
        scene.tets,
        scene.mu,
        scene.lambda,
        tetAdjacency,
        scene.timeStep,
        coloring,
        options);
    return scene.positions;
  };

  const std::vector<Vec3> plain = solve(false);
  const std::vector<Vec3> accelerated = solve(true);
  for (std::size_t i = 0; i < plain.size(); ++i) {
    EXPECT_NEAR((plain[i] - accelerated[i]).norm(), 0.0, 1e-7)
        << "vertex " << i;
  }
}

//==============================================================================
// Rayleigh damping keeps the augmented per-vertex system positive definite (the
// solve still drives the residual down) while measurably changing the converged
// iterate relative to the undamped solve.
TEST(VbdCombinedDescent, RayleighDampingIsStableAndChangesResult)
{
  const CombinedScene base = makeSpringTetBar();
  const auto springAdjacency
      = vbd::SpringAdjacency::build(base.positions.size(), base.springs);
  const auto tetAdjacency
      = vbd::TetAdjacency::build(base.positions.size(), base.tets);
  const auto coloring
      = vbd::colorDeformable(base.positions.size(), base.springs, base.tets);

  const auto solve = [&](double rayleigh, double& residualOut) {
    CombinedScene scene = base;
    const std::vector<Vec3> stepStart = scene.positions;
    vbd::BlockDescentOptions options;
    options.iterations = 300;
    options.rayleighDamping = rayleigh;
    const vbd::BlockDescentStats stats = vbd::blockDescentDeformable(
        scene.positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.springStiffness,
        springAdjacency,
        scene.tets,
        scene.mu,
        scene.lambda,
        tetAdjacency,
        scene.timeStep,
        coloring,
        options,
        stepStart);
    residualOut = stats.finalResidualNormSquared;
    return scene.positions;
  };

  double undampedResidual = 0.0;
  double dampedResidual = 0.0;
  const std::vector<Vec3> undamped = solve(0.0, undampedResidual);
  const std::vector<Vec3> damped = solve(0.05, dampedResidual);

  // Both solves converge (the damped system stays SPD) ...
  EXPECT_LT(undampedResidual, 1e-12);
  EXPECT_LT(dampedResidual, 1e-12);
  // ... but damping opposes the step displacement, so the iterates differ.
  double maxDifference = 0.0;
  for (std::size_t i = 0; i < damped.size(); ++i) {
    maxDifference = std::max(maxDifference, (damped[i] - undamped[i]).norm());
  }
  EXPECT_GT(maxDifference, 1e-6);
}

//==============================================================================
// Contact barriers are inequality constraints, not elastic material stiffness:
// stiffness-proportional Rayleigh damping must ignore self-contact Hessians.
TEST(VbdCombinedDescent, RayleighDampingIgnoresSelfContactBarrierStiffness)
{
  namespace dc = dart::simulation::detail::deformable_contact;
  namespace sim = dart::simulation;

  const std::vector<Vec3> start
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<double> masses(start.size(), 1.0);
  const std::vector<std::uint8_t> fixed = {0u, 1u, 1u, 1u};
  const std::vector<Vec3> inertialTargets = start;
  const std::vector<vbd::SpringElement> springs;
  const std::vector<vbd::TetMeshElement> tets;
  const vbd::SpringAdjacency springAdjacency
      = vbd::SpringAdjacency::build(start.size(), springs);
  const vbd::TetAdjacency tetAdjacency
      = vbd::TetAdjacency::build(start.size(), tets);
  const vbd::VertexColoring coloring
      = vbd::colorDeformable(start.size(), springs, tets);

  dc::ContactCandidateSet candidates;
  candidates.pointTriangleCandidates.push_back(
      {/*point=*/0, /*triangle=*/0, 0.0});
  const std::vector<sim::DeformableSurfaceTriangle> triangles = {{1, 2, 3}};
  const vbd::SelfContactAdjacency selfContact
      = vbd::SelfContactAdjacency::build(
          start.size(),
          candidates,
          triangles,
          /*squaredActivationDistance=*/4e-4,
          /*stiffness=*/1e5);
  ASSERT_TRUE(selfContact.active());

  const std::vector<Vec3> stepStart
      = {start[0] - Vec3(0.0, 0.0, 0.003), start[1], start[2], start[3]};

  const auto solve = [&](double rayleigh) {
    std::vector<Vec3> positions = start;
    vbd::BlockDescentOptions options;
    options.iterations = 4;
    options.rayleighDamping = rayleigh;
    vbd::blockDescentDeformable(
        positions,
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
        stepStart,
        {},
        0.0,
        &selfContact);
    return positions;
  };

  const std::vector<Vec3> undamped = solve(0.0);
  const std::vector<Vec3> damped = solve(0.5);

  EXPECT_GT((undamped[0] - start[0]).norm(), 1e-8);
  ASSERT_EQ(damped.size(), undamped.size());
  for (std::size_t i = 0; i < damped.size(); ++i) {
    EXPECT_NEAR((damped[i] - undamped[i]).norm(), 0.0, 1e-12) << "node " << i;
  }
}

//==============================================================================
// Option B: with useFemTetKernel set, the VBD tetrahedral term is routed
// through the shared deformable_elasticity FEM kernels, so a VBD body honors
// the body's hyperelastic material. The per-vertex block force must equal the
// corresponding FEM element force, and selecting fixed-corotational must change
// the force away from Stable Neo-Hookean (the live divergence this fixes: VBD
// previously always applied its own Stable Neo-Hookean, ignoring the material
// choice).
TEST(VbdCombinedDescent, FemTetKernelHonorsMaterialChoice)
{
  namespace fem = dart::simulation::detail::deformable_elasticity;

  // One deformed (stretched + sheared) tetrahedron; inertial target == current
  // position so the inertia term contributes zero force and the block force is
  // exactly the elastic tet force.
  const std::array<Vec3, 4> rest
      = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
  const std::vector<Vec3> positions
      = {Vec3(0, 0, 0),
         Vec3(1.25, 0.12, -0.03),
         Vec3(-0.06, 1.10, 0.08),
         Vec3(0.02, 0.05, 0.92)};
  const std::vector<double> masses(4, 1.0);
  const std::vector<Vec3> inertialTargets = positions;
  std::vector<vbd::TetMeshElement> tets;
  tets.push_back({{0, 1, 2, 3}, vbd::makeTetRestShape(rest)});
  const auto adjacency = vbd::TetAdjacency::build(4, tets);
  const double mu = 3000.0;
  const double lambda = 6000.0;
  const double timeStep = 0.01;

  // Independent FEM reference element forces for vertex 0.
  fem::TetRestShape femRest;
  femRest.inverseRestEdges = tets[0].rest.restShapeInverse;
  femRest.restVolume = tets[0].rest.restVolume;
  femRest.valid = true;
  const fem::LameParameters lame{mu, lambda};
  const Vec3 snhForce0 = -fem::evaluateStableNeoHookeanTet(
                              positions[0],
                              positions[1],
                              positions[2],
                              positions[3],
                              femRest,
                              lame)
                              .gradient.segment<3>(0);
  const Vec3 fcrForce0 = -fem::evaluateFixedCorotationalTet(
                              positions[0],
                              positions[1],
                              positions[2],
                              positions[3],
                              femRest,
                              lame)
                              .gradient.segment<3>(0);

  // The two materials genuinely disagree on this deformed configuration.
  EXPECT_GT((snhForce0 - fcrForce0).norm(), 1e-6);

  const vbd::VertexBlock snhBlock = vbd::detail::assembleTetVertexBlock(
      0,
      positions,
      masses,
      inertialTargets,
      tets,
      adjacency,
      mu,
      lambda,
      timeStep,
      /*useFemTetKernel=*/true,
      /*useFixedCorotationalTets=*/false);
  const vbd::VertexBlock fcrBlock = vbd::detail::assembleTetVertexBlock(
      0,
      positions,
      masses,
      inertialTargets,
      tets,
      adjacency,
      mu,
      lambda,
      timeStep,
      /*useFemTetKernel=*/true,
      /*useFixedCorotationalTets=*/true);

  // Each routed block force matches its FEM element force exactly...
  EXPECT_LT((snhBlock.force - snhForce0).norm(), 1e-9);
  EXPECT_LT((fcrBlock.force - fcrForce0).norm(), 1e-9);
  // ...and the material selection actually changes the VBD force.
  EXPECT_GT((snhBlock.force - fcrBlock.force).norm(), 1e-6);
}

//==============================================================================
// Option A: a self-contact constraint scatters the IPC point-triangle / edge-
// edge barrier's 12-vector into per-vertex blocks. Each involved vertex's block
// must carry exactly that primitive's 3x1 force (negated gradient sub-block)
// and 3x3 diagonal Hessian block, and the repulsion must push a point hovering
// above a triangle away from it.
TEST(VbdCombinedDescent, SelfContactBlockMatchesBarrier)
{
  namespace dc = dart::simulation::detail::deformable_contact;
  namespace sim = dart::simulation;

  // A point 1 cm above a triangle, inside the barrier activation band.
  const std::vector<Vec3> positions
      = {Vec3(0.3, 0.3, 0.01), // point (node 0)
         Vec3(0.0, 0.0, 0.0),  // triangle node a (node 1)
         Vec3(1.0, 0.0, 0.0),  // b (node 2)
         Vec3(0.0, 1.0, 0.0)}; // c (node 3)
  const std::vector<sim::DeformableSurfaceTriangle> triangles = {{1, 2, 3}};
  dc::ContactCandidateSet candidates;
  candidates.pointTriangleCandidates.push_back(
      {/*point=*/0, /*triangle=*/0, 0.0});

  const double dHat = 0.02;
  const double kappa = 1.0e5;
  const auto adjacency = vbd::SelfContactAdjacency::build(
      positions.size(), candidates, triangles, dHat * dHat, kappa);
  ASSERT_TRUE(adjacency.active());

  const auto barrier = dc::pointTriangleBarrier(
      positions[0],
      positions[1],
      positions[2],
      positions[3],
      dHat * dHat,
      kappa);
  ASSERT_TRUE(barrier.active);

  // The point's block (local index 0) carries the barrier's vertex-0 sub-block.
  vbd::VertexBlock pointBlock;
  vbd::addSelfContactTerms(pointBlock, 0, adjacency, positions);
  EXPECT_LT(
      (pointBlock.force - (-barrier.gradient.segment<3>(0))).norm(), 1e-12);
  EXPECT_LT(
      (pointBlock.hessian - barrier.hessian.block<3, 3>(0, 0)).norm(), 1e-12);

  // A triangle node's block (node 1 = local index 1) carries its sub-block.
  vbd::VertexBlock triBlock;
  vbd::addSelfContactTerms(triBlock, 1, adjacency, positions);
  EXPECT_LT((triBlock.force - (-barrier.gradient.segment<3>(3))).norm(), 1e-12);
  EXPECT_LT(
      (triBlock.hessian - barrier.hessian.block<3, 3>(3, 3)).norm(), 1e-12);

  // The repulsion pushes the point up, away from the triangle below it.
  EXPECT_GT(pointBlock.force.z(), 0.0);
}

//==============================================================================
// The AVBD self-contact normal row uses the IPC barrier only to recover the
// primitive's local repulsion direction. The scalar row then applies AVBD's
// hard-contact bounded force and a rank-1 positive-semidefinite block Hessian.
TEST(VbdCombinedDescent, AvbdSelfContactNormalRowUsesBarrierDirection)
{
  namespace dc = dart::simulation::detail::deformable_contact;

  const std::vector<Vec3> positions
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const double dHat = 0.02;

  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0, 1, 2, 3};
  row.state.stiffness = 200.0;
  row.squaredActivationDistance = dHat * dHat;

  const dc::PrimitiveBarrierResult primitive = dc::pointTriangleBarrier(
      positions[0], positions[1], positions[2], positions[3], dHat * dHat, 1.0);
  ASSERT_TRUE(primitive.active);
  const Vec3 expectedDirection
      = (-primitive.gradient.segment<3>(0)).normalized();
  const double constraintValue
      = vbd::avbdSelfContactNormalConstraintValue(row, positions);
  ASSERT_GT(constraintValue, 0.0);

  vbd::VertexBlock block;
  const double forceMagnitude = vbd::addAvbdSelfContactNormal(
      block, positions, row, /*localVertex=*/0, /*alpha=*/0.0);

  EXPECT_NEAR(forceMagnitude, row.state.stiffness * constraintValue, 1e-12);
  EXPECT_NEAR(block.force.normalized().dot(expectedDirection), 1.0, 1e-12);
  EXPECT_GT(block.force.z(), 0.0);
  EXPECT_NEAR(
      (block.hessian
       - row.state.stiffness
             * (expectedDirection * expectedDirection.transpose()))
          .norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactNormalRowUpdatesDualState)
{
  const std::vector<Vec3> positions
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};

  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0, 1, 2, 3};
  row.state.stiffness = 10.0;
  row.state.lambda = 0.0;
  row.squaredActivationDistance = 4e-4;

  vbd::AvbdSelfContactNormalOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  const double constraintValue
      = vbd::avbdSelfContactNormalConstraintValue(row, positions);
  ASSERT_GT(constraintValue, 0.0);
  const vbd::AvbdScalarRowState updated
      = vbd::updateAvbdSelfContactNormalRow(row.state, positions, row, options);

  EXPECT_NEAR(updated.lambda, row.state.stiffness * constraintValue, 1e-12);
  EXPECT_NEAR(
      updated.stiffness,
      row.state.stiffness + options.beta * constraintValue,
      1e-12);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactEdgeEdgeRowUsesBarrierDirection)
{
  namespace dc = dart::simulation::detail::deformable_contact;

  const std::vector<Vec3> positions
      = {Vec3(-0.5, 0.0, 0.01),
         Vec3(0.5, 0.0, 0.01),
         Vec3(0.0, -0.5, 0.0),
         Vec3(0.0, 0.5, 0.0)};
  const double dHat = 0.02;

  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0, 1, 2, 3};
  row.isEdgeEdge = true;
  row.state.stiffness = 300.0;
  row.squaredActivationDistance = dHat * dHat;

  const dc::PrimitiveBarrierResult primitive = dc::edgeEdgeBarrier(
      positions[0], positions[1], positions[2], positions[3], dHat * dHat, 1.0);
  ASSERT_TRUE(primitive.active);
  const Vec3 expectedDirection
      = (-primitive.gradient.segment<3>(0)).normalized();

  vbd::VertexBlock block;
  const double forceMagnitude = vbd::addAvbdSelfContactNormal(
      block, positions, row, /*localVertex=*/0, /*alpha=*/0.0);

  EXPECT_GT(forceMagnitude, 0.0);
  EXPECT_NEAR(block.force.normalized().dot(expectedDirection), 1.0, 1e-12);
  EXPECT_GT(block.force.z(), 0.0);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactNormalRowNoopsOutsideActivationBand)
{
  const std::vector<Vec3> positions
      = {Vec3(0.3, 0.3, 0.05),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};

  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0, 1, 2, 3};
  row.state.stiffness = 200.0;
  row.squaredActivationDistance = 4e-4;

  vbd::VertexBlock block;
  const double forceMagnitude = vbd::addAvbdSelfContactNormal(
      block, positions, row, /*localVertex=*/0, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 0.0);
  EXPECT_DOUBLE_EQ(block.force.norm(), 0.0);
  EXPECT_DOUBLE_EQ(block.hessian.norm(), 0.0);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionTangentOpposesSlip)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<Vec3> positions
      = {stepStart[0] + Vec3(0.1, 0.0, 0.0),
         stepStart[1],
         stepStart[2],
         stepStart[3]};

  vbd::AvbdSelfContactFrictionRow row;
  row.nodes = {0, 1, 2, 3};
  row.stepStartPositions = stepStart;
  row.axis = 0;
  row.state.stiffness = 100.0;
  row.bounds = {-100.0, 100.0};

  vbd::VertexBlock block;
  const double forceMagnitude = vbd::addAvbdSelfContactFrictionTangent(
      block, positions, row, /*localVertex=*/0, /*alpha=*/0.0);

  EXPECT_NEAR(forceMagnitude, -10.0, 1e-12);
  EXPECT_NEAR(block.force.x(), -10.0, 1e-12);
  EXPECT_NEAR(block.force.y(), 0.0, 1e-12);
  EXPECT_NEAR(block.force.z(), 0.0, 1e-12);
  EXPECT_NEAR(block.hessian(0, 0), 100.0, 1e-12);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionPairProjectsStaticForceToCone)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<Vec3> positions
      = {stepStart[0] + Vec3(1.0, 1.0, 0.0),
         stepStart[1],
         stepStart[2],
         stepStart[3]};

  vbd::AvbdSelfContactFrictionRow rowX;
  rowX.nodes = {0, 1, 2, 3};
  rowX.stepStartPositions = stepStart;
  rowX.axis = 0;
  rowX.state.stiffness = 10.0;
  rowX.bounds = {-5.0, 5.0};

  vbd::AvbdSelfContactFrictionRow rowY = rowX;
  rowY.axis = 1;

  vbd::AvbdSelfContactFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_TRUE(vbd::avbdSelfContactFrictionPreviousDualInsideCone(rowX, rowY));
  bool clamped = false;
  const Eigen::Vector2d force = vbd::avbdSelfContactFrictionTangentPairForce(
      rowX, rowY, positions, options, &clamped);

  EXPECT_TRUE(clamped);
  EXPECT_NEAR(force.norm(), 5.0, 1e-12);
  EXPECT_NEAR(force.x(), force.y(), 1e-12);
  EXPECT_LT(force.x(), 0.0);

  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_NEAR(std::hypot(rowX.state.lambda, rowY.state.lambda), 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionPairSwitchesToDynamicSlip)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<Vec3> positions
      = {stepStart[0] + Vec3(0.0, 2.0, 0.0),
         stepStart[1],
         stepStart[2],
         stepStart[3]};

  vbd::AvbdSelfContactFrictionRow rowX;
  rowX.nodes = {0, 1, 2, 3};
  rowX.stepStartPositions = stepStart;
  rowX.axis = 0;
  rowX.state.stiffness = 10.0;
  rowX.state.lambda = -5.0;
  rowX.bounds = {-5.0, 5.0};

  vbd::AvbdSelfContactFrictionRow rowY = rowX;
  rowY.axis = 1;
  rowY.state.lambda = 0.0;

  vbd::AvbdSelfContactFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_FALSE(vbd::avbdSelfContactFrictionPreviousDualInsideCone(rowX, rowY));
  const Eigen::Vector2d force = vbd::avbdSelfContactFrictionTangentPairForce(
      rowX, rowY, positions, options);
  EXPECT_NEAR(force.x(), 0.0, 1e-12);
  EXPECT_NEAR(force.y(), -5.0, 1e-12);

  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_NEAR(rowX.state.lambda, 0.0, 1e-12);
  EXPECT_NEAR(rowY.state.lambda, -5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);
}

//==============================================================================
TEST(
    VbdCombinedDescent,
    AvbdSelfContactFrictionDualProjectionPreservesGeneralizedImpulse)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};

  vbd::AvbdSelfContactFrictionRow rowX;
  rowX.nodes = {0, 1, 2, 3};
  rowX.stepStartPositions = stepStart;
  rowX.axis = 0;
  rowX.state.lambda = 3.0;

  vbd::AvbdSelfContactFrictionRow rowY = rowX;
  rowY.axis = 1;
  rowY.state.lambda = 4.0;

  const Eigen::Vector2d unchanged
      = vbd::projectAvbdSelfContactFrictionDualToTangentPair(
          rowX.state.lambda, rowY.state.lambda, rowX, rowY, rowX, rowY);
  EXPECT_NEAR(unchanged.x(), 3.0, 1e-12);
  EXPECT_NEAR(unchanged.y(), 4.0, 1e-12);

  const std::array<Vec3, 4> rotatedStepStart
      = {Vec3(-0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0),
         Vec3(-1.0, 0.0, 0.0)};
  vbd::AvbdSelfContactFrictionRow rotatedX = rowX;
  rotatedX.stepStartPositions = rotatedStepStart;
  rotatedX.axis = 0;
  vbd::AvbdSelfContactFrictionRow rotatedY = rotatedX;
  rotatedY.axis = 1;

  const Eigen::Vector2d projected
      = vbd::projectAvbdSelfContactFrictionDualToTangentPair(
          rowX.state.lambda, rowY.state.lambda, rowX, rowY, rotatedX, rotatedY);
  EXPECT_NEAR(projected.x(), 4.0, 1e-12);
  EXPECT_NEAR(projected.y(), -3.0, 1e-12);
  EXPECT_NEAR(projected.norm(), 5.0, 1e-12);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionRowsReduceTangentialMotion)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<double> masses(stepStart.size(), 1.0);
  const std::vector<std::uint8_t> fixed = {0u, 1u, 1u, 1u};
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(stepStart.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(stepStart.size(), springs);

  std::vector<Vec3> inertialTargets(stepStart.begin(), stepStart.end());
  inertialTargets[0] += Vec3(1.0, 0.0, 0.0);

  const auto solve = [&](bool enableFriction) {
    std::vector<Vec3> positions(stepStart.begin(), stepStart.end());
    std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
    std::vector<vbd::AvbdPointAttachmentRow> attachments;
    std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;
    std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows;

    vbd::AvbdSelfContactFrictionRow rowX;
    rowX.nodes = {0, 1, 2, 3};
    rowX.stepStartPositions = stepStart;
    rowX.axis = 0;
    rowX.state.stiffness = 200.0;
    rowX.bounds = {-100.0, 100.0};
    vbd::AvbdSelfContactFrictionRow rowY = rowX;
    rowY.axis = 1;
    if (enableFriction) {
      frictionRows = {rowX, rowY};
    }

    vbd::BlockDescentOptions options;
    options.iterations = 4;
    vbd::AvbdHalfSpaceContactOptions contactOptions;
    vbd::AvbdPointAttachmentOptions attachmentOptions;
    vbd::AvbdSpringFiniteStiffnessOptions springOptions;
    vbd::AvbdSelfContactFrictionOptions frictionOptions;
    auto* noHalfSpaceFrictionRows
        = static_cast<std::vector<vbd::AvbdHalfSpaceFrictionRow>*>(nullptr);
    auto* noSelfContactRows
        = static_cast<std::vector<vbd::AvbdSelfContactNormalRow>*>(nullptr);

    vbd::blockDescentMassSpringAvbdRows(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        /*fallbackSpringStiffness=*/0.0,
        /*timeStep=*/0.1,
        contacts,
        attachments,
        springRows,
        coloring,
        adjacency,
        options,
        contactOptions,
        attachmentOptions,
        springOptions,
        noHalfSpaceFrictionRows,
        nullptr,
        noSelfContactRows,
        nullptr,
        nullptr,
        enableFriction ? &frictionRows : nullptr,
        enableFriction ? &frictionOptions : nullptr);
    return positions[0].x() - stepStart[0].x();
  };

  const double withoutFriction = solve(false);
  const double withFriction = solve(true);
  EXPECT_GT(withoutFriction, 0.9);
  EXPECT_GT(withFriction, 0.05);
  EXPECT_LT(withFriction, withoutFriction);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdRowsHonorConvergenceDisplacement)
{
  std::vector<Vec3> positions = {Vec3::Zero()};
  const std::vector<double> masses = {1.0};
  const std::vector<std::uint8_t> fixed = {0u};
  const std::vector<Vec3> inertialTargets = positions;
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(positions.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(positions.size(), springs);

  std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
  std::vector<vbd::AvbdPointAttachmentRow> attachments(1);
  attachments[0].vertex = 0;
  attachments[0].target = positions[0];
  attachments[0].axis = Vec3::UnitX();
  attachments[0].state.stiffness = 100.0;
  std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;

  vbd::BlockDescentOptions options;
  options.iterations = 12;
  options.convergenceDisplacement = 1e-12;
  vbd::AvbdHalfSpaceContactOptions contactOptions;
  vbd::AvbdPointAttachmentOptions attachmentOptions;
  vbd::AvbdSpringFiniteStiffnessOptions springOptions;

  const vbd::BlockDescentStats stats = vbd::blockDescentMassSpringAvbdRows(
      positions,
      masses,
      fixed,
      inertialTargets,
      springs,
      /*fallbackSpringStiffness=*/0.0,
      /*timeStep=*/0.1,
      contacts,
      attachments,
      springRows,
      coloring,
      adjacency,
      options,
      contactOptions,
      attachmentOptions,
      springOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.vertexUpdates, 1u);
  EXPECT_NEAR(positions[0].norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(
    VbdCombinedDescent,
    AvbdDualUpdateRowsAreBitwiseDeterministicAcrossWorkerCounts)
{
  constexpr std::size_t scalarRowCount = 8193u;
  constexpr std::size_t frictionPairCount = 8193u;

  struct SolveResult
  {
    std::vector<Vec3> positions;
    std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
    std::vector<vbd::AvbdPointAttachmentRow> attachments;
    std::vector<vbd::AvbdSpringFiniteStiffnessRow> springs;
    std::vector<vbd::AvbdHalfSpaceFrictionRow> friction;
    std::vector<vbd::AvbdSelfContactNormalRow> selfContact;
    std::vector<vbd::AvbdSelfContactFrictionRow> selfContactFriction;
    vbd::BlockDescentStats stats;
  };

  const auto solve = [](std::size_t workerCount) {
    SolveResult result;
    result.positions
        = {Vec3(0.3, 0.3, 0.01),
           Vec3(0.0, 0.0, 0.0),
           Vec3(1.0, 0.0, 0.0),
           Vec3(0.0, 1.0, 0.0)};
    const std::array<Vec3, 4> stepStart{
        result.positions[0],
        result.positions[1],
        result.positions[2],
        result.positions[3]};
    const std::vector<double> masses(result.positions.size(), 1.0);
    const std::vector<std::uint8_t> fixed = {0u, 1u, 1u, 1u};
    std::vector<Vec3> inertialTargets = result.positions;
    inertialTargets[0] += Vec3(0.02, -0.01, 0.005);

    const std::vector<vbd::SpringElement> springElements{
        {0u, 1u, (result.positions[0] - result.positions[1]).norm()}};
    const auto coloring
        = vbd::colorSprings(result.positions.size(), springElements);
    const auto adjacency
        = vbd::SpringAdjacency::build(result.positions.size(), springElements);

    result.contacts.reserve(scalarRowCount);
    result.attachments.reserve(scalarRowCount);
    result.springs.reserve(scalarRowCount);
    result.selfContact.reserve(scalarRowCount);
    for (std::size_t i = 0u; i < scalarRowCount; ++i) {
      const double index = static_cast<double>(i);

      vbd::AvbdHalfSpaceContactRow contact;
      contact.vertex = 0u;
      contact.plane.normal = Vec3::UnitZ();
      contact.plane.offset = 0.02 + 1e-6 * index;
      contact.state.stiffness = 20.0 + 0.01 * index;
      contact.state.lambda = 0.001 * index;
      result.contacts.push_back(contact);

      vbd::AvbdPointAttachmentRow attachment;
      attachment.vertex = 0u;
      attachment.target = Vec3(0.32, 0.29, 0.015);
      attachment.axis
          = vbd::canonicalAvbdAttachmentAxis(static_cast<std::uint8_t>(i % 3u));
      attachment.state.stiffness = 10.0 + 0.02 * index;
      attachment.state.lambda = -0.0005 * index;
      result.attachments.push_back(attachment);

      vbd::AvbdSpringFiniteStiffnessRow spring;
      spring.spring = 0u;
      spring.state.stiffness = 5.0 + 0.01 * index;
      spring.state.lambda = 1.0 + index;
      spring.materialStiffness = 50.0 + 0.1 * index;
      result.springs.push_back(spring);

      vbd::AvbdSelfContactNormalRow selfContact;
      selfContact.nodes = {0u, 1u, 2u, 3u};
      selfContact.state.stiffness = 15.0 + 0.01 * index;
      selfContact.state.lambda = 0.00025 * index;
      selfContact.squaredActivationDistance = 4e-4;
      result.selfContact.push_back(selfContact);
    }

    result.friction.reserve(2u * frictionPairCount);
    result.selfContactFriction.reserve(2u * frictionPairCount);
    for (std::size_t pair = 0u; pair < frictionPairCount; ++pair) {
      const double index = static_cast<double>(pair);

      vbd::AvbdHalfSpaceFrictionRow first;
      first.vertex = 0u;
      first.stepStartPosition = result.positions[0] - Vec3(0.01, -0.005, 0.0);
      first.axis = Vec3::UnitX();
      first.state.stiffness = 12.0 + 0.01 * index;
      first.state.lambda = 0.001 * index;
      first.bounds = {-25.0, 25.0};
      vbd::AvbdHalfSpaceFrictionRow second = first;
      second.axis = Vec3::UnitY();
      second.state.lambda = -0.0015 * index;
      result.friction.push_back(first);
      result.friction.push_back(second);

      vbd::AvbdSelfContactFrictionRow selfFirst;
      selfFirst.nodes = {0u, 1u, 2u, 3u};
      selfFirst.stepStartPositions = stepStart;
      selfFirst.stepStartPositions[0] -= Vec3(0.005, 0.004, 0.0);
      selfFirst.axis = 0u;
      selfFirst.state.stiffness = 8.0 + 0.02 * index;
      selfFirst.state.lambda = 0.00075 * index;
      selfFirst.bounds = {-20.0, 20.0};
      vbd::AvbdSelfContactFrictionRow selfSecond = selfFirst;
      selfSecond.axis = 1u;
      selfSecond.state.lambda = -0.00025 * index;
      result.selfContactFriction.push_back(selfFirst);
      result.selfContactFriction.push_back(selfSecond);
    }
    result.contacts.back().vertex = 99u;
    result.attachments.back().vertex = 99u;
    result.springs.back().spring = 99u;
    result.selfContact.back().nodes = {99u, 100u, 101u, 102u};
    result.friction[result.friction.size() - 2u].vertex = 99u;
    result.friction.back().vertex = 99u;
    result.selfContactFriction[result.selfContactFriction.size() - 2u].nodes
        = {99u, 100u, 101u, 102u};
    result.selfContactFriction.back().nodes = {99u, 100u, 101u, 102u};

    vbd::BlockDescentOptions options;
    options.iterations = 2u;
    options.regularization = 1e-12;
    vbd::AvbdHalfSpaceContactOptions contactOptions;
    contactOptions.beta = 3.0;
    contactOptions.maxStiffness = 1000.0;
    vbd::AvbdPointAttachmentOptions attachmentOptions;
    attachmentOptions.beta = 4.0;
    attachmentOptions.maxStiffness = 1000.0;
    vbd::AvbdSpringFiniteStiffnessOptions springOptions;
    springOptions.beta = 2.0;
    springOptions.maxStiffness = 1000.0;
    vbd::AvbdHalfSpaceFrictionOptions frictionOptions;
    frictionOptions.beta = 5.0;
    frictionOptions.maxStiffness = 1000.0;
    vbd::AvbdSelfContactNormalOptions selfContactOptions;
    selfContactOptions.beta = 6.0;
    selfContactOptions.maxStiffness = 1000.0;
    vbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;
    selfContactFrictionOptions.beta = 7.0;
    selfContactFrictionOptions.maxStiffness = 1000.0;

    compute::ParallelExecutor executor(std::max<std::size_t>(1u, workerCount));
    result.stats = vbd::blockDescentMassSpringAvbdRows(
        result.positions,
        masses,
        fixed,
        inertialTargets,
        springElements,
        /*fallbackSpringStiffness=*/0.0,
        /*timeStep=*/0.02,
        result.contacts,
        result.attachments,
        result.springs,
        coloring,
        adjacency,
        options,
        contactOptions,
        attachmentOptions,
        springOptions,
        &result.friction,
        &frictionOptions,
        &result.selfContact,
        nullptr,
        &selfContactOptions,
        &result.selfContactFriction,
        &selfContactFrictionOptions,
        workerCount == 0u ? nullptr : &executor);
    return result;
  };

  const SolveResult serial = solve(0u);
  const double lastIndex = static_cast<double>(scalarRowCount - 1u);
  EXPECT_EQ(serial.contacts.back().state.lambda, 0.001 * lastIndex);
  EXPECT_EQ(serial.attachments.back().state.lambda, -0.0005 * lastIndex);
  EXPECT_EQ(serial.springs.back().state.lambda, 1.0 + lastIndex);
  EXPECT_EQ(serial.selfContact.back().state.lambda, 0.00025 * lastIndex);
  EXPECT_EQ(
      serial.friction[serial.friction.size() - 2u].state.lambda,
      0.001 * lastIndex);
  EXPECT_EQ(serial.friction.back().state.lambda, -0.0015 * lastIndex);
  EXPECT_EQ(
      serial.selfContactFriction[serial.selfContactFriction.size() - 2u]
          .state.lambda,
      0.00075 * lastIndex);
  EXPECT_EQ(
      serial.selfContactFriction.back().state.lambda, -0.00025 * lastIndex);
  const auto expectSame = [&](const SolveResult& parallel) {
    ASSERT_EQ(parallel.positions.size(), serial.positions.size());
    for (std::size_t i = 0u; i < serial.positions.size(); ++i) {
      EXPECT_TRUE(
          (parallel.positions[i].array() == serial.positions[i].array()).all())
          << "position=" << i;
    }
    EXPECT_EQ(parallel.stats.iterations, serial.stats.iterations);
    EXPECT_EQ(parallel.stats.vertexUpdates, serial.stats.vertexUpdates);
    EXPECT_EQ(
        parallel.stats.finalResidualNormSquared,
        serial.stats.finalResidualNormSquared);

    const auto expectStates = [](const auto& actual, const auto& expected) {
      ASSERT_EQ(actual.size(), expected.size());
      for (std::size_t i = 0u; i < expected.size(); ++i) {
        EXPECT_EQ(actual[i].state.lambda, expected[i].state.lambda)
            << "row=" << i;
        EXPECT_EQ(actual[i].state.stiffness, expected[i].state.stiffness)
            << "row=" << i;
      }
    };
    expectStates(parallel.contacts, serial.contacts);
    expectStates(parallel.attachments, serial.attachments);
    expectStates(parallel.springs, serial.springs);
    expectStates(parallel.friction, serial.friction);
    expectStates(parallel.selfContact, serial.selfContact);
    expectStates(parallel.selfContactFriction, serial.selfContactFriction);
  };

  expectSame(solve(2u));
  expectSame(solve(4u));
}

//==============================================================================
TEST(VbdCombinedDescent, ParallelDualUpdatePreservesNonfiniteRowFailureBehavior)
{
  constexpr std::size_t rowCount = 8193u;
  constexpr std::size_t nonfiniteRow = 4096u;
  std::vector<Vec3> positions(rowCount, Vec3(0.0, 0.01, 0.0));
  positions[nonfiniteRow].y() = std::numeric_limits<double>::quiet_NaN();

  std::vector<vbd::AvbdHalfSpaceContactRow> serial(rowCount);
  for (std::size_t i = 0u; i < rowCount; ++i) {
    serial[i].vertex = static_cast<std::uint32_t>(i);
    serial[i].state.stiffness = 10.0 + 0.001 * static_cast<double>(i);
    serial[i].state.lambda = 0.25;
  }
  auto parallel = serial;
  vbd::AvbdHalfSpaceContactOptions options;
  options.beta = 2.0;
  options.maxStiffness = 1000.0;

  const auto update = [&](auto& rows, compute::ComputeExecutor* executor) {
    vbd::forEachAvbdRowUpdateRange(
        executor, rows.size(), [&](std::size_t begin, std::size_t end) {
          for (std::size_t i = begin; i < end; ++i) {
            auto& row = rows[i];
            row.state = vbd::updateAvbdHalfSpaceContactNormalRow(
                row.state,
                positions[row.vertex],
                row.plane,
                options,
                row.previousConstraintValue,
                row.bounds);
          }
        });
  };

  update(serial, nullptr);
  compute::ParallelExecutor executor(4u);
  update(parallel, &executor);

  for (std::size_t i = 0u; i < rowCount; ++i) {
    if (i == nonfiniteRow) {
      EXPECT_TRUE(std::isnan(serial[i].state.lambda));
      EXPECT_TRUE(std::isnan(parallel[i].state.lambda));
    } else {
      EXPECT_EQ(parallel[i].state.lambda, serial[i].state.lambda)
          << "row=" << i;
    }
    EXPECT_EQ(parallel[i].state.stiffness, serial[i].state.stiffness)
        << "row=" << i;
  }
}
