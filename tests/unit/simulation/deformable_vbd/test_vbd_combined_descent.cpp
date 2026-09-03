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
// One scalar primitive row must use one generalized distance gradient. In
// particular, the triangle vertices retain their barycentric weights; stamping
// four independently normalized local normals injects net force and torque.
TEST(VbdCombinedDescent, AvbdSelfContactNormalPreservesGeneralizedGradient)
{
  const std::vector<Vec3> positions
      = {Vec3(0.2, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};

  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0u, 1u, 2u, 3u};
  row.state.stiffness = 200.0;
  row.squaredActivationDistance = 4e-4;

  const double constraintValue
      = vbd::avbdSelfContactNormalConstraintValue(row, positions);
  ASSERT_GT(constraintValue, 0.0);
  const double expectedForceMagnitude = row.state.stiffness * constraintValue;

  std::array<Vec3, 4> forces;
  std::array<Eigen::Matrix3d, 4> hessians;
  for (std::uint8_t localVertex = 0u; localVertex < 4u; ++localVertex) {
    vbd::VertexBlock block;
    const double forceMagnitude = vbd::addAvbdSelfContactNormal(
        block, positions, row, localVertex, /*alpha=*/0.0);
    EXPECT_NEAR(forceMagnitude, expectedForceMagnitude, 1e-12);
    forces[localVertex] = block.force;
    hessians[localVertex] = block.hessian;
  }

  Vec3 netForce = Vec3::Zero();
  Vec3 netTorque = Vec3::Zero();
  for (std::size_t vertex = 0u; vertex < forces.size(); ++vertex) {
    netForce += forces[vertex];
    netTorque += positions[vertex].cross(forces[vertex]);
  }
  EXPECT_NEAR(netForce.norm(), 0.0, 1e-12);
  EXPECT_NEAR(netTorque.norm(), 0.0, 1e-12);

  constexpr double gradientEpsilon = 1e-7;
  const auto numericalConstraintGradient = [&](const std::vector<Vec3>& state,
                                               std::size_t vertex) {
    Vec3 gradient = Vec3::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<Vec3> plus = state;
      std::vector<Vec3> minus = state;
      plus[vertex][axis] += gradientEpsilon;
      minus[vertex][axis] -= gradientEpsilon;
      gradient[axis] = (vbd::avbdSelfContactNormalConstraintValue(row, plus)
                        - vbd::avbdSelfContactNormalConstraintValue(row, minus))
                       / (2.0 * gradientEpsilon);
    }
    return gradient;
  };

  for (std::size_t vertex = 0u; vertex < forces.size(); ++vertex) {
    const Vec3 constraintGradient
        = numericalConstraintGradient(positions, vertex);
    const Vec3 distanceGradient = -constraintGradient;
    EXPECT_NEAR(
        (forces[vertex] - expectedForceMagnitude * distanceGradient).norm(),
        0.0,
        2e-7)
        << "vertex=" << vertex;

    constexpr double hessianEpsilon = 1e-5;
    Eigen::Matrix3d constraintHessian;
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<Vec3> plus = positions;
      std::vector<Vec3> minus = positions;
      plus[vertex][axis] += hessianEpsilon;
      minus[vertex][axis] -= hessianEpsilon;
      constraintHessian.col(axis)
          = (numericalConstraintGradient(plus, vertex)
             - numericalConstraintGradient(minus, vertex))
            / (2.0 * hessianEpsilon);
    }
    Eigen::Matrix3d expectedHessian
        = row.state.stiffness
          * (distanceGradient * distanceGradient.transpose());
    expectedHessian.diagonal() += vbd::avbdQuasiNewtonGeometricDiagonal(
        expectedForceMagnitude * constraintHessian);
    EXPECT_NEAR((hessians[vertex] - expectedHessian).norm(), 0.0, 2e-4)
        << "vertex=" << vertex;
  }
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
TEST(VbdCombinedDescent, AvbdSelfContactEdgeEdgePreservesGeneralizedGradient)
{
  const std::vector<Vec3> positions
      = {Vec3(-0.7, 0.0, 0.01),
         Vec3(0.3, 0.0, 0.01),
         Vec3(0.0, -0.2, 0.0),
         Vec3(0.0, 0.8, 0.0)};
  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0u, 1u, 2u, 3u};
  row.isEdgeEdge = true;
  row.state.stiffness = 300.0;
  row.squaredActivationDistance = 4e-4;

  const double constraintValue
      = vbd::avbdSelfContactNormalConstraintValue(row, positions);
  ASSERT_GT(constraintValue, 0.0);
  const double forceMagnitude = row.state.stiffness * constraintValue;

  std::array<Vec3, 4> forces;
  std::array<Eigen::Matrix3d, 4> hessians;
  for (std::uint8_t localVertex = 0u; localVertex < 4u; ++localVertex) {
    vbd::VertexBlock block;
    EXPECT_NEAR(
        vbd::addAvbdSelfContactNormal(
            block, positions, row, localVertex, /*alpha=*/0.0),
        forceMagnitude,
        1e-12);
    forces[localVertex] = block.force;
    hessians[localVertex] = block.hessian;
  }

  Vec3 netForce = Vec3::Zero();
  Vec3 netTorque = Vec3::Zero();
  for (std::size_t vertex = 0u; vertex < forces.size(); ++vertex) {
    netForce += forces[vertex];
    netTorque += positions[vertex].cross(forces[vertex]);
  }
  EXPECT_NEAR(netForce.norm(), 0.0, 1e-12);
  EXPECT_NEAR(netTorque.norm(), 0.0, 1e-12);

  constexpr double epsilon = 1e-7;
  const auto numericalConstraintGradient = [&](const std::vector<Vec3>& state,
                                               std::size_t vertex) {
    Vec3 gradient = Vec3::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<Vec3> plus = state;
      std::vector<Vec3> minus = state;
      plus[vertex][axis] += epsilon;
      minus[vertex][axis] -= epsilon;
      gradient[axis] = (vbd::avbdSelfContactNormalConstraintValue(row, plus)
                        - vbd::avbdSelfContactNormalConstraintValue(row, minus))
                       / (2.0 * epsilon);
    }
    return gradient;
  };
  for (std::size_t vertex = 0u; vertex < forces.size(); ++vertex) {
    const Vec3 constraintGradient
        = numericalConstraintGradient(positions, vertex);
    const Vec3 distanceGradient = -constraintGradient;
    EXPECT_NEAR(
        (forces[vertex] - forceMagnitude * distanceGradient).norm(), 0.0, 2e-7)
        << "vertex=" << vertex;

    constexpr double hessianEpsilon = 1e-5;
    Eigen::Matrix3d constraintHessian;
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<Vec3> plus = positions;
      std::vector<Vec3> minus = positions;
      plus[vertex][axis] += hessianEpsilon;
      minus[vertex][axis] -= hessianEpsilon;
      constraintHessian.col(axis)
          = (numericalConstraintGradient(plus, vertex)
             - numericalConstraintGradient(minus, vertex))
            / (2.0 * hessianEpsilon);
    }
    Eigen::Matrix3d expectedHessian
        = row.state.stiffness
          * (distanceGradient * distanceGradient.transpose());
    expectedHessian.diagonal() += vbd::avbdQuasiNewtonGeometricDiagonal(
        forceMagnitude * constraintHessian);
    EXPECT_NEAR((hessians[vertex] - expectedHessian).norm(), 0.0, 2e-4)
        << "vertex=" << vertex;
  }
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
TEST(VbdCombinedDescent, AvbdSelfContactNormalDualClearsAcrossSeparation)
{
  const std::array<std::array<Vec3, 4>, 2> activeStates{
      std::array<Vec3, 4>{
          Vec3(0.2, 0.3, 0.01),
          Vec3(0.0, 0.0, 0.0),
          Vec3(1.0, 0.0, 0.0),
          Vec3(0.0, 1.0, 0.0)},
      std::array<Vec3, 4>{
          Vec3(-0.7, 0.0, 0.01),
          Vec3(0.3, 0.0, 0.01),
          Vec3(0.0, -0.2, 0.0),
          Vec3(0.0, 0.8, 0.0)}};

  for (std::size_t primitive = 0u; primitive < activeStates.size();
       ++primitive) {
    SCOPED_TRACE(primitive == 0u ? "point-triangle" : "edge-edge");
    const bool isEdgeEdge = primitive == 1u;
    std::vector<Vec3> active(
        activeStates[primitive].begin(), activeStates[primitive].end());
    std::vector<Vec3> separated = active;
    if (isEdgeEdge) {
      separated[0].z() = 0.03;
      separated[1].z() = 0.03;
    } else {
      separated[0].z() = 0.03;
    }

    vbd::AvbdSelfContactNormalRow row;
    row.nodes = {0u, 1u, 2u, 3u};
    row.isEdgeEdge = isEdgeEdge;
    row.state.stiffness = 1000.0;
    row.state.lambda = 5.0;
    row.squaredActivationDistance = 4e-4;

    const auto separatedPrimitive
        = vbd::evaluateAvbdSelfContactPrimitive(row, separated);
    ASSERT_FALSE(separatedPrimitive.active);
    ASSERT_FALSE(separatedPrimitive.clampedToSafetyFloor);
    vbd::VertexBlock inactiveBlock;
    EXPECT_DOUBLE_EQ(
        vbd::addAvbdSelfContactNormal(
            inactiveBlock, separated, row, /*localVertex=*/0u, /*alpha=*/0.0),
        0.0);
    EXPECT_TRUE(inactiveBlock.force.isZero());
    EXPECT_TRUE(inactiveBlock.hessian.isZero());

    vbd::AvbdSelfContactNormalOptions options;
    options.alpha = 0.0;
    const vbd::AvbdScalarRowState separatedState
        = vbd::updateAvbdSelfContactNormalRow(
            row.state, separated, row, options);
    EXPECT_DOUBLE_EQ(separatedState.lambda, 0.0);

    row.state = separatedState;
    const double reentryForce
        = vbd::avbdSelfContactNormalTrialForce(row, active, /*alpha=*/0.0);
    row.state.lambda = 0.0;
    const double coldReferenceForce
        = vbd::avbdSelfContactNormalTrialForce(row, active, /*alpha=*/0.0);
    EXPECT_GT(reentryForce, 0.0);
    EXPECT_DOUBLE_EQ(reentryForce, coldReferenceForce);
  }
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactRejectsUnderFloorDifferential)
{
  namespace dc = dart::simulation::detail::deformable_contact;

  vbd::AvbdSelfContactNormalRow row;
  row.nodes = {0u, 1u, 2u, 3u};
  row.state.stiffness = 200.0;
  row.state.lambda = 3.0;
  row.squaredActivationDistance = 4e-4;

  const double safetyFloor = dc::detail::safeSquaredBarrierDistance(
      0.0, row.squaredActivationDistance);
  const double floorHeight = std::sqrt(safetyFloor);
  for (const double height : {0.0, 0.5 * floorHeight, floorHeight}) {
    SCOPED_TRACE(height);
    // Keep the in-plane barycentric coordinates exactly representable so the
    // floor oracle measures only the normal separation, not roundoff in the
    // closest-point reconstruction.
    const std::vector<Vec3> positions
        = {Vec3(0.25, 0.25, height),
           Vec3(0.0, 0.0, 0.0),
           Vec3(1.0, 0.0, 0.0),
           Vec3(0.0, 1.0, 0.0)};
    const vbd::AvbdSelfContactPrimitiveResult primitive
        = vbd::evaluateAvbdSelfContactPrimitive(row, positions);
    EXPECT_FALSE(primitive.active);
    EXPECT_FALSE(primitive.differentialValid);
    EXPECT_TRUE(primitive.clampedToSafetyFloor);
    EXPECT_TRUE(primitive.distanceGradient.isZero());
    EXPECT_TRUE(primitive.constraintHessian.isZero());

    vbd::VertexBlock block;
    EXPECT_DOUBLE_EQ(
        vbd::addAvbdSelfContactNormal(
            block, positions, row, /*localVertex=*/0u, /*alpha=*/0.0),
        0.0);
    EXPECT_TRUE(block.force.isZero());
    EXPECT_TRUE(block.hessian.isZero());

    vbd::AvbdSelfContactNormalOptions options;
    const vbd::AvbdScalarRowState updated = vbd::updateAvbdSelfContactNormalRow(
        row.state, positions, row, options);
    EXPECT_DOUBLE_EQ(updated.lambda, row.state.lambda);
    EXPECT_DOUBLE_EQ(updated.stiffness, row.state.stiffness);
  }
}

//==============================================================================
TEST(
    VbdCombinedDescent,
    AvbdSelfContactUnderFloorPreservesCoupledContinuationAcrossFullSweep)
{
  namespace dc = dart::simulation::detail::deformable_contact;
  namespace sim = dart::simulation;

  constexpr double kSquaredActivationDistance = 4e-4;
  const double safetyFloor
      = dc::detail::safeSquaredBarrierDistance(0.0, kSquaredActivationDistance);
  const double floorHeight = 0.5 * std::sqrt(safetyFloor);

  for (const bool isEdgeEdge : {false, true}) {
    SCOPED_TRACE(isEdgeEdge ? "edge-edge" : "point-triangle");
    std::vector<Vec3> positions;
    std::vector<sim::DeformableSurfaceTriangle> triangles;
    dc::ContactCandidateSet candidates;
    if (isEdgeEdge) {
      positions
          = {Vec3(-1.0, 0.0, floorHeight),
             Vec3(1.0, 0.0, floorHeight),
             Vec3(0.0, -1.0, 0.0),
             Vec3(0.0, 1.0, 0.0)};
      candidates.surfaceEdges.push_back({0u, 1u});
      candidates.surfaceEdges.push_back({2u, 3u});
      candidates.edgeEdgeCandidates.push_back(
          {/*edgeA=*/0u, /*edgeB=*/1u, 0.0});
    } else {
      positions
          = {Vec3(0.25, 0.25, floorHeight),
             Vec3(0.0, 0.0, 0.0),
             Vec3(1.0, 0.0, 0.0),
             Vec3(0.0, 1.0, 0.0)};
      triangles.push_back({1u, 2u, 3u});
      candidates.pointTriangleCandidates.push_back(
          {/*point=*/0u, /*triangle=*/0u, 0.0});
    }

    const vbd::SelfContactAdjacency selfContact
        = vbd::SelfContactAdjacency::build(
            positions.size(),
            candidates,
            triangles,
            kSquaredActivationDistance,
            /*stiffness=*/1.0);
    ASSERT_TRUE(selfContact.active());

    std::vector<vbd::AvbdSelfContactNormalRow> normalRows(1);
    normalRows[0].nodes = {0u, 1u, 2u, 3u};
    normalRows[0].isEdgeEdge = isEdgeEdge;
    normalRows[0].state = {/*stiffness=*/200.0, /*lambda=*/3.0};
    normalRows[0].previousConstraintValue = 0.0125;
    normalRows[0].squaredActivationDistance = kSquaredActivationDistance;

    std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows(2);
    for (std::uint8_t axis = 0u; axis < 2u; ++axis) {
      auto& row = frictionRows[axis];
      row.nodes = normalRows[0].nodes;
      row.stepStartPositions
          = {positions[0], positions[1], positions[2], positions[3]};
      row.isEdgeEdge = isEdgeEdge;
      row.axis = axis;
      row.state.stiffness = 80.0 + static_cast<double>(axis);
      row.state.lambda = axis == 0u ? 1.25 : -0.75;
      row.previousConstraintValue = axis == 0u ? 0.125 : -0.25;
      row.bounds = {-5.0, 5.0};
      row.accumulatedTangentialDisplacement = Vec3(0.003, -0.004, 0.0);
      row.normalRow = 0u;
      row.frictionCoefficient = 0.5;
      row.sticking = true;
    }

    const auto originalNormalState = normalRows[0].state;
    const auto originalFrictionRows = frictionRows;
    const std::vector<double> masses(positions.size(), 1.0);
    const std::vector<std::uint8_t> fixed = {0u, 1u, 1u, 1u};
    const std::vector<vbd::SpringElement> springs;
    const auto coloring = vbd::colorSprings(positions.size(), springs);
    const auto springAdjacency
        = vbd::SpringAdjacency::build(positions.size(), springs);

    const auto solveOneSweep = [&](std::vector<Vec3>& solvePositions,
                                   auto& solveNormalRows,
                                   auto& solveFrictionRows) {
      const std::vector<Vec3> inertialTargets = solvePositions;
      std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
      std::vector<vbd::AvbdPointAttachmentRow> attachments;
      std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;
      vbd::BlockDescentOptions options;
      options.iterations = 1u;
      vbd::AvbdHalfSpaceContactOptions contactOptions;
      vbd::AvbdPointAttachmentOptions attachmentOptions;
      vbd::AvbdSpringFiniteStiffnessOptions springOptions;
      vbd::AvbdSelfContactNormalOptions selfContactOptions;
      vbd::AvbdSelfContactFrictionOptions frictionOptions;
      auto* noHalfSpaceFrictionRows
          = static_cast<std::vector<vbd::AvbdHalfSpaceFrictionRow>*>(nullptr);

      vbd::blockDescentMassSpringAvbdRows(
          solvePositions,
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
          springAdjacency,
          options,
          contactOptions,
          attachmentOptions,
          springOptions,
          noHalfSpaceFrictionRows,
          nullptr,
          &solveNormalRows,
          &selfContact,
          &selfContactOptions,
          &solveFrictionRows,
          &frictionOptions);
    };

    const std::vector<Vec3> underFloorPositions = positions;
    solveOneSweep(positions, normalRows, frictionRows);
    EXPECT_EQ(normalRows[0].state.stiffness, originalNormalState.stiffness);
    EXPECT_EQ(normalRows[0].state.lambda, originalNormalState.lambda);
    for (std::size_t axis = 0u; axis < 2u; ++axis) {
      EXPECT_EQ(
          frictionRows[axis].state.stiffness,
          originalFrictionRows[axis].state.stiffness);
      EXPECT_EQ(
          frictionRows[axis].state.lambda,
          originalFrictionRows[axis].state.lambda);
      EXPECT_EQ(
          frictionRows[axis].bounds.lower,
          originalFrictionRows[axis].bounds.lower);
      EXPECT_EQ(
          frictionRows[axis].bounds.upper,
          originalFrictionRows[axis].bounds.upper);
      EXPECT_EQ(
          frictionRows[axis].sticking, originalFrictionRows[axis].sticking);
      EXPECT_TRUE(frictionRows[axis].differentialSuspended);
      EXPECT_TRUE((frictionRows[axis].accumulatedTangentialDisplacement.array()
                   == originalFrictionRows[axis]
                          .accumulatedTangentialDisplacement.array())
                      .all());
    }
    for (std::size_t i = 0u; i < positions.size(); ++i) {
      EXPECT_TRUE(
          (positions[i].array() == underFloorPositions[i].array()).all());
    }

    vbd::VertexBlock suspendedBlock;
    EXPECT_DOUBLE_EQ(
        vbd::addAvbdSelfContactNormal(
            suspendedBlock,
            positions,
            normalRows[0],
            /*localVertex=*/0u,
            /*alpha=*/0.0),
        0.0);
    EXPECT_TRUE(
        vbd::addAvbdSelfContactFrictionTangentPair(
            suspendedBlock,
            positions,
            frictionRows[0],
            frictionRows[1],
            /*localVertex=*/0u,
            vbd::AvbdSelfContactFrictionOptions{})
            .isZero());
    EXPECT_TRUE(suspendedBlock.force.isZero());
    EXPECT_TRUE(suspendedBlock.hessian.isZero());

    // Once the primitive has a valid active-band differential, the preserved
    // continuation must re-enter exactly like an otherwise identical control
    // that never carried the transient suspension marker.
    std::vector<Vec3> activePositions = underFloorPositions;
    if (isEdgeEdge) {
      activePositions[0].z() = 0.01;
      activePositions[1].z() = 0.01;
    } else {
      activePositions[0].z() = 0.01;
    }
    std::vector<Vec3> controlPositions = activePositions;
    auto controlNormalRows = normalRows;
    auto controlFrictionRows = frictionRows;
    for (auto& row : controlFrictionRows) {
      row.differentialSuspended = false;
    }
    solveOneSweep(activePositions, normalRows, frictionRows);
    solveOneSweep(controlPositions, controlNormalRows, controlFrictionRows);
    for (std::size_t i = 0u; i < activePositions.size(); ++i) {
      EXPECT_TRUE(
          (activePositions[i].array() == controlPositions[i].array()).all());
    }
    EXPECT_EQ(
        normalRows[0].state.stiffness, controlNormalRows[0].state.stiffness);
    EXPECT_EQ(normalRows[0].state.lambda, controlNormalRows[0].state.lambda);
    for (std::size_t axis = 0u; axis < 2u; ++axis) {
      EXPECT_EQ(
          frictionRows[axis].state.stiffness,
          controlFrictionRows[axis].state.stiffness);
      EXPECT_EQ(
          frictionRows[axis].state.lambda,
          controlFrictionRows[axis].state.lambda);
      EXPECT_EQ(
          frictionRows[axis].sticking, controlFrictionRows[axis].sticking);
      EXPECT_FALSE(frictionRows[axis].differentialSuspended);
    }
  }
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSeparatedHalfSpaceFrictionIsNoOpAcrossFullSweep)
{
  struct SolveResult
  {
    std::vector<Vec3> positions;
    std::vector<vbd::AvbdHalfSpaceContactRow> normalRows;
    std::vector<vbd::AvbdHalfSpaceFrictionRow> frictionRows;
  };

  const auto solve = [](bool withFriction) {
    SolveResult result;
    result.positions = {Vec3(0.0, 0.0, 0.01)};
    const std::vector<double> masses = {1.0};
    const std::vector<std::uint8_t> fixed = {0u};
    const std::vector<Vec3> inertialTargets = {Vec3(1.0, -0.5, 0.01)};
    const std::vector<vbd::SpringElement> springs;
    const auto coloring = vbd::colorSprings(result.positions.size(), springs);
    const auto adjacency
        = vbd::SpringAdjacency::build(result.positions.size(), springs);

    result.normalRows.resize(1u);
    result.normalRows[0].vertex = 0u;
    result.normalRows[0].plane.normal = Vec3::UnitZ();
    result.normalRows[0].plane.offset = 0.0;
    result.normalRows[0].state.stiffness = 100.0;

    if (withFriction) {
      vbd::AvbdHalfSpaceFrictionRow first;
      first.vertex = 0u;
      first.stepStartPosition = result.positions[0];
      first.axis = Vec3::UnitX();
      first.state = {/*stiffness=*/500.0, /*lambda=*/3.0};
      first.bounds = {-10.0, 10.0};
      first.accumulatedTangentialDisplacement = Vec3(0.25, -0.5, 0.0);
      first.normalRow = 0u;
      first.frictionCoefficient = 0.5;
      first.sticking = true;
      vbd::AvbdHalfSpaceFrictionRow second = first;
      second.axis = Vec3::UnitY();
      second.state.stiffness = 700.0;
      second.state.lambda = -4.0;
      result.frictionRows = {first, second};
    }

    std::vector<vbd::AvbdPointAttachmentRow> attachments;
    std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;
    vbd::BlockDescentOptions options;
    options.iterations = 1u;
    vbd::AvbdHalfSpaceContactOptions contactOptions;
    vbd::AvbdPointAttachmentOptions attachmentOptions;
    vbd::AvbdSpringFiniteStiffnessOptions springOptions;
    vbd::AvbdHalfSpaceFrictionOptions frictionOptions;
    vbd::blockDescentMassSpringAvbdRows(
        result.positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        /*fallbackSpringStiffness=*/0.0,
        /*timeStep=*/0.1,
        result.normalRows,
        attachments,
        springRows,
        coloring,
        adjacency,
        options,
        contactOptions,
        attachmentOptions,
        springOptions,
        withFriction ? &result.frictionRows : nullptr,
        withFriction ? &frictionOptions : nullptr);
    return result;
  };

  const SolveResult control = solve(false);
  const SolveResult separated = solve(true);
  ASSERT_EQ(separated.positions.size(), control.positions.size());
  EXPECT_TRUE(
      (separated.positions[0].array() == control.positions[0].array()).all());
  ASSERT_EQ(separated.frictionRows.size(), 2u);
  EXPECT_DOUBLE_EQ(separated.frictionRows[0].state.stiffness, 500.0);
  EXPECT_DOUBLE_EQ(separated.frictionRows[1].state.stiffness, 700.0);
  for (const auto& row : separated.frictionRows) {
    EXPECT_DOUBLE_EQ(row.state.lambda, 0.0);
    EXPECT_DOUBLE_EQ(row.bounds.lower, 0.0);
    EXPECT_DOUBLE_EQ(row.bounds.upper, 0.0);
    EXPECT_FALSE(row.sticking);
    EXPECT_TRUE(row.accumulatedTangentialDisplacement.isZero(0.0));
  }
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSeparatedSelfContactFrictionIsNoOpAcrossFullSweep)
{
  namespace dc = dart::simulation::detail::deformable_contact;
  constexpr double kSquaredActivationDistance = 4e-4;
  for (const bool isEdgeEdge : {false, true}) {
    SCOPED_TRACE(isEdgeEdge ? "edge-edge" : "point-triangle");

    std::vector<Vec3> initialPositions;
    std::vector<dart::simulation::DeformableSurfaceTriangle> triangles;
    dc::ContactCandidateSet candidates;
    if (isEdgeEdge) {
      initialPositions
          = {Vec3(-1.0, 0.0, 0.03),
             Vec3(1.0, 0.0, 0.03),
             Vec3(0.0, -1.0, 0.0),
             Vec3(0.0, 1.0, 0.0)};
      candidates.surfaceEdges.push_back({0u, 1u});
      candidates.surfaceEdges.push_back({2u, 3u});
      candidates.edgeEdgeCandidates.push_back(
          {/*edgeA=*/0u, /*edgeB=*/1u, 0.0});
    } else {
      initialPositions
          = {Vec3(0.25, 0.25, 0.03),
             Vec3(0.0, 0.0, 0.0),
             Vec3(1.0, 0.0, 0.0),
             Vec3(0.0, 1.0, 0.0)};
      triangles.push_back({1u, 2u, 3u});
      candidates.pointTriangleCandidates.push_back(
          {/*point=*/0u, /*triangle=*/0u, 0.0});
    }
    const vbd::SelfContactAdjacency selfContact
        = vbd::SelfContactAdjacency::build(
            initialPositions.size(),
            candidates,
            triangles,
            kSquaredActivationDistance,
            /*stiffness=*/1.0);
    ASSERT_TRUE(selfContact.active());

    struct SolveResult
    {
      std::vector<Vec3> positions;
      std::vector<vbd::AvbdSelfContactNormalRow> normalRows;
      std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows;
    };
    const auto solve = [&](bool withFriction) {
      SolveResult result;
      result.positions = initialPositions;
      result.normalRows.resize(1u);
      result.normalRows[0].nodes = {0u, 1u, 2u, 3u};
      result.normalRows[0].isEdgeEdge = isEdgeEdge;
      result.normalRows[0].state = {/*stiffness=*/100.0, /*lambda=*/1.0};
      result.normalRows[0].squaredActivationDistance
          = kSquaredActivationDistance;

      if (withFriction) {
        vbd::AvbdSelfContactFrictionRow first;
        first.nodes = result.normalRows[0].nodes;
        first.stepStartPositions
            = {initialPositions[0],
               initialPositions[1],
               initialPositions[2],
               initialPositions[3]};
        first.isEdgeEdge = isEdgeEdge;
        first.axis = 0u;
        first.state = {/*stiffness=*/500.0, /*lambda=*/3.0};
        first.bounds = {-10.0, 10.0};
        first.accumulatedTangentialDisplacement = Vec3(0.25, -0.5, 0.0);
        first.normalRow = 0u;
        first.frictionCoefficient = 0.5;
        first.sticking = true;
        vbd::AvbdSelfContactFrictionRow second = first;
        second.axis = 1u;
        second.state.stiffness = 700.0;
        second.state.lambda = -4.0;
        result.frictionRows = {first, second};
      }

      const std::vector<double> masses(result.positions.size(), 1.0);
      const std::vector<std::uint8_t> fixed = {0u, 1u, 1u, 1u};
      std::vector<Vec3> inertialTargets = result.positions;
      inertialTargets[0] += Vec3(1.0, -0.5, 0.0);
      const std::vector<vbd::SpringElement> springs;
      const auto coloring = vbd::colorSprings(result.positions.size(), springs);
      const auto adjacency
          = vbd::SpringAdjacency::build(result.positions.size(), springs);
      std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
      std::vector<vbd::AvbdPointAttachmentRow> attachments;
      std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;
      vbd::BlockDescentOptions options;
      options.iterations = 1u;
      vbd::AvbdHalfSpaceContactOptions contactOptions;
      vbd::AvbdPointAttachmentOptions attachmentOptions;
      vbd::AvbdSpringFiniteStiffnessOptions springOptions;
      vbd::AvbdSelfContactNormalOptions selfContactOptions;
      vbd::AvbdSelfContactFrictionOptions frictionOptions;
      auto* noHalfSpaceFrictionRows
          = static_cast<std::vector<vbd::AvbdHalfSpaceFrictionRow>*>(nullptr);
      vbd::blockDescentMassSpringAvbdRows(
          result.positions,
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
          &result.normalRows,
          &selfContact,
          &selfContactOptions,
          withFriction ? &result.frictionRows : nullptr,
          withFriction ? &frictionOptions : nullptr);
      return result;
    };

    const SolveResult control = solve(false);
    const SolveResult separated = solve(true);
    ASSERT_EQ(separated.positions.size(), control.positions.size());
    for (std::size_t i = 0u; i < separated.positions.size(); ++i) {
      EXPECT_TRUE(
          (separated.positions[i].array() == control.positions[i].array())
              .all());
    }
    ASSERT_EQ(separated.frictionRows.size(), 2u);
    EXPECT_DOUBLE_EQ(separated.frictionRows[0].state.stiffness, 500.0);
    EXPECT_DOUBLE_EQ(separated.frictionRows[1].state.stiffness, 700.0);
    // The end-of-solve friction state is whatever the last dual update leaves;
    // the residual pass no longer writes to solver rows, so it can no longer
    // re-clear them from the exactly-zero trial force of a separated
    // primitive. A separated normal row settles at half an eps of dual, which
    // is the width of the tangent cone seen here: zero to machine precision,
    // and still an exact no-op for the motion asserted above.
    constexpr double kDualNoise = 4.0 * std::numeric_limits<double>::epsilon();
    for (const auto& row : separated.frictionRows) {
      EXPECT_NEAR(row.state.lambda, 0.0, kDualNoise);
      EXPECT_LE(row.bounds.lower, 0.0);
      EXPECT_GE(row.bounds.upper, 0.0);
      EXPECT_NEAR(row.bounds.lower, 0.0, kDualNoise);
      EXPECT_NEAR(row.bounds.upper, 0.0, kDualNoise);
      EXPECT_FALSE(row.sticking);
      EXPECT_FALSE(row.differentialSuspended);
      EXPECT_TRUE(row.accumulatedTangentialDisplacement.isZero(0.0));
    }
  }
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
TEST(VbdCombinedDescent, AvbdSelfContactFrictionKeepsPersistentAnchor)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.2, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<Vec3> positions(stepStart.begin(), stepStart.end());

  vbd::AvbdSelfContactFrictionRow row;
  row.nodes = {0u, 1u, 2u, 3u};
  row.stepStartPositions = stepStart;
  row.axis = 0u;
  const auto basis = vbd::avbdSelfContactFrictionBasis(row);
  row.accumulatedTangentialDisplacement = 0.2 * basis.col(0);

  EXPECT_NEAR(
      vbd::avbdSelfContactFrictionConstraintValue(row, positions), -0.2, 1e-12);
  EXPECT_NEAR(
      (vbd::avbdSelfContactFrictionTotalTangentialDisplacement(row, positions)
       - row.accumulatedTangentialDisplacement)
          .norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionUsesCurrentNormalForce)
{
  const std::array<std::array<Vec3, 4>, 2> stepStarts{
      std::array<Vec3, 4>{
          Vec3(0.2, 0.3, 0.01),
          Vec3(0.0, 0.0, 0.0),
          Vec3(1.0, 0.0, 0.0),
          Vec3(0.0, 1.0, 0.0)},
      std::array<Vec3, 4>{
          Vec3(-0.7, 0.0, 0.01),
          Vec3(0.3, 0.0, 0.01),
          Vec3(0.0, -0.2, 0.0),
          Vec3(0.0, 0.8, 0.0)}};

  for (std::size_t primitive = 0u; primitive < stepStarts.size(); ++primitive) {
    SCOPED_TRACE(primitive == 0u ? "point-triangle" : "edge-edge");
    const bool isEdgeEdge = primitive == 1u;
    const auto& stepStart = stepStarts[primitive];

    vbd::AvbdSelfContactNormalRow normal;
    normal.nodes = {0u, 1u, 2u, 3u};
    normal.isEdgeEdge = isEdgeEdge;
    normal.state.stiffness = 1000.0;
    normal.squaredActivationDistance = 4e-4;
    std::vector<vbd::AvbdSelfContactNormalRow> normalRows = {normal};

    vbd::AvbdSelfContactFrictionRow tangentX;
    tangentX.nodes = normal.nodes;
    tangentX.stepStartPositions = stepStart;
    tangentX.isEdgeEdge = isEdgeEdge;
    tangentX.axis = 0u;
    tangentX.state.stiffness = 100.0;
    tangentX.bounds = vbd::avbdFrictionTangentBounds(0.0);
    tangentX.normalRow = 0u;
    tangentX.frictionCoefficient = 0.5;
    vbd::AvbdSelfContactFrictionRow tangentY = tangentX;
    tangentY.axis = 1u;
    std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows
        = {tangentX, tangentY};

    std::vector<Vec3> positions(stepStart.begin(), stepStart.end());
    positions[0] += 0.03 * vbd::avbdSelfContactFrictionBasis(tangentX).col(0);
    const double normalTrial = vbd::avbdSelfContactNormalTrialForce(
        normalRows[0], positions, /*alpha=*/0.0);
    ASSERT_GT(normalTrial, 0.0);
    vbd::refreshAvbdSelfContactFrictionCones(
        positions,
        normalRows,
        frictionRows,
        /*alpha=*/0.0,
        /*useTrialNormalForce=*/true);

    vbd::AvbdSelfContactFrictionOptions options;
    options.alpha = 0.0;
    vbd::updateAvbdSelfContactFrictionTangentPair(
        frictionRows[0], frictionRows[1], positions, options);
    const double tangentForce = std::hypot(
        frictionRows[0].state.lambda, frictionRows[1].state.lambda);
    EXPECT_GT(tangentForce, 0.0);
    EXPECT_LE(
        tangentForce,
        frictionRows[0].frictionCoefficient * normalTrial
            + 64.0 * std::numeric_limits<double>::epsilon());
  }
}

//==============================================================================
TEST(
    VbdCombinedDescent,
    AvbdSelfContactFrictionRefreshesAfterEarlierStencilBlockMoves)
{
  const std::array<std::array<Vec3, 4>, 2> stepStarts{
      std::array<Vec3, 4>{
          Vec3(0.2, 0.3, 0.01),
          Vec3(0.0, 0.0, 0.0),
          Vec3(1.0, 0.0, 0.0),
          Vec3(0.0, 1.0, 0.0)},
      std::array<Vec3, 4>{
          Vec3(-0.7, 0.0, 0.01),
          Vec3(0.3, 0.0, 0.01),
          Vec3(0.0, -0.2, 0.0),
          Vec3(0.0, 0.8, 0.0)}};

  for (std::size_t primitive = 0u; primitive < stepStarts.size(); ++primitive) {
    SCOPED_TRACE(primitive == 0u ? "point-triangle" : "edge-edge");
    const bool isEdgeEdge = primitive == 1u;
    const auto& stepStart = stepStarts[primitive];

    vbd::AvbdSelfContactNormalRow normal;
    normal.nodes = {0u, 1u, 2u, 3u};
    normal.isEdgeEdge = isEdgeEdge;
    normal.state.stiffness = 1000.0;
    normal.squaredActivationDistance = 4e-4;
    std::vector<vbd::AvbdSelfContactNormalRow> normalRows = {normal};

    vbd::AvbdSelfContactFrictionRow tangentX;
    tangentX.nodes = normal.nodes;
    tangentX.stepStartPositions = stepStart;
    tangentX.isEdgeEdge = isEdgeEdge;
    tangentX.axis = 0u;
    tangentX.state.stiffness = 1.0e6;
    tangentX.normalRow = 0u;
    tangentX.frictionCoefficient = 0.5;
    vbd::AvbdSelfContactFrictionRow tangentY = tangentX;
    tangentY.axis = 1u;

    std::vector<Vec3> positions(stepStart.begin(), stepStart.end());
    positions[0] += 0.03 * vbd::avbdSelfContactFrictionBasis(tangentX).col(0);
    vbd::refreshAvbdSelfContactFrictionCone(
        positions,
        normalRows,
        tangentX,
        tangentY,
        /*alpha=*/0.0,
        /*useTrialNormalForce=*/true);
    const double staleLimit
        = vbd::avbdSelfContactFrictionPairForceLimit(tangentX, tangentY);

    // Model an earlier GS block moving node 0 away from the opposing
    // primitive. The next incident block (node 1) must see the changed normal
    // trial, not the iteration-start cone above.
    positions[0].z() += 0.005;
    const double liveNormalTrial = vbd::avbdSelfContactNormalTrialForce(
        normalRows[0], positions, /*alpha=*/0.0);
    ASSERT_GT(liveNormalTrial, 0.0);
    const double expectedLiveLimit
        = tangentX.frictionCoefficient * liveNormalTrial;
    ASSERT_LT(expectedLiveLimit, staleLimit);

    vbd::refreshAvbdSelfContactFrictionCone(
        positions,
        normalRows,
        tangentX,
        tangentY,
        /*alpha=*/0.0,
        /*useTrialNormalForce=*/true);
    EXPECT_NEAR(
        vbd::avbdSelfContactFrictionPairForceLimit(tangentX, tangentY),
        expectedLiveLimit,
        1e-12);

    vbd::AvbdSelfContactFrictionOptions options;
    options.alpha = 0.0;
    vbd::VertexBlock block;
    const Eigen::Vector2d force = vbd::addAvbdSelfContactFrictionTangentPair(
        block,
        positions,
        tangentX,
        tangentY,
        /*localVertex=*/1u,
        options);
    EXPECT_NEAR(force.norm(), expectedLiveLimit, 1e-12);
    EXPECT_TRUE(block.force.allFinite());
    EXPECT_TRUE(block.hessian.allFinite());
  }
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionPreservesStencilWeights)
{
  const std::array<std::array<Vec3, 4>, 2> stepStarts{
      std::array<Vec3, 4>{
          Vec3(0.2, 0.3, 0.01),
          Vec3(0.0, 0.0, 0.0),
          Vec3(1.0, 0.0, 0.0),
          Vec3(0.0, 1.0, 0.0)},
      std::array<Vec3, 4>{
          Vec3(-0.7, 0.0, 0.01),
          Vec3(0.3, 0.0, 0.01),
          Vec3(0.0, -0.2, 0.0),
          Vec3(0.0, 0.8, 0.0)}};

  for (std::size_t primitive = 0u; primitive < stepStarts.size(); ++primitive) {
    for (std::uint8_t axis = 0u; axis < 2u; ++axis) {
      vbd::AvbdSelfContactFrictionRow row;
      row.nodes = {0u, 1u, 2u, 3u};
      row.stepStartPositions = stepStarts[primitive];
      row.isEdgeEdge = primitive == 1u;
      row.axis = axis;
      row.state.stiffness = 100.0;
      row.bounds = {-1000.0, 1000.0};

      const auto generalizedDirection
          = vbd::avbdSelfContactFrictionGeneralizedDirection(row);
      std::vector<Vec3> positions(
          row.stepStartPositions.begin(), row.stepStartPositions.end());
      for (std::size_t vertex = 0u; vertex < positions.size(); ++vertex) {
        positions[vertex]
            += 0.01
               * generalizedDirection.segment<3>(3 * static_cast<int>(vertex));
      }

      const double constraintValue
          = vbd::avbdSelfContactFrictionConstraintValue(row, positions);
      ASSERT_NE(constraintValue, 0.0);
      const double forceMagnitude = row.state.stiffness * constraintValue;

      std::array<Vec3, 4> forces;
      Vec3 netForce = Vec3::Zero();
      for (std::uint8_t localVertex = 0u; localVertex < 4u; ++localVertex) {
        vbd::VertexBlock block;
        EXPECT_NEAR(
            vbd::addAvbdSelfContactFrictionTangent(
                block, positions, row, localVertex, /*alpha=*/0.0),
            forceMagnitude,
            1e-12);
        forces[localVertex] = block.force;
        netForce += block.force;
      }
      EXPECT_NEAR(netForce.norm(), 0.0, 1e-12)
          << "primitive=" << primitive << " axis=" << static_cast<int>(axis);

      constexpr double epsilon = 1e-7;
      for (std::size_t vertex = 0u; vertex < forces.size(); ++vertex) {
        Vec3 constraintGradient = Vec3::Zero();
        for (int coordinate = 0; coordinate < 3; ++coordinate) {
          std::vector<Vec3> plus = positions;
          std::vector<Vec3> minus = positions;
          plus[vertex][coordinate] += epsilon;
          minus[vertex][coordinate] -= epsilon;
          constraintGradient[coordinate]
              = (vbd::avbdSelfContactFrictionConstraintValue(row, plus)
                 - vbd::avbdSelfContactFrictionConstraintValue(row, minus))
                / (2.0 * epsilon);
        }
        EXPECT_NEAR(
            (forces[vertex] + forceMagnitude * constraintGradient).norm(),
            0.0,
            1e-8)
            << "primitive=" << primitive << " axis=" << static_cast<int>(axis)
            << " vertex=" << vertex;
      }
    }
  }
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
TEST(
    VbdCombinedDescent,
    AvbdSelfContactFrictionPairProjectsAugmentedTrialFromBoundary)
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
  rowY.state.stiffness = 20.0;
  rowY.state.lambda = 0.0;

  vbd::AvbdSelfContactFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_TRUE(vbd::avbdSelfContactFrictionPreviousDualInsideCone(rowX, rowY));
  const Eigen::Vector2d constraintValues
      = vbd::avbdSelfContactFrictionConstraintValues(
          rowX, rowY, positions, options.alpha);
  const Eigen::Vector2d trial(
      rowX.state.stiffness * constraintValues.x() + rowX.state.lambda,
      rowY.state.stiffness * constraintValues.y() + rowY.state.lambda);
  ASSERT_GT(trial.norm(), 5.0);

  bool clamped = false;
  const Eigen::Vector2d force = vbd::avbdSelfContactFrictionTangentPairForce(
      rowX, rowY, positions, options, &clamped);
  const Eigen::Vector2d expected = 5.0 * trial.normalized();
  EXPECT_TRUE(clamped);
  EXPECT_TRUE(force.isApprox(expected, 1e-12));
  EXPECT_NE(force.x(), 0.0);

  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_NEAR(rowX.state.lambda, expected.x(), 1e-12);
  EXPECT_NEAR(rowY.state.lambda, expected.y(), 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 20.0);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionBoundaryCanReturnAndRestick)
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
  rowX.state.stiffness = 10.0;
  rowX.state.lambda = 5.0;
  rowX.bounds = {-5.0, 5.0};

  vbd::AvbdSelfContactFrictionRow rowY = rowX;
  rowY.axis = 1;
  rowY.state.stiffness = 20.0;
  rowY.state.lambda = 0.0;

  vbd::AvbdSelfContactFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  const Eigen::Vector3d tangent
      = vbd::avbdSelfContactFrictionBasis(rowX).col(0);
  std::vector<Vec3> positions(stepStart.begin(), stepStart.end());
  positions[0] += 5e-7 * tangent;

  ASSERT_TRUE(vbd::avbdSelfContactFrictionPreviousDualInsideCone(rowX, rowY));
  bool clamped = true;
  const std::vector<Vec3> boundaryPositions(stepStart.begin(), stepStart.end());
  const Eigen::Vector2d boundaryForce
      = vbd::avbdSelfContactFrictionTangentPairForce(
          rowX, rowY, boundaryPositions, options, &clamped);
  EXPECT_FALSE(clamped);
  EXPECT_NEAR(boundaryForce.norm(), 5.0, 1e-12);

  const Eigen::Vector2d returnedForce
      = vbd::avbdSelfContactFrictionTangentPairForce(
          rowX, rowY, positions, options, &clamped);
  EXPECT_FALSE(clamped);
  EXPECT_LT(returnedForce.norm(), 5.0);

  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_TRUE(rowX.sticking);
  EXPECT_TRUE(rowY.sticking);
  EXPECT_GT(rowX.state.stiffness, 10.0);

  rowX.state.stiffness = 10.0;
  rowX.state.lambda = 5.0;
  rowX.sticking = true;
  rowY.state.stiffness = 20.0;
  rowY.state.lambda = 0.0;
  rowY.sticking = true;
  positions.assign(stepStart.begin(), stepStart.end());
  positions[0] += 0.01 * tangent;
  const Eigen::Vector2d residualForce
      = vbd::avbdSelfContactFrictionTangentPairForce(
          rowX, rowY, positions, options, &clamped);
  EXPECT_FALSE(clamped);
  EXPECT_LT(residualForce.norm(), 5.0);

  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_FALSE(rowX.sticking);
  EXPECT_FALSE(rowY.sticking);
  EXPECT_GT(rowX.state.stiffness, 10.0);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionZeroConeIsInactive)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  vbd::AvbdSelfContactFrictionRow rowX;
  rowX.nodes = {0u, 1u, 2u, 3u};
  rowX.stepStartPositions = stepStart;
  rowX.axis = 0u;
  rowX.state.stiffness = 10.0;
  rowX.bounds = vbd::avbdFrictionTangentBounds(0.0);
  vbd::AvbdSelfContactFrictionRow rowY = rowX;
  rowY.axis = 1u;
  vbd::AvbdSelfContactFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 2.0;
  const Vec3 tangent = vbd::avbdSelfContactFrictionBasis(rowX).col(0);

  const auto setCancellingDual = [&](const auto& positions) {
    const Eigen::Vector2d residual
        = vbd::avbdSelfContactFrictionConstraintValues(
            rowX, rowY, positions, options.alpha);
    rowX.state.lambda = -rowX.state.stiffness * residual.x();
    rowY.state.lambda = -rowY.state.stiffness * residual.y();
    return residual;
  };

  std::vector<Vec3> positions(stepStart.begin(), stepStart.end());
  positions[0] += 0.1 * tangent;
  const Eigen::Vector2d slidingResidual = setCancellingDual(positions);
  ASSERT_GT(slidingResidual.norm(), options.staticFrictionTolerance);
  bool clamped = true;
  const Eigen::Vector2d force = vbd::avbdSelfContactFrictionTangentPairForce(
      rowX, rowY, positions, options, &clamped);
  EXPECT_FALSE(clamped);
  EXPECT_TRUE(force.isZero(0.0));
  vbd::VertexBlock slidingBlock;
  EXPECT_TRUE(
      vbd::addAvbdSelfContactFrictionTangentPair(
          slidingBlock,
          positions,
          rowX,
          rowY,
          /*localVertex=*/0u,
          options)
          .isZero(0.0));
  EXPECT_TRUE(slidingBlock.force.isZero(0.0));
  EXPECT_TRUE(slidingBlock.hessian.isZero(0.0));
  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_FALSE(rowX.sticking);
  EXPECT_FALSE(rowY.sticking);
  EXPECT_DOUBLE_EQ(rowX.state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(rowY.state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);

  rowX.state.stiffness = 10.0;
  rowY.state.stiffness = 10.0;
  positions.assign(stepStart.begin(), stepStart.end());
  positions[0] += 1e-6 * tangent;
  const Eigen::Vector2d stickingResidual = setCancellingDual(positions);
  ASSERT_LT(stickingResidual.norm(), options.staticFrictionTolerance);
  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_FALSE(rowX.sticking);
  EXPECT_FALSE(rowY.sticking);
  EXPECT_DOUBLE_EQ(rowX.state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(rowY.state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdFrictionRefreshRejectsNonfiniteNormalForce)
{
  std::vector<Vec3> halfSpacePositions = {Vec3(0.0, 0.0, 0.01)};
  std::vector<vbd::AvbdHalfSpaceContactRow> halfSpaceNormalRows(1u);
  halfSpaceNormalRows[0].vertex = 0u;
  halfSpaceNormalRows[0].state.lambda = std::numeric_limits<double>::infinity();
  std::vector<vbd::AvbdHalfSpaceFrictionRow> halfSpaceFrictionRows(2u);
  for (std::size_t axis = 0u; axis < 2u; ++axis) {
    auto& row = halfSpaceFrictionRows[axis];
    row.vertex = 0u;
    row.axis = axis == 0u ? Vec3::UnitX() : Vec3::UnitY();
    row.state = {/*stiffness=*/100.0, /*lambda=*/axis == 0u ? 3.0 : -4.0};
    row.bounds = {-5.0, 5.0};
    row.accumulatedTangentialDisplacement = Vec3(0.25, -0.5, 0.0);
    row.normalRow = 0u;
    row.frictionCoefficient = 0.5;
    row.sticking = true;
  }
  vbd::refreshAvbdHalfSpaceFrictionCones(
      halfSpacePositions,
      halfSpaceNormalRows,
      halfSpaceFrictionRows,
      /*alpha=*/0.0,
      /*useTrialNormalForce=*/false);
  for (const auto& row : halfSpaceFrictionRows) {
    EXPECT_DOUBLE_EQ(row.bounds.lower, 0.0);
    EXPECT_DOUBLE_EQ(row.bounds.upper, 0.0);
    EXPECT_DOUBLE_EQ(row.state.lambda, 0.0);
    EXPECT_FALSE(row.sticking);
    EXPECT_TRUE(row.accumulatedTangentialDisplacement.isZero(0.0));
  }

  const std::array<Vec3, 4> stepStart
      = {Vec3(0.25, 0.25, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<Vec3> selfContactPositions(
      stepStart.begin(), stepStart.end());
  std::vector<vbd::AvbdSelfContactNormalRow> selfContactNormalRows(1u);
  selfContactNormalRows[0].nodes = {0u, 1u, 2u, 3u};
  selfContactNormalRows[0].state.lambda
      = std::numeric_limits<double>::infinity();
  selfContactNormalRows[0].squaredActivationDistance = 4e-4;
  std::vector<vbd::AvbdSelfContactFrictionRow> selfContactFrictionRows(2u);
  for (std::uint8_t axis = 0u; axis < 2u; ++axis) {
    auto& row = selfContactFrictionRows[axis];
    row.nodes = selfContactNormalRows[0].nodes;
    row.stepStartPositions = stepStart;
    row.axis = axis;
    row.state = {/*stiffness=*/100.0, /*lambda=*/axis == 0u ? 3.0 : -4.0};
    row.bounds = {-5.0, 5.0};
    row.accumulatedTangentialDisplacement = Vec3(0.25, -0.5, 0.0);
    row.normalRow = 0u;
    row.frictionCoefficient = 0.5;
    row.sticking = true;
  }
  vbd::refreshAvbdSelfContactFrictionCones(
      selfContactPositions,
      selfContactNormalRows,
      selfContactFrictionRows,
      /*alpha=*/0.0,
      /*useTrialNormalForce=*/false);
  for (const auto& row : selfContactFrictionRows) {
    EXPECT_DOUBLE_EQ(row.bounds.lower, 0.0);
    EXPECT_DOUBLE_EQ(row.bounds.upper, 0.0);
    EXPECT_DOUBLE_EQ(row.state.lambda, 0.0);
    EXPECT_FALSE(row.sticking);
    EXPECT_FALSE(row.differentialSuspended);
    EXPECT_TRUE(row.accumulatedTangentialDisplacement.isZero(0.0));
  }
}

//==============================================================================
TEST(VbdCombinedDescent, AvbdSelfContactFrictionRejectsNonfiniteTrial)
{
  const std::array<Vec3, 4> stepStart
      = {Vec3(0.3, 0.3, 0.01),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0)};
  const std::vector<Vec3> positions(stepStart.begin(), stepStart.end());
  vbd::AvbdSelfContactFrictionRow rowX;
  rowX.nodes = {0u, 1u, 2u, 3u};
  rowX.stepStartPositions = stepStart;
  rowX.axis = 0u;
  rowX.state = {/*stiffness=*/10.0,
                /*lambda=*/std::numeric_limits<double>::infinity()};
  rowX.bounds = {-5.0, 5.0};
  rowX.accumulatedTangentialDisplacement = Vec3(0.25, 0.5, 0.0);
  rowX.sticking = true;
  vbd::AvbdSelfContactFrictionRow rowY = rowX;
  rowY.axis = 1u;
  rowY.state.lambda = 0.0;
  const vbd::AvbdSelfContactFrictionOptions options;

  bool valid = true;
  EXPECT_TRUE(
      vbd::avbdSelfContactFrictionTangentPairForce(
          rowX, rowY, positions, options, nullptr, &valid)
          .isZero(0.0));
  EXPECT_FALSE(valid);
  vbd::VertexBlock block;
  EXPECT_TRUE(
      vbd::addAvbdSelfContactFrictionTangentPair(
          block, positions, rowX, rowY, /*localVertex=*/0u, options)
          .isZero(0.0));
  EXPECT_TRUE(block.force.isZero(0.0));
  EXPECT_TRUE(block.hessian.isZero(0.0));
  vbd::updateAvbdSelfContactFrictionTangentPair(rowX, rowY, positions, options);
  EXPECT_DOUBLE_EQ(rowX.state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(rowY.state.lambda, 0.0);
  EXPECT_FALSE(rowX.sticking);
  EXPECT_FALSE(rowY.sticking);
  EXPECT_TRUE(rowX.accumulatedTangentialDisplacement.isZero(0.0));
  EXPECT_TRUE(rowY.accumulatedTangentialDisplacement.isZero(0.0));
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

//==============================================================================
namespace {

struct SelfContactFrictionPairFixture
{
  std::array<Vec3, 4> stepStart{
      Vec3(0.2, 0.3, 0.5),
      Vec3(0.0, 0.0, 0.0),
      Vec3(1.0, 0.0, 0.0),
      Vec3(0.0, 1.0, 0.0)};
  std::vector<vbd::AvbdSelfContactNormalRow> normalRows;
  std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows;

  SelfContactFrictionPairFixture()
  {
    vbd::AvbdSelfContactNormalRow normal;
    normal.nodes = {0u, 1u, 2u, 3u};
    normal.isEdgeEdge = false;
    normal.state.stiffness = 1000.0;
    normal.squaredActivationDistance = 4e-4;
    normalRows = {normal};

    vbd::AvbdSelfContactFrictionRow tangentX;
    tangentX.nodes = normal.nodes;
    tangentX.stepStartPositions = stepStart;
    tangentX.isEdgeEdge = false;
    tangentX.axis = 0u;
    tangentX.state = {/*stiffness=*/500.0, /*lambda=*/0.75};
    tangentX.bounds = vbd::avbdFrictionTangentBounds(5.0);
    tangentX.accumulatedTangentialDisplacement = Vec3(0.25, -0.5, 0.125);
    tangentX.normalRow = 0u;
    tangentX.frictionCoefficient = 0.5;
    tangentX.sticking = true;
    vbd::AvbdSelfContactFrictionRow tangentY = tangentX;
    tangentY.axis = 1u;
    tangentY.state = {/*stiffness=*/700.0, /*lambda=*/-0.4};
    frictionRows = {tangentX, tangentY};
  }
};

void expectSelfContactFrictionRowUnchanged(
    const vbd::AvbdSelfContactFrictionRow& actual,
    const vbd::AvbdSelfContactFrictionRow& expected)
{
  EXPECT_EQ(actual.state.lambda, expected.state.lambda);
  EXPECT_EQ(actual.state.stiffness, expected.state.stiffness);
  EXPECT_EQ(actual.bounds.lower, expected.bounds.lower);
  EXPECT_EQ(actual.bounds.upper, expected.bounds.upper);
  EXPECT_EQ(actual.sticking, expected.sticking);
  EXPECT_EQ(actual.differentialSuspended, expected.differentialSuspended);
  EXPECT_EQ(actual.previousConstraintValue, expected.previousConstraintValue);
  EXPECT_TRUE((actual.accumulatedTangentialDisplacement.array()
               == expected.accumulatedTangentialDisplacement.array())
                  .all());
}

} // namespace

//==============================================================================
// Residual evaluation must not write to persisted rows. The final residual pass
// reuses the same per-vertex assembly as the primal sweep, so a cone refresh
// hidden inside that assembly silently rewrote the friction duals, bounds and
// sticking anchors of a body that ran zero sweeps.
TEST(
    VbdCombinedDescent,
    AvbdSelfContactFrictionResidualEvaluationLeavesRowsUnchanged)
{
  SelfContactFrictionPairFixture fixture;
  const std::vector<vbd::AvbdSelfContactFrictionRow> expected
      = fixture.frictionRows;

  std::vector<Vec3> positions(
      fixture.stepStart.begin(), fixture.stepStart.end());
  const std::vector<double> masses(positions.size(), 1.0);
  const std::vector<std::uint8_t> fixed(positions.size(), 0u);
  const std::vector<Vec3> inertialTargets = positions;
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(positions.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(positions.size(), springs);

  std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
  std::vector<vbd::AvbdPointAttachmentRow> attachments;
  std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;

  vbd::BlockDescentOptions options;
  options.iterations = 0u;
  vbd::AvbdHalfSpaceContactOptions contactOptions;
  vbd::AvbdPointAttachmentOptions attachmentOptions;
  vbd::AvbdSpringFiniteStiffnessOptions springOptions;
  vbd::AvbdSelfContactNormalOptions selfContactOptions;
  vbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;

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
      static_cast<std::vector<vbd::AvbdHalfSpaceFrictionRow>*>(nullptr),
      nullptr,
      &fixture.normalRows,
      nullptr,
      &selfContactOptions,
      &fixture.frictionRows,
      &selfContactFrictionOptions);

  ASSERT_EQ(fixture.frictionRows.size(), expected.size());
  for (std::size_t i = 0u; i < expected.size(); ++i) {
    SCOPED_TRACE(i);
    expectSelfContactFrictionRowUnchanged(fixture.frictionRows[i], expected[i]);
  }
}

//==============================================================================
TEST(
    VbdCombinedDescent,
    AvbdTetMeshSelfContactFrictionResidualEvaluationLeavesRowsUnchanged)
{
  SelfContactFrictionPairFixture fixture;
  const std::vector<vbd::AvbdSelfContactFrictionRow> expected
      = fixture.frictionRows;

  std::vector<Vec3> positions(
      fixture.stepStart.begin(), fixture.stepStart.end());
  const std::vector<double> masses(positions.size(), 1.0);
  const std::vector<std::uint8_t> fixed(positions.size(), 0u);
  const std::vector<Vec3> inertialTargets = positions;

  const std::array<std::uint32_t, 4> vertices{0u, 1u, 2u, 3u};
  const std::vector<vbd::TetMeshElement> tets{
      {vertices,
       vbd::makeTetRestShape(
           {positions[0], positions[1], positions[2], positions[3]})}};
  const auto coloring = vbd::colorTetMesh(positions.size(), tets);
  const auto adjacency = vbd::TetAdjacency::build(positions.size(), tets);

  vbd::BlockDescentOptions options;
  options.iterations = 0u;
  vbd::AvbdSelfContactNormalOptions selfContactOptions;
  vbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;

  vbd::blockDescentTetMeshAvbdSelfContact(
      positions,
      masses,
      fixed,
      inertialTargets,
      tets,
      /*mu=*/500.0,
      /*lambda=*/800.0,
      /*timeStep=*/0.02,
      coloring,
      adjacency,
      options,
      nullptr,
      &fixture.normalRows,
      &selfContactOptions,
      &fixture.frictionRows,
      &selfContactFrictionOptions);

  ASSERT_EQ(fixture.frictionRows.size(), expected.size());
  for (std::size_t i = 0u; i < expected.size(); ++i) {
    SCOPED_TRACE(i);
    expectSelfContactFrictionRowUnchanged(fixture.frictionRows[i], expected[i]);
  }
}

//==============================================================================
// The cone refresh is a pair walk. Layouts that the driver does not declare
// paired must keep their own cones: pairing two rows that belong to different
// primitives fails the validity check and used to zero both of their duals.
TEST(
    VbdCombinedDescent,
    AvbdUnpairedSelfContactFrictionConeRefreshLeavesRowsUntouched)
{
  std::vector<Vec3> positions
      = {Vec3(0.2, 0.3, 0.005),
         Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0),
         Vec3(0.5, 0.5, -0.4)};
  const std::array<Vec3, 4> firstStencil
      = {positions[0], positions[1], positions[2], positions[3]};
  const std::array<Vec3, 4> secondStencil
      = {positions[1], positions[2], positions[3], positions[4]};
  const std::vector<double> masses(positions.size(), 1.0);
  const std::vector<std::uint8_t> fixed(positions.size(), 0u);
  std::vector<Vec3> inertialTargets = positions;
  inertialTargets[0] += Vec3(0.05, 0.0, 0.0);
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(positions.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(positions.size(), springs);

  std::vector<vbd::AvbdSelfContactNormalRow> normalRows(2u);
  normalRows[0].nodes = {0u, 1u, 2u, 3u};
  normalRows[0].state.stiffness = 1000.0;
  normalRows[0].squaredActivationDistance = 4e-4;
  normalRows[1].nodes = {1u, 2u, 3u, 4u};
  normalRows[1].state.stiffness = 1000.0;
  normalRows[1].squaredActivationDistance = 4e-4;

  // Two independent single-axis rows: neither the row array size nor the
  // stencils declare an adjacent tangent pair.
  vbd::AvbdSelfContactFrictionRow first;
  first.nodes = normalRows[0].nodes;
  first.stepStartPositions = firstStencil;
  first.axis = 0u;
  first.state = {/*stiffness=*/500.0, /*lambda=*/0.75};
  first.bounds = vbd::avbdFrictionTangentBounds(5.0);
  first.normalRow = 0u;
  first.frictionCoefficient = 0.5;
  vbd::AvbdSelfContactFrictionRow second = first;
  second.nodes = normalRows[1].nodes;
  second.stepStartPositions = secondStencil;
  second.state = {/*stiffness=*/700.0, /*lambda=*/-0.4};
  second.normalRow = 1u;
  std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows = {first, second};

  std::vector<vbd::AvbdHalfSpaceContactRow> contacts;
  std::vector<vbd::AvbdPointAttachmentRow> attachments;
  std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;

  vbd::BlockDescentOptions options;
  options.iterations = 1u;
  vbd::AvbdHalfSpaceContactOptions contactOptions;
  vbd::AvbdPointAttachmentOptions attachmentOptions;
  vbd::AvbdSpringFiniteStiffnessOptions springOptions;
  vbd::AvbdSelfContactNormalOptions selfContactOptions;
  vbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;

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
      static_cast<std::vector<vbd::AvbdHalfSpaceFrictionRow>*>(nullptr),
      nullptr,
      &normalRows,
      nullptr,
      &selfContactOptions,
      &frictionRows,
      &selfContactFrictionOptions);

  ASSERT_EQ(frictionRows.size(), 2u);
  for (const auto& row : frictionRows) {
    EXPECT_EQ(row.bounds.lower, first.bounds.lower);
    EXPECT_EQ(row.bounds.upper, first.bounds.upper);
  }
  EXPECT_NE(
      std::hypot(frictionRows[0].state.lambda, frictionRows[1].state.lambda),
      0.0);
}

//==============================================================================
TEST(
    VbdCombinedDescent,
    AvbdUnpairedHalfSpaceFrictionConeRefreshLeavesRowsUntouched)
{
  std::vector<Vec3> positions = {Vec3(0.0, 0.0, 0.01), Vec3(1.0, 0.0, 0.01)};
  const std::vector<double> masses(positions.size(), 1.0);
  const std::vector<std::uint8_t> fixed(positions.size(), 0u);
  const std::vector<Vec3> inertialTargets
      = {Vec3(0.2, 0.0, 0.01), Vec3(1.2, 0.0, 0.01)};
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(positions.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(positions.size(), springs);

  std::vector<vbd::AvbdHalfSpaceContactRow> contacts(2u);
  for (std::size_t i = 0u; i < contacts.size(); ++i) {
    contacts[i].vertex = static_cast<std::uint32_t>(i);
    contacts[i].plane.normal = Vec3::UnitZ();
    contacts[i].plane.offset = 0.0;
    contacts[i].state.stiffness = 100.0;
  }

  // One tangent row per vertex: the driver does not declare this layout paired
  // because the two rows do not share a vertex.
  vbd::AvbdHalfSpaceFrictionRow first;
  first.vertex = 0u;
  first.stepStartPosition = positions[0];
  first.axis = Vec3::UnitX();
  first.state = {/*stiffness=*/500.0, /*lambda=*/3.0};
  first.bounds = {-10.0, 10.0};
  first.normalRow = 0u;
  first.frictionCoefficient = 0.5;
  vbd::AvbdHalfSpaceFrictionRow second = first;
  second.vertex = 1u;
  second.stepStartPosition = positions[1];
  second.state = {/*stiffness=*/700.0, /*lambda=*/-4.0};
  second.normalRow = 1u;
  std::vector<vbd::AvbdHalfSpaceFrictionRow> frictionRows = {first, second};

  std::vector<vbd::AvbdPointAttachmentRow> attachments;
  std::vector<vbd::AvbdSpringFiniteStiffnessRow> springRows;

  vbd::BlockDescentOptions options;
  options.iterations = 1u;
  vbd::AvbdHalfSpaceContactOptions contactOptions;
  vbd::AvbdPointAttachmentOptions attachmentOptions;
  vbd::AvbdSpringFiniteStiffnessOptions springOptions;
  vbd::AvbdHalfSpaceFrictionOptions frictionOptions;

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
      &frictionRows,
      &frictionOptions);

  ASSERT_EQ(frictionRows.size(), 2u);
  for (const auto& row : frictionRows) {
    EXPECT_EQ(row.bounds.lower, -10.0);
    EXPECT_EQ(row.bounds.upper, 10.0);
  }
  EXPECT_NE(
      std::hypot(frictionRows[0].state.lambda, frictionRows[1].state.lambda),
      0.0);
}
