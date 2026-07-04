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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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
#include <dart/simulation/compute/detail/world_step_stages.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/io/deformable_scene_io.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace sx = dart::simulation;

namespace {

//==============================================================================
void setGroundBarrierPolicy(sx::RigidBody& body)
{
  auto policy = body.getDeformableObstaclePolicy();
  policy.groundBarrier = true;
  body.setDeformableObstaclePolicy(policy);
}

//==============================================================================
void setSurfaceObstaclePolicy(sx::RigidBody& body)
{
  auto policy = body.getDeformableObstaclePolicy();
  policy.surfaceObstacle = true;
  body.setDeformableObstaclePolicy(policy);
}

//==============================================================================
constexpr std::string_view kBenchmarkCubeMsh = R"msh($MeshFormat
4.1 0 8
$EndMeshFormat
$Nodes
1 8 1 8
3 0 0 8
1
2
3
4
5
6
7
8
0.000000e+00 0.000000e+00 0.000000e+00
1.000000e+00 0.000000e+00 0.000000e+00
1.000000e+00 0.000000e+00 1.000000e+00
0.000000e+00 0.000000e+00 1.000000e+00
0.000000e+00 1.000000e+00 0.000000e+00
1.000000e+00 1.000000e+00 0.000000e+00
1.000000e+00 1.000000e+00 1.000000e+00
0.000000e+00 1.000000e+00 1.000000e+00
$EndNodes
$Elements
1 6 1 6
3 0 4 6
1 1 5 7 6
2 4 7 3 1
3 1 5 8 7
4 4 7 1 8
5 3 1 7 2
6 7 1 6 2
$EndElements
$Surface
12
1 5 6
1 6 2
1 8 5
3 1 2
3 2 7
4 1 3
4 3 7
4 7 8
4 8 1
5 7 6
5 8 7
7 2 6
$EndSurface
)msh";

//==============================================================================
std::filesystem::path benchmarkScenePath()
{
  static const auto scenePath = [] {
    const auto root = std::filesystem::temp_directory_path()
                      / "dart_ipc_scene_replay_benchmark";
    std::filesystem::create_directories(root / "input" / "tetMeshes");
    {
      std::ofstream mesh(root / "input" / "tetMeshes" / "cube.msh");
      mesh << kBenchmarkCubeMsh;
    }
    {
      std::ofstream scene(root / "scene.txt");
      scene << R"scene(
turnOffGravity
time 1.0 0.01
shapes input 1
input/tetMeshes/cube.msh 0 0 0  0 0 0  1 1 1 DBC -0.1 -0.1 -0.1  0.1 1.1 1.1  0.25 0 0  0 0 0  NBC 0.9 -0.1 -0.1  1.1 1.1 1.1  -0.2 0 0
ground 0.1 0
)scene";
    }
    return root / "scene.txt";
  }();
  return scenePath;
}

//==============================================================================
sx::io::DeformableSceneLoadOptions makeBenchmarkSceneLoadOptions()
{
  sx::io::DeformableSceneLoadOptions options;
  options.assetRoot = benchmarkScenePath().parent_path();
  options.structuralSpringStiffness = 80.0;
  options.damping = 0.05;
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeTetraMeshOptions(int tetrahedronCount)
{
  tetrahedronCount = std::max(tetrahedronCount, 1);

  sx::DeformableBodyOptions options;
  options.edgeStiffness = 40.0;
  options.damping = 0.2;
  options.material.density = 12.0;

  for (int i = 0; i < tetrahedronCount; ++i) {
    const auto base = static_cast<std::size_t>(4 * i);
    const double offset = 0.18 * static_cast<double>(i);
    options.positions.push_back(Eigen::Vector3d(offset, 0.0, 1.0));
    options.positions.push_back(Eigen::Vector3d(offset + 0.08, 0.0, 1.0));
    options.positions.push_back(Eigen::Vector3d(offset, 0.08, 1.0));
    options.positions.push_back(Eigen::Vector3d(offset, 0.0, 1.08));
    for (int node = 0; node < 4; ++node) {
      options.velocities.push_back(Eigen::Vector3d::Zero());
    }
    options.tetrahedra.push_back({base, base + 1u, base + 2u, base + 3u});
    options.edges.push_back({base, base + 1u, -1.0});
    options.edges.push_back({base, base + 2u, -1.0});
    options.edges.push_back({base, base + 3u, -1.0});
    options.edges.push_back({base + 1u, base + 2u, -1.0});
    options.edges.push_back({base + 1u, base + 3u, -1.0});
    options.edges.push_back({base + 2u, base + 3u, -1.0});
  }

  return options;
}

//==============================================================================
// A connected FEM beam of ``cellsX`` hexahedral cells (each split into six
// tetrahedra), pinned at its x == 0 face and opting in to stable neo-Hookean
// FEM elasticity (no spring edges). Used to benchmark the FEM assembly + sparse
// projected-Newton solve cost as the element count grows.
sx::DeformableBodyOptions makeFemBarOptions(
    int cellsX,
    bool useFixedCorotational = false,
    bool useIterativeSolver = false,
    bool useMatrixFreeSolver = false)
{
  cellsX = std::max(cellsX, 1);
  const int nx = cellsX + 1;
  const int ny = 2;
  const int nz = 2;
  constexpr double h = 0.1;
  const auto nodeIndex = [&](int i, int j, int k) {
    return static_cast<std::size_t>(i + nx * (j + ny * k));
  };

  sx::DeformableBodyOptions options;
  options.material.density = 1.0e3;
  options.material.youngsModulus = 5.0e5;
  options.material.poissonRatio = 0.3;
  options.material.useFiniteElementElasticity = true;
  options.material.useFixedCorotationalElasticity = useFixedCorotational;
  options.material.useIterativeLinearSolver = useIterativeSolver;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeSolver;

  for (int k = 0; k < nz; ++k) {
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        options.positions.push_back(Eigen::Vector3d(i * h, j * h, 1.0 + k * h));
      }
    }
  }

  static constexpr int kCubeTets[6][4]
      = {{0, 1, 3, 7},
         {0, 3, 2, 7},
         {0, 2, 6, 7},
         {0, 6, 4, 7},
         {0, 4, 5, 7},
         {0, 5, 1, 7}};
  for (int ci = 0; ci < cellsX; ++ci) {
    std::array<std::size_t, 8> corner{};
    for (int b = 0; b < 8; ++b) {
      corner[static_cast<std::size_t>(b)]
          = nodeIndex(ci + (b & 1), (b >> 1) & 1, (b >> 2) & 1);
    }
    for (const auto& tet : kCubeTets) {
      options.tetrahedra.push_back(
          {corner[static_cast<std::size_t>(tet[0])],
           corner[static_cast<std::size_t>(tet[1])],
           corner[static_cast<std::size_t>(tet[2])],
           corner[static_cast<std::size_t>(tet[3])]});
    }
  }

  for (int k = 0; k < nz; ++k) {
    for (int j = 0; j < ny; ++j) {
      options.fixedNodes.push_back(nodeIndex(0, j, k));
    }
  }
  return options;
}

//==============================================================================
// A solid cellsPerSide^3 cube of FEM tetrahedra (each hexahedral cell split
// into six tets), pinned at its x == 0 face. Unlike the thin makeFemBarOptions
// beam (2x2 cross-section), this is a chunky 3D mesh, so the projected-Newton
// Hessian has a wide bandwidth: the sparse Cholesky direct solve suffers
// super-linear fill-in here while a preconditioned conjugate gradient stays
// near O(nnz), which is the regime the iterative solver is built for. Used to
// benchmark the direct-vs-iterative crossover as the element count grows.
sx::DeformableBodyOptions makeFemCubeOptions(
    int cellsPerSide, bool useIterativeSolver, bool useMatrixFreeSolver = false)
{
  const int cells = std::max(cellsPerSide, 1);
  const int n = cells + 1;
  constexpr double h = 0.04;
  const auto nodeIndex = [&](int i, int j, int k) {
    return static_cast<std::size_t>(i + n * (j + n * k));
  };

  sx::DeformableBodyOptions options;
  options.material.density = 1.0e3;
  options.material.youngsModulus = 2.0e5;
  options.material.poissonRatio = 0.3;
  options.material.useFiniteElementElasticity = true;
  options.material.useIterativeLinearSolver = useIterativeSolver;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeSolver;

  for (int k = 0; k < n; ++k) {
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i < n; ++i) {
        options.positions.push_back(Eigen::Vector3d(i * h, j * h, 1.0 + k * h));
      }
    }
  }

  static constexpr int kCubeTets[6][4]
      = {{0, 1, 3, 7},
         {0, 3, 2, 7},
         {0, 2, 6, 7},
         {0, 6, 4, 7},
         {0, 4, 5, 7},
         {0, 5, 1, 7}};
  for (int ck = 0; ck < cells; ++ck) {
    for (int cj = 0; cj < cells; ++cj) {
      for (int ci = 0; ci < cells; ++ci) {
        std::array<std::size_t, 8> corner{};
        for (int b = 0; b < 8; ++b) {
          corner[static_cast<std::size_t>(b)] = nodeIndex(
              ci + (b & 1), cj + ((b >> 1) & 1), ck + ((b >> 2) & 1));
        }
        for (const auto& tet : kCubeTets) {
          options.tetrahedra.push_back(
              {corner[static_cast<std::size_t>(tet[0])],
               corner[static_cast<std::size_t>(tet[1])],
               corner[static_cast<std::size_t>(tet[2])],
               corner[static_cast<std::size_t>(tet[3])]});
        }
      }
    }
  }

  for (int k = 0; k < n; ++k) {
    for (int j = 0; j < n; ++j) {
      options.fixedNodes.push_back(nodeIndex(0, j, k));
    }
  }
  return options;
}

//==============================================================================
// A solid cellsPerSide^3 FEM cube, unpinned and started just above a static
// ground barrier (top face at z == 0). Under gravity it drops and settles on
// the barrier, so the projected-Newton solve is dominated by the near-singular
// clamped-log barrier Hessian on the contacting bottom nodes -- the stiff,
// ill-conditioned regime where the matrix-free block-Jacobi preconditioner and
// the sparse incomplete-Cholesky preconditioner diverge in CG iterations and
// per-step cost. This is the contact-heavy fixture behind the matrix-free vs
// sparse-CG crossover benchmarks; the crossover it measures (per-step time, CG
// iterations, and fallbacks-to-steepest-descent as the mesh scales) is the
// evidence a future automatic large-mesh matrix-free selection policy needs.
sx::DeformableBodyOptions makeFemContactCubeOptions(
    int cellsPerSide, bool useIterativeSolver, bool useMatrixFreeSolver = false)
{
  const int cells = std::max(cellsPerSide, 1);
  const int n = cells + 1;
  constexpr double h = 0.04;
  // Bottom face starts one small gap above the barrier top (z == 0) so the
  // first steps drive it into the activation band.
  constexpr double kGroundGap = 0.02;

  sx::DeformableBodyOptions options;
  options.material.density = 1.0e3;
  options.material.youngsModulus = 1.5e5;
  options.material.poissonRatio = 0.3;
  options.material.useFiniteElementElasticity = true;
  options.material.useIterativeLinearSolver = useIterativeSolver;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeSolver;

  for (int k = 0; k < n; ++k) {
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i < n; ++i) {
        options.positions.push_back(
            Eigen::Vector3d(i * h, j * h, kGroundGap + k * h));
      }
    }
  }

  const auto nodeIndex = [&](int i, int j, int k) {
    return static_cast<std::size_t>(i + n * (j + n * k));
  };
  static constexpr int kCubeTets[6][4]
      = {{0, 1, 3, 7},
         {0, 3, 2, 7},
         {0, 2, 6, 7},
         {0, 6, 4, 7},
         {0, 4, 5, 7},
         {0, 5, 1, 7}};
  for (int ck = 0; ck < cells; ++ck) {
    for (int cj = 0; cj < cells; ++cj) {
      for (int ci = 0; ci < cells; ++ci) {
        std::array<std::size_t, 8> corner{};
        for (int b = 0; b < 8; ++b) {
          corner[static_cast<std::size_t>(b)] = nodeIndex(
              ci + (b & 1), cj + ((b >> 1) & 1), ck + ((b >> 2) & 1));
        }
        for (const auto& tet : kCubeTets) {
          options.tetrahedra.push_back(
              {corner[static_cast<std::size_t>(tet[0])],
               corner[static_cast<std::size_t>(tet[1])],
               corner[static_cast<std::size_t>(tet[2])],
               corner[static_cast<std::size_t>(tet[3])]});
        }
      }
    }
  }
  // No fixed nodes: the cube is free to fall and rest on the ground barrier.
  return options;
}

//==============================================================================
struct DeformableGridWorld
{
  DeformableGridWorld(int columns, int rows, bool withGround)
  {
    columns = std::max(columns, 2);
    rows = std::max(rows, 2);

    sx::DeformableBodyOptions options;
    options.edgeStiffness = 80.0;
    options.damping = 1.0;

    const auto index = [columns](int col, int row) {
      return static_cast<std::size_t>(row * columns + col);
    };

    constexpr double spacing = 0.08;
    const double halfWidth = 0.5 * spacing * static_cast<double>(columns - 1);
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < columns; ++col) {
        options.positions.push_back(
            Eigen::Vector3d(
                spacing * static_cast<double>(col) - halfWidth,
                0.01 * std::sin(0.31 * static_cast<double>(col + row)),
                1.0 - 0.05 * static_cast<double>(row)));
        options.velocities.push_back(Eigen::Vector3d::Zero());
        options.masses.push_back(0.05);
      }
    }

    options.fixedNodes = {index(0, 0), index(columns - 1, 0)};

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < columns; ++col) {
        if (col + 1 < columns) {
          options.edges.push_back({index(col, row), index(col + 1, row), -1.0});
        }
        if (row + 1 < rows) {
          options.edges.push_back({index(col, row), index(col, row + 1), -1.0});
        }
        if (col + 1 < columns && row + 1 < rows) {
          options.edges.push_back(
              {index(col, row), index(col + 1, row + 1), -1.0});
          options.edges.push_back(
              {index(col + 1, row), index(col, row + 1), -1.0});
        }
      }
    }

    if (withGround) {
      sx::RigidBodyOptions groundOptions;
      groundOptions.isStatic = true;
      groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
      auto ground = world.addRigidBody("ground", groundOptions);
      ground.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 4.0, 0.05)));
      setGroundBarrierPolicy(ground);
    }

    body = world.addDeformableBody("grid", options);
    nodeCount = body.getNodeCount();
    edgeCount = body.getEdgeCount();

    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t edgeCount = 0;
};

// A deformable mat draped over a raised box ground barrier onto the flat ground
// (the deformable drape demo's solver scenario). The mat is
// started near its draped equilibrium so the benchmark measures the per-step
// sparse-Newton solve under two active ground barriers rather than the dynamic
// fall.
struct DeformableDrapeWorld
{
  DeformableDrapeWorld(int columns, int rows, double frictionCoefficient = 0.0)
  {
    columns = std::max(columns, 2);
    rows = std::max(rows, 2);

    constexpr double boxHalf = 0.30;
    constexpr double boxTop = 0.24;
    constexpr double spacing = 0.05;

    sx::DeformableBodyOptions options;
    options.edgeStiffness = 25.0;
    options.damping = 1.5;
    options.material.frictionCoefficient = frictionCoefficient;

    const auto index = [columns](int col, int row) {
      return static_cast<std::size_t>(row * columns + col);
    };

    const double halfWidth = 0.5 * spacing * static_cast<double>(columns - 1);
    const double halfDepth = 0.5 * spacing * static_cast<double>(rows - 1);
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < columns; ++col) {
        const double x = spacing * static_cast<double>(col) - halfWidth;
        const double y = spacing * static_cast<double>(row) - halfDepth;
        const bool overBox = std::abs(x) <= boxHalf && std::abs(y) <= boxHalf;
        options.positions.push_back(
            Eigen::Vector3d(x, y, overBox ? boxTop + 0.005 : 0.005));
        options.velocities.push_back(Eigen::Vector3d::Zero());
        options.masses.push_back(0.05);
      }
    }

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < columns; ++col) {
        if (col + 1 < columns) {
          options.edges.push_back({index(col, row), index(col + 1, row), -1.0});
        }
        if (row + 1 < rows) {
          options.edges.push_back({index(col, row), index(col, row + 1), -1.0});
        }
        if (col + 1 < columns && row + 1 < rows) {
          options.edges.push_back(
              {index(col, row), index(col + 1, row + 1), -1.0});
          options.edges.push_back(
              {index(col + 1, row), index(col, row + 1), -1.0});
        }
      }
    }

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
    setGroundBarrierPolicy(ground);

    sx::RigidBodyOptions boxOptions;
    boxOptions.isStatic = true;
    boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5 * boxTop);
    auto box = world.addRigidBody("step", boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(
            Eigen::Vector3d(boxHalf, boxHalf, 0.5 * boxTop)));
    setGroundBarrierPolicy(box);

    body = world.addDeformableBody("drape", options);
    nodeCount = body.getNodeCount();
    edgeCount = body.getEdgeCount();

    world.setTimeStep(1.0 / 120.0);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t edgeCount = 0;
};

//==============================================================================
struct DeformableTetraMeshWorld
{
  explicit DeformableTetraMeshWorld(int tetrahedronCount)
  {
    auto options = makeTetraMeshOptions(tetrahedronCount);
    body = world.addDeformableBody("tetra_mesh", options);
    nodeCount = body.getNodeCount();
    edgeCount = body.getEdgeCount();
    surfaceTriangleCount = body.getSurfaceTriangleCount();
    this->tetrahedronCount = body.getTetrahedronCount();
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      totalMass += body.getMass(i);
    }

    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t edgeCount = 0;
  std::size_t surfaceTriangleCount = 0;
  std::size_t tetrahedronCount = 0;
  double totalMass = 0.0;
};

//==============================================================================
struct DeformableSurfaceContactWorld
{
  DeformableSurfaceContactWorld(int pairCount, bool crossing)
  {
    pairCount = std::max(pairCount, 1);

    sx::DeformableBodyOptions options;
    options.edgeStiffness = 0.0;
    options.damping = 0.0;

    constexpr double spacing = 3.0;
    for (int i = 0; i < pairCount; ++i) {
      const auto base = static_cast<std::size_t>(4 * i);
      const double offset = spacing * static_cast<double>(i);
      options.positions.push_back(Eigen::Vector3d(offset - 1.0, -1.0, 0.0));
      options.positions.push_back(Eigen::Vector3d(offset + 1.0, -1.0, 0.0));
      options.positions.push_back(Eigen::Vector3d(offset, 1.0, 0.0));
      options.positions.push_back(Eigen::Vector3d(offset, 0.0, 1.0));

      options.velocities.push_back(Eigen::Vector3d::Zero());
      options.velocities.push_back(Eigen::Vector3d::Zero());
      options.velocities.push_back(Eigen::Vector3d::Zero());
      options.velocities.push_back(
          Eigen::Vector3d(0.0, 0.0, crossing ? -20.0 : 1.0));

      for (int node = 0; node < 4; ++node) {
        options.masses.push_back(1.0);
      }
      options.fixedNodes.push_back(base);
      options.fixedNodes.push_back(base + 1u);
      options.fixedNodes.push_back(base + 2u);
      options.surfaceTriangles.push_back({base, base + 1u, base + 2u});
    }

    initialPositions = options.positions;
    initialVelocities = options.velocities;
    body = world.addDeformableBody("surface_contact", options);
    nodeCount = body.getNodeCount();
    surfaceTriangleCount = body.getSurfaceTriangleCount();

    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    world.enterSimulationMode();
  }

  void reset()
  {
    for (std::size_t i = 0; i < initialPositions.size(); ++i) {
      body.setPosition(i, initialPositions[i]);
      body.setVelocity(i, initialVelocities[i]);
    }
  }

  sx::World world;
  sx::DeformableBody body;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::size_t nodeCount = 0;
  std::size_t surfaceTriangleCount = 0;
};

//==============================================================================
struct DeformableInterBodySurfaceContactWorld
{
  DeformableInterBodySurfaceContactWorld(int obstacleCount, bool crossing)
  {
    obstacleCount = std::max(obstacleCount, 1);

    sx::DeformableBodyOptions movingOptions;
    movingOptions.edgeStiffness = 0.0;
    movingOptions.damping = 0.0;

    constexpr double spacing = 3.0;
    for (int i = 0; i < obstacleCount; ++i) {
      const auto base = static_cast<std::size_t>(4 * i);
      const double offset = spacing * static_cast<double>(i);
      movingOptions.positions.push_back(
          Eigen::Vector3d(offset - 1.0, -1.0, 3.0));
      movingOptions.positions.push_back(
          Eigen::Vector3d(offset + 1.0, -1.0, 3.0));
      movingOptions.positions.push_back(Eigen::Vector3d(offset, 1.0, 3.0));
      movingOptions.positions.push_back(Eigen::Vector3d(offset, 0.0, 1.0));

      movingOptions.velocities.push_back(Eigen::Vector3d::Zero());
      movingOptions.velocities.push_back(Eigen::Vector3d::Zero());
      movingOptions.velocities.push_back(Eigen::Vector3d::Zero());
      movingOptions.velocities.push_back(
          Eigen::Vector3d(0.0, 0.0, crossing ? -20.0 : 1.0));

      for (int node = 0; node < 4; ++node) {
        movingOptions.masses.push_back(1.0);
      }
      movingOptions.fixedNodes.push_back(base);
      movingOptions.fixedNodes.push_back(base + 1u);
      movingOptions.fixedNodes.push_back(base + 2u);
      movingOptions.surfaceTriangles.push_back({base, base + 1u, base + 2u});

      sx::DeformableBodyOptions obstacleOptions;
      obstacleOptions.positions
          = {Eigen::Vector3d(offset - 1.0, -1.0, 0.0),
             Eigen::Vector3d(offset + 1.0, -1.0, 0.0),
             Eigen::Vector3d(offset, 1.0, 0.0)};
      obstacleOptions.velocities
          = {Eigen::Vector3d::Zero(),
             Eigen::Vector3d::Zero(),
             Eigen::Vector3d::Zero()};
      obstacleOptions.masses = {1.0, 1.0, 1.0};
      obstacleOptions.fixedNodes = {0, 1, 2};
      obstacleOptions.surfaceTriangles = {{0, 1, 2}};
      obstacleBodies.push_back(world.addDeformableBody(
          "inter_body_obstacle_" + std::to_string(i), obstacleOptions));
    }

    initialPositions = movingOptions.positions;
    initialVelocities = movingOptions.velocities;
    movingBody = world.addDeformableBody("inter_body_moving", movingOptions);
    nodeCount = movingBody.getNodeCount()
                + 3u * static_cast<std::size_t>(obstacleCount);
    surfaceTriangleCount = movingBody.getSurfaceTriangleCount()
                           + static_cast<std::size_t>(obstacleCount);

    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    world.enterSimulationMode();
  }

  void reset()
  {
    for (std::size_t i = 0; i < initialPositions.size(); ++i) {
      movingBody.setPosition(i, initialPositions[i]);
      movingBody.setVelocity(i, initialVelocities[i]);
    }
  }

  sx::World world;
  sx::DeformableBody movingBody;
  std::vector<sx::DeformableBody> obstacleBodies;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::size_t nodeCount = 0;
  std::size_t surfaceTriangleCount = 0;
};

//==============================================================================
struct DeformableStaticGroundBarrierCcdWorld
{
  DeformableStaticGroundBarrierCcdWorld(int barrierCount, bool crossing)
  {
    barrierCount = std::max(barrierCount, 1);

    sx::DeformableBodyOptions movingOptions;
    movingOptions.edgeStiffness = 0.0;
    movingOptions.damping = 0.0;

    constexpr double spacing = 3.0;
    for (int i = 0; i < barrierCount; ++i) {
      const double offset = spacing * static_cast<double>(i);
      movingOptions.positions.push_back(Eigen::Vector3d(offset, 0.0, 1.0));
      movingOptions.velocities.push_back(
          Eigen::Vector3d(0.0, 0.0, crossing ? -20.0 : 1.0));
      movingOptions.masses.push_back(1.0);

      sx::RigidBodyOptions groundOptions;
      groundOptions.isStatic = true;
      groundOptions.position = Eigen::Vector3d(offset, 0.0, -0.05);
      auto ground
          = world.addRigidBody("ground_" + std::to_string(i), groundOptions);
      ground.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 1.0, 0.05)));
      setGroundBarrierPolicy(ground);
      groundBodies.push_back(ground);
    }

    initialPositions = movingOptions.positions;
    initialVelocities = movingOptions.velocities;
    movingBody = world.addDeformableBody("ground_ccd_nodes", movingOptions);
    nodeCount = movingBody.getNodeCount();
    this->barrierCount = static_cast<std::size_t>(barrierCount);

    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    world.enterSimulationMode();
  }

  void reset()
  {
    for (std::size_t i = 0; i < initialPositions.size(); ++i) {
      movingBody.setPosition(i, initialPositions[i]);
      movingBody.setVelocity(i, initialVelocities[i]);
    }
  }

  sx::World world;
  sx::DeformableBody movingBody;
  std::vector<sx::RigidBody> groundBodies;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::size_t nodeCount = 0;
  std::size_t barrierCount = 0;
};

//==============================================================================
struct DeformableStaticRigidSurfaceCcdWorld
{
  DeformableStaticRigidSurfaceCcdWorld(int boxCount, bool crossing)
  {
    boxCount = std::max(boxCount, 0);

    sx::DeformableBodyOptions movingOptions;
    movingOptions.edgeStiffness = 0.0;
    movingOptions.damping = 0.0;

    constexpr double spacing = 3.0;
    const int nodeCount = std::max(boxCount, 1);
    for (int i = 0; i < nodeCount; ++i) {
      const double offset = spacing * static_cast<double>(i);
      movingOptions.positions.push_back(
          Eigen::Vector3d(offset - 1.0, 0.0, 0.0));
      movingOptions.velocities.push_back(
          Eigen::Vector3d(crossing ? 20.0 : -1.0, 0.0, 0.0));
      movingOptions.masses.push_back(1.0);

      if (i < boxCount) {
        sx::RigidBodyOptions boxOptions;
        boxOptions.isStatic = true;
        boxOptions.position = Eigen::Vector3d(offset, 0.0, 0.0);
        auto box = world.addRigidBody(
            "surface_ccd_box_" + std::to_string(i), boxOptions);
        box.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
        setSurfaceObstaclePolicy(box);
        boxes.push_back(box);
      }
    }

    initialPositions = movingOptions.positions;
    initialVelocities = movingOptions.velocities;
    movingBody = world.addDeformableBody("surface_ccd_points", movingOptions);
    this->nodeCount = movingBody.getNodeCount();
    this->boxCount = static_cast<std::size_t>(boxCount);

    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    world.enterSimulationMode();
  }

  void reset()
  {
    for (std::size_t i = 0; i < initialPositions.size(); ++i) {
      movingBody.setPosition(i, initialPositions[i]);
      movingBody.setVelocity(i, initialVelocities[i]);
    }
  }

  sx::World world;
  sx::DeformableBody movingBody;
  std::vector<sx::RigidBody> boxes;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::size_t nodeCount = 0;
  std::size_t boxCount = 0;
};

//==============================================================================
struct DeformableMovingRigidSurfaceCcdWorld
{
  DeformableMovingRigidSurfaceCcdWorld(int boxCount, bool crossing)
  {
    boxCount = std::max(boxCount, 0);

    sx::DeformableBodyOptions movingOptions;
    movingOptions.edgeStiffness = 0.0;
    movingOptions.damping = 0.0;

    constexpr double spacing = 3.0;
    const int nodeCount = std::max(boxCount, 1);
    for (int i = 0; i < nodeCount; ++i) {
      const double offset = spacing * static_cast<double>(i);
      movingOptions.positions.push_back(
          Eigen::Vector3d(offset - 1.0, 0.0, 0.0));
      movingOptions.velocities.push_back(
          Eigen::Vector3d(crossing ? 20.0 : -1.0, 0.0, 0.0));
      movingOptions.masses.push_back(1.0);

      if (i < boxCount) {
        // Free (non-static) obstacle moving toward the node when crossing, so
        // the deformable stage predicts its swept motion and limits against it.
        sx::RigidBodyOptions boxOptions;
        boxOptions.isStatic = false;
        boxOptions.position = Eigen::Vector3d(offset, 0.0, 0.0);
        auto box = world.addRigidBody(
            "moving_surface_ccd_box_" + std::to_string(i), boxOptions);
        box.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
        setSurfaceObstaclePolicy(box);
        box.setLinearVelocity(Eigen::Vector3d(crossing ? -2.0 : 2.0, 0.0, 0.0));
        boxes.push_back(box);
      }
    }

    initialPositions = movingOptions.positions;
    initialVelocities = movingOptions.velocities;
    movingBody = world.addDeformableBody("surface_ccd_points", movingOptions);
    this->nodeCount = movingBody.getNodeCount();
    this->boxCount = static_cast<std::size_t>(boxCount);

    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    world.enterSimulationMode();
  }

  void reset()
  {
    for (std::size_t i = 0; i < initialPositions.size(); ++i) {
      movingBody.setPosition(i, initialPositions[i]);
      movingBody.setVelocity(i, initialVelocities[i]);
    }
  }

  sx::World world;
  sx::DeformableBody movingBody;
  std::vector<sx::RigidBody> boxes;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::size_t nodeCount = 0;
  std::size_t boxCount = 0;
};

//==============================================================================
struct DeformableSelfContactBarrierWorld
{
  explicit DeformableSelfContactBarrierWorld(int pairCount)
  {
    pairCount = std::max(pairCount, 1);

    sx::DeformableBodyOptions options;
    options.edgeStiffness = 0.0;
    options.damping = 0.0;

    constexpr double gap = 0.015; // inside the barrier activation band (2e-2)
    constexpr double spacing = 3.0;
    for (int i = 0; i < pairCount; ++i) {
      const double x = spacing * static_cast<double>(i);
      const auto base = static_cast<std::size_t>(options.positions.size());
      // Lower (fixed) triangle.
      options.positions.push_back(Eigen::Vector3d(x + 0.0, 0.0, 0.0));
      options.positions.push_back(Eigen::Vector3d(x + 1.0, 0.0, 0.0));
      options.positions.push_back(Eigen::Vector3d(x + 0.0, 1.0, 0.0));
      // Upper triangle driven down onto the lower one.
      options.positions.push_back(Eigen::Vector3d(x + 0.0, 0.0, gap));
      options.positions.push_back(Eigen::Vector3d(x + 1.0, 0.0, gap));
      options.positions.push_back(Eigen::Vector3d(x + 0.0, 1.0, gap));
      for (int k = 0; k < 3; ++k) {
        options.velocities.push_back(Eigen::Vector3d::Zero());
      }
      for (int k = 0; k < 3; ++k) {
        options.velocities.push_back(Eigen::Vector3d(0.0, 0.0, -0.2));
      }
      for (int k = 0; k < 6; ++k) {
        options.masses.push_back(1.0);
      }
      options.fixedNodes.push_back(base + 0);
      options.fixedNodes.push_back(base + 1);
      options.fixedNodes.push_back(base + 2);
      options.surfaceTriangles.push_back(
          sx::DeformableSurfaceTriangle{base + 0, base + 1, base + 2});
      options.surfaceTriangles.push_back(
          sx::DeformableSurfaceTriangle{base + 3, base + 4, base + 5});
    }

    initialPositions = options.positions;
    initialVelocities = options.velocities;
    body = world.addDeformableBody("self_contact_barrier", options);
    nodeCount = body.getNodeCount();
    this->pairCount = static_cast<std::size_t>(pairCount);

    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    world.enterSimulationMode();
  }

  void reset()
  {
    for (std::size_t i = 0; i < initialPositions.size(); ++i) {
      body.setPosition(i, initialPositions[i]);
      body.setVelocity(i, initialVelocities[i]);
    }
  }

  sx::World world;
  sx::DeformableBody body;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::size_t nodeCount = 0;
  std::size_t pairCount = 0;
};

//==============================================================================
struct RigidOnlyWorld
{
  explicit RigidOnlyWorld(int staticBodyCount)
  {
    for (int i = 0; i < staticBodyCount; ++i) {
      sx::RigidBodyOptions options;
      options.isStatic = true;
      options.position
          = Eigen::Vector3d(2.0 * static_cast<double>(i), 0.0, -0.05);
      auto body
          = world.addRigidBody("static_box_" + std::to_string(i), options);
      body.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 1.0, 0.05)));
    }

    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
};

//==============================================================================
void BM_WorldStepWithoutDeformables(benchmark::State& state)
{
  const auto staticBodyCount = static_cast<int>(state.range(0));
  RigidOnlyWorld fixture(staticBodyCount);

  for (auto _ : state) {
    fixture.world.step();
    benchmark::DoNotOptimize(fixture.world.getFrame());
  }

  state.counters["static_bodies"] = static_cast<double>(staticBodyCount);
}

//==============================================================================
void BM_DeformableGridStep(benchmark::State& state)
{
  const auto columns = static_cast<int>(state.range(0));
  const auto rows = static_cast<int>(state.range(1));
  const bool withGround = state.range(2) != 0;
  DeformableGridWorld fixture(columns, rows, withGround);

  for (auto _ : state) {
    fixture.world.step();
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["ground"] = withGround ? 1.0 : 0.0;
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableTetraMeshSetup(benchmark::State& state)
{
  const auto tetrahedronCount = static_cast<int>(state.range(0));

  for (auto _ : state) {
    sx::World world;
    auto body = world.addDeformableBody(
        "tetra_mesh", makeTetraMeshOptions(tetrahedronCount));
    benchmark::DoNotOptimize(body.getSurfaceTriangleCount());
    benchmark::DoNotOptimize(body.getMass(body.getNodeCount() - 1u));
  }

  state.counters["tetrahedra"] = static_cast<double>(tetrahedronCount);
  state.counters["nodes"] = static_cast<double>(4 * tetrahedronCount);
  state.counters["surface_triangles"]
      = static_cast<double>(4 * tetrahedronCount);
  state.counters["contact_constraints"] = 0.0;
}

//==============================================================================
void BM_DeformableTetraMeshStep(benchmark::State& state)
{
  const auto tetrahedronCount = static_cast<int>(state.range(0));
  DeformableTetraMeshWorld fixture(tetrahedronCount);

  for (auto _ : state) {
    fixture.world.step();
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["surface_triangles"]
      = static_cast<double>(fixture.surfaceTriangleCount);
  state.counters["fixed_nodes"] = 0.0;
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["contact_constraints"] = 0.0;
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
struct DeformableFemBarWorld
{
  explicit DeformableFemBarWorld(
      int cellsX,
      bool useFixedCorotational = false,
      bool useIterativeSolver = false,
      bool useMatrixFreeSolver = false)
  {
    body = world.addDeformableBody(
        "fem_bar",
        makeFemBarOptions(
            cellsX,
            useFixedCorotational,
            useIterativeSolver,
            useMatrixFreeSolver));
    nodeCount = body.getNodeCount();
    tetrahedronCount = body.getTetrahedronCount();
    for (std::size_t i = 0; i < nodeCount; ++i) {
      totalMass += body.getMass(i);
    }
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t tetrahedronCount = 0;
  double totalMass = 0.0;
};

//==============================================================================
// Steps a connected FEM beam (stable neo-Hookean elasticity) under gravity:
// measures the per-step cost of the FEM energy/gradient/Hessian assembly plus
// the sparse projected-Newton solve as the tetrahedron count grows.
void BM_DeformableFemBarStep(benchmark::State& state)
{
  const auto cellsX = static_cast<int>(state.range(0));
  DeformableFemBarWorld fixture(cellsX);

  double totalNewtonIterations = 0.0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxHessianStorageBytes = 0;
  for (auto _ : state) {
    fixture.world.step();
    const auto& diagnostics
        = fixture.world.getLastDeformableSolverDiagnostics();
    totalNewtonIterations += static_cast<double>(diagnostics.solverIterations);
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxHessianStorageBytes = std::max(
        maxHessianStorageBytes, diagnostics.projectedNewtonHessianStorageBytes);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["contact_constraints"] = 0.0;
  // Average projected-Newton iterations per step: the convergence axis the IPC
  // paper's Table 1 reports, alongside the wall-clock per-step time.
  state.counters["newton_iters_per_step"]
      = totalNewtonIterations / static_cast<double>(state.iterations());
  state.counters["hessian_nonzeros"] = static_cast<double>(maxHessianNonZeros);
  state.counters["hessian_storage_bytes"]
      = static_cast<double>(maxHessianStorageBytes);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
// Identical beam stepped with the fixed-corotational material instead of stable
// neo-Hookean. Run at the same cell counts as BM_DeformableFemBarStep to
// compare per-step solver cost between the two FEM materials at matching mesh
// resolution (the per-step-time axis of the IPC paper's Fig. 23 / Table 1).
void BM_DeformableFcrBarStep(benchmark::State& state)
{
  const auto cellsX = static_cast<int>(state.range(0));
  DeformableFemBarWorld fixture(cellsX, /*useFixedCorotational=*/true);

  double totalNewtonIterations = 0.0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxHessianStorageBytes = 0;
  for (auto _ : state) {
    fixture.world.step();
    const auto& diagnostics
        = fixture.world.getLastDeformableSolverDiagnostics();
    totalNewtonIterations += static_cast<double>(diagnostics.solverIterations);
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxHessianStorageBytes = std::max(
        maxHessianStorageBytes, diagnostics.projectedNewtonHessianStorageBytes);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["contact_constraints"] = 0.0;
  // Average projected-Newton iterations per step, so the fixed-corotational
  // material's convergence can be compared to neo-Hookean at equal resolution.
  state.counters["newton_iters_per_step"]
      = totalNewtonIterations / static_cast<double>(state.iterations());
  state.counters["hessian_nonzeros"] = static_cast<double>(maxHessianNonZeros);
  state.counters["hessian_storage_bytes"]
      = static_cast<double>(maxHessianStorageBytes);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
// The same stable neo-Hookean beam stepped through the iterative (conjugate-
// gradient) projected-Newton linear solve instead of the sparse Cholesky
// direct solve. Run at the same cell counts as BM_DeformableFemBarStep so the
// per-step wall-clock and Newton-iteration counts can be compared directly:
// CG never factorizes, so its memory stays near O(nnz) and its per-step cost
// grows more gently than the direct solve's fill-in as the mesh chunks up --
// the scaling axis of the IPC paper's Fig. 23 / Table 1. (On the thin bar
// here, the direct solve's bandwidth is small, so CG's asymptotic advantage
// only shows on larger, chunkier meshes; this benchmark measures the CG path's
// cost and convergence so that scaling can be tracked.)
void BM_DeformableCgBarStep(benchmark::State& state)
{
  const auto cellsX = static_cast<int>(state.range(0));
  DeformableFemBarWorld fixture(
      cellsX, /*useFixedCorotational=*/false, /*useIterativeSolver=*/true);

  double totalNewtonIterations = 0.0;
  double totalCgIterations = 0.0;
  double maxCgError = 0.0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxHessianStorageBytes = 0;
  for (auto _ : state) {
    fixture.world.step();
    const auto& diagnostics
        = fixture.world.getLastDeformableSolverDiagnostics();
    totalNewtonIterations += static_cast<double>(diagnostics.solverIterations);
    totalCgIterations
        += static_cast<double>(diagnostics.projectedNewtonIterativeIterations);
    maxCgError
        = std::max(maxCgError, diagnostics.projectedNewtonIterativeMaxError);
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxHessianStorageBytes = std::max(
        maxHessianStorageBytes, diagnostics.projectedNewtonHessianStorageBytes);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["contact_constraints"] = 0.0;
  state.counters["newton_iters_per_step"]
      = totalNewtonIterations / static_cast<double>(state.iterations());
  state.counters["cg_iters_per_step"]
      = totalCgIterations / static_cast<double>(state.iterations());
  state.counters["cg_max_error"] = maxCgError;
  state.counters["hessian_nonzeros"] = static_cast<double>(maxHessianNonZeros);
  state.counters["hessian_storage_bytes"]
      = static_cast<double>(maxHessianStorageBytes);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
// The same FEM beam through the explicit matrix-free CG path. Unlike
// BM_DeformableCgBarStep, this path does not assemble an Eigen SparseMatrix for
// the Hessian; it applies local Hessian blocks directly and uses a block-Jacobi
// preconditioner. This gives the M7 profiling surface a memory-removal axis
// before the larger AMG/GPU solve work.
void BM_DeformableMatrixFreeCgBarStep(benchmark::State& state)
{
  const auto cellsX = static_cast<int>(state.range(0));
  DeformableFemBarWorld fixture(
      cellsX,
      /*useFixedCorotational=*/false,
      /*useIterativeSolver=*/false,
      /*useMatrixFreeSolver=*/true);

  double totalNewtonIterations = 0.0;
  double totalCgIterations = 0.0;
  double totalMatrixFreeSolves = 0.0;
  double maxCgError = 0.0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxHessianStorageBytes = 0;
  for (auto _ : state) {
    fixture.world.step();
    const auto& diagnostics
        = fixture.world.getLastDeformableSolverDiagnostics();
    totalNewtonIterations += static_cast<double>(diagnostics.solverIterations);
    totalCgIterations
        += static_cast<double>(diagnostics.projectedNewtonIterativeIterations);
    totalMatrixFreeSolves
        += static_cast<double>(diagnostics.projectedNewtonMatrixFreeSolves);
    maxCgError
        = std::max(maxCgError, diagnostics.projectedNewtonIterativeMaxError);
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxHessianStorageBytes = std::max(
        maxHessianStorageBytes, diagnostics.projectedNewtonHessianStorageBytes);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["contact_constraints"] = 0.0;
  state.counters["newton_iters_per_step"]
      = totalNewtonIterations / static_cast<double>(state.iterations());
  state.counters["cg_iters_per_step"]
      = totalCgIterations / static_cast<double>(state.iterations());
  state.counters["matrix_free_solves_per_step"]
      = totalMatrixFreeSolves / static_cast<double>(state.iterations());
  state.counters["cg_max_error"] = maxCgError;
  state.counters["hessian_nonzeros"] = static_cast<double>(maxHessianNonZeros);
  state.counters["hessian_storage_bytes"]
      = static_cast<double>(maxHessianStorageBytes);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
struct DeformableFemCubeWorld
{
  DeformableFemCubeWorld(
      int cellsPerSide,
      bool useIterativeSolver,
      bool useMatrixFreeSolver = false)
  {
    body = world.addDeformableBody(
        "fem_cube",
        makeFemCubeOptions(
            cellsPerSide, useIterativeSolver, useMatrixFreeSolver));
    nodeCount = body.getNodeCount();
    tetrahedronCount = body.getTetrahedronCount();
    for (std::size_t i = 0; i < nodeCount; ++i) {
      totalMass += body.getMass(i);
    }
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t tetrahedronCount = 0;
  double totalMass = 0.0;
};

//==============================================================================
struct DeformableFemContactCubeWorld
{
  DeformableFemContactCubeWorld(
      int cellsPerSide,
      bool useIterativeSolver,
      bool useMatrixFreeSolver = false)
  {
    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
    setGroundBarrierPolicy(ground); // top face at z = 0

    body = world.addDeformableBody(
        "contact_cube",
        makeFemContactCubeOptions(
            cellsPerSide, useIterativeSolver, useMatrixFreeSolver));
    nodeCount = body.getNodeCount();
    tetrahedronCount = body.getTetrahedronCount();
    for (std::size_t i = 0; i < nodeCount; ++i) {
      totalMass += body.getMass(i);
    }
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t tetrahedronCount = 0;
  double totalMass = 0.0;
};

//==============================================================================
// Steps a chunky cellsPerSide^3 FEM cube and records the per-step cost and
// Newton-iteration count. The cube is run with both the sparse Cholesky direct
// solve (BM_DeformableCube3dDirectStep) and the incomplete-Cholesky
// preconditioned conjugate-gradient iterative solve (BM_DeformableCube3dCgStep)
// at matching cell counts, so the direct-vs-iterative crossover is measurable:
// the direct solve's 3D fill-in makes its per-step time climb super-linearly
// with the element count, while the iterative solve stays near O(nnz) and
// overtakes it by a few thousand nodes (the IPC paper's Fig. 23 / Table 1
// per-step-scaling axis, on the chunky mesh where it bites -- the thin
// BM_DeformableFemBarStep beam has too small a bandwidth to show it).
void BM_DeformableCube3dStep(
    benchmark::State& state, bool useIterativeSolver, bool useMatrixFreeSolver)
{
  const auto cellsPerSide = static_cast<int>(state.range(0));
  DeformableFemCubeWorld fixture(
      cellsPerSide, useIterativeSolver, useMatrixFreeSolver);

  double totalNewtonIterations = 0.0;
  double totalCgIterations = 0.0;
  double totalMatrixFreeSolves = 0.0;
  double maxCgError = 0.0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxHessianStorageBytes = 0;
  for (auto _ : state) {
    fixture.world.step();
    const auto& diagnostics
        = fixture.world.getLastDeformableSolverDiagnostics();
    totalNewtonIterations += static_cast<double>(diagnostics.solverIterations);
    totalCgIterations
        += static_cast<double>(diagnostics.projectedNewtonIterativeIterations);
    totalMatrixFreeSolves
        += static_cast<double>(diagnostics.projectedNewtonMatrixFreeSolves);
    maxCgError
        = std::max(maxCgError, diagnostics.projectedNewtonIterativeMaxError);
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxHessianStorageBytes = std::max(
        maxHessianStorageBytes, diagnostics.projectedNewtonHessianStorageBytes);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["contact_constraints"] = 0.0;
  state.counters["newton_iters_per_step"]
      = totalNewtonIterations / static_cast<double>(state.iterations());
  state.counters["cg_iters_per_step"]
      = totalCgIterations / static_cast<double>(state.iterations());
  state.counters["matrix_free_solves_per_step"]
      = totalMatrixFreeSolves / static_cast<double>(state.iterations());
  state.counters["cg_max_error"] = maxCgError;
  state.counters["hessian_nonzeros"] = static_cast<double>(maxHessianNonZeros);
  state.counters["hessian_storage_bytes"]
      = static_cast<double>(maxHessianStorageBytes);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

void BM_DeformableCube3dDirectStep(benchmark::State& state)
{
  BM_DeformableCube3dStep(
      state, /*useIterativeSolver=*/false, /*useMatrixFreeSolver=*/false);
}

void BM_DeformableCube3dCgStep(benchmark::State& state)
{
  BM_DeformableCube3dStep(
      state, /*useIterativeSolver=*/true, /*useMatrixFreeSolver=*/false);
}

void BM_DeformableCube3dMatrixFreeCgStep(benchmark::State& state)
{
  BM_DeformableCube3dStep(
      state, /*useIterativeSolver=*/false, /*useMatrixFreeSolver=*/true);
}

//==============================================================================
// Contact-heavy matrix-free-vs-sparse-CG crossover: an unpinned FEM cube
// settles on a static ground barrier, so the projected-Newton solve is
// dominated by the stiff clamped-log barrier Hessian. Running the same scene
// under sparse incomplete-Cholesky CG and matrix-free block-Jacobi CG at
// increasing cube resolutions makes their per-step time, CG iterations, and
// fallbacks-to-steepest-descent directly comparable on contact -- the evidence
// a future automatic large-mesh matrix-free selection policy needs. Neither
// path is the default (both are opt-in), so this measures without changing any
// production selection.
void BM_DeformableContactCubeStep(
    benchmark::State& state, bool useIterativeSolver, bool useMatrixFreeSolver)
{
  const auto cellsPerSide = static_cast<int>(state.range(0));
  DeformableFemContactCubeWorld fixture(
      cellsPerSide, useIterativeSolver, useMatrixFreeSolver);

  double totalNewtonIterations = 0.0;
  double totalCgIterations = 0.0;
  double totalMatrixFreeSolves = 0.0;
  double totalFallbacks = 0.0;
  double maxCgError = 0.0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxHessianStorageBytes = 0;
  for (auto _ : state) {
    fixture.world.step();
    const auto& diagnostics
        = fixture.world.getLastDeformableSolverDiagnostics();
    totalNewtonIterations += static_cast<double>(diagnostics.solverIterations);
    totalCgIterations
        += static_cast<double>(diagnostics.projectedNewtonIterativeIterations);
    totalMatrixFreeSolves
        += static_cast<double>(diagnostics.projectedNewtonMatrixFreeSolves);
    totalFallbacks += static_cast<double>(diagnostics.projectedNewtonFallbacks);
    maxCgError
        = std::max(maxCgError, diagnostics.projectedNewtonIterativeMaxError);
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxHessianStorageBytes = std::max(
        maxHessianStorageBytes, diagnostics.projectedNewtonHessianStorageBytes);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  const auto steps = static_cast<double>(state.iterations());
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["tetrahedra"] = static_cast<double>(fixture.tetrahedronCount);
  state.counters["total_mass"] = fixture.totalMass;
  state.counters["newton_iters_per_step"] = totalNewtonIterations / steps;
  state.counters["cg_iters_per_step"] = totalCgIterations / steps;
  state.counters["matrix_free_solves_per_step"] = totalMatrixFreeSolves / steps;
  // Steepest-descent fallbacks per step: the key convergence-regression signal
  // when a preconditioner cannot carry the stiff-contact solve within the cap.
  state.counters["fallbacks_per_step"] = totalFallbacks / steps;
  state.counters["cg_max_error"] = maxCgError;
  state.counters["hessian_nonzeros"] = static_cast<double>(maxHessianNonZeros);
  state.counters["hessian_storage_bytes"]
      = static_cast<double>(maxHessianStorageBytes);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

void BM_DeformableContactCubeCgStep(benchmark::State& state)
{
  BM_DeformableContactCubeStep(
      state, /*useIterativeSolver=*/true, /*useMatrixFreeSolver=*/false);
}

void BM_DeformableContactCubeMatrixFreeCgStep(benchmark::State& state)
{
  BM_DeformableContactCubeStep(
      state, /*useIterativeSolver=*/false, /*useMatrixFreeSolver=*/true);
}

//==============================================================================
void BM_DeformableGridStage(benchmark::State& state)
{
  const auto columns = static_cast<int>(state.range(0));
  const auto rows = static_cast<int>(state.range(1));
  const bool withGround = state.range(2) != 0;
  DeformableGridWorld fixture(columns, rows, withGround);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1).z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["tetrahedra"] = 0.0;
  state.counters["surface_triangles"] = 0.0;
  state.counters["fixed_nodes"] = 2.0;
  state.counters["ground"] = withGround ? 1.0 : 0.0;
  state.counters["contact_constraints"] = 0.0;
  state.counters["objective_evals"]
      = static_cast<double>(stats.objectiveEvaluations);
  state.counters["solver_iterations"]
      = static_cast<double>(stats.solverIterations);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.counters["initial_projections"]
      = static_cast<double>(stats.initialProjectionCount);
  state.counters["projected_newton_steps"]
      = static_cast<double>(stats.projectedNewtonSteps);
  state.counters["projected_newton_fallbacks"]
      = static_cast<double>(stats.projectedNewtonFallbacks);
  state.counters["newton_symbolic_factorizations"]
      = static_cast<double>(stats.projectedNewtonSymbolicFactorizations);
  state.counters["newton_numeric_factorizations"]
      = static_cast<double>(stats.projectedNewtonNumericFactorizations);
  state.counters["final_gradient_residual"] = stats.finalGradientResidualNorm;
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableDrapeStage(benchmark::State& state)
{
  const auto columns = static_cast<int>(state.range(0));
  const auto rows = static_cast<int>(state.range(1));
  // range(2) is the friction coefficient in tenths (0 = frictionless).
  const double frictionCoefficient = static_cast<double>(state.range(2)) / 10.0;
  DeformableDrapeWorld fixture(columns, rows, frictionCoefficient);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1).z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["ground_barriers"]
      = static_cast<double>(stats.staticGroundBarrierCount);
  state.counters["solver_iterations"]
      = static_cast<double>(stats.solverIterations);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["projected_newton_steps"]
      = static_cast<double>(stats.projectedNewtonSteps);
  state.counters["projected_newton_fallbacks"]
      = static_cast<double>(stats.projectedNewtonFallbacks);
  state.counters["newton_symbolic_factorizations"]
      = static_cast<double>(stats.projectedNewtonSymbolicFactorizations);
  state.counters["newton_numeric_factorizations"]
      = static_cast<double>(stats.projectedNewtonNumericFactorizations);
  state.counters["final_gradient_residual"] = stats.finalGradientResidualNorm;
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableSurfaceContactStage(benchmark::State& state)
{
  const auto pairCount = static_cast<int>(state.range(0));
  const bool crossing = state.range(1) != 0;
  DeformableSurfaceContactWorld fixture(pairCount, crossing);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["surface_triangles"]
      = static_cast<double>(fixture.surfaceTriangleCount);
  state.counters["crossing"] = crossing ? 1.0 : 0.0;
  state.counters["candidate_builds"]
      = static_cast<double>(stats.surfaceContactCandidateBuilds);
  state.counters["candidate_pair_capacity"]
      = static_cast<double>(stats.surfaceContactCandidatePairCapacity);
  state.counters["candidate_rejected_pairs"]
      = static_cast<double>(stats.surfaceContactCandidateRejectedPairs);
  state.counters["pt_candidates"]
      = static_cast<double>(stats.surfaceContactPointTriangleCandidates);
  state.counters["ee_candidates"]
      = static_cast<double>(stats.surfaceContactEdgeEdgeCandidates);
  state.counters["ccd_pt_checks"]
      = static_cast<double>(stats.surfaceContactCcdPointTriangleChecks);
  state.counters["ccd_ee_checks"]
      = static_cast<double>(stats.surfaceContactCcdEdgeEdgeChecks);
  state.counters["ccd_hits"] = static_cast<double>(stats.surfaceContactCcdHits);
  state.counters["ccd_indeterminate"]
      = static_cast<double>(stats.surfaceContactCcdIndeterminateCount);
  state.counters["ccd_limited_steps"]
      = static_cast<double>(stats.surfaceContactCcdLimitedSteps);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableInterBodySurfaceContactStage(benchmark::State& state)
{
  const auto obstacleCount = static_cast<int>(state.range(0));
  const bool crossing = state.range(1) != 0;
  DeformableInterBodySurfaceContactWorld fixture(obstacleCount, crossing);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.movingBody.getPosition(fixture.movingBody.getNodeCount() - 1u)
            .z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["surface_triangles"]
      = static_cast<double>(fixture.surfaceTriangleCount);
  state.counters["obstacle_bodies"] = static_cast<double>(obstacleCount);
  state.counters["crossing"] = crossing ? 1.0 : 0.0;
  state.counters["candidate_builds"]
      = static_cast<double>(stats.interBodySurfaceContactCandidateBuilds);
  state.counters["candidate_pair_capacity"]
      = static_cast<double>(stats.interBodySurfaceContactCandidatePairCapacity);
  state.counters["candidate_rejected_pairs"] = static_cast<double>(
      stats.interBodySurfaceContactCandidateRejectedPairs);
  state.counters["pt_candidates"] = static_cast<double>(
      stats.interBodySurfaceContactPointTriangleCandidates);
  state.counters["ee_candidates"]
      = static_cast<double>(stats.interBodySurfaceContactEdgeEdgeCandidates);
  state.counters["ccd_pt_checks"] = static_cast<double>(
      stats.interBodySurfaceContactCcdPointTriangleChecks);
  state.counters["ccd_ee_checks"]
      = static_cast<double>(stats.interBodySurfaceContactCcdEdgeEdgeChecks);
  state.counters["ccd_hits"]
      = static_cast<double>(stats.interBodySurfaceContactCcdHits);
  state.counters["ccd_indeterminate"]
      = static_cast<double>(stats.interBodySurfaceContactCcdIndeterminateCount);
  state.counters["ccd_limited_steps"]
      = static_cast<double>(stats.interBodySurfaceContactCcdLimitedSteps);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableStaticGroundBarrierCcdStage(benchmark::State& state)
{
  const auto barrierCount = static_cast<int>(state.range(0));
  const bool crossing = state.range(1) != 0;
  DeformableStaticGroundBarrierCcdWorld fixture(barrierCount, crossing);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.movingBody.getPosition(fixture.nodeCount - 1u).z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["ground_barriers"]
      = static_cast<double>(stats.staticGroundBarrierCount);
  state.counters["crossing"] = crossing ? 1.0 : 0.0;
  state.counters["ground_ccd_node_checks"]
      = static_cast<double>(stats.staticGroundBarrierCcdNodeChecks);
  state.counters["ground_ccd_sample_checks"]
      = static_cast<double>(stats.staticGroundBarrierCcdSampleChecks);
  state.counters["ground_ccd_hits"]
      = static_cast<double>(stats.staticGroundBarrierCcdHits);
  state.counters["ground_ccd_limited_steps"]
      = static_cast<double>(stats.staticGroundBarrierCcdLimitedSteps);
  state.counters["ground_ccd_zero_steps"]
      = static_cast<double>(stats.staticGroundBarrierCcdZeroStepCount);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableStaticRigidSurfaceCcdStage(benchmark::State& state)
{
  const auto boxCount = static_cast<int>(state.range(0));
  const bool crossing = state.range(1) != 0;
  DeformableStaticRigidSurfaceCcdWorld fixture(boxCount, crossing);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.movingBody.getPosition(fixture.nodeCount - 1u).x());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["rigid_surface_boxes"]
      = static_cast<double>(stats.staticRigidSurfaceCcdBoxCount);
  state.counters["rigid_surface_triangles"]
      = static_cast<double>(stats.staticRigidSurfaceCcdTriangleCount);
  state.counters["rigid_surface_edges"]
      = static_cast<double>(stats.staticRigidSurfaceCcdEdgeCount);
  state.counters["crossing"] = crossing ? 1.0 : 0.0;
  state.counters["candidate_builds"]
      = static_cast<double>(stats.staticRigidSurfaceCcdCandidateBuilds);
  state.counters["candidate_pair_capacity"]
      = static_cast<double>(stats.staticRigidSurfaceCcdCandidatePairCapacity);
  state.counters["candidate_rejected_pairs"]
      = static_cast<double>(stats.staticRigidSurfaceCcdCandidateRejectedPairs);
  state.counters["pt_candidates"]
      = static_cast<double>(stats.staticRigidSurfaceCcdPointTriangleCandidates);
  state.counters["ee_candidates"]
      = static_cast<double>(stats.staticRigidSurfaceCcdEdgeEdgeCandidates);
  state.counters["ccd_pt_checks"]
      = static_cast<double>(stats.staticRigidSurfaceCcdPointTriangleChecks);
  state.counters["ccd_ee_checks"]
      = static_cast<double>(stats.staticRigidSurfaceCcdEdgeEdgeChecks);
  state.counters["ccd_hits"]
      = static_cast<double>(stats.staticRigidSurfaceCcdHits);
  state.counters["ccd_indeterminate"]
      = static_cast<double>(stats.staticRigidSurfaceCcdIndeterminateCount);
  state.counters["ccd_limited_steps"]
      = static_cast<double>(stats.staticRigidSurfaceCcdLimitedSteps);
  state.counters["ccd_zero_steps"]
      = static_cast<double>(stats.staticRigidSurfaceCcdZeroStepCount);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableMovingRigidSurfaceCcdStage(benchmark::State& state)
{
  const auto boxCount = static_cast<int>(state.range(0));
  const bool crossing = state.range(1) != 0;
  DeformableMovingRigidSurfaceCcdWorld fixture(boxCount, crossing);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.movingBody.getPosition(fixture.nodeCount - 1u).x());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["moving_surface_boxes"]
      = static_cast<double>(stats.movingRigidSurfaceCcdBoxCount);
  state.counters["moving_surface_samples"]
      = static_cast<double>(stats.movingRigidSurfaceCcdSampleCount);
  state.counters["moving_surface_triangles"]
      = static_cast<double>(stats.movingRigidSurfaceCcdTriangleCount);
  state.counters["moving_surface_edges"]
      = static_cast<double>(stats.movingRigidSurfaceCcdEdgeCount);
  state.counters["crossing"] = crossing ? 1.0 : 0.0;
  state.counters["candidate_builds"]
      = static_cast<double>(stats.movingRigidSurfaceCcdCandidateBuilds);
  state.counters["candidate_pair_capacity"]
      = static_cast<double>(stats.movingRigidSurfaceCcdCandidatePairCapacity);
  state.counters["candidate_rejected_pairs"]
      = static_cast<double>(stats.movingRigidSurfaceCcdCandidateRejectedPairs);
  state.counters["pt_candidates"]
      = static_cast<double>(stats.movingRigidSurfaceCcdPointTriangleCandidates);
  state.counters["ee_candidates"]
      = static_cast<double>(stats.movingRigidSurfaceCcdEdgeEdgeCandidates);
  state.counters["ccd_pt_checks"]
      = static_cast<double>(stats.movingRigidSurfaceCcdPointTriangleChecks);
  state.counters["ccd_ee_checks"]
      = static_cast<double>(stats.movingRigidSurfaceCcdEdgeEdgeChecks);
  state.counters["ccd_hits"]
      = static_cast<double>(stats.movingRigidSurfaceCcdHits);
  state.counters["ccd_indeterminate"]
      = static_cast<double>(stats.movingRigidSurfaceCcdIndeterminateCount);
  state.counters["ccd_limited_steps"]
      = static_cast<double>(stats.movingRigidSurfaceCcdLimitedSteps);
  state.counters["ccd_zero_steps"]
      = static_cast<double>(stats.movingRigidSurfaceCcdZeroStepCount);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableSelfContactBarrierStage(benchmark::State& state)
{
  const auto pairCount = static_cast<int>(state.range(0));
  DeformableSelfContactBarrierWorld fixture(pairCount);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1u).z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["barrier_candidate_builds"]
      = static_cast<double>(stats.selfContactBarrierCandidateBuilds);
  state.counters["barrier_active_contacts"]
      = static_cast<double>(stats.selfContactBarrierActiveContacts);
  state.counters["converged_active_contacts"]
      = static_cast<double>(stats.convergedActiveContactCount);
  // Peak single-iteration active contacts (the IPC Fig. 23 "max contacts per
  // step" axis) for the most recent step.
  state.counters["max_active_contacts"]
      = static_cast<double>(stats.maxActiveContactCount);
  state.counters["min_active_contact_distance"]
      = stats.minActiveContactDistance;
  state.counters["solver_iterations"]
      = static_cast<double>(stats.solverIterations);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableSceneLoad(benchmark::State& state)
{
  const auto scenePath = benchmarkScenePath();
  const auto options = makeBenchmarkSceneLoadOptions();

  for (auto _ : state) {
    sx::World world;
    const auto info = sx::io::loadDeformableScene(world, scenePath, options);
    auto nodeCount = info.bodies[0].nodeCount;
    benchmark::DoNotOptimize(nodeCount);
  }

  state.counters["scenes"] = 1.0;
  state.counters["nodes"] = 8.0;
  state.counters["tetrahedra"] = 6.0;
  state.counters["surface_triangles"] = 12.0;
  state.counters["spring_edges"] = 19.0;
  state.counters["dbc_regions"] = 1.0;
  state.counters["nbc_regions"] = 1.0;
  state.counters["unsupported_directives"] = 1.0;
  state.counters["contact_constraints"] = 0.0;
}

//==============================================================================
void BM_DeformableSceneReplay(benchmark::State& state)
{
  sx::World world;
  const auto info = sx::io::loadDeformableScene(
      world, benchmarkScenePath(), makeBenchmarkSceneLoadOptions());
  world.enterSimulationMode();

  for (auto _ : state) {
    world.step();
    benchmark::DoNotOptimize(world.getFrame());
  }

  const auto diagnostics = sx::io::collectDeformableSceneDiagnostics(world);
  state.counters["scenes"] = 1.0;
  state.counters["nodes"] = static_cast<double>(diagnostics.nodeCount);
  state.counters["tetrahedra"]
      = static_cast<double>(diagnostics.tetrahedronCount);
  state.counters["surface_triangles"]
      = static_cast<double>(diagnostics.surfaceTriangleCount);
  state.counters["spring_edges"]
      = static_cast<double>(info.bodies[0].body.getEdgeCount());
  state.counters["dbc_regions"]
      = static_cast<double>(diagnostics.dirichletConditionCount);
  state.counters["nbc_regions"]
      = static_cast<double>(diagnostics.neumannConditionCount);
  state.counters["unsupported_directives"]
      = static_cast<double>(info.warnings.size());
  state.counters["contact_constraints"] = 0.0;
  state.counters["frames"] = static_cast<double>(world.getFrame());
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * diagnostics.nodeCount));
}

} // namespace

BENCHMARK(BM_WorldStepWithoutDeformables)->Arg(0)->Arg(32);

BENCHMARK(BM_DeformableGridStep)
    ->Args({8, 4, 0})
    ->Args({16, 8, 0})
    ->Args({32, 16, 0})
    ->Args({16, 8, 1})
    ->Args({32, 16, 1});

BENCHMARK(BM_DeformableTetraMeshSetup)->Arg(1)->Arg(8)->Arg(32);

BENCHMARK(BM_DeformableTetraMeshStep)->Arg(1)->Arg(8)->Arg(32);

BENCHMARK(BM_DeformableFemBarStep)->Arg(2)->Arg(8)->Arg(24)->Arg(48);
BENCHMARK(BM_DeformableFcrBarStep)->Arg(2)->Arg(8)->Arg(24);
BENCHMARK(BM_DeformableCgBarStep)->Arg(2)->Arg(8)->Arg(24)->Arg(48);
BENCHMARK(BM_DeformableMatrixFreeCgBarStep)->Arg(2)->Arg(8)->Arg(24);
// Chunky 3D cube, direct vs iterative at matching cell counts: the direct
// solve's per-step time climbs super-linearly with the 3D fill-in while the
// sparse IC-CG and explicit matrix-free CG rows expose the solve-effort and
// sparse-Hessian footprint tradeoff.
BENCHMARK(BM_DeformableCube3dDirectStep)->Arg(4)->Arg(6)->Arg(8)->Arg(10);
BENCHMARK(BM_DeformableCube3dCgStep)->Arg(4)->Arg(6)->Arg(8)->Arg(10);
BENCHMARK(BM_DeformableCube3dMatrixFreeCgStep)->Arg(4)->Arg(6)->Arg(8);
// Contact-heavy crossover: same ground-contact cube under sparse IC-CG and
// matrix-free block-Jacobi CG at increasing resolutions (27/64/125/216 nodes).
BENCHMARK(BM_DeformableContactCubeCgStep)->Arg(2)->Arg(3)->Arg(4)->Arg(5);
BENCHMARK(BM_DeformableContactCubeMatrixFreeCgStep)
    ->Arg(2)
    ->Arg(3)
    ->Arg(4)
    ->Arg(5);

// The 32x16 (512-node) and 48x24 (1152-node) grids exceed the former 256-node
// dense-solve cap, so they exercise the sparse projected-Newton path; compare
// projected_newton_steps against projected_newton_fallbacks to confirm Newton
// stays engaged as the mesh scales.
BENCHMARK(BM_DeformableGridStage)
    ->Args({8, 4, 0})
    ->Args({16, 8, 0})
    ->Args({32, 16, 0})
    ->Args({48, 24, 0})
    ->Args({16, 8, 1})
    ->Args({32, 16, 1})
    ->Args({48, 24, 1});

// The deformable drape demo solver scenario: a mat draped
// over a raised box ground barrier onto the flat ground (two active barriers).
// The 18x16 (288-node) and 24x20 (480-node) mats exceed the former 256-node
// dense-solve cap and are solved on the sparse projected-Newton path.
// range(2) is the friction coefficient in tenths; the mu=0.5 row exercises the
// lagged smoothed Coulomb ground-friction path.
BENCHMARK(BM_DeformableDrapeStage)
    ->Args({18, 16, 0})
    ->Args({24, 20, 0})
    ->Args({18, 16, 5});

BENCHMARK(BM_DeformableSurfaceContactStage)
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({32, 1});

BENCHMARK(BM_DeformableInterBodySurfaceContactStage)
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({8, 1})
    ->Args({32, 1});

BENCHMARK(BM_DeformableStaticGroundBarrierCcdStage)
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({8, 1})
    ->Args({32, 1});

BENCHMARK(BM_DeformableStaticRigidSurfaceCcdStage)
    ->Args({0, 1})
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({8, 1})
    ->Args({32, 1});

BENCHMARK(BM_DeformableMovingRigidSurfaceCcdStage)
    ->Args({0, 1})
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({8, 1})
    ->Args({32, 1});

BENCHMARK(BM_DeformableSelfContactBarrierStage)->Arg(1)->Arg(8)->Arg(32);

BENCHMARK(BM_DeformableSceneLoad);

BENCHMARK(BM_DeformableSceneReplay);

BENCHMARK_MAIN();
