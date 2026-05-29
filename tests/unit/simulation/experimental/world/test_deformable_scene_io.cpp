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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/deformable_body.hpp>
#include <dart/simulation/experimental/io/deformable_scene_io.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <cstdint>

namespace sx = dart::simulation::experimental;
namespace sxio = dart::simulation::experimental::io;

namespace {

//==============================================================================
constexpr std::string_view kCubeMsh = R"msh($MeshFormat
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
constexpr std::string_view kSubdividedTetraMsh = R"msh($MeshFormat
4.1 0 8
$EndMeshFormat
$Nodes
1 5 1 5
3 0 0 5
1
2
3
4
5
0.000000e+00 0.000000e+00 0.000000e+00
1.000000e+00 0.000000e+00 0.000000e+00
0.000000e+00 1.000000e+00 0.000000e+00
0.000000e+00 0.000000e+00 1.000000e+00
2.500000e-01 2.500000e-01 2.500000e-01
$EndNodes
$Elements
1 4 1 4
3 0 4 4
1 5 2 3 4
2 1 5 3 4
3 1 2 5 4
4 1 2 3 5
$EndElements
$Surface
4
2 3 4
1 4 3
1 2 4
1 3 2
$EndSurface
)msh";

//==============================================================================
constexpr std::string_view kSingleTetraMsh = R"msh($MeshFormat
4.1 0 8
$EndMeshFormat
$Nodes
1 4 1 4
3 0 0 4
1
2
3
4
0.000000e+00 0.000000e+00 0.000000e+00
1.000000e+00 0.000000e+00 0.000000e+00
0.000000e+00 1.000000e+00 0.000000e+00
0.000000e+00 0.000000e+00 1.000000e+00
$EndNodes
$Elements
1 1 1 1
3 0 4 1
1 1 2 3 4
$EndElements
)msh";

//==============================================================================
class TempSceneDir
{
public:
  TempSceneDir()
  {
    const auto base = std::filesystem::temp_directory_path();
    m_path = base
             / ("dart_deformable_scene_io_"
                + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    std::filesystem::create_directories(m_path / "input" / "tetMeshes");
    writeFile(m_path / "input" / "tetMeshes" / "cube.msh", kCubeMsh);
  }

  ~TempSceneDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(m_path, ec);
  }

  [[nodiscard]] const std::filesystem::path& path() const
  {
    return m_path;
  }

  std::filesystem::path writeScene(
      std::string_view fileName, std::string_view contents)
  {
    const auto scenePath = m_path / fileName;
    writeFile(scenePath, contents);
    return scenePath;
  }

  std::filesystem::path writeMesh(
      std::string_view fileName, std::string_view contents)
  {
    const auto meshPath = m_path / "input" / "tetMeshes" / fileName;
    writeFile(meshPath, contents);
    return meshPath;
  }

private:
  static void writeFile(
      const std::filesystem::path& path, std::string_view contents)
  {
    std::ofstream output(path);
    ASSERT_TRUE(output);
    output << contents;
  }

  std::filesystem::path m_path;
};

//==============================================================================
void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance = 1e-12)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

//==============================================================================
std::string cubeMeshWithoutSurface()
{
  std::string mesh{kCubeMsh};
  const auto surfaceStart = mesh.find("$Surface");
  EXPECT_NE(surfaceStart, std::string::npos);
  if (surfaceStart == std::string::npos) {
    return mesh;
  }
  mesh.erase(surfaceStart);
  return mesh;
}

//==============================================================================
bool hasWarning(const sxio::DeformableSceneInfo& info, std::string_view needle)
{
  return std::ranges::any_of(info.warnings, [&](const std::string& warning) {
    return warning.find(needle) != std::string::npos;
  });
}

} // namespace

//==============================================================================
TEST(DeformableSceneIo, LoadsContactFreeCubeSceneWithReferenceCounts)
{
  TempSceneDir temp;
  const auto scenePath = temp.writeScene(
      "scene.txt",
      R"scene(
energy IPC
timeIntegration BE
density 12
stiffness 1000 0.25
time 0.5 0.1
shapes input 1
input/tetMeshes/cube.msh 0 0 0  0 0 0  1 1 1 material 6 100 0.2
selfFric 0.1
ground 0.1 0
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  const auto info = sxio::loadDeformableScene(world, scenePath, options);

  ASSERT_EQ(info.bodies.size(), 1u);
  EXPECT_EQ(info.bodies[0].nodeCount, 8u);
  EXPECT_EQ(info.bodies[0].tetrahedronCount, 6u);
  EXPECT_EQ(info.bodies[0].surfaceTriangleCount, 12u);
  EXPECT_TRUE(hasWarning(info, "energy"));
  EXPECT_TRUE(hasWarning(info, "timeIntegration"));
  EXPECT_TRUE(hasWarning(info, "selfFric"));
  EXPECT_TRUE(hasWarning(info, "ground"));

  const auto body = info.bodies[0].body;
  EXPECT_EQ(body.getEdgeCount(), 19u);
  EXPECT_EQ(body.getSurfaceTriangleCount(), 12u);
  EXPECT_EQ(body.getTetrahedronCount(), 6u);
  EXPECT_DOUBLE_EQ(body.getMaterialProperties().density, 6.0);
  EXPECT_NEAR(body.getMass(0), 1.5, 1e-12);

  expectVectorNear(world.getGravity(), Eigen::Vector3d(0.0, -9.80665, 0.0));

  const auto diagnostics = sxio::collectDeformableSceneDiagnostics(world);
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 8u);
  EXPECT_EQ(diagnostics.tetrahedronCount, 6u);
  EXPECT_EQ(diagnostics.surfaceTriangleCount, 12u);
  EXPECT_NEAR(diagnostics.totalMass, 6.0, 1e-12);

  std::ostringstream json;
  sxio::writeDeformableSceneDiagnosticsJson(json, diagnostics);
  EXPECT_NE(json.str().find("\"node_count\":8"), std::string::npos);
}

//==============================================================================
TEST(DeformableSceneIo, DerivesSurfaceTrianglesWhenMeshOmitsSurfaceSection)
{
  TempSceneDir temp;
  temp.writeMesh("cube_without_surface.msh", cubeMeshWithoutSurface());
  const auto scenePath = temp.writeScene(
      "scene.txt",
      R"scene(
time 0.5 0.1
shapes input 1
input/tetMeshes/cube_without_surface.msh 0 0 0  0 0 0  1 1 1
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  const auto info = sxio::loadDeformableScene(world, scenePath, options);
  const auto body = info.bodies[0].body;

  EXPECT_EQ(info.bodies[0].surfaceTriangleCount, 12u);
  EXPECT_EQ(body.getSurfaceTriangleCount(), 12u);

  const auto diagnostics = sxio::collectDeformableSceneDiagnostics(world);
  EXPECT_EQ(diagnostics.surfaceTriangleCount, 12u);
}

//==============================================================================
TEST(DeformableSceneIo, LoadsGmshParametricNodeBlocks)
{
  TempSceneDir temp;
  temp.writeMesh(
      "parametric_tet.msh",
      R"msh($MeshFormat
4.1 0 8
$EndMeshFormat
$Nodes
1 4 1 4
3 0 1 4
1
2
3
4
0.000000e+00 0.000000e+00 0.000000e+00  0.1 0.2 0.3
1.000000e+00 0.000000e+00 0.000000e+00  0.4 0.5 0.6
0.000000e+00 1.000000e+00 0.000000e+00  0.7 0.8 0.9
0.000000e+00 0.000000e+00 1.000000e+00  1.0 1.1 1.2
$EndNodes
$Elements
1 1 1 1
3 0 4 1
1 1 2 3 4
$EndElements
)msh");
  const auto scenePath = temp.writeScene(
      "parametric_scene.txt",
      R"scene(
turnOffGravity
time 0.1 0.1
shapes input 1
input/tetMeshes/parametric_tet.msh 0 0 0  0 0 0  1 1 1
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  const auto info = sxio::loadDeformableScene(world, scenePath, options);

  ASSERT_EQ(info.bodies.size(), 1u);
  EXPECT_EQ(info.bodies[0].nodeCount, 4u);
  EXPECT_EQ(info.bodies[0].tetrahedronCount, 1u);
  EXPECT_EQ(info.bodies[0].surfaceTriangleCount, 4u);
}

//==============================================================================
TEST(DeformableSceneIo, BoundarySelectionIgnoresInteriorTetraNodes)
{
  TempSceneDir temp;
  temp.writeMesh("subdivided_tet.msh", kSubdividedTetraMsh);
  const auto scenePath = temp.writeScene(
      "interior_boundary.txt",
      R"scene(
turnOffGravity
time 0.1 0.1
shapes input 1
input/tetMeshes/subdivided_tet.msh 0 0 0  0 0 0  1 1 1 DBC 0.2 0.2 0.2  0.3 0.3 0.3  1 0 0  0 0 0
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  const auto info = sxio::loadDeformableScene(world, scenePath, options);

  ASSERT_EQ(info.bodies.size(), 1u);
  EXPECT_EQ(info.bodies[0].nodeCount, 5u);
  EXPECT_EQ(info.bodies[0].surfaceTriangleCount, 4u);
  EXPECT_EQ(info.bodies[0].dirichletConditionCount, 0u);
}

//==============================================================================
TEST(DeformableSceneIo, LoadsUpstreamDirichletTutorialRecord)
{
  TempSceneDir temp;
  const auto scenePath = temp.writeScene(
      "2cubesFall_DBC.txt",
      R"scene(
shapes input 2
input/tetMeshes/cube.msh 0 3 0  0 0 0  1 1 1
input/tetMeshes/cube.msh 0 1 0  0 0 0  1 1 1  DBC -0.1 -0.1 -0.1  0.1 1.1 0.1  -0.2 0 -0.2  0 0 0  DBC 0.9 -0.1 0.9  1.1 1.1 1.1  0.2 0 0.2  0 0 0

selfFric 0.1

ground 0.1 0
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  const auto info = sxio::loadDeformableScene(world, scenePath, options);

  ASSERT_EQ(info.bodies.size(), 2u);
  EXPECT_EQ(info.bodies[0].dirichletConditionCount, 0u);
  EXPECT_EQ(info.bodies[1].dirichletConditionCount, 2u);
  EXPECT_TRUE(hasWarning(info, "selfFric"));
  EXPECT_TRUE(hasWarning(info, "ground"));
  expectVectorNear(world.getGravity(), Eigen::Vector3d(0.0, -9.80665, 0.0));

  const auto body = info.bodies[1].body;
  world.step();
  EXPECT_NEAR(body.getPosition(0).x(), -0.005, 1e-12);
  EXPECT_NEAR(body.getPosition(0).y(), 1.0, 1e-12);
  EXPECT_NEAR(body.getPosition(0).z(), -0.005, 1e-12);
  EXPECT_NEAR(body.getPosition(2).x(), 1.005, 1e-12);
  EXPECT_NEAR(body.getPosition(2).z(), 1.005, 1e-12);
}

//==============================================================================
TEST(DeformableSceneIo, AppliesScriptedDirichletBoundaryFromLocalBox)
{
  TempSceneDir temp;
  const auto scenePath = temp.writeScene(
      "dbc.txt",
      R"scene(
turnOffGravity
time 0.2 0.1
shapes input 1
input/tetMeshes/cube.msh 2 0 0  0 0 0  1 1 1 DBC -0.1 -0.1 -0.1  0.1 1.1 1.1  1 0 0  0 0 0
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  const auto info = sxio::loadDeformableScene(world, scenePath, options);
  const auto body = info.bodies[0].body;

  world.step();

  for (const auto node : {0u, 3u, 4u, 7u}) {
    EXPECT_NEAR(body.getPosition(node).x(), 2.1, 1e-12);
    EXPECT_NEAR(body.getVelocity(node).x(), 1.0, 1e-12);
  }

  const auto diagnostics = sxio::collectDeformableSceneDiagnostics(world);
  EXPECT_EQ(diagnostics.dirichletConditionCount, 1u);
}

//==============================================================================
TEST(DeformableSceneIo, AppliesGlobalBoundaryRangesAfterShapesBlock)
{
  TempSceneDir temp;
  temp.writeMesh("single_tet.msh", kSingleTetraMsh);
  const auto scenePath = temp.writeScene(
      "late_ranges.txt",
      R"scene(
turnOffGravity
density 48
time 0.3 0.1
shapes input 1
input/tetMeshes/single_tet.msh 0 0 0  0 0 0  1 1 1  DBC -0.1 -0.1 -0.1  0.1 0.1 0.1  1 0 0  0 0 0  NBC 0.9 -0.1 -0.1  1.1 0.1 0.1  4 0 0
DBCTimeRange 0.1 0.2
NBCTimeRange 0.1 0.2
)scene");

  sx::World world;
  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();
  options.addStructuralSprings = false;
  const auto info = sxio::loadDeformableScene(world, scenePath, options);

  const auto body = info.bodies[0].body;

  world.step();
  EXPECT_NEAR(body.getPosition(0).x(), 0.0, 1e-12);
  EXPECT_NEAR(body.getPosition(1).x(), 1.0, 1e-12);

  world.step();
  EXPECT_NEAR(body.getPosition(0).x(), 0.1, 1e-12);
  EXPECT_NEAR(body.getPosition(1).x(), 1.04, 1e-12);
}

//==============================================================================
TEST(DeformableSceneIo, NeumannLoadsAreAccelerationLikeAndHalfOpen)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.masses = {2.0};
  sx::DeformableNeumannBoundaryCondition condition;
  condition.nodes = {0};
  condition.acceleration = Eigen::Vector3d(4.0, 0.0, 0.0);
  condition.startTime = 0.0;
  condition.endTime = 0.2;
  options.neumannBoundaryConditions.push_back(condition);

  const auto body = world.addDeformableBody("load", options);

  world.step();
  EXPECT_NEAR(body.getPosition(0).x(), 0.04, 1e-12);
  EXPECT_NEAR(body.getVelocity(0).x(), 0.4, 1e-12);

  world.step();
  EXPECT_NEAR(body.getPosition(0).x(), 0.12, 1e-12);
  EXPECT_NEAR(body.getVelocity(0).x(), 0.8, 1e-12);

  world.step();
  EXPECT_NEAR(body.getPosition(0).x(), 0.20, 1e-12);
  EXPECT_NEAR(body.getVelocity(0).x(), 0.8, 1e-12);
}

//==============================================================================
TEST(DeformableSceneIo, RejectsOverlappingDirichletAndNeumannNodes)
{
  sx::World world;

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.dirichletBoundaryConditions.push_back(
      sx::DeformableDirichletBoundaryCondition{
          {0},
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          0.0,
          1.0});
  options.neumannBoundaryConditions.push_back(
      sx::DeformableNeumannBoundaryCondition{
          {0}, Eigen::Vector3d::UnitX(), 0.5, 2.0});

  EXPECT_THROW(
      world.addDeformableBody("conflict", options),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(DeformableSceneIo, RejectsInvalidBoundaryComponentState)
{
  sx::World world;

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  sx::DeformableDirichletBoundaryCondition condition;
  condition.nodes = {0};
  condition.linearVelocity = Eigen::Vector3d::UnitX();
  options.dirichletBoundaryConditions.push_back(condition);
  world.addDeformableBody("boundary", options);

  auto view
      = world.getRegistry().view<sx::comps::DeformableBoundaryConditions>();
  ASSERT_NE(view.begin(), view.end());
  auto& boundary
      = view.get<sx::comps::DeformableBoundaryConditions>(*view.begin());
  boundary.dirichlet[0].referencePositions.clear();

  EXPECT_THROW(world.step(), sx::InvalidArgumentException);
}

//==============================================================================
TEST(DeformableSceneIo, RestartContinuationMatchesUninterruptedReplay)
{
  TempSceneDir temp;
  const auto scenePath = temp.writeScene(
      "restart.txt",
      R"scene(
turnOffGravity
time 0.5 0.1
shapes input 1
input/tetMeshes/cube.msh 0 0 0  0 0 0  1 1 1 DBC -0.1 -0.1 -0.1  0.1 1.1 1.1  1 0 0  0 0 0
)scene");

  sxio::DeformableSceneLoadOptions options;
  options.assetRoot = temp.path();

  sx::World uninterrupted;
  const auto uninterruptedInfo
      = sxio::loadDeformableScene(uninterrupted, scenePath, options);
  uninterrupted.step(5);

  sx::World restarted;
  sxio::loadDeformableScene(restarted, scenePath, options);
  restarted.step(2);
  std::stringstream restartImage;
  sxio::saveDeformableSceneRestart(restarted, restartImage);

  sx::World continued;
  sxio::loadDeformableSceneRestart(continued, restartImage);
  continued.step(3);

  ASSERT_TRUE(continued.getDeformableBody("deformable_scene_0").has_value());
  const auto expected = uninterruptedInfo.bodies[0].body;
  const auto actual = *continued.getDeformableBody("deformable_scene_0");

  EXPECT_EQ(continued.getFrame(), uninterrupted.getFrame());
  EXPECT_DOUBLE_EQ(continued.getTime(), uninterrupted.getTime());
  ASSERT_EQ(actual.getNodeCount(), expected.getNodeCount());
  for (std::size_t i = 0; i < actual.getNodeCount(); ++i) {
    expectVectorNear(actual.getPosition(i), expected.getPosition(i));
    expectVectorNear(actual.getVelocity(i), expected.getVelocity(i));
  }
}

//==============================================================================
// Corpus replay-validation pattern: a tutorial-style scene (one DBC-anchored
// cube, one free cube falling under gravity) replayed for many frames follows a
// sane trajectory (the anchor stays put, the free body falls, mass conserved,
// all finite) and is reproducible across runs -- the deterministic per-scene
// invariant the upstream corpus rows require. This exercises the
// loader -> solver -> diagnostics replay pipeline on a DART-native scene; the
// full ipc-sim/IPC corpus port additionally needs the upstream assets vendored
// and the contact-capable solver.
TEST(DeformableSceneIo, ReplaysTutorialSceneDeterministicallyWithSaneTrajectory)
{
  static constexpr std::string_view kScene = R"scene(
time 0.5 0.01
shapes input 2
input/tetMeshes/cube.msh 0 0 0  0 0 0  1 1 1 DBC -0.2 -0.2 -0.2  1.2 1.2 1.2  0 0 0  0 0 0
input/tetMeshes/cube.msh 3 0 0  0 0 0  1 1 1
)scene";

  struct ReplayResult
  {
    sxio::DeformableSceneDiagnostics diagnostics;
    Eigen::Vector3d anchorInitial = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchorFinal = Eigen::Vector3d::Zero();
    Eigen::Vector3d freeInitial = Eigen::Vector3d::Zero();
    Eigen::Vector3d freeFinal = Eigen::Vector3d::Zero();
  };

  const auto replay = [&]() {
    TempSceneDir temp;
    const auto scenePath = temp.writeScene("tutorial_replay.txt", kScene);
    sx::World world;
    sxio::DeformableSceneLoadOptions options;
    options.assetRoot = temp.path();
    const auto info = sxio::loadDeformableScene(world, scenePath, options);
    EXPECT_EQ(info.bodies.size(), 2u);
    const auto anchorBody = info.bodies[0].body;
    const auto freeBody = info.bodies[1].body;

    ReplayResult result;
    result.anchorInitial = anchorBody.getPosition(0);
    result.freeInitial = freeBody.getPosition(0);
    for (int frame = 0; frame < 30; ++frame) {
      world.step();
    }
    result.anchorFinal = anchorBody.getPosition(0);
    result.freeFinal = freeBody.getPosition(0);
    result.diagnostics = sxio::collectDeformableSceneDiagnostics(world);
    return result;
  };

  const ReplayResult first = replay();
  const ReplayResult second = replay();

  // The DBC-anchored cube stays put throughout (scripted zero velocity).
  expectVectorNear(first.anchorFinal, first.anchorInitial, 1e-9);
  // The free cube falls under gravity (the loader uses Y-up, gravity along -Y).
  EXPECT_LT(first.freeFinal.y(), first.freeInitial.y() - 0.1);
  // Sane, finite aggregate diagnostics; the free body's fall shows up as a
  // nonzero maximum node displacement.
  EXPECT_EQ(first.diagnostics.bodyCount, 2u);
  EXPECT_GT(first.diagnostics.totalMass, 0.0);
  EXPECT_GT(first.diagnostics.maxDisplacement, 0.1);
  EXPECT_TRUE(std::isfinite(first.diagnostics.minZ));
  EXPECT_TRUE(std::isfinite(first.diagnostics.maxDisplacement));
  // Reproducible: a second identical replay yields the same trajectory.
  EXPECT_DOUBLE_EQ(first.diagnostics.minZ, second.diagnostics.minZ);
  EXPECT_DOUBLE_EQ(
      first.diagnostics.maxDisplacement, second.diagnostics.maxDisplacement);
  EXPECT_DOUBLE_EQ(first.diagnostics.totalMass, second.diagnostics.totalMass);
  expectVectorNear(first.freeFinal, second.freeFinal, 1e-12);
}
