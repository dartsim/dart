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

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/rigid_ipc_ccd.hpp>
#include <dart/simulation/io/detail/rigid_ipc_fixture.hpp>
#include <dart/simulation/world.hpp>

#include <dart/collision/native/narrow_phase/primitive_ccd.hpp>
#include <dart/collision/native/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string_view>
#include <vector>

#include <cstdint>

namespace {

namespace sx = dart::simulation;
namespace expio = dart::simulation::io::detail;
namespace expdetail = dart::simulation::detail;

constexpr double kRigidIpcReferenceToiTolerance = 1e-4;

[[nodiscard]] expio::RigidIpcFixture loadFixture(std::string_view source)
{
  std::istringstream input{std::string(source)};
  return expio::loadRigidIpcFixture(input);
}

[[nodiscard]] expio::RigidIpcFixture loadComparisonScript(
    std::string_view source)
{
  std::istringstream input{std::string(source)};
  return expio::loadRigidIpcComparisonScript(input);
}

[[nodiscard]] expio::RigidIpcCcdCase loadCcdCase(std::string_view source)
{
  std::istringstream input{std::string(source)};
  return expio::loadRigidIpcCcdCase(input);
}

void expectLoadedCcdCaseHit(
    std::string_view source,
    const expio::RigidIpcCcdCaseType expectedType,
    const double expectedTimeOfImpact)
{
  const expio::RigidIpcCcdCase ccdCase = loadCcdCase(source);
  EXPECT_FALSE(ccdCase.hasErrors());
  EXPECT_TRUE(ccdCase.diagnostics.empty());
  EXPECT_EQ(ccdCase.type, expectedType);

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  EXPECT_TRUE(expio::evaluateRigidIpcCcdCase(ccdCase, option, result));
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(
      result.timeOfImpact,
      expectedTimeOfImpact,
      std::max(option.tolerance, kRigidIpcReferenceToiTolerance));
}

[[nodiscard]] expio::RigidIpcCcdCase expectLoadedCcdCase(
    std::string_view source,
    const expio::RigidIpcCcdCaseType expectedType,
    const std::size_t expectedBodyAVertexCount,
    const std::size_t expectedBodyBVertexCount)
{
  const expio::RigidIpcCcdCase ccdCase = loadCcdCase(source);
  EXPECT_FALSE(ccdCase.hasErrors());
  EXPECT_TRUE(ccdCase.diagnostics.empty());
  EXPECT_EQ(ccdCase.type, expectedType);
  EXPECT_EQ(ccdCase.bodyA.vertices.size(), expectedBodyAVertexCount);
  EXPECT_EQ(ccdCase.bodyB.vertices.size(), expectedBodyBVertexCount);
  return ccdCase;
}

void expectLoadedCcdCaseMiss(
    std::string_view source,
    const expio::RigidIpcCcdCaseType expectedType,
    const std::size_t expectedBodyAVertexCount,
    const std::size_t expectedBodyBVertexCount)
{
  const expio::RigidIpcCcdCase ccdCase = expectLoadedCcdCase(
      source, expectedType, expectedBodyAVertexCount, expectedBodyBVertexCount);

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  EXPECT_FALSE(expio::evaluateRigidIpcCcdCase(ccdCase, option, result));
  EXPECT_FALSE(result.isHit());
}

[[nodiscard]] bool hasDiagnosticContaining(
    const expio::RigidIpcFixture& fixture, std::string_view text)
{
  return std::any_of(
      fixture.diagnostics.begin(),
      fixture.diagnostics.end(),
      [&](const expio::RigidIpcFixtureDiagnostic& diagnostic) {
        return diagnostic.message.find(text) != std::string::npos
               || diagnostic.path.find(text) != std::string::npos;
      });
}

void writeTextFile(const std::filesystem::path& path, std::string_view text)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  ASSERT_TRUE(output.is_open()) << path;
  output << text;
}

void writeLittleEndianUInt16(std::ofstream& output, const std::uint16_t value)
{
  const std::array<char, 2> bytes{
      static_cast<char>(value & 0xffu),
      static_cast<char>((value >> 8u) & 0xffu)};
  output.write(bytes.data(), bytes.size());
}

void writeLittleEndianUInt32(std::ofstream& output, const std::uint32_t value)
{
  const std::array<char, 4> bytes{
      static_cast<char>(value & 0xffu),
      static_cast<char>((value >> 8u) & 0xffu),
      static_cast<char>((value >> 16u) & 0xffu),
      static_cast<char>((value >> 24u) & 0xffu)};
  output.write(bytes.data(), bytes.size());
}

void writeLittleEndianFloat(std::ofstream& output, const float value)
{
  writeLittleEndianUInt32(output, std::bit_cast<std::uint32_t>(value));
}

void writeBigEndianUInt32(std::ofstream& output, const std::uint32_t value)
{
  const std::array<char, 4> bytes{
      static_cast<char>((value >> 24u) & 0xffu),
      static_cast<char>((value >> 16u) & 0xffu),
      static_cast<char>((value >> 8u) & 0xffu),
      static_cast<char>(value & 0xffu)};
  output.write(bytes.data(), bytes.size());
}

void writeBigEndianUInt64(std::ofstream& output, const std::uint64_t value)
{
  for (int shift = 56; shift >= 0; shift -= 8) {
    const char byte = static_cast<char>((value >> shift) & 0xffu);
    output.write(&byte, 1);
  }
}

void writeBigEndianDouble(std::ofstream& output, const double value)
{
  writeBigEndianUInt64(output, std::bit_cast<std::uint64_t>(value));
}

void writeBinaryStl(
    const std::filesystem::path& path,
    const std::vector<std::array<Eigen::Vector3f, 3>>& triangles)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  ASSERT_TRUE(output.is_open()) << path;

  std::array<char, 80> header{};
  const std::string_view label = "DART rigid IPC fixture test STL";
  std::copy(label.begin(), label.end(), header.begin());
  output.write(header.data(), header.size());
  writeLittleEndianUInt32(output, static_cast<std::uint32_t>(triangles.size()));

  for (const auto& triangle : triangles) {
    const Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    for (Eigen::Index i = 0; i < normal.size(); ++i) {
      writeLittleEndianFloat(output, normal[i]);
    }
    for (const Eigen::Vector3f& vertex : triangle) {
      for (Eigen::Index i = 0; i < vertex.size(); ++i) {
        writeLittleEndianFloat(output, vertex[i]);
      }
    }
    writeLittleEndianUInt16(output, 0u);
  }
}

void writeBinaryVtkTetraSurface(const std::filesystem::path& path)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  ASSERT_TRUE(output.is_open()) << path;

  output << "# vtk DataFile Version 4.2\n";
  output << "DART rigid IPC fixture test VTK\n";
  output << "BINARY\n";
  output << "DATASET UNSTRUCTURED_GRID\n";
  output << "POINTS 4 double\n";
  const std::array<Eigen::Vector3d, 4> vertices{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0)};
  // The first binary payload byte is 0x09 (ASCII tab) to verify that the VTK
  // loader consumes only the text delimiter before the binary block.
  writeBigEndianUInt64(output, 0x0900000000000000ull);
  for (Eigen::Index i = 1; i < vertices.front().size(); ++i) {
    writeBigEndianDouble(output, vertices.front()[i]);
  }
  for (std::size_t vertexIndex = 1u; vertexIndex < vertices.size();
       ++vertexIndex) {
    const Eigen::Vector3d& vertex = vertices[vertexIndex];
    for (Eigen::Index i = 0; i < vertex.size(); ++i) {
      writeBigEndianDouble(output, vertex[i]);
    }
  }

  output << "\nCELLS 4 16\n";
  const std::array<Eigen::Vector3i, 4> triangles{
      Eigen::Vector3i(0, 2, 1),
      Eigen::Vector3i(0, 1, 3),
      Eigen::Vector3i(1, 2, 3),
      Eigen::Vector3i(2, 0, 3)};
  for (const Eigen::Vector3i& triangle : triangles) {
    writeBigEndianUInt32(output, 3u);
    for (Eigen::Index i = 0; i < triangle.size(); ++i) {
      writeBigEndianUInt32(output, static_cast<std::uint32_t>(triangle[i]));
    }
  }

  output << "\nCELL_TYPES 4\n";
  for (int i = 0; i < 4; ++i) {
    writeBigEndianUInt32(output, 5u);
  }
}

void writeRigidIpcTetraMsh(const std::filesystem::path& path)
{
  writeTextFile(
      path,
      R"msh($MeshFormat
4 0 8
$EndMeshFormat
$Entities
0 0 0 1
0 0 0 0 1 1 1 0 0
$EndEntities
$Nodes
1 4
0 3 0 4
1 0 0 0
2 1 0 0
3 0 1 0
4 0 0 1
$EndNodes
$Elements
1 1
0 3 4 1
1 1 3 4 2
$EndElements
$Surface
4
3 2 1
4 3 1
3 4 2
2 4 1
$EndSurface
)msh");
}

void expectResidualNearZero(
    const Eigen::Vector3d& residual, const double tolerance = 1e-12)
{
  EXPECT_LE(residual.norm(), tolerance);
}

void expectIntervalCcdTimeOfImpact(
    const dart::collision::native::CcdPrimitiveResult& result,
    const double expectedTimeOfImpact)
{
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(
      result.timeOfImpact,
      expectedTimeOfImpact,
      kRigidIpcReferenceToiTolerance);
}

TEST(RigidIpcFixture, LoadsTunnelingFixtureSubset)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 1e-2,
  "max_time": 1.0,
  "distance_barrier_constraint": {
    "initial_barrier_activation_distance": 1e-2
  },
  "ipc_solver": {
    "velocity_conv_tol": 1e-5
  },
  "rigid_body_problem": {
    "coefficient_restitution": -1,
    "gravity": [0, 0, 0],
    "rigid_bodies": [{
      "mesh": "plane.obj",
      "position": [0, 0, 0],
      "rotation": [0, 0, 90],
      "is_dof_fixed": [true, true, true, true, true, true]
    }, {
      "mesh": "cube.obj",
      "position": [-5, 0, 0],
      "linear_velocity": [1e3, 0, 0],
      "rotation": [32, 247, 53]
    }]
  }
}
)json");

  EXPECT_FALSE(fixture.hasErrors());
  EXPECT_TRUE(fixture.diagnostics.empty());
  EXPECT_EQ(fixture.sceneType, "distance_barrier_rb_problem");
  EXPECT_DOUBLE_EQ(fixture.timeStep, 0.01);
  EXPECT_DOUBLE_EQ(fixture.maxTime, 1.0);
  ASSERT_TRUE(fixture.barrierActivationDistance.has_value());
  EXPECT_DOUBLE_EQ(*fixture.barrierActivationDistance, 0.01);
  ASSERT_TRUE(fixture.velocityConvergenceTolerance.has_value());
  EXPECT_DOUBLE_EQ(*fixture.velocityConvergenceTolerance, 1e-5);
  EXPECT_DOUBLE_EQ(fixture.coefficientRestitution, -1.0);
  ASSERT_EQ(fixture.bodies.size(), 2u);

  const expio::RigidIpcBodyRecord& plane = fixture.bodies[0];
  EXPECT_EQ(plane.meshPath, "plane.obj");
  EXPECT_TRUE(plane.rotationDegrees.isApprox(Eigen::Vector3d(0.0, 0.0, 90.0)));
  ASSERT_EQ(plane.fixedDofs.size(), 6u);
  EXPECT_TRUE(
      std::all_of(
          plane.fixedDofs.begin(), plane.fixedDofs.end(), [](bool fixed) {
            return fixed;
          }));

  const expio::RigidIpcBodyRecord& cube = fixture.bodies[1];
  EXPECT_EQ(cube.meshPath, "cube.obj");
  EXPECT_TRUE(cube.position.isApprox(Eigen::Vector3d(-5.0, 0.0, 0.0)));
  EXPECT_TRUE(cube.linearVelocity.isApprox(Eigen::Vector3d(1000.0, 0.0, 0.0)));
  EXPECT_TRUE(
      cube.rotationDegrees.isApprox(Eigen::Vector3d(32.0, 247.0, 53.0)));
}

TEST(RigidIpcFixture, LoadsComparisonScriptSubset)
{
  const expio::RigidIpcFixture fixture = loadComparisonScript(R"script(
energy NH
warmStart 0
time 5 0.01
turnOffGravity
shapes input 2
meshes/tetra.msh 0 1 0 0 0 0 1 1 1 material 1000 2e11 0.3 initVel 1 2 3 4 5 6 NBC -1 -1 -1 1 1 1 7 8 9
meshes/plane.obj 0 -1 0 0 0 0 2 3 4 material 2500 1e5 0.25 linearVelocity 0 0 0 angularVelocity 0 0 9
selfCollisionOn
selfFric 0.2
constraintSolver interiorPoint
dHat 0.001
epsv 0.0001
useAbsParameters
fricIterAmt 3
tol 1
0.01
)script");

  EXPECT_FALSE(fixture.hasErrors());
  EXPECT_TRUE(fixture.diagnostics.empty());
  EXPECT_EQ(fixture.sceneType, "ipc_comparison_script");
  EXPECT_EQ(fixture.solverName, "interiorPoint");
  ASSERT_TRUE(fixture.energyModel.has_value());
  EXPECT_EQ(*fixture.energyModel, "NH");
  EXPECT_DOUBLE_EQ(fixture.maxTime, 5.0);
  EXPECT_DOUBLE_EQ(fixture.timeStep, 0.01);
  EXPECT_TRUE(fixture.gravity.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(fixture.gravityDisabled.has_value());
  EXPECT_TRUE(*fixture.gravityDisabled);
  EXPECT_DOUBLE_EQ(fixture.coefficientFriction, 0.2);
  ASSERT_TRUE(fixture.barrierActivationDistance.has_value());
  EXPECT_DOUBLE_EQ(*fixture.barrierActivationDistance, 0.001);
  ASSERT_TRUE(fixture.staticFrictionSpeedBound.has_value());
  EXPECT_DOUBLE_EQ(*fixture.staticFrictionSpeedBound, 0.0001);
  ASSERT_TRUE(fixture.velocityConvergenceTolerance.has_value());
  EXPECT_DOUBLE_EQ(*fixture.velocityConvergenceTolerance, 0.01);
  ASSERT_TRUE(fixture.velocityConvergenceToleranceIsAbsolute.has_value());
  EXPECT_TRUE(*fixture.velocityConvergenceToleranceIsAbsolute);
  ASSERT_TRUE(fixture.frictionIterations.has_value());
  EXPECT_EQ(*fixture.frictionIterations, 3);
  ASSERT_TRUE(fixture.warmStartEnabled.has_value());
  EXPECT_FALSE(*fixture.warmStartEnabled);
  ASSERT_TRUE(fixture.selfCollisionEnabled.has_value());
  EXPECT_TRUE(*fixture.selfCollisionEnabled);
  ASSERT_EQ(fixture.bodies.size(), 2u);

  const expio::RigidIpcBodyRecord& dynamicBody = fixture.bodies[0];
  EXPECT_EQ(dynamicBody.meshPath, "meshes/tetra.msh");
  EXPECT_TRUE(dynamicBody.position.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_TRUE(dynamicBody.rotationDegrees.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(dynamicBody.scale.isApprox(Eigen::Vector3d::Ones()));
  ASSERT_TRUE(dynamicBody.density.has_value());
  EXPECT_DOUBLE_EQ(*dynamicBody.density, 1000.0);
  ASSERT_TRUE(dynamicBody.youngModulus.has_value());
  EXPECT_DOUBLE_EQ(*dynamicBody.youngModulus, 2e11);
  ASSERT_TRUE(dynamicBody.poissonsRatio.has_value());
  EXPECT_DOUBLE_EQ(*dynamicBody.poissonsRatio, 0.3);
  EXPECT_TRUE(
      dynamicBody.linearVelocity.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      dynamicBody.angularVelocity.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  EXPECT_TRUE(dynamicBody.force.isApprox(Eigen::Vector3d(7.0, 8.0, 9.0)));
  EXPECT_EQ(dynamicBody.mode, expio::RigidIpcBodyMode::Dynamic);

  const expio::RigidIpcBodyRecord& prescribedBody = fixture.bodies[1];
  EXPECT_EQ(prescribedBody.meshPath, "meshes/plane.obj");
  EXPECT_TRUE(
      prescribedBody.position.isApprox(Eigen::Vector3d(0.0, -1.0, 0.0)));
  EXPECT_TRUE(prescribedBody.scale.isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
  ASSERT_TRUE(prescribedBody.density.has_value());
  EXPECT_DOUBLE_EQ(*prescribedBody.density, 2500.0);
  ASSERT_TRUE(prescribedBody.youngModulus.has_value());
  EXPECT_DOUBLE_EQ(*prescribedBody.youngModulus, 1e5);
  ASSERT_TRUE(prescribedBody.poissonsRatio.has_value());
  EXPECT_DOUBLE_EQ(*prescribedBody.poissonsRatio, 0.25);
  EXPECT_TRUE(prescribedBody.linearVelocity.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(
      prescribedBody.angularVelocity.isApprox(Eigen::Vector3d(0.0, 0.0, 9.0)));
  EXPECT_EQ(prescribedBody.mode, expio::RigidIpcBodyMode::Kinematic);
}

TEST(RigidIpcFixture, AppliesFixtureMetadataToStageOptions)
{
  expio::RigidIpcFixture fixture = loadComparisonScript(R"script(
time 1 0.01
shapes input 1
mesh.obj 0 0 0 0 0 0 1 1 1
dHat 0.02
epsv 0.0003
useAbsParameters
fricIterAmt 4
tol 1
0.05
)script");

  sx::compute::RigidIpcContactStageOptions options;
  options.maxIterations = 7u;
  expio::applyRigidIpcFixtureStageOptions(fixture, options);

  EXPECT_EQ(options.maxIterations, 7u);
  EXPECT_DOUBLE_EQ(options.activationDistance, 0.02);
  EXPECT_DOUBLE_EQ(options.staticFrictionSpeedBound, 0.0003);
  EXPECT_EQ(options.frictionIterations, 4u);
  EXPECT_DOUBLE_EQ(options.frictionConvergenceTolerance, 0.05);

  fixture.barrierActivationDistance = -1.0;
  fixture.staticFrictionSpeedBound = -1.0;
  fixture.frictionIterations = -1;
  fixture.velocityConvergenceTolerance = 0.25;
  fixture.velocityConvergenceToleranceIsAbsolute = false;

  sx::compute::RigidIpcContactStageOptions guardedOptions;
  guardedOptions.activationDistance = 0.03;
  guardedOptions.staticFrictionSpeedBound = 0.0007;
  guardedOptions.frictionIterations = 3u;
  guardedOptions.frictionConvergenceTolerance = 0.125;
  expio::applyRigidIpcFixtureStageOptions(fixture, guardedOptions);

  EXPECT_DOUBLE_EQ(guardedOptions.activationDistance, 0.03);
  EXPECT_DOUBLE_EQ(guardedOptions.staticFrictionSpeedBound, 0.0007);
  EXPECT_EQ(guardedOptions.frictionIterations, 3u);
  EXPECT_DOUBLE_EQ(guardedOptions.frictionConvergenceTolerance, 0.125);
}

TEST(RigidIpcFixture, ParsesFrictionAndScalarFixedDofs)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "solver": "ipc_solver",
  "timestep": 0.025,
  "max_time": 25,
  "distance_barrier_constraint": {
    "initial_barrier_activation_distance": 1e-3
  },
  "ipc_solver": {
    "velocity_conv_tol": 1e-4
  },
  "friction_constraints": {
    "static_friction_speed_bound": 1e-5,
    "iterations": -1
  },
  "rigid_body_problem": {
    "coefficient_friction": 0.5,
    "gravity": [0, -9.8, 0],
    "rigid_bodies": [{
      "mesh": "cube.obj",
      "position": [0, 0, 0],
      "rotation": [0, 0, -26.565]
    }, {
      "mesh": "plane.obj",
      "is_dof_fixed": true,
      "position": [2, -1.565, 0],
      "rotation": [0, 0, -26.565],
      "scale": 1
    }]
  }
}
)json");

  EXPECT_FALSE(fixture.hasErrors());
  EXPECT_TRUE(fixture.diagnostics.empty());
  EXPECT_EQ(fixture.solverName, "ipc_solver");
  EXPECT_DOUBLE_EQ(fixture.coefficientFriction, 0.5);
  EXPECT_TRUE(fixture.gravity.isApprox(Eigen::Vector3d(0.0, -9.8, 0.0)));
  ASSERT_TRUE(fixture.staticFrictionSpeedBound.has_value());
  EXPECT_DOUBLE_EQ(*fixture.staticFrictionSpeedBound, 1e-5);
  ASSERT_TRUE(fixture.frictionIterations.has_value());
  EXPECT_EQ(*fixture.frictionIterations, -1);
  ASSERT_EQ(fixture.bodies.size(), 2u);

  const expio::RigidIpcBodyRecord& plane = fixture.bodies[1];
  EXPECT_TRUE(plane.scale.isApprox(Eigen::Vector3d::Ones()));
  ASSERT_EQ(plane.fixedDofs.size(), 6u);
  EXPECT_TRUE(
      std::all_of(
          plane.fixedDofs.begin(), plane.fixedDofs.end(), [](bool fixed) {
            return fixed;
          }));
}

TEST(RigidIpcFixture, ParsesInlineGeometryAndReportsUnsupportedFields)
{
  const std::string_view source = R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "unexpected_root": true,
  "rigid_body_problem": {
    "rigid_bodies": [{
      "vertices": [[0, 0], [2, 0], [2, 3]],
      "edges": [[0, 1], [1, 2]],
      "polygons": [[[0, 0], [2, 0], [2, 3]]],
      "oriented": true,
      "linear_velocity": [0, 0],
      "angular_velocity": [0],
      "torque": [0, 0, 4],
      "surprise": 7
    }]
  }
}
)json";

  const expio::RigidIpcFixture fixture = loadFixture(source);
  EXPECT_FALSE(fixture.hasErrors());
  ASSERT_EQ(fixture.bodies.size(), 1u);
  EXPECT_TRUE(fixture.bodies[0].hasInlineGeometry);
  EXPECT_EQ(fixture.bodies[0].inlineVertices.size(), 3u);
  EXPECT_EQ(fixture.bodies[0].inlineEdges.size(), 2u);
  EXPECT_EQ(fixture.bodies[0].inlinePolygons.size(), 1u);
  ASSERT_TRUE(fixture.bodies[0].inlinePolygonsOriented.has_value());
  EXPECT_TRUE(*fixture.bodies[0].inlinePolygonsOriented);
  EXPECT_TRUE(
      fixture.bodies[0].torque.isApprox(Eigen::Vector3d(0.0, 0.0, 4.0)));
  EXPECT_TRUE(hasDiagnosticContaining(fixture, "unexpected_root"));
  EXPECT_TRUE(hasDiagnosticContaining(fixture, "surprise"));

  expio::RigidIpcFixtureLoadOptions options;
  options.strict = true;
  std::istringstream input{std::string(source)};
  EXPECT_THROW(
      expio::loadRigidIpcFixture(input, options),
      dart::simulation::InvalidArgumentException);
}

TEST(RigidIpcFixture, RejectsMissingRigidBodies)
{
  std::istringstream input(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "rigid_body_problem": {}
}
)json");

  EXPECT_THROW(
      expio::loadRigidIpcFixture(input),
      dart::simulation::InvalidArgumentException);
}

TEST(RigidIpcFixtureReplay, PopulatesWorldWithFixtureStateAndMetadata)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.01,
  "rigid_body_problem": {
    "coefficient_friction": 0.5,
    "coefficient_restitution": 0.25,
    "gravity": [0.0, 0.0, -9.81],
    "rigid_bodies": [{
      "mesh": "plane.obj",
      "position": [1.0, 2.0, 3.0],
      "rotation": [10.0, 20.0, 30.0],
      "scale": [2.0, 3.0, 4.0],
      "linear_velocity": [4.0, 5.0, 6.0],
      "angular_velocity": [0.0, 0.0, 180.0],
      "force": [7.0, 8.0, 9.0],
      "is_dof_fixed": true,
      "group_id": 42,
      "density": 500.0
    }, {
      "mesh": "cube.obj",
      "position": [-1.0, 0.0, 0.0],
      "angular_velocity": [90.0, 0.0, 0.0],
      "type": "kinematic",
      "kinematic_max_time": 1.5
    }]
  }
}
)json");

  dart::simulation::World world;
  expio::RigidIpcReplayOptions options;
  options.assetRoot = std::filesystem::path(::testing::TempDir())
                      / "rigid_ipc_fixture_replay_assets";
  std::filesystem::remove_all(options.assetRoot);
  writeTextFile(
      options.assetRoot / "plane.obj",
      R"obj(
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
)obj");
  writeTextFile(
      options.assetRoot / "cube.obj",
      R"obj(
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)obj");

  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture, options);

  EXPECT_DOUBLE_EQ(world.getTimeStep(), 0.01);
  EXPECT_TRUE(world.getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
  EXPECT_EQ(world.getRigidBodyCount(), 2u);
  ASSERT_EQ(state.bodies.size(), 2u);

  const auto body0 = world.getRigidBody("rigid_ipc_body_000");
  ASSERT_TRUE(body0.has_value());
  EXPECT_TRUE(body0->isStatic());
  EXPECT_TRUE(body0->getTransform().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      body0->getTransform().linear().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(
      body0->getLinearVelocity().isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  EXPECT_TRUE(body0->getAngularVelocity().isApprox(
      Eigen::Vector3d(0.0, 0.0, std::numbers::pi)));
  EXPECT_TRUE(body0->getForce().isApprox(Eigen::Vector3d(7.0, 8.0, 9.0)));
  EXPECT_DOUBLE_EQ(body0->getFriction(), 0.5);
  EXPECT_DOUBLE_EQ(body0->getRestitution(), 0.25);

  EXPECT_EQ(state.bodies[0].sourceBodyIndex, 0u);
  EXPECT_EQ(state.bodies[0].resolvedMeshPath, options.assetRoot / "plane.obj");
  EXPECT_TRUE(state.bodies[0].resolvedMeshExists);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_EQ(state.bodies[0].collisionMeshVertexCount, 3u);
  EXPECT_EQ(state.bodies[0].collisionMeshTriangleCount, 1u);
  EXPECT_EQ(state.bodies[0].collisionMeshStatus, "loaded");
  EXPECT_TRUE(
      state.bodies[0].geometryScale.isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
  EXPECT_TRUE(state.bodies[0].geometryRotationDegrees.isApprox(
      Eigen::Vector3d(10.0, 20.0, 30.0)));
  ASSERT_EQ(state.bodies[0].fixedDofs.size(), 6u);
  EXPECT_TRUE(
      std::all_of(
          state.bodies[0].fixedDofs.begin(),
          state.bodies[0].fixedDofs.end(),
          [](const bool fixed) { return fixed; }));
  ASSERT_TRUE(state.bodies[0].groupId.has_value());
  EXPECT_EQ(*state.bodies[0].groupId, 42);
  ASSERT_TRUE(state.bodies[0].density.has_value());
  EXPECT_DOUBLE_EQ(*state.bodies[0].density, 500.0);
  ASSERT_TRUE(body0->hasCollisionShape());
  ASSERT_TRUE(body0->getCollisionShape().has_value());
  EXPECT_EQ(
      body0->getCollisionShape()->type,
      dart::simulation::CollisionShapeType::Mesh);
  EXPECT_EQ(body0->getCollisionShape()->vertices.size(), 3u);
  EXPECT_EQ(body0->getCollisionShape()->triangles.size(), 1u);

  const auto body1 = world.getRigidBody("rigid_ipc_body_001");
  ASSERT_TRUE(body1.has_value());
  EXPECT_FALSE(body1->isStatic());
  EXPECT_TRUE(body1->isKinematic());
  EXPECT_TRUE(body1->getAngularVelocity().isApprox(
      Eigen::Vector3d(0.5 * std::numbers::pi, 0.0, 0.0)));
  EXPECT_EQ(state.bodies[1].mode, expio::RigidIpcBodyMode::Kinematic);
  ASSERT_TRUE(state.bodies[1].kinematicMaxTime.has_value());
  EXPECT_DOUBLE_EQ(*state.bodies[1].kinematicMaxTime, 1.5);
  EXPECT_TRUE(state.bodies[1].collisionMeshLoaded);
}

TEST(RigidIpcFixtureReplay, MeshBodiesParticipateInWorldCollision)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "rigid_body_problem": {
    "coefficient_friction": 0.5,
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "mesh": "tetra.obj",
      "is_dof_fixed": true
    }, {
      "mesh": "tetra.obj",
      "position": [0.1, 0.1, 0.1]
    }]
  }
}
)json");

  const std::filesystem::path assetRoot
      = std::filesystem::path(::testing::TempDir())
        / "rigid_ipc_fixture_collision_assets";
  std::filesystem::remove_all(assetRoot);
  writeTextFile(
      assetRoot / "tetra.obj",
      R"obj(
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)obj");

  dart::simulation::World world;
  expio::RigidIpcReplayOptions options;
  options.assetRoot = assetRoot;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture, options);

  ASSERT_EQ(state.bodies.size(), 2u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_TRUE(state.bodies[1].collisionMeshLoaded);
  EXPECT_FALSE(world.collide().empty());
}

TEST(RigidIpcFixtureReplay, RuntimeReplayUsesDefaultRigidBodyStep)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.1,
  "rigid_body_problem": {
    "gravity": [0.0, 0.0, -9.81],
    "rigid_bodies": [{
      "vertices": [[0, 0], [1, 0], [0, 1], [0, 0, 1]],
      "polygons": [
        [[0, 0], [0, 1], [1, 0]],
        [[0, 0], [1, 0], [0, 0, 1]],
        [[1, 0], [0, 1], [0, 0, 1]],
        [[0, 1], [0, 0], [0, 0, 1]]
      ],
      "position": [0.0, 0.0, 10.0],
      "linear_velocity": [0.5, 0.0, 0.0],
      "force": [1.0, 2.0, 3.0]
    }]
  }
}
)json");

  dart::simulation::World world;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture);

  ASSERT_EQ(state.bodies.size(), 1u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);

  const auto body = world.getRigidBody(state.bodies[0].bodyName);
  ASSERT_TRUE(body.has_value());

  world.enterSimulationMode();
  world.step();

  const double dt = 0.1;
  const Eigen::Vector3d expectedVelocity
      = Eigen::Vector3d(0.5, 0.0, 0.0)
        + (Eigen::Vector3d(1.0, 2.0, 3.0) + Eigen::Vector3d(0.0, 0.0, -9.81))
              * dt;
  const Eigen::Vector3d expectedPosition
      = Eigen::Vector3d(0.0, 0.0, 10.0) + expectedVelocity * dt;

  EXPECT_TRUE(body->getLinearVelocity().isApprox(expectedVelocity));
  EXPECT_TRUE(body->getTranslation().isApprox(expectedPosition));
  EXPECT_DOUBLE_EQ(world.getTime(), dt);
  EXPECT_EQ(world.getFrame(), 1u);
}

TEST(RigidIpcFixtureReplay, RuntimeReplayAdvancesParsedKinematicBody)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.1,
  "rigid_body_problem": {
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "type": "kinematic",
      "linear_velocity": [2.0, 0.0, 0.0],
      "angular_velocity": [0.0, 0.0, 90.0]
    }]
  }
}
)json");

  sx::World world;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture);

  ASSERT_EQ(state.bodies.size(), 1u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_EQ(state.bodies[0].mode, expio::RigidIpcBodyMode::Kinematic);

  const auto body = world.getRigidBody(state.bodies[0].bodyName);
  ASSERT_TRUE(body.has_value());
  EXPECT_FALSE(body->isStatic());
  EXPECT_TRUE(body->isKinematic());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(body->getTranslation().isApprox(Eigen::Vector3d(0.2, 0.0, 0.0)));
  EXPECT_TRUE(body->getTransform().linear().isApprox(
      Eigen::AngleAxisd(0.05 * std::numbers::pi, Eigen::Vector3d::UnitZ())
          .toRotationMatrix()));
  EXPECT_EQ(ipcStage.getLastStats().bodyCount, 1u);
  EXPECT_EQ(ipcStage.getLastStats().dynamicBodyCount, 0u);
}

TEST(RigidIpcFixtureReplay, RuntimeReplayStopsKinematicBodyAtMaxTime)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.1,
  "rigid_body_problem": {
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "type": "kinematic",
      "linear_velocity": [2.0, 0.0, 0.0],
      "angular_velocity": [0.0, 0.0, 90.0],
      "kinematic_max_time": 0.15
    }]
  }
}
)json");

  sx::World world;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture);

  ASSERT_EQ(state.bodies.size(), 1u);
  ASSERT_TRUE(state.bodies[0].kinematicMaxTime.has_value());
  EXPECT_DOUBLE_EQ(*state.bodies[0].kinematicMaxTime, 0.15);

  const auto body = world.getRigidBody(state.bodies[0].bodyName);
  ASSERT_TRUE(body.has_value());
  EXPECT_TRUE(body->isKinematic());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  world.step(executor, pipeline);
  EXPECT_TRUE(body->getTranslation().isApprox(Eigen::Vector3d(0.2, 0.0, 0.0)));
  EXPECT_TRUE(body->getTransform().linear().isApprox(
      Eigen::AngleAxisd(0.05 * std::numbers::pi, Eigen::Vector3d::UnitZ())
          .toRotationMatrix()));

  world.step(executor, pipeline);
  EXPECT_TRUE(body->getTranslation().isApprox(Eigen::Vector3d(0.3, 0.0, 0.0)));
  EXPECT_TRUE(body->getTransform().linear().isApprox(
      Eigen::AngleAxisd(0.075 * std::numbers::pi, Eigen::Vector3d::UnitZ())
          .toRotationMatrix()));

  world.step(executor, pipeline);
  EXPECT_TRUE(body->getTranslation().isApprox(Eigen::Vector3d(0.3, 0.0, 0.0)));
  EXPECT_TRUE(body->getTransform().linear().isApprox(
      Eigen::AngleAxisd(0.075 * std::numbers::pi, Eigen::Vector3d::UnitZ())
          .toRotationMatrix()));
}

TEST(RigidIpcFixtureReplay, RuntimeReplayCarriesFrictionIntoRigidIpcStage)
{
  struct SlideOutcome
  {
    sx::compute::RigidIpcSolverStats stats;
    double x = 0.0;
    double z = 0.0;
    double velocityX = 0.0;
  };

  const auto runSlide = [](const double friction) -> SlideOutcome {
    constexpr double initialHeight = 0.005;
    constexpr double tangentialSpeed = 1.0;

    std::ostringstream source;
    source << R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.05,
  "rigid_body_problem": {
    "coefficient_friction": )json"
           << friction << R"json(,
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "is_dof_fixed": true
    }, {
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "position": [0.0, 0.0, )json"
           << initialHeight << R"json(],
      "linear_velocity": [)json"
           << tangentialSpeed << R"json(, 0.0, 0.0]
    }]
  }
}
)json";

    const std::string fixtureSource = source.str();
    const expio::RigidIpcFixture fixture = loadFixture(fixtureSource);
    if (fixture.hasErrors()) {
      ADD_FAILURE() << "fixture failed to parse";
      return {};
    }

    sx::World world;
    const expio::RigidIpcReplayState state
        = expio::populateRigidIpcReplayWorld(world, fixture);
    if (state.bodies.size() != 2u) {
      ADD_FAILURE() << "expected two replay bodies";
      return {};
    }
    for (const expio::RigidIpcReplayBodyState& bodyState : state.bodies) {
      EXPECT_TRUE(bodyState.collisionMeshLoaded);
      EXPECT_EQ(bodyState.collisionMeshTriangleCount, 1u);
      const auto body = world.getRigidBody(bodyState.bodyName);
      if (!body.has_value()) {
        ADD_FAILURE() << "missing replay body " << bodyState.bodyName;
        return {};
      }
      EXPECT_DOUBLE_EQ(body->getFriction(), friction);
    }

    const auto body = world.getRigidBody(state.bodies[1].bodyName);
    if (!body.has_value()) {
      ADD_FAILURE() << "missing dynamic replay body";
      return {};
    }

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidIpcContactStage ipcStage;
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    world.step(executor, pipeline);

    return SlideOutcome{
        ipcStage.getLastStats(),
        body->getTranslation().x(),
        body->getTranslation().z(),
        body->getLinearVelocity().x()};
  };

  const SlideOutcome frictionless = runSlide(0.0);
  const SlideOutcome frictional = runSlide(1.0);

  EXPECT_TRUE(frictionless.stats.converged);
  EXPECT_TRUE(frictional.stats.converged);
  EXPECT_GT(frictionless.z, 0.005);
  EXPECT_GT(frictional.z, 0.005);

  EXPECT_EQ(frictionless.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(frictionless.stats.frictionIterations, 0u);
  EXPECT_GT(frictional.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(frictional.stats.frictionIterations, 1u);

  EXPECT_GT(frictionless.x, 0.0);
  EXPECT_GT(frictional.x, 0.0);
  EXPECT_LT(frictional.x, frictionless.x);
  EXPECT_GT(frictional.velocityX, 0.0);
  EXPECT_LT(frictional.velocityX, frictionless.velocityX);
}

TEST(RigidIpcFixtureReplay, RuntimeReplayCanUseParsedSolverSettings)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The rigid IPC solver-settings replay is too slow and "
         "condition-sensitive under coverage; normal Release/Debug CI runs the "
         "full regression.";
#endif

  constexpr double initialHeight = 0.005;
  constexpr double tangentialSpeed = 1.0;

  struct SolverSettingOutcome
  {
    sx::compute::RigidIpcSolverStats stats;
    double x = 0.0;
    double z = 0.0;
  };

  const auto runSlide
      = [=](const double activationDistance,
            const int frictionIterations,
            const double staticFrictionSpeedBound,
            const double frictionConvergenceTolerance) -> SolverSettingOutcome {
    std::ostringstream source;
    source << R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.05,
  "ipc_solver": {
    "velocity_conv_tol": )json"
           << frictionConvergenceTolerance << R"json(,
    "is_velocity_conv_tol_abs": true
  },
  "distance_barrier_constraint": {
    "initial_barrier_activation_distance": )json"
           << activationDistance << R"json(
  },
  "friction_constraints": {
    "static_friction_speed_bound": )json"
           << staticFrictionSpeedBound << R"json(,
    "iterations": )json"
           << frictionIterations << R"json(
  },
  "rigid_body_problem": {
    "coefficient_friction": 1.0,
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "is_dof_fixed": true
    }, {
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "position": [0.0, 0.0, )json"
           << initialHeight << R"json(],
      "linear_velocity": [)json"
           << tangentialSpeed << R"json(, 0.0, 0.0]
    }]
  }
}
)json";

    const std::string fixtureSource = source.str();
    const expio::RigidIpcFixture fixture = loadFixture(fixtureSource);
    if (fixture.hasErrors()) {
      ADD_FAILURE() << "fixture failed to parse";
      return {};
    }
    if (!fixture.barrierActivationDistance.has_value()) {
      ADD_FAILURE() << "missing parsed activation distance";
      return {};
    }
    if (!fixture.frictionIterations.has_value()
        || *fixture.frictionIterations < 0) {
      ADD_FAILURE() << "missing parsed non-negative friction iterations";
      return {};
    }
    if (!fixture.staticFrictionSpeedBound.has_value()
        || *fixture.staticFrictionSpeedBound < 0.0) {
      ADD_FAILURE() << "missing parsed non-negative static friction speed";
      return {};
    }
    if (!fixture.velocityConvergenceTolerance.has_value()
        || *fixture.velocityConvergenceTolerance < 0.0) {
      ADD_FAILURE() << "missing parsed non-negative convergence tolerance";
      return {};
    }

    sx::World world;
    const expio::RigidIpcReplayState state
        = expio::populateRigidIpcReplayWorld(world, fixture);
    if (state.bodies.size() != 2u) {
      ADD_FAILURE() << "expected two replay bodies";
      return {};
    }

    sx::compute::RigidIpcContactStageOptions stageOptions;
    stageOptions.maxIterations = 4u;
    expio::applyRigidIpcFixtureStageOptions(fixture, stageOptions);
    sx::compute::RigidIpcContactStage ipcStage(stageOptions);
    EXPECT_EQ(ipcStage.getMaxIterations(), 4u);
    EXPECT_DOUBLE_EQ(ipcStage.getActivationDistance(), activationDistance);
    EXPECT_EQ(
        ipcStage.getFrictionIterations(),
        static_cast<std::size_t>(frictionIterations));
    EXPECT_DOUBLE_EQ(
        ipcStage.getStaticFrictionSpeedBound(), staticFrictionSpeedBound);
    EXPECT_DOUBLE_EQ(
        ipcStage.getFrictionConvergenceTolerance(),
        frictionConvergenceTolerance);
    const sx::compute::RigidIpcContactStageOptions appliedOptions
        = ipcStage.getOptions();
    EXPECT_DOUBLE_EQ(appliedOptions.activationDistance, activationDistance);
    EXPECT_EQ(
        appliedOptions.frictionIterations,
        static_cast<std::size_t>(frictionIterations));
    EXPECT_DOUBLE_EQ(
        appliedOptions.staticFrictionSpeedBound, staticFrictionSpeedBound);
    EXPECT_DOUBLE_EQ(
        appliedOptions.frictionConvergenceTolerance,
        frictionConvergenceTolerance);

    sx::compute::SequentialExecutor executor;
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    world.step(executor, pipeline);

    const auto body = world.getRigidBody(state.bodies[1].bodyName);
    if (!body.has_value()) {
      ADD_FAILURE() << "missing dynamic replay body";
      return {};
    }

    return SolverSettingOutcome{
        ipcStage.getLastStats(),
        body->getTranslation().x(),
        body->getTranslation().z()};
  };

  const SolverSettingOutcome inactiveNarrowBand = runSlide(0.001, 1, 1e-3, 0.0);
  const SolverSettingOutcome activeFrictionDisabled
      = runSlide(0.01, 0, 1e-3, 0.0);
  const SolverSettingOutcome staticFrictionDisabled
      = runSlide(0.01, 3, 0.0, 0.0);
  const SolverSettingOutcome frictionToleranceEarlyStop
      = runSlide(0.01, 3, 1e-3, 1e100);

  EXPECT_TRUE(inactiveNarrowBand.stats.converged);
  EXPECT_EQ(inactiveNarrowBand.stats.activeConstraints, 0u);
  EXPECT_EQ(inactiveNarrowBand.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(inactiveNarrowBand.stats.frictionIterations, 0u);
  EXPECT_GT(inactiveNarrowBand.x, 0.0);
  EXPECT_NEAR(inactiveNarrowBand.z, initialHeight, 1e-12);

  EXPECT_TRUE(activeFrictionDisabled.stats.converged);
  EXPECT_GT(activeFrictionDisabled.stats.activeConstraints, 0u);
  EXPECT_EQ(activeFrictionDisabled.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(activeFrictionDisabled.stats.frictionIterations, 0u);
  EXPECT_GT(activeFrictionDisabled.z, initialHeight);

  EXPECT_TRUE(staticFrictionDisabled.stats.converged);
  EXPECT_GT(staticFrictionDisabled.stats.activeConstraints, 0u);
  EXPECT_EQ(staticFrictionDisabled.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(staticFrictionDisabled.stats.frictionIterations, 0u);
  EXPECT_GT(staticFrictionDisabled.z, initialHeight);

  EXPECT_GT(frictionToleranceEarlyStop.stats.activeConstraints, 0u);
  EXPECT_GT(frictionToleranceEarlyStop.stats.activeFrictionConstraints, 0u);
  EXPECT_FALSE(frictionToleranceEarlyStop.stats.failed);
  EXPECT_TRUE(frictionToleranceEarlyStop.stats.resultApplied);

  sx::compute::RigidIpcContactStageOptions invalidOptions;
  invalidOptions.activationDistance = -1.0;
  invalidOptions.staticFrictionSpeedBound = -1.0;
  invalidOptions.frictionConvergenceTolerance = -1.0;
  const sx::compute::RigidIpcContactStage invalidStage(invalidOptions);
  EXPECT_DOUBLE_EQ(invalidStage.getActivationDistance(), 1e-2);
  EXPECT_DOUBLE_EQ(invalidStage.getStaticFrictionSpeedBound(), 1e-3);
  EXPECT_DOUBLE_EQ(invalidStage.getFrictionConvergenceTolerance(), 0.0);
}

TEST(RigidIpcFixtureReplay, RuntimeReplayHelperAppliesStagePolicy)
{
  constexpr double initialHeight = 0.005;
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "timestep": 0.05,
  "distance_barrier_constraint": {
    "initial_barrier_activation_distance": 0.01
  },
  "friction_constraints": {
    "iterations": 0,
    "static_friction_speed_bound": 0.001
  },
  "rigid_body_problem": {
    "coefficient_friction": 1.0,
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "is_dof_fixed": true
    }, {
      "polygons": [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]],
      "position": [0.0, 0.0, 0.005],
      "linear_velocity": [1.0, 0.0, 0.0]
    }]
  }
}
)json");

  sx::World world;
  sx::compute::RigidIpcContactStageOptions stageOptions;
  stageOptions.activationDistance = 0.001;
  stageOptions.frictionIterations = 3u;

  sx::compute::RigidIpcSolverStats stats;
  const expio::RigidIpcReplayState state
      = expio::populateAndStepRigidIpcReplayWorld(
          world, fixture, {}, stageOptions, &stats);

  ASSERT_EQ(state.bodies.size(), 2u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_TRUE(state.bodies[1].collisionMeshLoaded);
  EXPECT_TRUE(stats.converged);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(stats.frictionIterations, 0u);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.05);
  EXPECT_EQ(world.getFrame(), 1u);

  const auto dynamicBody = world.getRigidBody(state.bodies[1].bodyName);
  ASSERT_TRUE(dynamicBody.has_value());
  EXPECT_GT(dynamicBody->getTranslation().x(), 0.0);
  EXPECT_GT(dynamicBody->getTranslation().z(), initialHeight);
}

TEST(RigidIpcFixtureReplay, ReplaysComparisonScriptMshScene)
{
  const std::filesystem::path assetRoot
      = std::filesystem::path(::testing::TempDir())
        / "rigid_ipc_comparison_script_assets";
  std::filesystem::remove_all(assetRoot);
  writeRigidIpcTetraMsh(assetRoot / "tetra.msh");
  writeTextFile(
      assetRoot / "scene.txt",
      R"script(
time 1 0.005
shapes input 2
tetra.msh 0 0 0 0 0 0 1 1 1 material 1000 2e11 0.3 linearVelocity 0 0 0
tetra.msh 0.1 0.1 0.1 0 0 0 1 1 1 material 1000 2e11 0.3 initVel 0 0 0 0 0 0
selfFric 0.4
)script");

  const expio::RigidIpcFixture fixture
      = expio::loadRigidIpcComparisonScript(assetRoot / "scene.txt");
  EXPECT_FALSE(fixture.hasErrors());
  ASSERT_EQ(fixture.bodies.size(), 2u);

  dart::simulation::World world;
  expio::RigidIpcReplayOptions options;
  options.assetRoot = assetRoot;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture, options);

  EXPECT_DOUBLE_EQ(world.getTimeStep(), 0.005);
  ASSERT_EQ(state.bodies.size(), 2u);
  EXPECT_EQ(state.bodies[0].mode, expio::RigidIpcBodyMode::Kinematic);
  EXPECT_EQ(state.bodies[1].mode, expio::RigidIpcBodyMode::Dynamic);
  for (const expio::RigidIpcReplayBodyState& bodyState : state.bodies) {
    EXPECT_TRUE(bodyState.collisionMeshLoaded);
    EXPECT_EQ(bodyState.collisionMeshVertexCount, 4u);
    EXPECT_EQ(bodyState.collisionMeshTriangleCount, 4u);
    EXPECT_EQ(bodyState.collisionMeshStatus, "loaded");
  }
  EXPECT_FALSE(world.collide().empty());
}

TEST(RigidIpcFixtureReplay, ReplaysPathLoadedComparisonScriptRelativeMeshes)
{
  const std::filesystem::path assetRoot
      = std::filesystem::path(::testing::TempDir())
        / "rigid_ipc_comparison_relative_assets";
  const std::filesystem::path scriptPath
      = assetRoot / "scripts" / "3D" / "unit-tests" / "scene.txt";
  std::filesystem::remove_all(assetRoot);
  writeRigidIpcTetraMsh(assetRoot / "meshes" / "tetra.msh");
  writeTextFile(
      scriptPath,
      R"script(
time 1 0.01
shapes input 2
../../../meshes/tetra.msh 0 0 0 0 0 0 1 1 1 material 1000 2e11 0.3 linearVelocity 0 0 0
../../../meshes/tetra.msh 0.1 0.1 0.1 0 0 0 1 1 1 material 1000 2e11 0.3 initVel 0 0 0 0 0 0
selfFric 0.1
)script");

  const expio::RigidIpcFixture fixture
      = expio::loadRigidIpcComparisonScript(scriptPath);
  EXPECT_EQ(fixture.sourceDirectory, scriptPath.parent_path());

  dart::simulation::World world;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture);

  ASSERT_EQ(state.bodies.size(), 2u);
  for (const expio::RigidIpcReplayBodyState& bodyState : state.bodies) {
    EXPECT_EQ(bodyState.sourceMeshPath, "../../../meshes/tetra.msh");
    EXPECT_EQ(bodyState.resolvedMeshPath, assetRoot / "meshes" / "tetra.msh");
    EXPECT_TRUE(bodyState.resolvedMeshExists);
    ASSERT_TRUE(bodyState.density.has_value());
    EXPECT_DOUBLE_EQ(*bodyState.density, 1000.0);
    ASSERT_TRUE(bodyState.youngModulus.has_value());
    EXPECT_DOUBLE_EQ(*bodyState.youngModulus, 2e11);
    ASSERT_TRUE(bodyState.poissonsRatio.has_value());
    EXPECT_DOUBLE_EQ(*bodyState.poissonsRatio, 0.3);
    EXPECT_TRUE(bodyState.collisionMeshLoaded);
    EXPECT_EQ(bodyState.collisionMeshTriangleCount, 4u);
  }
  EXPECT_FALSE(world.collide().empty());
}

TEST(RigidIpcFixtureReplay, LoadsInlinePolygonGeometry)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "rigid_body_problem": {
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "vertices": [[0, 0], [1, 0], [0, 1], [0, 0, 1]],
      "edges": [[0, 1], [1, 2], [2, 0]],
      "polygons": [
        [[0, 0], [0, 1], [1, 0]],
        [[0, 0], [1, 0], [0, 0, 1]],
        [[1, 0], [0, 1], [0, 0, 1]],
        [[0, 1], [0, 0], [0, 0, 1]]
      ],
      "oriented": true,
      "is_dof_fixed": true,
      "torque": [0, 0, 3]
    }, {
      "vertices": [[0, 0], [1, 0], [0, 1], [0, 0, 1]],
      "polygons": [
        [[0, 0], [0, 1], [1, 0]],
        [[0, 0], [1, 0], [0, 0, 1]],
        [[1, 0], [0, 1], [0, 0, 1]],
        [[0, 1], [0, 0], [0, 0, 1]]
      ],
      "position": [0.1, 0.1, 0.1]
    }]
  }
}
)json");

  dart::simulation::World world;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture);

  ASSERT_EQ(state.bodies.size(), 2u);
  EXPECT_TRUE(state.bodies[0].hasInlineGeometry);
  EXPECT_EQ(state.bodies[0].inlineGeometryVertexCount, 4u);
  EXPECT_EQ(state.bodies[0].inlineGeometryEdgeCount, 3u);
  EXPECT_EQ(state.bodies[0].inlineGeometryPolygonCount, 4u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_EQ(state.bodies[0].collisionMeshVertexCount, 12u);
  EXPECT_EQ(state.bodies[0].collisionMeshTriangleCount, 4u);
  EXPECT_EQ(state.bodies[0].collisionMeshStatus, "loaded inline geometry");
  EXPECT_TRUE(state.bodies[1].collisionMeshLoaded);
  EXPECT_EQ(state.bodies[1].collisionMeshStatus, "loaded inline geometry");

  const auto body0 = world.getRigidBody(state.bodies[0].bodyName);
  ASSERT_TRUE(body0.has_value());
  EXPECT_TRUE(body0->isStatic());
  EXPECT_TRUE(body0->getTorque().isApprox(Eigen::Vector3d(0.0, 0.0, 3.0)));
  ASSERT_TRUE(body0->getCollisionShape().has_value());
  EXPECT_EQ(
      body0->getCollisionShape()->type,
      dart::simulation::CollisionShapeType::Mesh);
  EXPECT_FALSE(world.collide().empty());
}

TEST(RigidIpcFixtureReplay, LoadsOffStlMshAndVtkMeshAssets)
{
  const expio::RigidIpcFixture fixture = loadFixture(R"json(
{
  "scene_type": "distance_barrier_rb_problem",
  "rigid_body_problem": {
    "gravity": [0.0, 0.0, 0.0],
    "rigid_bodies": [{
      "mesh": "tetra.off",
      "is_dof_fixed": true
    }, {
      "mesh": "tetra_binary.stl",
      "position": [0.1, 0.1, 0.1]
    }, {
      "mesh": "tetra_ascii.stl",
      "position": [0.2, 0.2, 0.2]
    }, {
      "mesh": "tetra.msh",
      "position": [0.3, 0.3, 0.3]
    }, {
      "mesh": "tetra.vtk",
      "position": [0.4, 0.4, 0.4]
    }, {
      "mesh": "tetra_ascii.vtk",
      "position": [0.5, 0.5, 0.5]
    }]
  }
}
)json");

  const std::filesystem::path assetRoot
      = std::filesystem::path(::testing::TempDir())
        / "rigid_ipc_fixture_mesh_format_assets";
  std::filesystem::remove_all(assetRoot);
  writeTextFile(
      assetRoot / "tetra.off",
      R"off(
OFF
4 4 0
0 0 0
1 0 0
0 1 0
0 0 1
3 0 2 1
3 0 1 3
3 1 2 3
3 2 0 3
)off");

  const std::vector<std::array<Eigen::Vector3f, 3>> tetraTriangles{
      std::array<Eigen::Vector3f, 3>{
          Eigen::Vector3f(0.0f, 0.0f, 0.0f),
          Eigen::Vector3f(0.0f, 1.0f, 0.0f),
          Eigen::Vector3f(1.0f, 0.0f, 0.0f)},
      std::array<Eigen::Vector3f, 3>{
          Eigen::Vector3f(0.0f, 0.0f, 0.0f),
          Eigen::Vector3f(1.0f, 0.0f, 0.0f),
          Eigen::Vector3f(0.0f, 0.0f, 1.0f)},
      std::array<Eigen::Vector3f, 3>{
          Eigen::Vector3f(1.0f, 0.0f, 0.0f),
          Eigen::Vector3f(0.0f, 1.0f, 0.0f),
          Eigen::Vector3f(0.0f, 0.0f, 1.0f)},
      std::array<Eigen::Vector3f, 3>{
          Eigen::Vector3f(0.0f, 1.0f, 0.0f),
          Eigen::Vector3f(0.0f, 0.0f, 0.0f),
          Eigen::Vector3f(0.0f, 0.0f, 1.0f)}};
  writeBinaryStl(assetRoot / "tetra_binary.stl", tetraTriangles);
  writeTextFile(
      assetRoot / "tetra_ascii.stl",
      R"stl(solid tetra
  facet normal 0 0 -1
    outer loop
      vertex 0 0 0
      vertex 0 1 0
      vertex 1 0 0
    endloop
  endfacet
  facet normal 0 -1 0
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 0 1
    endloop
  endfacet
  facet normal 1 1 1
    outer loop
      vertex 1 0 0
      vertex 0 1 0
      vertex 0 0 1
    endloop
  endfacet
  facet normal -1 0 0
    outer loop
      vertex 0 1 0
      vertex 0 0 0
      vertex 0 0 1
    endloop
  endfacet
endsolid tetra
)stl");
  writeBinaryVtkTetraSurface(assetRoot / "tetra.vtk");
  writeTextFile(
      assetRoot / "tetra.msh",
      R"msh($MeshFormat
4 0 8
$EndMeshFormat
$Entities
0 0 0 1
0 0 0 0 1 1 1 0 0
$EndEntities
$Nodes
1 4
0 3 0 4
1 0 0 0
2 1 0 0
3 0 1 0
4 0 0 1
$EndNodes
$Elements
1 1
0 3 4 1
1 1 3 4 2
$EndElements
$Surface
4
3 2 1
4 3 1
3 4 2
2 4 1
$EndSurface
)msh");
  writeTextFile(
      assetRoot / "tetra_ascii.vtk",
      R"vtk(# vtk DataFile Version 4.2
DART rigid IPC fixture test ASCII VTK
ASCII
DATASET UNSTRUCTURED_GRID
POINTS 4 double
0 0 0
1 0 0
0 1 0
0 0 1
CELLS 4 16
3 0 2 1
3 0 1 3
3 1 2 3
3 2 0 3
CELL_TYPES 4
5
5
5
5
)vtk");

  dart::simulation::World world;
  expio::RigidIpcReplayOptions options;
  options.assetRoot = assetRoot;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture, options);

  ASSERT_EQ(state.bodies.size(), 6u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_EQ(state.bodies[0].collisionMeshVertexCount, 4u);
  EXPECT_EQ(state.bodies[0].collisionMeshTriangleCount, 4u);
  EXPECT_EQ(state.bodies[0].collisionMeshStatus, "loaded");
  for (std::size_t i = 1; i < state.bodies.size(); ++i) {
    EXPECT_TRUE(state.bodies[i].collisionMeshLoaded);
    EXPECT_EQ(state.bodies[i].collisionMeshVertexCount, i >= 3u ? 4u : 12u);
    EXPECT_EQ(state.bodies[i].collisionMeshTriangleCount, 4u);
    EXPECT_EQ(state.bodies[i].collisionMeshStatus, "loaded");
  }

  for (std::size_t i = 0; i < state.bodies.size(); ++i) {
    const auto body = world.getRigidBody(state.bodies[i].bodyName);
    ASSERT_TRUE(body.has_value());
    ASSERT_TRUE(body->getCollisionShape().has_value());
    EXPECT_EQ(
        body->getCollisionShape()->type,
        dart::simulation::CollisionShapeType::Mesh);
  }
  EXPECT_FALSE(world.collide().empty());
}

TEST(RigidIpcCcdCase, LoadsFaceVertexCase)
{
  const expio::RigidIpcCcdCase ccdCase = loadCcdCase(R"json(
{
  "face": {
    "pose_t0": {
      "position": [-1.4456028966473393e-18, 4.625929269271485e-18, 3.700743415417188e-17],
      "rotation": [0.0, 1.2847228507565187, 0.0]
    },
    "pose_t1": {
      "position": [-1.4456028966473393e-18, 4.625929269271485e-18, 3.700743415417188e-17],
      "rotation": [0.0, 1.2847232462549631, 0.0]
    },
    "vertex0": [0.30618621784789724, -0.1767766952966369, -2.0],
    "vertex1": [0.1767766952966369, 0.30618621784789724, 2.0],
    "vertex2": [0.30618621784789724, -0.1767766952966369, 2.0]
  },
  "type": "fv",
  "vertex": {
    "pose_t0": {
      "position": [1.5032191088348958, 0.0031371238924182815, -0.01102932471453623],
      "rotation": [0.1100229671648746, -0.12286962030474363, -0.0068598726786883585]
    },
    "pose_t1": {
      "position": [1.502464630201717, 0.002401860480133274, -0.008444326734566802],
      "rotation": [0.08399953239222324, -0.09380760141963806, -0.005237325552368608]
    },
    "vertex": [-0.125, -0.125, 0.125]
  }
}
)json");

  EXPECT_FALSE(ccdCase.hasErrors());
  EXPECT_TRUE(ccdCase.diagnostics.empty());
  EXPECT_EQ(ccdCase.type, expio::RigidIpcCcdCaseType::FaceVertex);
  ASSERT_EQ(ccdCase.bodyA.vertices.size(), 3u);
  ASSERT_EQ(ccdCase.bodyB.vertices.size(), 1u);
  EXPECT_TRUE(ccdCase.bodyA.vertices[0].isApprox(
      Eigen::Vector3d(0.30618621784789724, -0.1767766952966369, -2.0)));
  EXPECT_TRUE(ccdCase.bodyB.vertices[0].isApprox(
      Eigen::Vector3d(-0.125, -0.125, 0.125)));
  EXPECT_TRUE(ccdCase.bodyB.poseT1.position.isApprox(
      Eigen::Vector3d(
          1.502464630201717, 0.002401860480133274, -0.008444326734566802)));
}

TEST(RigidIpcCcdCase, LoadsEdgeEdgeCase)
{
  const expio::RigidIpcCcdCase ccdCase = loadCcdCase(R"json(
{
  "edge0": {
    "pose_t0": {
      "position": [-1.4456028966473393e-18, 4.625929269271485e-18, 3.700743415417188e-17],
      "rotation": [0.0, 1.858006073626625, 0.0]
    },
    "pose_t1": {
      "position": [-1.4456028966473393e-18, 4.625929269271485e-18, 3.700743415417188e-17],
      "rotation": [0.0, 1.8580099461090607, 0.0]
    },
    "vertex0": [0.30618621784789724, -0.1767766952966369, -2.0],
    "vertex1": [0.30618621784789724, -0.1767766952966369, 2.0]
  },
  "edge1": {
    "pose_t0": {
      "position": [1.6152987419335303, -0.2261956159086164, -0.9495801406522546],
      "rotation": [0.2118369662746981, 0.26933045491938123, 0.14445906418961982]
    },
    "pose_t1": {
      "position": [1.6251226560504626, -0.22805327573239847, -0.943487705773452],
      "rotation": [0.19467042682211233, 0.27445700448823973, 0.08746526363891724]
    },
    "vertex0": [0.125, 0.125, 0.125],
    "vertex1": [0.125, -0.125, 0.125]
  },
  "type": "ee"
}
)json");

  EXPECT_FALSE(ccdCase.hasErrors());
  EXPECT_TRUE(ccdCase.diagnostics.empty());
  EXPECT_EQ(ccdCase.type, expio::RigidIpcCcdCaseType::EdgeEdge);
  ASSERT_EQ(ccdCase.bodyA.vertices.size(), 2u);
  ASSERT_EQ(ccdCase.bodyB.vertices.size(), 2u);
  EXPECT_TRUE(ccdCase.bodyA.poseT1.rotation.isApprox(
      Eigen::Vector3d(0.0, 1.8580099461090607, 0.0)));
  EXPECT_TRUE(ccdCase.bodyB.vertices[1].isApprox(
      Eigen::Vector3d(0.125, -0.125, 0.125)));
}

TEST(RigidIpcCcdCase, LoadsEdgeVertexCase)
{
  const expio::RigidIpcCcdCase ccdCase = loadCcdCase(R"json(
{
  "edge": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-2.0, 0.0, 0.0],
    "vertex1": [2.0, 0.0, 0.0]
  },
  "type": "ev",
  "vertex": {
    "pose_t0": {
      "position": [0.0, 0.5, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, -0.5, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex": [0.0, 0.0, 0.0]
  }
}
)json");

  EXPECT_FALSE(ccdCase.hasErrors());
  EXPECT_TRUE(ccdCase.diagnostics.empty());
  EXPECT_EQ(ccdCase.type, expio::RigidIpcCcdCaseType::EdgeVertex);
  ASSERT_EQ(ccdCase.bodyA.vertices.size(), 2u);
  ASSERT_EQ(ccdCase.bodyB.vertices.size(), 1u);
  EXPECT_TRUE(
      ccdCase.bodyA.vertices[0].isApprox(Eigen::Vector3d(-2.0, 0.0, 0.0)));
  EXPECT_TRUE(
      ccdCase.bodyB.poseT1.position.isApprox(Eigen::Vector3d(0.0, -0.5, 0.0)));
}

TEST(RigidIpcCcdCase, RejectsUnsupportedCaseType)
{
  std::istringstream input(R"json(
{
  "type": "vv"
}
)json");

  EXPECT_THROW(
      expio::loadRigidIpcCcdCase(input),
      dart::simulation::InvalidArgumentException);
}

TEST(RigidIpcCcdCase, EvaluatesLoadedFaceVertexRow)
{
  expectLoadedCcdCaseHit(
      R"json(
{
  "face": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-2.0, -2.0, 0.0],
    "vertex1": [2.0, -2.0, 0.0],
    "vertex2": [0.0, 2.0, 0.0]
  },
  "type": "fv",
  "vertex": {
    "pose_t0": {
      "position": [0.0, 0.0, 1.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, -1.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex": [0.0, 0.0, 0.0]
  }
}
)json",
      expio::RigidIpcCcdCaseType::FaceVertex,
      0.5);
}

TEST(RigidIpcCcdCase, EvaluatesLoadedEdgeEdgeRow)
{
  expectLoadedCcdCaseHit(
      R"json(
{
  "edge0": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-1.0, 0.0, 0.0],
    "vertex1": [1.0, 0.0, 0.0]
  },
  "edge1": {
    "pose_t0": {
      "position": [0.0, 1.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, -1.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [0.0, 0.0, -1.0],
    "vertex1": [0.0, 0.0, 1.0]
  },
  "type": "ee"
}
)json",
      expio::RigidIpcCcdCaseType::EdgeEdge,
      0.5);
}

TEST(RigidIpcCcdCase, EvaluatesLoadedEdgeVertexRow)
{
  expectLoadedCcdCaseHit(
      R"json(
{
  "edge": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-2.0, 0.0, 0.0],
    "vertex1": [2.0, 0.0, 0.0]
  },
  "type": "ev",
  "vertex": {
    "pose_t0": {
      "position": [0.0, 1.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, -1.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex": [0.0, 0.0, 0.0]
  }
}
)json",
      expio::RigidIpcCcdCaseType::EdgeVertex,
      0.5);
}

TEST(RigidIpcCcdCase, EvaluatesUpstreamStyleRigidToiRows)
{
  expectLoadedCcdCaseHit(
      R"json(
{
  "edge": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-2.0, 0.0, 0.0],
    "vertex1": [2.0, 0.0, 0.0]
  },
  "type": "ev",
  "vertex": {
    "pose_t0": {
      "position": [0.0, 0.5, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, -10.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex": [0.0, 0.0, 0.0]
  }
}
)json",
      expio::RigidIpcCcdCaseType::EdgeVertex,
      1.0 / 21.0);
  expectLoadedCcdCaseHit(
      R"json(
{
  "edge": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-2.0, 0.0, 0.0],
    "vertex1": [2.0, 0.0, 0.0]
  },
  "type": "ev",
  "vertex": {
    "pose_t0": {
      "position": [0.0, 0.5, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.5, 0.0],
      "rotation": [0.0, 0.0, 1.0471975511965976]
    },
    "vertex": [-1.0, 0.0, 0.0]
  }
}
)json",
      expio::RigidIpcCcdCaseType::EdgeVertex,
      0.5);
  expectLoadedCcdCaseHit(
      R"json(
{
  "edge0": {
    "pose_t0": {
      "position": [0.0, 1.0, 0.5],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, -2.0, 0.5],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-1.0, 0.0, 0.0],
    "vertex1": [1.0, 0.0, 0.0]
  },
  "edge1": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [0.0, 0.0, -1.0],
    "vertex1": [0.0, 0.0, 1.0]
  },
  "type": "ee"
}
)json",
      expio::RigidIpcCcdCaseType::EdgeEdge,
      1.0 / 3.0);
  expectLoadedCcdCaseHit(
      R"json(
{
  "face": {
    "pose_t0": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, 0.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex0": [-1.0, 0.0, 0.0],
    "vertex1": [1.0, 0.0, 0.0],
    "vertex2": [0.0, 1.0, 0.0]
  },
  "type": "fv",
  "vertex": {
    "pose_t0": {
      "position": [0.0, 0.0, 1.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "pose_t1": {
      "position": [0.0, 0.0, -2.0],
      "rotation": [0.0, 0.0, 0.0]
    },
    "vertex": [0.0, 0.5, 0.0]
  }
}
)json",
      expio::RigidIpcCcdCaseType::FaceVertex,
      1.0 / 3.0);
}

TEST(RigidIpcCcdCase, PointEdgeResidualMatchesExpectedToiRows)
{
  const expdetail::RigidIpcPose staticPose;
  const Eigen::Vector3d edgeA(-2.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB(2.0, 0.0, 0.0);
  const Eigen::Vector3d point(0.0, 0.0, 0.0);

  const expdetail::RigidIpcPose translatingPointPoseStart{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose translatingPointPoseEnd{
      Eigen::Vector3d(0.0, -10.0, 0.0), Eigen::Vector3d::Zero()};
  expectResidualNearZero(
      expdetail::rigidIpcPointEdgeResidual(
          point,
          translatingPointPoseStart,
          translatingPointPoseEnd,
          edgeA,
          edgeB,
          staticPose,
          staticPose,
          1.0 / 21.0,
          0.5));

  const expdetail::RigidIpcPose rotatingPointPoseStart{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose rotatingPointPoseEnd{
      Eigen::Vector3d(0.0, 0.5, 0.0),
      Eigen::Vector3d(0.0, 0.0, std::numbers::pi / 3.0)};
  constexpr double time = 0.5;
  const Eigen::Vector3d rotatingPoint = Eigen::Vector3d(-1.0, 0.0, 0.0);
  const Eigen::Vector3d contactPointWorld = expdetail::transformRigidIpcPoint(
      rotatingPoint, rotatingPointPoseStart, rotatingPointPoseEnd, time);
  const double alpha
      = (contactPointWorld.x() - edgeA.x()) / (edgeB.x() - edgeA.x());

  EXPECT_GE(alpha, 0.0);
  EXPECT_LE(alpha, 1.0);
  expectResidualNearZero(
      expdetail::rigidIpcPointEdgeResidual(
          rotatingPoint,
          rotatingPointPoseStart,
          rotatingPointPoseEnd,
          edgeA,
          edgeB,
          staticPose,
          staticPose,
          time,
          alpha));
}

TEST(RigidIpcCcdCase, EdgeEdgeResidualMatchesExpectedToiRow)
{
  const Eigen::Vector3d edgeA0(-1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB0(0.0, 0.0, -1.0);
  const Eigen::Vector3d edgeB1(0.0, 0.0, 1.0);
  const expdetail::RigidIpcPose movingEdgePoseStart{
      Eigen::Vector3d(0.0, 1.0, 0.5), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose movingEdgePoseEnd{
      Eigen::Vector3d(0.0, -2.0, 0.5), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose staticPose;

  expectResidualNearZero(
      expdetail::rigidIpcEdgeEdgeResidual(
          edgeA0,
          edgeA1,
          movingEdgePoseStart,
          movingEdgePoseEnd,
          edgeB0,
          edgeB1,
          staticPose,
          staticPose,
          1.0 / 3.0,
          0.5,
          0.75));
}

TEST(RigidIpcCcdCase, PointTriangleResidualMatchesExpectedToiRow)
{
  const Eigen::Vector3d triangleA(-1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const Eigen::Vector3d point(0.0, 0.5, 0.0);
  const expdetail::RigidIpcPose staticPose;
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, 0.0, -2.0), Eigen::Vector3d::Zero()};

  const double u = 0.25;
  const double v = 0.5;
  EXPECT_TRUE(expdetail::rigidIpcFaceVertexDomainContains(u, v));
  EXPECT_FALSE(expdetail::rigidIpcFaceVertexDomainContains(-1e-8, v));
  EXPECT_TRUE(expdetail::rigidIpcFaceVertexDomainContains(-1e-8, v, 1e-7));
  EXPECT_FALSE(expdetail::rigidIpcFaceVertexDomainContains(u, 0.8));

  expectResidualNearZero(
      expdetail::rigidIpcPointTriangleResidual(
          point,
          pointPoseStart,
          pointPoseEnd,
          triangleA,
          triangleB,
          triangleC,
          staticPose,
          staticPose,
          1.0 / 3.0,
          u,
          v));
}

TEST(RigidIpcCcdCase, PointEdgeIntervalSubdivisionFindsExpectedToiRows)
{
  const expdetail::RigidIpcPose staticPose;
  const Eigen::Vector3d edgeA(-2.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB(2.0, 0.0, 0.0);
  const Eigen::Vector3d point(0.0, 0.0, 0.0);
  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();

  dart::collision::native::CcdPrimitiveResult result;
  const expdetail::RigidIpcPose translatingPointPoseStart{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose translatingPointPoseEnd{
      Eigen::Vector3d(0.0, -10.0, 0.0), Eigen::Vector3d::Zero()};
  EXPECT_TRUE(
      expdetail::rigidIpcPointEdgeIntervalCcd(
          point,
          translatingPointPoseStart,
          translatingPointPoseEnd,
          edgeA,
          edgeB,
          staticPose,
          staticPose,
          option,
          result));
  expectIntervalCcdTimeOfImpact(result, 1.0 / 21.0);

  const expdetail::RigidIpcPose rotatingPointPoseStart{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose rotatingPointPoseEnd{
      Eigen::Vector3d(0.0, 0.5, 0.0),
      Eigen::Vector3d(0.0, 0.0, std::numbers::pi / 3.0)};
  EXPECT_TRUE(
      expdetail::rigidIpcPointEdgeIntervalCcd(
          Eigen::Vector3d(-1.0, 0.0, 0.0),
          rotatingPointPoseStart,
          rotatingPointPoseEnd,
          edgeA,
          edgeB,
          staticPose,
          staticPose,
          option,
          result));
  expectIntervalCcdTimeOfImpact(result, 0.5);
}

TEST(RigidIpcCcdCase, GeneratedPointEdgeLinearImpactsMatchExpectedToi)
{
  struct GeneratedImpact
  {
    std::string_view name;
    Eigen::Vector3d edgeA;
    Eigen::Vector3d edgeB;
    Eigen::Vector3d edgeVelocity;
    Eigen::Vector3d pointStart;
    double timeOfImpact;
    double alpha;
  };

  const auto impacts = std::to_array<GeneratedImpact>({
      GeneratedImpact{
          "static edge midpoint hit",
          Eigen::Vector3d(-2.0, 0.0, 0.0),
          Eigen::Vector3d(2.0, 0.0, 0.0),
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          0.5},
      GeneratedImpact{
          "translating edge interior hit",
          Eigen::Vector3d(-2.0, 0.0, 0.0),
          Eigen::Vector3d(2.0, 0.0, 0.0),
          Eigen::Vector3d(0.5, 0.25, 0.0),
          Eigen::Vector3d(-1.5, 1.0, 0.0),
          0.4,
          0.25},
      GeneratedImpact{
          "flipped edge interior hit",
          Eigen::Vector3d(2.0, 0.0, 0.0),
          Eigen::Vector3d(-2.0, 0.0, 0.0),
          Eigen::Vector3d(-0.25, 0.1, 0.0),
          Eigen::Vector3d(0.75, -1.0, 0.0),
          0.5,
          0.75},
  });

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  const Eigen::Vector3d point = Eigen::Vector3d::Zero();
  const expdetail::RigidIpcPose edgePoseStart;
  for (const GeneratedImpact& impact : impacts) {
    SCOPED_TRACE(impact.name);
    const expdetail::RigidIpcPose edgePoseEnd{
        impact.edgeVelocity, Eigen::Vector3d::Zero()};
    const Eigen::Vector3d edgeAAtToi = expdetail::transformRigidIpcPoint(
        impact.edgeA, edgePoseStart, edgePoseEnd, impact.timeOfImpact);
    const Eigen::Vector3d edgeBAtToi = expdetail::transformRigidIpcPoint(
        impact.edgeB, edgePoseStart, edgePoseEnd, impact.timeOfImpact);
    const Eigen::Vector3d impactPoint
        = edgeAAtToi + impact.alpha * (edgeBAtToi - edgeAAtToi);
    const Eigen::Vector3d pointVelocity
        = (impactPoint - impact.pointStart) / impact.timeOfImpact;
    const expdetail::RigidIpcPose pointPoseStart{
        impact.pointStart, Eigen::Vector3d::Zero()};
    const expdetail::RigidIpcPose pointPoseEnd{
        impact.pointStart + pointVelocity, Eigen::Vector3d::Zero()};

    expectResidualNearZero(
        expdetail::rigidIpcPointEdgeResidual(
            point,
            pointPoseStart,
            pointPoseEnd,
            impact.edgeA,
            impact.edgeB,
            edgePoseStart,
            edgePoseEnd,
            impact.timeOfImpact,
            impact.alpha));

    dart::collision::native::CcdPrimitiveResult result;
    EXPECT_TRUE(
        expdetail::rigidIpcPointEdgeIntervalCcd(
            point,
            pointPoseStart,
            pointPoseEnd,
            impact.edgeA,
            impact.edgeB,
            edgePoseStart,
            edgePoseEnd,
            option,
            result));
    expectIntervalCcdTimeOfImpact(result, impact.timeOfImpact);
  }

  const expdetail::RigidIpcPose parallelPointPoseStart{
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose parallelPointPoseEnd{
      Eigen::Vector3d(0.0, 2.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose parallelEdgePoseStart;
  const expdetail::RigidIpcPose parallelEdgePoseEnd{
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d::Zero()};
  dart::collision::native::CcdPrimitiveResult missResult;
  EXPECT_FALSE(
      expdetail::rigidIpcPointEdgeIntervalCcd(
          point,
          parallelPointPoseStart,
          parallelPointPoseEnd,
          Eigen::Vector3d(1.0, -1.0, 0.0),
          Eigen::Vector3d(1.0, 1.0, 0.0),
          parallelEdgePoseStart,
          parallelEdgePoseEnd,
          option,
          missResult));
  EXPECT_FALSE(missResult.isHit());
}

TEST(RigidIpcCcdCase, EdgeEdgeIntervalSubdivisionFindsExpectedToiRow)
{
  const Eigen::Vector3d edgeA0(-1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB0(0.0, 0.0, -1.0);
  const Eigen::Vector3d edgeB1(0.0, 0.0, 1.0);
  const expdetail::RigidIpcPose movingEdgePoseStart{
      Eigen::Vector3d(0.0, 1.0, 0.5), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose movingEdgePoseEnd{
      Eigen::Vector3d(0.0, -2.0, 0.5), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose staticPose;
  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  EXPECT_TRUE(
      expdetail::rigidIpcEdgeEdgeIntervalCcd(
          edgeA0,
          edgeA1,
          movingEdgePoseStart,
          movingEdgePoseEnd,
          edgeB0,
          edgeB1,
          staticPose,
          staticPose,
          option,
          result));
  expectIntervalCcdTimeOfImpact(result, 1.0 / 3.0);
}

TEST(RigidIpcCcdCase, PointTriangleIntervalSubdivisionFindsExpectedToiRow)
{
  const Eigen::Vector3d triangleA(-1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const Eigen::Vector3d point(0.0, 0.5, 0.0);
  const expdetail::RigidIpcPose staticPose;
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, 0.0, -2.0), Eigen::Vector3d::Zero()};
  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  EXPECT_TRUE(
      expdetail::rigidIpcPointTriangleIntervalCcd(
          point,
          pointPoseStart,
          pointPoseEnd,
          triangleA,
          triangleB,
          triangleC,
          staticPose,
          staticPose,
          option,
          result));
  expectIntervalCcdTimeOfImpact(result, 1.0 / 3.0);
}

TEST(RigidIpcCcdCase, EvaluatesAuditedUpstreamRootCcdRowsAsMisses)
{
  expectLoadedCcdCaseMiss(
      R"json(
{"face":{"pose_t0":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.2847228507565187,0.0]},"pose_t1":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.2847232462549631,0.0]},"vertex0":[0.30618621784789724,-0.1767766952966369,-2.0],"vertex1":[0.1767766952966369,0.30618621784789724,2.0],"vertex2":[0.30618621784789724,-0.1767766952966369,2.0]},"type":"fv","vertex":{"pose_t0":{"position":[1.5032191088348958,0.0031371238924182815,-0.01102932471453623],"rotation":[0.1100229671648746,-0.12286962030474363,-0.0068598726786883585]},"pose_t1":{"position":[1.502464630201717,0.002401860480133274,-0.008444326734566802],"rotation":[0.08399953239222324,-0.09380760141963806,-0.005237325552368608]},"vertex":[-0.125,-0.125,0.125]}}
)json",
      expio::RigidIpcCcdCaseType::FaceVertex,
      3u,
      1u);
  expectLoadedCcdCaseMiss(
      R"json(
{"edge0":{"pose_t0":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.858006073626625,0.0]},"pose_t1":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.8580099461090607,0.0]},"vertex0":[0.30618621784789724,-0.1767766952966369,-2.0],"vertex1":[0.30618621784789724,-0.1767766952966369,2.0]},"edge1":{"pose_t0":{"position":[1.6152987419335303,-0.2261956159086164,-0.9495801406522546],"rotation":[0.2118369662746981,0.26933045491938123,0.14445906418961982]},"pose_t1":{"position":[1.6251226560504626,-0.22805327573239847,-0.943487705773452],"rotation":[0.19467042682211233,0.27445700448823973,0.08746526363891724]},"vertex0":[0.125,0.125,0.125],"vertex1":[0.125,-0.125,0.125]},"type":"ee"}
)json",
      expio::RigidIpcCcdCaseType::EdgeEdge,
      2u,
      2u);
  expectLoadedCcdCaseMiss(
      R"json(
{"edge0":{"pose_t0":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.9640868587422395,0.0]},"pose_t1":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.965583950452753,0.0]},"vertex0":[-0.1767766952966369,-0.30618621784789724,-2.0],"vertex1":[0.30618621784789724,-0.1767766952966369,2.0]},"edge1":{"pose_t0":{"position":[1.571812816430235,-0.15937923590094102,-1.1303993061288904],"rotation":[0.14417714552100477,0.38574720393314293,0.0669773714434179]},"pose_t1":{"position":[1.5995701391217367,-0.19807661854141448,-1.140279480872887],"rotation":[0.10765182167081871,0.38822434928491234,0.07226339941986092]},"vertex0":[-0.125,-0.125,-0.125],"vertex1":[-0.125,0.125,0.125]},"type":"ee"}
)json",
      expio::RigidIpcCcdCaseType::EdgeEdge,
      2u,
      2u);
  expectLoadedCcdCaseMiss(
      R"json(
{"edge0":{"pose_t0":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.9642739952060537,0.0]},"pose_t1":{"position":[-1.4456028966473393e-18,4.625929269271485e-18,3.700743415417188e-17],"rotation":[0.0,1.9656873128724306,0.0]},"vertex0":[-0.1767766952966369,-0.30618621784789724,-2.0],"vertex1":[0.30618621784789724,-0.1767766952966369,2.0]},"edge1":{"pose_t0":{"position":[1.5752824817666726,-0.1642164087310002,-1.13163432797189],"rotation":[0.1396114800397315,0.38605684710211413,0.06763812494047328]},"pose_t1":{"position":[1.6014865849165514,-0.20074839769878888,-1.1409616366971578],"rotation":[0.10514666220687766,0.38839425374824244,0.07262595565903739]},"vertex0":[-0.125,-0.125,-0.125],"vertex1":[-0.125,0.125,0.125]},"type":"ee"}
)json",
      expio::RigidIpcCcdCaseType::EdgeEdge,
      2u,
      2u);
}

TEST(RigidIpcCcdCase, EvaluatesFirstAuditedWreckingBallRowsConservatively)
{
  struct CcdRow
  {
    std::string_view path;
    std::string_view source;
    expio::RigidIpcCcdCaseType expectedType;
    std::size_t expectedBodyAVertexCount;
    std::size_t expectedBodyBVertexCount;
  };

  // clang-format off
  const auto rows = std::to_array<CcdRow>({
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-000.json",
          R"json({"edge0":{"pose_t0":{"position":[10.911941628241879,18.299962690659203,2.9979775168507227e-07],"rotation":[-7.88292571871591e-09,2.9419479400467917e-08,-2.6179938779914966]},"pose_t1":{"position":[10.911941628241879,18.299962690659203,2.9979775168507227e-07],"rotation":[-7.88292571871591e-09,2.9419479400467917e-08,-2.6179938779914966]},"vertex0":[0.49270500000000045,0.17629191888410803,-2.9583562134437356e-07],"vertex1":[0.42633600000000016,0.2426619188841084,-2.9434396709567954e-07]},"edge1":{"pose_t0":{"position":[10.132497374176813,17.68224874036763,-4.308111590143753e-05],"rotation":[-1.5315598870200058,-0.41038023488691217,0.41038024411013746]},"pose_t1":{"position":[10.132497374176813,17.666656324962364,-4.308111590143753e-05],"rotation":[-1.5315598870200058,-0.41038023488691217,0.41038024411013746]},"vertex0":[0.5534799999999948,-0.0986500824369427,-0.05877869798510722],"vertex1":[0.5690979999999954,-4.308243694273532e-05,-0.058778700201280884]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-001.json",
          R"json({"edge0":{"pose_t0":{"position":[10.911941628241879,18.299962690659203,2.9979775168507227e-07],"rotation":[-7.88292571871591e-09,2.9419479400467917e-08,-2.6179938779914966]},"pose_t1":{"position":[10.911941628241879,18.299962690659203,2.9979775168507227e-07],"rotation":[-7.88292571871591e-09,2.9419479400467917e-08,-2.6179938779914966]},"vertex0":[0.49270500000000045,0.17629191888410803,-2.9583562134437356e-07],"vertex1":[0.42633600000000016,0.2426619188841084,-2.9434396709567954e-07]},"edge1":{"pose_t0":{"position":[10.132497374176813,17.66360974036764,-4.308111590143753e-05],"rotation":[-1.5315598870200058,-0.41038023488691217,0.41038024411013746]},"pose_t1":{"position":[10.132497374176813,17.65577308852684,-4.308111590143753e-05],"rotation":[-1.5315598870200058,-0.41038023488691217,0.41038024411013746]},"vertex0":[0.5534799999999948,-0.0986500824369427,-0.05877869798510722],"vertex1":[0.5690979999999954,-4.308243694273532e-05,-0.058778700201280884]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-002.json",
          R"json({"edge0":{"pose_t0":{"position":[10.207506350209275,17.539962972908675,-0.0006577352476800118],"rotation":[-1.4605106803170946,-0.5834968683845286,0.5937552733256987]},"pose_t1":{"position":[10.211037527980215,17.535645126754144,-0.0006744659611807837],"rotation":[-1.4573303009287428,-0.5903533479248949,0.6012701582193285]},"vertex0":[-0.5534800000000055,-0.09865008243694236,-0.058778697985107084],"vertex1":[-0.5500000000000034,-4.3081115894523724e-05,2.997987190318685e-07]},"edge1":{"pose_t0":{"position":[9.353095901429889,16.899459173081468,2.9979775167903616e-07],"rotation":[-7.88292573350226e-09,2.941947940749099e-08,-2.6179938779914966]},"pose_t1":{"position":[9.353095901429889,16.8864339803509,2.9979775167903616e-07],"rotation":[-7.88292573350226e-09,2.941947940749099e-08,-2.6179938779914966]},"vertex0":[-0.5690979999999998,-4.308243694191998e-05,0.05877870020128006],"vertex1":[-0.5353170000000005,0.09266191888410408,-2.9771519111287383e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-003.json",
          R"json({"face":{"pose_t0":{"position":[8.573651647364821,16.12428865897167,-4.3081115901472945e-05],"rotation":[-1.5315598870200058,-0.41038023488691167,0.41038024411013696]},"pose_t1":{"position":[8.573651647364821,16.116494637349273,-4.3081115901472945e-05],"rotation":[-1.5315598870200058,-0.41038023488691167,0.41038024411013696]},"vertex0":[0.5353169999999972,0.09266191888410505,2.977151876023982e-07],"vertex1":[0.5690979999999971,-4.3082436942797264e-05,-0.05877870020128147],"vertex2":[0.5499999999999953,-4.308111589493167e-05,2.997987163950925e-07]},"type":"fv","vertex":{"pose_t0":{"position":[9.477068538027583,16.717321680545123,-0.0032506824297151575],"rotation":[-0.0016292251296850125,-0.0032961014152704405,-2.6040052601339085]},"pose_t1":{"position":[9.480557509892252,16.714166087042365,-0.003322593255655234],"rotation":[-0.001597311873067204,-0.003543686492204609,-2.603354968663781]},"vertex":[0.535316999999999,0.09266191888410619,-2.9771519110106164e-07]}})json",
          expio::RigidIpcCcdCaseType::FaceVertex,
          3u,
          1u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-004.json",
          R"json({"edge0":{"pose_t0":{"position":[7.879043406144753,15.320578499278904,-0.0009276409156087907],"rotation":[-0.0012797789604315983,0.00336630568846746,-2.611186095803618]},"pose_t1":{"position":[7.89769602966136,15.302400818095863,-0.0010369625502564062],"rotation":[-0.0012723531621059668,0.004092888712867576,-2.6093719446096366]},"vertex0":[0.5499999999999996,-4.308111589477308e-05,-2.99798719903075e-07],"vertex1":[0.5353169999999993,0.09266191888410566,-2.977151910888039e-07]},"edge1":{"pose_t0":{"position":[7.01480592055283,14.789462293335486,-4.3081115901564255e-05],"rotation":[-1.5315598870200169,-0.4103802348869024,0.41038024411013785]},"pose_t1":{"position":[7.01480592055283,14.753689653140572,-4.3081115901564255e-05],"rotation":[-1.531559887020017,-0.41038023488690206,0.4103802441101376]},"vertex0":[0.549999999999996,-4.308111589491369e-05,2.9979871578445793e-07],"vertex1":[0.5353169999999964,0.09266191888410505,2.9771518790554186e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-005.json",
          R"json({"edge0":{"pose_t0":{"position":[7.913803374563371,15.287029292056902,-0.0011355389074375842],"rotation":[-0.0012822072659966987,0.0047041753741014465,-2.607741206831411]},"pose_t1":{"position":[7.925246449203736,15.276107453219804,-0.001203500384281765],"rotation":[-0.001287444821505691,0.00513401407469223,-2.6065712130212875]},"vertex0":[0.5499999999999996,-4.308111589477308e-05,-2.99798719903075e-07],"vertex1":[0.5534809999999992,0.09856392020513037,-0.058778297582544645]},"edge1":{"pose_t0":{"position":[7.01480592055283,14.723186313176308,-4.3081115901564255e-05],"rotation":[-1.531559887020017,-0.41038023488690173,0.41038024411013746]},"pose_t1":{"position":[7.01480592055283,14.701518885991936,-4.3081115901564255e-05],"rotation":[-1.5315598870200173,-0.4103802348869017,0.41038024411013735]},"vertex0":[0.569097999999997,-4.308243694275926e-05,-0.05877870020128163],"vertex1":[0.5353169999999964,0.09266191888410505,2.9771518790554186e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-006.json",
          R"json({"edge0":{"pose_t0":{"position":[7.913803374563371,15.287029292056902,-0.0011355389074375842],"rotation":[-0.0012822072659966987,0.0047041753741014465,-2.607741206831411]},"pose_t1":{"position":[7.922957834275663,15.278291820987224,-0.0011899080889129288],"rotation":[-0.0012863973104038924,0.005048046334574074,-2.6068052117833123]},"vertex0":[0.5499999999999996,-4.308111589477308e-05,-2.99798719903075e-07],"vertex1":[0.5534809999999992,0.09856392020513037,-0.058778297582544645]},"edge1":{"pose_t0":{"position":[7.01480592055283,14.723186313176308,-4.3081115901564255e-05],"rotation":[-1.531559887020017,-0.41038023488690173,0.41038024411013746]},"pose_t1":{"position":[7.01480592055283,14.70585237142881,-4.3081115901564255e-05],"rotation":[-1.5315598870200173,-0.4103802348869017,0.4103802441101374]},"vertex0":[0.549999999999996,-4.308111589491369e-05,2.9979871578445793e-07],"vertex1":[0.5353169999999964,0.09266191888410505,2.9771518790554186e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-007.json",
          R"json({"edge0":{"pose_t0":{"position":[7.1484585526211735,14.532910356255064,-0.0007979805305141525],"rotation":[-1.5212359212594082,-0.45888331097127455,0.4596592796921352]},"pose_t1":{"position":[7.159801915534379,14.52172033238264,-0.0008271654857834298],"rotation":[-1.520599519219108,-0.4617444823691199,0.4624655034432531]},"vertex0":[-0.535317000000003,-0.0927480811158947,3.0188224632790927e-07],"vertex1":[-0.5500000000000036,-4.3081115894739e-05,2.9979871692243653e-07]},"edge1":{"pose_t0":{"position":[6.2354044478059105,13.974760283244208,2.9979775172496085e-07],"rotation":[-7.882925742662685e-09,2.9419479388521943e-08,-2.617993877991494]},"pose_t1":{"position":[6.2354044478059105,13.955140337045547,2.9979775172496085e-07],"rotation":[-7.882925742662707e-09,2.9419479388521957e-08,-2.617993877991494]},"vertex0":[-0.5499999999999984,-4.3081115893524076e-05,-2.997987199660609e-07],"vertex1":[-0.5353169999999985,0.09266191888410658,-2.977151911544554e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-008.json",
          R"json({"edge0":{"pose_t0":{"position":[7.1484585526211735,14.532910356255064,-0.0007979805305141525],"rotation":[-1.5212359212594082,-0.45888331097127455,0.4596592796921352]},"pose_t1":{"position":[7.157533242951738,14.523958337157126,-0.0008213284947295743],"rotation":[-1.520726799627168,-0.4611722480895508,0.46190425869302953]},"vertex0":[-0.5534800000000033,-0.09865008243694254,-0.05877869798510855],"vertex1":[-0.5690980000000039,-4.3082436942578505e-05,-0.05877870020128338]},"edge1":{"pose_t0":{"position":[6.2354044478059105,13.974760283244208,2.9979775172496085e-07],"rotation":[-7.882925742662685e-09,2.9419479388521943e-08,-2.617993877991494]},"pose_t1":{"position":[6.2354044478059105,13.95906432628528,2.9979775172496085e-07],"rotation":[-7.882925742662703e-09,2.9419479388521953e-08,-2.617993877991494]},"vertex0":[-0.5499999999999984,-4.3081115893524076e-05,-2.997987199660609e-07],"vertex1":[-0.5353169999999985,0.09266191888410658,-2.977151911544554e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-009.json",
          R"json({"edge0":{"pose_t0":{"position":[7.1557183048856245,14.525748740976713,-0.00081665890188649],"rotation":[-1.520828623953616,-0.4607144606658956,0.46145526289285066]},"pose_t1":{"position":[7.160816387624655,14.520756634014779,-0.0008314084119709349],"rotation":[-1.520543016512156,-0.4620044494563353,0.462723960053946]},"vertex0":[-0.535317000000003,-0.0927480811158947,3.0188224632790927e-07],"vertex1":[-0.5500000000000036,-4.3081115894739e-05,2.9979871692243653e-07]},"edge1":{"pose_t0":{"position":[6.2354044478059105,13.962203517677064,2.9979775172496085e-07],"rotation":[-7.8829257426627e-09,2.9419479388521953e-08,-2.617993877991494]},"pose_t1":{"position":[6.2354044478059105,13.95343600449955,2.9979775172496085e-07],"rotation":[-7.88292574266271e-09,2.9419479388521957e-08,-2.617993877991494]},"vertex0":[-0.5499999999999984,-4.3081115893524076e-05,-2.997987199660609e-07],"vertex1":[-0.5353169999999985,0.09266191888410658,-2.977151911544554e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-010.json",
          R"json({"edge0":{"pose_t0":{"position":[6.332076992297075,13.843292089918595,-0.00027062451384046753],"rotation":[-0.0002503542930464432,-0.00040997704130250363,-2.6181790787342973]},"pose_t1":{"position":[6.352852528109589,13.822396456001464,-0.00029357026469275176],"rotation":[-0.00022559459350386058,-0.000418566932123113,-2.617707215962341]},"vertex0":[0.5500000000000019,-4.308111589307999e-05,-2.997987199650907e-07],"vertex1":[0.5534810000000018,0.09856392020513209,-0.058778297582544714]},"edge1":{"pose_t0":{"position":[5.455960193740842,13.32464377811749,-4.3081115901505945e-05],"rotation":[-1.5315598870200064,-0.4103802348869128,0.4103802441101366]},"pose_t1":{"position":[5.455960193740842,13.287766510518194,-4.3081115901505945e-05],"rotation":[-1.5315598870200064,-0.4103802348869128,0.4103802441101366]},"vertex0":[0.5499999999999978,-4.3081115894478316e-05,2.997987208082035e-07],"vertex1":[0.5353169999999974,0.09266191888410548,2.9771519143202886e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
      CcdRow{
          "tests/data/wrecking-ball/ccd-test-011.json",
          R"json({"edge0":{"pose_t0":{"position":[6.36435012465926,13.811244665397208,-0.00030841674433297486],"rotation":[-0.00020263036272448635,-0.000421010489237828,-2.6172313708639687]},"pose_t1":{"position":[6.37535110638482,13.800257242541772,-0.0003202945161899684],"rotation":[-0.00018450561305171335,-0.0004238507510265779,-2.6170097657921167]},"vertex0":[0.5500000000000019,-4.308111589307999e-05,-2.997987199650907e-07],"vertex1":[0.5534810000000018,0.09856392020513209,-0.058778297582544714]},"edge1":{"pose_t0":{"position":[5.455960193740842,13.267815130892737,-4.3081115901505945e-05],"rotation":[-1.5315598870200064,-0.4103802348869128,0.4103802441101366]},"pose_t1":{"position":[5.455960193740842,13.248476246166735,-4.3081115901505945e-05],"rotation":[-1.5315598870200064,-0.4103802348869128,0.4103802441101366]},"vertex0":[0.5499999999999978,-4.3081115894478316e-05,2.997987208082035e-07],"vertex1":[0.5353169999999974,0.09266191888410548,2.9771519143202886e-07]},"type":"ee"})json",
          expio::RigidIpcCcdCaseType::EdgeEdge,
          2u,
          2u},
  });
  // clang-format on

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  const auto interpolatePoseRecord = [](const expio::RigidIpcPoseRecord& start,
                                        const expio::RigidIpcPoseRecord& end,
                                        const double t) {
    expio::RigidIpcPoseRecord interpolated;
    interpolated.position
        = start.position + (end.position - start.position) * t;
    interpolated.rotation
        = start.rotation + (end.rotation - start.rotation) * t;
    return interpolated;
  };

  for (const CcdRow& row : rows) {
    SCOPED_TRACE(row.path);
    expio::RigidIpcCcdCase ccdCase = expectLoadedCcdCase(
        row.source,
        row.expectedType,
        row.expectedBodyAVertexCount,
        row.expectedBodyBVertexCount);
    dart::collision::native::CcdPrimitiveResult result;

    const bool hit = expio::evaluateRigidIpcCcdCase(ccdCase, option, result);
    if (!hit) {
      EXPECT_FALSE(result.isHit());
      continue;
    }

    EXPECT_TRUE(result.isHit());
    EXPECT_EQ(result.status, dart::collision::native::CcdPrimitiveStatus::Hit);
    EXPECT_GE(result.timeOfImpact, 0.0);
    EXPECT_LE(result.timeOfImpact, 1.0);

    ccdCase.bodyA.poseT1 = interpolatePoseRecord(
        ccdCase.bodyA.poseT0, ccdCase.bodyA.poseT1, result.timeOfImpact);
    ccdCase.bodyB.poseT1 = interpolatePoseRecord(
        ccdCase.bodyB.poseT0, ccdCase.bodyB.poseT1, result.timeOfImpact);

    dart::collision::native::CcdPrimitiveResult truncatedResult;
    EXPECT_FALSE(
        expio::evaluateRigidIpcCcdCase(ccdCase, option, truncatedResult));
    EXPECT_FALSE(truncatedResult.isHit());
  }
}

TEST(RigidIpcCcdCase, EvaluatesAuditedWreckingBallCorpusConservatively)
{
  const std::filesystem::path corpusPath
      = (std::filesystem::path(__FILE__).parent_path() / ".." / ".." / ".."
         / "fixtures" / "rigid_ipc" / "wrecking_ball_ccd.tsv")
            .lexically_normal();
  std::ifstream corpus(corpusPath);
  ASSERT_TRUE(corpus.is_open()) << corpusPath;

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  const auto interpolatePoseRecord = [](const expio::RigidIpcPoseRecord& start,
                                        const expio::RigidIpcPoseRecord& end,
                                        const double t) {
    expio::RigidIpcPoseRecord interpolated;
    interpolated.position
        = start.position + (end.position - start.position) * t;
    interpolated.rotation
        = start.rotation + (end.rotation - start.rotation) * t;
    return interpolated;
  };

  std::size_t rowCount = 0;
  std::string line;
  while (std::getline(corpus, line)) {
    if (line.empty()) {
      continue;
    }

    const std::size_t separator = line.find('\t');
    ASSERT_NE(separator, std::string::npos) << line;
    const std::string path = line.substr(0, separator);
    const std::string source = line.substr(separator + 1);

    SCOPED_TRACE(path);
    expio::RigidIpcCcdCase ccdCase = loadCcdCase(source);
    ASSERT_FALSE(ccdCase.hasErrors());
    ASSERT_TRUE(ccdCase.diagnostics.empty());
    switch (ccdCase.type) {
      case expio::RigidIpcCcdCaseType::EdgeVertex:
        ASSERT_EQ(ccdCase.bodyA.vertices.size(), 2u);
        ASSERT_EQ(ccdCase.bodyB.vertices.size(), 1u);
        break;
      case expio::RigidIpcCcdCaseType::EdgeEdge:
        ASSERT_EQ(ccdCase.bodyA.vertices.size(), 2u);
        ASSERT_EQ(ccdCase.bodyB.vertices.size(), 2u);
        break;
      case expio::RigidIpcCcdCaseType::FaceVertex:
        ASSERT_EQ(ccdCase.bodyA.vertices.size(), 3u);
        ASSERT_EQ(ccdCase.bodyB.vertices.size(), 1u);
        break;
    }

    dart::collision::native::CcdPrimitiveResult result;
    const bool hit = expio::evaluateRigidIpcCcdCase(ccdCase, option, result);
    if (!hit) {
      EXPECT_FALSE(result.isHit());
      ++rowCount;
      continue;
    }

    EXPECT_TRUE(result.isHit());
    EXPECT_EQ(result.status, dart::collision::native::CcdPrimitiveStatus::Hit);
    EXPECT_GE(result.timeOfImpact, 0.0);
    EXPECT_LE(result.timeOfImpact, 1.0);

    ccdCase.bodyA.poseT1 = interpolatePoseRecord(
        ccdCase.bodyA.poseT0, ccdCase.bodyA.poseT1, result.timeOfImpact);
    ccdCase.bodyB.poseT1 = interpolatePoseRecord(
        ccdCase.bodyB.poseT0, ccdCase.bodyB.poseT1, result.timeOfImpact);

    dart::collision::native::CcdPrimitiveResult truncatedResult;
    EXPECT_FALSE(
        expio::evaluateRigidIpcCcdCase(ccdCase, option, truncatedResult));
    EXPECT_FALSE(truncatedResult.isHit());
    ++rowCount;
  }

  EXPECT_EQ(rowCount, 386u);
}

TEST(RigidIpcCcdCase, EvaluatesAuditedKinematicRowsWithoutZeroTimeHits)
{
  struct CcdRow
  {
    std::string_view path;
    std::string_view source;
  };

  // clang-format off
  const auto rows = std::to_array<CcdRow>({
      CcdRow{
          "tests/data/kinematic/ccd-test-000.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.06464293009609998,-1.7124144938538095,0.23577611771346524],"rotation":[-0.005541911644287225,-0.0017852096358039146,1.514953719459307]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.953094536060126]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-001.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[-188.8666200203556,191.02295227278836,-0.2736538437914776],"rotation":[17.96891289460509,0.3388921386394125,225.05593383393384]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-32682.6730068579]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-002.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.04287524019931291,-1.7208290967550464,0.23764450180774682],"rotation":[-0.005253472356882234,0.000967751788754904,1.5286111347063949]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-3.010895206994393]},"vertex0":[0.04989031636268643,0.5711346212544138,0.2007290016720854],"vertex1":[0.04989031636268643,0.5711346212544138,0.36072900167208544]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-003.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[-188.88771516097086,191.01538193057422,-0.26794058935848275],"rotation":[17.96378830143519,0.34436505861467015,225.07560883107874]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-409330.1236004919]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-004.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.04284434952248971,-1.7208236334784988,0.2376555580466178],"rotation":[-0.005251598335474483,0.0009845487605744292,1.528580079130178]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-3.010895204576507]},"vertex0":[0.04989031636268643,0.5711346212544138,0.2007290016720854],"vertex1":[0.04989031636268643,0.5711346212544138,0.36072900167208544]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-005.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[-188.86510692578787,191.02658748520588,-0.2686935239513957],"rotation":[17.96455130359756,0.3435701537068805,225.05430914005535]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-409330.1236004919]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-006.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.042844349523819235,-1.7208236334786133,0.23765555804608907],"rotation":[-0.005251598335636669,0.0009845487597225455,1.5285800791314184]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-3.010895204576507]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-007.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[-188.86510692578787,191.02658748520582,-0.2686935239513958],"rotation":[17.964551303597553,0.34357015370688015,225.05430914005532]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-409330.1236004919]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-008.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.042844349524471734,-1.720823633478722,0.2376555580456161],"rotation":[-0.005251598335496949,0.0009845487591719183,1.5285800791320534]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-3.010895204576507]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-009.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[-188.86510692578784,191.0265874852058,-0.2686935239513958],"rotation":[17.964551303597553,0.3435701537068803,225.05430914005532]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-409330.1236004919]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-010.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.0428443495237289,-1.7208236334788245,0.23765555804507665],"rotation":[-0.005251598335696727,0.0009845487582799478,1.528580079131528]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-3.0108952045765074]},"vertex0":[0.04989031636268643,0.5711346212544138,0.2007290016720854],"vertex1":[0.04989031636268643,0.5711346212544138,0.36072900167208544]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-011.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[-188.86510692578784,191.02658748520574,-0.2686935239513957],"rotation":[17.964551303597563,0.3435701537068801,225.05430914005527]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-409330.1236004919]},"vertex0":[0.04989031636268643,0.5711346212544138,0.36072900167208544],"vertex1":[0.04989031636268643,0.5711346212544138,0.2007290016720854]},"type":"ee"})json"},
      CcdRow{
          "tests/data/kinematic/ccd-test-012.json",
          R"json({"edge0":{"pose_t0":{"position":[0.06645567526560373,-1.7117800373895864,0.23612110416527277],"rotation":[-0.004142222049721008,-0.0003305093180525144,1.513657457007691]},"pose_t1":{"position":[0.04284434952453921,-1.7208236334786313,0.2376555580460089],"rotation":[-0.005251598335716702,0.0009845487595517156,1.5285800791320514]},"vertex0":[1.1852423663342049,0.07029444633957141,-0.07495858873600351],"vertex1":[1.168928700047883,0.08498176251828493,-0.07495073019079]},"edge1":{"pose_t0":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-2.948064743119094]},"pose_t1":{"position":[0.0001096836373135734,0.042263388999492495,-0.050729001672085425],"rotation":[0.0,0.0,-3.010895204576507]},"vertex0":[0.04989031636268643,0.5711346212544138,0.2007290016720854],"vertex1":[0.04989031636268643,0.5711346212544138,0.36072900167208544]},"type":"ee"})json"},
  });
  // clang-format on

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  for (const CcdRow& row : rows) {
    SCOPED_TRACE(row.path);
    const expio::RigidIpcCcdCase ccdCase = expectLoadedCcdCase(
        row.source, expio::RigidIpcCcdCaseType::EdgeEdge, 2u, 2u);
    dart::collision::native::CcdPrimitiveResult result;
    const bool hit = expio::evaluateRigidIpcCcdCase(ccdCase, option, result);

    if (hit) {
      EXPECT_TRUE(result.isHit());
      EXPECT_GT(result.timeOfImpact, 0.0);
      EXPECT_LE(result.timeOfImpact, 1.0);
    } else {
      EXPECT_FALSE(result.isHit());
    }
  }
}

TEST(RigidIpcCcdCase, FaceVertexRotationalTrajectoryUsesCurvedCcd)
{
  const Eigen::Vector3d triangleA(-2.0, -2.0, 0.0);
  const Eigen::Vector3d triangleB(2.0, -2.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 2.0, 0.0);
  const Eigen::Vector3d point(0.0, 0.0, 1.0);

  const expdetail::RigidIpcPose trianglePose;
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 0.5, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, 0.5, 0.0),
      Eigen::Vector3d(2.0 * std::numbers::pi, 0.0, 0.0)};

  const Eigen::Vector3d pointStart
      = expdetail::transformRigidIpcPoint(point, pointPoseStart);
  const Eigen::Vector3d pointEnd
      = expdetail::transformRigidIpcPoint(point, pointPoseEnd);

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  const bool linearHit = dart::collision::native::pointTriangleCcd(
      pointStart,
      pointEnd,
      triangleA,
      triangleA,
      triangleB,
      triangleB,
      triangleC,
      triangleC,
      option,
      result);

  EXPECT_FALSE(linearHit);
  EXPECT_FALSE(result.isHit());
  EXPECT_DOUBLE_EQ(result.timeOfImpact, 1.0);

  const bool curvedHit = expdetail::rigidIpcPointTriangleCcd(
      point,
      pointPoseStart,
      pointPoseEnd,
      triangleA,
      triangleB,
      triangleC,
      trianglePose,
      trianglePose,
      option,
      result);

  EXPECT_TRUE(curvedHit);
  EXPECT_TRUE(result.isHit());
  EXPECT_LE(result.timeOfImpact, 0.25);
  EXPECT_NEAR(result.timeOfImpact, 0.25, option.tolerance);
}

TEST(RigidIpcCcdCase, FaceVertexCcdHonorsMinimumSeparation)
{
  const Eigen::Vector3d triangleA(-2.0, -2.0, 0.0);
  const Eigen::Vector3d triangleB(2.0, -2.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 2.0, 0.0);
  const Eigen::Vector3d point = Eigen::Vector3d::Zero();

  const expdetail::RigidIpcPose trianglePose;
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, 0.0, -1.0), Eigen::Vector3d::Zero()};

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  option.minSeparation = 0.25;
  dart::collision::native::CcdPrimitiveResult result;

  const bool hit = expdetail::rigidIpcPointTriangleCcd(
      point,
      pointPoseStart,
      pointPoseEnd,
      triangleA,
      triangleB,
      triangleC,
      trianglePose,
      trianglePose,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_LE(result.timeOfImpact, 0.375);
  EXPECT_NEAR(result.timeOfImpact, 0.375, option.tolerance);
}

TEST(RigidIpcCcdCase, PointEdgeRotationalTrajectoryUsesCurvedCcd)
{
  const Eigen::Vector3d edgeA(0.0, -2.0, 0.0);
  const Eigen::Vector3d edgeB(0.0, 2.0, 0.0);
  const Eigen::Vector3d point(1.0, 0.0, 0.0);

  const expdetail::RigidIpcPose edgePose;
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.0, 0.0, 2.0 * std::numbers::pi)};

  const Eigen::Vector3d pointStart
      = expdetail::transformRigidIpcPoint(point, pointPoseStart);
  const Eigen::Vector3d pointEnd
      = expdetail::transformRigidIpcPoint(point, pointPoseEnd);
  ASSERT_TRUE(pointStart.isApprox(pointEnd));

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  const bool curvedHit = expdetail::rigidIpcPointEdgeCcd(
      point,
      pointPoseStart,
      pointPoseEnd,
      edgeA,
      edgeB,
      edgePose,
      edgePose,
      option,
      result);

  EXPECT_TRUE(curvedHit);
  EXPECT_TRUE(result.isHit());
  EXPECT_LE(result.timeOfImpact, 0.25);
  EXPECT_NEAR(result.timeOfImpact, 0.25, option.tolerance);
}

TEST(RigidIpcCcdCase, PointEdgeCcdHonorsMinimumSeparation)
{
  const Eigen::Vector3d edgeA(-2.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB(2.0, 0.0, 0.0);
  const Eigen::Vector3d point = Eigen::Vector3d::Zero();

  const expdetail::RigidIpcPose edgePose;
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d::Zero()};

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  option.minSeparation = 0.25;
  dart::collision::native::CcdPrimitiveResult result;

  const bool hit = expdetail::rigidIpcPointEdgeCcd(
      point,
      pointPoseStart,
      pointPoseEnd,
      edgeA,
      edgeB,
      edgePose,
      edgePose,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_LE(result.timeOfImpact, 0.375);
  EXPECT_NEAR(result.timeOfImpact, 0.375, option.tolerance);
}

TEST(RigidIpcCcdCase, EdgeEdgeRotationalTrajectoryUsesCurvedCcd)
{
  const Eigen::Vector3d edgeA0(-1.0, 0.5, 0.0);
  const Eigen::Vector3d edgeA1(1.0, 0.5, 0.0);
  const Eigen::Vector3d edgeB0(0.0, 0.0, -1.0);
  const Eigen::Vector3d edgeB1(0.0, 0.0, 1.0);

  const expdetail::RigidIpcPose edgeAPose;
  const expdetail::RigidIpcPose edgeBPoseStart{
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose edgeBPoseEnd{
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(2.0 * std::numbers::pi, 0.0, 0.0)};

  const Eigen::Vector3d edgeB0Start
      = expdetail::transformRigidIpcPoint(edgeB0, edgeBPoseStart);
  const Eigen::Vector3d edgeB0End
      = expdetail::transformRigidIpcPoint(edgeB0, edgeBPoseEnd);
  const Eigen::Vector3d edgeB1Start
      = expdetail::transformRigidIpcPoint(edgeB1, edgeBPoseStart);
  const Eigen::Vector3d edgeB1End
      = expdetail::transformRigidIpcPoint(edgeB1, edgeBPoseEnd);

  dart::collision::native::CcdOption option
      = dart::collision::native::CcdOption::precise();
  dart::collision::native::CcdPrimitiveResult result;

  const bool linearHit = dart::collision::native::edgeEdgeCcd(
      edgeA0,
      edgeA0,
      edgeA1,
      edgeA1,
      edgeB0Start,
      edgeB0End,
      edgeB1Start,
      edgeB1End,
      option,
      result);

  EXPECT_FALSE(linearHit);
  EXPECT_FALSE(result.isHit());
  EXPECT_DOUBLE_EQ(result.timeOfImpact, 1.0);

  const bool curvedHit = expdetail::rigidIpcEdgeEdgeCcd(
      edgeA0,
      edgeA1,
      edgeAPose,
      edgeAPose,
      edgeB0,
      edgeB1,
      edgeBPoseStart,
      edgeBPoseEnd,
      option,
      result);

  EXPECT_TRUE(curvedHit);
  EXPECT_TRUE(result.isHit());
  EXPECT_LE(result.timeOfImpact, 0.25);
  EXPECT_NEAR(result.timeOfImpact, 0.25, option.tolerance);
}

} // namespace
