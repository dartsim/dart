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

#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/detail/rigid_ipc_ccd.hpp>
#include <dart/simulation/experimental/io/detail/rigid_ipc_fixture.hpp>
#include <dart/simulation/experimental/world.hpp>

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

namespace sx = dart::simulation::experimental;
namespace expio = dart::simulation::experimental::io::detail;
namespace expdetail = dart::simulation::experimental::detail;

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
      dart::simulation::experimental::InvalidArgumentException);
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
      dart::simulation::experimental::InvalidArgumentException);
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

  dart::simulation::experimental::World world;
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
      dart::simulation::experimental::CollisionShapeType::Mesh);
  EXPECT_EQ(body0->getCollisionShape()->vertices.size(), 3u);
  EXPECT_EQ(body0->getCollisionShape()->triangles.size(), 1u);

  const auto body1 = world.getRigidBody("rigid_ipc_body_001");
  ASSERT_TRUE(body1.has_value());
  EXPECT_TRUE(body1->isStatic());
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

  dart::simulation::experimental::World world;
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

  dart::simulation::experimental::World world;
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
    stageOptions.activationDistance = *fixture.barrierActivationDistance;
    stageOptions.frictionIterations
        = static_cast<std::size_t>(*fixture.frictionIterations);
    stageOptions.staticFrictionSpeedBound = *fixture.staticFrictionSpeedBound;
    stageOptions.frictionConvergenceTolerance
        = *fixture.velocityConvergenceTolerance;
    sx::compute::RigidIpcContactStage ipcStage(stageOptions);
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

  EXPECT_TRUE(frictionToleranceEarlyStop.stats.converged);
  EXPECT_GT(frictionToleranceEarlyStop.stats.activeConstraints, 0u);
  EXPECT_GT(frictionToleranceEarlyStop.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(frictionToleranceEarlyStop.stats.frictionIterations, 1u);

  sx::compute::RigidIpcContactStageOptions invalidOptions;
  invalidOptions.activationDistance = -1.0;
  invalidOptions.staticFrictionSpeedBound = -1.0;
  invalidOptions.frictionConvergenceTolerance = -1.0;
  const sx::compute::RigidIpcContactStage invalidStage(invalidOptions);
  EXPECT_DOUBLE_EQ(invalidStage.getActivationDistance(), 1e-2);
  EXPECT_DOUBLE_EQ(invalidStage.getStaticFrictionSpeedBound(), 1e-3);
  EXPECT_DOUBLE_EQ(invalidStage.getFrictionConvergenceTolerance(), 0.0);
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

  dart::simulation::experimental::World world;
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

  dart::simulation::experimental::World world;
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

  dart::simulation::experimental::World world;
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
      dart::simulation::experimental::CollisionShapeType::Mesh);
  EXPECT_FALSE(world.collide().empty());
}

TEST(RigidIpcFixtureReplay, LoadsOffStlAndMshMeshAssets)
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

  dart::simulation::experimental::World world;
  expio::RigidIpcReplayOptions options;
  options.assetRoot = assetRoot;
  const expio::RigidIpcReplayState state
      = expio::populateRigidIpcReplayWorld(world, fixture, options);

  ASSERT_EQ(state.bodies.size(), 4u);
  EXPECT_TRUE(state.bodies[0].collisionMeshLoaded);
  EXPECT_EQ(state.bodies[0].collisionMeshVertexCount, 4u);
  EXPECT_EQ(state.bodies[0].collisionMeshTriangleCount, 4u);
  EXPECT_EQ(state.bodies[0].collisionMeshStatus, "loaded");
  for (std::size_t i = 1; i < state.bodies.size(); ++i) {
    EXPECT_TRUE(state.bodies[i].collisionMeshLoaded);
    EXPECT_EQ(state.bodies[i].collisionMeshVertexCount, i == 3u ? 4u : 12u);
    EXPECT_EQ(state.bodies[i].collisionMeshTriangleCount, 4u);
    EXPECT_EQ(state.bodies[i].collisionMeshStatus, "loaded");
  }

  for (std::size_t i = 0; i < state.bodies.size(); ++i) {
    const auto body = world.getRigidBody(state.bodies[i].bodyName);
    ASSERT_TRUE(body.has_value());
    ASSERT_TRUE(body->getCollisionShape().has_value());
    EXPECT_EQ(
        body->getCollisionShape()->type,
        dart::simulation::experimental::CollisionShapeType::Mesh);
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
      dart::simulation::experimental::InvalidArgumentException);
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
