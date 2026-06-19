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

#pragma once

#include <dart/simulation/compute/detail/world_step_stages.hpp>
#include <dart/simulation/export.hpp>

#include <Eigen/Core>

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace dart::collision::native {
struct CcdOption;
struct CcdPrimitiveResult;
} // namespace dart::collision::native

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::simulation::io::detail {

/// Diagnostic severity emitted while reading a reference rigid-contact fixture.
enum class RigidIpcFixtureDiagnosticSeverity
{
  Warning,
  Error
};

/// Non-fatal importer diagnostic with a JSON path.
struct RigidIpcFixtureDiagnostic
{
  RigidIpcFixtureDiagnosticSeverity severity{
      RigidIpcFixtureDiagnosticSeverity::Warning};
  std::string path;
  std::string message;
};

/// Body motion mode recorded by the fixture format.
enum class RigidIpcBodyMode
{
  Dynamic,
  Static,
  Kinematic
};

/// Body record extracted from a reference rigid-contact fixture.
struct RigidIpcBodyRecord
{
  std::string meshPath;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotationDegrees = Eigen::Vector3d::Zero();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
  std::vector<bool> fixedDofs;
  std::optional<int> groupId;
  std::optional<double> density;
  std::optional<double> youngModulus;
  std::optional<double> poissonsRatio;
  std::optional<double> kinematicMaxTime;
  RigidIpcBodyMode mode{RigidIpcBodyMode::Dynamic};
  bool hasInlineGeometry{false};
  std::vector<Eigen::Vector3d> inlineVertices;
  std::vector<Eigen::Vector2i> inlineEdges;
  std::vector<std::vector<Eigen::Vector3d>> inlinePolygons;
  std::optional<bool> inlinePolygonsOriented;
};

/// Options for loading a reference rigid-contact fixture.
struct RigidIpcFixtureLoadOptions
{
  /// Treat warnings as fatal. Schema errors are always fatal.
  bool strict{false};
};

/// DART-owned summary of one reference rigid-contact fixture.
struct RigidIpcFixture
{
  /// Directory containing the loaded fixture or comparison script, when known.
  /// Replay uses this as the default base for relative mesh paths.
  std::filesystem::path sourceDirectory;
  std::string sceneType;
  std::string solverName;
  std::optional<std::string> energyModel;
  double timeStep{0.0};
  double maxTime{0.0};
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  double coefficientFriction{0.0};
  double coefficientRestitution{0.0};
  std::optional<double> barrierActivationDistance;
  std::optional<double> velocityConvergenceTolerance;
  std::optional<bool> velocityConvergenceToleranceIsAbsolute;
  std::optional<double> staticFrictionSpeedBound;
  std::optional<int> frictionIterations;
  std::optional<bool> warmStartEnabled;
  std::optional<bool> selfCollisionEnabled;
  std::optional<bool> gravityDisabled;
  std::vector<RigidIpcBodyRecord> bodies;
  std::vector<RigidIpcFixtureDiagnostic> diagnostics;

  [[nodiscard]] DART_SIMULATION_API bool hasErrors() const noexcept;
};

/// Options for creating an DART 7 World replay state from a loaded
/// rigid IPC fixture.
struct RigidIpcReplayOptions
{
  /// Root used to resolve relative mesh paths recorded by upstream fixtures.
  /// When empty, path-loaded fixtures default to their source directory.
  std::filesystem::path assetRoot;

  /// Prefix for deterministic body names created in the target World.
  std::string bodyNamePrefix{"rigid_ipc_body"};

  /// Load supported mesh files into collision shapes attached to replay bodies.
  bool loadMeshCollisionShapes{true};
};

/// DART-owned metadata linking one fixture rigid body row to the replay World.
struct RigidIpcReplayBodyState
{
  std::size_t sourceBodyIndex{0};
  std::string bodyName;
  std::filesystem::path sourceMeshPath;
  std::filesystem::path resolvedMeshPath;
  bool resolvedMeshExists{false};
  Eigen::Vector3d geometryScale = Eigen::Vector3d::Ones();
  Eigen::Vector3d geometryRotationDegrees = Eigen::Vector3d::Zero();
  RigidIpcBodyMode mode{RigidIpcBodyMode::Dynamic};
  std::vector<bool> fixedDofs;
  std::optional<int> groupId;
  std::optional<double> density;
  std::optional<double> youngModulus;
  std::optional<double> poissonsRatio;
  std::optional<double> kinematicMaxTime;
  bool hasInlineGeometry{false};
  std::size_t inlineGeometryVertexCount{0};
  std::size_t inlineGeometryEdgeCount{0};
  std::size_t inlineGeometryPolygonCount{0};
  bool collisionMeshLoaded{false};
  std::size_t collisionMeshVertexCount{0};
  std::size_t collisionMeshTriangleCount{0};
  std::string collisionMeshStatus;
};

/// Metadata for the bodies populated into an DART 7 World.
struct RigidIpcReplayState
{
  std::vector<RigidIpcReplayBodyState> bodies;
};

/// Direct CCD case family recorded by the reference test-data corpus.
enum class RigidIpcCcdCaseType
{
  EdgeVertex,
  EdgeEdge,
  FaceVertex
};

/// Pose record used by direct CCD test-data rows.
struct RigidIpcPoseRecord
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
};

/// Primitive body record used by direct CCD test-data rows.
struct RigidIpcCcdBodyRecord
{
  RigidIpcPoseRecord poseT0;
  RigidIpcPoseRecord poseT1;
  std::vector<Eigen::Vector3d> vertices;
};

/// DART-owned summary of one direct CCD test-data row.
struct RigidIpcCcdCase
{
  RigidIpcCcdCaseType type{RigidIpcCcdCaseType::EdgeEdge};
  RigidIpcCcdBodyRecord bodyA;
  RigidIpcCcdBodyRecord bodyB;
  std::vector<RigidIpcFixtureDiagnostic> diagnostics;

  [[nodiscard]] DART_SIMULATION_API bool hasErrors() const noexcept;
};

DART_SIMULATION_API RigidIpcFixture loadRigidIpcFixture(
    std::istream& input, const RigidIpcFixtureLoadOptions& options = {});

DART_SIMULATION_API RigidIpcFixture loadRigidIpcFixture(
    const std::filesystem::path& path,
    const RigidIpcFixtureLoadOptions& options = {});

/// Load the upstream IPC comparison script shape-row subset as a replay
/// fixture. This importer preserves mesh poses, material density, initial or
/// prescribed velocities, Neumann body forces, and the scalar solver metadata
/// already represented by RigidIpcFixture; it does not select or enable a
/// solver.
DART_SIMULATION_API RigidIpcFixture loadRigidIpcComparisonScript(
    std::istream& input, const RigidIpcFixtureLoadOptions& options = {});

DART_SIMULATION_API RigidIpcFixture loadRigidIpcComparisonScript(
    const std::filesystem::path& path,
    const RigidIpcFixtureLoadOptions& options = {});

/// Apply fixture-level solver metadata to an opt-in rigid IPC contact stage
/// options object.
///
/// This internal bridge preserves any caller-provided option when the fixture
/// omits that field or records metadata that does not have an exact runtime
/// stage meaning.
DART_SIMULATION_API void applyRigidIpcFixtureStageOptions(
    const RigidIpcFixture& fixture,
    compute::RigidIpcContactStageOptions& options);

/// Populate an DART 7 World with the currently supported fixture state.
///
/// Supported OBJ, OFF, STL, legacy VTK, and rigid-ipc MSH mesh paths plus
/// polygonal inline geometry are loaded as native-backed mesh collision shapes.
/// Unsupported or missing mesh assets are preserved in the returned replay
/// state with a status message. This replay path still does not enable a rigid
/// IPC solver.
[[nodiscard]] DART_SIMULATION_API RigidIpcReplayState
populateRigidIpcReplayWorld(
    World& world,
    const RigidIpcFixture& fixture,
    const RigidIpcReplayOptions& options = {});

/// Populate an DART 7 World from a fixture and run exactly one opt-in
/// rigid IPC contact-stage step.
///
/// This internal helper demonstrates the owned runtime replay path for manifest
/// drivers: fixture population remains separate from solver selection, parsed
/// fixture metadata is applied to the supplied stage options, and the default
/// World step pipeline is left unchanged.
[[nodiscard]] DART_SIMULATION_API RigidIpcReplayState
populateAndStepRigidIpcReplayWorld(
    World& world,
    const RigidIpcFixture& fixture,
    const RigidIpcReplayOptions& replayOptions = {},
    compute::RigidIpcContactStageOptions stageOptions = {},
    compute::RigidIpcSolverStats* stats = nullptr);

DART_SIMULATION_API RigidIpcCcdCase loadRigidIpcCcdCase(
    std::istream& input, const RigidIpcFixtureLoadOptions& options = {});

DART_SIMULATION_API RigidIpcCcdCase loadRigidIpcCcdCase(
    const std::filesystem::path& path,
    const RigidIpcFixtureLoadOptions& options = {});

/// Replay one loaded direct CCD test-data row through the internal rigid
/// curved-trajectory CCD implementation.
[[nodiscard]] DART_SIMULATION_API bool evaluateRigidIpcCcdCase(
    const RigidIpcCcdCase& ccdCase,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

} // namespace dart::simulation::io::detail
