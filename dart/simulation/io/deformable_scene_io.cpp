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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/simulation/io/deformable_scene_io.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/deformable_body.hpp"
#include "dart/simulation/comps/name.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>

#include <cmath>
#include <cstdint>

namespace dart::simulation::io {
namespace {

//==============================================================================
struct LoadedTetMesh
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<DeformableTetrahedron> tetrahedra;
  std::vector<DeformableSurfaceTriangle> surfaceTriangles;
};

//==============================================================================
struct ShapeBoundaryDirichlet
{
  Eigen::Vector3d minCorner = Eigen::Vector3d::Zero();
  Eigen::Vector3d maxCorner = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  std::optional<double> startTime;
  std::optional<double> endTime;
};

//==============================================================================
struct ShapeBoundaryNeumann
{
  Eigen::Vector3d minCorner = Eigen::Vector3d::Zero();
  Eigen::Vector3d maxCorner = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  std::optional<double> startTime;
  std::optional<double> endTime;
};

//==============================================================================
struct ShapeRecord
{
  std::filesystem::path meshPath;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotationDegrees = Eigen::Vector3d::Zero();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  std::optional<DeformableMaterialProperties> material;
  std::optional<Eigen::Vector3d> linearVelocity;
  std::optional<Eigen::Vector3d> angularVelocity;
  std::optional<Eigen::Vector3d> initialLinearVelocity;
  std::optional<Eigen::Vector3d> initialAngularVelocity;
  std::vector<ShapeBoundaryDirichlet> dirichlet;
  std::vector<ShapeBoundaryNeumann> neumann;
};

//==============================================================================
struct ParsedScene
{
  double density = 1000.0;
  double youngsModulus = 1.0e5;
  double poissonRatio = 0.4;
  double duration = 5.0;
  double timeStep = 0.025;
  bool gravityEnabled = true;
  std::array<double, 2> dirichletTimeRange{
      0.0, std::numeric_limits<double>::infinity()};
  std::array<double, 2> neumannTimeRange{
      0.0, std::numeric_limits<double>::infinity()};
  std::vector<ShapeRecord> shapes;
  std::vector<std::string> warnings;
};

//==============================================================================
std::string trim(std::string_view value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return std::string(value.substr(first, last - first + 1));
}

//==============================================================================
std::string stripComment(std::string_view line)
{
  const auto comment = line.find('#');
  if (comment == std::string_view::npos) {
    return trim(line);
  }
  return trim(line.substr(0, comment));
}

//==============================================================================
std::string readLogicalLine(std::istream& input, std::size_t& lineNumber)
{
  std::string line;
  while (std::getline(input, line)) {
    ++lineNumber;
    std::string logical = stripComment(line);
    while (!logical.empty() && logical.back() == '\\') {
      logical.pop_back();
      std::string continuation;
      if (!std::getline(input, continuation)) {
        break;
      }
      ++lineNumber;
      logical += " " + stripComment(continuation);
    }

    logical = trim(logical);
    if (!logical.empty()) {
      return logical;
    }
  }

  return "";
}

//==============================================================================
template <typename T>
T readRequired(std::istream& input, std::string_view field)
{
  T value{};
  DART_SIMULATION_THROW_T_IF(
      !(input >> value),
      InvalidArgumentException,
      "Failed to parse deformable scene field '{}'",
      field);
  return value;
}

//==============================================================================
std::optional<double> readOptionalDouble(std::istream& input)
{
  double value = 0.0;
  if (input >> value) {
    return value;
  }
  input.clear();
  return std::nullopt;
}

//==============================================================================
Eigen::Vector3d readVector3(std::istream& input, std::string_view field)
{
  return {
      readRequired<double>(input, field),
      readRequired<double>(input, field),
      readRequired<double>(input, field)};
}

//==============================================================================
constexpr double kDegreesToRadians
    = 3.141592653589793238462643383279502884 / 180.0;

//==============================================================================
Eigen::Matrix3d rotationFromDegrees(const Eigen::Vector3d& degrees)
{
  return (Eigen::AngleAxisd(
              degrees.x() * kDegreesToRadians, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(
              degrees.y() * kDegreesToRadians, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(
              degrees.z() * kDegreesToRadians, Eigen::Vector3d::UnitZ()))
      .toRotationMatrix();
}

//==============================================================================
std::filesystem::path resolveScenePath(
    const std::filesystem::path& scenePath,
    const std::filesystem::path& assetRoot,
    const std::filesystem::path& rawPath)
{
  if (rawPath.is_absolute()) {
    return rawPath;
  }

  const auto sceneRelative = scenePath.parent_path() / rawPath;
  if (std::filesystem::exists(sceneRelative)) {
    return sceneRelative;
  }

  if (!assetRoot.empty()) {
    const auto rootRelative = assetRoot / rawPath;
    if (std::filesystem::exists(rootRelative)) {
      return rootRelative;
    }
  }

  for (auto cursor = scenePath.parent_path(); !cursor.empty();
       cursor = cursor.parent_path()) {
    const auto candidate = cursor / rawPath;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
    if (cursor == cursor.root_path()) {
      break;
    }
  }

  return sceneRelative;
}

//==============================================================================
void requireFileExists(const std::filesystem::path& path)
{
  DART_SIMULATION_THROW_T_IF(
      !std::filesystem::exists(path),
      InvalidArgumentException,
      "Deformable scene asset '{}' does not exist",
      path.string());
}

//==============================================================================
int gmshElementNodeCount(int elementType)
{
  switch (elementType) {
    case 1:
      return 2;
    case 2:
      return 3;
    case 4:
      return 4;
    case 15:
      return 1;
    default:
      return -1;
  }
}

//==============================================================================
LoadedTetMesh loadGmshTetMesh(const std::filesystem::path& path)
{
  requireFileExists(path);
  std::ifstream input(path);
  DART_SIMULATION_THROW_T_IF(
      !input,
      InvalidArgumentException,
      "Unable to open deformable scene mesh '{}'",
      path.string());

  LoadedTetMesh mesh;
  std::map<std::uint64_t, std::size_t> tagToIndex;
  std::string token;
  while (input >> token) {
    if (token == "$Nodes") {
      const auto blockCount = readRequired<std::size_t>(input, "$Nodes blocks");
      const auto nodeCount = readRequired<std::size_t>(input, "$Nodes count");
      (void)readRequired<std::uint64_t>(input, "$Nodes min tag");
      (void)readRequired<std::uint64_t>(input, "$Nodes max tag");
      mesh.positions.reserve(nodeCount);

      for (std::size_t block = 0; block < blockCount; ++block) {
        const auto entityDim = readRequired<int>(input, "$Nodes entity dim");
        DART_SIMULATION_THROW_T_IF(
            entityDim < 0 || entityDim > 3,
            InvalidArgumentException,
            "Invalid Gmsh node entity dimension {} in '{}'",
            entityDim,
            path.string());
        (void)readRequired<int>(input, "$Nodes entity tag");
        const auto parametric = readRequired<int>(input, "$Nodes parametric");
        const auto blockNodeCount
            = readRequired<std::size_t>(input, "$Nodes block count");
        std::vector<std::uint64_t> tags(blockNodeCount);
        for (auto& tag : tags) {
          tag = readRequired<std::uint64_t>(input, "$Nodes tag");
        }
        for (std::size_t i = 0; i < blockNodeCount; ++i) {
          const auto position = readVector3(input, "$Nodes position");
          if (parametric != 0) {
            for (int coord = 0; coord < entityDim; ++coord) {
              (void)readRequired<double>(input, "$Nodes parametric coordinate");
            }
          }
          tagToIndex.emplace(tags[i], mesh.positions.size());
          mesh.positions.push_back(position);
        }
      }
    } else if (token == "$Elements") {
      const auto blockCount
          = readRequired<std::size_t>(input, "$Elements blocks");
      (void)readRequired<std::size_t>(input, "$Elements count");
      (void)readRequired<std::uint64_t>(input, "$Elements min tag");
      (void)readRequired<std::uint64_t>(input, "$Elements max tag");
      for (std::size_t block = 0; block < blockCount; ++block) {
        (void)readRequired<int>(input, "$Elements entity dim");
        (void)readRequired<int>(input, "$Elements entity tag");
        const auto elementType = readRequired<int>(input, "$Elements type");
        const auto elementCount
            = readRequired<std::size_t>(input, "$Elements block count");
        const int elementNodeCount = gmshElementNodeCount(elementType);
        DART_SIMULATION_THROW_T_IF(
            elementNodeCount < 0,
            InvalidArgumentException,
            "Unsupported Gmsh element type {} in '{}'",
            elementType,
            path.string());
        for (std::size_t element = 0; element < elementCount; ++element) {
          (void)readRequired<std::uint64_t>(input, "$Elements element tag");
          std::array<std::size_t, 4> nodes{};
          for (int i = 0; i < elementNodeCount; ++i) {
            const auto tag
                = readRequired<std::uint64_t>(input, "$Elements node tag");
            const auto it = tagToIndex.find(tag);
            DART_SIMULATION_THROW_T_IF(
                it == tagToIndex.end(),
                InvalidArgumentException,
                "Gmsh element in '{}' references unknown node tag {}",
                path.string(),
                tag);
            if (i < 4) {
              nodes[static_cast<std::size_t>(i)] = it->second;
            }
          }
          if (elementType == 4) {
            mesh.tetrahedra.push_back({nodes[0], nodes[1], nodes[2], nodes[3]});
          }
        }
      }
    } else if (token == "$Surface") {
      const auto triangleCount
          = readRequired<std::size_t>(input, "$Surface count");
      mesh.surfaceTriangles.reserve(triangleCount);
      for (std::size_t i = 0; i < triangleCount; ++i) {
        std::array<std::size_t, 3> nodes{};
        for (auto& node : nodes) {
          const auto tag = readRequired<std::uint64_t>(input, "$Surface node");
          const auto it = tagToIndex.find(tag);
          DART_SIMULATION_THROW_T_IF(
              it == tagToIndex.end(),
              InvalidArgumentException,
              "Surface triangle in '{}' references unknown node tag {}",
              path.string(),
              tag);
          node = it->second;
        }
        mesh.surfaceTriangles.push_back({nodes[0], nodes[1], nodes[2]});
      }
    }
  }

  DART_SIMULATION_THROW_T_IF(
      mesh.positions.empty() || mesh.tetrahedra.empty(),
      InvalidArgumentException,
      "Deformable scene mesh '{}' must contain nodes and tetrahedra",
      path.string());
  return mesh;
}

//==============================================================================
void addBoundaryFace(
    std::map<
        std::array<std::size_t, 3>,
        std::pair<DeformableSurfaceTriangle, std::size_t>>& faces,
    DeformableSurfaceTriangle face)
{
  std::array<std::size_t, 3> key{face.nodeA, face.nodeB, face.nodeC};
  std::ranges::sort(key);
  auto [it, inserted] = faces.emplace(key, std::pair{face, 0u});
  ++it->second.second;
  DART_SIMULATION_THROW_T_IF(
      it->second.second > 2u,
      InvalidArgumentException,
      "Deformable scene mesh creates a nonmanifold surface face");
  if (!inserted && it->second.second == 2u) {
    it->second.first = face;
  }
}

//==============================================================================
std::vector<DeformableSurfaceTriangle> deriveSurfaceTriangles(
    const std::vector<DeformableTetrahedron>& tetrahedra)
{
  std::map<
      std::array<std::size_t, 3>,
      std::pair<DeformableSurfaceTriangle, std::size_t>>
      faces;
  for (const auto& tetrahedron : tetrahedra) {
    addBoundaryFace(
        faces, {tetrahedron.nodeA, tetrahedron.nodeC, tetrahedron.nodeB});
    addBoundaryFace(
        faces, {tetrahedron.nodeA, tetrahedron.nodeB, tetrahedron.nodeD});
    addBoundaryFace(
        faces, {tetrahedron.nodeA, tetrahedron.nodeD, tetrahedron.nodeC});
    addBoundaryFace(
        faces, {tetrahedron.nodeB, tetrahedron.nodeC, tetrahedron.nodeD});
  }

  std::vector<DeformableSurfaceTriangle> surfaceTriangles;
  for (const auto& [_, faceAndCount] : faces) {
    if (faceAndCount.second == 1u) {
      surfaceTriangles.push_back(faceAndCount.first);
    }
  }
  return surfaceTriangles;
}

//==============================================================================
std::vector<DeformableEdge> makeStructuralEdges(
    const std::vector<DeformableTetrahedron>& tetrahedra,
    const std::vector<DeformableSurfaceTriangle>& surfaceTriangles)
{
  std::set<std::array<std::size_t, 2>> edges;
  const auto addEdge = [&](std::size_t a, std::size_t b) {
    if (a > b) {
      std::swap(a, b);
    }
    edges.insert({a, b});
  };
  for (const auto& tetrahedron : tetrahedra) {
    addEdge(tetrahedron.nodeA, tetrahedron.nodeB);
    addEdge(tetrahedron.nodeA, tetrahedron.nodeC);
    addEdge(tetrahedron.nodeA, tetrahedron.nodeD);
    addEdge(tetrahedron.nodeB, tetrahedron.nodeC);
    addEdge(tetrahedron.nodeB, tetrahedron.nodeD);
    addEdge(tetrahedron.nodeC, tetrahedron.nodeD);
  }
  for (const auto& triangle : surfaceTriangles) {
    addEdge(triangle.nodeA, triangle.nodeB);
    addEdge(triangle.nodeB, triangle.nodeC);
    addEdge(triangle.nodeC, triangle.nodeA);
  }

  std::vector<DeformableEdge> result;
  result.reserve(edges.size());
  for (const auto& edge : edges) {
    result.push_back({edge[0], edge[1], -1.0});
  }
  return result;
}

//==============================================================================
bool containsPoint(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& minCorner,
    const Eigen::Vector3d& maxCorner)
{
  constexpr double tolerance = 1e-10;
  return (point.array() >= (minCorner.array() - tolerance)).all()
         && (point.array() <= (maxCorner.array() + tolerance)).all();
}

//==============================================================================
std::vector<std::uint8_t> makeSurfaceNodeMask(
    std::size_t nodeCount,
    const std::vector<DeformableSurfaceTriangle>& surfaceTriangles)
{
  std::vector<std::uint8_t> mask(nodeCount, 0u);
  for (const auto& triangle : surfaceTriangles) {
    mask[triangle.nodeA] = 1u;
    mask[triangle.nodeB] = 1u;
    mask[triangle.nodeC] = 1u;
  }
  return mask;
}

//==============================================================================
std::vector<std::size_t> selectSurfaceNodesInBox(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& surfaceNodeMask,
    const Eigen::Vector3d& minCorner,
    const Eigen::Vector3d& maxCorner)
{
  std::vector<std::size_t> nodes;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (surfaceNodeMask[i] != 0u
        && containsPoint(positions[i], minCorner, maxCorner)) {
      nodes.push_back(i);
    }
  }
  return nodes;
}

//==============================================================================
Eigen::Vector3d boxCenter(
    const Eigen::Vector3d& minCorner, const Eigen::Vector3d& maxCorner)
{
  return 0.5 * (minCorner + maxCorner);
}

//==============================================================================
void appendUnsupportedWarning(
    ParsedScene& scene, std::string_view token, bool ignoreDirective)
{
  DART_SIMULATION_THROW_T_IF(
      !ignoreDirective,
      InvalidArgumentException,
      "Deformable scene directive '{}' is outside the contact-free loader "
      "slice",
      token);
  scene.warnings.push_back(
      "ignored contact/friction directive '" + std::string(token) + "'");
}

//==============================================================================
bool isUnsupportedContactOrSolverDirective(std::string_view token)
{
  return token == "ground" || token == "halfSpace" || token == "meshCO"
         || token == "selfFric" || token == "selfCollisionOn"
         || token == "selfCollisionOff" || token == "dHat" || token == "epsv"
         || token == "constraintSolver" || token == "constraintType"
         || token == "QPSolver" || token == "CCDMethod"
         || token == "CCDTolerance" || token == "fricIterAmt"
         || token == "kappaMinMultiplier"
         || token == "minBarrierStiffnessScale";
}

//==============================================================================
void parseShapeExtra(
    std::istream& input,
    ShapeRecord& shape,
    ParsedScene& scene,
    const DeformableSceneLoadOptions& options)
{
  std::string token;
  while (input >> token) {
    if (token == "material") {
      DeformableMaterialProperties material;
      material.density = readRequired<double>(input, "material density");
      material.youngsModulus = readRequired<double>(input, "material E");
      material.poissonRatio = readRequired<double>(input, "material nu");
      shape.material = material;
    } else if (token == "linearVelocity") {
      shape.linearVelocity = readVector3(input, "linearVelocity");
      scene.warnings.push_back(
          "interpreted shape linearVelocity as initial velocity in "
          "contact-free replay");
    } else if (token == "angularVelocity") {
      shape.angularVelocity
          = readVector3(input, "angularVelocity") * kDegreesToRadians;
      scene.warnings.push_back(
          "interpreted shape angularVelocity as initial angular velocity in "
          "contact-free replay");
    } else if (token == "initVel") {
      shape.initialLinearVelocity = readVector3(input, "initVel linear");
      shape.initialAngularVelocity
          = readVector3(input, "initVel angular") * kDegreesToRadians;
    } else if (token == "DBC") {
      ShapeBoundaryDirichlet boundary;
      boundary.minCorner = readVector3(input, "DBC min");
      boundary.maxCorner = readVector3(input, "DBC max");
      boundary.linearVelocity = readVector3(input, "DBC linear velocity");
      boundary.angularVelocity
          = readVector3(input, "DBC angular velocity") * kDegreesToRadians;
      boundary.startTime = readOptionalDouble(input);
      boundary.endTime = readOptionalDouble(input);
      shape.dirichlet.push_back(boundary);
    } else if (token == "NBC") {
      ShapeBoundaryNeumann boundary;
      boundary.minCorner = readVector3(input, "NBC min");
      boundary.maxCorner = readVector3(input, "NBC max");
      boundary.acceleration = readVector3(input, "NBC acceleration");
      boundary.startTime = readOptionalDouble(input);
      boundary.endTime = readOptionalDouble(input);
      shape.neumann.push_back(boundary);
    } else if (token == "meshSeq") {
      std::string unusedPath;
      input >> unusedPath;
      appendUnsupportedWarning(scene, token, options.ignoreContactDirectives);
    } else {
      appendUnsupportedWarning(scene, token, options.ignoreContactDirectives);
    }
  }
}

//==============================================================================
ParsedScene parseSceneText(
    const std::filesystem::path& scenePath,
    const DeformableSceneLoadOptions& options)
{
  std::ifstream input(scenePath);
  DART_SIMULATION_THROW_T_IF(
      !input,
      InvalidArgumentException,
      "Unable to open deformable scene '{}'",
      scenePath.string());

  ParsedScene scene;
  std::size_t lineNumber = 0;
  while (true) {
    const auto line = readLogicalLine(input, lineNumber);
    if (line.empty()) {
      break;
    }

    std::istringstream stream(line);
    const auto token = readRequired<std::string>(stream, "scene token");
    if (token == "energy") {
      std::string unusedEnergy;
      stream >> unusedEnergy;
      scene.warnings.push_back("ignored scene metadata directive 'energy'");
    } else if (token == "timeIntegration") {
      std::string unusedIntegration;
      stream >> unusedIntegration;
      scene.warnings.push_back(
          "ignored scene metadata directive 'timeIntegration'");
    } else if (token == "density") {
      scene.density = readRequired<double>(stream, "density");
    } else if (token == "stiffness") {
      scene.youngsModulus = readRequired<double>(stream, "stiffness E");
      scene.poissonRatio = readRequired<double>(stream, "stiffness nu");
    } else if (token == "time") {
      scene.duration = readRequired<double>(stream, "duration");
      scene.timeStep = readRequired<double>(stream, "time step");
    } else if (token == "turnOffGravity") {
      scene.gravityEnabled = false;
    } else if (token == "DBCTimeRange") {
      scene.dirichletTimeRange[0]
          = readRequired<double>(stream, "DBCTimeRange start");
      scene.dirichletTimeRange[1]
          = readRequired<double>(stream, "DBCTimeRange end");
    } else if (token == "NBCTimeRange") {
      scene.neumannTimeRange[0]
          = readRequired<double>(stream, "NBCTimeRange start");
      scene.neumannTimeRange[1]
          = readRequired<double>(stream, "NBCTimeRange end");
    } else if (token == "shapes") {
      std::string type;
      stream >> type;
      DART_SIMULATION_THROW_T_IF(
          type != "input",
          InvalidArgumentException,
          "Only 'shapes input' deformable scenes are supported");
      const auto shapeCount = readRequired<std::size_t>(stream, "shape count");
      for (std::size_t i = 0; i < shapeCount; ++i) {
        const auto shapeLine = readLogicalLine(input, lineNumber);
        DART_SIMULATION_THROW_T_IF(
            shapeLine.empty(),
            InvalidArgumentException,
            "Expected shape record {} in deformable scene '{}'",
            i,
            scenePath.string());
        std::istringstream shapeStream(shapeLine);
        ShapeRecord shape;
        std::string rawPath;
        shapeStream >> rawPath;
        shape.meshPath
            = resolveScenePath(scenePath, options.assetRoot, rawPath);
        shape.translation = readVector3(shapeStream, "shape translation");
        shape.rotationDegrees = readVector3(shapeStream, "shape rotation");
        shape.scale = readVector3(shapeStream, "shape scale");
        parseShapeExtra(shapeStream, shape, scene, options);
        scene.shapes.push_back(std::move(shape));
      }
    } else if (isUnsupportedContactOrSolverDirective(token)) {
      appendUnsupportedWarning(scene, token, options.ignoreContactDirectives);
    } else {
      scene.warnings.push_back(
          "ignored unsupported scene directive '" + token + "'");
    }
  }

  DART_SIMULATION_THROW_T_IF(
      scene.shapes.empty(),
      InvalidArgumentException,
      "Deformable scene '{}' did not define any input shapes",
      scenePath.string());
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(scene.timeStep) || scene.timeStep <= 0.0,
      InvalidArgumentException,
      "Deformable scene time step must be positive and finite");
  return scene;
}

//==============================================================================
DeformableBodyOptions makeBodyOptions(
    const ParsedScene& scene,
    const ShapeRecord& shape,
    const DeformableSceneLoadOptions& loadOptions)
{
  auto mesh = loadGmshTetMesh(shape.meshPath);
  const auto rotation = rotationFromDegrees(shape.rotationDegrees);
  const auto localPositions = mesh.positions;

  for (auto& position : mesh.positions) {
    position
        = rotation * position.cwiseProduct(shape.scale) + shape.translation;
  }

  DeformableBodyOptions options;
  options.positions = std::move(mesh.positions);
  options.tetrahedra = std::move(mesh.tetrahedra);
  if (mesh.surfaceTriangles.empty()) {
    mesh.surfaceTriangles = deriveSurfaceTriangles(options.tetrahedra);
  }
  options.surfaceTriangles = std::move(mesh.surfaceTriangles);
  const auto surfaceNodeMask
      = makeSurfaceNodeMask(localPositions.size(), options.surfaceTriangles);
  options.edgeStiffness = loadOptions.structuralSpringStiffness;
  options.damping = loadOptions.damping;
  options.material.density = scene.density;
  options.material.youngsModulus = scene.youngsModulus;
  options.material.poissonRatio = scene.poissonRatio;
  if (shape.material.has_value()) {
    options.material = *shape.material;
  }

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& position : options.positions) {
    center += position;
  }
  center /= static_cast<double>(options.positions.size());
  const auto initialLinearVelocity = shape.initialLinearVelocity.value_or(
      shape.linearVelocity.value_or(Eigen::Vector3d::Zero()));
  const auto initialAngularVelocity = shape.initialAngularVelocity.value_or(
      shape.angularVelocity.value_or(Eigen::Vector3d::Zero()));
  options.velocities.reserve(options.positions.size());
  for (const auto& position : options.positions) {
    options.velocities.push_back(
        initialLinearVelocity
        + initialAngularVelocity.cross(position - center));
  }

  if (loadOptions.addStructuralSprings) {
    options.edges
        = makeStructuralEdges(options.tetrahedra, options.surfaceTriangles);
  }

  for (const auto& boundary : shape.dirichlet) {
    DeformableDirichletBoundaryCondition condition;
    condition.nodes = selectSurfaceNodesInBox(
        localPositions,
        surfaceNodeMask,
        boundary.minCorner,
        boundary.maxCorner);
    if (condition.nodes.empty()) {
      continue;
    }
    condition.linearVelocity = boundary.linearVelocity;
    condition.angularVelocity = boundary.angularVelocity;
    condition.center = rotation
                           * boxCenter(boundary.minCorner, boundary.maxCorner)
                                 .cwiseProduct(shape.scale)
                       + shape.translation;
    condition.startTime
        = boundary.startTime.value_or(scene.dirichletTimeRange[0]);
    condition.endTime = boundary.endTime.value_or(scene.dirichletTimeRange[1]);
    options.dirichletBoundaryConditions.push_back(std::move(condition));
  }

  for (const auto& boundary : shape.neumann) {
    DeformableNeumannBoundaryCondition condition;
    condition.nodes = selectSurfaceNodesInBox(
        localPositions,
        surfaceNodeMask,
        boundary.minCorner,
        boundary.maxCorner);
    if (condition.nodes.empty()) {
      continue;
    }
    condition.acceleration = boundary.acceleration;
    condition.startTime
        = boundary.startTime.value_or(scene.neumannTimeRange[0]);
    condition.endTime = boundary.endTime.value_or(scene.neumannTimeRange[1]);
    options.neumannBoundaryConditions.push_back(std::move(condition));
  }

  return options;
}

} // namespace

//==============================================================================
DeformableSceneInfo loadDeformableScene(
    World& world,
    const std::filesystem::path& scenePath,
    const DeformableSceneLoadOptions& options)
{
  const auto scene = parseSceneText(scenePath, options);
  world.setTimeStep(scene.timeStep);
  world.setGravity(
      scene.gravityEnabled ? Eigen::Vector3d(0.0, -9.80665, 0.0)
                           : Eigen::Vector3d::Zero());

  DeformableSceneInfo info;
  info.duration = scene.duration;
  info.timeStep = scene.timeStep;
  info.gravityEnabled = scene.gravityEnabled;
  info.warnings = scene.warnings;

  for (std::size_t i = 0; i < scene.shapes.size(); ++i) {
    auto bodyOptions = makeBodyOptions(scene, scene.shapes[i], options);
    const auto name
        = options.bodyNamePrefix + "_" + std::to_string(info.bodies.size());
    auto body = world.addDeformableBody(name, bodyOptions);

    DeformableSceneBodyInfo bodyInfo;
    bodyInfo.name = name;
    bodyInfo.body = body;
    bodyInfo.nodeCount = bodyOptions.positions.size();
    bodyInfo.tetrahedronCount = bodyOptions.tetrahedra.size();
    bodyInfo.surfaceTriangleCount = bodyOptions.surfaceTriangles.size();
    bodyInfo.dirichletConditionCount
        = bodyOptions.dirichletBoundaryConditions.size();
    bodyInfo.neumannConditionCount
        = bodyOptions.neumannBoundaryConditions.size();
    info.bodies.push_back(bodyInfo);
  }

  return info;
}

//==============================================================================
DeformableSceneDiagnostics collectDeformableSceneDiagnostics(const World& world)
{
  DeformableSceneDiagnostics diagnostics;
  diagnostics.frame = world.getFrame();
  diagnostics.time = world.getTime();
  diagnostics.minZ = std::numeric_limits<double>::infinity();
  diagnostics.maxZ = -std::numeric_limits<double>::infinity();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableMeshTopology>();
  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& model = registry.get<comps::DeformableNodeModel>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    ++diagnostics.bodyCount;
    diagnostics.nodeCount += state.positions.size();
    diagnostics.tetrahedronCount += topology.tetrahedra.size();
    diagnostics.surfaceTriangleCount += topology.surfaceTriangles.size();

    for (std::size_t i = 0; i < state.positions.size(); ++i) {
      diagnostics.totalMass += model.masses[i];
      diagnostics.maxDisplacement = std::max(
          diagnostics.maxDisplacement,
          (state.positions[i] - topology.restPositions[i]).norm());
      diagnostics.minZ = std::min(diagnostics.minZ, state.positions[i].z());
      diagnostics.maxZ = std::max(diagnostics.maxZ, state.positions[i].z());
    }

    if (const auto* boundary
        = registry.try_get<comps::DeformableBoundaryConditions>(entity)) {
      diagnostics.dirichletConditionCount += boundary->dirichlet.size();
      diagnostics.neumannConditionCount += boundary->neumann.size();
    }
  }

  if (diagnostics.nodeCount == 0) {
    diagnostics.minZ = 0.0;
    diagnostics.maxZ = 0.0;
  }

  return diagnostics;
}

//==============================================================================
void writeDeformableSceneDiagnosticsJson(
    std::ostream& output, const DeformableSceneDiagnostics& diagnostics)
{
  output << std::setprecision(17);
  output << "{";
  output << "\"frame\":" << diagnostics.frame << ",";
  output << "\"time\":" << diagnostics.time << ",";
  output << "\"body_count\":" << diagnostics.bodyCount << ",";
  output << "\"node_count\":" << diagnostics.nodeCount << ",";
  output << "\"tetrahedron_count\":" << diagnostics.tetrahedronCount << ",";
  output << "\"surface_triangle_count\":" << diagnostics.surfaceTriangleCount
         << ",";
  output << "\"dirichlet_condition_count\":"
         << diagnostics.dirichletConditionCount << ",";
  output << "\"neumann_condition_count\":" << diagnostics.neumannConditionCount
         << ",";
  output << "\"total_mass\":" << diagnostics.totalMass << ",";
  output << "\"max_displacement\":" << diagnostics.maxDisplacement << ",";
  output << "\"min_z\":" << diagnostics.minZ << ",";
  output << "\"max_z\":" << diagnostics.maxZ;
  output << "}";
}

//==============================================================================
void saveDeformableSceneRestart(const World& world, std::ostream& output)
{
  world.saveBinary(output);
}

//==============================================================================
void loadDeformableSceneRestart(World& world, std::istream& input)
{
  world.loadBinary(input);
}

} // namespace dart::simulation::io
