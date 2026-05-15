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

#include "scenes.hpp"

#include <dart/all.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>
#include <dart/config.hpp>
#include <dart/io/read.hpp>
#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/package_resource_retriever.hpp>
#include <dart/utils/urdf/All.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace dart::gui::experimental::filament {

using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::HeightmapShaped;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::PointCloudShape;
using dart::dynamics::ShapePtr;
using dart::dynamics::ShapeNode;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;

#if DART_HAVE_OCTOMAP
using dart::dynamics::VoxelGridShape;
#endif

using dart::gui::experimental::OrbitCamera;
using dart::gui::experimental::makeRenderableId;
using dart::gui::experimental::normalizeRunOptions;
using dart::simulation::World;

std::filesystem::path resolveExamplePath(const std::string& path)
{
  const std::filesystem::path input(path);
  if (input.is_absolute() || std::filesystem::exists(input)) {
    return std::filesystem::absolute(input);
  }

  const std::filesystem::path sourcePath
      = std::filesystem::path(DART_FILAMENT_GUI_REPOSITORY_ROOT) / input;
  if (std::filesystem::exists(sourcePath)) {
    return std::filesystem::absolute(sourcePath);
  }

  return std::filesystem::absolute(input);
}

const char* sceneName(ExampleScene scene)
{
  switch (scene) {
    case ExampleScene::Mvp:
      return "mvp";
    case ExampleScene::DragAndDrop:
      return "drag-and-drop";
    case ExampleScene::Polyhedron:
      return "polyhedron";
    case ExampleScene::Heightmap:
      return "heightmap";
    case ExampleScene::G1:
      return "g1";
  }
  return "mvp";
}

bool parseSceneName(std::string_view name, ExampleScene& scene)
{
  if (name == "mvp") {
    scene = ExampleScene::Mvp;
    return true;
  }
  if (name == "drag-and-drop") {
    scene = ExampleScene::DragAndDrop;
    return true;
  }
  if (name == "polyhedron") {
    scene = ExampleScene::Polyhedron;
    return true;
  }
  if (name == "heightmap") {
    scene = ExampleScene::Heightmap;
    return true;
  }
  if (name == "g1") {
    scene = ExampleScene::G1;
    return true;
  }
  return false;
}

OrbitCamera initialCameraForScene(ExampleScene scene)
{
  OrbitCamera camera;
  switch (scene) {
    case ExampleScene::DragAndDrop:
      camera.target = Eigen::Vector3d(0.35, 0.15, 0.9);
      camera.yaw = -0.72;
      camera.pitch = 0.58;
      camera.distance = 9.5;
      break;
    case ExampleScene::Polyhedron:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.45);
      camera.yaw = -0.78;
      camera.pitch = 0.42;
      camera.distance = 3.0;
      break;
    case ExampleScene::Heightmap:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.25);
      camera.yaw = -0.74;
      camera.pitch = 0.48;
      camera.distance = 4.3;
      break;
    case ExampleScene::G1:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.85);
      camera.yaw = -0.78;
      camera.pitch = 0.24;
      camera.distance = 3.4;
      break;
    case ExampleScene::Mvp:
      camera.target = Eigen::Vector3d(0.15, 0.55, 0.75);
      camera.yaw = -0.95;
      camera.pitch = 0.38;
      camera.distance = 7.2;
      break;
  }
  return camera;
}

std::shared_ptr<dart::math::TriMesh<double>> createTetraMesh()
{
  auto mesh = std::make_shared<dart::math::TriMesh<double>>();
  mesh->reserveVertices(4);
  mesh->reserveTriangles(4);
  mesh->addVertex(-0.45, -0.35, -0.25);
  mesh->addVertex(0.45, -0.35, -0.25);
  mesh->addVertex(0.0, 0.45, -0.25);
  mesh->addVertex(0.0, 0.0, 0.55);
  mesh->addTriangle(0, 2, 1);
  mesh->addTriangle(0, 1, 3);
  mesh->addTriangle(1, 2, 3);
  mesh->addTriangle(2, 0, 3);
  mesh->computeVertexNormals();
  return mesh;
}

std::array<Eigen::Vector3d, 8> polyhedronFixtureVertices()
{
  return {Eigen::Vector3d(-0.5, -0.5, 0.0),
          Eigen::Vector3d(0.5, -0.5, 0.2),
          Eigen::Vector3d(0.6, 0.5, 0.1),
          Eigen::Vector3d(-0.4, 0.6, 0.15),
          Eigen::Vector3d(-0.2, -0.2, 0.9),
          Eigen::Vector3d(0.35, -0.3, 0.8),
          Eigen::Vector3d(0.4, 0.35, 0.75),
          Eigen::Vector3d(-0.35, 0.4, 0.7)};
}

std::shared_ptr<dart::math::TriMesh<double>> createPolyhedronMesh()
{
  auto mesh = std::make_shared<dart::math::TriMesh<double>>();
  const auto vertices = polyhedronFixtureVertices();
  mesh->reserveVertices(vertices.size());
  mesh->reserveTriangles(12);
  for (const auto& vertex : vertices) {
    mesh->addVertex(vertex);
  }

  mesh->addTriangle(0, 2, 1);
  mesh->addTriangle(0, 3, 2);
  mesh->addTriangle(4, 5, 6);
  mesh->addTriangle(4, 6, 7);
  mesh->addTriangle(0, 1, 5);
  mesh->addTriangle(0, 5, 4);
  mesh->addTriangle(1, 2, 6);
  mesh->addTriangle(1, 6, 5);
  mesh->addTriangle(2, 3, 7);
  mesh->addTriangle(2, 7, 6);
  mesh->addTriangle(3, 0, 4);
  mesh->addTriangle(3, 4, 7);
  mesh->computeVertexNormals();
  return mesh;
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createPolyhedronWireframe()
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(2.0f);
  const auto vertices = polyhedronFixtureVertices();
  for (const auto& vertex : vertices) {
    shape->addVertex(vertex);
  }
  while (!shape->getConnections().empty()) {
    shape->removeConnection(0);
  }

  const std::array<std::pair<std::size_t, std::size_t>, 12> edges{{
      {0, 1},
      {1, 2},
      {2, 3},
      {3, 0},
      {4, 5},
      {5, 6},
      {6, 7},
      {7, 4},
      {0, 4},
      {1, 5},
      {2, 6},
      {3, 7},
  }};
  for (const auto& edge : edges) {
    shape->addConnection(edge.first, edge.second);
  }
  return shape;
}

std::shared_ptr<HeightmapShaped> createMvpHeightmapShape()
{
  auto shape = std::make_shared<HeightmapShaped>();
  const std::array<double, 12> heights{
      0.02, 0.10, 0.06, 0.12, 0.08, 0.18,
      0.14, 0.05, 0.10, 0.24, 0.16, 0.08};
  shape->setHeightField(4u, 3u, heights);
  shape->setScale(Eigen::Vector3d(0.18, 0.18, 1.0));
  return shape;
}

std::shared_ptr<HeightmapShaped> createHeightmapExampleShape()
{
  auto shape = std::make_shared<HeightmapShaped>();
  constexpr std::size_t xResolution = 7;
  constexpr std::size_t yResolution = 5;
  std::vector<double> heights;
  heights.reserve(xResolution * yResolution);
  for (std::size_t y = 0; y < yResolution; ++y) {
    for (std::size_t x = 0; x < xResolution; ++x) {
      const double xPhase = static_cast<double>(x) * 0.75;
      const double yPhase = static_cast<double>(y) * 0.9;
      const double ridge = (x == 3 || y == 2) ? 0.08 : 0.0;
      heights.push_back(
          0.08 + ridge + 0.08 * std::sin(xPhase) * std::cos(yPhase));
    }
  }
  shape->setHeightField(xResolution, yResolution, heights);
  shape->setScale(Eigen::Vector3d(0.32, 0.32, 1.0));
  return shape;
}

std::shared_ptr<MeshShape> loadExampleMeshShape(
    const std::string& path, const Eigen::Vector3d& scale)
{
  const std::filesystem::path resolvedPath = resolveExamplePath(path);
  dart::utils::MeshLoaderd loader;
  auto mesh = loader.load(resolvedPath.string());
  if (!mesh) {
    std::cerr << "Failed to load example mesh: " << resolvedPath.string()
              << "\n";
    return nullptr;
  }

  auto sharedMesh = std::shared_ptr<dart::math::TriMesh<double>>(
      std::move(mesh));
  const auto uri
      = dart::common::Uri::createFromStringOrPath(resolvedPath.string());
  return std::make_shared<MeshShape>(scale, std::move(sharedMesh), uri);
}

std::shared_ptr<MeshShape> loadRequiredExampleMeshShape(
    const std::string& path, const Eigen::Vector3d& scale)
{
  auto meshShape = loadExampleMeshShape(path, scale);
  if (!meshShape) {
    throw std::runtime_error("Failed to load required mesh fixture: " + path);
  }
  return meshShape;
}

void disableSkeletonCollisionAndGravity(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return;
  }

  skeleton->disableSelfCollisionCheck();
  skeleton->setAdjacentBodyCheck(false);
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->setCollidable(false);
    body->setGravityMode(false);
    body->eachShapeNodeWith<CollisionAspect>([](ShapeNode* shapeNode) {
      shapeNode->getCollisionAspect()->setCollidable(false);
    });
  }
}

void makeVisualOnlySkeleton(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return;
  }

  skeleton->setMobile(false);
  disableSkeletonCollisionAndGravity(skeleton);
}

dart::dynamics::SkeletonPtr loadRequiredWamRobotSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      "herb_description", dart::config::dataPath("urdf/wam"));
  const auto wamUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("urdf/wam/wam.urdf"));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (!wam) {
    throw std::runtime_error(
        "Failed to load WAM robot fixture from " + wamUri.toString());
  }

  wam->setName(kWamFixtureSkeletonName);
  const std::array<std::pair<const char*, double>, 7> jointPositions{
      { {"/j1", 0.0},
        {"/j2", -0.55},
        {"/j3", 0.25},
        {"/j4", 1.05},
        {"/j5", 0.0},
        {"/j6", 0.7},
        {"/j7", 0.0} }};
  for (const auto& [name, position] : jointPositions) {
    auto* dof = wam->getDof(name);
    if (dof == nullptr) {
      throw std::runtime_error(
          "WAM robot fixture is missing expected DOF " + std::string(name));
    }
    dof->setPosition(position);
  }

  makeVisualOnlySkeleton(wam);
  return wam;
}

dart::dynamics::SkeletonPtr loadRequiredAtlasRobotSkeleton()
{
  const auto atlasUri = dart::common::Uri::createFromString(
      "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (!atlas) {
    throw std::runtime_error(
        "Failed to load Atlas robot fixture from " + atlasUri.toString());
  }

  atlas->setName(kAtlasRobotFixtureSkeletonName);
  auto* rootBody = atlas->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<FreeJoint*>(rootBody->getParentJoint()) != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation() = Eigen::Vector3d(0.35, 1.8, 0.95);
    FreeJoint::setTransformOf(rootBody, transform);
  }

  makeVisualOnlySkeleton(atlas);
  return atlas;
}

dart::dynamics::SkeletonPtr createRequiredPbrEnvironmentSkeleton()
{
  auto environment = Skeleton::create(kPbrEnvironmentFixtureSkeletonName);
  auto* body = environment->createJointAndBodyNodePair<WeldJoint>().second;
  auto pbrPanelShape = loadRequiredExampleMeshShape(
      "data/gltf/pbr_multi_material.gltf", Eigen::Vector3d(0.85, 0.85, 0.85));

  const auto addPanel = [&](const std::string& name,
                            const Eigen::Isometry3d& transform) {
    auto* shapeNode = body->createShapeNodeWith<VisualAspect>(pbrPanelShape);
    shapeNode->setName(name);
    shapeNode->setRelativeTransform(transform);
    shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d::Ones());
  };

  constexpr double halfPi = 1.5707963267948966;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-2.7, 2.0, 0.12);
  addPanel("pbr_environment_floor_left", transform);

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-1.15, 2.0, 0.12);
  addPanel("pbr_environment_floor_right", transform);

  transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(-2.7, 2.9, 1.05);
  addPanel("pbr_environment_backdrop_left", transform);

  transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(-1.15, 2.9, 1.05);
  addPanel("pbr_environment_backdrop_right", transform);

  return environment;
}

std::optional<std::string> getLastPathSegment(std::string value)
{
  if (value.empty()) {
    return std::nullopt;
  }

  const std::size_t terminator = value.find_first_of("?#");
  if (terminator != std::string::npos) {
    value.erase(terminator);
  }

  while (value.ends_with('/') || value.ends_with('\\')) {
    value.pop_back();
  }

  if (value.empty()) {
    return std::nullopt;
  }

  const std::size_t slash = value.find_last_of("/\\");
  std::string segment
      = value.substr(slash == std::string::npos ? 0 : slash + 1);

  if (segment.empty()) {
    return std::nullopt;
  }

  return segment;
}

std::optional<std::string> inferPackageNameFromRobotUri(
    const std::string& robotUri)
{
  if (robotUri.empty()) {
    return std::nullopt;
  }

  dart::common::Uri uri;
  if (!uri.fromStringOrPath(robotUri)) {
    return std::nullopt;
  }

  if (!uri.mScheme || *uri.mScheme != "package" || !uri.mAuthority) {
    return std::nullopt;
  }

  return uri.mAuthority.get();
}

std::optional<std::string> inferPackageNameFromPackageUri(
    const std::string& packageUri)
{
  if (packageUri.empty()) {
    return std::nullopt;
  }

  dart::common::Uri uri;
  if (uri.fromStringOrPath(packageUri)) {
    if (uri.mScheme && *uri.mScheme == "package" && uri.mAuthority) {
      return uri.mAuthority.get();
    }

    if (uri.mPath) {
      if (auto segment = getLastPathSegment(uri.mPath.get())) {
        return segment;
      }
    }
  }

  return getLastPathSegment(packageUri);
}

dart::common::ResourceRetrieverPtr createG1ResourceRetriever(
    const AppOptions& options)
{
  auto local = std::make_shared<dart::common::LocalResourceRetriever>();
  auto dartRetriever = std::make_shared<dart::utils::DartResourceRetriever>();
  auto http = std::make_shared<dart::utils::HttpResourceRetriever>();

  auto passthrough = std::make_shared<dart::utils::CompositeResourceRetriever>();
  passthrough->addSchemaRetriever("file", local);
  passthrough->addSchemaRetriever("dart", dartRetriever);
  passthrough->addSchemaRetriever("http", http);
  passthrough->addSchemaRetriever("https", http);
  passthrough->addDefaultRetriever(local);

  auto packageRetriever
      = std::make_shared<dart::utils::PackageResourceRetriever>(passthrough);
  packageRetriever->addPackageDirectory(
      options.g1PackageName, options.g1PackageUri);

  auto resolver = std::make_shared<dart::utils::CompositeResourceRetriever>();
  resolver->addSchemaRetriever("package", packageRetriever);
  resolver->addSchemaRetriever("file", local);
  resolver->addSchemaRetriever("dart", dartRetriever);
  resolver->addSchemaRetriever("http", http);
  resolver->addSchemaRetriever("https", http);
  resolver->addDefaultRetriever(local);

  return resolver;
}

dart::dynamics::SkeletonPtr createG1Ground()
{
  auto ground = Skeleton::create("g1_ground");
  auto* body = ground->createJointAndBodyNodePair<WeldJoint>().second;

  constexpr double thickness = 0.04;
  auto* shapeNode = body->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(4.0, 4.0, thickness)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.86, 0.88, 0.9, 1.0));
  return ground;
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return std::nullopt;
  }

  bool hasBounds = false;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();

  const auto includePoint = [&](const Eigen::Vector3d& point) {
    if (!hasBounds) {
      min = point;
      max = point;
      hasBounds = true;
      return;
    }
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  };

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->eachShapeNodeWith<VisualAspect>([&](const ShapeNode* shapeNode) {
      if (shapeNode == nullptr || shapeNode->getShape() == nullptr
          || shapeNode->getVisualAspect()->isHidden()) {
        return;
      }

      const auto& bounds = shapeNode->getShape()->getBoundingBox();
      const Eigen::Vector3d localMin = bounds.getMin();
      const Eigen::Vector3d localMax = bounds.getMax();
      if (!localMin.allFinite() || !localMax.allFinite()) {
        return;
      }

      const Eigen::Isometry3d transform = shapeNode->getWorldTransform();
      for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
          for (int z = 0; z < 2; ++z) {
            includePoint(transform * Eigen::Vector3d(
                                         x == 0 ? localMin.x() : localMax.x(),
                                         y == 0 ? localMin.y() : localMax.y(),
                                         z == 0 ? localMin.z() : localMax.z()));
          }
        }
      }
    });
  }

  if (!hasBounds) {
    return std::nullopt;
  }

  return std::make_pair(min, max);
}

dart::dynamics::SkeletonPtr loadG1Skeleton(const AppOptions& options)
{
  dart::io::ReadOptions readOptions;
  readOptions.resourceRetriever = createG1ResourceRetriever(options);
  const dart::common::Uri robotUri(options.g1RobotUri);
  auto robot = dart::io::readSkeleton(robotUri, readOptions);
  if (!robot) {
    throw std::runtime_error(
        "Failed to load G1 robot fixture from " + options.g1RobotUri);
  }

  robot->setName(kG1FixtureSkeletonName);
  if (auto* rootBody = robot->getRootBodyNode()) {
    if (auto* freeJoint
        = dynamic_cast<FreeJoint*>(rootBody->getParentJoint())) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      FreeJoint::setTransformOf(freeJoint, transform);
      if (const auto bounds = computeVisualWorldBounds(robot)) {
        constexpr double g1GroundClearance = 0.015;
        transform.translation().z() = g1GroundClearance - bounds->first.z();
        FreeJoint::setTransformOf(freeJoint, transform);
      }
    }
  }
  disableSkeletonCollisionAndGravity(robot);
  return robot;
}

dart::math::SupportGeometry makeG1FootSupportGeometry()
{
  dart::math::SupportGeometry geometry;
  geometry.emplace_back(Eigen::Vector3d(0.12, 0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, 0.06, 0.0));
  return geometry;
}

void addG1IkTargets(
    DartScene& scene, const dart::dynamics::SkeletonPtr& robot)
{
  struct Config
  {
    const char* bodyNode;
    const char* targetName;
    const char* label;
    int hotkey;
    Eigen::Vector4d color;
    bool supportContact;
  };

  const auto footSupportGeometry = makeG1FootSupportGeometry();
  const std::array<Config, 4> configs{{
      {"left_rubber_hand",
       "ik_target_left_hand",
       "1 left hand",
       '1',
       {0.18, 0.55, 1.0, 0.92},
       false},
      {"right_rubber_hand",
       "ik_target_right_hand",
       "2 right hand",
       '2',
       {1.0, 0.40, 0.24, 0.92},
       false},
      {"left_ankle_roll_link",
       "ik_target_left_foot",
       "3 left foot",
       '3',
       {0.26, 0.86, 0.34, 0.92},
       true},
      {"right_ankle_roll_link",
       "ik_target_right_foot",
       "4 right foot",
       '4',
       {0.95, 0.72, 0.18, 0.92},
       true},
  }};

  if (!scene.world || !robot) {
    return;
  }

  for (const Config& config : configs) {
    auto* bodyNode = robot->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      std::cerr << "Unable to find G1 body node '" << config.bodyNode
                << "' for IK target.\n";
      continue;
    }

    auto* endEffector = bodyNode->createEndEffector(
        std::string(config.targetName) + "_effector");
    if (config.supportContact) {
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    }

    auto ik = endEffector->getIK(true);
    ik->setGradientMethod<InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);

    auto target = SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    target->setShape(std::make_shared<SphereShape>(0.055));
    target->getVisualAspect(true)->setRGBA(config.color);
    scene.world->addSimpleFrame(target);
    ik->setTarget(target);

    G1IkHandle handle;
    handle.targetRenderableId = makeRenderableId(*target);
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = std::move(target);
    handle.ik = std::move(ik);
    scene.ikHandles.push_back(std::move(handle));
  }
}

AppOptions parseOptions(int argc, char* argv[])
{
  AppOptions options;
  bool g1PackageNameExplicit = false;
  bool g1PackageUriExplicit = false;
  bool g1RobotUriExplicit = false;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--frames" && i + 1 < argc) {
      options.run.maxFrames = std::atoi(argv[++i]);
    } else if (arg == "--width" && i + 1 < argc) {
      options.run.width = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--height" && i + 1 < argc) {
      options.run.height = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--screenshot" && i + 1 < argc) {
      options.run.screenshotPath = argv[++i];
    } else if (arg == "--headless") {
      options.run.headless = true;
    } else if (arg == "--hide-ui") {
      options.showUi = false;
      options.showUiExplicit = true;
    } else if (arg == "--show-ui") {
      options.showUi = true;
      options.showUiExplicit = true;
    } else if (arg == "--orbit-light") {
      options.orbitLight = true;
    } else if (arg == "--no-orbit-light") {
      options.orbitLight = false;
    } else if (arg == "--orbit-light-period" && i + 1 < argc) {
      char* end = nullptr;
      const char* value = argv[++i];
      const double orbitLightPeriodSeconds = std::strtod(value, &end);
      if (end == value || *end != '\0' || !std::isfinite(orbitLightPeriodSeconds)
          || orbitLightPeriodSeconds <= 0.0) {
        std::cerr << "Invalid --orbit-light-period value '" << value
                  << "'. Expected a positive number of seconds.\n";
        std::exit(2);
      }
      options.orbitLightPeriodSeconds = orbitLightPeriodSeconds;
    } else if (arg == "--gui-scale" && i + 1 < argc) {
      char* end = nullptr;
      const char* value = argv[++i];
      const float guiScale = std::strtof(value, &end);
      if (end == value || *end != '\0' || !std::isfinite(guiScale)
          || guiScale <= 0.0f) {
        std::cerr << "Invalid --gui-scale value '" << value
                  << "'. Expected a positive number.\n";
        std::exit(2);
      }
      options.run.guiScale
          = std::clamp(static_cast<double>(guiScale), 0.5, 4.0);
    } else if (arg == "--profile") {
      options.profile = true;
    } else if (arg == "--scene" && i + 1 < argc) {
      const std::string_view sceneArg(argv[++i]);
      if (!parseSceneName(sceneArg, options.scene)) {
        std::cerr << "Unknown scene '" << sceneArg
                  << "'. Expected 'mvp', 'drag-and-drop', 'polyhedron', "
                     "'heightmap', or 'g1'.\n";
        std::exit(2);
      }
    } else if (
        (arg == "--g1-package-uri" || arg == "--package-uri")
        && i + 1 < argc) {
      options.g1PackageUri = argv[++i];
      g1PackageUriExplicit = true;
    } else if (
        (arg == "--g1-robot-uri" || arg == "--robot-uri") && i + 1 < argc) {
      options.g1RobotUri = argv[++i];
      g1RobotUriExplicit = true;
    } else if (
        (arg == "--g1-package-name" || arg == "--package-name")
        && i + 1 < argc) {
      options.g1PackageName = argv[++i];
      g1PackageNameExplicit = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--frames N] [--width N] [--height N]"
                   " [--screenshot PATH] [--headless]"
                   " [--hide-ui|--show-ui]"
                   " [--orbit-light|--no-orbit-light]"
                   " [--orbit-light-period SECONDS]"
                   " [--gui-scale N]"
                   " [--profile]"
                   " [--scene mvp|drag-and-drop|polyhedron|heightmap|g1]"
                   " [--g1-package-uri URI] [--g1-robot-uri URI]"
                   " [--g1-package-name NAME]\n";
      std::exit(0);
    }
  }

  if (!g1PackageNameExplicit) {
    if (g1RobotUriExplicit) {
      if (auto packageName
          = inferPackageNameFromRobotUri(options.g1RobotUri)) {
        options.g1PackageName = *packageName;
      }
    } else if (g1PackageUriExplicit) {
      if (auto packageName
          = inferPackageNameFromPackageUri(options.g1PackageUri)) {
        options.g1PackageName = *packageName;
      }
    } else if (auto packageName
               = inferPackageNameFromRobotUri(options.g1RobotUri)) {
      options.g1PackageName = *packageName;
    }
  }

  normalizeRunOptions(options.run);
  if (options.run.guiScale != 1.0) {
    options.run.width = std::max(
        1,
        static_cast<int>(std::lround(
            static_cast<double>(options.run.width) * options.run.guiScale)));
    options.run.height = std::max(
        1,
        static_cast<int>(std::lround(
            static_cast<double>(options.run.height) * options.run.guiScale)));
  }
  if (options.run.headless && !options.showUiExplicit) {
    options.showUi = false;
  }
  return options;
}

dart::dynamics::SkeletonPtr createStaticVisualSkeleton(
    const std::string& name,
    const ShapePtr& shape,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double alpha = 1.0)
{
  auto skeleton = Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<WeldJoint>().second;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  auto* shapeNode = body->createShapeNodeWith<VisualAspect>(shape);
  shapeNode->setRelativeTransform(transform);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(color.x(), color.y(), color.z(), alpha));
  return skeleton;
}

DartScene createMvpDartScene()
{
  DartScene scene;

  auto createDynamicBox = [](const std::string& name,
                             const Eigen::Vector3d& size,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& color) {
    auto box = Skeleton::create(name);
    auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>();
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = position;
    joint->setTransformFromParentBodyNode(tf);

    auto boxShape = std::make_shared<BoxShape>(size);
    auto boxShapeNode = body->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(boxShape);
    boxShapeNode->getVisualAspect()->setColor(color);
    body->setInertia(
        dart::dynamics::Inertia(
            1.0,
            Eigen::Vector3d::Zero(),
            boxShapeNode->getShape()->computeInertia(1.0)));
    return box;
  };

  auto blueBox = createDynamicBox(
      "falling_blue_box",
      Eigen::Vector3d(0.35, 0.35, 0.35),
      Eigen::Vector3d(-0.35, 0.0, 1.5),
      dart::Color::Blue());
  auto orangeBox = createDynamicBox(
      "falling_orange_box",
      Eigen::Vector3d(0.28, 0.28, 0.28),
      Eigen::Vector3d(0.35, 0.1, 2.15),
      dart::Color::Orange());
  auto contactBox = createDynamicBox(
      "contact_green_box",
      Eigen::Vector3d(0.3, 0.3, 0.3),
      Eigen::Vector3d(-1.05, 0.25, 0.18),
      dart::Color::Green());

  auto ground = Skeleton::create("ground");
  auto groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
  auto groundShapeNode = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(8.0, 8.0, 0.1)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  auto createStaticVisual = [](const std::string& name,
                               const ShapePtr& shape,
                               const Eigen::Vector3d& position,
                               const Eigen::Vector3d& color,
                               double alpha = 1.0) {
    return createStaticVisualSkeleton(name, shape, position, color, alpha);
  };

  auto createLineSegmentShape = [] {
    auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(
        Eigen::Vector3d(-0.35, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.25, 0.25),
        2.0f);
    shape->addVertex(Eigen::Vector3d(0.35, 0.0, 0.0), 1);
    shape->addVertex(Eigen::Vector3d(0.0, -0.25, 0.25), 0);
    return shape;
  };

  auto createPointCloudShape = [] {
    auto shape = std::make_shared<PointCloudShape>(0.14);
    shape->setPointShapeType(PointCloudShape::BOX);
    shape->setColorMode(PointCloudShape::BIND_PER_POINT);
    shape->addPoint(Eigen::Vector3d(-0.24, -0.18, 0.0));
    shape->addPoint(Eigen::Vector3d(0.0, 0.16, 0.12));
    shape->addPoint(Eigen::Vector3d(0.24, -0.14, 0.02));
    shape->addPoint(Eigen::Vector3d(-0.02, -0.02, 0.3));
    const std::array<Eigen::Vector4d, 4> pointColors{
        Eigen::Vector4d(0.95, 0.25, 0.20, 1.0),
        Eigen::Vector4d(0.30, 0.70, 1.00, 0.85),
        Eigen::Vector4d(0.98, 0.80, 0.25, 1.0),
        Eigen::Vector4d(0.50, 0.90, 0.38, 0.75)};
    shape->setColors(pointColors);
    return shape;
  };

#if DART_HAVE_OCTOMAP
  auto createVoxelGridShape = [] {
    auto shape = std::make_shared<VoxelGridShape>(0.12);
    const std::array<Eigen::Vector3d, 7> occupiedVoxels{
        Eigen::Vector3d(-0.18, -0.06, 0.0),
        Eigen::Vector3d(-0.06, -0.06, 0.0),
        Eigen::Vector3d(0.06, -0.06, 0.0),
        Eigen::Vector3d(0.18, -0.06, 0.0),
        Eigen::Vector3d(-0.06, 0.06, 0.12),
        Eigen::Vector3d(0.06, 0.06, 0.12),
        Eigen::Vector3d(0.0, 0.18, 0.24)};
    for (const Eigen::Vector3d& voxel : occupiedVoxels) {
      shape->updateOccupancy(voxel);
    }
    return shape;
  };
#endif

  auto createSoftMeshSkeleton = [] {
    auto skeleton = Skeleton::create(kSoftMeshFixtureSkeletonName);
    auto [joint, softBody]
        = skeleton->createJointAndBodyNodePair<WeldJoint, SoftBodyNode>();
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(-0.05, 0.55, 0.9);
    joint->setTransformFromParentBodyNode(tf);

    SoftBodyNodeHelper::setBox(
        softBody,
        Eigen::Vector3d(0.42, 0.32, 0.26),
        Eigen::Isometry3d::Identity(),
        Eigen::Vector3i(3, 3, 3),
        1.0,
        10.0,
        10.0,
        0.1);

    for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
      auto* pointMass = softBody->getPointMass(i);
      if (pointMass == nullptr) {
        continue;
      }
      const Eigen::Vector3d& resting = pointMass->getRestingPosition();
      pointMass->setPositions(Eigen::Vector3d(
          0.0,
          0.0,
          0.05
              * std::sin(7.0 * resting.x() + 5.0 * resting.y()
                         + static_cast<double>(i) * 0.3)));
    }

    for (std::size_t i = 0; i < softBody->getNumShapeNodes(); ++i) {
      auto* shapeNode = softBody->getShapeNode(i);
      if (shapeNode == nullptr || shapeNode->getVisualAspect() == nullptr) {
        continue;
      }
      shapeNode->getVisualAspect()->setRGBA(
          Eigen::Vector4d(0.88, 0.44, 0.72, 0.82));
    }

    return skeleton;
  };

  scene.world = World::create("filament_gui_mvp");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(blueBox);
  scene.world->addSkeleton(orangeBox);
  scene.world->addSkeleton(contactBox);
  scene.world->addSkeleton(ground);
  scene.world->addSkeleton(createStaticVisual(
      "visual_sphere",
      std::make_shared<dart::dynamics::SphereShape>(0.3),
      Eigen::Vector3d(0.0, 0.0, 2.7),
      Eigen::Vector3d(0.2, 0.72, 0.55)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_cylinder",
      std::make_shared<dart::dynamics::CylinderShape>(0.22, 0.7),
      Eigen::Vector3d(-0.75, -0.15, 0.8),
      Eigen::Vector3d(0.78, 0.58, 0.18)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_capsule",
      std::make_shared<dart::dynamics::CapsuleShape>(0.18, 0.62),
      Eigen::Vector3d(0.0, -0.15, 0.85),
      Eigen::Vector3d(0.72, 0.28, 0.68)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_cone",
      std::make_shared<dart::dynamics::ConeShape>(0.25, 0.65),
      Eigen::Vector3d(0.75, -0.15, 0.8),
      Eigen::Vector3d(0.92, 0.38, 0.2)));
  scene.world->addSkeleton(createStaticVisual(
      kPyramidFixtureSkeletonName,
      std::make_shared<dart::dynamics::PyramidShape>(0.55, 0.45, 0.7),
      Eigen::Vector3d(-2.15, -0.15, 0.82),
      Eigen::Vector3d(0.9, 0.72, 0.24)));
  scene.world->addSkeleton(createStaticVisual(
      kMultiSphereFixtureSkeletonName,
      std::make_shared<dart::dynamics::MultiSphereConvexHullShape>(
          dart::dynamics::MultiSphereConvexHullShape::Spheres{
              {0.18, Eigen::Vector3d(-0.22, 0.0, 0.0)},
              {0.26, Eigen::Vector3d(0.16, 0.0, 0.04)},
              {0.14, Eigen::Vector3d(0.42, 0.0, -0.03)}}),
      Eigen::Vector3d(-2.85, -0.15, 0.72),
      Eigen::Vector3d(0.38, 0.74, 0.92)));
  scene.world->addSkeleton(createStaticVisual(
      kLineSegmentFixtureSkeletonName,
      createLineSegmentShape(),
      Eigen::Vector3d(-2.85, 0.55, 0.88),
      Eigen::Vector3d(0.96, 0.68, 0.22)));
  scene.world->addSkeleton(createStaticVisual(
      kConvexMeshFixtureSkeletonName,
      std::make_shared<ConvexMeshShape>(createTetraMesh()),
      Eigen::Vector3d(-2.15, 0.55, 0.95),
      Eigen::Vector3d(0.48, 0.86, 0.38)));
  scene.world->addSkeleton(createStaticVisual(
      kPointCloudFixtureSkeletonName,
      createPointCloudShape(),
      Eigen::Vector3d(-1.45, 0.55, 0.82),
      Eigen::Vector3d(0.48, 0.86, 0.38)));
  scene.world->addSkeleton(createStaticVisual(
      kHeightmapFixtureSkeletonName,
      createMvpHeightmapShape(),
      Eigen::Vector3d(-0.75, 0.55, 0.65),
      Eigen::Vector3d(0.42, 0.74, 0.36)));
  scene.world->addSkeleton(createSoftMeshSkeleton());
#if DART_HAVE_OCTOMAP
  scene.world->addSkeleton(createStaticVisual(
      kVoxelGridFixtureSkeletonName,
      createVoxelGridShape(),
      Eigen::Vector3d(0.55, 0.55, 0.72),
      Eigen::Vector3d(0.94, 0.55, 0.24),
      0.78));
#endif
  scene.world->addSkeleton(createStaticVisual(
      "visual_ellipsoid",
      std::make_shared<dart::dynamics::EllipsoidShape>(
          Eigen::Vector3d(0.7, 0.4, 0.32)),
      Eigen::Vector3d(1.5, -0.15, 0.8),
      Eigen::Vector3d(0.25, 0.48, 0.88),
      0.55));
  scene.world->addSkeleton(createStaticVisual(
      "visual_mesh",
      std::make_shared<MeshShape>(
          Eigen::Vector3d(0.8, 0.8, 0.8),
          createTetraMesh(),
          dart::common::Uri{}),
      Eigen::Vector3d(-1.5, -0.15, 0.9),
      Eigen::Vector3d(0.45, 0.4, 0.92)));
  scene.world->addSkeleton(createStaticVisual(
      kAtlasFixtureSkeletonName,
      loadRequiredExampleMeshShape(
          "data/sdf/atlas/utorso.dae", Eigen::Vector3d(0.75, 0.75, 0.75)),
      Eigen::Vector3d(2.2, 0.25, 1.1),
      Eigen::Vector3d(0.72, 0.72, 0.78)));
  if (auto wamBaseMesh = loadExampleMeshShape(
          "data/urdf/wam/meshes/wam/wam1.dae",
          Eigen::Vector3d(0.65, 0.65, 0.65))) {
    scene.world->addSkeleton(createStaticVisual(
        "visual_wam_textured_mesh",
        wamBaseMesh,
        Eigen::Vector3d(2.35, 0.55, 0.45),
        Eigen::Vector3d::Ones()));
  }
  scene.world->addSkeleton(loadRequiredWamRobotSkeleton());
  scene.world->addSkeleton(loadRequiredAtlasRobotSkeleton());
  if (auto pbrMesh = loadExampleMeshShape(
          "data/gltf/pbr_triangle.gltf",
          Eigen::Vector3d(0.9, 0.9, 0.9))) {
    scene.world->addSkeleton(createStaticVisual(
        "visual_gltf_pbr_mesh",
        pbrMesh,
        Eigen::Vector3d(2.2, -0.75, 0.35),
        Eigen::Vector3d::Ones()));
  }
  if (auto multiMaterialPbrMesh = loadExampleMeshShape(
          "data/gltf/pbr_multi_material.gltf",
          Eigen::Vector3d(0.5, 0.5, 0.5))) {
    scene.world->addSkeleton(createStaticVisual(
        "visual_gltf_multi_material_pbr_mesh",
        multiMaterialPbrMesh,
        Eigen::Vector3d(1.45, -1.05, 0.55),
        Eigen::Vector3d::Ones()));
  }
  scene.world->addSkeleton(createRequiredPbrEnvironmentSkeleton());
  scene.world->addSkeleton(createStaticVisual(
      "visual_plane",
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d(0.0, 2.65, 0.08),
      Eigen::Vector3d(0.25, 0.7, 0.78),
      0.65));

  Eigen::Isometry3d simpleFrameTransform = Eigen::Isometry3d::Identity();
  simpleFrameTransform.translation() = Eigen::Vector3d(-2.25, 1.05, 0.55);
  auto simpleFrame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      "visual_draggable_simple_frame",
      simpleFrameTransform);
  simpleFrame->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.35, 0.35, 0.35)));
  simpleFrame->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.95, 0.72, 0.18, 0.82));
  scene.world->addSimpleFrame(simpleFrame);

  return scene;
}

DartScene createDragAndDropScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_drag_and_drop");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(4.0, -4.0, 0.0);
  auto anchor = std::make_shared<SimpleFrame>(
      dart::dynamics::Frame::World(), "interactive frame", transform);
  anchor->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.45, 0.45, 0.45)));
  anchor->getVisualAspect(true)->setColor(Eigen::Vector3d(0.95, 0.7, 0.15));
  scene.world->addSimpleFrame(anchor);

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-4.0, 4.0, 0.0);
  auto draggable = anchor->spawnChildSimpleFrame("draggable", transform);
  draggable->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  draggable->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
  scene.world->addSimpleFrame(draggable);

  const auto addMarker = [&](const std::string& name,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& color) {
    Eigen::Isometry3d markerTransform = Eigen::Isometry3d::Identity();
    markerTransform.translation() = position;
    auto marker = std::make_shared<SimpleFrame>(
        dart::dynamics::Frame::World(), name, markerTransform);
    marker->setShape(
        std::make_shared<BoxShape>(Eigen::Vector3d(0.25, 0.25, 0.25)));
    marker->getVisualAspect(true)->setColor(color);
    scene.world->addSimpleFrame(marker);
  };

  addMarker("X", Eigen::Vector3d(8.0, 0.0, 0.0), Eigen::Vector3d(0.9, 0.0, 0.0));
  addMarker("Y", Eigen::Vector3d(0.0, 8.0, 0.0), Eigen::Vector3d(0.0, 0.9, 0.0));
  addMarker("Z", Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.9));

  return scene;
}

DartScene createPolyhedronScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_polyhedron");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kPolyhedronFixtureSkeletonName,
      std::make_shared<ConvexMeshShape>(createPolyhedronMesh()),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.1, 0.8, 0.6),
      0.5));
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kPolyhedronWireframeFixtureSkeletonName,
      createPolyhedronWireframe(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.05, 0.05, 0.05)));
  return scene;
}

DartScene createHeightmapScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_heightmap");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kHeightmapFixtureSkeletonName,
      createHeightmapExampleShape(),
      Eigen::Vector3d(-0.25, 0.0, 0.0),
      Eigen::Vector3d(0.24, 0.58, 0.88)));
  scene.world->addSkeleton(createStaticVisualSkeleton(
      "visual_heightmap_reference_box",
      std::make_shared<BoxShape>(Eigen::Vector3d(0.48, 0.48, 0.28)),
      Eigen::Vector3d(0.72, 0.0, 0.20),
      Eigen::Vector3d(0.20, 0.72, 0.28),
      0.48));

  int index = 0;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      scene.world->addSkeleton(createStaticVisualSkeleton(
          "visual_heightmap_sample_ball_" + std::to_string(index++),
          std::make_shared<SphereShape>(0.07),
          Eigen::Vector3d(
              -0.25 + static_cast<double>(x) * 0.42,
              static_cast<double>(y) * 0.32,
              0.45),
          Eigen::Vector3d(0.92, 0.48, 0.16)));
    }
  }
  return scene;
}

DartScene createG1DartScene(const AppOptions& options)
{
  DartScene scene;
  scene.world = World::create("filament_gui_g1");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createG1Ground());

  auto g1 = loadG1Skeleton(options);
  std::cout << "Loaded G1 robot from '" << options.g1RobotUri << "'.\n"
            << "Package root for '" << options.g1PackageName << "' set to '"
            << options.g1PackageUri << "'.\n";
  scene.world->addSkeleton(g1);
  addG1IkTargets(scene, g1);

  return scene;
}

DartScene createDartScene(const AppOptions& options)
{
  switch (options.scene) {
    case ExampleScene::Mvp:
      return createMvpDartScene();
    case ExampleScene::DragAndDrop:
      return createDragAndDropScene();
    case ExampleScene::Polyhedron:
      return createPolyhedronScene();
    case ExampleScene::Heightmap:
      return createHeightmapScene();
    case ExampleScene::G1:
      return createG1DartScene(options);
  }
  return createMvpDartScene();
}

} // namespace dart::gui::experimental::filament
