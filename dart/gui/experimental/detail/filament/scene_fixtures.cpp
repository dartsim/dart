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

#include "scene_fixtures.hpp"

#include <dart/all.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>
#include <dart/config.hpp>
#include <dart/io/read.hpp>
#include <dart/sensor/sensor.hpp>
#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/package_resource_retriever.hpp>
#include <dart/utils/urdf/All.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::gui::experimental::filament {

using dart::dynamics::BoxShape;
using dart::dynamics::CapsuleShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::Frame;
using dart::dynamics::FreeJoint;
using dart::dynamics::HeightmapShaped;
using dart::dynamics::Inertia;
using dart::dynamics::InverseKinematics;
using dart::dynamics::LineSegmentShape;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::PointCloudShape;
using dart::dynamics::RevoluteJoint;
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

using dart::gui::experimental::makeRenderableId;
using dart::simulation::World;

struct SensorMarker
{
  std::shared_ptr<SimpleFrame> frame;
  VisualAspect* visual = nullptr;
};

class BlinkingMarkerSensor final : public dart::sensor::Sensor
{
public:
  BlinkingMarkerSensor(
      const Properties& properties,
      const SensorMarker& marker,
      const Eigen::Vector4d& activeColor,
      const Eigen::Vector4d& inactiveColor)
    : dart::sensor::Sensor(properties),
      mMarker(marker.frame),
      mVisual(marker.visual),
      mActiveColor(activeColor),
      mInactiveColor(inactiveColor)
  {
    if (mVisual) {
      mVisual->setRGBA(mInactiveColor);
    }
  }

private:
  void updateImpl(
      const World&, const dart::sensor::SensorUpdateContext&) override
  {
    if (mVisual) {
      mVisual->setRGBA(mPulseOn ? mActiveColor : mInactiveColor);
    }
    mPulseOn = !mPulseOn;
  }

  void resetImpl() override
  {
    mPulseOn = false;
    if (mVisual) {
      mVisual->setRGBA(mInactiveColor);
    }
  }

  std::shared_ptr<SimpleFrame> mMarker;
  VisualAspect* mVisual = nullptr;
  Eigen::Vector4d mActiveColor;
  Eigen::Vector4d mInactiveColor;
  bool mPulseOn = false;
};

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

dart::dynamics::SkeletonPtr createDynamicBoxSkeleton(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color,
    double restitution = 0.0)
{
  auto box = Skeleton::create(name);
  auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>();
  joint->setTransformFromParentBodyNode(transform);

  auto boxShape = std::make_shared<BoxShape>(size);
  auto* boxShapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape);
  boxShapeNode->getVisualAspect()->setColor(color);
  boxShapeNode->getDynamicsAspect()->setRestitutionCoeff(restitution);
  body->setInertia(dart::dynamics::Inertia(
      1.0, Eigen::Vector3d::Zero(), boxShape->computeInertia(1.0)));
  return box;
}

dart::dynamics::SkeletonPtr createDynamicBoxSkeleton(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double restitution = 0.0)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  return createDynamicBoxSkeleton(name, size, transform, color, restitution);
}

dart::dynamics::SkeletonPtr createDynamicSphereSkeleton(
    const std::string& name,
    double radius,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double mass = 1.0)
{
  auto sphere = Skeleton::create(name);
  auto [joint, body] = sphere->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  joint->setTransformFromParentBodyNode(transform);

  auto sphereShape = std::make_shared<SphereShape>(radius);
  auto* sphereNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(sphereShape);
  sphereNode->getVisualAspect()->setColor(color);
  body->setInertia(Inertia(
      mass, Eigen::Vector3d::Zero(), sphereShape->computeInertia(mass)));
  return sphere;
}

dart::dynamics::SkeletonPtr createBoxGroundSkeleton(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color,
    double restitution = 0.0)
{
  auto ground = Skeleton::create(name);
  auto* groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
  auto* groundShapeNode = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(std::make_shared<BoxShape>(size));
  groundShapeNode->getVisualAspect()->setColor(color);
  groundShapeNode->getDynamicsAspect()->setRestitutionCoeff(restitution);
  return ground;
}

SensorMarker createSensorMarker(
    const std::string& name,
    Frame* parent,
    const Eigen::Isometry3d& relativeTransform,
    double radius,
    const Eigen::Vector4d& color)
{
  auto marker = SimpleFrame::createShared(
      parent ? parent : Frame::World(), name, relativeTransform);
  marker->setShape(std::make_shared<SphereShape>(radius));
  VisualAspect* visual = marker->createVisualAspect();
  visual->setRGBA(color);
  return SensorMarker{std::move(marker), visual};
}

DartScene createMvpDartScene()
{
  DartScene scene;

  auto blueBox = createDynamicBoxSkeleton(
      "falling_blue_box",
      Eigen::Vector3d(0.35, 0.35, 0.35),
      Eigen::Vector3d(-0.35, 0.0, 1.5),
      dart::Color::Blue());
  auto orangeBox = createDynamicBoxSkeleton(
      "falling_orange_box",
      Eigen::Vector3d(0.28, 0.28, 0.28),
      Eigen::Vector3d(0.35, 0.1, 2.15),
      dart::Color::Orange());
  auto contactBox = createDynamicBoxSkeleton(
      "contact_green_box",
      Eigen::Vector3d(0.3, 0.3, 0.3),
      Eigen::Vector3d(-1.05, 0.25, 0.18),
      dart::Color::Green());

  auto ground = createBoxGroundSkeleton(
      "ground", Eigen::Vector3d(8.0, 8.0, 0.1), dart::Color::LightGray());

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

DartScene createHelloWorldScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_hello_world");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  const Eigen::Quaterniond orientation
      = Eigen::AngleAxisd(0.45, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY());
  transform.linear() = orientation.toRotationMatrix();

  scene.world->addSkeleton(createDynamicBoxSkeleton(
      kHelloWorldBoxFixtureSkeletonName,
      Eigen::Vector3d(0.3, 0.3, 0.3),
      transform,
      dart::Color::Blue()));
  scene.world->addSkeleton(createBoxGroundSkeleton(
      kHelloWorldGroundFixtureSkeletonName,
      Eigen::Vector3d(10.0, 10.0, 0.1),
      dart::Color::LightGray()));

  return scene;
}

DartScene createBoxesScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_boxes");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  const int dim = 5;
  std::size_t boxIndex = 0;
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      for (int k = 0; k < dim; ++k) {
        scene.world->addSkeleton(createDynamicBoxSkeleton(
            std::string(kBoxesFixtureBoxSkeletonPrefix)
                + std::to_string(boxIndex++),
            Eigen::Vector3d(0.9, 0.9, 0.9),
            Eigen::Vector3d(i - dim / 2, j - dim / 2, k + 5),
            Eigen::Vector3d(
                static_cast<double>(i) / dim,
                static_cast<double>(j) / dim,
                static_cast<double>(k) / dim),
            0.9));
      }
    }
  }

  scene.world->addSkeleton(createBoxGroundSkeleton(
      kBoxesFixtureGroundSkeletonName,
      Eigen::Vector3d(25.0, 25.0, 0.1),
      dart::Color::LightGray(),
      0.9));

  return scene;
}

dart::dynamics::SkeletonPtr createHardcodedDesignSkeleton()
{
  auto skeleton = Skeleton::create(kHardcodedDesignFixtureSkeletonName);
  constexpr double mass = 1.0;

  dart::dynamics::BodyNode::Properties body;
  body.mName = "LHY";
  body.mInertia.setMass(mass);
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.3, 1.0));

  dart::dynamics::RevoluteJoint::Properties joint;
  joint.mName = "LHY";
  joint.mAxis = Eigen::Vector3d::UnitZ();
  joint.mPositionLowerLimits[0] = -dart::math::pi;
  joint.mPositionUpperLimits[0] = dart::math::pi;

  auto [rootJoint, rootBody]
      = skeleton
            ->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                nullptr, joint, body);
  (void)rootJoint;
  auto rootShapeNode = rootBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  rootShapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.18, 0.38, 0.92, 0.9));
  rootBody->setMass(mass);

  body = dart::dynamics::BodyNode::Properties();
  body.mName = "LHR";
  body.mInertia.setMass(mass);
  shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.3, 1.0));
  joint = dart::dynamics::RevoluteJoint::Properties();
  joint.mName = "LHR";
  joint.mAxis = Eigen::Vector3d::UnitX();
  joint.mPositionLowerLimits[0] = -dart::math::pi;
  joint.mPositionUpperLimits[0] = dart::math::pi;
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.5);

  auto [hipRollJoint, hipRollBody]
      = skeleton
            ->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                rootBody, joint, body);
  (void)hipRollJoint;
  auto hipRollShapeNode = hipRollBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  hipRollShapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  hipRollShapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.95, 0.65, 0.12, 0.9));
  hipRollBody->setLocalCOM(hipRollShapeNode->getRelativeTranslation());
  hipRollBody->setMass(mass);

  body = dart::dynamics::BodyNode::Properties();
  body.mName = "LHP";
  body.mInertia.setMass(mass);
  shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.3, 1.0));
  joint = dart::dynamics::RevoluteJoint::Properties();
  joint.mName = "LHP";
  joint.mAxis = Eigen::Vector3d::UnitY();
  joint.mPositionLowerLimits[0] = -dart::math::pi;
  joint.mPositionUpperLimits[0] = dart::math::pi;
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);

  auto [hipPitchJoint, hipPitchBody]
      = skeleton
            ->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                hipRollBody, joint, body);
  (void)hipPitchJoint;
  auto hipPitchShapeNode = hipPitchBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  hipPitchShapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  hipPitchShapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.84, 0.18, 0.18, 0.9));
  hipPitchBody->setLocalCOM(hipPitchShapeNode->getRelativeTranslation());
  hipPitchBody->setMass(mass);

  return skeleton;
}

DartScene createHardcodedDesignScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_hardcoded_design");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createHardcodedDesignSkeleton());
  return scene;
}

DartScene createRigidChainScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/chain.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load rigid_chain fixture from "
        "dart://sample/skel/chain.skel");
  }

  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  scene.world->setTimeStep(1.0 / 2000.0);

  auto chain = scene.world->getSkeleton(0);
  if (!chain) {
    throw std::runtime_error("rigid_chain fixture did not contain a skeleton");
  }
  chain->setName(kRigidChainFixtureSkeletonName);

  const auto dof = static_cast<int>(chain->getNumDofs());
  Eigen::VectorXd initialPose(dof);
  for (int i = 0; i < dof; ++i) {
    initialPose[i] = 0.18 * std::sin(static_cast<double>(i) * 0.7);
  }
  chain->setPositions(initialPose);

  const std::size_t numBodyNodes = chain->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodyNodes; ++i) {
    const double t = numBodyNodes <= 1
                         ? 0.0
                         : static_cast<double>(i)
                               / static_cast<double>(numBodyNodes - 1);
    const Eigen::Vector3d color(
        0.20 + 0.60 * t, 0.58 - 0.28 * t, 0.90 - 0.45 * t);
    chain->getBodyNode(i)->setColor(color);
  }

  return scene;
}

DartScene createRigidLoopScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/chain.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load rigid_loop fixture from "
        "dart://sample/skel/chain.skel");
  }

  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  scene.world->setTimeStep(1.0 / 2000.0);

  auto chain = scene.world->getSkeleton(0);
  if (!chain) {
    throw std::runtime_error("rigid_loop fixture did not contain a skeleton");
  }
  chain->setName(kRigidLoopFixtureSkeletonName);

  Eigen::VectorXd initialPose = Eigen::VectorXd::Zero(chain->getNumDofs());
  for (const int index : std::array{20, 23, 26, 29}) {
    if (index < initialPose.size()) {
      initialPose[index] = 0.4 * dart::math::pi;
    }
  }
  chain->setPositions(initialPose);

  for (std::size_t i = 0; i < chain->getNumBodyNodes(); ++i) {
    chain->getBodyNode(i)->setColor(Eigen::Vector3d(0.35, 0.55, 0.85));
  }

  auto* link6 = chain->getBodyNode("link 6");
  auto* link10 = chain->getBodyNode("link 10");
  if (!link6 || !link10) {
    throw std::runtime_error("rigid_loop fixture is missing loop link bodies");
  }

  link6->setColor(Eigen::Vector3d(0.95, 0.12, 0.12));
  link10->setColor(Eigen::Vector3d(0.95, 0.12, 0.12));

  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d jointPosition = link6->getTransform() * offset;
  scene.world->getConstraintSolver()->addConstraint(
      std::make_shared<dart::constraint::BallJointConstraint>(
          link6, link10, jointPosition));

  return scene;
}

DartScene createMixedChainScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld(
      "dart://sample/skel/test/test_articulated_bodies_10bodies.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load mixed_chain fixture from "
        "dart://sample/skel/test/test_articulated_bodies_10bodies.skel");
  }

  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto chain = scene.world->getSkeleton(1);
  if (!chain) {
    throw std::runtime_error("mixed_chain fixture did not contain a chain");
  }
  chain->setName(kMixedChainFixtureSkeletonName);

  Eigen::VectorXd initialPose = Eigen::VectorXd::Zero(chain->getNumDofs());
  if (initialPose.size() >= 3) {
    initialPose[0] = -0.25;
    initialPose[1] = 0.18;
    initialPose[2] = 0.12;
  }
  chain->setPositions(initialPose);

  for (std::size_t i = 0; i < chain->getNumBodyNodes(); ++i) {
    auto* bodyNode = chain->getBodyNode(i);
    const bool softLink
        = dynamic_cast<const dart::dynamics::SoftBodyNode*>(bodyNode) != nullptr;
    bodyNode->setColor(
        softLink ? Eigen::Vector3d(0.90, 0.42, 0.18)
                 : Eigen::Vector3d(0.30, 0.55, 0.85));
  }

  return scene;
}

struct CouplerConstraintFixtureAssembly
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::Joint* referenceJoint = nullptr;
  dart::dynamics::Joint* followerJoint = nullptr;
  dart::dynamics::BodyNode* referenceBody = nullptr;
  dart::dynamics::BodyNode* followerBody = nullptr;
};

CouplerConstraintFixtureAssembly createCouplerConstraintFixtureAssembly(
    const std::string& label,
    const Eigen::Vector3d& baseOffset,
    bool useCouplerConstraint,
    const Eigen::Vector3d& referenceColor,
    const Eigen::Vector3d& followerColor,
    double targetAngle)
{
  auto skeleton
      = Skeleton::create(std::string(kCouplerConstraintFixtureSkeletonPrefix)
                         + label);

  dart::dynamics::WeldJoint::Properties baseJointProps;
  baseJointProps.mName = label + "_base_joint";
  baseJointProps.mT_ParentBodyToJoint = Eigen::Translation3d(baseOffset);

  dart::dynamics::BodyNode::Properties baseBodyProps;
  baseBodyProps.mName = label + "_base";
  Inertia baseInertia;
  baseInertia.setMass(0.1);
  baseInertia.setMoment(1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0);
  baseBodyProps.mInertia = baseInertia;

  auto basePair = skeleton->createJointAndBodyNodePair<WeldJoint>(
      nullptr, baseJointProps, baseBodyProps);
  auto* baseBody = basePair.second;

  RevoluteJoint::Properties referenceJointProps;
  referenceJointProps.mName = label + "_reference_joint";
  referenceJointProps.mAxis = Eigen::Vector3d::UnitZ();
  referenceJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, 0.15, 0.0);
  referenceJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  dart::dynamics::BodyNode::Properties referenceBodyProps;
  referenceBodyProps.mName = label + "_reference_body";
  Inertia referenceInertia;
  referenceInertia.setMass(1.0);
  referenceInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  referenceBodyProps.mInertia = referenceInertia;

  auto referencePair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      baseBody, referenceJointProps, referenceBodyProps);
  auto* referenceJoint = referencePair.first;
  auto* referenceBody = referencePair.second;

  referenceJoint->setActuatorType(dart::dynamics::Joint::FORCE);
  referenceJoint->setForceLowerLimit(0, -120.0);
  referenceJoint->setForceUpperLimit(0, 120.0);
  referenceJoint->setDampingCoefficient(0, 0.02);

  auto referenceShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.05, 0.05));
  auto referenceShapeNode = referenceBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(referenceShape);
  referenceShapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));
  referenceShapeNode->getVisualAspect()->setColor(referenceColor);

  RevoluteJoint::Properties followerJointProps;
  followerJointProps.mName = label + "_follower_joint";
  followerJointProps.mAxis = Eigen::Vector3d::UnitZ();
  followerJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, -0.15, 0.0);
  followerJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  dart::dynamics::BodyNode::Properties followerBodyProps;
  followerBodyProps.mName = label + "_follower_body";
  Inertia followerInertia;
  followerInertia.setMass(1.0);
  followerInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  followerBodyProps.mInertia = followerInertia;

  auto followerPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      baseBody, followerJointProps, followerBodyProps);
  auto* followerJoint = followerPair.first;
  auto* followerBody = followerPair.second;

  followerJoint->setActuatorType(dart::dynamics::Joint::MIMIC);
  followerJoint->setMimicJoint(referenceJoint, -1.0, 0.0);
  followerJoint->setUseCouplerConstraint(useCouplerConstraint);
  followerJoint->setForceLowerLimit(0, -120.0);
  followerJoint->setForceUpperLimit(0, 120.0);
  followerJoint->setDampingCoefficient(0, 0.02);
  followerJoint->setPositionLowerLimit(0, -0.35);
  followerJoint->setPositionUpperLimit(0, 0.35);
  followerJoint->setLimitEnforcement(true);

  auto followerShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.05, 0.05));
  auto followerShapeNode = followerBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(followerShape);
  followerShapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));
  followerShapeNode->getVisualAspect()->setColor(followerColor);

  referenceJoint->setPosition(0, targetAngle);
  followerJoint->setPosition(0, followerJoint->getPositionLowerLimit(0));

  return {
      skeleton,
      referenceJoint,
      followerJoint,
      referenceBody,
      followerBody,
  };
}

void addCouplerConstraintLimitGuide(
    World& world,
    const std::string& name,
    dart::dynamics::Joint* followerJoint,
    double angle,
    const Eigen::Vector3d& color)
{
  auto* revolute = dynamic_cast<RevoluteJoint*>(followerJoint);
  if (revolute == nullptr) {
    throw std::runtime_error("coupler fixture expected a revolute follower");
  }

  auto frame = SimpleFrame::createShared(Frame::World(), name);
  auto line = std::make_shared<LineSegmentShape>(0.05f);
  line->addVertex(Eigen::Vector3d::Zero());
  line->addVertex(Eigen::Vector3d::UnitX() * 0.65);
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->createVisualAspect()->setColor(color);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (const auto* parent = followerJoint->getParentBodyNode()) {
    transform = parent->getWorldTransform();
  }
  transform = transform * followerJoint->getTransformFromParentBodyNode();
  transform.rotate(Eigen::AngleAxisd(angle, revolute->getAxis()));
  transform.translate(Eigen::Vector3d(0.0, 0.0, 0.02));
  frame->setRelativeTransform(transform);

  world.addSimpleFrame(frame);
}

void addCouplerConstraintPairLink(
    World& world,
    const std::string& name,
    float width,
    const Eigen::Vector3d& color,
    const CouplerConstraintFixtureAssembly& assembly)
{
  auto frame = SimpleFrame::createShared(Frame::World(), name);
  auto line = std::make_shared<LineSegmentShape>(width);
  line->addVertex(assembly.referenceBody->getWorldTransform().translation());
  line->addVertex(assembly.followerBody->getWorldTransform().translation());
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->createVisualAspect()->setColor(color);
  world.addSimpleFrame(frame);
}

DartScene createCouplerConstraintScene()
{
  DartScene scene;
  scene.world = World::create();
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->setTimeStep(1e-3);

  const double targetAngle = 45.0 * dart::math::pi / 180.0;
  auto coupler = createCouplerConstraintFixtureAssembly(
      "coupler",
      Eigen::Vector3d(-0.45, 0.0, 0.0),
      true,
      Eigen::Vector3d(0.85, 0.35, 0.25),
      Eigen::Vector3d(0.25, 0.58, 0.92),
      targetAngle);
  auto motor = createCouplerConstraintFixtureAssembly(
      "motor",
      Eigen::Vector3d(0.45, 0.0, 0.0),
      false,
      Eigen::Vector3d(0.68, 0.32, 0.70),
      Eigen::Vector3d(0.25, 0.75, 0.70),
      targetAngle);

  scene.world->addSkeleton(coupler.skeleton);
  scene.world->addSkeleton(motor.skeleton);

  addCouplerConstraintLimitGuide(
      *scene.world,
      std::string(kCouplerConstraintFixtureFramePrefix) + "coupler_lower_limit",
      coupler.followerJoint,
      coupler.followerJoint->getPositionLowerLimit(0),
      Eigen::Vector3d(0.92, 0.35, 0.35));
  addCouplerConstraintLimitGuide(
      *scene.world,
      std::string(kCouplerConstraintFixtureFramePrefix) + "coupler_upper_limit",
      coupler.followerJoint,
      coupler.followerJoint->getPositionUpperLimit(0),
      Eigen::Vector3d(0.35, 0.92, 0.35));
  addCouplerConstraintLimitGuide(
      *scene.world,
      std::string(kCouplerConstraintFixtureFramePrefix) + "motor_lower_limit",
      motor.followerJoint,
      motor.followerJoint->getPositionLowerLimit(0),
      Eigen::Vector3d(0.92, 0.35, 0.35));
  addCouplerConstraintLimitGuide(
      *scene.world,
      std::string(kCouplerConstraintFixtureFramePrefix) + "motor_upper_limit",
      motor.followerJoint,
      motor.followerJoint->getPositionUpperLimit(0),
      Eigen::Vector3d(0.35, 0.92, 0.35));
  addCouplerConstraintPairLink(
      *scene.world,
      std::string(kCouplerConstraintFixtureFramePrefix) + "coupler_link",
      0.06f,
      Eigen::Vector3d(0.95, 0.95, 0.2),
      coupler);
  addCouplerConstraintPairLink(
      *scene.world,
      std::string(kCouplerConstraintFixtureFramePrefix) + "motor_link",
      0.04f,
      Eigen::Vector3d(0.75, 0.75, 0.9),
      motor);

  return scene;
}

DartScene createAddDeleteSkelsScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/ground.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load add_delete_skels fixture from "
        "dart://sample/skel/ground.skel");
  }
  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = scene.world->getSkeleton(0);
  if (!ground) {
    throw std::runtime_error("add_delete_skels fixture did not contain ground");
  }
  ground->setName(kAddDeleteSkelsFixtureGroundSkeletonName);

  const std::array<Eigen::Vector3d, kAddDeleteSkelsFixtureCubeCount> positions{
          Eigen::Vector3d(-0.8, 0.65, -0.7),
          Eigen::Vector3d(-0.35, 1.05, 0.25),
          Eigen::Vector3d(0.15, 0.75, -0.15),
          Eigen::Vector3d(0.55, 1.25, 0.55),
          Eigen::Vector3d(0.9, 0.95, -0.45),
      };
  const std::array<Eigen::Vector3d, kAddDeleteSkelsFixtureCubeCount> sizes{
      Eigen::Vector3d(0.25, 0.22, 0.30),
      Eigen::Vector3d(0.18, 0.28, 0.22),
      Eigen::Vector3d(0.32, 0.18, 0.24),
      Eigen::Vector3d(0.22, 0.34, 0.20),
      Eigen::Vector3d(0.28, 0.24, 0.18),
  };
  const std::array<Eigen::Vector3d, kAddDeleteSkelsFixtureCubeCount> colors{
      Eigen::Vector3d(0.85, 0.30, 0.24),
      Eigen::Vector3d(0.25, 0.55, 0.90),
      Eigen::Vector3d(0.95, 0.72, 0.25),
      Eigen::Vector3d(0.35, 0.78, 0.45),
      Eigen::Vector3d(0.68, 0.38, 0.85),
  };

  for (std::size_t i = 0; i < kAddDeleteSkelsFixtureCubeCount; ++i) {
    scene.world->addSkeleton(createDynamicBoxSkeleton(
        std::string(kAddDeleteSkelsFixtureCubeSkeletonPrefix)
            + std::to_string(i),
        sizes[i],
        positions[i],
        colors[i],
        0.0));
  }

  return scene;
}

DartScene createVehicleScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/vehicle.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load vehicle fixture from "
        "dart://sample/skel/vehicle.skel");
  }
  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  const auto requireSkeleton = [&](const std::string& name) {
    auto skeleton = scene.world->getSkeleton(name);
    if (!skeleton) {
      throw std::runtime_error(
          "vehicle fixture is missing skeleton: " + name);
    }
    return skeleton;
  };

  auto ground = requireSkeleton("ground skeleton");
  ground->setName(kVehicleFixtureGroundSkeletonName);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.45, 0.50, 0.45));
  }

  auto car = requireSkeleton("car_skeleton");
  car->setName(kVehicleFixtureCarSkeletonName);
  if (auto* body = car->getBodyNode("main_body")) {
    body->setColor(Eigen::Vector3d(0.18, 0.36, 0.82));
  }
  const std::array<const char*, kVehicleFixtureWheelCylinderCount> wheelNames{
      "wheel_front_left",
      "wheel_front_right",
      "wheel_back_left",
      "wheel_back_right"};
  for (const char* wheelName : wheelNames) {
    if (auto* wheel = car->getBodyNode(wheelName)) {
      wheel->setColor(Eigen::Vector3d(0.06, 0.06, 0.07));
    }
  }

  const std::array<const char*, kVehicleFixtureObstacleCount> obstacles{
      "skeleton_box1", "skeleton_box2"};
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    auto obstacle = requireSkeleton(obstacles[i]);
    obstacle->setName(
        std::string(kVehicleFixtureObstacleSkeletonPrefix) + std::to_string(i));
    if (auto* body = obstacle->getBodyNode("box")) {
      body->setColor(
          i == 0 ? Eigen::Vector3d(0.86, 0.42, 0.20)
                 : Eigen::Vector3d(0.78, 0.64, 0.18));
    }
  }

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

DartScene createSimpleFramesScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_simple_frames");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translate(Eigen::Vector3d(0.1, -0.1, 0.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translate(Eigen::Vector3d(0.0, 0.1, 0.0));
  tf2.rotate(Eigen::AngleAxisd(
      dart::math::toRadian(45.0), Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translate(Eigen::Vector3d(0.0, 0.0, 0.1));
  tf3.rotate(Eigen::AngleAxisd(
      dart::math::toRadian(60.0), Eigen::Vector3d::UnitY()));

  const auto addFrame = [&](const std::shared_ptr<SimpleFrame>& frame) {
    scene.world->addSimpleFrame(frame);
    return frame;
  };
  const auto setColor = [](const std::shared_ptr<SimpleFrame>& frame,
                           const Eigen::Vector4d& color) {
    frame->getVisualAspect(true)->setRGBA(color);
  };

  auto f1 = SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      std::string(kSimpleFramesFixtureBoxFramePrefix) + "1",
      tf1);
  f1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f1, Eigen::Vector4d(0.92, 0.32, 0.24, 1.0));
  addFrame(f1);

  auto f2 = f1->spawnChildSimpleFrame(
      std::string(kSimpleFramesFixtureBoxFramePrefix) + "2", tf2);
  f2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f2, Eigen::Vector4d(0.24, 0.58, 0.92, 1.0));
  addFrame(f2);

  auto f3 = f2->spawnChildSimpleFrame(
      std::string(kSimpleFramesFixtureBoxFramePrefix) + "3", tf3);
  f3->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.05, 0.05, 0.02)));
  setColor(f3, Eigen::Vector4d(0.28, 0.76, 0.34, 1.0));
  addFrame(f3);

  auto markerRoot = SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      std::string(kSimpleFramesFixtureEllipsoidFramePrefix) + "root");
  markerRoot->setShape(std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(0.02, 0.02, 0.02)));
  setColor(markerRoot, Eigen::Vector4d(0.95, 0.75, 0.20, 1.0));
  addFrame(markerRoot);

  const auto addMarker = [&](const std::string& name,
                             const Eigen::Isometry3d& transform) {
    auto marker = markerRoot->spawnChildSimpleFrame(name, transform);
    marker->setShape(std::make_shared<dart::dynamics::EllipsoidShape>(
        Eigen::Vector3d(0.01, 0.01, 0.01)));
    setColor(marker, Eigen::Vector4d(0.95, 0.75, 0.20, 1.0));
    addFrame(marker);
  };
  addMarker(
      std::string(kSimpleFramesFixtureEllipsoidFramePrefix) + "1",
      f1->getTransform(markerRoot.get()));
  addMarker(
      std::string(kSimpleFramesFixtureEllipsoidFramePrefix) + "2",
      f2->getTransform(markerRoot.get()));
  addMarker(
      std::string(kSimpleFramesFixtureEllipsoidFramePrefix) + "3",
      f3->getTransform(markerRoot.get()));

  auto arrow = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kSimpleFramesFixtureArrowFrameName);
  auto arrowShape = std::make_shared<dart::dynamics::LineSegmentShape>(
      Eigen::Vector3d(0.1, -0.1, 0.0),
      Eigen::Vector3d(0.1, 0.0, 0.0),
      2.0f);
  arrowShape->addVertex(Eigen::Vector3d(0.075, -0.025, 0.0), 1);
  arrowShape->addVertex(Eigen::Vector3d(0.125, -0.025, 0.0), 1);
  arrow->setShape(arrowShape);
  setColor(arrow, Eigen::Vector4d(1.0, 0.5, 0.5, 1.0));
  addFrame(arrow);

  return scene;
}

DartScene createSoftBodiesScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/softBodies.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load soft_bodies fixture from "
        "dart://sample/skel/softBodies.skel");
  }

  return scene;
}

DartScene createPointCloudScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_point_cloud");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  auto pointCloudShape = std::make_shared<PointCloudShape>(0.055);
  pointCloudShape->setPointShapeType(PointCloudShape::BOX);
  pointCloudShape->setColorMode(PointCloudShape::BIND_PER_POINT);

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector4d> colors;
  points.reserve(48);
  colors.reserve(48);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 6; ++y) {
      const double xf = -0.42 + 0.12 * static_cast<double>(x);
      const double yf = -0.30 + 0.12 * static_cast<double>(y);
      const double zf
          = 0.05 * std::sin(5.0 * xf) + 0.06 * std::cos(4.0 * yf);
      points.emplace_back(xf, yf, zf);

      const double mix = static_cast<double>(x + y) / 12.0;
      colors.emplace_back(0.18 + 0.70 * mix, 0.28, 0.92 - 0.55 * mix, 0.82);
    }
  }
  pointCloudShape->addPoint(points);
  pointCloudShape->setColors(colors);

  scene.world->addSkeleton(createStaticVisualSkeleton(
      kPointCloudFixtureSkeletonName,
      pointCloudShape,
      Eigen::Vector3d(-0.38, -0.05, 0.55),
      Eigen::Vector3d(0.25, 0.48, 0.95),
      0.82));

#if DART_HAVE_OCTOMAP
  auto voxelGridShape = std::make_shared<VoxelGridShape>(0.11);
  const std::array<Eigen::Vector3d, 9> voxelCenters{{
      Eigen::Vector3d(-0.22, -0.10, 0.00),
      Eigen::Vector3d(-0.11, -0.10, 0.00),
      Eigen::Vector3d(0.00, -0.10, 0.00),
      Eigen::Vector3d(0.11, -0.10, 0.00),
      Eigen::Vector3d(-0.11, 0.01, 0.11),
      Eigen::Vector3d(0.00, 0.01, 0.11),
      Eigen::Vector3d(0.11, 0.01, 0.11),
      Eigen::Vector3d(0.00, 0.12, 0.22),
      Eigen::Vector3d(0.11, 0.12, 0.22),
  }};
  for (const Eigen::Vector3d& voxel : voxelCenters) {
    voxelGridShape->updateOccupancy(voxel);
  }
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kVoxelGridFixtureSkeletonName,
      voxelGridShape,
      Eigen::Vector3d(0.45, 0.06, 0.48),
      Eigen::Vector3d(0.94, 0.52, 0.20),
      0.72));
#endif

  Eigen::Isometry3d sensorTransform = Eigen::Isometry3d::Identity();
  sensorTransform.translation() = Eigen::Vector3d(0.72, -0.42, 0.76);
  auto sensor = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "point_cloud_sensor", sensorTransform);
  sensor->setShape(std::make_shared<SphereShape>(0.055));
  sensor->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.95, 0.18, 0.12, 1.0));
  scene.world->addSimpleFrame(sensor);

  return scene;
}

DartScene createCapsuleGroundContactScene()
{
  constexpr double kCapsuleRadius = 0.2;
  constexpr double kCapsuleHeight = 0.6;
  constexpr double kGroundVisualThickness = 0.08;

  DartScene scene;
  scene.world = World::create("filament_gui_capsule_ground_contact");
  scene.world->setTimeStep(0.001);
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
#if DART_HAVE_ODE
  scene.world->setCollisionDetector(
      dart::simulation::CollisionDetectorType::Ode);
#endif

  auto ground = Skeleton::create(kCapsuleGroundContactGroundSkeletonName);
  auto* groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
  groundBody->setName("ground");
  const Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ();
  groundBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<PlaneShape>(planeNormal, 0.0));

  auto* groundVisual = groundBody->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(
          Eigen::Vector3d(4.0, 4.0, kGroundVisualThickness)));
  Eigen::Isometry3d groundOffset = Eigen::Isometry3d::Identity();
  groundOffset.translation()
      = planeNormal * (-0.5 * kGroundVisualThickness);
  groundVisual->setRelativeTransform(groundOffset);
  groundVisual->getVisualAspect()->setColor(Eigen::Vector3d(0.70, 0.70, 0.70));
  groundVisual->getVisualAspect()->setShadowed(false);
  ground->setMobile(false);
  scene.world->addSkeleton(ground);

  auto capsule = Skeleton::create(kCapsuleGroundContactCapsuleSkeletonName);
  auto [joint, body] = capsule->createJointAndBodyNodePair<FreeJoint>();
  Eigen::Isometry3d capsuleTransform = Eigen::Isometry3d::Identity();
  capsuleTransform.translation()
      = Eigen::Vector3d(0.0, 0.0, kCapsuleRadius + 0.12);
  capsuleTransform.rotate(
      Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitY()));
  joint->setTransformFromParentBodyNode(capsuleTransform);

  auto capsuleShape
      = std::make_shared<CapsuleShape>(kCapsuleRadius, kCapsuleHeight);
  auto* capsuleNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(capsuleShape);
  capsuleNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.4, 0.8));

  constexpr double kCapsuleMass = 1.0;
  body->setInertia(Inertia(
      kCapsuleMass,
      Eigen::Vector3d::Zero(),
      capsuleShape->computeInertia(kCapsuleMass)));
  scene.world->addSkeleton(capsule);

  return scene;
}

DartScene createSimulationEventHandlerScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_simulation_event_handler");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->setTimeStep(0.001);

  scene.world->addSkeleton(createBoxGroundSkeleton(
      kSimulationEventHandlerGroundSkeletonName,
      Eigen::Vector3d(8.0, 8.0, 0.1),
      Eigen::Vector3d(0.3, 0.3, 0.3)));

  auto sensorCarrier = createDynamicBoxSkeleton(
      std::string(kSimulationEventHandlerBoxSkeletonPrefix) + "sensor_carrier",
      Eigen::Vector3d(0.4, 0.4, 0.4),
      Eigen::Vector3d(-1.0, 0.0, 2.0),
      Eigen::Vector3d(0.8, 0.2, 0.2));
  auto* sensorParent = sensorCarrier->getBodyNode(0);
  scene.world->addSkeleton(sensorCarrier);

  scene.world->addSkeleton(createDynamicBoxSkeleton(
      std::string(kSimulationEventHandlerBoxSkeletonPrefix) + "green",
      Eigen::Vector3d(0.6, 0.3, 0.3),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      Eigen::Vector3d(0.2, 0.8, 0.2)));
  scene.world->addSkeleton(createDynamicBoxSkeleton(
      std::string(kSimulationEventHandlerBoxSkeletonPrefix) + "yellow",
      Eigen::Vector3d(0.3, 0.5, 0.4),
      Eigen::Vector3d(0.5, 1.0, 2.2),
      Eigen::Vector3d(0.9, 0.9, 0.2)));
  scene.world->addSkeleton(createDynamicSphereSkeleton(
      kSimulationEventHandlerSphereSkeletonName,
      0.3,
      Eigen::Vector3d(1.0, 0.0, 2.5),
      Eigen::Vector3d(0.2, 0.2, 0.8)));

  Eigen::Isometry3d fastOffset = Eigen::Isometry3d::Identity();
  fastOffset.translation() = Eigen::Vector3d(0.2, 0.0, 0.35);
  Eigen::Isometry3d slowOffset = Eigen::Isometry3d::Identity();
  slowOffset.translation() = Eigen::Vector3d(-0.2, 0.0, 0.35);

  const Eigen::Vector4d fastInactive(0.1, 0.8, 0.2, 0.25);
  const Eigen::Vector4d fastActive(0.1, 0.95, 0.25, 0.9);
  const Eigen::Vector4d slowInactive(0.95, 0.45, 0.1, 0.25);
  const Eigen::Vector4d slowActive(1.0, 0.65, 0.15, 0.9);
  auto fastMarker = createSensorMarker(
      kSimulationEventHandlerFastSensorFrameName,
      sensorParent,
      fastOffset,
      0.08,
      fastInactive);
  auto slowMarker = createSensorMarker(
      kSimulationEventHandlerSlowSensorFrameName,
      sensorParent,
      slowOffset,
      0.1,
      slowInactive);

  dart::sensor::Sensor::Properties fastProps;
  fastProps.name = "fast_sensor";
  fastProps.updateRate = 30.0;
  fastProps.relativeTransform = fastOffset;
  auto fastSensor = std::make_shared<BlinkingMarkerSensor>(
      fastProps, fastMarker, fastActive, fastInactive);
  fastSensor->setParentFrame(sensorParent);

  dart::sensor::Sensor::Properties slowProps;
  slowProps.name = "slow_sensor";
  slowProps.updateRate = 5.0;
  slowProps.relativeTransform = slowOffset;
  auto slowSensor = std::make_shared<BlinkingMarkerSensor>(
      slowProps, slowMarker, slowActive, slowInactive);
  slowSensor->setParentFrame(sensorParent);

  scene.world->addSensor(fastSensor);
  scene.world->addSensor(slowSensor);
  scene.world->addSimpleFrame(fastMarker.frame);
  scene.world->addSimpleFrame(slowMarker.frame);

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

} // namespace dart::gui::experimental::filament
