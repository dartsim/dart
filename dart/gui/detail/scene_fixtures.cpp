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

#include <dart/config.hpp>

#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/package_resource_retriever.hpp>
#include <dart/utils/urdf/All.hpp>

#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>
#include <dart/sensor/sensor.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::gui::detail {

using dart::dynamics::BallJoint;
using dart::dynamics::BoxShape;
using dart::dynamics::CapsuleShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::CylinderShape;
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
using dart::dynamics::ShapeNode;
using dart::dynamics::ShapePtr;
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

using dart::gui::makeRenderableId;
using dart::simulation::World;

struct SensorMarker
{
  std::shared_ptr<SimpleFrame> frame;
  VisualAspect* visual = nullptr;
};

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton);

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
      = std::filesystem::path(DART_GUI_FILAMENT_REPOSITORY_ROOT) / input;
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
  return {
      Eigen::Vector3d(-0.5, -0.5, 0.0),
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
      0.02, 0.10, 0.06, 0.12, 0.08, 0.18, 0.14, 0.05, 0.10, 0.24, 0.16, 0.08};
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

  auto sharedMesh
      = std::shared_ptr<dart::math::TriMesh<double>>(std::move(mesh));
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

void setRequiredJointPositions(
    const dart::dynamics::SkeletonPtr& skeleton,
    const char* jointName,
    const std::vector<double>& positions)
{
  if (!skeleton) {
    throw std::runtime_error(
        "Cannot pose joint " + std::string(jointName)
        + " because the skeleton is null");
  }

  auto* joint = skeleton->getJoint(jointName);
  if (joint == nullptr) {
    throw std::runtime_error(
        "human_joint_limits fixture is missing joint: "
        + std::string(jointName));
  }

  if (joint->getNumDofs() != positions.size()) {
    throw std::runtime_error(
        "human_joint_limits fixture joint " + std::string(jointName)
        + " has an unexpected DOF count");
  }

  Eigen::VectorXd q(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    q[static_cast<Eigen::Index>(i)] = positions[i];
  }
  joint->setPositions(q);
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
      {{"/j1", 0.0},
       {"/j2", -0.55},
       {"/j3", 0.25},
       {"/j4", 1.05},
       {"/j5", 0.0},
       {"/j6", 0.7},
       {"/j7", 0.0}}};
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

dart::dynamics::SkeletonPtr loadOperationalSpaceControlWamSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      "herb_description", dart::config::dataPath("urdf/wam"));
  const auto wamUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("urdf/wam/wam.urdf"));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (!wam) {
    throw std::runtime_error(
        "Failed to load operational-space WAM fixture from "
        + wamUri.toString());
  }

  wam->setName(kOperationalSpaceControlWamSkeletonName);
  const std::array<std::pair<const char*, double>, 7> jointPositions{{
      {"/j1", 0.0},
      {"/j2", 0.0},
      {"/j3", 0.0},
      {"/j4", 0.0},
      {"/j5", 0.0},
      {"/j6", 0.0},
      {"/j7", 0.0},
  }};
  for (const auto& [name, position] : jointPositions) {
    auto* dof = wam->getDof(name);
    if (dof == nullptr) {
      throw std::runtime_error(
          "Operational-space WAM fixture is missing expected DOF "
          + std::string(name));
    }
    dof->setPosition(position);
  }

  wam->eachJoint([](dart::dynamics::Joint* joint) {
    joint->setLimitEnforcement(false);
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      joint->setDampingCoefficient(i, 0.5);
    }
  });
  return wam;
}

dart::dynamics::SkeletonPtr loadWamIkFastSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      "herb_description", dart::config::dataPath("urdf/wam"));
  const auto wamUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("urdf/wam/wam.urdf"));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (!wam) {
    throw std::runtime_error(
        "Failed to load WAM IKFast fixture from " + wamUri.toString());
  }

  wam->setName(kWamIkFastFixtureSkeletonName);
  const std::array<std::pair<const char*, double>, 7> jointPositions{{
      {"/j1", 0.0},
      {"/j2", 0.0},
      {"/j3", 0.0},
      {"/j4", 0.0},
      {"/j5", 0.0},
      {"/j6", 0.0},
      {"/j7", 0.0},
  }};
  for (const auto& [name, position] : jointPositions) {
    auto* dof = wam->getDof(name);
    if (dof == nullptr) {
      throw std::runtime_error(
          "WAM IKFast fixture is missing expected DOF " + std::string(name));
    }
    dof->setPosition(position);
  }

  makeVisualOnlySkeleton(wam);
  return wam;
}

class OperationalSpaceControlState
{
public:
  OperationalSpaceControlState(
      dart::dynamics::SkeletonPtr robot, dart::dynamics::SimpleFramePtr target)
    : mRobot(std::move(robot)),
      mEndEffector(mRobot ? mRobot->getBodyNode("/wam7") : nullptr),
      mTarget(std::move(target)),
      mOffset(0.05, 0.0, 0.0)
  {
    if (mRobot == nullptr || mEndEffector == nullptr || mTarget == nullptr) {
      throw std::runtime_error(
          "Operational-space fixture is missing WAM, end-effector, or target");
    }

    const std::size_t dofs = mEndEffector->getNumDependentGenCoords();
    mKp.setZero();
    for (std::size_t i = 0; i < 3; ++i) {
      mKp(i, i) = 50.0;
    }
    mKd.setZero(dofs, dofs);
    for (std::size_t i = 0; i < dofs; ++i) {
      mKd(i, i) = 5.0;
    }

    Eigen::Isometry3d targetTransform = mEndEffector->getWorldTransform();
    targetTransform.pretranslate(mOffset);
    mTarget->setTransform(targetTransform);
    mOffset
        = mEndEffector->getWorldTransform().rotation().transpose() * mOffset;
  }

  void preStep()
  {
    const Eigen::MatrixXd mass = mRobot->getMassMatrix();

    const dart::math::LinearJacobian jacobian
        = mEndEffector->getLinearJacobian(mOffset);
    const Eigen::MatrixXd jacobianInverse
        = jacobian.transpose()
          * (jacobian * jacobian.transpose()
             + 0.0025 * Eigen::Matrix3d::Identity())
                .inverse();

    const dart::math::LinearJacobian jacobianDeriv
        = mEndEffector->getLinearJacobianDeriv(mOffset);
    const Eigen::MatrixXd jacobianDerivInverse
        = jacobianDeriv.transpose()
          * (jacobianDeriv * jacobianDeriv.transpose()
             + 0.0025 * Eigen::Matrix3d::Identity())
                .inverse();

    const Eigen::Vector3d positionError
        = mTarget->getWorldTransform().translation()
          - mEndEffector->getWorldTransform() * mOffset;
    const Eigen::Vector3d velocityError
        = -mEndEffector->getLinearVelocity(mOffset);
    const Eigen::VectorXd coriolisAndGravity
        = mRobot->getCoriolisAndGravityForces();

    const Eigen::VectorXd forces
        = mass
              * (jacobianInverse * mKp * velocityError
                 + jacobianDerivInverse * mKp * positionError)
          + coriolisAndGravity + mKd * jacobianInverse * mKp * positionError;
    mRobot->setForces(forces);
  }

private:
  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::BodyNode* mEndEffector = nullptr;
  dart::dynamics::SimpleFramePtr mTarget;
  Eigen::Vector3d mOffset;
  Eigen::Matrix3d mKp;
  Eigen::MatrixXd mKd;
};

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

dart::dynamics::SkeletonPtr loadAtlasSimbiconSkeleton()
{
  const auto atlasUri = dart::common::Uri::createFromString(
      "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (!atlas) {
    throw std::runtime_error(
        "Failed to load Atlas Simbicon fixture from " + atlasUri.toString());
  }

  atlas->setName(kAtlasRobotFixtureSkeletonName);
  if (atlas->getNumDofs() == 0) {
    throw std::runtime_error("Atlas Simbicon fixture has no root DOF");
  }

  constexpr double halfPi = 1.5707963267948966;
  atlas->setPosition(0, -halfPi);
  if (auto* rootBody = atlas->getRootBodyNode();
      rootBody != nullptr
      && dynamic_cast<FreeJoint*>(rootBody->getParentJoint()) != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation() = Eigen::Vector3d(0.0, 0.92, 0.0);
    FreeJoint::setTransformOf(rootBody, transform);
  }

  makeVisualOnlySkeleton(atlas);
  return atlas;
}

void setRequiredDofPosition(
    const dart::dynamics::SkeletonPtr& skeleton,
    const char* name,
    double position)
{
  auto* dof = skeleton ? skeleton->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Atlas puppet fixture is missing expected DOF " + std::string(name));
  }
  dof->setPosition(position);
}

void setupAtlasPuppetStartConfiguration(
    const dart::dynamics::SkeletonPtr& atlas)
{
  constexpr double degrees = 3.14159265358979323846 / 180.0;

  setRequiredDofPosition(atlas, "r_leg_hpy", -45.0 * degrees);
  setRequiredDofPosition(atlas, "r_leg_kny", 90.0 * degrees);
  setRequiredDofPosition(atlas, "r_leg_aky", -45.0 * degrees);
  setRequiredDofPosition(atlas, "l_leg_hpy", -45.0 * degrees);
  setRequiredDofPosition(atlas, "l_leg_kny", 90.0 * degrees);
  setRequiredDofPosition(atlas, "l_leg_aky", -45.0 * degrees);

  setRequiredDofPosition(atlas, "r_arm_shx", 65.0 * degrees);
  setRequiredDofPosition(atlas, "r_arm_ely", 90.0 * degrees);
  setRequiredDofPosition(atlas, "r_arm_elx", -90.0 * degrees);
  setRequiredDofPosition(atlas, "r_arm_wry", 65.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_shx", -65.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_ely", 90.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_elx", 90.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_wry", 65.0 * degrees);

  atlas->getDof("r_leg_kny")->setPositionLowerLimit(10.0 * degrees);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit(10.0 * degrees);
}

dart::dynamics::SkeletonPtr loadAtlasPuppetSkeleton()
{
  const auto atlasUri = dart::common::Uri::createFromString(
      "dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (!atlas) {
    throw std::runtime_error(
        "Failed to load Atlas puppet fixture from " + atlasUri.toString());
  }

  atlas->setName(kAtlasRobotFixtureSkeletonName);
  setupAtlasPuppetStartConfiguration(atlas);

  auto* rootBody = atlas->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<FreeJoint*>(rootBody->getParentJoint()) != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation().x() = 0.0;
    transform.translation().y() = 0.0;
    FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(atlas)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      FreeJoint::setTransformOf(rootBody, transform);
    }
  }

  auto* torso = atlas->getRootBodyNode();
  if (torso != nullptr) {
    auto rootShape
        = std::make_shared<BoxShape>(Eigen::Vector3d(0.25, 0.25, 0.125));
    auto* shapeNode = torso->createShapeNodeWith<VisualAspect>(rootShape);
    shapeNode->setName("atlas_puppet_root_handle");
    shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.1));
    shapeNode->getVisualAspect()->setRGBA(
        Eigen::Vector4d(0.08, 0.09, 0.10, 1.0));
  }

  disableSkeletonCollisionAndGravity(atlas);
  return atlas;
}

dart::math::SupportGeometry makeAtlasPuppetFootSupportGeometry()
{
  dart::math::SupportGeometry support;
  constexpr double supportPositiveX = 0.10 - 0.186;
  constexpr double supportNegativeX = -0.03 - 0.186;
  constexpr double supportPositiveY = 0.03;
  constexpr double supportNegativeY = -0.03;
  support.emplace_back(supportNegativeX, supportNegativeY, 0.0);
  support.emplace_back(supportPositiveX, supportNegativeY, 0.0);
  support.emplace_back(supportPositiveX, supportPositiveY, 0.0);
  support.emplace_back(supportNegativeX, supportPositiveY, 0.0);
  return support;
}

void addAtlasPuppetIkTargets(
    DartScene& scene, const dart::dynamics::SkeletonPtr& atlas)
{
  struct Config
  {
    const char* bodyNode;
    const char* effectorName;
    const char* targetName;
    const char* label;
    int hotkey;
    Eigen::Isometry3d relativeTransform;
    Eigen::Vector4d color;
    bool supportContact;
  };

  constexpr double halfPi = 1.5707963267948966;
  Eigen::Isometry3d leftHand = Eigen::Isometry3d::Identity();
  leftHand.translation() = Eigen::Vector3d(0.0009, 0.1254, 0.012);
  leftHand.rotate(Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d rightHand = leftHand;
  rightHand.translation().x() = -rightHand.translation().x();
  rightHand.translation().y() = -rightHand.translation().y();
  rightHand.linear() = rightHand.linear().inverse().eval();

  Eigen::Isometry3d foot = Eigen::Isometry3d::Identity();
  foot.translation() = Eigen::Vector3d(0.186, 0.0, -0.08);

  const std::array<Config, 4> configs{{
      {"l_hand",
       "atlas_puppet_left_hand",
       "atlas_puppet_ik_target_left_hand",
       "1 left hand",
       '1',
       leftHand,
       {0.18, 0.55, 1.0, 0.92},
       false},
      {"r_hand",
       "atlas_puppet_right_hand",
       "atlas_puppet_ik_target_right_hand",
       "2 right hand",
       '2',
       rightHand,
       {1.0, 0.40, 0.24, 0.92},
       false},
      {"l_foot",
       "atlas_puppet_left_foot",
       "atlas_puppet_ik_target_left_foot",
       "3 left foot",
       '3',
       foot,
       {0.26, 0.86, 0.34, 0.92},
       true},
      {"r_foot",
       "atlas_puppet_right_foot",
       "atlas_puppet_ik_target_right_foot",
       "4 right foot",
       '4',
       foot,
       {0.95, 0.72, 0.18, 0.92},
       true},
  }};

  if (!scene.world || !atlas) {
    return;
  }

  const auto footSupportGeometry = makeAtlasPuppetFootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = atlas->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "Atlas puppet fixture is missing body node "
          + std::string(config.bodyNode));
    }

    auto* endEffector = bodyNode->createEndEffector(config.effectorName);
    if (config.supportContact) {
      endEffector->setRelativeTransform(config.relativeTransform);
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    } else {
      endEffector->setDefaultRelativeTransform(config.relativeTransform, true);
    }

    auto ik = endEffector->getIK(true);
    ik->useWholeBody();
    ik->setGradientMethod<InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);
    if (config.supportContact) {
      ik->setHierarchyLevel(1);
      Eigen::Vector3d linearBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d angularBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      linearBounds.z() = 1e-8;
      angularBounds.x() = 1e-8;
      angularBounds.y() = 1e-8;
      ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
      ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
    }

    auto target = SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    target->setShape(std::make_shared<SphereShape>(0.06));
    target->getVisualAspect(true)->setRGBA(config.color);
    scene.world->addSimpleFrame(target);
    ik->setTarget(target);

    IkHandle handle;
    handle.targetRenderableId = makeRenderableId(*target);
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = std::move(target);
    handle.ik = std::move(ik);
    scene.ikHandles.push_back(std::move(handle));
  }
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

  auto passthrough
      = std::make_shared<dart::utils::CompositeResourceRetriever>();
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
  shapeNode->setRelativeTranslation(
      Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
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
            includePoint(
                transform
                * Eigen::Vector3d(
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

void addG1IkTargets(DartScene& scene, const dart::dynamics::SkeletonPtr& robot)
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

    IkHandle handle;
    handle.targetRenderableId = makeRenderableId(*target);
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = std::move(target);
    handle.ik = std::move(ik);
    scene.ikHandles.push_back(std::move(handle));
  }
}

void setRequiredHuboPuppetDofPosition(
    const dart::dynamics::SkeletonPtr& hubo, const char* name, double position)
{
  auto* dof = hubo ? hubo->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Hubo puppet fixture is missing expected DOF " + std::string(name));
  }
  dof->setPosition(position);
}

void setRequiredHuboPuppetDofLimits(
    const dart::dynamics::SkeletonPtr& hubo,
    const char* name,
    double lower,
    double upper)
{
  auto* dof = hubo ? hubo->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Hubo puppet fixture is missing expected DOF " + std::string(name));
  }
  dof->setPositionLowerLimit(lower);
  dof->setPositionUpperLimit(upper);
}

void setupHuboPuppetStartConfiguration(const dart::dynamics::SkeletonPtr& hubo)
{
  const std::array<std::pair<const char*, double>, 10> jointPositions{{
      {"LHP", -45.0},
      {"LKP", 90.0},
      {"LAP", -45.0},
      {"RHP", -45.0},
      {"RKP", 90.0},
      {"RAP", -45.0},
      {"LSP", 30.0},
      {"LEP", -120.0},
      {"RSP", 30.0},
      {"REP", -120.0},
  }};
  for (const auto& [name, degrees] : jointPositions) {
    setRequiredHuboPuppetDofPosition(hubo, name, dart::math::toRadian(degrees));
  }

  const std::array<const char*, 4> limitedDofs{{"LSY", "LWY", "RSY", "RWY"}};
  for (const char* name : limitedDofs) {
    setRequiredHuboPuppetDofLimits(
        hubo, name, dart::math::toRadian(-90.0), dart::math::toRadian(90.0));
  }
}

void removeHuboPuppetFingerBodyNodes(const dart::dynamics::SkeletonPtr& hubo)
{
  if (!hubo) {
    return;
  }

  for (std::size_t i = 0; i < hubo->getNumBodyNodes();) {
    auto* body = hubo->getBodyNode(i);
    if (body == nullptr) {
      ++i;
      continue;
    }

    const std::string name = body->getName();
    if (name.starts_with("Body_LF") || name.starts_with("Body_RF")) {
      body->remove();
      continue;
    }
    ++i;
  }
}

dart::dynamics::SkeletonPtr loadHuboPuppetSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      "drchubo", dart::config::dataPath("urdf/drchubo"));
  const dart::common::Uri huboUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("urdf/drchubo/drchubo.urdf"));
  auto hubo = dart::io::readSkeleton(huboUri, options);
  if (!hubo) {
    throw std::runtime_error(
        "Failed to load Hubo puppet fixture from " + huboUri.toString());
  }

  hubo->setName(kHuboPuppetRobotFixtureSkeletonName);
  removeHuboPuppetFingerBodyNodes(hubo);
  setupHuboPuppetStartConfiguration(hubo);

  auto* rootBody = hubo->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<FreeJoint*>(rootBody->getParentJoint()) != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation().x() = 0.0;
    transform.translation().y() = 0.0;
    FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(hubo)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      FreeJoint::setTransformOf(rootBody, transform);
    }
  }

  disableSkeletonCollisionAndGravity(hubo);
  return hubo;
}

dart::math::SupportGeometry makeHuboPuppetFootSupportGeometry()
{
  dart::math::SupportGeometry support;
  support.emplace_back(-0.08, 0.05, 0.0);
  support.emplace_back(-0.18, 0.05, 0.0);
  support.emplace_back(-0.18, -0.05, 0.0);
  support.emplace_back(-0.08, -0.05, 0.0);
  return support;
}

void addHuboPuppetIkTargets(
    DartScene& scene, const dart::dynamics::SkeletonPtr& hubo)
{
  struct Config
  {
    const char* bodyNode;
    const char* effectorName;
    const char* targetSuffix;
    const char* label;
    int hotkey;
    Eigen::Isometry3d relativeTransform;
    Eigen::Vector4d color;
    bool supportContact;
  };

  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  hand.translation() = Eigen::Vector3d(0.0, 0.0, -0.09);

  Eigen::Isometry3d foot = Eigen::Isometry3d::Identity();
  foot.translation() = Eigen::Vector3d(0.14, 0.0, -0.126);

  Eigen::Isometry3d peg = Eigen::Isometry3d::Identity();
  peg.translation() = Eigen::Vector3d(0.0, 0.0, 0.09);

  const std::array<Config, 6> configs{{
      {"Body_LWR",
       "hubo_puppet_left_hand",
       "left_hand",
       "1 left hand",
       '1',
       hand,
       {0.18, 0.55, 1.0, 0.92},
       false},
      {"Body_RWR",
       "hubo_puppet_right_hand",
       "right_hand",
       "2 right hand",
       '2',
       hand,
       {1.0, 0.40, 0.24, 0.92},
       false},
      {"Body_LAR",
       "hubo_puppet_left_foot",
       "left_foot",
       "3 left foot",
       '3',
       foot,
       {0.26, 0.86, 0.34, 0.92},
       true},
      {"Body_RAR",
       "hubo_puppet_right_foot",
       "right_foot",
       "4 right foot",
       '4',
       foot,
       {0.95, 0.72, 0.18, 0.92},
       true},
      {"Body_LWP",
       "hubo_puppet_left_peg",
       "left_peg",
       "5 left peg",
       '5',
       peg,
       {0.70, 0.43, 0.96, 0.92},
       false},
      {"Body_RWP",
       "hubo_puppet_right_peg",
       "right_peg",
       "6 right peg",
       '6',
       peg,
       {0.25, 0.88, 0.78, 0.92},
       false},
  }};

  if (!scene.world || !hubo) {
    return;
  }

  const auto footSupportGeometry = makeHuboPuppetFootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = hubo->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "Hubo puppet fixture is missing body node "
          + std::string(config.bodyNode));
    }

    auto* endEffector = bodyNode->createEndEffector(config.effectorName);
    endEffector->setDefaultRelativeTransform(config.relativeTransform, true);
    if (config.supportContact) {
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    }

    auto ik = endEffector->getIK(true);
    ik->useWholeBody();
    ik->setGradientMethod<InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);
    if (config.supportContact) {
      ik->setHierarchyLevel(1);
      Eigen::Vector3d linearBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d angularBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      linearBounds.z() = 1e-8;
      angularBounds.x() = 1e-8;
      angularBounds.y() = 1e-8;
      ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
      ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
    }

    const std::string targetName
        = std::string(kHuboPuppetIkTargetFramePrefix) + config.targetSuffix;
    auto target = SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        targetName,
        endEffector->getWorldTransform());
    target->setShape(std::make_shared<SphereShape>(0.055));
    target->getVisualAspect(true)->setRGBA(config.color);
    scene.world->addSimpleFrame(target);
    ik->setTarget(target);

    IkHandle handle;
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
  body->setInertia(
      dart::dynamics::Inertia(
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

struct TinkertoyFixtureShapes
{
  ShapePtr weldJointShape;
  ShapePtr revoluteJointShape;
  ShapePtr ballJointShape;
  ShapePtr blockShape;
  Eigen::Isometry3d blockOffset = Eigen::Isometry3d::Identity();
};

TinkertoyFixtureShapes createTinkertoyFixtureShapes()
{
  constexpr double blockLength = 0.5;
  constexpr double blockWidth = 0.075;
  constexpr double jointRadius = 1.5 * blockWidth / 2.0;

  TinkertoyFixtureShapes shapes;
  shapes.weldJointShape = std::make_shared<BoxShape>(
      Eigen::Vector3d(2.0 * jointRadius, blockWidth, blockWidth));
  shapes.revoluteJointShape
      = std::make_shared<CylinderShape>(jointRadius, 1.5 * blockWidth);
  shapes.ballJointShape = std::make_shared<SphereShape>(jointRadius);
  shapes.blockShape = std::make_shared<BoxShape>(
      Eigen::Vector3d(blockLength, blockWidth, blockWidth));
  shapes.blockOffset.translation().x() = blockLength / 2.0;
  return shapes;
}

template <class JointType>
dart::dynamics::BodyNode* addTinkertoyBlock(
    World& world,
    const TinkertoyFixtureShapes& shapes,
    dart::dynamics::BodyNode* parent,
    const Eigen::Isometry3d& relativeTransform,
    const ShapePtr& jointShape,
    std::size_t& nextToyIndex)
{
  constexpr double blockMass = 0.16 * 10e3 * 0.5 * 0.075 * 0.075;
  const Eigen::Vector4d pausedColor(0xEE / 255.0, 0xC9 / 255.0, 0.0, 1.0);
  const Eigen::Vector4d jointColor(0.50, 0.50, 1.0, 1.0);

  dart::dynamics::SkeletonPtr skeleton;
  if (parent != nullptr) {
    skeleton = parent->getSkeleton();
  } else {
    skeleton = Skeleton::create(
        std::string(kTinkertoyFixtureSkeletonPrefix)
        + std::to_string(nextToyIndex++));
    world.addSkeleton(skeleton);
  }

  auto [joint, body] = skeleton->createJointAndBodyNodePair<JointType>(parent);
  body->setName("block_" + std::to_string(skeleton->getNumBodyNodes()));
  joint->setName("joint_" + std::to_string(skeleton->getNumJoints()));
  joint->setTransformFromParentBodyNode(relativeTransform);
  if constexpr (std::is_same_v<JointType, RevoluteJoint>) {
    joint->setAxis(Eigen::Vector3d::UnitZ());
  }
  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    joint->setDampingCoefficient(i, 0.4);
  }

  auto* jointShapeNode
      = body->template createShapeNodeWith<VisualAspect>(jointShape);
  jointShapeNode->getVisualAspect()->setRGBA(jointColor);

  auto* blockShapeNode
      = body->template createShapeNodeWith<VisualAspect>(shapes.blockShape);
  blockShapeNode->setRelativeTransform(shapes.blockOffset);
  blockShapeNode->getVisualAspect()->setRGBA(pausedColor);

  body->setInertia(Inertia(
      blockMass,
      0.25 * Eigen::Vector3d::UnitX(),
      std::dynamic_pointer_cast<BoxShape>(shapes.blockShape)
          ->computeInertia(blockMass)));

  return body;
}

void addTinkertoyAxis(
    World& world,
    const std::string& name,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& color)
{
  auto frame = SimpleFrame::createShared(Frame::World(), name);
  auto line = std::make_shared<LineSegmentShape>(2.5f);
  line->addVertex(Eigen::Vector3d::Zero());
  line->addVertex(axis * 0.55);
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->createVisualAspect()->setColor(color);
  world.addSimpleFrame(frame);
}

void addTinkertoyReferenceFrames(World& world)
{
  addTinkertoyAxis(
      world,
      std::string(kTinkertoyAxisFramePrefix) + "x",
      Eigen::Vector3d::UnitX(),
      dart::Color::Red());
  addTinkertoyAxis(
      world,
      std::string(kTinkertoyAxisFramePrefix) + "y",
      Eigen::Vector3d::UnitY(),
      dart::Color::Green());
  addTinkertoyAxis(
      world,
      std::string(kTinkertoyAxisFramePrefix) + "z",
      Eigen::Vector3d::UnitZ(),
      dart::Color::Blue());

  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0.35, -0.55, 0.35);
  auto target = SimpleFrame::createShared(
      Frame::World(), kTinkertoyTargetFrameName, targetTransform);
  target->setShape(std::make_shared<SphereShape>(0.055));
  target->getVisualAspect(true)->setRGBA(dart::Color::Fuchsia(1.0));
  world.addSimpleFrame(target);

  auto forceLine
      = SimpleFrame::createShared(Frame::World(), kTinkertoyForceLineFrameName);
  auto forceLineShape = std::make_shared<LineSegmentShape>(
      Eigen::Vector3d(0.08, -0.15, 0.18), targetTransform.translation(), 3.0f);
  forceLine->setShape(forceLineShape);
  forceLine->createVisualAspect()->setRGBA(
      Eigen::Vector4d(1.0, 0.63, 0.0, 1.0));
  world.addSimpleFrame(forceLine);
}

void addTinkertoyInitialAssemblies(World& world)
{
  TinkertoyFixtureShapes shapes = createTinkertoyFixtureShapes();
  std::size_t nextToyIndex = 0;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(45.0), Eigen::Vector3d::UnitY()));
  auto* firstToy = addTinkertoyBlock<BallJoint>(
      world, shapes, nullptr, transform, shapes.ballJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.5;
  transform.prerotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitX()));
  firstToy = addTinkertoyBlock<RevoluteJoint>(
      world,
      shapes,
      firstToy,
      transform,
      shapes.revoluteJointShape,
      nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitZ()));
  firstToy = addTinkertoyBlock<WeldJoint>(
      world, shapes, firstToy, transform, shapes.weldJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.25;
  transform.translation().z() = 0.075;
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-30.0), Eigen::Vector3d::UnitZ()));
  addTinkertoyBlock<BallJoint>(
      world, shapes, firstToy, transform, shapes.ballJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitY()));
  transform.pretranslate(-1.0 * Eigen::Vector3d::UnitX());
  auto* secondToy = addTinkertoyBlock<BallJoint>(
      world, shapes, nullptr, transform, shapes.ballJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.5;
  transform.translation().z() = 0.25;
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitY()));
  secondToy = addTinkertoyBlock<WeldJoint>(
      world, shapes, secondToy, transform, shapes.weldJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-90.0), Eigen::Vector3d::UnitX()));
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-90.0), Eigen::Vector3d::UnitZ()));
  transform.translation().z() = 0.075 / 2.0;
  addTinkertoyBlock<RevoluteJoint>(
      world,
      shapes,
      secondToy,
      transform,
      shapes.revoluteJointShape,
      nextToyIndex);

  transform.translation().x() = 0.5;
  secondToy = addTinkertoyBlock<RevoluteJoint>(
      world,
      shapes,
      secondToy,
      transform,
      shapes.revoluteJointShape,
      nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.5;
  addTinkertoyBlock<BallJoint>(
      world, shapes, secondToy, transform, shapes.ballJointShape, nextToyIndex);

  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    makeVisualOnlySkeleton(world.getSkeleton(i));
  }
}

dart::dynamics::SkeletonPtr createLcpPhysicsBoxSkeleton(
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color)
{
  auto box = Skeleton::create(name);
  auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>();
  joint->setTransformFromParentBodyNode(transform);

  auto boxShape = std::make_shared<BoxShape>(size);
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(0.0);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(0.0);
  body->setInertia(
      Inertia(mass, Eigen::Vector3d::Zero(), boxShape->computeInertia(mass)));
  return box;
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
      pointMass->setPositions(
          Eigen::Vector3d(
              0.0,
              0.0,
              0.05
                  * std::sin(
                      7.0 * resting.x() + 5.0 * resting.y()
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

  scene.world = World::create("dartsim_mvp");
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
          "data/gltf/pbr_triangle.gltf", Eigen::Vector3d(0.9, 0.9, 0.9))) {
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
  scene.world = World::create("dartsim_hello_world");
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
  scene.world = World::create("dartsim_boxes");
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
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
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
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
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
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
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
  scene.world = World::create("dartsim_hardcoded_design");
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
    const double t
        = numBodyNodes <= 1
              ? 0.0
              : static_cast<double>(i) / static_cast<double>(numBodyNodes - 1);
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
        = dynamic_cast<const dart::dynamics::SoftBodyNode*>(bodyNode)
          != nullptr;
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
  auto skeleton = Skeleton::create(
      std::string(kCouplerConstraintFixtureSkeletonPrefix) + label);

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
      throw std::runtime_error("vehicle fixture is missing skeleton: " + name);
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

class JointConstraintsControllerState
{
public:
  JointConstraintsControllerState(
      dart::dynamics::SkeletonPtr biped, double timeStep)
    : mBiped(std::move(biped)), mTimeStep(timeStep)
  {
    if (!mBiped) {
      throw std::runtime_error("joint_constraints fixture is missing biped");
    }

    mHeelLeft = mBiped->getBodyNode("h_heel_left");
    if (mHeelLeft == nullptr) {
      throw std::runtime_error(
          "joint_constraints fixture is missing h_heel_left");
    }

    const std::size_t dofs = mBiped->getNumDofs();
    mKp = Eigen::MatrixXd::Identity(dofs, dofs);
    mKd = Eigen::MatrixXd::Identity(dofs, dofs);
    mTorques = Eigen::VectorXd::Zero(dofs);
    mDesiredDofs = mBiped->getPositions();
    mConstraintForces = Eigen::VectorXd::Zero(dofs);

    for (std::size_t i = 0; i < std::min<std::size_t>(6, dofs); ++i) {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }
    for (std::size_t i = 6; i < std::min<std::size_t>(22, dofs); ++i) {
      mKp(i, i) = 200.0;
      mKd(i, i) = 100.0;
    }
    for (std::size_t i = 22; i < dofs; ++i) {
      mKp(i, i) = 20.0;
      mKd(i, i) = 10.0;
    }
  }

  void preStep()
  {
    computeTorques(mBiped->getPositions(), mBiped->getVelocities());
    mBiped->setForces(mTorques);
  }

private:
  void computeTorques(
      const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities)
  {
    const Eigen::MatrixXd inverseMass
        = (mBiped->getMassMatrix() + mKd * mTimeStep).inverse();
    const Eigen::VectorXd proportional
        = -mKp * (positions + velocities * mTimeStep - mDesiredDofs);
    const Eigen::VectorXd derivative = -mKd * velocities;
    const Eigen::VectorXd acceleration
        = inverseMass
          * (-mBiped->getCoriolisAndGravityForces() + proportional + derivative
             + mConstraintForces);
    mTorques = proportional + derivative - mKd * acceleration * mTimeStep;

    const Eigen::Vector3d centerOfMass = mBiped->getCOM();
    const Eigen::Vector3d centerOfPressure
        = mHeelLeft->getTransform() * Eigen::Vector3d(0.05, 0.0, 0.0);
    const Eigen::Vector2d offset(
        centerOfMass[0] - centerOfPressure[0],
        centerOfMass[2] - centerOfPressure[2]);
    if (mTorques.size() > 26 && offset[0] < 0.1) {
      const double sagittalOffset = centerOfMass[0] - centerOfPressure[0];
      constexpr double kAnkleHipGain = 20.0;
      constexpr double kBackGain = 10.0;
      constexpr double kDerivativeGain = 100.0;
      const double correction
          = kDerivativeGain * (mPreviousSagittalOffset - sagittalOffset);
      mTorques[17] += -kAnkleHipGain * sagittalOffset + correction;
      mTorques[25] += -kBackGain * sagittalOffset + correction;
      mTorques[19] += -kAnkleHipGain * sagittalOffset + correction;
      mTorques[26] += -kBackGain * sagittalOffset + correction;
      mPreviousSagittalOffset = sagittalOffset;
    }

    for (std::size_t i = 0; i < std::min<std::size_t>(6, mTorques.size());
         ++i) {
      mTorques[i] = 0.0;
    }
  }

  dart::dynamics::SkeletonPtr mBiped;
  dart::dynamics::BodyNode* mHeelLeft = nullptr;
  double mTimeStep = 0.0;
  double mPreviousSagittalOffset = 0.0;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  Eigen::VectorXd mConstraintForces;
};

DartScene createHybridDynamicsScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/fullbody1.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load hybrid_dynamics fixture from "
        "dart://sample/skel/fullbody1.skel");
  }
  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = scene.world->getSkeleton("ground skeleton");
  if (!ground) {
    throw std::runtime_error("hybrid_dynamics fixture is missing ground");
  }
  ground->setName(kHybridDynamicsFixtureGroundSkeletonName);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.46, 0.50, 0.46));
  }

  auto biped = scene.world->getSkeleton("fullbody1");
  if (!biped) {
    throw std::runtime_error("hybrid_dynamics fixture is missing fullbody1");
  }
  biped->setName(kHybridDynamicsFixtureBipedSkeletonName);

  const std::vector<std::size_t> genCoordIds{
      1,  // global orientation y
      6,  // left hip
      9,  // left knee
      10, // left ankle
      13, // right hip
      16, // right knee
      17, // right ankle
      21, // lower back
  };
  Eigen::VectorXd initConfig(8);
  initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
  biped->setPositions(genCoordIds, initConfig);

  if (auto* rootJoint = biped->getJoint(0)) {
    rootJoint->setActuatorType(dart::dynamics::Joint::PASSIVE);
  }
  for (std::size_t i = 1; i < biped->getNumJoints(); ++i) {
    if (auto* joint = biped->getJoint(i)) {
      joint->setActuatorType(dart::dynamics::Joint::VELOCITY);
    }
  }

  for (std::size_t i = 0; i < biped->getNumBodyNodes(); ++i) {
    auto* body = biped->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    const double t
        = biped->getNumBodyNodes() <= 1
              ? 0.0
              : static_cast<double>(i)
                    / static_cast<double>(biped->getNumBodyNodes() - 1);
    body->setColor(
        Eigen::Vector3d(0.25 + 0.45 * t, 0.42 + 0.22 * t, 0.78 - 0.35 * t));
  }
  if (auto* head = biped->getBodyNode("h_head")) {
    head->setColor(Eigen::Vector3d(0.88, 0.70, 0.46));
  }
  if (auto* spine = biped->getBodyNode("h_spine")) {
    spine->setColor(Eigen::Vector3d(0.22, 0.48, 0.86));
  }

  return scene;
}

DartScene createJointConstraintsScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld("dart://sample/skel/fullbody1.skel");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load joint_constraints fixture from "
        "dart://sample/skel/fullbody1.skel");
  }
  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = scene.world->getSkeleton("ground skeleton");
  if (!ground) {
    throw std::runtime_error("joint_constraints fixture is missing ground");
  }
  ground->setName(kJointConstraintsFixtureGroundSkeletonName);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.42, 0.46, 0.44));
  }

  auto biped = scene.world->getSkeleton("fullbody1");
  if (!biped) {
    throw std::runtime_error("joint_constraints fixture is missing fullbody1");
  }
  biped->setName(kJointConstraintsFixtureBipedSkeletonName);

  const std::vector<std::size_t> genCoordIds{
      1,  // global orientation y
      4,  // global position y
      6,  // left hip
      9,  // left knee
      10, // left ankle
      13, // right hip
      16, // right knee
      17, // right ankle
      21, // lower back
  };
  Eigen::VectorXd initConfig(9);
  initConfig << -0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1;
  biped->setPositions(genCoordIds, initConfig);

  for (std::size_t i = 0; i < biped->getNumBodyNodes(); ++i) {
    auto* body = biped->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    const double t
        = biped->getNumBodyNodes() <= 1
              ? 0.0
              : static_cast<double>(i)
                    / static_cast<double>(biped->getNumBodyNodes() - 1);
    body->setColor(
        Eigen::Vector3d(0.24 + 0.34 * t, 0.52 - 0.20 * t, 0.34 + 0.24 * t));
  }
  if (auto* head = biped->getBodyNode("h_head")) {
    head->setColor(Eigen::Vector3d(0.84, 0.68, 0.50));
  }
  if (auto* spine = biped->getBodyNode("h_spine")) {
    spine->setColor(Eigen::Vector3d(0.30, 0.58, 0.34));
  }

  auto controller = std::make_shared<JointConstraintsControllerState>(
      biped, scene.world->getTimeStep());
  scene.preStep = [controller = std::move(controller)]() {
    controller->preStep();
  };

  return scene;
}

struct FreeJointCasesTorqueFreeState
{
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d omegaBody{Eigen::Vector3d::Zero()};
};

struct FreeJointCasesFixture
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::FreeJoint* joint = nullptr;
  dart::dynamics::SkeletonPtr referenceSkeleton;
  dart::dynamics::FreeJoint* referenceJoint = nullptr;
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inertiaInv{Eigen::Matrix3d::Identity()};
  FreeJointCasesTorqueFreeState torqueFreeState;
  Eigen::Vector6d initialPositions{Eigen::Vector6d::Zero()};
  Eigen::Vector6d initialVelocities{Eigen::Vector6d::Zero()};
};

struct FreeJointCasesTorqueFreeDeriv
{
  Eigen::Vector4d orientationDot{Eigen::Vector4d::Zero()};
  Eigen::Vector3d omegaBodyDot{Eigen::Vector3d::Zero()};
};

Eigen::Vector3d computeFreeJointCasesOmegaBodyDot(
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv,
    const Eigen::Vector3d& omegaBody)
{
  return -inertiaInv * (omegaBody.cross(inertia * omegaBody));
}

Eigen::Vector4d computeFreeJointCasesQuaternionCoeffDot(
    const Eigen::Quaterniond& orientation, const Eigen::Vector3d& omegaBody)
{
  const Eigen::Quaterniond omegaQuat(
      0.0, omegaBody.x(), omegaBody.y(), omegaBody.z());
  Eigen::Quaterniond qdot = orientation * omegaQuat;
  qdot.coeffs() *= 0.5;
  return qdot.coeffs();
}

FreeJointCasesTorqueFreeDeriv evaluateFreeJointCasesTorqueFreeDeriv(
    const FreeJointCasesTorqueFreeState& state,
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv)
{
  FreeJointCasesTorqueFreeDeriv deriv;
  deriv.orientationDot = computeFreeJointCasesQuaternionCoeffDot(
      state.orientation, state.omegaBody);
  deriv.omegaBodyDot
      = computeFreeJointCasesOmegaBodyDot(inertia, inertiaInv, state.omegaBody);
  return deriv;
}

void integrateFreeJointCasesTorqueFreeStep(
    FreeJointCasesTorqueFreeState& state,
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv,
    double dt)
{
  if (!(dt > 0.0)) {
    return;
  }

  const FreeJointCasesTorqueFreeDeriv k1
      = evaluateFreeJointCasesTorqueFreeDeriv(state, inertia, inertiaInv);

  FreeJointCasesTorqueFreeState s2 = state;
  s2.orientation.coeffs() += 0.5 * dt * k1.orientationDot;
  s2.omegaBody += 0.5 * dt * k1.omegaBodyDot;
  s2.orientation.normalize();
  const FreeJointCasesTorqueFreeDeriv k2
      = evaluateFreeJointCasesTorqueFreeDeriv(s2, inertia, inertiaInv);

  FreeJointCasesTorqueFreeState s3 = state;
  s3.orientation.coeffs() += 0.5 * dt * k2.orientationDot;
  s3.omegaBody += 0.5 * dt * k2.omegaBodyDot;
  s3.orientation.normalize();
  const FreeJointCasesTorqueFreeDeriv k3
      = evaluateFreeJointCasesTorqueFreeDeriv(s3, inertia, inertiaInv);

  FreeJointCasesTorqueFreeState s4 = state;
  s4.orientation.coeffs() += dt * k3.orientationDot;
  s4.omegaBody += dt * k3.omegaBodyDot;
  s4.orientation.normalize();
  const FreeJointCasesTorqueFreeDeriv k4
      = evaluateFreeJointCasesTorqueFreeDeriv(s4, inertia, inertiaInv);

  state.orientation.coeffs()
      += (dt / 6.0)
         * (k1.orientationDot + 2.0 * k2.orientationDot
            + 2.0 * k3.orientationDot + k4.orientationDot);
  state.omegaBody += (dt / 6.0)
                     * (k1.omegaBodyDot + 2.0 * k2.omegaBodyDot
                        + 2.0 * k3.omegaBodyDot + k4.omegaBodyDot);
  state.orientation.normalize();
}

FreeJointCasesFixture createFreeJointCasesFixture(
    std::size_t index,
    const char* label,
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& velocities,
    const Eigen::Vector4d& color)
{
  FreeJointCasesFixture fixture;
  fixture.skeleton = Skeleton::create(
      std::string(kFreeJointCasesActiveSkeletonPrefix) + std::to_string(index));
  auto [joint, body]
      = fixture.skeleton->createJointAndBodyNodePair<FreeJoint>();
  fixture.joint = joint;
  body->setName(label);
  body->setMass(1.0);
  body->setMomentOfInertia(0.02, 0.04, 0.06);
  body->setCollidable(false);
  fixture.inertia = Eigen::Vector3d(0.02, 0.04, 0.06).asDiagonal();
  fixture.inertiaInv = fixture.inertia.inverse();

  const Eigen::Vector3d boxSize(0.3, 0.2, 0.15);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(std::make_shared<BoxShape>(boxSize));
  shapeNode->getVisualAspect()->setRGBA(color);

  joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  fixture.initialPositions = FreeJoint::convertToPositions(pose);
  fixture.initialVelocities = velocities;
  joint->setPositions(fixture.initialPositions);
  joint->setVelocities(fixture.initialVelocities);

  const Eigen::Matrix3d rotation = pose.linear();
  fixture.torqueFreeState.orientation = Eigen::Quaterniond(rotation);
  fixture.torqueFreeState.orientation.normalize();
  fixture.torqueFreeState.omegaBody
      = rotation.transpose() * velocities.head<3>();

  fixture.referenceSkeleton = Skeleton::create(
      std::string(kFreeJointCasesReferenceSkeletonPrefix)
      + std::to_string(index));
  fixture.referenceSkeleton->setMobile(false);
  auto [referenceJoint, referenceBody]
      = fixture.referenceSkeleton->createJointAndBodyNodePair<FreeJoint>();
  fixture.referenceJoint = referenceJoint;
  referenceBody->setName(std::string(label) + " reference");
  referenceBody->setCollidable(false);

  Eigen::Vector4d referenceColor = color;
  referenceColor.head<3>()
      = 0.25 * Eigen::Vector3d::Ones() + 0.75 * referenceColor.head<3>();
  referenceColor[3] = std::min(referenceColor[3], 0.22);

  auto referenceShapeNode = referenceBody->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(boxSize * 1.05));
  referenceShapeNode->getVisualAspect()->setRGBA(referenceColor);
  referenceShapeNode->getVisualAspect()->setShadowed(false);

  referenceJoint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  referenceJoint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  referenceJoint->setPositions(fixture.initialPositions);
  referenceJoint->setVelocities(Eigen::Vector6d::Zero());

  return fixture;
}

class FreeJointCasesControllerState
{
public:
  FreeJointCasesControllerState(
      dart::simulation::WorldPtr world,
      std::vector<FreeJointCasesFixture> fixtures)
    : mWorld(std::move(world)), mFixtures(std::move(fixtures))
  {
    if (!mWorld) {
      throw std::runtime_error("free_joint_cases fixture is missing world");
    }
  }

  void preStep()
  {
    const double targetTime = mWorld->getTime() + mWorld->getTimeStep();
    if (targetTime < mTorqueFreeTime) {
      resetTorqueFreeState();
    }
    advanceTorqueFreeState(targetTime);
    updateReferenceBodies(targetTime);
  }

private:
  void resetTorqueFreeState()
  {
    mTorqueFreeTime = 0.0;
    for (auto& fixture : mFixtures) {
      const Eigen::Isometry3d initial
          = FreeJoint::convertToTransform(fixture.initialPositions);
      fixture.torqueFreeState.orientation
          = Eigen::Quaterniond(initial.linear());
      fixture.torqueFreeState.orientation.normalize();
      fixture.torqueFreeState.omegaBody
          = initial.linear().transpose() * fixture.initialVelocities.head<3>();
    }
  }

  void advanceTorqueFreeState(double targetTime)
  {
    const double timeStep = mWorld->getTimeStep();
    if (!(timeStep > 0.0) || !(targetTime > mTorqueFreeTime)) {
      return;
    }

    constexpr int kSubsteps = 10;
    const double baseStep = timeStep / static_cast<double>(kSubsteps);
    double time = mTorqueFreeTime;
    while (time + 1e-12 < targetTime) {
      const double dt = std::min(baseStep, targetTime - time);
      for (auto& fixture : mFixtures) {
        integrateFreeJointCasesTorqueFreeStep(
            fixture.torqueFreeState, fixture.inertia, fixture.inertiaInv, dt);
      }
      time += dt;
    }
    mTorqueFreeTime = targetTime;
  }

  void updateReferenceBodies(double time)
  {
    for (auto& fixture : mFixtures) {
      if (fixture.referenceJoint == nullptr) {
        continue;
      }

      Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
      expected.linear()
          = fixture.torqueFreeState.orientation.toRotationMatrix();
      expected.translation() = fixture.initialPositions.tail<3>()
                               + fixture.initialVelocities.tail<3>() * time;
      fixture.referenceJoint->setPositions(
          FreeJoint::convertToPositions(expected));
    }
  }

  dart::simulation::WorldPtr mWorld;
  std::vector<FreeJointCasesFixture> mFixtures;
  double mTorqueFreeTime = 0.0;
};

DartScene createFreeJointCasesScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_free_joint_cases");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->setTimeStep(1e-3);

  std::vector<FreeJointCasesFixture> fixtures;
  fixtures.reserve(kFreeJointCasesCaseCount);

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.tail<3>() = Eigen::Vector3d(0.7, 0.0, 0.0);
    fixtures.push_back(createFreeJointCasesFixture(
        0,
        "linear_only",
        pose,
        velocity,
        Eigen::Vector4d(0.15, 0.35, 0.95, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(0.0, 0.9, 0.0);
    fixtures.push_back(createFreeJointCasesFixture(
        1,
        "angular_only",
        pose,
        velocity,
        Eigen::Vector4d(0.90, 0.12, 0.12, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
    pose.linear() = dart::math::expMapRot(Eigen::Vector3d(0.6, -0.3, 0.2));
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(0.6, -0.3, 0.4);
    velocity.tail<3>() = Eigen::Vector3d(0.5, 0.2, -0.1);
    fixtures.push_back(createFreeJointCasesFixture(
        2,
        "linear_angular",
        pose,
        velocity,
        Eigen::Vector4d(0.24, 0.78, 0.32, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(6.0, 0.0, 0.0);
    pose.linear() = dart::math::expMapRot(
        Eigen::Vector3d(dart::math::pi - 1e-6, 0.0, 0.0));
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(0.2, 0.8, -0.4);
    velocity.tail<3>() = Eigen::Vector3d(0.3, -0.1, 0.2);
    fixtures.push_back(createFreeJointCasesFixture(
        3,
        "near_pi_rotation",
        pose,
        velocity,
        Eigen::Vector4d(0.95, 0.58, 0.10, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(8.0, 0.0, 0.0);
    pose.linear() = dart::math::expMapRot(Eigen::Vector3d(0.3, 1.2, -0.4));
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(1.2, 0.9, -1.0);
    velocity.tail<3>() = Eigen::Vector3d(0.2, 0.3, 0.0);
    fixtures.push_back(createFreeJointCasesFixture(
        4,
        "high_omega_multi_axis",
        pose,
        velocity,
        Eigen::Vector4d(0.82, 0.22, 0.78, 0.7)));
  }

  for (const auto& fixture : fixtures) {
    scene.world->addSkeleton(fixture.skeleton);
    scene.world->addSkeleton(fixture.referenceSkeleton);
  }

  auto controller = std::make_shared<FreeJointCasesControllerState>(
      scene.world, std::move(fixtures));
  scene.preStep = [controller = std::move(controller)]() {
    controller->preStep();
  };

  return scene;
}

DartScene createHumanJointLimitsScene()
{
  DartScene scene;
  const dart::common::Uri worldUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("skel/kima/kima_human_edited.skel"));
  scene.world = dart::io::readWorld(worldUri);
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load human_joint_limits fixture from "
        + worldUri.toString());
  }
  scene.world->setGravity(Eigen::Vector3d::Zero());

  auto ground = scene.world->getSkeleton("ground skeleton");
  if (!ground) {
    throw std::runtime_error("human_joint_limits fixture is missing ground");
  }
  ground->setName(kHumanJointLimitsFixtureGroundSkeletonName);
  makeVisualOnlySkeleton(ground);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.58, 0.62, 0.60));
  }

  auto human = scene.world->getSkeleton("human");
  if (!human) {
    throw std::runtime_error("human_joint_limits fixture is missing human");
  }
  human->setName(kHumanJointLimitsFixtureSkeletonName);
  makeVisualOnlySkeleton(human);
  human->eachJoint(
      [](dart::dynamics::Joint* joint) { joint->setLimitEnforcement(true); });

  setRequiredJointPositions(human, "j_bicep_left", {-0.35, 0.20, 0.55});
  setRequiredJointPositions(human, "j_forearm_left", {0.75});
  setRequiredJointPositions(human, "j_bicep_right", {0.35, 0.20, -0.55});
  setRequiredJointPositions(human, "j_forearm_right", {0.75});
  setRequiredJointPositions(human, "j_thigh_left", {0.08, -0.12, 0.08});
  setRequiredJointPositions(human, "j_shin_left", {0.28});
  setRequiredJointPositions(human, "j_heel_left", {-0.12, 0.05});
  setRequiredJointPositions(human, "j_thigh_right", {-0.08, -0.12, -0.08});
  setRequiredJointPositions(human, "j_shin_right", {0.28});
  setRequiredJointPositions(human, "j_heel_right", {-0.12, -0.05});

  for (std::size_t i = 0; i < human->getNumBodyNodes(); ++i) {
    auto* body = human->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    const double t
        = human->getNumBodyNodes() <= 1
              ? 0.0
              : static_cast<double>(i)
                    / static_cast<double>(human->getNumBodyNodes() - 1);
    body->setColor(
        Eigen::Vector3d(0.68 + 0.10 * t, 0.52 + 0.10 * t, 0.38 + 0.08 * t));
  }
  if (auto* pelvis = human->getBodyNode("pelvis")) {
    pelvis->setColor(Eigen::Vector3d(0.42, 0.50, 0.66));
  }
  if (auto* thorax = human->getBodyNode("thorax")) {
    thorax->setColor(Eigen::Vector3d(0.36, 0.54, 0.70));
  }
  if (auto* head = human->getBodyNode("head")) {
    head->setColor(Eigen::Vector3d(0.84, 0.68, 0.52));
  }

  return scene;
}

DartScene createLcpPhysicsScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_lcp_physics");
  scene.world->setTimeStep(0.001);
  scene.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  if (auto* solver = scene.world->getConstraintSolver()) {
    solver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
  }

  constexpr double groundThickness = 0.08;
  auto ground = createBoxGroundSkeleton(
      kLcpPhysicsFixtureGroundSkeletonName,
      Eigen::Vector3d(7.0, groundThickness, 4.5),
      Eigen::Vector3d(0.72, 0.74, 0.72),
      0.3);
  if (auto* groundBody = ground->getBodyNode(0)) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation().y() = -0.5 * groundThickness;
    groundBody->getParentJoint()->setTransformFromParentBodyNode(transform);
  }
  scene.world->addSkeleton(ground);

  auto addBox = [&](std::string suffix,
                    const Eigen::Vector3d& size,
                    double mass,
                    const Eigen::Isometry3d& transform,
                    const Eigen::Vector3d& color) {
    scene.world->addSkeleton(createLcpPhysicsBoxSkeleton(
        std::string(kLcpPhysicsFixtureBoxSkeletonPrefix) + std::move(suffix),
        size,
        mass,
        transform,
        color));
  };
  auto addTranslatedBox = [&](std::string suffix,
                              const Eigen::Vector3d& size,
                              double mass,
                              const Eigen::Vector3d& position,
                              const Eigen::Vector3d& color) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = position;
    addBox(std::move(suffix), size, mass, transform, color);
  };

  constexpr double massBoxSize = 0.36;
  addTranslatedBox(
      "light_mass",
      Eigen::Vector3d(massBoxSize, massBoxSize, massBoxSize),
      1.0,
      Eigen::Vector3d(-2.0, 0.5 * massBoxSize, -0.85),
      Eigen::Vector3d(0.30, 0.72, 0.36));
  addTranslatedBox(
      "heavy_mass",
      Eigen::Vector3d(massBoxSize, massBoxSize, massBoxSize),
      1000.0,
      Eigen::Vector3d(-2.0, 1.5 * massBoxSize, -0.85),
      Eigen::Vector3d(0.84, 0.24, 0.20));

  constexpr int pyramidLayers = 4;
  constexpr double pyramidBoxSize = 0.28;
  int pyramidBoxIndex = 0;
  for (int layer = 0; layer < pyramidLayers; ++layer) {
    const int boxesInLayer = pyramidLayers - layer;
    const double y = 0.5 * pyramidBoxSize + layer * pyramidBoxSize * 1.05;
    const double startX = -0.5 * (boxesInLayer - 1) * pyramidBoxSize * 1.12;
    for (int i = 0; i < boxesInLayer; ++i) {
      const double x = startX + i * pyramidBoxSize * 1.12;
      const double t
          = static_cast<double>(pyramidBoxIndex)
            / static_cast<double>(pyramidLayers * (pyramidLayers + 1) / 2);
      addTranslatedBox(
          "stack_" + std::to_string(pyramidBoxIndex),
          Eigen::Vector3d::Constant(pyramidBoxSize),
          1.0,
          Eigen::Vector3d(x, y, -0.55),
          Eigen::Vector3d(0.28 + 0.42 * t, 0.52, 0.82 - 0.34 * t));
      ++pyramidBoxIndex;
    }
  }

  constexpr int dominoCount = 8;
  constexpr double dominoSpacing = 0.16;
  for (int i = 0; i < dominoCount; ++i) {
    const double x
        = (static_cast<double>(i) - 0.5 * (dominoCount - 1)) * dominoSpacing;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.18, 0.85);
    if (i == 0) {
      transform.rotate(Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()));
    }
    const double t = static_cast<double>(i) / static_cast<double>(dominoCount);
    addBox(
        "domino_" + std::to_string(i),
        Eigen::Vector3d(0.055, 0.32, 0.15),
        0.5,
        transform,
        Eigen::Vector3d(0.18 + 0.52 * t, 0.34, 0.84 - 0.42 * t));
  }

  int sphereIndex = 0;
  for (int xIndex = 0; xIndex < 4; ++xIndex) {
    for (int zIndex = 0; zIndex < 3; ++zIndex) {
      const double t = static_cast<double>(sphereIndex)
                       / static_cast<double>(kLcpPhysicsFixtureSphereCount);
      scene.world->addSkeleton(createDynamicSphereSkeleton(
          std::string(kLcpPhysicsFixtureSphereSkeletonPrefix)
              + std::to_string(sphereIndex),
          0.11,
          Eigen::Vector3d(
              1.45 + (static_cast<double>(xIndex) - 1.5) * 0.28,
              0.65 + 0.10 * static_cast<double>(zIndex),
              -0.55 + (static_cast<double>(zIndex) - 1.0) * 0.28),
          Eigen::Vector3d(0.86 - 0.35 * t, 0.42 + 0.35 * t, 0.36),
          0.5));
      ++sphereIndex;
    }
  }

  return scene;
}

DartScene createMimicPendulumsScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld(
      "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf");
  if (!scene.world) {
    throw std::runtime_error(
        "Failed to load mimic_pendulums fixture from "
        "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf");
  }
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto ground = scene.world->getSkeleton("ground_plane");
  if (!ground) {
    throw std::runtime_error("mimic_pendulums fixture is missing ground_plane");
  }
  ground->setName("visual_mimic_pendulums_imported_ground");
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kMimicPendulumsFixtureGroundSkeletonName,
      std::make_shared<BoxShape>(Eigen::Vector3d(2.2, 7.2, 0.04)),
      Eigen::Vector3d(0.0, 3.0, -0.02),
      Eigen::Vector3d(0.47, 0.50, 0.47)));

  const std::array<std::pair<const char*, Eigen::Vector3d>, 3> rigs{{
      {"pendulum_with_base", Eigen::Vector3d(0.62, 0.62, 0.62)},
      {"pendulum_with_base_mimic_slow_follows_fast",
       Eigen::Vector3d(0.90, 0.34, 0.34)},
      {"pendulum_with_base_mimic_fast_follows_slow",
       Eigen::Vector3d(0.28, 0.48, 0.92)},
  }};

  for (std::size_t i = 0; i < rigs.size(); ++i) {
    auto skeleton = scene.world->getSkeleton(rigs[i].first);
    if (!skeleton) {
      throw std::runtime_error(
          "mimic_pendulums fixture is missing skeleton: "
          + std::string(rigs[i].first));
    }
    skeleton->setName(
        std::string(kMimicPendulumsFixtureSkeletonPrefix) + std::to_string(i));
    for (std::size_t bodyIndex = 0; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      if (auto* body = skeleton->getBodyNode(bodyIndex)) {
        body->setColor(rigs[i].second);
      }
    }
  }

  return scene;
}

DartScene createAtlasPuppetScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_atlas_puppet");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kAtlasPuppetFixtureGroundSkeletonName,
      std::make_shared<BoxShape>(Eigen::Vector3d(4.0, 4.0, 0.04)),
      Eigen::Vector3d(0.0, 0.0, -0.02),
      Eigen::Vector3d(0.86, 0.88, 0.90)));

  auto atlas = loadAtlasPuppetSkeleton();
  scene.world->addSkeleton(atlas);
  addAtlasPuppetIkTargets(scene, atlas);

  return scene;
}

DartScene createHuboPuppetScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_hubo_puppet");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kHuboPuppetFixtureGroundSkeletonName,
      std::make_shared<BoxShape>(Eigen::Vector3d(4.0, 4.0, 0.04)),
      Eigen::Vector3d(0.0, 0.0, -0.02),
      Eigen::Vector3d(0.86, 0.88, 0.90)));

  auto hubo = loadHuboPuppetSkeleton();
  scene.world->addSkeleton(hubo);
  addHuboPuppetIkTargets(scene, hubo);

  return scene;
}

DartScene createAtlasSimbiconScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_atlas_simbicon");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  constexpr double groundThickness = 0.08;
  auto ground = createBoxGroundSkeleton(
      kAtlasSimbiconFixtureGroundSkeletonName,
      Eigen::Vector3d(5.5, groundThickness, 4.0),
      Eigen::Vector3d(0.74, 0.76, 0.72));
  if (auto* groundBody = ground->getBodyNode(0)) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation().y() = -0.5 * groundThickness;
    groundBody->getParentJoint()->setTransformFromParentBodyNode(transform);
  }
  scene.world->addSkeleton(ground);
  scene.world->addSkeleton(loadAtlasSimbiconSkeleton());

  return scene;
}

DartScene createOperationalSpaceControlScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_operational_space_control");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kOperationalSpaceControlGroundSkeletonName,
      std::make_shared<BoxShape>(Eigen::Vector3d(5.0, 5.0, 0.01)),
      Eigen::Vector3d(0.0, 0.0, -0.005),
      Eigen::Vector3d(0.18, 0.32, 0.58)));

  auto wam = loadOperationalSpaceControlWamSkeleton();
  auto* endEffector = wam->getBodyNode("/wam7");
  if (endEffector == nullptr) {
    throw std::runtime_error(
        "Operational-space WAM fixture is missing /wam7 body node");
  }

  Eigen::Isometry3d targetTransform = endEffector->getWorldTransform();
  targetTransform.pretranslate(Eigen::Vector3d(0.05, 0.0, 0.0));
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      kOperationalSpaceControlTargetFrameName,
      targetTransform);
  target->setShape(std::make_shared<SphereShape>(0.04));
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.92, 0.08, 0.08, 1.0));
  scene.world->addSimpleFrame(target);

  auto controller = std::make_shared<OperationalSpaceControlState>(wam, target);
  scene.world->addSkeleton(wam);
  scene.preStep = [controller = std::move(controller)]() {
    controller->preStep();
  };

  return scene;
}

DartScene createWamIkFastScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_wam_ikfast");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createStaticVisualSkeleton(
      kWamIkFastGroundSkeletonName,
      std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.01)),
      Eigen::Vector3d(0.0, 0.0, -0.005),
      Eigen::Vector3d(0.18, 0.32, 0.58)));

  auto wam = loadWamIkFastSkeleton();
  auto* endEffector = wam->getBodyNode("/wam7");
  if (endEffector == nullptr) {
    throw std::runtime_error("WAM IKFast fixture is missing /wam7 body node");
  }

  Eigen::Isometry3d targetTransform = endEffector->getWorldTransform();
  targetTransform.translate(Eigen::Vector3d(0.0, 0.0, -0.09));
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      kWamIkFastTargetFrameName,
      targetTransform);
  target->setShape(std::make_shared<SphereShape>(0.045));
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.18, 0.55, 1.0, 0.92));
  scene.world->addSimpleFrame(target);
  scene.world->addSkeleton(wam);

  return scene;
}

DartScene createFetchScene()
{
  DartScene scene;
  scene.world = dart::io::readWorld(
      "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml");
  if (!scene.world) {
    throw std::runtime_error("Failed to load Fetch pick-and-place MJCF world");
  }
  scene.world->setGravity(Eigen::Vector3d::Zero());

  auto robot = scene.world->getSkeleton(kFetchRobotFixtureSkeletonName);
  if (!robot) {
    throw std::runtime_error("Fetch fixture is missing robot0:base_link");
  }
  const auto setRobotPosition = [&](std::size_t index, double value) {
    if (index >= robot->getNumDofs()) {
      throw std::runtime_error("Fetch fixture robot has too few DOFs");
    }
    robot->setPosition(index, value);
  };
  setRobotPosition(0, 0.405);
  setRobotPosition(1, 0.480);
  setRobotPosition(2, 0.000);
  setRobotPosition(6, 0.01);
  setRobotPosition(7, -0.73);
  setRobotPosition(8, 0.00);
  setRobotPosition(9, 1.64);
  setRobotPosition(10, 0.0);
  setRobotPosition(11, 0.66);
  setRobotPosition(12, 0.01);

  auto object = scene.world->getSkeleton(kFetchObjectFixtureSkeletonName);
  if (!object) {
    throw std::runtime_error("Fetch fixture is missing object0");
  }
  const auto setObjectPosition = [&](std::size_t index, double value) {
    if (index >= object->getNumDofs()) {
      throw std::runtime_error("Fetch fixture object has too few DOFs");
    }
    object->setPosition(index, value);
  };
  setObjectPosition(3, 1.25);
  setObjectPosition(4, 0.53);
  setObjectPosition(5, 0.40);

  auto mocap = scene.world->getSkeleton("robot0:mocap");
  if (!mocap) {
    throw std::runtime_error("Fetch fixture is missing robot0:mocap");
  }
  auto* constraintSolver = scene.world->getConstraintSolver();
  if (constraintSolver != nullptr
      && constraintSolver->getNumConstraints() > 0) {
    if (auto weldJointConstraint
        = std::dynamic_pointer_cast<dart::constraint::WeldJointConstraint>(
            constraintSolver->getConstraint(0))) {
      weldJointConstraint->setRelativeTransform(Eigen::Isometry3d::Identity());
    }
  }

  constexpr double halfPi = 1.5707963267948966;
  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(1.3, 0.75, 0.50);
  targetTransform.linear()
      = Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto target = SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kFetchTargetFrameName, targetTransform);
  target->setShape(std::make_shared<SphereShape>(0.06));
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.18, 0.86, 0.34, 0.92));
  scene.world->addSimpleFrame(target);

  for (std::size_t i = 0; i < scene.world->getNumSkeletons(); ++i) {
    makeVisualOnlySkeleton(scene.world->getSkeleton(i));
  }

  auto* mocapRoot = mocap->getRootBodyNode();
  scene.preStep = [mocapRoot, target]() {
    if (mocapRoot != nullptr && mocapRoot->getParentJoint() != nullptr) {
      mocapRoot->getParentJoint()->setTransformFromParentBodyNode(
          target->getTransform());
    }
  };

  return scene;
}

DartScene createTinkertoyScene()
{
  DartScene scene;
  scene.world = World::create();
  scene.world->setGravity(Eigen::Vector3d::Zero());
  addTinkertoyReferenceFrames(*scene.world);
  addTinkertoyInitialAssemblies(*scene.world);
  return scene;
}

DartScene createDragAndDropScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_drag_and_drop");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(4.0, -4.0, 0.0);
  auto anchor = std::make_shared<SimpleFrame>(
      dart::dynamics::Frame::World(), "interactive frame", transform);
  anchor->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.45, 0.45, 0.45)));
  anchor->getVisualAspect(true)->setColor(Eigen::Vector3d(0.95, 0.7, 0.15));
  scene.world->addSimpleFrame(anchor);

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-4.0, 4.0, 0.0);
  auto draggable = anchor->spawnChildSimpleFrame("draggable", transform);
  draggable->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
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

  addMarker(
      "X", Eigen::Vector3d(8.0, 0.0, 0.0), Eigen::Vector3d(0.9, 0.0, 0.0));
  addMarker(
      "Y", Eigen::Vector3d(0.0, 8.0, 0.0), Eigen::Vector3d(0.0, 0.9, 0.0));
  addMarker(
      "Z", Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.9));

  return scene;
}

DartScene createSimpleFramesScene()
{
  DartScene scene;
  scene.world = World::create("dartsim_simple_frames");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translate(Eigen::Vector3d(0.1, -0.1, 0.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translate(Eigen::Vector3d(0.0, 0.1, 0.0));
  tf2.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(45.0), Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translate(Eigen::Vector3d(0.0, 0.0, 0.1));
  tf3.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(60.0), Eigen::Vector3d::UnitY()));

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
  markerRoot->setShape(
      std::make_shared<dart::dynamics::EllipsoidShape>(
          Eigen::Vector3d(0.02, 0.02, 0.02)));
  setColor(markerRoot, Eigen::Vector4d(0.95, 0.75, 0.20, 1.0));
  addFrame(markerRoot);

  const auto addMarker
      = [&](const std::string& name, const Eigen::Isometry3d& transform) {
          auto marker = markerRoot->spawnChildSimpleFrame(name, transform);
          marker->setShape(
              std::make_shared<dart::dynamics::EllipsoidShape>(
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
      Eigen::Vector3d(0.1, -0.1, 0.0), Eigen::Vector3d(0.1, 0.0, 0.0), 2.0f);
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
  scene.world = World::create("dartsim_point_cloud");
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
      const double zf = 0.05 * std::sin(5.0 * xf) + 0.06 * std::cos(4.0 * yf);
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
  sensor->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.95, 0.18, 0.12, 1.0));
  scene.world->addSimpleFrame(sensor);

  return scene;
}

DartScene createCapsuleGroundContactScene()
{
  constexpr double kCapsuleRadius = 0.2;
  constexpr double kCapsuleHeight = 0.6;
  constexpr double kGroundVisualThickness = 0.08;

  DartScene scene;
  scene.world = World::create("dartsim_capsule_ground_contact");
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
  groundOffset.translation() = planeNormal * (-0.5 * kGroundVisualThickness);
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
  scene.world = World::create("dartsim_simulation_event_handler");
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
  scene.world = World::create("dartsim_polyhedron");
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
  scene.world = World::create("dartsim_heightmap");
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
  scene.world = World::create("dartsim_g1");
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

} // namespace dart::gui::detail
