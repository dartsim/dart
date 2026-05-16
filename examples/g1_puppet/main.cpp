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

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/package_resource_retriever.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/geometry.hpp>

#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr const char* kG1SkeletonName = "visual_g1_robot";
constexpr const char* kGroundSkeletonName = "g1_ground";

struct G1Options
{
  std::string packageName = "g1_description";
  std::string packageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string robotUri = "package://g1_description/g1_29dof.urdf";
};

std::optional<std::string> inferPackageNameFromRobotUri(const std::string& uri)
{
  constexpr std::string_view prefix = "package://";
  if (!uri.starts_with(prefix)) {
    return std::nullopt;
  }

  const std::string rest = uri.substr(prefix.size());
  const std::size_t slash = rest.find('/');
  if (slash == std::string::npos || slash == 0) {
    return std::nullopt;
  }
  return rest.substr(0, slash);
}

std::optional<std::string> inferPackageNameFromPackageUri(
    const std::string& uri)
{
  const std::size_t end = uri.find_last_not_of('/');
  if (end == std::string::npos) {
    return std::nullopt;
  }

  const std::size_t slash = uri.find_last_of('/', end);
  const std::size_t start = slash == std::string::npos ? 0 : slash + 1;
  if (start > end) {
    return std::nullopt;
  }

  const std::string name = uri.substr(start, end - start + 1);
  return name.empty() ? std::nullopt : std::optional<std::string>(name);
}

G1Options parseG1Options(int argc, char* argv[])
{
  G1Options options;
  bool packageNameExplicit = false;
  bool packageUriExplicit = false;
  bool robotUriExplicit = false;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if ((arg == "--g1-package-uri" || arg == "--package-uri") && i + 1 < argc) {
      options.packageUri = argv[++i];
      packageUriExplicit = true;
    } else if (
        (arg == "--g1-robot-uri" || arg == "--robot-uri") && i + 1 < argc) {
      options.robotUri = argv[++i];
      robotUriExplicit = true;
    } else if (
        (arg == "--g1-package-name" || arg == "--package-name")
        && i + 1 < argc) {
      options.packageName = argv[++i];
      packageNameExplicit = true;
    }
  }

  if (!packageNameExplicit) {
    if (robotUriExplicit) {
      if (auto packageName = inferPackageNameFromRobotUri(options.robotUri)) {
        options.packageName = *packageName;
      }
    } else if (packageUriExplicit) {
      if (auto packageName
          = inferPackageNameFromPackageUri(options.packageUri)) {
        options.packageName = *packageName;
      }
    } else if (
        auto packageName = inferPackageNameFromRobotUri(options.robotUri)) {
      options.packageName = *packageName;
    }
  }

  return options;
}

void disableSkeletonCollisionAndGravity(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
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
    body->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
        [](dart::dynamics::ShapeNode* shapeNode) {
          shapeNode->getCollisionAspect()->setCollidable(false);
        });
  }
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
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

    body->eachShapeNodeWith<dart::dynamics::VisualAspect>(
        [&](const dart::dynamics::ShapeNode* shapeNode) {
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

dart::common::ResourceRetrieverPtr createG1ResourceRetriever(
    const G1Options& options)
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
      options.packageName, options.packageUri);

  auto resolver = std::make_shared<dart::utils::CompositeResourceRetriever>();
  resolver->addSchemaRetriever("package", packageRetriever);
  resolver->addSchemaRetriever("file", local);
  resolver->addSchemaRetriever("dart", dartRetriever);
  resolver->addSchemaRetriever("http", http);
  resolver->addSchemaRetriever("https", http);
  resolver->addDefaultRetriever(local);

  return resolver;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;

  constexpr double thickness = 0.04;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, thickness)));
  shapeNode->setRelativeTranslation(
      Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.86, 0.88, 0.90, 1.0));
  return ground;
}

dart::dynamics::SkeletonPtr loadG1Skeleton(const G1Options& options)
{
  dart::io::ReadOptions readOptions;
  readOptions.resourceRetriever = createG1ResourceRetriever(options);
  const dart::common::Uri robotUri(options.robotUri);
  auto robot = dart::io::readSkeleton(robotUri, readOptions);
  if (robot == nullptr) {
    throw std::runtime_error(
        "Failed to load G1 robot model from " + options.robotUri);
  }

  robot->setName(kG1SkeletonName);
  if (auto* rootBody = robot->getRootBodyNode()) {
    if (auto* freeJoint = dynamic_cast<dart::dynamics::FreeJoint*>(
            rootBody->getParentJoint())) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      dart::dynamics::FreeJoint::setTransformOf(freeJoint, transform);
      if (const auto bounds = computeVisualWorldBounds(robot)) {
        constexpr double groundClearance = 0.015;
        transform.translation().z() = groundClearance - bounds->first.z();
        dart::dynamics::FreeJoint::setTransformOf(freeJoint, transform);
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

struct G1Scene
{
  dart::simulation::WorldPtr world;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
};

void addG1IkTargets(G1Scene& scene, const dart::dynamics::SkeletonPtr& robot)
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

  const auto footSupportGeometry = makeG1FootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = robot->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "G1 robot model is missing body node "
          + std::string(config.bodyNode));
    }

    auto* endEffector = bodyNode->createEndEffector(
        std::string(config.targetName) + "_effector");
    if (config.supportContact) {
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    }

    auto ik = endEffector->getIK(true);
    ik->setGradientMethod<
        dart::dynamics::InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);

    auto target = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    target->setShape(std::make_shared<dart::dynamics::SphereShape>(0.055));
    target->getVisualAspect(true)->setRGBA(config.color);
    scene.world->addSimpleFrame(target);
    ik->setTarget(target);

    dart::gui::InverseKinematicsHandle handle;
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = std::move(target);
    handle.ik = std::move(ik);
    scene.ikHandles.push_back(std::move(handle));
  }
}

G1Scene createG1Scene(const G1Options& options)
{
  G1Scene scene;
  scene.world = dart::simulation::World::create("dartsim_g1");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createGround());

  auto robot = loadG1Skeleton(options);
  std::cout << "Loaded G1 robot from '" << options.robotUri << "'.\n"
            << "Package root for '" << options.packageName << "' set to '"
            << options.packageUri << "'.\n";
  scene.world->addSkeleton(robot);
  addG1IkTargets(scene, robot);
  return scene;
}

dart::gui::Panel createG1Panel(const G1Options& options)
{
  dart::gui::Panel panel;
  panel.title = "G1 Puppet";
  panel.buildWithContext = [options](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("G1 whole-body IK puppet");
    builder.text("Press 1-4 or select a colored target.");
    builder.separator();
    builder.text("robot: " + options.robotUri);
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    const G1Options g1Options = parseG1Options(argc, argv);
    G1Scene scene = createG1Scene(g1Options);

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.panels.push_back(createG1Panel(g1Options));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "g1_puppet: " << e.what() << "\n";
    return 1;
  }
}
