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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/geometry.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kHuboSkeletonName = "visual_hubo_puppet_robot";
constexpr const char* kGroundSkeletonName = "visual_hubo_puppet_ground";
constexpr double kDegrees = 3.14159265358979323846 / 180.0;

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

void setRequiredDofPosition(
    const dart::dynamics::SkeletonPtr& hubo, const char* name, double position)
{
  auto* dof = hubo ? hubo->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Hubo puppet model is missing expected DOF " + std::string(name));
  }
  dof->setPosition(position);
}

void setRequiredDofLimits(
    const dart::dynamics::SkeletonPtr& hubo,
    const char* name,
    double lower,
    double upper)
{
  auto* dof = hubo ? hubo->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Hubo puppet model is missing expected DOF " + std::string(name));
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
    setRequiredDofPosition(hubo, name, degrees * kDegrees);
  }

  const std::array<const char*, 4> limitedDofs{{"LSY", "LWY", "RSY", "RWY"}};
  for (const char* name : limitedDofs) {
    setRequiredDofLimits(hubo, name, -90.0 * kDegrees, 90.0 * kDegrees);
  }
}

void removeHuboPuppetFingerBodyNodes(const dart::dynamics::SkeletonPtr& hubo)
{
  if (hubo == nullptr) {
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
  if (hubo == nullptr) {
    throw std::runtime_error(
        "Failed to load Hubo puppet model from " + huboUri.toString());
  }

  hubo->setName(kHuboSkeletonName);
  removeHuboPuppetFingerBodyNodes(hubo);
  setupHuboPuppetStartConfiguration(hubo);

  auto* rootBody = hubo->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation().x() = 0.0;
    transform.translation().y() = 0.0;
    dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(hubo)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
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

std::shared_ptr<dart::dynamics::LineSegmentShape> createIkTargetHandleShape(
    double radius)
{
  auto handle = std::make_shared<dart::dynamics::LineSegmentShape>(7.0f);
  const std::size_t center = handle->addVertex(Eigen::Vector3d::Zero());
  const auto addAxis = [&](const Eigen::Vector3d& axis) {
    handle->addVertex(axis, center);
    handle->addVertex(-axis, center);
  };
  addAxis(Eigen::Vector3d(radius, 0.0, 0.0));
  addAxis(Eigen::Vector3d(0.0, radius, 0.0));
  addAxis(Eigen::Vector3d(0.0, 0.0, 0.75 * radius));
  return handle;
}

void setUnconstrainedIkBounds(const dart::dynamics::InverseKinematicsPtr& ik)
{
  Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, 0.04)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.02));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.86, 0.88, 0.90, 1.0));
  return ground;
}

struct HuboPuppetScene
{
  dart::simulation::WorldPtr world;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
};

void addHuboPuppetIkTargets(
    HuboPuppetScene& scene, const dart::dynamics::SkeletonPtr& hubo)
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

  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  hand.translation() = Eigen::Vector3d(0.0, 0.0, -0.09);

  Eigen::Isometry3d foot = Eigen::Isometry3d::Identity();
  foot.translation() = Eigen::Vector3d(0.14, 0.0, -0.126);

  Eigen::Isometry3d peg = Eigen::Isometry3d::Identity();
  peg.translation() = Eigen::Vector3d(0.0, 0.0, 0.09);

  const std::array<Config, 6> configs{{
      {"Body_LWR",
       "hubo_puppet_left_hand",
       "hubo_puppet_ik_target_left_hand",
       "1 left hand",
       '1',
       hand,
       {0.18, 0.55, 1.0, 0.92},
       false},
      {"Body_RWR",
       "hubo_puppet_right_hand",
       "hubo_puppet_ik_target_right_hand",
       "2 right hand",
       '2',
       hand,
       {1.0, 0.40, 0.24, 0.92},
       false},
      {"Body_LAR",
       "hubo_puppet_left_foot",
       "hubo_puppet_ik_target_left_foot",
       "3 left foot",
       '3',
       foot,
       {0.26, 0.86, 0.34, 0.92},
       true},
      {"Body_RAR",
       "hubo_puppet_right_foot",
       "hubo_puppet_ik_target_right_foot",
       "4 right foot",
       '4',
       foot,
       {0.95, 0.72, 0.18, 0.92},
       true},
      {"Body_LWP",
       "hubo_puppet_left_peg",
       "hubo_puppet_ik_target_left_peg",
       "5 left peg",
       '5',
       peg,
       {0.70, 0.43, 0.96, 0.92},
       false},
      {"Body_RWP",
       "hubo_puppet_right_peg",
       "hubo_puppet_ik_target_right_peg",
       "6 right peg",
       '6',
       peg,
       {0.25, 0.88, 0.78, 0.92},
       false},
  }};

  const auto footSupportGeometry = makeHuboPuppetFootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = hubo->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "Hubo puppet model is missing body node "
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
    ik->setGradientMethod<
        dart::dynamics::InverseKinematics::JacobianTranspose>();
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
    } else {
      setUnconstrainedIkBounds(ik);
    }

    auto target = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    target->setShape(createIkTargetHandleShape(0.15));
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

HuboPuppetScene createHuboPuppetScene()
{
  HuboPuppetScene scene;
  scene.world = dart::simulation::World::create("dartsim_hubo_puppet");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createGround());

  auto hubo = loadHuboPuppetSkeleton();
  scene.world->addSkeleton(hubo);
  addHuboPuppetIkTargets(scene, hubo);
  return scene;
}

dart::gui::Panel createHuboPuppetPanel()
{
  dart::gui::Panel panel;
  panel.title = "Hubo Puppet";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Hubo whole-body IK puppet");
    builder.text("Press 1-6 or select a target handle.");
    builder.text("Ctrl-left drag moves the selected handle.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.separator();
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
    HuboPuppetScene scene = createHuboPuppetScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.panels.push_back(createHuboPuppetPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "hubo_puppet: " << e.what() << "\n";
    return 1;
  }
}
