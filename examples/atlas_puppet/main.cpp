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
#include <dart/gui/gizmo.hpp>
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

constexpr const char* kAtlasUri
    = "dart://sample/sdf/atlas/atlas_v3_no_head.urdf";
constexpr const char* kAtlasSkeletonName = "visual_atlas_robot";
constexpr const char* kGroundSkeletonName = "visual_atlas_puppet_ground";
constexpr const char* kIkTargetPrefix = "atlas_puppet_ik_target_";
constexpr double kTeleopLinearStep = 0.01;
constexpr double kTeleopElevationStep = 0.2 * kTeleopLinearStep;
constexpr double kTeleopYawStep = 2.0 * 3.14159265358979323846 / 180.0;

enum class PuppetMotion
{
  Forward,
  Backward,
  Left,
  Right,
  Up,
  Down,
  YawLeft,
  YawRight,
};

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
    const dart::dynamics::SkeletonPtr& skeleton,
    const char* name,
    double position)
{
  auto* dof = skeleton ? skeleton->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Atlas puppet model is missing expected DOF " + std::string(name));
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
  const auto atlasUri = dart::common::Uri::createFromString(kAtlasUri);
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (atlas == nullptr) {
    throw std::runtime_error(
        "Failed to load Atlas puppet model from " + atlasUri.toString());
  }

  atlas->setName(kAtlasSkeletonName);
  setupAtlasPuppetStartConfiguration(atlas);

  auto* rootBody = atlas->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation().x() = 0.0;
    transform.translation().y() = 0.0;
    dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(atlas)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    }
  }

  auto* torso = atlas->getRootBodyNode();
  if (torso != nullptr) {
    auto rootShape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.25, 0.25, 0.125));
    auto* shapeNode
        = torso->createShapeNodeWith<dart::dynamics::VisualAspect>(rootShape);
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

struct AtlasPuppetScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
};

void addAtlasPuppetIkTargets(
    AtlasPuppetScene& scene, const dart::dynamics::SkeletonPtr& atlas)
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

  const auto footSupportGeometry = makeAtlasPuppetFootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = atlas->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "Atlas puppet model is missing body node "
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
    handle.target = target;
    handle.ik = std::move(ik);

    dart::gui::Gizmo gizmo;
    gizmo.label = config.targetName;
    gizmo.target = target;
    gizmo.size = 0.24;

    scene.gizmos.push_back(std::move(gizmo));
    scene.ikHandles.push_back(std::move(handle));
  }
}

AtlasPuppetScene createAtlasPuppetScene()
{
  AtlasPuppetScene scene;
  scene.world = dart::simulation::World::create("dartsim_atlas_puppet");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createGround());

  auto atlas = loadAtlasPuppetSkeleton();
  scene.atlas = atlas;
  scene.world->addSkeleton(atlas);
  addAtlasPuppetIkTargets(scene, atlas);
  return scene;
}

void solveIkHandles(
    const std::vector<dart::gui::InverseKinematicsHandle>& handles)
{
  for (const auto& handle : handles) {
    if (handle.ik != nullptr) {
      handle.ik->solveAndApply(true);
    }
  }
}

bool applyRootTeleoperationStep(
    const dart::dynamics::SkeletonPtr& atlas, PuppetMotion motion)
{
  auto* rootBody = atlas ? atlas->getRootBodyNode() : nullptr;
  if (rootBody == nullptr
      || dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             == nullptr) {
    return false;
  }

  const Eigen::Isometry3d current = rootBody->getWorldTransform();
  Eigen::Isometry3d next = current;

  Eigen::Vector3d forward = current.linear().col(0);
  forward.z() = 0.0;
  if (forward.norm() > 1e-10) {
    forward.normalize();
  } else {
    forward.setZero();
  }

  Eigen::Vector3d left = current.linear().col(1);
  left.z() = 0.0;
  if (left.norm() > 1e-10) {
    left.normalize();
  } else {
    left.setZero();
  }

  switch (motion) {
    case PuppetMotion::Forward:
      next.translation() += kTeleopLinearStep * forward;
      break;
    case PuppetMotion::Backward:
      next.translation() -= kTeleopLinearStep * forward;
      break;
    case PuppetMotion::Left:
      next.translation() += kTeleopLinearStep * left;
      break;
    case PuppetMotion::Right:
      next.translation() -= kTeleopLinearStep * left;
      break;
    case PuppetMotion::Up:
      next.translation() += kTeleopElevationStep * Eigen::Vector3d::UnitZ();
      break;
    case PuppetMotion::Down:
      next.translation() -= kTeleopElevationStep * Eigen::Vector3d::UnitZ();
      break;
    case PuppetMotion::YawLeft:
      next.linear()
          = Eigen::AngleAxisd(kTeleopYawStep, Eigen::Vector3d::UnitZ())
                .toRotationMatrix()
            * current.linear();
      break;
    case PuppetMotion::YawRight:
      next.linear()
          = Eigen::AngleAxisd(-kTeleopYawStep, Eigen::Vector3d::UnitZ())
                .toRotationMatrix()
            * current.linear();
      break;
  }

  dart::dynamics::FreeJoint::setTransformOf(rootBody, next);
  return true;
}

std::vector<dart::gui::KeyboardAction> createAtlasPuppetKeyboardActions(
    const dart::dynamics::SkeletonPtr& atlas,
    const std::vector<dart::gui::InverseKinematicsHandle>& ikHandles)
{
  struct Config
  {
    char key;
    const char* label;
    PuppetMotion motion;
  };

  const std::array<Config, 8> configs{{
      {'w', "Move Atlas forward", PuppetMotion::Forward},
      {'s', "Move Atlas backward", PuppetMotion::Backward},
      {'a', "Move Atlas left", PuppetMotion::Left},
      {'d', "Move Atlas right", PuppetMotion::Right},
      {'f', "Raise Atlas root", PuppetMotion::Up},
      {'z', "Lower Atlas root", PuppetMotion::Down},
      {'q', "Yaw Atlas left", PuppetMotion::YawLeft},
      {'e', "Yaw Atlas right", PuppetMotion::YawRight},
  }};

  auto handles
      = std::make_shared<std::vector<dart::gui::InverseKinematicsHandle>>(
          ikHandles);
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(configs.size());
  for (const Config& config : configs) {
    dart::gui::KeyboardAction action;
    action.label = config.label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(config.key);
    action.repeat = true;
    action.callback = [atlas, handles, motion = config.motion](
                          dart::gui::KeyboardActionContext& context) {
      if (applyRootTeleoperationStep(atlas, motion)) {
        solveIkHandles(*handles);
        if (context.lifecycle != nullptr) {
          context.lifecycle->paused = true;
        }
      }
    };
    actions.push_back(std::move(action));
  }
  return actions;
}

dart::gui::Panel createAtlasPuppetPanel()
{
  dart::gui::Panel panel;
  panel.title = "Atlas Puppet";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Atlas whole-body IK puppet");
    builder.text("Press 1-4 or select a target handle.");
    builder.text("Left-drag gizmo arrows/rings or Ctrl-left drag a handle.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.text("WASD moves the root; Q/E yaw; F/Z height.");
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

dart::gui::RunOptions makeAtlasPuppetRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeAtlasPuppetCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 1.0);
  camera.yaw = 0.5118558424318241;
  camera.pitch = 0.22626228031830084;
  camera.distance = 6.285196894290584;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    AtlasPuppetScene scene = createAtlasPuppetScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.gizmos = scene.gizmos;
    options.runDefaults = makeAtlasPuppetRunDefaults();
    options.camera = makeAtlasPuppetCamera();
    options.preStep = [handles = scene.ikHandles]() {
      solveIkHandles(handles);
    };
    options.keyboardActions
        = createAtlasPuppetKeyboardActions(scene.atlas, scene.ikHandles);
    options.panels.push_back(createAtlasPuppetPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "atlas_puppet: " << e.what() << "\n";
    return 1;
  }
}
