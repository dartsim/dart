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
#include <dart/gui/debug.hpp>
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

constexpr const char* kHuboSkeletonName = "hubo_copy";
constexpr const char* kGroundSkeletonName = "ground";
constexpr const char* kHuboSupportOverlayName = "hubo_support_polygon_overlay";
constexpr double kDegrees = 3.14159265358979323846 / 180.0;
constexpr double kTeleopLinearStep = 0.01;
constexpr double kTeleopElevationStep = 0.2 * kTeleopLinearStep;
constexpr double kTeleopYawStep = 2.0 * 3.14159265358979323846 / 180.0;
constexpr double kSupportVisualElevation = 0.05;

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

std::shared_ptr<dart::dynamics::LineSegmentShape> createLineShape(
    const std::vector<dart::gui::DebugLineDescriptor>& lines)
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(3.0f);
  for (const auto& line : lines) {
    const auto start = shape->addVertex(line.from);
    shape->addVertex(line.to, start);
  }
  return shape;
}

std::vector<dart::gui::DebugLineDescriptor> makeHuboSupportPolygonLines(
    const dart::dynamics::SkeletonPtr& hubo)
{
  if (hubo == nullptr) {
    return {};
  }

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawSupportPolygons = true;
  options.supportPolygonElevation = kSupportVisualElevation;
  return dart::gui::makeSupportPolygonDebugLines(*hubo, options, "hubo");
}

dart::dynamics::SimpleFramePtr createHuboSupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& hubo)
{
  auto overlay = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kHuboSupportOverlayName);
  overlay->setShape(createLineShape(makeHuboSupportPolygonLines(hubo)));
  overlay->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.22, 0.86, 0.38, 0.86));
  return overlay;
}

void updateHuboSupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& hubo,
    const dart::dynamics::SimpleFramePtr& overlay)
{
  if (overlay == nullptr) {
    return;
  }

  overlay->setShape(createLineShape(makeHuboSupportPolygonLines(hubo)));
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
  constexpr double thickness = 0.01;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, thickness)));
  shapeNode->setRelativeTranslation(
      Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.0, 0.0, 0.2, 1.0));
  return ground;
}

struct HuboPuppetScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr hubo;
  dart::dynamics::SimpleFramePtr supportOverlay;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
  struct TargetState
  {
    dart::simulation::WorldPtr world;
    dart::dynamics::EndEffector* effector = nullptr;
    dart::dynamics::InverseKinematicsPtr ik;
    dart::dynamics::SimpleFramePtr target;
    std::pair<Eigen::Vector6d, Eigen::Vector6d> defaultBounds;
    Eigen::Isometry3d defaultTargetTransform = Eigen::Isometry3d::Identity();
    std::string label;
    char hotkey = '\0';
    bool active = false;

    void activate()
    {
      if (active || world == nullptr || effector == nullptr || target == nullptr
          || ik == nullptr) {
        return;
      }

      ik->getErrorMethod().setBounds();
      target->setTransform(effector->getWorldTransform());
      world->addSimpleFrame(target);
      active = true;
      std::cout << "Activated IK target '" << effector->getName() << "'.\n";
      solve();
    }

    void deactivate()
    {
      if (!active || world == nullptr || target == nullptr || ik == nullptr) {
        return;
      }

      ik->getErrorMethod().setBounds(defaultBounds);
      target->setTransform(defaultTargetTransform);
      world->removeSimpleFrame(target);
      active = false;
      std::cout << "Deactivated IK target '" << effector->getName() << "'.\n";
    }

    void toggle()
    {
      if (active) {
        deactivate();
      } else {
        activate();
      }
    }

    void solve()
    {
      if (active && ik != nullptr) {
        ik->solveAndApply(true);
      }
    }
  };

  std::vector<std::shared_ptr<TargetState>> targetStates;
  Eigen::VectorXd restConfiguration;
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
       "l_hand",
       "hubo_puppet_ik_target_left_hand",
       "1 left hand",
       '1',
       hand,
       false},
      {"Body_RWR",
       "r_hand",
       "hubo_puppet_ik_target_right_hand",
       "2 right hand",
       '2',
       hand,
       false},
      {"Body_LAR",
       "l_foot",
       "hubo_puppet_ik_target_left_foot",
       "3 left foot",
       '3',
       foot,
       true},
      {"Body_RAR",
       "r_foot",
       "hubo_puppet_ik_target_right_foot",
       "4 right foot",
       '4',
       foot,
       true},
      {"Body_LWP",
       "l_peg",
       "hubo_puppet_ik_target_left_peg",
       "5 left peg",
       '5',
       peg,
       false},
      {"Body_RWP",
       "r_peg",
       "hubo_puppet_ik_target_right_peg",
       "6 right peg",
       '6',
       peg,
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
    ik->setTarget(target);

    auto state = std::make_shared<HuboPuppetScene::TargetState>();
    state->world = scene.world;
    state->effector = endEffector;
    state->ik = ik;
    state->target = target;
    state->defaultBounds = ik->getErrorMethod().getBounds();
    state->defaultTargetTransform = target->getRelativeTransform();
    state->label = config.label;
    state->hotkey = config.hotkey;

    dart::gui::InverseKinematicsHandle handle;
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = target;
    handle.ik = ik;
    scene.ikHandles.push_back(std::move(handle));

    dart::gui::Gizmo gizmo;
    gizmo.label = config.targetName;
    gizmo.target = target;
    gizmo.size = 0.24;
    gizmo.isVisible = [state]() {
      return state->active;
    };
    scene.gizmos.push_back(std::move(gizmo));
    scene.targetStates.push_back(std::move(state));
  }
}

HuboPuppetScene createHuboPuppetScene()
{
  HuboPuppetScene scene;
  scene.world = dart::simulation::World::create("dartsim_hubo_puppet");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(createGround());

  auto hubo = loadHuboPuppetSkeleton();
  scene.hubo = hubo;
  scene.world->addSkeleton(hubo);
  addHuboPuppetIkTargets(scene, hubo);
  scene.restConfiguration = hubo->getPositions();
  scene.supportOverlay = createHuboSupportPolygonOverlay(scene.hubo);
  scene.world->addSimpleFrame(scene.supportOverlay);
  return scene;
}

void solveActiveHuboTargets(
    const std::vector<std::shared_ptr<HuboPuppetScene::TargetState>>&
        targetStates)
{
  for (const auto& state : targetStates) {
    if (state != nullptr) {
      state->solve();
    }
  }
}

bool applyRootTeleoperationStep(
    const dart::dynamics::SkeletonPtr& hubo, PuppetMotion motion)
{
  auto* rootBody = hubo ? hubo->getRootBodyNode() : nullptr;
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

std::vector<dart::gui::KeyboardAction> createHuboPuppetKeyboardActions(
    const dart::dynamics::SkeletonPtr& hubo,
    const std::vector<std::shared_ptr<HuboPuppetScene::TargetState>>&
        targetStates,
    const Eigen::VectorXd& restConfiguration)
{
  struct Config
  {
    char key;
    const char* label;
    PuppetMotion motion;
  };

  const std::array<Config, 8> configs{{
      {'w', "Move Hubo forward", PuppetMotion::Forward},
      {'s', "Move Hubo backward", PuppetMotion::Backward},
      {'a', "Move Hubo left", PuppetMotion::Left},
      {'d', "Move Hubo right", PuppetMotion::Right},
      {'f', "Raise Hubo root", PuppetMotion::Up},
      {'z', "Lower Hubo root", PuppetMotion::Down},
      {'q', "Yaw Hubo left", PuppetMotion::YawLeft},
      {'e', "Yaw Hubo right", PuppetMotion::YawRight},
  }};

  auto states = std::make_shared<
      std::vector<std::shared_ptr<HuboPuppetScene::TargetState>>>(targetStates);
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(configs.size() + targetStates.size() + 4);
  for (const Config& config : configs) {
    dart::gui::KeyboardAction action;
    action.label = config.label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(config.key);
    action.repeat = true;
    action.callback = [hubo, states, motion = config.motion](
                          dart::gui::KeyboardActionContext& context) {
      if (applyRootTeleoperationStep(hubo, motion)) {
        solveActiveHuboTargets(*states);
        if (context.lifecycle != nullptr) {
          context.lifecycle->paused = true;
        }
      }
    };
    actions.push_back(std::move(action));
  }

  for (const auto& state : targetStates) {
    if (state == nullptr || state->hotkey == '\0') {
      continue;
    }

    dart::gui::KeyboardAction action;
    action.label = "Toggle Hubo target " + state->label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(state->hotkey);
    action.callback = [state](dart::gui::KeyboardActionContext& context) {
      state->toggle();
      if (context.lifecycle != nullptr) {
        context.lifecycle->paused = true;
      }
    };
    actions.push_back(std::move(action));
  }

  const auto addSupportToggle
      = [&](char key, const char* effectorName, const char* label) {
          dart::gui::KeyboardAction action;
          action.label = label;
          action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
          action.callback =
              [hubo, effectorName](dart::gui::KeyboardActionContext& context) {
                auto* effector
                    = hubo ? hubo->getEndEffector(effectorName) : nullptr;
                auto* support = effector ? effector->getSupport() : nullptr;
                if (support != nullptr) {
                  support->setActive(!support->isActive());
                  if (context.lifecycle != nullptr) {
                    context.lifecycle->paused = true;
                  }
                }
              };
          actions.push_back(std::move(action));
        };
  addSupportToggle('x', "l_foot", "Toggle left Hubo foot support");
  addSupportToggle('c', "r_foot", "Toggle right Hubo foot support");

  dart::gui::KeyboardAction printDofs;
  printDofs.label = "Print Hubo DOFs";
  printDofs.shortcut = dart::gui::KeyboardShortcut::characterKey('p');
  printDofs.callback = [hubo](dart::gui::KeyboardActionContext&) {
    if (hubo == nullptr) {
      return;
    }
    for (std::size_t i = 0; i < hubo->getNumDofs(); ++i) {
      auto* dof = hubo->getDof(i);
      std::cout << dof->getName() << ": " << dof->getPosition() << "\n";
    }
  };
  actions.push_back(std::move(printDofs));

  dart::gui::KeyboardAction resetPosture;
  resetPosture.label = "Reset Hubo relaxed posture";
  resetPosture.shortcut = dart::gui::KeyboardShortcut::characterKey('t');
  resetPosture.callback = [hubo, restConfiguration, states](
                              dart::gui::KeyboardActionContext& context) {
    if (hubo == nullptr
        || static_cast<std::size_t>(restConfiguration.size())
               != hubo->getNumDofs()) {
      return;
    }

    for (std::size_t i = 0; i < hubo->getNumDofs(); ++i) {
      if (i < 2 || 4 < i) {
        hubo->getDof(i)->setPosition(restConfiguration[static_cast<int>(i)]);
      }
    }
    solveActiveHuboTargets(*states);
    if (context.lifecycle != nullptr) {
      context.lifecycle->paused = true;
    }
  };
  actions.push_back(std::move(resetPosture));
  return actions;
}

dart::gui::Panel createHuboPuppetPanel()
{
  dart::gui::Panel panel;
  panel.title = "Hubo Puppet";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Hubo whole-body IK puppet");
    builder.text("Press 1-6 to toggle/select targets.");
    builder.text("Left-drag active target gizmo handles.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.text("WASD moves the root; Q/E yaw; F/Z height.");
    builder.text("X/C toggles foot support; P prints DOFs; T resets posture.");
    builder.text("The support polygon overlay follows active foot support.");
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

dart::gui::RunOptions makeHuboPuppetRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeHuboPuppetCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.50);
  camera.up = Eigen::Vector3d(-0.20, -0.08, 0.98);
  camera.yaw = 0.5118558424318241;
  camera.pitch = 0.22626228031830078;
  camera.distance = 6.285196894290584;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    HuboPuppetScene scene = createHuboPuppetScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.gizmos = scene.gizmos;
    options.runDefaults = makeHuboPuppetRunDefaults();
    options.camera = makeHuboPuppetCamera();
    options.preStep = [targetStates = scene.targetStates,
                       hubo = scene.hubo,
                       supportOverlay = scene.supportOverlay]() {
      solveActiveHuboTargets(targetStates);
      updateHuboSupportPolygonOverlay(hubo, supportOverlay);
    };
    options.keyboardActions = createHuboPuppetKeyboardActions(
        scene.hubo, scene.targetStates, scene.restConfiguration);
    options.panels.push_back(createHuboPuppetPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "hubo_puppet: " << e.what() << "\n";
    return 1;
  }
}
