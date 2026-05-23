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
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/shared_library_ik_fast.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kWamPackageName = "herb_description";
constexpr const char* kWamPackagePath = "urdf/wam";
constexpr const char* kWamUrdfPath = "urdf/wam/wam.urdf";
constexpr const char* kWamSkeletonName = "visual_wam_ikfast_robot";
constexpr const char* kEndEffectorName = "ee";
constexpr const char* kTargetFrameName = "lh_target";
constexpr const char* kGroundSkeletonName = "visual_wam_ikfast_ground";

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

dart::dynamics::SkeletonPtr loadWamIkFastSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      kWamPackageName, dart::config::dataPath(kWamPackagePath));
  const auto wamUri
      = dart::common::Uri::createFromPath(dart::config::dataPath(kWamUrdfPath));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (wam == nullptr) {
    throw std::runtime_error(
        "Failed to load WAM IKFast robot from " + wamUri.toString());
  }

  wam->setName(kWamSkeletonName);
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
          "WAM IKFast robot is missing expected DOF " + std::string(name));
    }
    dof->setPosition(position);
  }

  disableSkeletonCollisionAndGravity(wam);
  return wam;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, 0.01)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.005));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.18, 0.32, 0.58, 1.0));
  return ground;
}

void setUnconstrainedIkBounds(const dart::dynamics::InverseKinematicsPtr& ik)
{
  const Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  const Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
}

std::string makeWamIkFastLibraryPath()
{
#ifdef DART_WAM_IKFAST_LIB_PATH
  return DART_WAM_IKFAST_LIB_PATH;
#else
  std::stringstream stream;
  stream << DART_SHARED_LIB_PREFIX << "wamIk";
  #if (DART_OS_LINUX || DART_OS_MACOS) && !defined(NDEBUG)
  stream << "d";
  #endif
  stream << "." << DART_SHARED_LIB_EXTENSION;
  return stream.str();
#endif
}

struct WamIkFastSetup
{
  dart::dynamics::EndEffector* effector = nullptr;
  dart::dynamics::InverseKinematicsPtr ik;
  dart::dynamics::SimpleFramePtr target;
};

WamIkFastSetup setupWamIkFastTarget(const dart::dynamics::SkeletonPtr& wam)
{
  auto* endEffectorBody = wam ? wam->getBodyNode("/wam7") : nullptr;
  if (endEffectorBody == nullptr) {
    throw std::runtime_error("WAM IKFast robot is missing /wam7 body node");
  }

  Eigen::Isometry3d handOffset = Eigen::Isometry3d::Identity();
  handOffset.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

  auto* effector = endEffectorBody->createEndEffector(kEndEffectorName);
  effector->setDefaultRelativeTransform(handOffset, true);

  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      kTargetFrameName,
      effector->getWorldTransform());

  auto ik = effector->getIK(true);
  ik->setTarget(target);
  ik->setHierarchyLevel(1);
  ik->setGradientMethod<dart::dynamics::SharedLibraryIkFast>(
      makeWamIkFastLibraryPath(),
      std::vector<std::size_t>{0, 1, 3, 4, 5, 6},
      std::vector<std::size_t>{2});
  setUnconstrainedIkBounds(ik);

  WamIkFastSetup setup;
  setup.effector = effector;
  setup.ik = std::move(ik);
  setup.target = std::move(target);
  return setup;
}

class WamIkFastTargetState
{
public:
  WamIkFastTargetState(
      dart::simulation::WorldPtr world,
      dart::dynamics::SkeletonPtr wam,
      WamIkFastSetup setup)
    : mWorld(std::move(world)),
      mWam(std::move(wam)),
      mEffector(setup.effector),
      mIk(std::move(setup.ik)),
      mTarget(std::move(setup.target))
  {
    if (mWorld == nullptr || mWam == nullptr || mEffector == nullptr
        || mIk == nullptr || mTarget == nullptr) {
      throw std::runtime_error("WAM IKFast target state is incomplete");
    }

    mRestConfig = mWam->getPositions();
    mDefaultBounds = mIk->getErrorMethod().getBounds();
    mDefaultTargetTransform = mTarget->getRelativeTransform();
  }

  void activate()
  {
    if (mActive) {
      return;
    }

    mIk->getErrorMethod().setBounds();
    mTarget->setTransform(mEffector->getWorldTransform());
    mWorld->addSimpleFrame(mTarget);
    mActive = true;
    std::cout << "Activated WAM IKFast target.\n";
    solve();
  }

  void deactivate()
  {
    if (!mActive) {
      return;
    }

    mIk->getErrorMethod().setBounds(mDefaultBounds);
    mTarget->setRelativeTransform(mDefaultTargetTransform);
    mWorld->removeSimpleFrame(mTarget);
    mActive = false;
    std::cout << "Deactivated WAM IKFast target.\n";
  }

  void toggle()
  {
    if (mActive) {
      deactivate();
    } else {
      activate();
    }
  }

  void printJointValues() const
  {
    for (std::size_t i = 0; i < mWam->getNumDofs(); ++i) {
      const auto* dof = mWam->getDof(i);
      if (dof != nullptr) {
        std::cout << dof->getName() << ": " << dof->getPosition() << "\n";
      }
    }
  }

  void resetPosture()
  {
    mWam->setPositions(mRestConfig);
    std::cout << "Reset WAM to relaxed posture.\n";
    solve();
  }

  void solve()
  {
    if (mActive && mIk != nullptr) {
      mIk->solveAndApply(true);
    }
  }

  bool active() const
  {
    return mActive;
  }

  const dart::dynamics::SimpleFramePtr& target() const
  {
    return mTarget;
  }

  const dart::dynamics::InverseKinematicsPtr& ik() const
  {
    return mIk;
  }

private:
  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mWam;
  dart::dynamics::EndEffector* mEffector = nullptr;
  dart::dynamics::InverseKinematicsPtr mIk;
  dart::dynamics::SimpleFramePtr mTarget;
  Eigen::VectorXd mRestConfig;
  std::pair<Eigen::Vector6d, Eigen::Vector6d> mDefaultBounds;
  Eigen::Isometry3d mDefaultTargetTransform = Eigen::Isometry3d::Identity();
  bool mActive = false;
};

struct WamIkFastScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr wam;
  std::shared_ptr<WamIkFastTargetState> targetState;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
};

WamIkFastScene createWamIkFastScene()
{
  WamIkFastScene scene;
  scene.world = dart::simulation::World::create("dartsim_wam_ikfast");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createGround());

  auto wam = loadWamIkFastSkeleton();
  auto setup = setupWamIkFastTarget(wam);
  scene.wam = wam;
  scene.world->addSkeleton(scene.wam);
  scene.targetState = std::make_shared<WamIkFastTargetState>(
      scene.world, scene.wam, std::move(setup));

  dart::gui::InverseKinematicsHandle handle;
  handle.label = "1 WAM end-effector";
  handle.hotkey = '1';
  handle.target = scene.targetState->target();
  handle.ik = scene.targetState->ik();
  scene.ikHandles.push_back(std::move(handle));

  dart::gui::Gizmo gizmo;
  gizmo.label = kTargetFrameName;
  gizmo.target = scene.targetState->target();
  gizmo.size = 0.24;
  gizmo.isVisible = [targetState = scene.targetState]() {
    return targetState->active();
  };
  scene.gizmos.push_back(std::move(gizmo));
  return scene;
}

std::vector<dart::gui::KeyboardAction> createWamIkFastKeyboardActions(
    const std::shared_ptr<WamIkFastTargetState>& targetState)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(3);

  dart::gui::KeyboardAction toggleTarget;
  toggleTarget.label = "Toggle WAM IKFast target";
  toggleTarget.shortcut = dart::gui::KeyboardShortcut::characterKey('1');
  toggleTarget.callback
      = [targetState](dart::gui::KeyboardActionContext& context) {
          targetState->toggle();
          if (context.lifecycle != nullptr) {
            context.lifecycle->paused = true;
          }
        };
  actions.push_back(std::move(toggleTarget));

  dart::gui::KeyboardAction printJoints;
  printJoints.label = "Print WAM joint values";
  printJoints.shortcut = dart::gui::KeyboardShortcut::characterKey('p');
  printJoints.callback = [targetState](dart::gui::KeyboardActionContext&) {
    targetState->printJointValues();
  };
  actions.push_back(std::move(printJoints));

  dart::gui::KeyboardAction resetPosture;
  resetPosture.label = "Reset WAM relaxed posture";
  resetPosture.shortcut = dart::gui::KeyboardShortcut::characterKey('t');
  resetPosture.callback
      = [targetState](dart::gui::KeyboardActionContext& context) {
          targetState->resetPosture();
          if (context.lifecycle != nullptr) {
            context.lifecycle->paused = true;
          }
        };
  actions.push_back(std::move(resetPosture));

  return actions;
}

std::vector<dart::gui::BodyNodeDragHandle> createWamBodyNodeDragHandles(
    const dart::dynamics::SkeletonPtr& wam)
{
  std::vector<dart::gui::BodyNodeDragHandle> handles;
  if (wam == nullptr) {
    return handles;
  }

  handles.reserve(wam->getNumBodyNodes());
  for (std::size_t i = 0; i < wam->getNumBodyNodes(); ++i) {
    auto* bodyNode = wam->getBodyNode(i);
    if (bodyNode == nullptr) {
      continue;
    }

    dart::gui::BodyNodeDragHandle handle;
    handle.label = bodyNode->getName();
    handle.bodyNode = bodyNode;
    handles.push_back(handle);
  }
  return handles;
}

dart::gui::RunOptions makeWamIkFastRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeWamIkFastCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.50);
  camera.yaw = 0.5118558424318241;
  camera.pitch = 0.22626228031830078;
  camera.distance = 6.285196894290584;
  return camera;
}

void printWamIkFastInstructions()
{
  std::cout
      << "WAM IKFast Example Controls:\n"
      << "Left-drag active target gizmo arrows/planes/rings\n"
      << "Alt + left-drag body: translate without changing orientation\n"
      << "Ctrl + left-drag body: rotate without changing translation\n"
      << "Shift + left-drag body: move using only its parent joint\n"
      << "Arrow keys and PageUp/PageDown: nudge the selected target gizmo\n"
      << "1: Toggle the interactive target of an EndEffector\n"
      << "P: Print the current joint values\n"
      << "T: Reset the robot to its relaxed posture\n"
      << "\n"
      << "Note that this is purely kinematic. Physical simulation is not "
         "allowed in this app.\n";
}

dart::gui::Panel createWamIkFastPanel()
{
  dart::gui::Panel panel;
  panel.title = "WAM IKFast";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("WAM IKFast kinematic target scene");
    builder.text("Press 1 to toggle/select the target.");
    builder.text("Press P to print joints; T resets posture.");
    builder.text("Left-drag active target gizmo handles.");
    builder.text("Alt/Ctrl/Shift-drag WAM body nodes.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("IK solves only while the target is active.");
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
    WamIkFastScene scene = createWamIkFastScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.gizmos = scene.gizmos;
    options.runDefaults = makeWamIkFastRunDefaults();
    options.camera = makeWamIkFastCamera();
    options.simulateWorld = false;
    options.preStep = [targetState = scene.targetState]() {
      targetState->solve();
    };
    options.keyboardActions = createWamIkFastKeyboardActions(scene.targetState);
    options.bodyNodeDragHandles = createWamBodyNodeDragHandles(scene.wam);
    options.panels.push_back(createWamIkFastPanel());

    printWamIkFastInstructions();
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "wam_ikfast: " << e.what() << "\n";
    return 1;
  }
}
