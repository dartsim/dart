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

#include "controller.hpp"
#include "state_machine.hpp"

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

constexpr const char* kAtlasUri
    = "dart://sample/sdf/atlas/atlas_v3_no_head.sdf";
constexpr double kDefaultGravity = 9.81;
constexpr double kDefaultPushForce = 500.0;
constexpr int kDefaultPushFrames = 100;
constexpr double kGroundHalfExtent = 12.5;
constexpr double kGroundThickness = 0.05;
constexpr double kGroundCenterZ = -0.95;

dart::dynamics::SkeletonPtr readRequiredSkeleton(const char* uri)
{
  auto skeleton = dart::io::readSkeleton(uri);
  if (skeleton == nullptr) {
    throw std::runtime_error(std::string("Failed to load ") + uri);
  }
  return skeleton;
}

void makeAtlasMeshVisualsReadable(const dart::dynamics::SkeletonPtr& atlas)
{
  const Eigen::Vector4d readableAtlasColor(0.18, 0.19, 0.21, 1.0);

  for (std::size_t i = 0; i < atlas->getNumBodyNodes(); ++i) {
    auto* body = atlas->getBodyNode(i);
    for (std::size_t j = 0; j < body->getNumShapeNodes(); ++j) {
      auto* shapeNode = body->getShapeNode(j);
      auto* visual = shapeNode->getVisualAspect();
      if (visual == nullptr) {
        continue;
      }

      const auto mesh = std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
          shapeNode->getShape());
      if (mesh == nullptr) {
        continue;
      }

      mesh->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
      const Eigen::Vector4d rgba = visual->getRGBA();
      Eigen::Vector4d readable = readableAtlasColor;
      readable.w() = rgba.w();
      visual->setRGBA(readable);
    }
  }
}

dart::dynamics::SkeletonPtr createZUpGround()
{
  auto ground = dart::dynamics::Skeleton::create("atlas_simbicon_ground");
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  body->setName("ground_link");

  auto shape = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
      2.0 * kGroundHalfExtent, 2.0 * kGroundHalfExtent, kGroundThickness));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, kGroundCenterZ));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.82, 0.84, 0.86, 1.0));
  ground->setMobile(false);
  return ground;
}

struct AtlasSimbiconScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
};

AtlasSimbiconScene createAtlasSimbiconScene()
{
  AtlasSimbiconScene scene;
  scene.world = dart::simulation::World::create("dartsim_atlas_simbicon");

  auto ground = createZUpGround();
  scene.atlas = readRequiredSkeleton(kAtlasUri);
  makeAtlasMeshVisualsReadable(scene.atlas);

  scene.world->setGravity(-kDefaultGravity * Eigen::Vector3d::UnitZ());
  scene.world->addSkeleton(ground);
  scene.world->addSkeleton(scene.atlas);
  return scene;
}

class AtlasSimbiconRuntime
{
public:
  explicit AtlasSimbiconRuntime(AtlasSimbiconScene scene)
    : mScene(std::move(scene)),
      mController(
          std::make_unique<Controller>(
              mScene.atlas, mScene.world->getConstraintSolver()))
  {
    // Keep the controller's historical default state machine.
  }

  dart::simulation::WorldPtr world() const
  {
    return mScene.world;
  }

  void preStep()
  {
    if (auto* pelvis = mController->getAtlasRobot()->getBodyNode("pelvis")) {
      pelvis->addExtForce(mExternalForce);
    }

    mController->update();

    if (mForceDuration > 0) {
      --mForceDuration;
    } else {
      mExternalForce.setZero();
    }
  }

  void reset()
  {
    mExternalForce.setZero();
    mForceDuration = 0;
    mController->resetRobot();
  }

  void pushForward()
  {
    push(Eigen::Vector3d::UnitX() * kDefaultPushForce);
  }

  void pushBackward()
  {
    push(-Eigen::Vector3d::UnitX() * kDefaultPushForce);
  }

  void pushLeft()
  {
    push(Eigen::Vector3d::UnitY() * kDefaultPushForce);
  }

  void pushRight()
  {
    push(-Eigen::Vector3d::UnitY() * kDefaultPushForce);
  }

  void nextState()
  {
    if (auto* stateMachine = mController->getCurrentState()) {
      stateMachine->transiteToNextState(mScene.world->getTime());
    }
  }

  void switchToStanding()
  {
    changeStateMachine("standing");
  }

  void switchToWalkingInPlace()
  {
    changeStateMachine("walking in place");
  }

  void switchToWalking()
  {
    changeStateMachine("walking");
  }

  void switchToRunning()
  {
    changeStateMachine("running");
  }

  void setGravity(double gravity)
  {
    mGravity = gravity;
    mScene.world->setGravity(-mGravity * Eigen::Vector3d::UnitZ());
  }

  double gravity() const
  {
    return mGravity;
  }

  void setPelvisHarnessed(bool enabled)
  {
    if (enabled == mPelvisHarnessed) {
      return;
    }

    enabled ? mController->harnessPelvis() : mController->unharnessPelvis();
    mPelvisHarnessed = enabled;
  }

  void setLeftFootHarnessed(bool enabled)
  {
    if (enabled == mLeftFootHarnessed) {
      return;
    }

    enabled ? mController->harnessLeftFoot() : mController->unharnessLeftFoot();
    mLeftFootHarnessed = enabled;
  }

  void setRightFootHarnessed(bool enabled)
  {
    if (enabled == mRightFootHarnessed) {
      return;
    }

    enabled ? mController->harnessRightFoot()
            : mController->unharnessRightFoot();
    mRightFootHarnessed = enabled;
  }

  bool pelvisHarnessed() const
  {
    return mPelvisHarnessed;
  }

  bool leftFootHarnessed() const
  {
    return mLeftFootHarnessed;
  }

  bool rightFootHarnessed() const
  {
    return mRightFootHarnessed;
  }

  std::string currentStateMachineName() const
  {
    if (auto* stateMachine = mController->getCurrentState()) {
      return stateMachine->getName();
    }
    return "none";
  }

private:
  void push(const Eigen::Vector3d& force)
  {
    mExternalForce = force;
    mForceDuration = kDefaultPushFrames;
  }

  void changeStateMachine(const std::string& name)
  {
    mController->changeStateMachine(name, mScene.world->getTime());
  }

  AtlasSimbiconScene mScene;
  std::unique_ptr<Controller> mController;
  Eigen::Vector3d mExternalForce = Eigen::Vector3d::Zero();
  int mForceDuration = 0;
  double mGravity = kDefaultGravity;
  bool mPelvisHarnessed = false;
  bool mLeftFootHarnessed = false;
  bool mRightFootHarnessed = false;
};

dart::gui::RunOptions makeAtlasSimbiconRunDefaults()
{
  dart::gui::RunOptions options;
  options.windowTitle = "Atlas Simbicon";
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeAtlasSimbiconCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, -0.25);
  camera.yaw = 0.55;
  camera.pitch = 0.32;
  camera.distance = 4.8;
  return camera;
}

dart::gui::KeyboardAction makeAtlasAction(
    std::string label, char key, std::function<void()> callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback
      = [callback = std::move(callback)](dart::gui::KeyboardActionContext&) {
          callback();
        };
  return action;
}

std::vector<dart::gui::KeyboardAction> createAtlasSimbiconKeyboardActions(
    const std::shared_ptr<AtlasSimbiconRuntime>& runtime)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(
      makeAtlasAction("Reset Atlas", 'r', [runtime]() { runtime->reset(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas forward", 'a', [runtime]() { runtime->pushForward(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas backward", 's', [runtime]() { runtime->pushBackward(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas left", 'd', [runtime]() { runtime->pushLeft(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas right", 'f', [runtime]() { runtime->pushRight(); }));
  actions.push_back(makeAtlasAction("Standing controller", '1', [runtime]() {
    runtime->switchToStanding();
  }));
  actions.push_back(
      makeAtlasAction("Walking-in-place controller", '2', [runtime]() {
        runtime->switchToWalkingInPlace();
      }));
  actions.push_back(makeAtlasAction(
      "Walking controller", '3', [runtime]() { runtime->switchToWalking(); }));
  actions.push_back(makeAtlasAction(
      "Running controller", '4', [runtime]() { runtime->switchToRunning(); }));
  return actions;
}

dart::gui::Panel createAtlasSimbiconPanel(
    const std::shared_ptr<AtlasSimbiconRuntime>& runtime)
{
  dart::gui::Panel panel;
  panel.title = "Atlas Control";
  panel.buildWithContext = [runtime](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Atlas robot controlled by Simbicon");
    builder.separator();
    builder.text("User Guide");
    builder.text("Press [r] to reset Atlas to the initial position.");
    builder.text("Press [a] to push forward Atlas torso.");
    builder.text("Press [s] to push backward Atlas torso.");
    builder.text("Press [d] to push left Atlas torso.");
    builder.text("Press [f] to push right Atlas torso.");
    builder.text("Press [1]/[2]/[3]/[4] to switch state machines.");
    builder.text("Select objects with left click; keyboard nudges move them.");
    builder.separator();

    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Play" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Exit")) {
        dart::gui::requestExit(*context.lifecycle);
      }
    }

    builder.separator();
    double gravity = runtime->gravity();
    if (builder.slider("Gravity Acc.", gravity, 5.0, 20.0)) {
      runtime->setGravity(gravity);
    }

    bool pelvis = runtime->pelvisHarnessed();
    if (builder.checkbox("Harness pelvis", pelvis)) {
      runtime->setPelvisHarnessed(pelvis);
    }
    bool leftFoot = runtime->leftFootHarnessed();
    if (builder.checkbox("Harness left foot", leftFoot)) {
      runtime->setLeftFootHarnessed(leftFoot);
    }
    bool rightFoot = runtime->rightFootHarnessed();
    if (builder.checkbox("Harness right foot", rightFoot)) {
      runtime->setRightFootHarnessed(rightFoot);
    }
    if (context.lighting.headlightsEnabled != nullptr) {
      bool headlights = *context.lighting.headlightsEnabled;
      if (builder.checkbox("Headlights On/Off", headlights)) {
        *context.lighting.headlightsEnabled = headlights;
      }
    }
    if (context.rendering.settings != nullptr) {
      bool shadows = context.rendering.settings->shadowsEnabled;
      if (builder.checkbox("Shadow On/Off", shadows)) {
        context.rendering.settings->shadowsEnabled = shadows;
      }

      bool depthMode = context.rendering.settings->outputMode
                       == dart::gui::RenderOutputMode::Depth;
      if (builder.checkbox("Depth mode", depthMode)) {
        context.rendering.settings->outputMode
            = depthMode ? dart::gui::RenderOutputMode::Depth
                        : dart::gui::RenderOutputMode::Color;
      }
    }

    builder.separator();
    if (builder.button("Reset Atlas")) {
      runtime->reset();
    }
    if (builder.button("No Control")) {
      runtime->switchToStanding();
    }
    builder.sameLine();
    if (builder.button("Walking In Place")) {
      runtime->switchToWalkingInPlace();
    }
    if (builder.button("Normal-Stride Walking")) {
      runtime->switchToWalking();
    }
    builder.sameLine();
    if (builder.button("Short-Stride Walking")) {
      runtime->switchToRunning();
    }
    if (builder.button("Next State")) {
      runtime->nextState();
    }

    builder.separator();
    builder.text("state machine: " + runtime->currentStateMachineName());
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto runtime
        = std::make_shared<AtlasSimbiconRuntime>(createAtlasSimbiconScene());

    dart::gui::ApplicationOptions options;
    options.world = runtime->world();
    options.runDefaults = makeAtlasSimbiconRunDefaults();
    options.camera = makeAtlasSimbiconCamera();
    options.preStep = [runtime]() {
      runtime->preStep();
    };
    options.panels.push_back(createAtlasSimbiconPanel(runtime));
    options.keyboardActions = createAtlasSimbiconKeyboardActions(runtime);

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "atlas_simbicon: " << e.what() << "\n";
    return 1;
  }
}
