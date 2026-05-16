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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

constexpr const char* kHybridWorldUri = "dart://sample/skel/fullbody1.skel";
constexpr const char* kGroundSkeletonName = "ground skeleton";
constexpr const char* kBipedSkeletonName = "fullbody1";

dart::dynamics::Joint* getRequiredJoint(
    const dart::dynamics::SkeletonPtr& skeleton, const char* name)
{
  auto* joint = skeleton == nullptr ? nullptr : skeleton->getJoint(name);
  if (joint == nullptr) {
    throw std::runtime_error(
        "hybrid_dynamics world is missing joint: " + std::string(name));
  }
  return joint;
}

std::size_t getRequiredCommandIndex(
    const dart::dynamics::SkeletonPtr& skeleton, const char* jointName)
{
  return getRequiredJoint(skeleton, jointName)->getIndexInSkeleton(0);
}

dart::simulation::WorldPtr createHybridDynamicsWorld()
{
  auto world = dart::io::readWorld(kHybridWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load hybrid_dynamics world from "
        + std::string(kHybridWorldUri));
  }
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = world->getSkeleton("ground skeleton");
  if (ground == nullptr) {
    throw std::runtime_error("hybrid_dynamics world is missing ground");
  }

  auto biped = world->getSkeleton(kBipedSkeletonName);
  if (biped == nullptr) {
    throw std::runtime_error("hybrid_dynamics world is missing fullbody1");
  }

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
    rootJoint->setActuatorType(dart::dynamics::Joint::ActuatorType::PASSIVE);
  }
  for (std::size_t i = 1; i < biped->getNumJoints(); ++i) {
    if (auto* joint = biped->getJoint(i)) {
      joint->setActuatorType(dart::dynamics::Joint::ActuatorType::VELOCITY);
    }
  }
  return world;
}

class HybridDynamicsController
{
public:
  explicit HybridDynamicsController(dart::simulation::WorldPtr world)
    : mWorld(std::move(world))
  {
    if (mWorld == nullptr) {
      throw std::runtime_error("hybrid_dynamics controller is missing world");
    }
    mBiped = mWorld->getSkeleton(kBipedSkeletonName);
    if (mBiped == nullptr) {
      throw std::runtime_error("hybrid_dynamics controller is missing biped");
    }

    mScapulaLeft = getRequiredCommandIndex(mBiped, "j_scapula_left");
    mScapulaRight = getRequiredCommandIndex(mBiped, "j_scapula_right");
    mForearmLeft = getRequiredCommandIndex(mBiped, "j_forearm_left");
    mForearmRight = getRequiredCommandIndex(mBiped, "j_forearm_right");
    mShinLeft = getRequiredCommandIndex(mBiped, "j_shin_left");
    mShinRight = getRequiredCommandIndex(mBiped, "j_shin_right");
  }

  void preStep()
  {
    const double armCycle = std::sin(mWorld->getTime() * 4.0);
    const double legCycle = std::sin(mWorld->getTime() * 2.0);
    mBiped->setCommand(mScapulaLeft, armCycle);
    mBiped->setCommand(mScapulaRight, -armCycle);
    mBiped->setCommand(mForearmLeft, 0.8 * armCycle);
    mBiped->setCommand(mForearmRight, 0.8 * armCycle);
    mBiped->setCommand(mShinLeft, 0.1 * legCycle);
    mBiped->setCommand(mShinRight, 0.1 * legCycle);
  }

  void toggleHarness()
  {
    auto* pelvis = mBiped->getBodyNode("h_pelvis");
    auto* joint = pelvis == nullptr ? nullptr : pelvis->getParentJoint();
    if (joint == nullptr) {
      return;
    }

    mHarnessOn = !mHarnessOn;
    joint->setActuatorType(
        mHarnessOn ? dart::dynamics::Joint::ActuatorType::LOCKED
                   : dart::dynamics::Joint::ActuatorType::PASSIVE);
    std::cout
        << (mHarnessOn ? "The pelvis is locked." : "The pelvis is unlocked.")
        << std::endl;
  }

  bool harnessOn() const
  {
    return mHarnessOn;
  }

private:
  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mBiped;
  std::size_t mScapulaLeft = 0;
  std::size_t mScapulaRight = 0;
  std::size_t mForearmLeft = 0;
  std::size_t mForearmRight = 0;
  std::size_t mShinLeft = 0;
  std::size_t mShinRight = 0;
  bool mHarnessOn = false;
};

dart::gui::OrbitCamera makeHybridDynamicsCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.5404195002705842;
  camera.pitch = 0.4758822496604165;
  camera.distance = 6.557438524302;
  return camera;
}

dart::gui::RunOptions makeHybridDynamicsRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

void printHybridDynamicsInstructions()
{
  std::cout << "'h': toggle harness on/off\n"
            << "space bar: simulation on/off\n";
}

std::vector<dart::gui::KeyboardAction> createHybridDynamicsKeyboardActions(
    const std::shared_ptr<HybridDynamicsController>& controller)
{
  dart::gui::KeyboardAction toggleHarness;
  toggleHarness.label = "Toggle hybrid-dynamics harness";
  toggleHarness.shortcut = dart::gui::KeyboardShortcut::characterKey('h');
  toggleHarness.callback = [controller](dart::gui::KeyboardActionContext&) {
    controller->toggleHarness();
  };
  return {std::move(toggleHarness)};
}

dart::gui::Panel createHybridDynamicsPanel(
    const std::shared_ptr<HybridDynamicsController>& controller)
{
  dart::gui::Panel panel;
  panel.title = "Hybrid Dynamics";
  panel.buildWithContext = [controller](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Scripted velocity commands on the fullbody biped");
    builder.text("'h': toggle harness on/off");
    builder.text("space bar: simulation on/off");
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
    if (builder.button(
            controller->harnessOn() ? "Unlock pelvis" : "Lock pelvis")) {
      controller->toggleHarness();
    }
    builder.text(
        "harness: " + std::string(controller->harnessOn() ? "on" : "off"));
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto world = createHybridDynamicsWorld();
    auto controller = std::make_shared<HybridDynamicsController>(world);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.camera = makeHybridDynamicsCamera();
    options.runDefaults = makeHybridDynamicsRunDefaults();
    options.preStep = [controller]() {
      controller->preStep();
    };
    options.keyboardActions = createHybridDynamicsKeyboardActions(controller);
    options.panels.push_back(createHybridDynamicsPanel(controller));

    printHybridDynamicsInstructions();
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "hybrid_dynamics: " << e.what() << "\n";
    return 1;
  }
}
