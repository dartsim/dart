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

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>

#include <dart/math/random.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kMixedChainUri
    = "dart://sample/skel/test/test_articulated_bodies_10bodies.skel";
constexpr const char* kMixedChainName = "mixed_chain";
constexpr double kForceMagnitude = 500.0;
constexpr int kImpulseFrames = 100;

dart::simulation::WorldPtr createMixedChainWorld()
{
  auto world = dart::io::readWorld(kMixedChainUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load test_articulated_bodies_10bodies.skel");
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto chain = world->getSkeleton(1);
  if (chain == nullptr) {
    throw std::runtime_error("Mixed chain world did not contain a chain");
  }
  chain->setName(kMixedChainName);

  Eigen::VectorXd initialPose = Eigen::VectorXd::Zero(chain->getNumDofs());
  for (Eigen::Index i = 0; i < std::min<Eigen::Index>(3, initialPose.size());
       ++i) {
    initialPose[i] = dart::math::Random::uniform(-0.5, 0.5);
  }
  chain->setPositions(initialPose);

  for (std::size_t i = 0; i < chain->getNumBodyNodes(); ++i) {
    auto* body = chain->getBodyNode(i);
    const bool softLink
        = dynamic_cast<const dart::dynamics::SoftBodyNode*>(body) != nullptr;
    body->setColor(
        softLink ? Eigen::Vector3d(0.90, 0.42, 0.18)
                 : Eigen::Vector3d(0.30, 0.55, 0.85));
  }

  return world;
}

dart::gui::OrbitCamera makeMixedChainCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.4636476090008061;
  camera.pitch = 0.7297276562269663;
  camera.distance = 3.0;
  return camera;
}

dart::gui::RunOptions makeMixedChainRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

void printMixedChainInstructions()
{
  std::cout << "Mixed Chain Example Controls:\n";
  std::cout << "'q'/'w': Apply force in -X/+X direction\n";
  std::cout << "'e'/'r': Apply force in -Y/+Y direction\n";
  std::cout << "'t'/'y': Apply force in -Z/+Z direction\n";
  std::cout << "Space: Toggle simulation\n";
}

struct MixedChainControls
{
  explicit MixedChainControls(dart::simulation::WorldPtr inputWorld)
    : world(std::move(inputWorld))
  {
  }

  void startImpulse(const Eigen::Vector3d& inputForce)
  {
    force = inputForce;
    framesRemaining = kImpulseFrames;
  }

  void applyImpulse()
  {
    if (world == nullptr || framesRemaining <= 0) {
      return;
    }

    auto chain = world->getSkeleton(kMixedChainName);
    if (chain != nullptr && chain->getNumSoftBodyNodes() > 3) {
      if (auto* softBody = chain->getSoftBodyNode(3)) {
        softBody->addExtForce(force);
      }
    }

    --framesRemaining;
    if (framesRemaining == 0) {
      force.setZero();
    }
  }

  dart::simulation::WorldPtr world;
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  int framesRemaining = 0;
};

dart::gui::KeyboardAction makeImpulseAction(
    const std::shared_ptr<MixedChainControls>& controls,
    char key,
    std::string label,
    const Eigen::Vector3d& force)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback = [controls, force](dart::gui::KeyboardActionContext&) {
    controls->startImpulse(force);
  };
  return action;
}

std::vector<dart::gui::KeyboardAction> createMixedChainKeyboardActions(
    const std::shared_ptr<MixedChainControls>& controls)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(6);
  actions.push_back(makeImpulseAction(
      controls,
      'q',
      "Apply mixed-chain impulse -X",
      Eigen::Vector3d(-kForceMagnitude, 0.0, 0.0)));
  actions.push_back(makeImpulseAction(
      controls,
      'w',
      "Apply mixed-chain impulse +X",
      Eigen::Vector3d(kForceMagnitude, 0.0, 0.0)));
  actions.push_back(makeImpulseAction(
      controls,
      'e',
      "Apply mixed-chain impulse -Y",
      Eigen::Vector3d(0.0, -kForceMagnitude, 0.0)));
  actions.push_back(makeImpulseAction(
      controls,
      'r',
      "Apply mixed-chain impulse +Y",
      Eigen::Vector3d(0.0, kForceMagnitude, 0.0)));
  actions.push_back(makeImpulseAction(
      controls,
      't',
      "Apply mixed-chain impulse -Z",
      Eigen::Vector3d(0.0, 0.0, -kForceMagnitude)));
  actions.push_back(makeImpulseAction(
      controls,
      'y',
      "Apply mixed-chain impulse +Z",
      Eigen::Vector3d(0.0, 0.0, kForceMagnitude)));
  return actions;
}

dart::gui::Panel createControlsPanel(
    const std::shared_ptr<MixedChainControls>& controls)
{
  dart::gui::Panel panel;
  panel.title = "Mixed Chain";
  panel.buildWithContext = [controls](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Apply a short impulse to the soft link.");
    builder.text("'q'/'w': apply force in -X/+X direction");
    builder.text("'e'/'r': apply force in -Y/+Y direction");
    builder.text("'t'/'y': apply force in -Z/+Z direction");
    builder.text("Space: Toggle simulation");
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

    if (builder.button("-X")) {
      controls->startImpulse(Eigen::Vector3d(-kForceMagnitude, 0.0, 0.0));
    }
    builder.sameLine();
    if (builder.button("+X")) {
      controls->startImpulse(Eigen::Vector3d(kForceMagnitude, 0.0, 0.0));
    }
    if (builder.button("-Y")) {
      controls->startImpulse(Eigen::Vector3d(0.0, -kForceMagnitude, 0.0));
    }
    builder.sameLine();
    if (builder.button("+Y")) {
      controls->startImpulse(Eigen::Vector3d(0.0, kForceMagnitude, 0.0));
    }
    if (builder.button("-Z")) {
      controls->startImpulse(Eigen::Vector3d(0.0, 0.0, -kForceMagnitude));
    }
    builder.sameLine();
    if (builder.button("+Z")) {
      controls->startImpulse(Eigen::Vector3d(0.0, 0.0, kForceMagnitude));
    }

    builder.text(
        "impulse frames: " + std::to_string(controls->framesRemaining));
    builder.text("time: " + std::to_string(context.simulationTime));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto world = createMixedChainWorld();
    auto controls = std::make_shared<MixedChainControls>(world);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.runDefaults = makeMixedChainRunDefaults();
    options.camera = makeMixedChainCamera();
    options.preStep = [controls]() {
      controls->applyImpulse();
    };
    options.keyboardActions = createMixedChainKeyboardActions(controls);
    options.panels.push_back(createControlsPanel(controls));
    printMixedChainInstructions();
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "mixed_chain: " << e.what() << "\n";
    return 1;
  }
}
