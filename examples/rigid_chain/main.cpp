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

#include <dart/math/random.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <iostream>
#include <stdexcept>

namespace {

constexpr const char* kRigidChainUri = "dart://sample/skel/chain.skel";
constexpr const char* kRigidChainName = "rigid_chain";

dart::simulation::WorldPtr createRigidChainWorld()
{
  auto world = dart::io::readWorld(kRigidChainUri);
  if (world == nullptr) {
    throw std::runtime_error("Failed to load dart://sample/skel/chain.skel");
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000.0);

  auto chain = world->getSkeleton(0);
  if (chain == nullptr) {
    throw std::runtime_error("Rigid chain world did not contain a skeleton");
  }
  chain->setName(kRigidChainName);

  Eigen::VectorXd initialPose(chain->getNumDofs());
  for (Eigen::Index i = 0; i < initialPose.size(); ++i) {
    initialPose[i] = dart::math::Random::uniform(-0.5, 0.5);
  }
  chain->setPositions(initialPose);

  const std::size_t numBodies = chain->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i) {
    const double t = numBodies <= 1 ? 0.0
                                    : static_cast<double>(i)
                                          / static_cast<double>(numBodies - 1);
    chain->getBodyNode(i)->setColor(
        Eigen::Vector3d(0.20 + 0.60 * t, 0.58 - 0.28 * t, 0.90 - 0.45 * t));
  }

  return world;
}

void applyChainDamping(const dart::dynamics::SkeletonPtr& chain)
{
  if (chain == nullptr) {
    return;
  }

  Eigen::VectorXd damping = -0.01 * chain->getVelocities();
  for (Eigen::Index i = 0; i < damping.size(); ++i) {
    if (i % 3 == 1) {
      damping[i] *= 0.1;
    }
  }
  chain->setForces(damping);
}

dart::gui::Panel createStatusPanel()
{
  dart::gui::Panel panel;
  panel.title = "Rigid Chain";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Damped articulated chain from chain.skel");
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
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

dart::gui::RunOptions makeRigidChainRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::OrbitCamera makeRigidChainCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.4636476090008061;
  camera.pitch = 0.7297276562269663;
  camera.distance = 3.0;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto world = createRigidChainWorld();
    auto chain = world->getSkeleton(kRigidChainName);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.runDefaults = makeRigidChainRunDefaults();
    options.camera = makeRigidChainCamera();
    options.preStep = [chain]() {
      applyChainDamping(chain);
    };
    options.panels.push_back(createStatusPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "rigid_chain: " << e.what() << "\n";
    return 1;
  }
}
