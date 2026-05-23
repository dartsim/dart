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

#include "human_joint_limit_constraints.hpp"

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <iostream>
#include <stdexcept>
#include <string>

#include <cstddef>

namespace {

constexpr const char* kGroundSkeletonName = "human_joint_limits_ground";
constexpr const char* kHumanSkeletonName = "human_joint_limits_human";

struct HumanJointLimitsScene
{
  dart::simulation::WorldPtr world;
  std::size_t customConstraintCount = 0;
};

void makeStaticGround(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return;
  }

  skeleton->setMobile(false);
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->setGravityMode(false);
  }
}

void colorHuman(const dart::dynamics::SkeletonPtr& human)
{
  const std::size_t numBodies = human->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i) {
    auto* body = human->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    const double t = numBodies <= 1 ? 0.0
                                    : static_cast<double>(i)
                                          / static_cast<double>(numBodies - 1);
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
}

HumanJointLimitsScene createHumanJointLimitsScene()
{
  const dart::common::Uri worldUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("skel/kima/kima_human_edited.skel"));
  auto world = dart::io::readWorld(worldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load human_joint_limits world from " + worldUri.toString());
  }

  auto ground = world->getSkeleton("ground skeleton");
  if (ground == nullptr) {
    throw std::runtime_error("human_joint_limits world is missing ground");
  }
  ground->setName(kGroundSkeletonName);
  makeStaticGround(ground);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.58, 0.62, 0.60));
  }

  auto human = world->getSkeleton("human");
  if (human == nullptr) {
    throw std::runtime_error("human_joint_limits world is missing human");
  }
  human->setName(kHumanSkeletonName);
  human->setMobile(true);
  human->eachJoint(
      [](dart::dynamics::Joint* joint) { joint->setLimitEnforcement(true); });
  const std::size_t customConstraintCount
      = installHumanJointLimitConstraints(*world, human);

  colorHuman(human);
  return {world, customConstraintCount};
}

dart::gui::RunOptions makeHumanJointLimitsRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::Panel createHumanJointLimitsPanel(std::size_t customConstraintCount)
{
  dart::gui::Panel panel;
  panel.title = "Human Joint Limits";
  panel.buildWithContext = [customConstraintCount](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Kima human skeleton with DART joint-limit enforcement");
    builder.text(
        "Neural arm/leg custom constraints: "
        + std::to_string(customConstraintCount));
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
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

void printHumanJointLimitsInstructions()
{
  std::cout << "Human joint limits example\n";
  std::cout << "Neural arm/leg custom constraints are installed\n";
  std::cout << "space bar: simulation on/off\n";
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto scene = createHumanJointLimitsScene();
    printHumanJointLimitsInstructions();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.runDefaults = makeHumanJointLimitsRunDefaults();
    options.panels.push_back(
        createHumanJointLimitsPanel(scene.customConstraintCount));

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "human_joint_limits: " << e.what() << "\n";
    return 1;
  }
}
