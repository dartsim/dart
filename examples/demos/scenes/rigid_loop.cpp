/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/ball_joint_constraint.hpp>
#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <stdexcept>

namespace dart::examples::demos {

namespace {

constexpr const char* kRigidLoopUri = "dart://sample/skel/chain.skel";
constexpr const char* kRigidLoopName = "rigid_loop";

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

dart::simulation::WorldPtr createRigidLoopWorld()
{
  auto world = dart::io::readWorld(kRigidLoopUri);
  if (world == nullptr) {
    throw std::runtime_error("Failed to load dart://sample/skel/chain.skel");
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000.0);

  auto chain = world->getSkeleton(0);
  if (chain == nullptr) {
    throw std::runtime_error("Rigid loop world did not contain a skeleton");
  }
  chain->setName(kRigidLoopName);

  Eigen::VectorXd initialPose = Eigen::VectorXd::Zero(chain->getNumDofs());
  for (const int index : std::array{20, 23, 26, 29}) {
    if (index < initialPose.size()) {
      initialPose[index] = 0.4 * dart::math::pi;
    }
  }
  chain->setPositions(initialPose);

  auto* link6 = chain->getBodyNode("link 6");
  auto* link10 = chain->getBodyNode("link 10");
  if (link6 == nullptr || link10 == nullptr) {
    throw std::runtime_error("Rigid loop world is missing link 6 or link 10");
  }

  link6->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  link10->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));

  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d jointPosition = link6->getTransform() * offset;
  world->getConstraintSolver()->addConstraint(
      std::make_shared<dart::constraint::BallJointConstraint>(
          link6, link10, jointPosition));

  return world;
}

dart::gui::Panel createStatusPanel()
{
  dart::gui::Panel panel;
  panel.title = "Rigid Loop";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Red links are connected by a ball joint constraint.");
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

} // namespace

dart::gui::ApplicationOptions makeRigidLoopScene()
{
  auto world = createRigidLoopWorld();
  auto chain = world->getSkeleton(kRigidLoopName);

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.preStep = [chain]() {
    applyChainDamping(chain);
  };
  options.panels.push_back(createStatusPanel());
  return options;
}

} // namespace dart::examples::demos
