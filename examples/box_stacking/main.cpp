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

#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

#include <cstddef>

namespace {

dart::dynamics::SkeletonPtr createBox(
    std::size_t index,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  auto skeleton
      = dart::dynamics::Skeleton::create("stack_box_" + std::to_string(index));
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 0.5));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  constexpr double mass = 1.0;
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));

  return skeleton;
}

dart::dynamics::SkeletonPtr createFloor()
{
  auto floor = dart::dynamics::Skeleton::create("floor");
  auto jointAndBody
      = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  constexpr double floorHeight = 0.02;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * floorHeight);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10.0, 10.0, floorHeight));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.82, 0.82, 0.82));

  return floor;
}

dart::simulation::WorldPtr createBoxStackingWorld()
{
  auto world = dart::simulation::World::create("box_stacking");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createFloor());

  constexpr int numBoxes = 5;
  for (int i = 0; i < numBoxes; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(numBoxes - 1);
    world->addSkeleton(createBox(
        static_cast<std::size_t>(i),
        Eigen::Vector3d(0.0, 0.0, 0.25 + i * 0.5),
        Eigen::Vector3d(0.2 + 0.5 * t, 0.35, 0.85 - 0.5 * t)));
  }

  return world;
}

dart::gui::Panel createControlsPanel()
{
  bool gravityEnabled = true;
  bool splitImpulseEnabled = false;

  dart::gui::Panel controls;
  controls.title = "Box Stacking";
  controls.buildWithContext = [gravityEnabled, splitImpulseEnabled](
                                  dart::gui::PanelBuilder& panel,
                                  dart::gui::PanelContext& context) mutable {
    panel.text("Stacked rigid-body contact demo");
    panel.separator();
    if (context.lifecycle != nullptr) {
      if (panel.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      panel.sameLine();
      if (panel.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }

    if (context.world != nullptr) {
      if (panel.checkbox("Gravity", gravityEnabled)) {
        context.world->setGravity(
            gravityEnabled ? Eigen::Vector3d(0.0, 0.0, -9.81)
                           : Eigen::Vector3d::Zero());
      }
      if (auto* solver = context.world->getConstraintSolver()) {
        if (panel.checkbox("Split impulse", splitImpulseEnabled)) {
          solver->setSplitImpulseEnabled(splitImpulseEnabled);
        }
      }
    }

    panel.text("time: " + std::to_string(context.simulationTime));
    panel.text("contacts: " + std::to_string(context.contactCount));
  };
  return controls;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createBoxStackingWorld();
  options.panels.push_back(createControlsPanel());
  return dart::gui::runApplication(argc, argv, options);
}
