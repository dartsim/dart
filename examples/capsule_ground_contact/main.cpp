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
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kCapsuleRadius = 0.2;
constexpr double kCapsuleHeight = 0.6;

void printCapsuleGroundContactInstructions()
{
  std::cout << "Controls:\n"
            << "  [H] Reset capsule to horizontal pose\n"
            << "  [V] Reset capsule to vertical pose\n"
            << "  [Space] Clear velocities\n"
            << "This example uses ODE with persistent manifolds to keep the\n"
            << "capsule from sinking when lying on its side.\n";
}

Eigen::Isometry3d makeHorizontalPose()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(0.0, 0.0, kCapsuleRadius + 0.12));
  pose.rotate(Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitY()));
  return pose;
}

Eigen::Isometry3d makeVerticalPose()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(
      Eigen::Vector3d(0.0, 0.0, 0.5 * kCapsuleHeight + kCapsuleRadius + 0.02));
  return pose;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto jointAndBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* body = jointAndBody.second;

  const Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ();
  body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(planeNormal, 0.0));

  constexpr double visualThickness = 0.08;
  auto* groundVisual = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, visualThickness)));
  Eigen::Isometry3d groundOffset = Eigen::Isometry3d::Identity();
  groundOffset.translation() = planeNormal * (-0.5 * visualThickness);
  groundVisual->setRelativeTransform(groundOffset);
  groundVisual->getVisualAspect()->setColor(Eigen::Vector3d(0.70, 0.70, 0.70));
  groundVisual->getVisualAspect()->setShadowed(false);
  ground->setMobile(false);

  return ground;
}

dart::dynamics::SkeletonPtr createCapsule()
{
  auto capsule = dart::dynamics::Skeleton::create("capsule");
  auto jointAndBody
      = capsule->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;
  joint->setName("capsule_joint");
  body->setName("capsule_body");
  joint->setTransformFromParentBodyNode(makeHorizontalPose());

  const auto capsuleShape = std::make_shared<dart::dynamics::CapsuleShape>(
      kCapsuleRadius, kCapsuleHeight);
  auto* capsuleNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(capsuleShape);
  capsuleNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.4, 0.8));

  constexpr double mass = 1.0;
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), capsuleShape->computeInertia(mass)));

  return capsule;
}

dart::simulation::WorldPtr createCapsuleGroundContactWorld()
{
  auto world = dart::simulation::World::create("capsule_ground_contact");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
#if DART_HAVE_ODE
  world->setCollisionDetector(dart::simulation::CollisionDetectorType::Ode);
#endif
  world->addSkeleton(createGround());
  world->addSkeleton(createCapsule());
  return world;
}

dart::dynamics::FreeJoint* getCapsuleJoint(dart::simulation::World* world)
{
  if (world == nullptr) {
    return nullptr;
  }

  const auto capsule = world->getSkeleton("capsule");
  if (capsule == nullptr || capsule->getNumJoints() == 0) {
    return nullptr;
  }

  return dynamic_cast<dart::dynamics::FreeJoint*>(capsule->getJoint(0));
}

void clearCapsuleVelocities(dart::simulation::World* world)
{
  auto* joint = getCapsuleJoint(world);
  if (joint == nullptr) {
    return;
  }

  joint->setVelocities(Eigen::Matrix<double, 6, 1>::Zero());
}

void resetCapsule(dart::simulation::World* world, const Eigen::Isometry3d& pose)
{
  auto* joint = getCapsuleJoint(world);
  if (joint == nullptr || world == nullptr) {
    return;
  }

  world->reset();
  joint->setRelativeTransform(pose);
  clearCapsuleVelocities(world);
}

dart::gui::Panel createControlsPanel()
{
  dart::gui::Panel controls;
  controls.title = "Capsule Contact";
  controls.buildWithContext
      = [](dart::gui::PanelBuilder& panel, dart::gui::PanelContext& context) {
          panel.text("Capsule and ground contact");
          panel.text("ODE persistent manifolds keep the capsule stable");
          panel.text("H: horizontal, V: vertical, Space: clear velocities");
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
          if (panel.button("Horizontal")) {
            resetCapsule(context.world, makeHorizontalPose());
          }
          panel.sameLine();
          if (panel.button("Vertical")) {
            resetCapsule(context.world, makeVerticalPose());
          }
          panel.text("time: " + std::to_string(context.simulationTime));
          panel.text("contacts: " + std::to_string(context.contactCount));
        };
  return controls;
}

dart::gui::KeyboardAction makeCapsuleAction(
    std::string label,
    dart::gui::KeyboardShortcut shortcut,
    std::function<void(dart::gui::KeyboardActionContext&)> callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = shortcut;
  action.callback = std::move(callback);
  return action;
}

std::vector<dart::gui::KeyboardAction> createCapsuleKeyboardActions(
    const dart::simulation::WorldPtr& world)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(makeCapsuleAction(
      "Reset capsule to horizontal pose",
      dart::gui::KeyboardShortcut::characterKey('h'),
      [world](dart::gui::KeyboardActionContext&) {
        resetCapsule(world.get(), makeHorizontalPose());
      }));
  actions.push_back(makeCapsuleAction(
      "Reset capsule to vertical pose",
      dart::gui::KeyboardShortcut::characterKey('v'),
      [world](dart::gui::KeyboardActionContext&) {
        resetCapsule(world.get(), makeVerticalPose());
      }));
  actions.push_back(makeCapsuleAction(
      "Clear capsule velocities",
      dart::gui::KeyboardShortcut::characterKey(' '),
      [world](dart::gui::KeyboardActionContext&) {
        clearCapsuleVelocities(world.get());
      }));
  return actions;
}

dart::gui::RunOptions makeCapsuleRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1024;
  options.height = 768;
  return options;
}

dart::gui::OrbitCamera makeCapsuleCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.2);
  camera.yaw = 0.7853981633974483;
  camera.pitch = 0.3523514203703505;
  camera.distance = 3.7669616403674726;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  printCapsuleGroundContactInstructions();
  auto world = createCapsuleGroundContactWorld();

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.runDefaults = makeCapsuleRunDefaults();
  options.camera = makeCapsuleCamera();
  options.panels.push_back(createControlsPanel());
  options.keyboardActions = createCapsuleKeyboardActions(world);
  return dart::gui::runApplication(argc, argv, options);
}
