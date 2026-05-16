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

dart::dynamics::SkeletonPtr createCube(
    std::size_t index,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  auto skeleton
      = dart::dynamics::Skeleton::create("rigid_cube_" + std::to_string(index));
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(0.6));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.65);

  constexpr double mass = 1.0;
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));

  return skeleton;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto jointAndBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(8.0, 8.0, 0.1));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.78, 0.78, 0.78));
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.65);

  return ground;
}

dart::simulation::WorldPtr createRigidCubesWorld()
{
  auto world = dart::simulation::World::create("rigid_cubes");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createCube(
      0, Eigen::Vector3d(-0.8, 0.0, 1.0), Eigen::Vector3d(0.9, 0.2, 0.1)));
  world->addSkeleton(createCube(
      1, Eigen::Vector3d(0.0, 0.0, 1.7), Eigen::Vector3d(0.1, 0.6, 0.9)));
  world->addSkeleton(createCube(
      2, Eigen::Vector3d(0.8, 0.0, 2.4), Eigen::Vector3d(0.2, 0.8, 0.3)));
  world->addSkeleton(createGround());
  return world;
}

void applyForce(dart::simulation::World* world, const Eigen::Vector3d& force)
{
  if (world == nullptr) {
    return;
  }

  const auto cube = world->getSkeleton("rigid_cube_1");
  if (cube == nullptr || cube->getNumBodyNodes() == 0) {
    return;
  }

  cube->getBodyNode(0)->addExtForce(force);
}

dart::gui::Panel createControlsPanel()
{
  double forceMagnitude = 500.0;

  dart::gui::Panel controls;
  controls.title = "Rigid Cube Forces";
  controls.buildWithContext = [forceMagnitude](
                                  dart::gui::PanelBuilder& panel,
                                  dart::gui::PanelContext& context) mutable {
    panel.text("Apply one-frame force to the center cube");
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
    panel.slider("Force", forceMagnitude, 0.0, 1000.0);
    if (panel.button("-X")) {
      applyForce(context.world, Eigen::Vector3d(-forceMagnitude, 0.0, 0.0));
    }
    panel.sameLine();
    if (panel.button("+X")) {
      applyForce(context.world, Eigen::Vector3d(forceMagnitude, 0.0, 0.0));
    }
    panel.sameLine();
    if (panel.button("+Z")) {
      applyForce(context.world, Eigen::Vector3d(0.0, 0.0, forceMagnitude));
    }
    panel.text("time: " + std::to_string(context.simulationTime));
  };
  return controls;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createRigidCubesWorld();
  options.panels.push_back(createControlsPanel());
  return dart::gui::runApplication(argc, argv, options);
}
