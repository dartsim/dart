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

#include <dart/common/profile.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>

namespace {

constexpr const char* kHelloWorldInstructions
    = "Press space to start free falling the box.";

dart::dynamics::SkeletonPtr createFallingBox()
{
  auto skeleton = dart::dynamics::Skeleton::create("falling_box");
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  transform.linear() = (Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(-0.45, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()))
                           .toRotationMatrix();
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(0.3));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.1, 0.2, 0.9));

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
      Eigen::Vector3d(10.0, 10.0, 0.1));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  return ground;
}

dart::simulation::WorldPtr createHelloWorld()
{
  auto world = dart::simulation::World::create("hello_world");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createFallingBox());
  world->addSkeleton(createGround());
  return world;
}

dart::gui::Panel createHelloWorldPanel()
{
  dart::gui::Panel panel;
  panel.title = "Hello World";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text(kHelloWorldInstructions);
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

dart::gui::RunOptions makeHelloWorldRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::OrbitCamera makeHelloWorldCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.50);
  camera.yaw = 0.8848934155088675;
  camera.pitch = 0.27389034471171636;
  camera.distance = 4.214747916542578;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  DART_PROFILE_SCOPED_N("hello_world_main");
  std::cout << kHelloWorldInstructions << std::endl;

  dart::gui::ApplicationOptions options;
  options.world = createHelloWorld();
  options.panels.push_back(createHelloWorldPanel());
  options.runDefaults = makeHelloWorldRunDefaults();
  options.camera = makeHelloWorldCamera();
  const int result = dart::gui::runApplication(argc, argv, options);
  DART_PROFILE_TEXT_DUMP();
  return result;
}
