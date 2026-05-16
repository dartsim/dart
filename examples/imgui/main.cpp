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
#include <utility>

namespace {

dart::dynamics::SkeletonPtr createDemoBox()
{
  auto skeleton = dart::dynamics::Skeleton::create("panel_demo_box");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.75);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.35, 0.35, 0.35));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.18, 0.44, 0.86));

  constexpr double mass = 1.0;
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));

  return skeleton;
}

dart::dynamics::SkeletonPtr createDemoGround()
{
  auto ground = dart::dynamics::Skeleton::create("panel_demo_ground");
  auto [joint, body]
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(4.0, 4.0, 0.1));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.78, 0.78, 0.72));

  return ground;
}

dart::simulation::WorldPtr createImguiDemoWorld()
{
  auto world = dart::simulation::World::create("panel_demo");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createDemoBox());
  world->addSkeleton(createDemoGround());
  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  bool showDiagnostics = true;
  double responseGain = 0.5;
  int buttonClicks = 0;

  dart::gui::ApplicationOptions options;
  options.world = createImguiDemoWorld();
  dart::gui::Panel controls;
  controls.title = "Example Controls";
  controls.buildWithContext
      = [&](dart::gui::PanelBuilder& panel, dart::gui::PanelContext& context) {
          panel.text("Custom example controls");
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
          if (panel.button("Trigger action")) {
            ++buttonClicks;
          }
          panel.sameLine();
          panel.text("clicks: " + std::to_string(buttonClicks));
          panel.checkbox("Diagnostics", showDiagnostics);
          panel.slider("Response gain", responseGain, 0.0, 1.0);
          if (showDiagnostics) {
            panel.text("gain: " + std::to_string(responseGain));
          }
        };
  options.panels.push_back(std::move(controls));

  return dart::gui::runApplication(argc, argv, options);
}
