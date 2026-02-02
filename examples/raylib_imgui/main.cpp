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

#include <dart/gui/interactive_frame.hpp>
#include <dart/gui/raylib/about_widget.hpp>
#include <dart/gui/raylib/inspector_widget.hpp>
#include <dart/gui/raylib/main_menu_widget.hpp>
#include <dart/gui/raylib/raylib_backend.hpp>
#include <dart/gui/raylib/sim_control_widget.hpp>
#include <dart/gui/scene_viewer.hpp>

#include <dart/all.hpp>

#include <string_view>

#include <cstdlib>

int main(int argc, char* argv[])
{
  int maxFrames = -1;
  for (int i = 1; i < argc; ++i) {
    if (std::string_view(argv[i]) == "--frames" && i + 1 < argc) {
      maxFrames = std::atoi(argv[i + 1]);
      ++i;
    }
  }

  // Create world
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  // Ground plane
  {
    auto ground = dart::dynamics::Skeleton::create("ground");
    auto [joint, body]
        = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
    auto shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(10, 10, 0.1));
    auto shapeNode = body->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect>(shape);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.7, 0.7, 0.7));
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation().z() = -0.05;
    joint->setTransformFromParentBodyNode(tf);
    world->addSkeleton(ground);
  }

  // Draggable SimpleFrame with a red box
  auto draggableFrame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "draggable");
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(-1.0, 1.0, 0.5);
    draggableFrame->setTransform(tf);
    auto shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.3, 0.3, 0.3));
    draggableFrame->setShape(shape);
    draggableFrame->getVisualAspect(true)->setColor(
        Eigen::Vector3d(0.9, 0.2, 0.2));
  }

  // InteractiveFrame with gizmo tools
  auto interactiveFrame = std::make_shared<dart::gui::InteractiveFrame>(
      dart::dynamics::Frame::World(), "interactive_frame");
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.5);
    interactiveFrame->setTransform(tf);
  }

  // Set up viewer
  auto config = dart::gui::ViewerConfig{};
  config.title = "DART Raylib - ImGui Demo";
  if (maxFrames >= 0) {
    config.headless = true;
    config.width = 320;
    config.height = 240;
  }

  auto backend = std::make_unique<dart::gui::RaylibBackend>();
  auto viewer = dart::gui::SceneViewer(std::move(backend), config);
  viewer.setWorld(world);

  // Register draggable objects
  world->addSimpleFrame(draggableFrame);
  viewer.enableDragAndDrop(draggableFrame.get());

  world->addSimpleFrame(interactiveFrame);
  viewer.enableDragAndDrop(interactiveFrame.get());

  // Create and add ImGui widgets
  auto aboutWidget = std::make_shared<dart::gui::RaylibAboutWidget>();
  auto menuWidget
      = std::make_shared<dart::gui::MainMenuWidget>(&viewer, aboutWidget);
  auto simControl = std::make_shared<dart::gui::SimControlWidget>(&viewer);
  auto inspector = std::make_shared<dart::gui::InspectorWidget>(&viewer);

  viewer.addWidget(simControl);
  viewer.addWidget(inspector);
  viewer.setAboutWidget(aboutWidget);
  viewer.setMenuWidget(menuWidget);

  // Position camera
  viewer.camera().position = Eigen::Vector3d(3.0, -3.0, 2.5);
  viewer.camera().target = Eigen::Vector3d(0.0, 0.0, 0.5);

  if (maxFrames >= 0) {
    for (int i = 0; i < maxFrames && viewer.frame(); ++i) {
    }
  } else {
    viewer.run();
  }

  return 0;
}
