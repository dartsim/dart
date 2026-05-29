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

#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

struct DragAndDropScene
{
  dart::simulation::WorldPtr world;
  std::vector<dart::gui::Gizmo> gizmos;
};

DragAndDropScene createDragAndDropScene()
{
  DragAndDropScene scene;
  scene.world = dart::simulation::World::create("drag_and_drop");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(4.0, -4.0, 0.0);
  auto anchor = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), "interactive frame", transform);
  scene.world->addSimpleFrame(anchor);

  dart::gui::Gizmo gizmo;
  gizmo.label = "interactive frame";
  gizmo.target = anchor;
  gizmo.size = 2.0;
  scene.gizmos.push_back(std::move(gizmo));

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-4.0, 4.0, 0.0);
  auto draggable = anchor->spawnChildSimpleFrame("draggable", transform);
  draggable->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(1.0, 1.0, 1.0)));
  draggable->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
  scene.world->addSimpleFrame(draggable);

  const auto addMarker = [&](const std::string& name,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& color) {
    Eigen::Isometry3d markerTransform = Eigen::Isometry3d::Identity();
    markerTransform.translation() = position;
    auto marker = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), name, markerTransform);
    marker->setShape(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(0.2, 0.2, 0.2)));
    marker->getVisualAspect(true)->setColor(color);
    scene.world->addSimpleFrame(marker);
  };

  addMarker(
      "X", Eigen::Vector3d(8.0, 0.0, 0.0), Eigen::Vector3d(0.9, 0.0, 0.0));
  addMarker(
      "Y", Eigen::Vector3d(0.0, 8.0, 0.0), Eigen::Vector3d(0.0, 0.9, 0.0));
  addMarker(
      "Z", Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.9));

  return scene;
}

dart::gui::OrbitCamera makeDragAndDropCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.0);
  camera.yaw = 0.7044940642422177;
  camera.pitch = 0.5743269238648862;
  camera.distance = 31.272991542223778;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeDragAndDropScene()
{
  auto showInteractionTips = std::make_shared<bool>(true);

  dart::gui::Panel controls;
  controls.title = "Drag Controls";
  controls.buildWithContext
      = [showInteractionTips](
            dart::gui::PanelBuilder& panel, dart::gui::PanelContext& context) {
          panel.text("Drag targets with selection handles");
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
          panel.checkbox("Show tips", *showInteractionTips);
          if (*showInteractionTips) {
            panel.text("Left-drag gizmo arrows, planes, and rings.");
            panel.text("Click renderables to inspect selection.");
            panel.text("Arrow/PageUp/PageDown nudge the selected frame.");
          }
          panel.text("selected: " + context.selectedLabel);
        };

  auto scene = createDragAndDropScene();
  dart::gui::ApplicationOptions options;
  options.world = scene.world;
  options.gizmos = scene.gizmos;
  options.camera = makeDragAndDropCamera();
  options.panels.push_back(std::move(controls));

  return options;
}

} // namespace dart::examples::demos
