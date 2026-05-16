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

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>

namespace {

dart::simulation::WorldPtr createDragAndDropWorld()
{
  auto world = dart::simulation::World::create("drag_and_drop");
  world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(4.0, -4.0, 0.0);
  auto anchor = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), "interactive frame", transform);
  anchor->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.45, 0.45, 0.45)));
  anchor->getVisualAspect(true)->setColor(Eigen::Vector3d(0.95, 0.7, 0.15));
  world->addSimpleFrame(anchor);

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-4.0, 4.0, 0.0);
  auto draggable = anchor->spawnChildSimpleFrame("draggable", transform);
  draggable->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(1.0, 1.0, 1.0)));
  draggable->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
  world->addSimpleFrame(draggable);

  const auto addMarker = [&](const std::string& name,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& color) {
    Eigen::Isometry3d markerTransform = Eigen::Isometry3d::Identity();
    markerTransform.translation() = position;
    auto marker = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), name, markerTransform);
    marker->setShape(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(0.25, 0.25, 0.25)));
    marker->getVisualAspect(true)->setColor(color);
    world->addSimpleFrame(marker);
  };

  addMarker(
      "X", Eigen::Vector3d(8.0, 0.0, 0.0), Eigen::Vector3d(0.9, 0.0, 0.0));
  addMarker(
      "Y", Eigen::Vector3d(0.0, 8.0, 0.0), Eigen::Vector3d(0.0, 0.9, 0.0));
  addMarker(
      "Z", Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.9));

  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  bool showInteractionTips = true;

  dart::gui::Panel controls;
  controls.title = "Drag Controls";
  controls.buildWithContext
      = [&](dart::gui::PanelBuilder& panel, dart::gui::PanelContext& context) {
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
          panel.checkbox("Show tips", showInteractionTips);
          if (showInteractionTips) {
            panel.text("Ctrl-left drag moves the selected object.");
          }
          panel.text("selected: " + context.selectedLabel);
        };

  dart::gui::ApplicationOptions options;
  options.world = createDragAndDropWorld();
  options.panels.push_back(std::move(controls));

  return dart::gui::runApplication(argc, argv, options);
}
