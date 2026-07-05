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

// Ported from examples/drag_and_drop: a gizmo-draggable InteractiveFrame
// carrying a red box as a child SimpleFrame, plus three axis-marker boxes at
// (8,0,0)/(0,8,0)/(0,0,8). No controller, no keyboard actions -- the entire
// demo is drag-and-drop interaction.
//
// Deviations from the original: the box is parented to the InteractiveFrame
// as in the original (dragging the gizmo carries the box with it), and the
// box itself is independently draggable via SimpleFrameDnD
// (DemoSceneSetup::dragFrames). The original's fixed 640x480 window and its
// own instruction-text addition are host chrome now; the "Ctrl + Left-click:
// Rotate the box" hint is folded into this scene's panel text instead.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <memory>

namespace dart_demos {

namespace {

using dart::dynamics::BoxShape;
using dart::dynamics::Frame;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SimpleFramePtr;

//==============================================================================
SimpleFramePtr createAxisMarker(
    const Eigen::Vector3d& position, const Eigen::Vector3d& color)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = position;

  auto marker = std::make_shared<SimpleFrame>(Frame::World(), "marker", tf);
  marker->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));
  marker->getVisualAspect(true)->setColor(color);
  return marker;
}

} // namespace

//==============================================================================
DemoScene makeDragAndDropScene()
{
  DemoScene scene;
  scene.id = "drag_and_drop";
  scene.title = "Drag and Drop";
  scene.category = "Visualization";
  scene.summary
      = "Drag and rotate a frame with a gizmo; click renderables to select.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(4, -4, 0);
    auto frame = std::make_shared<dart::gui::osg::InteractiveFrame>(
        Frame::World(), "interactive frame", tf, 2.0);
    world->addSimpleFrame(frame);

    tf.translation() = Eigen::Vector3d(-4, 4, 0);
    auto draggable
        = std::make_shared<SimpleFrame>(frame.get(), "draggable", tf);
    draggable->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1)));
    draggable->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
    world->addSimpleFrame(draggable);

    world->addSimpleFrame(createAxisMarker(
        Eigen::Vector3d(8.0, 0.0, 0.0), Eigen::Vector3d(0.9, 0.0, 0.0)));
    world->addSimpleFrame(createAxisMarker(
        Eigen::Vector3d(0.0, 8.0, 0.0), Eigen::Vector3d(0.0, 0.9, 0.0)));
    world->addSimpleFrame(createAxisMarker(
        Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.9)));

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(20.0, 17.0, 17.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.onActivate = [frame, draggable](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();
      if (auto* dnd = viewer->enableDragAndDrop(frame.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
      if (auto* dnd = viewer->enableDragAndDrop(draggable.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    };

    setup.renderPanel = [] {
      ImGui::TextWrapped(
          "Drag the gizmo's arrows/rings/planes to translate or rotate the "
          "frame; the red box rides along as its child. The red box is also "
          "independently draggable. Ctrl + Left-click rotates a dragged "
          "object without changing its translation.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
