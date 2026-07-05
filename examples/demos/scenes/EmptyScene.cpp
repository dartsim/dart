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

// Ported from examples/empty: a minimal world whose only content is a
// draggable InteractiveFrame gizmo target.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <memory>

namespace dart_demos {

//==============================================================================
DemoScene makeEmptyScene()
{
  DemoScene scene;
  scene.id = "empty";
  scene.title = "Empty Scaffold";
  scene.category = "Getting Started";
  scene.summary
      = "A minimal empty world with a draggable InteractiveFrame gizmo.";

  scene.factory = [] {
    DemoSceneSetup setup;
    setup.world = dart::simulation::World::create();
    setup.enableShadows = false;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.57, 3.14, 1.64),
        ::osg::Vec3d(0.00, 0.00, 0.00),
        ::osg::Vec3d(-0.24, -0.25, 0.94)};

    auto target = std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World());
    setup.world->addSimpleFrame(target);

    // Drag `target` through the InteractiveFrame-typed
    // Viewer::enableDragAndDrop() overload (InteractiveFrameDnD) so its
    // per-axis/plane gizmo tools are usable. InteractiveFrameDnD allocates 9
    // raw per-tool sub-DnDs; the accompanying gui-osg library fix gives it a
    // real destructor that deletes them, so it is now safe to tear down on a
    // scene switch. Registered in onActivate and released via
    // ctx.addTeardown(disableDragAndDrop), which runs before the world is
    // destroyed. (DemoSceneSetup::dragFrames / SimpleFrameDnD remains the right
    // tool for plain SimpleFrames wanting only an unconstrained free-drag.)
    setup.onActivate = [target](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();
      if (auto* dnd = viewer->enableDragAndDrop(target.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    };

    setup.renderPanel = [target] {
      const Eigen::Vector3d p = target->getWorldTransform().translation();
      ImGui::TextWrapped(
          "Drag the gizmo target in the 3D view to move it around.");
      ImGui::Text("Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
