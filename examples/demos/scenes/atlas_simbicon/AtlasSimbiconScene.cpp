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

// Ported from examples/atlas_simbicon: an Atlas humanoid balanced by a
// Simbicon (Simple Biped Locomotion Control) state-machine controller
// (atlas_simbicon::Controller, State, StateMachine, TerminalCondition, ported
// wholesale into this subdirectory), with keyboard push perturbations and an
// ImGui control panel.
//
// Deviations from the original:
//  - Y-UP, NOT reoriented to Z-up. Every other Y-up .skel/.sdf world in this
//    catalog goes through ZUp.hpp, but that helper only rotates a skeleton's
//    root-joint transform and gravity -- it cannot retrofit "up" into
//    application code that reads world-Y directly. State::getCOMFrame()
//    (the Simbicon balance feedback frame), Controller::isAllowingControl()
//    (the pelvis-height emergency cutoff), and this file's own gravity
//    slider all hardcode Y as vertical; reorienting the world without
//    rewriting that math throughout State.cpp would silently break balance.
//    This scene therefore keeps its own Y-up camera (up (0, 0.1, 0)), the
//    same considered exception vehicle.cpp (this same batch) already makes
//    for an analogous reason.
//  - AtlasSimbiconWorldNode::customPreStep() becomes this scene's preStep
//    lambda (pelvis external force, controller->update(), force-duration
//    countdown) -- direct translation, no behavior change.
//  - The custom 4096x4096-texture shadow (node->showShadow()/hideShadow())
//    is applied in onActivate instead of host-default shadowing
//    (setup.enableShadows = false), and the panel's Shadow checkbox toggles
//    the same custom technique on/off, matching the original exactly
//    (rather than falling back to the host's plain default technique).
//  - Menu bar (Exit/About DART), the Play/Pause radio, and the "About DART"
//    help text are dropped as redundant with host chrome, matching every
//    other B2-B4 ImGui-panel port; the Headlights/Shadow/Depth-mode toggles
//    and the Gravity-magnitude slider are kept because they are genuine
//    per-scene tunables the host chrome does not otherwise expose.
//  - The widget's control-mode radio buttons are ported byte-for-byte
//    including their documented label/state-machine mismatch:
//    "Short-Stride Walking" actually selects the "running" state machine,
//    and "Normal-Stride Walking" selects "walking". The GUI also starts
//    with guiControlMode/controlMode both defaulted to 2 without invoking a
//    state switch, so the panel displays "Normal-Stride Walking" selected
//    while the Controller's own actual default (set by
//    Controller::_buildStateMachines()) is "walking in place" -- the GUI
//    only resyncs after the user changes the radio selection, exactly as
//    upstream.
//  - Controller::keyboard() (h/j/k harness toggles, n force-next-state, 1-4
//    direct state-machine select) is ported (see Controller.hpp) but, as in
//    the original, is never registered with any event handler -- it was
//    dead code upstream and remains so here; wiring it up would be new
//    functionality, not a port.
//  - Window renaming (realize() + getWindows() + setWindowName) is
//    host-specific chrome (the host owns one window across every scene) and
//    is not replicated, matching tinkertoy's (B2) identical case.

#include "../Scenes.hpp"
#include "Controller.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <osgShadow/ShadowMap>

#include <algorithm>
#include <memory>
#include <stdexcept>

#include <cmath>

namespace dart_demos {

namespace {

//==============================================================================
/// Per-instance state captured by this scene's preStep/renderPanel/key-action
/// lambdas.
struct AtlasSimbiconState
{
  std::shared_ptr<atlas_simbicon::Controller> controller;
  dart::dynamics::BodyNode* pelvis = nullptr;

  Eigen::Vector3d externalForce = Eigen::Vector3d::Zero();
  int forceDuration = 0;

  dart::gui::osg::WorldNode* worldNode = nullptr;
  dart::gui::osg::Viewer* viewer = nullptr;

  float gravityAcc = 9.81f;
  bool depthMode = false;

  // Bug-compatible with the original AtlasSimbiconWidget: both start at 2
  // ("Normal-Stride Walking" in the panel) without invoking a state switch,
  // so the Controller's own default ("walking in place") stays active until
  // the user changes the radio selection -- see the file comment.
  int guiControlMode = 2;
  int controlMode = 2;
};

//==============================================================================
void pushAtlas(
    AtlasSimbiconState& state, double fx, double fz, double force, int frames)
{
  // Merge semantics matching the original push helpers (pushForwardAtlas wrote
  // .x() only, pushLeftAtlas wrote .z() only): each push updates only its own
  // axis component, so two pushes on different axes within the shared force
  // window combine into a diagonal force instead of the later push zeroing the
  // earlier push's axis.
  if (fx != 0.0)
    state.externalForce.x() = fx * force;
  if (fz != 0.0)
    state.externalForce.z() = fz * force;
  state.forceDuration = frames;
}

//==============================================================================
void showShadow(
    dart::gui::osg::WorldNode* worldNode, dart::gui::osg::Viewer* viewer)
{
  auto shadow = dart::gui::osg::WorldNode::createDefaultShadowTechnique(viewer);
  if (auto* sm = dynamic_cast<::osgShadow::ShadowMap*>(shadow.get())) {
    const auto mapResolution = static_cast<short>(std::pow(2, 12));
    sm->setTextureSize(::osg::Vec2s(mapResolution, mapResolution));
  }
  worldNode->setShadowTechnique(shadow);
}

//==============================================================================
void hideShadow(dart::gui::osg::WorldNode* worldNode)
{
  worldNode->setShadowTechnique(nullptr);
}

} // namespace

//==============================================================================
DemoScene makeAtlasSimbiconScene()
{
  DemoScene scene;
  scene.id = "atlas_simbicon";
  scene.title = "Atlas SIMBICON";
  scene.category = "Control & IK";
  scene.summary
      = "Atlas humanoid balanced by a SIMBICON state-machine controller.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();

    dart::utils::DartLoader urdfLoader;
    auto ground
        = urdfLoader.parseSkeleton("dart://sample/sdf/atlas/ground.urdf");
    if (!ground)
      throw std::runtime_error(
          "failed to load dart://sample/sdf/atlas/ground.urdf");

    auto atlas = dart::utils::SdfParser::readSkeleton(
        "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
    if (!atlas)
      throw std::runtime_error(
          "failed to load dart://sample/sdf/atlas/atlas_v3_no_head.sdf");

    world->addSkeleton(ground);
    world->addSkeleton(atlas);

    // Y-up, deliberately not reoriented -- see the file comment.
    atlas->setPosition(0, -0.5 * dart::math::constantsd::pi());
    world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

    auto* pelvis = atlas->getBodyNode("pelvis");
    if (!pelvis)
      throw std::runtime_error(
          "atlas_v3_no_head.sdf: missing body node 'pelvis'");

    auto state = std::make_shared<AtlasSimbiconState>();
    state->controller = std::make_shared<atlas_simbicon::Controller>(
        atlas, world->getConstraintSolver());
    state->pelvis = pelvis;

    DemoSceneSetup setup;
    setup.world = world;
    // Applied via the custom-resolution showShadow() in onActivate instead
    // (see the file comment).
    setup.enableShadows = false;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.14, 3.28, 6.28) * 2.0,
        ::osg::Vec3d(1.00, 0.00, 0.00),
        ::osg::Vec3d(0.00, 0.1, 0.00)};

    setup.preStep = [state] {
      state->pelvis->addExtForce(state->externalForce);
      state->controller->update();

      if (state->forceDuration > 0)
        --state->forceDuration;
      else
        state->externalForce.setZero();
    };

    setup.onActivate = [state](DemoHostContext& ctx) {
      state->worldNode = ctx.worldNode();
      state->viewer = ctx.viewer();
      showShadow(state->worldNode, state->viewer);

      // The Depth-mode and Headlights checkboxes drive viewer-global state
      // that nothing else in the host resets, so capture the viewer's current
      // camera mode + headlights and restore exactly those on teardown -- a
      // scene switch must never leave the whole app stuck in depth mode or
      // unlit. Also resync the Depth-mode checkbox from the viewer's actual
      // state so a revisit never shows a desynced control.
      auto* viewer = state->viewer;
      const auto priorCameraMode = viewer->getCameraMode();
      const bool priorHeadlights = viewer->checkHeadlights();
      state->depthMode = priorCameraMode == dart::gui::osg::CameraMode::DEPTH;
      ctx.addTeardown([viewer, priorCameraMode, priorHeadlights] {
        viewer->setCameraMode(priorCameraMode);
        viewer->switchHeadlights(priorHeadlights);
      });
    };

    setup.keyActions.push_back(KeyAction{'a', "Push forward", [state] {
                                           pushAtlas(*state, 1, 0, 500, 100);
                                         }});
    setup.keyActions.push_back(KeyAction{'s', "Push backward", [state] {
                                           pushAtlas(*state, -1, 0, 500, 100);
                                         }});
    setup.keyActions.push_back(KeyAction{'d', "Push left", [state] {
                                           pushAtlas(*state, 0, 1, 500, 100);
                                         }});
    setup.keyActions.push_back(KeyAction{'f', "Push right", [state] {
                                           pushAtlas(*state, 0, -1, 500, 100);
                                         }});
    setup.keyActions.push_back(KeyAction{'r', "Reset Atlas", [state] {
                                           state->controller->resetRobot();
                                           state->externalForce.setZero();
                                         }});

    setup.renderPanel = [state, world] {
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Gravity Acc.",
              &state->gravityAcc,
              5.0f,
              20.0f,
              "-%.2f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(state->gravityAcc)) {
        state->gravityAcc = std::clamp(state->gravityAcc, 5.0f, 20.0f);
        world->setGravity(
            -static_cast<double>(state->gravityAcc) * Eigen::Vector3d::UnitY());
      }

      bool headlights = state->viewer && state->viewer->checkHeadlights();
      if (ImGui::Checkbox("Headlights On/Off", &headlights) && state->viewer)
        state->viewer->switchHeadlights(headlights);

      bool shadow = state->worldNode && state->worldNode->isShadowed();
      if (ImGui::Checkbox("Shadow On/Off", &shadow) && state->worldNode
          && state->viewer) {
        if (shadow)
          showShadow(state->worldNode, state->viewer);
        else
          hideShadow(state->worldNode);
      }

      if (ImGui::Checkbox("Depth mode", &state->depthMode) && state->viewer) {
        state->viewer->setCameraMode(
            state->depthMode ? dart::gui::osg::CameraMode::DEPTH
                             : dart::gui::osg::CameraMode::RGBA);
      }

      ImGui::Separator();

      if (ImGui::Button("Reset Atlas")) {
        state->controller->resetRobot();
        state->externalForce.setZero();
      }

      ImGui::Spacing();

      // Byte-for-byte label/state-machine mapping from the original,
      // mismatch included -- see the file comment.
      ImGui::RadioButton("No Control", &state->guiControlMode, 0);
      ImGui::RadioButton("Short-Stride Walking", &state->guiControlMode, 1);
      ImGui::RadioButton("Normal-Stride Walking", &state->guiControlMode, 2);

      if (state->guiControlMode != state->controlMode) {
        switch (state->guiControlMode) {
          case 0:
            state->controller->changeStateMachine("standing", world->getTime());
            break;
          case 1:
            state->controller->changeStateMachine("running", world->getTime());
            break;
          case 2:
            state->controller->changeStateMachine("walking", world->getTime());
            break;
          default:
            break;
        }
        state->controlMode = state->guiControlMode;
      }

      ImGui::Separator();
      ImGui::TextWrapped(
          "a/s push the pelvis forward/backward, d/f push left/right (500 N "
          "for 100 steps); 'r' resets Atlas to its initial configuration.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
