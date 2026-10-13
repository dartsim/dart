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

// Ported from examples/soft_bodies: softBodies.skel with a state-history
// scrubber. Every step's full world state (skeleton configurations + body
// composite/aspect states, which capture soft-body point positions) is
// recorded; '[' / ']' step the playback cursor back/forward one frame, '{' /
// '}' by 10, 'r' jumps to frame 0, and '\\' jumps to the latest frame. Moving
// the cursor always pauses the simulation, matching the original.
//
// Deviations from the original: the history is capped at kMaxHistoryFrames
// (a ring buffer via std::deque, oldest frame evicted once full) rather than
// growing without bound -- the original's history holds a full skeleton
// Configuration and every BodyNode's composite Aspect state every step
// forever, which is a documented portRisk ("memory grows fast; a host should
// cap or ring-buffer it") given dart-demos scenes can run indefinitely.
// Pausing on scrub is done through the host's Viewer (captured via
// onActivate), rather than a WorldNode-owned mViewer pointer that is only
// valid after addWorldNode -- this also resolves the original's "moveTo()
// dereferences mViewer before it may be set" portRisk, since the viewer
// reference here is only ever populated once the scene is fully installed.
// softBodies.skel is authored Y-up (the original paired it with the default
// camera, leaving the ground as a vertical plane); it is reoriented to the
// host's Z-up convention (reorientWorldToZUp) and given a framing camera. The
// recorded history is captured after reorientation, so scrubbing is unaffected.
// moveBackward clamps to frame 0 when stepping back past the start, where the
// original's size_t underflow wrapped and clamped to the LATEST frame instead
// (a deliberate fix); as a knock-on, '[' at frame 0 now pauses via moveTo(0)
// where the original was a no-op.

#include "GuiBackend.hpp"
#include "Scenes.hpp"
#include "ZUp.hpp"
#include "demo_models/WorldHistory.hpp"

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

using dart::dynamics::SkeletonPtr;
using dart::dynamics::SoftMeshShape;
using dart::simulation::WorldPtr;

//==============================================================================
struct SoftBodyDisplayOptions
{
  float softMeshAlpha = 0.5f;
  bool showEmbeddedVisuals = true;
};

//==============================================================================
struct SoftBodySceneSpec
{
  const char* id;
  const char* title;
  const char* uri;
  const char* summary;
  CameraHome cameraHome;
};

//==============================================================================
/// Per-instance state captured by this scene's postStep/key-action lambdas.
struct SoftBodiesState
{
  std::unique_ptr<WorldHistory> history;
  GuiViewer* viewer = nullptr;
  demo_gui::WorldNode* worldNode = nullptr;
  SoftBodyDisplayOptions display;
};

//==============================================================================
void styleSoftBodyVisuals(
    const WorldPtr& world, const SoftBodyDisplayOptions& display)
{
  const double alpha = std::clamp<double>(display.softMeshAlpha, 0.0, 1.0);
  const std::array<Eigen::Vector4d, 5> softBodyColors = {
      Eigen::Vector4d(0.35, 0.48, 1.00, alpha),
      Eigen::Vector4d(0.95, 0.45, 0.30, alpha),
      Eigen::Vector4d(0.30, 0.72, 0.48, alpha),
      Eigen::Vector4d(0.95, 0.76, 0.28, alpha),
      Eigen::Vector4d(0.70, 0.45, 0.90, alpha),
  };

  std::size_t softBodyIndex = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const SkeletonPtr& skeleton = world->getSkeleton(i);
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      auto* softBodyNode = skeleton->getBodyNode(j)->asSoftBodyNode();
      if (!softBodyNode)
        continue;

      const Eigen::Vector4d& color
          = softBodyColors[softBodyIndex % softBodyColors.size()];
      ++softBodyIndex;

      for (std::size_t k = 0; k < softBodyNode->getNumShapeNodes(); ++k) {
        auto* shapeNode = softBodyNode->getShapeNode(k);
        auto* visualAspect = shapeNode->getVisualAspect(false);
        if (!visualAspect)
          continue;

        const auto shape = shapeNode->getShape();
        if (shape && shape->getType() == SoftMeshShape::getStaticType()) {
          visualAspect->setRGBA(color);
          visualAspect->show();
        } else if (display.showEmbeddedVisuals) {
          visualAspect->show();
        } else {
          visualAspect->hide();
        }
      }
    }
  }
}

//==============================================================================
void refreshSoftBodyDisplay(SoftBodiesState& state)
{
  if (!state.worldNode)
    return;

  const bool wasSimulating = state.worldNode->isSimulating();
  state.worldNode->simulate(false);
  state.worldNode->refresh();
  state.worldNode->simulate(wasSimulating);
}

//==============================================================================
void moveTo(SoftBodiesState& state, std::size_t index)
{
  if (state.viewer)
    state.viewer->simulate(false);
  state.history->moveTo(index);
}

//==============================================================================
void moveForward(SoftBodiesState& state, std::size_t delta)
{
  if (state.viewer)
    state.viewer->simulate(false);
  state.history->moveForward(delta);
}

//==============================================================================
void moveBackward(SoftBodiesState& state, std::size_t delta)
{
  if (state.viewer)
    state.viewer->simulate(false);
  state.history->moveBackward(delta);
}

//==============================================================================
DemoScene makeSoftBodyPlaybackScene(const SoftBodySceneSpec& spec)
{
  DemoScene scene;
  scene.id = spec.id;
  scene.title = spec.title;
  scene.category = "Soft Bodies";
  scene.summary = spec.summary;

  scene.factory = [spec] {
    auto world = dart::utils::SkelParser::readWorld(spec.uri);
    if (!world)
      throw std::runtime_error(std::string("failed to load ") + spec.uri);
    reorientWorldToZUp(world);

    auto state = std::make_shared<SoftBodiesState>();
    styleSoftBodyVisuals(world, state->display);
    // Prime frame 0 at build time, as the original's RecordingWorld
    // constructor does, so a fresh (still paused) scene already has a
    // recorded frame to scrub to.
    state->history = std::make_unique<WorldHistory>(world);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = spec.cameraHome;

    setup.onActivate = [state](DemoHostContext& ctx) {
      state->viewer = ctx.viewer();
      state->worldNode = ctx.worldNode();
    };

    setup.postStep = [state] {
      state->history->record();
    };

    setup.keyActions.push_back(KeyAction{'[', "Step back 1 frame", [state] {
                                           moveBackward(*state, 1);
                                         }});
    setup.keyActions.push_back(KeyAction{']', "Step forward 1 frame", [state] {
                                           moveForward(*state, 1);
                                         }});
    setup.keyActions.push_back(KeyAction{'{', "Step back 10 frames", [state] {
                                           moveBackward(*state, 10);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'}', "Step forward 10 frames", [state] {
                    moveForward(*state, 10);
                  }});
    setup.keyActions.push_back(KeyAction{'r', "Restart (frame 0)", [state] {
                                           moveTo(*state, 0);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'\\', "Jump to latest frame", [state] {
                    moveTo(*state, state->history->getSize() - 1);
                  }});

    setup.renderPanel = [world, state] {
      bool displayChanged = false;
      ImGui::TextUnformatted("Soft mesh alpha");
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
      displayChanged |= ImGui::SliderFloat(
          "##soft_mesh_alpha",
          &state->display.softMeshAlpha,
          0.0f,
          1.0f,
          "%.2f");
      displayChanged |= ImGui::Checkbox(
          "Embedded visuals", &state->display.showEmbeddedVisuals);
      if (displayChanged) {
        styleSoftBodyVisuals(world, state->display);
        refreshSoftBodyDisplay(*state);
      }

      ImGui::Separator();
      ImGui::Text(
          "Playback frame: %zu / %zu",
          state->history->getCurrentIndex(),
          state->history->getSize() - 1);
      ImGui::Text(
          "Frames recorded (lifetime): %zu",
          state->history->getFramesRecorded());
      ImGui::TextWrapped(
          "'[' / ']' step one recorded frame back/forward, '{' / '}' step "
          "10, 'r' restarts, '\\' jumps to the latest frame. Scrubbing "
          "pauses the simulation; history is capped at %zu frames.",
          WorldHistory::kMaxHistoryFrames);
    };

    return setup;
  };

  return scene;
}

} // namespace

//==============================================================================
DemoScene makeSoftBodiesScene()
{
  return makeSoftBodyPlaybackScene(SoftBodySceneSpec{
      "soft_bodies",
      "Soft Bodies",
      "dart://sample/skel/softBodies.skel",
      "Soft-body simulation with recorded-state playback.",
      CameraHome{
          GuiVector3(3.0, -3.0, 2.2),
          GuiVector3(0.0, 0.0, 0.4),
          GuiVector3(0.0, 0.0, 1.0)}});
}

//==============================================================================
DemoScene makeSoftCubesScene()
{
  return makeSoftBodyPlaybackScene(SoftBodySceneSpec{
      "soft_cubes",
      "Soft Cubes",
      "dart://sample/skel/soft_cubes.skel",
      "Soft cube stack with recorded-state playback.",
      CameraHome{
          GuiVector3(2.5, -2.8, 2.0),
          GuiVector3(0.0, 0.0, 0.35),
          GuiVector3(0.0, 0.0, 1.0)}});
}

//==============================================================================
DemoScene makeSoftOpenChainScene()
{
  return makeSoftBodyPlaybackScene(SoftBodySceneSpec{
      "soft_open_chain",
      "Soft Open Chain",
      "dart://sample/skel/soft_open_chain.skel",
      "Open chain of soft links with recorded-state playback.",
      CameraHome{
          GuiVector3(2.7, -2.7, 1.9),
          GuiVector3(0.0, 0.0, 0.25),
          GuiVector3(0.0, 0.0, 1.0)}});
}

} // namespace dart_demos
