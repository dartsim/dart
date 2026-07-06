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
// Pausing on scrub is done through the host's ImGuiViewer (captured via
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

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <array>
#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::SoftMeshShape;
using dart::simulation::WorldPtr;

constexpr std::size_t kMaxHistoryFrames = 4000;

//==============================================================================
struct SoftBodyDisplayOptions
{
  float softMeshAlpha = 0.5f;
  bool showEmbeddedVisuals = true;
};

//==============================================================================
struct HistoryState
{
  Skeleton::Configuration config;
  std::vector<dart::common::Composite::State> bodyStates;
};

//==============================================================================
using TimeSlice = std::vector<HistoryState>;

//==============================================================================
/// Per-instance state captured by this scene's postStep/key-action lambdas.
struct SoftBodiesState
{
  std::deque<TimeSlice> history;
  std::size_t currentIndex = 0;
  std::size_t framesRecorded = 0;
  dart::gui::osg::ImGuiViewer* viewer = nullptr;
  dart::gui::osg::WorldNode* worldNode = nullptr;
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
TimeSlice grabTimeSlice(const WorldPtr& world)
{
  TimeSlice slice;
  slice.reserve(world->getNumSkeletons());

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const SkeletonPtr& skeleton = world->getSkeleton(i);
    HistoryState state;
    state.config = skeleton->getConfiguration();
    state.bodyStates.reserve(skeleton->getNumBodyNodes());
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j)
      state.bodyStates.push_back(skeleton->getBodyNode(j)->getCompositeState());
    slice.push_back(std::move(state));
  }

  return slice;
}

//==============================================================================
void recordTimeSlice(const WorldPtr& world, SoftBodiesState& state)
{
  // Rewinding then resuming truncates the recorded "future" before appending
  // the new slice, matching the original's branching-timeline semantics.
  if (state.currentIndex + 1 < state.history.size())
    state.history.resize(state.currentIndex + 1);

  state.history.push_back(grabTimeSlice(world));
  ++state.framesRecorded;
  ++state.currentIndex;

  if (state.history.size() > kMaxHistoryFrames) {
    state.history.pop_front();
    --state.currentIndex;
  }
}

//==============================================================================
void moveTo(const WorldPtr& world, SoftBodiesState& state, std::size_t index)
{
  if (state.viewer)
    state.viewer->simulate(false);

  if (state.history.empty())
    return;

  if (index >= state.history.size())
    index = state.history.size() - 1;

  const TimeSlice& slice = state.history[index];
  for (std::size_t i = 0; i < slice.size() && i < world->getNumSkeletons();
       ++i) {
    const HistoryState& saved = slice[i];
    const SkeletonPtr& skeleton = world->getSkeleton(i);
    skeleton->setConfiguration(saved.config);
    for (std::size_t j = 0;
         j < skeleton->getNumBodyNodes() && j < saved.bodyStates.size();
         ++j)
      skeleton->getBodyNode(j)->setCompositeState(saved.bodyStates[j]);
  }

  state.currentIndex = index;
}

//==============================================================================
void moveForward(
    const WorldPtr& world, SoftBodiesState& state, std::size_t delta)
{
  moveTo(world, state, state.currentIndex + delta);
}

//==============================================================================
void moveBackward(
    const WorldPtr& world, SoftBodiesState& state, std::size_t delta)
{
  const std::size_t target
      = delta > state.currentIndex ? 0 : state.currentIndex - delta;
  moveTo(world, state, target);
}

} // namespace

//==============================================================================
DemoScene makeSoftBodiesScene()
{
  DemoScene scene;
  scene.id = "soft_bodies";
  scene.title = "Soft Bodies";
  scene.category = "Soft Bodies";
  scene.summary = "Soft-body simulation with recorded-state playback.";

  scene.factory = [] {
    auto world = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/softBodies.skel");
    if (!world)
      throw std::runtime_error(
          "failed to load dart://sample/skel/softBodies.skel");
    reorientWorldToZUp(world);

    auto state = std::make_shared<SoftBodiesState>();
    styleSoftBodyVisuals(world, state->display);
    // Prime frame 0 at build time, as the original's RecordingWorld
    // constructor does, so a fresh (still paused) scene already has a
    // recorded frame to scrub to.
    state->history.push_back(grabTimeSlice(world));
    state->framesRecorded = 1;
    state->currentIndex = 0;

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(3.0, -3.0, 2.2),
        ::osg::Vec3d(0.0, 0.0, 0.4),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.onActivate = [state](DemoHostContext& ctx) {
      state->viewer = ctx.viewer();
      state->worldNode = ctx.worldNode();
    };

    setup.postStep = [world, state] {
      recordTimeSlice(world, *state);
    };

    setup.keyActions.push_back(
        KeyAction{'[', "Step back 1 frame", [world, state] {
                    moveBackward(world, *state, 1);
                  }});
    setup.keyActions.push_back(
        KeyAction{']', "Step forward 1 frame", [world, state] {
                    moveForward(world, *state, 1);
                  }});
    setup.keyActions.push_back(
        KeyAction{'{', "Step back 10 frames", [world, state] {
                    moveBackward(world, *state, 10);
                  }});
    setup.keyActions.push_back(
        KeyAction{'}', "Step forward 10 frames", [world, state] {
                    moveForward(world, *state, 10);
                  }});
    setup.keyActions.push_back(
        KeyAction{'r', "Restart (frame 0)", [world, state] {
                    moveTo(world, *state, 0);
                  }});
    setup.keyActions.push_back(
        KeyAction{'\\', "Jump to latest frame", [world, state] {
                    if (!state->history.empty())
                      moveTo(world, *state, state->history.size() - 1);
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
          state->currentIndex,
          state->history.empty() ? 0 : state->history.size() - 1);
      ImGui::Text("Frames recorded (lifetime): %zu", state->framesRecorded);
      ImGui::TextWrapped(
          "'[' / ']' step one recorded frame back/forward, '{' / '}' step "
          "10, 'r' restarts, '\\' jumps to the latest frame. Scrubbing "
          "pauses the simulation; history is capped at %zu frames.",
          kMaxHistoryFrames);
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
