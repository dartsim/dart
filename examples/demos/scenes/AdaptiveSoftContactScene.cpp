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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// A soft ellipsoid repeatedly contacted by the ground and a periodic pusher.
// Gold point markers estimate the active contact-local region using the exact
// public active-point count; blue markers show the remainder. The scene is
// programmatic and uses only the public adaptive-contact activation API.

#include "AdaptiveSoftContactModel.hpp"
#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace dart_demos {

namespace {

using dart::dynamics::Frame;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SimpleFramePtr;
using dart::dynamics::SphereShape;

namespace adaptive_model = adaptive_soft_contact_model;

constexpr int kMaximumRingCount = 4;
constexpr int kMaximumLingerSteps = 120;

//==============================================================================
struct AdaptiveSoftContactState
{
  adaptive_model::Model model;
  std::vector<SimpleFramePtr> pointMarkers;
  bool adaptiveEnabled = true;
  bool finite = true;
  std::size_t ringCount = adaptive_model::kDefaultRingCount;
  std::size_t lingerSteps = adaptive_model::kDefaultLingerSteps;
};

//==============================================================================
void createPointMarkers(AdaptiveSoftContactState& state)
{
  const std::size_t count = state.model.softBody->getNumPointMasses();
  state.pointMarkers.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto marker = std::make_shared<SimpleFrame>(
        Frame::World(), "activation_marker_" + std::to_string(i));
    marker->setShape(std::make_shared<SphereShape>(0.018));
    marker->createVisualAspect();
    state.model.world->addSimpleFrame(marker);
    state.pointMarkers.push_back(std::move(marker));
  }
}

//==============================================================================
void applySettings(AdaptiveSoftContactState& state)
{
  state.ringCount = std::min<std::size_t>(
      state.ringCount, static_cast<std::size_t>(kMaximumRingCount));
  state.lingerSteps = std::min<std::size_t>(
      state.lingerSteps, static_cast<std::size_t>(kMaximumLingerSteps));

  adaptive_model::configure(
      state.model, state.adaptiveEnabled, state.ringCount, state.lingerSteps);
}

//==============================================================================
void updatePointMarkers(AdaptiveSoftContactState& state)
{
  std::vector<Eigen::Vector3d> contactPoints;
  for (const auto& contact :
       state.model.world->getLastCollisionResult().getContacts()) {
    if (contact.point.allFinite())
      contactPoints.push_back(contact.point);
  }

  std::vector<std::pair<double, std::size_t>> ranked;
  ranked.reserve(state.pointMarkers.size());
  for (std::size_t i = 0; i < state.pointMarkers.size(); ++i) {
    const Eigen::Vector3d position
        = state.model.softBody->getPointMass(i)->getWorldPosition();
    if (!position.allFinite()) {
      ranked.emplace_back(std::numeric_limits<double>::infinity(), i);
      continue;
    }
    state.pointMarkers[i]->setRelativeTranslation(position);
    double distance = contactPoints.empty()
                          ? position.z()
                          : std::numeric_limits<double>::infinity();
    for (const auto& contactPoint : contactPoints)
      distance = std::min(distance, (position - contactPoint).squaredNorm());
    ranked.emplace_back(distance, i);
  }
  std::stable_sort(ranked.begin(), ranked.end());

  std::vector<bool> highlighted(state.pointMarkers.size(), false);
  const std::size_t active = std::min(
      state.model.softBody->getNumActivePointMasses(),
      state.pointMarkers.size());
  for (std::size_t i = 0; i < active; ++i)
    highlighted[ranked[i].second] = true;

  for (std::size_t i = 0; i < state.pointMarkers.size(); ++i) {
    const Eigen::Vector4d color = highlighted[i]
                                      ? Eigen::Vector4d(1.0, 0.78, 0.08, 1.0)
                                      : Eigen::Vector4d(0.08, 0.18, 0.48, 0.45);
    state.pointMarkers[i]->getVisualAspect(true)->setColor(color);
  }
}

} // namespace

//==============================================================================
DemoScene makeAdaptiveSoftContactScene()
{
  DemoScene scene;
  scene.id = "adaptive_soft_contact";
  scene.title = "Adaptive Soft Contact";
  scene.category = "Soft Bodies";
  scene.summary
      = "Visualize contact-local point-mass activation around a soft body.";

  scene.factory = [] {
    auto state = std::make_shared<AdaptiveSoftContactState>();
    state->model = adaptive_model::createModel(
        state->adaptiveEnabled, state->ringCount, state->lingerSteps);
    createPointMarkers(*state);
    updatePointMarkers(*state);

    DemoSceneSetup setup;
    setup.world = state->model.world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(3.1, 2.4, 2.0),
        ::osg::Vec3d(0.0, 0.0, 0.42),
        ::osg::Vec3d(0.0, 0.0, 1.0)};
    setup.preStep = [state] {
      applySettings(*state);
      adaptive_model::prepareStep(state->model);
    };
    setup.postStep = [state] {
      state->finite = adaptive_model::isFinite(state->model);
      updatePointMarkers(*state);
    };
    setup.preRefresh = [state] {
      applySettings(*state);
      updatePointMarkers(*state);
    };
    setup.keyActions.push_back(
        KeyAction{'a', "Toggle adaptive activation", [state] {
                    state->adaptiveEnabled = !state->adaptiveEnabled;
                  }});
    setup.renderPanel = [state] {
      bool adaptive = state->adaptiveEnabled;
      if (ImGui::Checkbox("Adaptive activation", &adaptive))
        state->adaptiveEnabled = adaptive;

      int ringCount = static_cast<int>(std::min<std::size_t>(
          state->ringCount, static_cast<std::size_t>(kMaximumRingCount)));
      if (ImGui::SliderInt(
              "Contact rings",
              &ringCount,
              0,
              kMaximumRingCount,
              "%d",
              ImGuiSliderFlags_AlwaysClamp)) {
        state->ringCount = static_cast<std::size_t>(
            std::clamp(ringCount, 0, kMaximumRingCount));
      }

      int lingerSteps = static_cast<int>(std::min<std::size_t>(
          state->lingerSteps, static_cast<std::size_t>(kMaximumLingerSteps)));
      if (ImGui::SliderInt(
              "Linger steps",
              &lingerSteps,
              0,
              kMaximumLingerSteps,
              "%d",
              ImGuiSliderFlags_AlwaysClamp)) {
        state->lingerSteps = static_cast<std::size_t>(
            std::clamp(lingerSteps, 0, kMaximumLingerSteps));
      }

      const std::size_t active
          = state->model.softBody->getNumActivePointMasses();
      const std::size_t total = state->model.softBody->getNumPointMasses();
      ImGui::Separator();
      ImGui::Text("Active point masses:   %zu", active);
      ImGui::Text("Inactive point masses: %zu", total - active);
      ImGui::Text(
          "Contacts:              %zu",
          state->model.world->getLastCollisionResult().getNumContacts());
      ImGui::Text(
          "Simulation step:       %d", state->model.world->getSimFrames());
      if (!state->finite)
        ImGui::TextColored(
            ImVec4(1.0f, 0.25f, 0.2f, 1.0f), "State is non-finite");
      ImGui::Separator();
      ImGui::TextWrapped(
          "Gold markers are a contact-nearest region estimate sized to the "
          "exact public active count; blue markers are the remainder. The "
          "orange pusher repeatedly makes and breaks side contact.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
