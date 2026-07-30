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

// Host adapter for the programmatic five-link soft-worm model. The reusable
// model/controller stays GUI-free so the locomotion and determinism contract
// can be exercised directly by an integration test.

#include "Scenes.hpp"
#include "SoftWormModel.hpp"

#include <dart/gui/osg/osg.hpp>

#include <memory>

namespace dart_demos {

namespace {

//==============================================================================
struct SoftWormState
{
  bool gaitEnabled = true;
  bool finite = true;
  double initialRootX = 0.0;
};

} // namespace

//==============================================================================
DemoScene makeSoftWormScene()
{
  DemoScene scene;
  scene.id = "soft_worm";
  scene.title = "Soft Worm";
  scene.category = "Soft Bodies";
  scene.summary
      = "A five-link soft-bodied worm driven by a traveling-wave gait.";

  scene.factory = [] {
    dart::dynamics::SkeletonPtr worm;
    auto world = soft_worm_model::createWorld(worm);
    auto state = std::make_shared<SoftWormState>();
    state->initialRootX = soft_worm_model::getRootX(worm);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(6.6, -6.8, 3.6),
        ::osg::Vec3d(2.4, 0.0, 0.25),
        ::osg::Vec3d(0.0, 0.0, 1.0)};
    setup.preStep = [world, worm, state] {
      soft_worm_model::applyGait(
          worm, world->getTime(), state->gaitEnabled && state->finite);
    };
    setup.postStep = [worm, state] {
      state->finite = soft_worm_model::isFinite(worm);
      if (!state->finite)
        state->gaitEnabled = false;
    };
    setup.keyActions.push_back(
        KeyAction{'g', "Toggle traveling-wave gait", [state] {
                    state->gaitEnabled = !state->gaitEnabled;
                  }});
    setup.renderPanel = [world, worm, state] {
      ImGui::Checkbox("Traveling-wave gait", &state->gaitEnabled);
      ImGui::Text("Links: %zu", soft_worm_model::kNumLinks);
      ImGui::Text("Driven joints: %zu", soft_worm_model::kNumLinks - 1);
      ImGui::Text("Root x: %.3f m", soft_worm_model::getRootX(worm));
      ImGui::Text(
          "Displacement: %.3f m",
          soft_worm_model::getRootX(worm) - state->initialRootX);
      ImGui::Text(
          "Frame: %d   contacts: %zu",
          world->getSimFrames(),
          world->getLastCollisionResult().getNumContacts());
      ImGui::Text(
          "Gait: %.2f Hz   amplitude: %.2f rad",
          soft_worm_model::kGaitFrequencyHz,
          soft_worm_model::kGaitAmplitude);
      ImGui::Text(
          "Position checksum: %.6e", soft_worm_model::positionChecksum(worm));
      if (!state->finite)
        ImGui::TextColored(
            ImVec4(1.0f, 0.25f, 0.2f, 1.0f), "State is non-finite");
      ImGui::TextWrapped(
          "Each articulated link carries 3x3x3 box-shaped soft flesh. The "
          "phase-offset sinusoidal gait can be paused with g.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
