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

// Host adapter for the GUI-free soft-foot SIMBICON biped model
// (SoftFootSimbiconModel), reproducing the Jain/Liu soft-vs-rigid push-recovery
// comparison interactively: an Atlas humanoid balanced by the (reused,
// GUI-free) atlas_simbicon SIMBICON controller, whose feet can be rigid boxes
// or SoftBodyNodes.
//
// Like AtlasSimbiconScene this world is deliberately Y-up (the SIMBICON control
// math hardcodes world-Y as vertical). The feet-geometry toggle swaps which SDF
// the biped is loaded from; because a live World cannot have its foot bodies
// swapped in place, the toggle records the selection and takes effect on the
// host's Rebuild/Reset (which re-runs this factory) -- the persistent selection
// is captured by the factory lambda so it survives the rebuild.

#include "../Scenes.hpp"
#include "../atlas_simbicon/Controller.hpp"
#include "../atlas_simbicon/State.hpp"
#include "../atlas_simbicon/StateMachine.hpp"
#include "SoftFootSimbiconModel.hpp"

#include <dart/gui/osg/osg.hpp>

#include <memory>
#include <string>

#include <cstdlib>

namespace dart_demos {

namespace {

namespace sfs = soft_foot_simbicon_model;

//==============================================================================
struct SoftFootSimbiconState
{
  sfs::Model model;
  bool finite = true;
};

//==============================================================================
const char* feetLabel(sfs::Feet feet)
{
  return feet == sfs::Feet::Soft ? "Soft (SoftBodyNode)" : "Rigid (box)";
}

} // namespace

//==============================================================================
DemoScene makeSoftFootSimbiconScene()
{
  DemoScene scene;
  scene.id = "soft_foot_simbicon";
  scene.title = "Soft-Foot SIMBICON";
  scene.category = "Soft Bodies";
  scene.summary
      = "A SIMBICON-balanced Atlas biped comparing soft vs rigid foot contact "
        "under pushes (Jain/Liu push recovery).";

  // Persists across Rebuild/Reset (which re-run this factory); the feet toggle
  // records the next geometry here and it is picked up on the next rebuild.
  //
  // Two environment variables exist so headless evidence capture is
  // reproducible, since the keyboard controls below cannot be reached without a
  // window: DART_DEMO_SOFT_FOOT_FEET selects rigid or soft geometry, and
  // DART_DEMO_SOFT_FOOT_PUSH_STEP schedules one automatic sideways push at the
  // given step so a capture shows the push-recovery behavior rather than a
  // biped standing still. Neither affects interactive use.
  auto feetSelection = std::make_shared<sfs::Feet>(sfs::Feet::Soft);
  if (const char* feetEnv = std::getenv("DART_DEMO_SOFT_FOOT_FEET")) {
    if (std::string(feetEnv) == "rigid")
      *feetSelection = sfs::Feet::Rigid;
  }
  const int scriptedPushStep = []() {
    const char* stepEnv = std::getenv("DART_DEMO_SOFT_FOOT_PUSH_STEP");
    return stepEnv != nullptr ? std::atoi(stepEnv) : 0;
  }();

  scene.factory = [feetSelection, scriptedPushStep] {
    auto state = std::make_shared<SoftFootSimbiconState>();
    state->model = sfs::createModel(*feetSelection);

    DemoSceneSetup setup;
    setup.world = state->model.world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.14, 3.28, 6.28) * 1.6,
        ::osg::Vec3d(0.00, 0.00, 0.00),
        ::osg::Vec3d(0.00, 0.10, 0.00)};

    setup.preStep = [state, scriptedPushStep] {
      if (scriptedPushStep > 0
          && state->model.world->getSimFrames() == scriptedPushStep) {
        sfs::applyPush(
            state->model,
            Eigen::Vector3d(0.0, 0.0, 1.0) * sfs::kDefaultPushMagnitude,
            sfs::kPushSteps);
      }
      if (state->finite)
        sfs::prepareStep(state->model);
    };
    setup.postStep = [state] {
      state->finite = sfs::isFinite(state->model);
    };

    const auto push = [state](double fx, double fz) {
      sfs::applyPush(
          state->model,
          Eigen::Vector3d(fx, 0.0, fz) * sfs::kDefaultPushMagnitude,
          sfs::kPushSteps);
    };
    setup.keyActions.push_back(KeyAction{'a', "Push forward (+X)", [push] {
                                           push(1.0, 0.0);
                                         }});
    setup.keyActions.push_back(KeyAction{'s', "Push backward (-X)", [push] {
                                           push(-1.0, 0.0);
                                         }});
    setup.keyActions.push_back(KeyAction{'d', "Push left (+Z)", [push] {
                                           push(0.0, 1.0);
                                         }});
    setup.keyActions.push_back(KeyAction{'f', "Push right (-Z)", [push] {
                                           push(0.0, -1.0);
                                         }});
    // Two different resets exist here and they are not interchangeable. This
    // one restarts the biped inside the running world, which cannot swap the
    // foot geometry because that means loading a different SDF. The host's
    // Reset/Rebuild buttons re-run this factory and build a new World, which
    // is what actually applies a pending feet selection. The labels say so.
    setup.keyActions.push_back(
        KeyAction{'r', "Restart biped (keeps current feet)", [state] {
                    sfs::resetModel(state->model);
                    state->finite = true;
                  }});
    setup.keyActions.push_back(KeyAction{
        't', "Toggle feet (applies on host Reset/Rebuild)", [feetSelection] {
          *feetSelection = (*feetSelection == sfs::Feet::Soft)
                               ? sfs::Feet::Rigid
                               : sfs::Feet::Soft;
        }});

    setup.renderPanel = [state, feetSelection] {
      const auto& model = state->model;
      ImGui::Text("Feet in world: %s", feetLabel(model.feet));
      if (*feetSelection != model.feet) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.80f, 0.25f, 1.0f),
            "Pending: %s -- press the host's Reset or Rebuild to apply",
            feetLabel(*feetSelection));
      }

      ImGui::Separator();

      auto* stateMachine = model.controller->getCurrentState();
      ImGui::Text(
          "Gait: %s / state %s",
          stateMachine->getName().c_str(),
          stateMachine->getCurrentState()->getName().c_str());
      ImGui::Text("Pelvis height Y: %.3f m", sfs::pelvisHeightY(model));
      ImGui::Text("Foot contacts: %zu", sfs::footContactCount(model));
      ImGui::Text(
          "Total contacts: %zu",
          model.world->getLastCollisionResult().getNumContacts());
      ImGui::Text("Upright: %s", sfs::isUpright(model) ? "yes" : "NO (fallen)");
      if (model.forceDuration > 0)
        ImGui::Text("Pushing: %d steps left", model.forceDuration);
      if (!state->finite)
        ImGui::TextColored(
            ImVec4(1.0f, 0.25f, 0.2f, 1.0f), "State is non-finite");

      ImGui::Separator();
      ImGui::TextWrapped(
          "a/s push the pelvis forward/back, d/f push left/right (%.0f N for "
          "%d steps). 'r' restarts the biped in place, keeping the current "
          "feet. 't' selects the other foot geometry, which needs a new world "
          "and so takes effect on the host's Reset or Rebuild.\n\n"
          "Compare the two. Soft feet spread more contact points (about 52 vs "
          "44 over a settled window), which is the Jain/Liu 2011 result. They "
          "do NOT withstand larger pushes here: measured recovery is 4000 N "
          "soft against 8000 N rigid, because this asset's feet (kv 5e4) are "
          "too compliant for a 147 kg Atlas.",
          sfs::kDefaultPushMagnitude,
          sfs::kPushSteps);
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
