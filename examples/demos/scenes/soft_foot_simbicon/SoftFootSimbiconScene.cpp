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
  return feet == sfs::Feet::Soft ? "Soft (SoftBodyNode)"
                                 : "Rigid (rest-mesh control)";
}

// Motor-noise level the 'm' key toggles: the same 20% held actuation error
// the motor-noise gate measures at, so what the demo shows is the gate's own
// operating point.
constexpr double kDemoMotorNoise = sfs::kMotorNoiseGateLevel;

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

  // Persists across Rebuild/Reset (which re-run this factory); the feet and
  // floor toggles record the next selection here and it is picked up on the
  // next rebuild.
  //
  // Environment variables exist so headless evidence capture is reproducible,
  // since the keyboard controls below cannot be reached without a window:
  // DART_DEMO_SOFT_FOOT_FEET selects rigid or soft geometry,
  // DART_DEMO_SOFT_FOOT_PUSH_STEP schedules one automatic sideways push at the
  // given step so a capture shows the push-recovery behavior rather than a
  // biped standing still, DART_DEMO_SOFT_FOOT_PUSH_N overrides that push's
  // magnitude so a capture can bracket the recovery threshold,
  // DART_DEMO_SOFT_FOOT_NOISE sets a multiplicative motor-noise level, and
  // DART_DEMO_SOFT_FOOT_FLOOR sets a noisy-tile-floor height amplitude in
  // meters. None of them affects interactive use.
  auto feetSelection = std::make_shared<sfs::Feet>(sfs::Feet::Soft);
  if (const char* feetEnv = std::getenv("DART_DEMO_SOFT_FOOT_FEET")) {
    if (std::string(feetEnv) == "rigid")
      *feetSelection = sfs::Feet::Rigid;
  }
  const int scriptedPushStep = []() {
    const char* stepEnv = std::getenv("DART_DEMO_SOFT_FOOT_PUSH_STEP");
    return stepEnv != nullptr ? std::atoi(stepEnv) : 0;
  }();
  const double scriptedPushMagnitude = []() {
    const char* magnitudeEnv = std::getenv("DART_DEMO_SOFT_FOOT_PUSH_N");
    return magnitudeEnv != nullptr ? std::atof(magnitudeEnv)
                                   : sfs::kDefaultPushMagnitude;
  }();
  auto noiseSelection = std::make_shared<double>([]() {
    const char* noiseEnv = std::getenv("DART_DEMO_SOFT_FOOT_NOISE");
    return noiseEnv != nullptr ? std::atof(noiseEnv) : 0.0;
  }());
  auto floorSelection = std::make_shared<double>([]() {
    const char* floorEnv = std::getenv("DART_DEMO_SOFT_FOOT_FLOOR");
    return floorEnv != nullptr ? std::atof(floorEnv) : 0.0;
  }());

  scene.factory = [feetSelection,
                   scriptedPushStep,
                   scriptedPushMagnitude,
                   noiseSelection,
                   floorSelection] {
    auto state = std::make_shared<SoftFootSimbiconState>();
    state->model = sfs::createModel(*feetSelection, *floorSelection);
    sfs::setMotorNoise(state->model, *noiseSelection);

    DemoSceneSetup setup;
    setup.world = state->model.world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.14, 3.28, 6.28) * 1.6,
        ::osg::Vec3d(0.00, 0.00, 0.00),
        ::osg::Vec3d(0.00, 0.10, 0.00)};

    setup.preStep = [state, scriptedPushStep, scriptedPushMagnitude] {
      if (scriptedPushStep > 0
          && state->model.world->getSimFrames() == scriptedPushStep) {
        sfs::applyPush(
            state->model,
            Eigen::Vector3d(0.0, 0.0, 1.0) * scriptedPushMagnitude,
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
    // The floor is world structure, so like the feet it applies on the host's
    // Reset/Rebuild; motor noise only scales what the controller writes each
    // step, so it can toggle live.
    setup.keyActions.push_back(KeyAction{
        'n',
        "Toggle noisy tile floor (applies on host Reset/Rebuild)",
        [floorSelection] {
          *floorSelection
              = *floorSelection > 0.0 ? 0.0 : sfs::kFloorPaperAmplitude;
        }});
    setup.keyActions.push_back(KeyAction{
        'm', "Toggle motor noise", [state, noiseSelection] {
          *noiseSelection = *noiseSelection > 0.0 ? 0.0 : kDemoMotorNoise;
          sfs::setMotorNoise(state->model, *noiseSelection);
        }});

    setup.renderPanel = [state, feetSelection, floorSelection] {
      const auto& model = state->model;
      ImGui::Text("Feet in world: %s", feetLabel(model.feet));
      if (*feetSelection != model.feet) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.80f, 0.25f, 1.0f),
            "Pending: %s -- press the host's Reset or Rebuild to apply",
            feetLabel(*feetSelection));
      }
      if (model.floorAmplitude > 0.0) {
        ImGui::Text(
            "Floor: noisy 5 cm tiles, %.1f cm amplitude",
            100.0 * model.floorAmplitude);
      } else {
        ImGui::Text("Floor: flat");
      }
      if (*floorSelection != model.floorAmplitude) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.80f, 0.25f, 1.0f),
            "Pending floor change -- press the host's Reset or Rebuild to "
            "apply");
      }
      if (model.motorNoise > 0.0)
        ImGui::Text("Motor noise: +/-%.0f%%", 100.0 * model.motorNoise);
      else
        ImGui::Text("Motor noise: off");

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
          "feet. 't' selects the other foot geometry and 'n' the noisy tile "
          "floor; both need a new world and so take effect on the host's "
          "Reset or Rebuild. 'm' toggles %.0f%% held motor noise live.\n\n"
          "Compare the two. Both foot types collide as the same rest-pose "
          "mesh with the same rest inertia; only the soft one deforms. Soft "
          "feet spread far more contact points (about 51 vs 16 over a "
          "settled window, gate-enforced at 1.5x) -- that Jain/Liu 2011 "
          "claim reproduces. Robust push recovery never shows a soft "
          "advantage (ensemble-measured, rigid vs soft: 8000 vs 4000 N "
          "clean, tied 4000 vs 4000 under 20%% motor noise, 6000 vs 2000 "
          "on the 2 cm floor); the paper's soft-advantage ordering is an "
          "open gap tracked in the parity matrix, and the gates publish "
          "the measured curves.",
          sfs::kDefaultPushMagnitude,
          sfs::kPushSteps,
          100.0 * kDemoMotorNoise);
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
