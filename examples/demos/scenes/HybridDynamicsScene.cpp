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

// Ported from examples/hybrid_dynamics: fullbody1.skel with the root joint
// PASSIVE and every other joint VELOCITY-actuated, driven by a hardcoded
// sinusoidal gait (scapulae/forearms at 4 rad/s, shins at 2 rad/s, all
// functions of world time only -- no per-step dt term, so this controller is
// unaffected by the host's runtime Timestep slider). 'h' toggles a "harness"
// that locks the pelvis's parent joint (Joint::LOCKED) in place, matching the
// original's HybridDynamicsEventHandler.
//
// Deviations from the original: fullbody1.skel is authored Y-up; reoriented
// to the host's Z-up convention via the shared ZUp.hpp helper (the same
// treatment as every other fullbody1.skel-based B3 scene). The harness here
// directly flips the pelvis parent joint's actuator type (LOCKED <-> PASSIVE),
// exactly as the original did -- this is a different mechanism from
// JointConstraintsScene's WeldJointConstraint harness (a separate B2 example
// with its own, unrelated "harness" feature). Since the scene factory rebuilds
// a fresh skeleton (defaulting joint 0 to PASSIVE) on every Rebuild/Reset/
// switch, the harness always starts unlocked, matching the original's initial
// state.

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>
#include <vector>

#include <cmath>

namespace dart_demos {

namespace {

using dart::dynamics::Joint;
using dart::dynamics::SkeletonPtr;

const char* const kPelvisName = "h_pelvis";

//==============================================================================
/// Per-instance state captured by this scene's preStep/key-action lambdas.
struct HybridDynamicsState
{
  bool harnessOn = false;
};

//==============================================================================
void toggleHarness(const SkeletonPtr& skel, HybridDynamicsState& state)
{
  auto* pelvis = skel->getBodyNode(kPelvisName);
  if (!pelvis)
    return;
  auto* joint = pelvis->getParentJoint();
  if (!joint)
    return;

  state.harnessOn = !state.harnessOn;
  joint->setActuatorType(state.harnessOn ? Joint::LOCKED : Joint::PASSIVE);
}

} // namespace

//==============================================================================
DemoScene makeHybridDynamicsScene()
{
  DemoScene scene;
  scene.id = "hybrid_dynamics";
  scene.title = "Hybrid Dynamics";
  scene.category = "Control & IK";
  scene.summary
      = "Velocity-driven sinusoidal gait with a lockable pelvis harness.";

  scene.factory = [] {
    auto world = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/fullbody1.skel");
    if (!world)
      throw std::runtime_error(
          "failed to load dart://sample/skel/fullbody1.skel");
    reorientWorldToZUp(world);

    if (world->getNumSkeletons() < 2)
      throw std::runtime_error("fullbody1.skel: expected biped at index 1");
    auto skel = world->getSkeleton(1);

    // Initial pose, unaffected by the reorientation (see ZUp.hpp): global
    // orientation/position y, hips, knees, ankles, lower back.
    const std::vector<std::size_t> genCoordIds = {1, 6, 9, 10, 13, 16, 17, 21};
    if (skel->getNumDofs() <= 21)
      throw std::runtime_error(
          "fullbody1.skel: expected >= 22 DOFs for the initial pose");
    Eigen::VectorXd initConfig(8);
    initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
    skel->setPositions(genCoordIds, initConfig);

    auto* joint0 = skel->getJoint(0);
    if (joint0)
      joint0->setActuatorType(Joint::PASSIVE);
    for (std::size_t i = 1; i < skel->getNumBodyNodes(); ++i) {
      auto* joint = skel->getJoint(i);
      if (joint)
        joint->setActuatorType(Joint::VELOCITY);
    }

    auto state = std::make_shared<HybridDynamicsState>();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.0, -3.0, 3.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [world, skel] {
      auto setSin = [&](const char* jointName, double amp, double freq) {
        auto* joint = skel->getJoint(jointName);
        if (!joint)
          return;
        skel->setCommand(
            joint->getIndexInSkeleton(0),
            amp * std::sin(world->getTime() * freq));
      };

      setSin("j_scapula_left", 1.0, 4.0);
      setSin("j_scapula_right", -1.0, 4.0);
      setSin("j_forearm_left", 0.8, 4.0);
      setSin("j_forearm_right", 0.8, 4.0);
      setSin("j_shin_left", 0.1, 2.0);
      setSin("j_shin_right", 0.1, 2.0);
    };

    setup.keyActions.push_back(KeyAction{'h', "Toggle harness", [skel, state] {
                                           toggleHarness(skel, *state);
                                         }});

    setup.renderPanel = [state] {
      ImGui::Text("Harness: %s", state->harnessOn ? "locked" : "unlocked");
      ImGui::TextWrapped(
          "Velocity-actuated sinusoidal gait (code-constant amplitudes/"
          "frequencies, no GUI). 'h' locks/unlocks the pelvis joint.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
