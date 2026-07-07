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

// Ported from examples/mixed_chain: a hybrid rigid/soft chain
// (test_articulated_bodies_10bodies.skel) with a random initial pose on its
// first 3 dofs; keys q/w/e/r/t/y apply a 100-step, +/-500 N impulse to the
// chain's 4th soft body node.
//
// Deviations from the original:
//  - The .skel is authored Y-up; the original paired it with a Z-up camera, so
//    its ground plane rendered as a vertical wall. Here the world is reoriented
//    to the host's Z-up convention (reorientWorldToZUp, geometry + gravity
//    rotated by RotX(+90deg)) and given a camera that frames the chain. The six
//    force keys are rotated through the same transform so each still pokes the
//    soft body in the physical direction the original did; because that
//    transform maps each source axis to a single reoriented axis
//    ((fx,fy,fz)->(fx,-fz,fy)), the labels below name the resulting Z-up axis
//    (source +-Y, vertical in the Y-up world, becomes +-Z; source +-Z becomes
//    -+Y; +-X is invariant).
//  - The impulse-duration countdown lives directly in this scene's own state
//    struct rather than a separate event-handler class, since DemoSceneSetup's
//    KeyAction callbacks already give every key its own host-managed hook. Each
//    key sets one axis of the force and re-arms the countdown, preserving the
//    other axes, so two keys pressed within the window compose -- matching the
//    original (an earlier port replaced the whole vector, dropping the other
//    axis). The impulse is constant at 500 N for 100 steps and then cut to zero
//    (it does not decay). The six keys are scoped to this scene only (the host
//    clears all key actions on scene switch), so despite sharing letters with
//    other scenes' bindings there is no actual collision.

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

constexpr double kForceOnSoftBody = 500.0;
constexpr int kImpulseDurationSteps = 100;
constexpr std::size_t kTargetSoftBodyIndex = 3;

//==============================================================================
/// Per-instance state captured by this scene's preStep/key-action lambdas.
struct MixedChainState
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  int impulseDuration = 0;
};

//==============================================================================
// Sets a single component of the impulse and (re)arms the countdown, keeping
// the other components -- matching the original, where each key writes just one
// axis of mForceOnRigidBody so two keys within the window compose.
void applyImpulse(MixedChainState& state, int axis, double value)
{
  state.force[axis] = value;
  state.impulseDuration = kImpulseDurationSteps;
}

} // namespace

//==============================================================================
DemoScene makeMixedChainScene()
{
  DemoScene scene;
  scene.id = "mixed_chain";
  scene.title = "Mixed Chain";
  scene.category = "Soft Bodies";
  scene.summary = "Apply impulses to a soft link in a mixed rigid/soft chain.";

  scene.factory = [] {
    auto world = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/test/test_articulated_bodies_10bodies.skel");
    if (!world) {
      throw std::runtime_error(
          "failed to load "
          "dart://sample/skel/test/test_articulated_bodies_10bodies.skel");
    }

    reorientWorldToZUp(world);

    auto skel = world->getSkeleton(1);
    if (skel) {
      const int dof = static_cast<int>(skel->getNumDofs());
      Eigen::VectorXd initPose = skel->getPositions();
      for (int i = 0; i < std::min(3, dof); ++i)
        initPose[i] = dart::math::Random::uniform(-0.5, 0.5);
      skel->setPositions(initPose);
    }

    auto state = std::make_shared<MixedChainState>();

    DemoSceneSetup setup;
    setup.world = world;
    // The chain is anchored high (its base sits at z~5 after reorientation)
    // and hangs down, so frame the vertical span rather than the ground plane.
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(7.5, -7.5, 3.2),
        ::osg::Vec3d(0.0, 0.0, 2.2),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [world, state] {
      if (state->impulseDuration <= 0)
        return;

      const auto& chain = world->getSkeleton(1);
      if (chain && chain->getNumSoftBodyNodes() > kTargetSoftBodyIndex) {
        if (auto* softBody = chain->getSoftBodyNode(kTargetSoftBodyIndex))
          softBody->addExtForce(state->force);
      }

      --state->impulseDuration;
      if (state->impulseDuration <= 0) {
        state->impulseDuration = 0;
        state->force.setZero();
      }
    };

    // Axes are named in the reoriented (Z-up) frame; the values are the source
    // world-frame pokes rotated by (fx,fy,fz)->(fx,-fz,fy) (see the file
    // comment), which is why e/r land on Z (source +-Y) and t/y on Y (source
    // +-Z). Each key sets one axis so they compose (see applyImpulse).
    setup.keyActions.push_back(KeyAction{'q', "Force -X on soft link", [state] {
                                           applyImpulse(
                                               *state, 0, -kForceOnSoftBody);
                                         }});
    setup.keyActions.push_back(KeyAction{'w', "Force +X on soft link", [state] {
                                           applyImpulse(
                                               *state, 0, kForceOnSoftBody);
                                         }});
    setup.keyActions.push_back(KeyAction{'e', "Force -Z on soft link", [state] {
                                           applyImpulse(
                                               *state, 2, -kForceOnSoftBody);
                                         }});
    setup.keyActions.push_back(KeyAction{'r', "Force +Z on soft link", [state] {
                                           applyImpulse(
                                               *state, 2, kForceOnSoftBody);
                                         }});
    setup.keyActions.push_back(KeyAction{'t', "Force +Y on soft link", [state] {
                                           applyImpulse(
                                               *state, 1, kForceOnSoftBody);
                                         }});
    setup.keyActions.push_back(KeyAction{'y', "Force -Y on soft link", [state] {
                                           applyImpulse(
                                               *state, 1, -kForceOnSoftBody);
                                         }});

    setup.renderPanel = [world, state] {
      const auto& chain = world->getSkeleton(1);
      ImGui::Text(
          "Soft body nodes: %zu",
          chain ? chain->getNumSoftBodyNodes() : std::size_t(0));
      ImGui::Text("Impulse remaining: %d steps", state->impulseDuration);
      ImGui::TextWrapped(
          "q/w apply -X/+X, e/r apply -Z/+Z, t/y apply +Y/-Y: a 500 N "
          "impulse on the chain's 4th soft body node, applied for 100 "
          "steps.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
