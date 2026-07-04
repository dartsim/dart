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

// Ported from examples/rigid_cubes: a loaded cube-stack skel with directional
// impulse forces (keys 1-4) and live contact-force visualization (key v).
//
// Deviation from the original: the recorded-frame playback mode (key 'p',
// World::bake()/getRecording()) is intentionally omitted. It relied on
// customPreRefresh, which runs once per render frame independent of the
// simulate/pause state; DemoSceneSetup (BRIEF-phase1.md's scene contract)
// only exposes preStep/postStep, which run once per simulation step and only
// while the host is not paused. Rather than ship a playback mode that
// silently freezes whenever Play/Pause is off (unlike the original), it is
// left out of this port; a preRefresh/postRefresh hook can be added to the
// contract in a later phase if per-frame (non-stepped) scene logic is needed
// again. Live force application and live contact-force visualization -- the
// more central interactive behaviors -- are preserved with full parity.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>
#include <vector>

namespace dart_demos {

namespace {

constexpr double kLiveContactForceScale = 0.1;

//==============================================================================
/// Per-instance state captured by this scene's preStep/postStep/key-action
/// lambdas (one instance per factory() call, i.e. per Rebuild/Reset too).
struct RigidCubesState
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  bool showContactForces = true;
  std::vector<dart::dynamics::SimpleFramePtr> contactFrames;
  std::vector<std::shared_ptr<dart::dynamics::ArrowShape>> contactArrows;
};

//==============================================================================
void ensureContactForceVisuals(
    RigidCubesState& state,
    const dart::simulation::WorldPtr& world,
    std::size_t count)
{
  while (state.contactFrames.size() < count) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World());
    auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitZ() * 0.01,
        dart::dynamics::ArrowShape::Properties(0.002, 2.0, 0.15),
        Eigen::Vector4d(0.2, 0.2, 0.8, 1.0));

    frame->setShape(arrow);
    frame->getVisualAspect(true)->setHidden(true);
    world->addSimpleFrame(frame);

    state.contactFrames.push_back(frame);
    state.contactArrows.push_back(arrow);
  }
}

//==============================================================================
void hideContactForcesFrom(RigidCubesState& state, std::size_t start)
{
  for (std::size_t i = start; i < state.contactFrames.size(); ++i)
    state.contactFrames[i]->getVisualAspect(true)->setHidden(true);
}

//==============================================================================
void updateContactForces(
    RigidCubesState& state, const dart::simulation::WorldPtr& world)
{
  if (!state.showContactForces) {
    hideContactForcesFrom(state, 0);
    return;
  }

  const auto& result = world->getConstraintSolver()->getLastCollisionResult();
  const auto& contacts = result.getContacts();
  ensureContactForceVisuals(state, world, contacts.size());

  for (std::size_t i = 0; i < contacts.size(); ++i) {
    const Eigen::Vector3d point = contacts[i].point;
    const Eigen::Vector3d force = kLiveContactForceScale * contacts[i].force;
    auto* visual = state.contactFrames[i]->getVisualAspect(true);
    if (force.norm() < 1e-8) {
      visual->setHidden(true);
    } else {
      visual->setHidden(false);
      state.contactArrows[i]->setPositions(point, point + force);
    }
  }

  hideContactForcesFrom(state, contacts.size());
}

//==============================================================================
// cubes.skel is authored Y-up (ground normal +Y; the .skel file itself embeds
// <gravity>0 -9.81 0</gravity>), which is why the original examples/rigid_cubes
// set gravity to (0, -9.81, 0) instead of reorienting the model. dart-demos
// uses one Z-up convention across every scene (matching DemoScene::cameraHome
// eye/center/up elsewhere in this file), so the model is reoriented here
// instead: premultiply every root joint's transform-from-parent by
// RotX(+90deg) (+Y -> +Z), then set gravity along -Z. Because the geometry and
// gravity are rotated by the same rigid transform, every joint's generalized
// coordinates evolve exactly as in the original Y-up world -- only the
// display frame changes. Ported from DART 7's `reorientWorldToZUp` helper
// (examples/demos/scenes/z_up.hpp at
// 1a5469960c2703accb2762e03fe8a6bb1156dc08).
void reorientWorldToZUp(const dart::simulation::WorldPtr& world)
{
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  // clang-format off
  rotation.linear() << 1.0, 0.0,  0.0,
                       0.0, 0.0, -1.0,
                       0.0, 1.0,  0.0;
  // clang-format on

  for (std::size_t si = 0; si < world->getNumSkeletons(); ++si) {
    const auto& skeleton = world->getSkeleton(si);
    for (std::size_t ji = 0; ji < skeleton->getNumJoints(); ++ji) {
      auto* joint = skeleton->getJoint(ji);
      if (joint->getParentBodyNode() != nullptr)
        continue; // only root joints connect to the world frame
      joint->setTransformFromParentBodyNode(
          rotation * joint->getTransformFromParentBodyNode());
    }
  }

  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
}

} // namespace

//==============================================================================
DemoScene makeRigidCubesScene()
{
  DemoScene scene;
  scene.id = "rigid_cubes";
  scene.title = "Rigid Cubes";
  scene.category = "Rigid Body";
  scene.summary
      = "A loaded cube stack; apply directional impulses and visualize "
        "live contact forces.";

  scene.factory = [] {
    auto world
        = dart::utils::SkelParser::readWorld("dart://sample/skel/cubes.skel");
    if (!world)
      throw std::runtime_error("failed to load dart://sample/skel/cubes.skel");
    reorientWorldToZUp(world);

    auto state = std::make_shared<RigidCubesState>();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.0, 5.0, 5.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [world, state] {
      // Apply the decaying force to the second skeleton's first body, as in
      // the original example.
      if (world->getNumSkeletons() > 1)
        world->getSkeleton(1)->getBodyNode(0)->addExtForce(state->force);
      state->force /= 2.0;
    };

    setup.postStep = [world, state] {
      updateContactForces(*state, world);
    };

    auto applyForce = [state](const Eigen::Vector3d& force) {
      state->force = force;
    };

    setup.keyActions.push_back(KeyAction{'1', "Force -X", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(-500, 0, 0));
                                         }});
    setup.keyActions.push_back(KeyAction{'2', "Force +X", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(500, 0, 0));
                                         }});
    setup.keyActions.push_back(KeyAction{'3', "Force -Z", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(0, 0, -500));
                                         }});
    setup.keyActions.push_back(KeyAction{'4', "Force +Z", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(0, 0, 500));
                                         }});
    setup.keyActions.push_back(KeyAction{'v', "Toggle contact forces", [state] {
                                           state->showContactForces
                                               = !state->showContactForces;
                                         }});

    setup.renderPanel = [world, state] {
      ImGui::Text("Skeletons: %zu", world->getNumSkeletons());
      ImGui::Text(
          "Contact force visualization: %s",
          state->showContactForces ? "on" : "off");
      ImGui::TextWrapped(
          "Force keys apply a 500 N impulse to the second skeleton's first "
          "body, decaying by half each step.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
