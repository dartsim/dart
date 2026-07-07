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
// Deviations from the original:
//  - The recorded-frame playback mode (key 'p', World::bake()/getRecording())
//    is intentionally omitted. It relied on customPreRefresh, which runs once
//    per render frame independent of the simulate/pause state; DemoSceneSetup
//    (BRIEF-phase1.md's scene contract) only exposed preStep/postStep at the
//    time of this port (once per simulation step, only while running).
//    Rather than ship a playback mode that silently freezes whenever
//    Play/Pause is off (unlike the original), it was left out; Phase 3 has
//    since added DemoSceneSetup::preRefresh, so a playback mode could be
//    revisited, but that is a separate, larger feature and out of scope here.
//  - This scene's own bespoke contact-force visualization (key 'v', the
//    contactFrames/contactArrows/showContactForces machinery) has been
//    removed: BRIEF-phase3.md #3 extracts exactly this logic into a
//    reusable host-level facility (DemoHost's ContactVisualizer, toggled in
//    Diagnostics) that now applies to every scene, not just this one. Force
//    application (keys 1-4) is unaffected and preserved with full parity.

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>
#include <vector>

namespace dart_demos {

namespace {

//==============================================================================
/// Per-instance state captured by this scene's preStep/key-action lambdas
/// (one instance per factory() call, i.e. per Rebuild/Reset too).
struct RigidCubesState
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
};

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
    // cubes.skel is a compact scene (a 2.5x2.5 ground plate, cubes 0.05-0.1m
    // across); the original example's eye(5,5,5) leaves the stack a tiny,
    // distant speck. Frame it tightly around the stack instead.
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(1.1, 1.1, 0.55),
        ::osg::Vec3d(0.0, 0.0, -0.3),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [world, state] {
      // Apply the decaying force to the second skeleton's first body, as in
      // the original example.
      if (world->getNumSkeletons() > 1)
        world->getSkeleton(1)->getBodyNode(0)->addExtForce(state->force);
      state->force /= 2.0;
    };

    auto applyForce = [state](const Eigen::Vector3d& force) {
      state->force = force;
    };

    // Force axes in the reoriented (Z-up) world: keys 1/2 push along world X
    // (invariant under the reorientation). The original's keys 3/4 pushed along
    // world +-Z -- a horizontal axis in the Y-up asset, but the vertical axis
    // after reorientation, which shoved the stack up/down instead of sideways.
    // Rotate them through the same transform ZUp applies to the geometry
    // ((fx,fy,fz)->(fx,-fz,fy)), so +-Z maps to -+Y and the push stays
    // horizontal, matching the original's intent.
    setup.keyActions.push_back(KeyAction{'1', "Force -X", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(-500, 0, 0));
                                         }});
    setup.keyActions.push_back(KeyAction{'2', "Force +X", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(500, 0, 0));
                                         }});
    setup.keyActions.push_back(KeyAction{'3', "Force +Y", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(0, 500, 0));
                                         }});
    setup.keyActions.push_back(KeyAction{'4', "Force -Y", [applyForce] {
                                           applyForce(
                                               Eigen::Vector3d(0, -500, 0));
                                         }});
    setup.renderPanel = [world, state] {
      ImGui::Text("Skeletons: %zu", world->getNumSkeletons());
      ImGui::TextWrapped(
          "Force keys apply a 500 N impulse to the second skeleton's first "
          "body, decaying by half each step. Contact forces are visualized "
          "host-wide via Diagnostics > Contact Visualizer.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
