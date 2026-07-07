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

// Ported from examples/rigid_chain: a 10-link chain loaded from chain.skel,
// randomly posed at startup, with a per-joint velocity damping controller
// (twist dofs damped less than the other two per-joint dofs).

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>

#include <cmath>

namespace dart_demos {

namespace {

//==============================================================================
/// Per-instance state captured by this scene's preStep lambda and panel.
struct RigidChainState
{
  double dampingCoeff = 0.01;
  double twistDampingScale = 0.1;
};

//==============================================================================
// Same damping law as the original example: -dampingCoeff * velocity, with
// every third dof (the twist dof of each ball-ish joint triple) damped by an
// additional twistDampingScale factor.
Eigen::VectorXd computeDamping(
    const dart::dynamics::SkeletonPtr& skel, const RigidChainState& state)
{
  Eigen::VectorXd damping = -state.dampingCoeff * skel->getVelocities();
  for (int i = 0; i < static_cast<int>(damping.size()); ++i)
    if (i % 3 == 1)
      damping[i] *= state.twistDampingScale;
  return damping;
}

} // namespace

//==============================================================================
DemoScene makeRigidChainScene()
{
  DemoScene scene;
  scene.id = "rigid_chain";
  scene.title = "Rigid Chain";
  scene.category = "Rigid Body";
  scene.summary = "A damped articulated chain loaded from a skeleton file.";

  scene.factory = [] {
    auto world
        = dart::utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
    if (!world)
      throw std::runtime_error("failed to load dart://sample/skel/chain.skel");
    reorientWorldToZUp(world);
    // Timestep and the initial-pose range are kept as fixed constants rather
    // than scene-panel tunables: 1/2000 s is the small step the original
    // example relies on to keep this stiff 10-link chain's damping controller
    // stable (a coarser step visibly diverges), and the +/-0.5 rad spread is
    // the original's startup randomization. Both are behavioral fidelity
    // details, not parameters a viewer is expected to sweep; the host's global
    // Timestep slider already covers ad-hoc step experiments generically.
    world->setTimeStep(1.0 / 2000);

    auto skel = world->getSkeleton(0);
    const int dof = static_cast<int>(skel->getNumDofs());
    Eigen::VectorXd initPose(dof);
    for (int i = 0; i < dof; ++i)
      initPose[i] = dart::math::Random::uniform(-0.5, 0.5);
    skel->setPositions(initPose);

    auto state = std::make_shared<RigidChainState>();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.0, 1.0, 2.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    // Captured directly (not looked up via world->getSkeleton(0) each call)
    // so the damping controller always targets this scene's own chain, never
    // whatever happens to occupy index 0 in a future revision of this scene.
    setup.preStep = [skel, state] {
      skel->setForces(computeDamping(skel, *state));
    };

    setup.renderPanel = [chain = skel, state] {
      ImGui::Text("DOFs: %zu", chain->getNumDofs());
      ImGui::Text("Speed: %.3f rad/s", chain->getVelocities().norm());

      auto damping = static_cast<float>(state->dampingCoeff);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Damping",
              &damping,
              0.0f,
              0.1f,
              "%.4f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(damping))
        state->dampingCoeff = std::clamp(damping, 0.0f, 0.1f);

      auto twistScale = static_cast<float>(state->twistDampingScale);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Twist scale",
              &twistScale,
              0.0f,
              1.0f,
              "%.3f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(twistScale))
        state->twistDampingScale = std::clamp(twistScale, 0.0f, 1.0f);

      ImGui::TextWrapped(
          "Every third dof (the twist dof of each joint) is damped by an "
          "extra scale factor relative to the other two.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
