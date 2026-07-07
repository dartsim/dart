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

// Ported from examples/rigid_loop: the same chain.skel as RigidChainScene,
// posed into a loop and closed with a BallJointConstraint between links 6 and
// 10 (highlighted red), with the same per-joint damping controller.

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
struct RigidLoopState
{
  double dampingCoeff = 0.01;
  double twistDampingScale = 0.1;
};

//==============================================================================
Eigen::VectorXd computeDamping(
    const dart::dynamics::SkeletonPtr& skel, const RigidLoopState& state)
{
  Eigen::VectorXd damping = -state.dampingCoeff * skel->getVelocities();
  for (int i = 0; i < static_cast<int>(damping.size()); ++i)
    if (i % 3 == 1)
      damping[i] *= state.twistDampingScale;
  return damping;
}

} // namespace

//==============================================================================
DemoScene makeRigidLoopScene()
{
  DemoScene scene;
  scene.id = "rigid_loop";
  scene.title = "Rigid Loop";
  scene.category = "Constraints & Joints";
  scene.summary = "A chain closed into a loop with a ball-joint constraint.";

  scene.factory = [] {
    auto world
        = dart::utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
    if (!world)
      throw std::runtime_error("failed to load dart://sample/skel/chain.skel");
    reorientWorldToZUp(world);
    world->setTimeStep(1.0 / 2000);

    auto skel = world->getSkeleton(0);
    const int dof = static_cast<int>(skel->getNumDofs());
    Eigen::VectorXd initPose(dof);
    initPose.setZero();
    if (dof > 29) {
      initPose[20] = 3.14159 * 0.4;
      initPose[23] = 3.14159 * 0.4;
      initPose[26] = 3.14159 * 0.4;
      initPose[29] = 3.14159 * 0.4;
    }
    skel->setPositions(initPose);

    // Close the chain into a loop with a ball-joint constraint between link 6
    // and link 10 (highlighted red), computed after reorientation so the
    // constraint's anchor point is expressed in the reoriented world frame.
    auto* bd1 = skel->getBodyNode("link 6");
    auto* bd2 = skel->getBodyNode("link 10");
    if (bd1 && bd2) {
      bd1->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
      bd2->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
      const Eigen::Vector3d offset(0.0, 0.025, 0.0);
      const Eigen::Vector3d jointPos = bd1->getTransform() * offset;
      // The original example closed the loop with a BallJointConstraint and
      // carried a commented-out `WeldJointConstraint(bd1, bd2)` as an
      // alternative (a rigid weld instead of a 3-dof ball). That alternative is
      // not carried over here: the ball-joint version is the one the original
      // actually built, and exposing a weld toggle is outside this scene's
      // ported scope.
      auto constraint = std::make_shared<dart::constraint::BallJointConstraint>(
          bd1, bd2, jointPos);
      world->getConstraintSolver()->addConstraint(constraint);
    }

    auto state = std::make_shared<RigidLoopState>();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.0, 1.0, 2.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    // Captured directly (not looked up via world->getSkeleton(0) each call)
    // so the damping controller always targets this scene's own chain.
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
          "The red links are connected by a ball-joint constraint, closing "
          "the chain into a loop.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
