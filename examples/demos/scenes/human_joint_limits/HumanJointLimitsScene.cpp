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

// Ported from examples/human_joint_limits: kima_human_edited.skel with four
// neural-net joint-limit constraints (HumanArmJointLimitConstraint /
// HumanLegJointLimitConstraint, ported alongside this file) enforcing
// biomechanical range-of-motion limits on the shoulders, elbows, hips, knees,
// and ankles.
//
// This entire scene is compiled only when TinyDNN and Threads are found (see
// examples/demos/CMakeLists.txt), mirroring the original example's own
// optional-dependency CMake guard. The registry omits the entry when
// unavailable rather than showing a broken row.
//
// Deviations from the original: kima_human_edited.skel is Y-up and is
// reoriented to Z-up via the shared ZUp.hpp helper; the constraint math only
// reads/writes local joint dofs, which are unaffected by the
// reorientation (see ZUp.hpp). This skel is authored <gravity>0 -9.8 0</...>
// (|g|=9.8, not the usual 9.81); ZUp.hpp rotates the source gravity rather
// than hard-coding -9.81 Z, so that 9.8 magnitude is preserved -- the port
// simulates under the same gravity the original did. The four constraints are
// created eagerly at scene-build time (this factory runs fresh on every
// Rebuild/Reset) rather than lazily on the first customPreRefresh -- the
// original's lazy init existed only because customPreRefresh was the sole
// per-frame hook available to it; DemoSceneSetup already builds a fresh scene
// (and thus fresh constraints) on every (re)build, so no lazy re-arming logic
// is needed. The commented-out external-force snippet from the original is not
// ported (dead code). The stub HumanJointLimitsEventHandler (which handled no
// keys) is dropped entirely. The original set no custom camera home either, so
// none is set here.

#include "../Scenes.hpp"
#include "../ZUp.hpp"
#include "HumanArmJointLimitConstraint.hpp"
#include "HumanLegJointLimitConstraint.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <stdexcept>

namespace dart_demos {

namespace {

dart::dynamics::Joint* requireJoint(
    const dart::dynamics::SkeletonPtr& skel, const char* name)
{
  auto* joint = skel->getJoint(name);
  if (!joint)
    throw std::runtime_error(
        std::string("human_joint_limits: missing joint ") + name);
  return joint;
}

} // namespace

//==============================================================================
DemoScene makeHumanJointLimitsScene()
{
  DemoScene scene;
  scene.id = "human_joint_limits";
  scene.title = "Human Joint Limits";
  scene.category = "Constraints & Joints";
  scene.summary
      = "A human skeleton with neural-network joint-limit constraints.";

  scene.factory = [] {
    auto world = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/kima/kima_human_edited.skel");
    if (!world) {
      throw std::runtime_error(
          "failed to load dart://sample/skel/kima/kima_human_edited.skel");
    }
    reorientWorldToZUp(world);

    auto skel = world->getSkeleton("human");
    if (!skel)
      throw std::runtime_error(
          "kima_human_edited.skel: missing 'human' skeleton");

    for (auto* joint : skel->getJoints())
      joint->setLimitEnforcement(true);

    auto* solver = world->getConstraintSolver();
    solver->addConstraint(std::make_shared<HumanArmJointLimitConstraint>(
        requireJoint(skel, "j_bicep_left"),
        requireJoint(skel, "j_forearm_left"),
        false));
    solver->addConstraint(std::make_shared<HumanArmJointLimitConstraint>(
        requireJoint(skel, "j_bicep_right"),
        requireJoint(skel, "j_forearm_right"),
        true));
    solver->addConstraint(std::make_shared<HumanLegJointLimitConstraint>(
        requireJoint(skel, "j_thigh_left"),
        requireJoint(skel, "j_shin_left"),
        requireJoint(skel, "j_heel_left"),
        false));
    solver->addConstraint(std::make_shared<HumanLegJointLimitConstraint>(
        requireJoint(skel, "j_thigh_right"),
        requireJoint(skel, "j_shin_right"),
        requireJoint(skel, "j_heel_right"),
        true));

    DemoSceneSetup setup;
    setup.world = world;

    setup.renderPanel = [] {
      ImGui::TextWrapped(
          "Four HumanArmJointLimitConstraint/HumanLegJointLimitConstraint "
          "instances (shoulders+elbows, hips+knees+ankles) enforce "
          "biomechanical range-of-motion limits via a small neural network "
          "per limb. Tunables are process-global static setters on those "
          "classes (error allowance/ERP/max ERV/CFM); there is no runtime "
          "panel for them in the original example either.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
