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

// Ported from examples/joint_constraints: fullbody1.skel balanced by an SPD
// tracking controller (holding its randomized-free initial pose) plus a
// sagittal-plane ankle strategy, with a togglable WeldJointConstraint
// "harness" on the pelvis and four scripted push perturbations on the spine.
//
// Deviations from the original: fullbody1.skel is authored Y-up (as required
// by BRIEF-phase2's ground rules, reoriented to the host's Z-up convention
// via the shared ZUp.hpp helper). The SPD gains, DOF-index coupling (6..21,
// 22..end), body names (h_pelvis/h_spine/h_heel_left), and the ankle
// strategy's use of com[0]/cop[0] (the sagittal-plane X axis) are all
// unaffected by the reorientation -- ZUp.hpp only premultiplies the root
// joint's fixed parent transform and rotates gravity, so generalized
// coordinates and any world-frame X component evolve identically to the
// original (X is invariant under the +90-degree rotation about world X that
// ZUp.hpp applies). The push-perturbation forces, however, ARE affected:
// they are world-frame forces along X ("forward/backward", unaffected -- X
// is invariant under the rotation) and Z ("right/left" in the original Y-up
// world, where Z was a horizontal axis). After reorientation Z is the
// vertical axis, so the original literal (0,0,+-50) force would now shove the
// character up/down instead of sideways; here the right/left force
// components are rotated through the same transform ZUp.hpp used for the
// geometry (new = (fx, -fz, fy)) so "push right"/"push left" still push
// sideways in the reoriented world, matching the labeled behavior. The
// harness toggle and perturbation countdown live in this scene's own state
// struct rather than a separate event handler class; behavior is unchanged.

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

namespace dart_demos {

namespace {

using dart::dynamics::SkeletonPtr;

const char* const kPelvisName = "h_pelvis";
const char* const kSpineName = "h_spine";
const char* const kLeftHeelName = "h_heel_left";

//==============================================================================
/// SPD tracking controller state, ported from examples/joint_constraints'
/// Controller class.
struct SpdController
{
  Eigen::MatrixXd kp;
  Eigen::MatrixXd kd;
  Eigen::VectorXd desiredDofs;
  Eigen::VectorXd
      constrForces; // Never fed from the solver, as in the original.
  double preOffset = 0.0;
  double timestep = 0.001;
};

//==============================================================================
SpdController makeController(const SkeletonPtr& skel, double timestep)
{
  SpdController controller;
  const int nDof = static_cast<int>(skel->getNumDofs());
  controller.kp = Eigen::MatrixXd::Zero(nDof, nDof);
  controller.kd = Eigen::MatrixXd::Zero(nDof, nDof);
  controller.constrForces = Eigen::VectorXd::Zero(nDof);
  controller.desiredDofs = skel->getPositions();
  controller.timestep = timestep;

  // Root 6 dofs keep zero gains (uncontrolled); lower body + lower back get
  // stiffer gains than the upper body, exactly as the original tuned them.
  for (int i = 6; i < std::min(22, nDof); ++i)
    controller.kp(i, i) = 200.0;
  for (int i = 22; i < nDof; ++i)
    controller.kp(i, i) = 20.0;
  for (int i = 6; i < std::min(22, nDof); ++i)
    controller.kd(i, i) = 100.0;
  for (int i = 22; i < nDof; ++i)
    controller.kd(i, i) = 10.0;

  return controller;
}

//==============================================================================
Eigen::VectorXd computeTorques(
    const SkeletonPtr& skel, SpdController& controller)
{
  const Eigen::VectorXd q = skel->getPositions();
  const Eigen::VectorXd dq = skel->getVelocities();
  const double dt = controller.timestep;

  const Eigen::MatrixXd invM
      = (skel->getMassMatrix() + controller.kd * dt).inverse();
  const Eigen::VectorXd p
      = -controller.kp * (q + dq * dt - controller.desiredDofs);
  const Eigen::VectorXd d = -controller.kd * dq;
  const Eigen::VectorXd qddot = invM
                                * (-skel->getCoriolisAndGravityForces() + p + d
                                   + controller.constrForces);
  Eigen::VectorXd torques = p + d - controller.kd * qddot * dt;

  // Ankle strategy for the sagittal plane: com[0]/cop[0] are the world-frame
  // X coordinates, invariant under the Y-up -> Z-up reorientation (see file
  // comment), so this logic is byte-for-byte identical to the original.
  auto* heel = skel->getBodyNode(kLeftHeelName);
  if (heel && torques.size() > 26) {
    const Eigen::Vector3d com = skel->getCOM();
    const Eigen::Vector3d cop
        = heel->getTransform() * Eigen::Vector3d(0.05, 0, 0);
    const double diff0 = com[0] - cop[0];
    if (diff0 < 0.1) {
      const double offset = com[0] - cop[0];
      constexpr double k1 = 20.0;
      constexpr double k2 = 10.0;
      constexpr double kdAnkle = 100.0;
      torques[17] += -k1 * offset + kdAnkle * (controller.preOffset - offset);
      torques[25] += -k2 * offset + kdAnkle * (controller.preOffset - offset);
      torques[19] += -k1 * offset + kdAnkle * (controller.preOffset - offset);
      torques[26] += -k2 * offset + kdAnkle * (controller.preOffset - offset);
      controller.preOffset = offset;
    }
  }

  // Illegal (uncontrolled) root torques are always zeroed, as in the original.
  for (int i = 0; i < std::min(6, static_cast<int>(torques.size())); ++i)
    torques[i] = 0.0;

  return torques;
}

//==============================================================================
/// Per-instance state captured by this scene's preStep/key-action lambdas.
struct JointConstraintsState
{
  SpdController controller;
  Eigen::Vector3d perturbationForce = Eigen::Vector3d::Zero();
  int impulseDuration = 0;

  bool harnessOn = false;
  dart::constraint::WeldJointConstraintPtr weldJoint;
};

//==============================================================================
// Sets a single world-frame component of the push and (re)arms the 100-step
// countdown, preserving the other components -- matching the original, where
// each key writes just one axis of mForce (mForce[0] or mForce[2]) so two keys
// pressed within the window compose into a diagonal push. (Replacing the whole
// vector, as an earlier port did, cancelled the other axis.)
void applyPerturbation(JointConstraintsState& state, int axis, double value)
{
  state.perturbationForce[axis] = value;
  state.impulseDuration = 100;
}

//==============================================================================
void toggleHarness(
    const dart::simulation::WorldPtr& world,
    const SkeletonPtr& skel,
    JointConstraintsState& state)
{
  state.harnessOn = !state.harnessOn;
  if (state.harnessOn) {
    auto* pelvis = skel->getBodyNode(kPelvisName);
    if (!pelvis)
      return;
    state.weldJoint
        = std::make_shared<dart::constraint::WeldJointConstraint>(pelvis);
    world->getConstraintSolver()->addConstraint(state.weldJoint);
  } else if (state.weldJoint) {
    world->getConstraintSolver()->removeConstraint(state.weldJoint);
    state.weldJoint = nullptr;
  }
}

} // namespace

//==============================================================================
DemoScene makeJointConstraintsScene()
{
  DemoScene scene;
  scene.id = "joint_constraints";
  scene.title = "Joint Constraints";
  scene.category = "Control & IK";
  scene.summary = "Balanced full-body PD control with a harness constraint.";

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

    // Initial pose: global orientation/position y, hips, knees, ankles, lower
    // back -- unaffected by the reorientation (see file comment). The indices
    // are hardcoded up to 21, so guard the DOF layout the same way
    // makeController and computeTorques guard theirs (every other
    // hardcoded-index path here is clamped or checked); a smaller skeleton
    // would otherwise index out of range.
    const std::vector<std::size_t> genCoordIds
        = {1, 4, 6, 9, 10, 13, 16, 17, 21};
    if (skel->getNumDofs() < 22)
      throw std::runtime_error(
          "fullbody1.skel: expected >= 22 DOFs for the initial pose");
    Eigen::VectorXd initConfig(9);
    initConfig << -0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1;
    skel->setPositions(genCoordIds, initConfig);

    auto state = std::make_shared<JointConstraintsState>();
    state->controller = makeController(skel, world->getTimeStep());

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.0, -3.0, 3.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [world, skel, state] {
      // SPD's implicit damping ((M + Kd*dt).inverse() and the p/d terms) must
      // use the world's actual integration step. The host's Timestep toolbar
      // can change the world dt at runtime without rebuilding the scene, so
      // re-read it every step instead of caching the build-time value -- a
      // stale dt makes the computed torques wrong for the real step and can
      // destabilize the biped.
      state->controller.timestep = world->getTimeStep();

      auto* spine = skel->getBodyNode(kSpineName);
      if (spine)
        spine->addExtForce(state->perturbationForce);

      skel->setForces(computeTorques(skel, state->controller));

      if (state->impulseDuration > 0) {
        --state->impulseDuration;
        if (state->impulseDuration <= 0)
          state->perturbationForce.setZero();
      }
    };

    // Push axes in the reoriented (Z-up) world: forward/backward is world X
    // (invariant under the reorientation); the original's right/left push was
    // along world +Z/-Z (a horizontal axis in the Y-up asset), which rotates to
    // -Y/+Y here (RotX(+90deg)*(0,0,+-50) == (0,-+50,0)). Each key sets one
    // axis so 1+3 etc. compose (see applyPerturbation).
    setup.keyActions.push_back(KeyAction{'1', "Push forward", [state] {
                                           applyPerturbation(*state, 0, 40);
                                         }});
    setup.keyActions.push_back(KeyAction{'2', "Push backward", [state] {
                                           applyPerturbation(*state, 0, -40);
                                         }});
    setup.keyActions.push_back(KeyAction{'3', "Push right", [state] {
                                           applyPerturbation(*state, 1, -50);
                                         }});
    setup.keyActions.push_back(KeyAction{'4', "Push left", [state] {
                                           applyPerturbation(*state, 1, 50);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'h', "Toggle harness", [world, skel, state] {
                    toggleHarness(world, skel, *state);
                  }});

    setup.renderPanel = [state] {
      ImGui::Text("Harness: %s", state->harnessOn ? "on" : "off");
      ImGui::Text("Perturbation active: %d steps left", state->impulseDuration);
      ImGui::TextWrapped(
          "1/2 push the spine forward/backward, 3/4 push right/left "
          "(40-50 N for 100 steps); 'h' toggles a pelvis weld-joint harness.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
