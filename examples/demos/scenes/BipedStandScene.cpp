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

// Ported from examples/biped_stand: fullbody1.skel balanced by an SPD
// tracking controller (holding its initial standing pose) plus a
// sagittal-plane ankle strategy, with four scripted push perturbations
// (1-4) on the spine.
//
// Deviations from the original: fullbody1.skel is authored Y-up; reoriented
// to the host's Z-up convention via ZUp.hpp, the same treatment as every
// other fullbody1.skel-based B3 scene (see JointConstraintsScene.cpp's file
// comment for why the SPD gains, named-DOF lookups, and the ankle strategy's
// use of com[0]/cop[0] are unaffected by the reorientation, while the
// perturbation forces' horizontal axes are rotated the same way the geometry
// is: (fx,fy,fz) -> (fx,-fz,fy)). The desired pose is captured (as
// mDesiredDofs was in the original) immediately after the initial DOF poses
// are set and before any controller runs, so the controller tracks the
// intended standing pose exactly as the original's CustomWorldNode
// constructor did.

#include "Scenes.hpp"
#include "ZUp.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::SkeletonPtr;

const char* const kSpineName = "h_spine";
const char* const kLeftHeelName = "h_heel_left";

//==============================================================================
/// SPD tracking + ankle-strategy controller state, ported from
/// examples/biped_stand's CustomWorldNode.
struct BipedStandController
{
  Eigen::MatrixXd kp;
  Eigen::MatrixXd kd;
  Eigen::VectorXd desiredDofs;
  double timestep = 0.001;

  BodyNode* leftHeel = nullptr;
  std::size_t leftHeelDof = 0;
  std::size_t leftToeDof = 0;
  std::size_t rightHeelDof = 0;
  std::size_t rightToeDof = 0;

  double preOffset = 0.0;
};

//==============================================================================
BipedStandController makeController(const SkeletonPtr& skel, double timestep)
{
  BipedStandController controller;
  const int nDof = static_cast<int>(skel->getNumDofs());
  controller.kp = Eigen::MatrixXd::Identity(nDof, nDof);
  controller.kd = Eigen::MatrixXd::Identity(nDof, nDof);
  for (int i = 0; i < 6 && i < nDof; ++i) {
    controller.kp(i, i) = 0.0;
    controller.kd(i, i) = 0.0;
  }
  for (int i = 6; i < nDof; ++i)
    controller.kp(i, i) = 400.0;
  for (int i = 6; i < nDof; ++i)
    controller.kd(i, i) = 40.0;

  controller.timestep = timestep;
  controller.desiredDofs = skel->getPositions();

  controller.leftHeel = skel->getBodyNode(kLeftHeelName);
  auto dofIndex = [&](const char* name) -> std::size_t {
    auto* dof = skel->getDof(name);
    if (!dof)
      throw std::runtime_error(
          std::string("fullbody1.skel: missing DOF ") + name);
    return dof->getIndexInSkeleton();
  };
  controller.leftHeelDof = dofIndex("j_heel_left_1");
  controller.leftToeDof = dofIndex("j_toe_left");
  controller.rightHeelDof = dofIndex("j_heel_right_1");
  controller.rightToeDof = dofIndex("j_toe_right");

  return controller;
}

//==============================================================================
Eigen::VectorXd computeTorques(
    const SkeletonPtr& skel, BipedStandController& controller)
{
  const Eigen::VectorXd q = skel->getPositions();
  const Eigen::VectorXd dq = skel->getVelocities();
  const Eigen::VectorXd constrForces = skel->getConstraintForces();
  const double dt = controller.timestep;

  const Eigen::MatrixXd invM
      = (skel->getMassMatrix() + controller.kd * dt).inverse();
  const Eigen::VectorXd p
      = -controller.kp * (q + dq * dt - controller.desiredDofs);
  const Eigen::VectorXd d = -controller.kd * dq;
  const Eigen::VectorXd qddot
      = invM * (-skel->getCoriolisAndGravityForces() + p + d + constrForces);

  Eigen::VectorXd torques = p + d - controller.kd * qddot * dt;

  // Ankle strategy for the sagittal plane: com[0]/cop[0] are world-frame X
  // coordinates, invariant under the Y-up -> Z-up reorientation (see the file
  // comment), so this logic is byte-for-byte identical to the original.
  if (controller.leftHeel) {
    const Eigen::Vector3d com = skel->getCOM();
    const Eigen::Vector3d cop
        = controller.leftHeel->getTransform() * Eigen::Vector3d(0.05, 0, 0);
    const double offset = com[0] - cop[0];

    if (offset < 0.1 && offset > 0.0) {
      constexpr double k1 = 200.0;
      constexpr double k2 = 100.0;
      constexpr double kdAnkle = 10.0;
      const double delta = kdAnkle * (controller.preOffset - offset);
      torques[controller.leftHeelDof] += -k1 * offset + delta;
      torques[controller.leftToeDof] += -k2 * offset + delta;
      torques[controller.rightHeelDof] += -k1 * offset + delta;
      torques[controller.rightToeDof] += -k2 * offset + delta;
      controller.preOffset = offset;
    } else if (offset > -0.2 && offset < -0.05) {
      constexpr double k1 = 2000.0;
      constexpr double k2 = 100.0;
      constexpr double kdAnkle = 100.0;
      const double delta = kdAnkle * (controller.preOffset - offset);
      torques[controller.leftHeelDof] += -k1 * offset + delta;
      torques[controller.leftToeDof] += -k2 * offset + delta;
      torques[controller.rightHeelDof] += -k1 * offset + delta;
      torques[controller.rightToeDof] += -k2 * offset + delta;
      controller.preOffset = offset;
    }
  }

  for (int i = 0; i < 6 && i < static_cast<int>(torques.size()); ++i)
    torques[i] = 0.0;

  return torques;
}

//==============================================================================
/// Per-instance state captured by this scene's preStep/key-action lambdas.
struct BipedStandState
{
  BipedStandController controller;
  Eigen::Vector3d perturbationForce = Eigen::Vector3d::Zero();
  int impulseDuration = 0;
};

//==============================================================================
void perturb(BipedStandState& state, const Eigen::Vector3d& force, int frames)
{
  state.perturbationForce = force;
  state.impulseDuration = frames;
}

} // namespace

//==============================================================================
DemoScene makeBipedStandScene()
{
  DemoScene scene;
  scene.id = "biped_stand";
  scene.title = "Biped Stand";
  scene.category = "Control & IK";
  scene.summary
      = "SPD-balanced standing biped with an ankle strategy and push tests.";

  scene.factory = [] {
    auto world = dart::utils::SkelParser::readWorld(
        "dart://sample/skel/fullbody1.skel");
    if (!world)
      throw std::runtime_error(
          "failed to load dart://sample/skel/fullbody1.skel");
    reorientWorldToZUp(world);

    auto skel = world->getSkeleton("fullbody1");
    if (!skel)
      throw std::runtime_error("fullbody1.skel: missing skeleton 'fullbody1'");

    const auto setDofPosition = [&](const char* name, double position) {
      auto* dof = skel->getDof(name);
      if (!dof)
        throw std::runtime_error(
            std::string("fullbody1.skel: missing DOF ") + name);
      dof->setPosition(position);
    };
    setDofPosition("j_pelvis_rot_y", -0.20);
    setDofPosition("j_thigh_left_z", 0.15);
    setDofPosition("j_shin_left", -0.40);
    setDofPosition("j_heel_left_1", 0.25);
    setDofPosition("j_thigh_right_z", 0.15);
    setDofPosition("j_shin_right", -0.40);
    setDofPosition("j_heel_right_1", 0.25);
    setDofPosition("j_abdomen_2", 0.00);

    auto state = std::make_shared<BipedStandState>();
    // Built after the initial pose is set, so the desired (tracked) pose
    // matches the standing configuration above -- see the file comment.
    state->controller = makeController(skel, world->getTimeStep());

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(3.0, -3.0, 1.5),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [world, skel, state] {
      // Re-read the world's actual timestep every step: the host's Timestep
      // toolbar can change dt at runtime without rebuilding the scene, and a
      // stale dt would make SPD's implicit-damping terms wrong for the real
      // step (see JointConstraintsScene.cpp).
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

    // Push axes in the reoriented (Z-up) world: X is invariant under the
    // reorientation (forward/backward); the original's Z-axis push (a
    // horizontal axis in the Y-up asset) rotates to -Y/+Y here
    // (RotX(+90deg)*(0,0,+-50) == (0,-+50,0)) -- see JointConstraintsScene.cpp.
    setup.keyActions.push_back(
        KeyAction{'1', "Push forward (+x, 50 N)", [state] {
                    perturb(*state, Eigen::Vector3d(50, 0, 0), 100);
                  }});
    setup.keyActions.push_back(
        KeyAction{'2', "Push backward (-x, 50 N)", [state] {
                    perturb(*state, Eigen::Vector3d(-50, 0, 0), 100);
                  }});
    setup.keyActions.push_back(
        KeyAction{'3', "Push right (-y, 50 N)", [state] {
                    perturb(*state, Eigen::Vector3d(0, -50, 0), 100);
                  }});
    setup.keyActions.push_back(
        KeyAction{'4', "Push left (+y, 50 N)", [state] {
                    perturb(*state, Eigen::Vector3d(0, 50, 0), 100);
                  }});

    setup.renderPanel = [state] {
      ImGui::Text("Perturbation active: %d steps left", state->impulseDuration);
      ImGui::TextWrapped(
          "SPD tracking (Kp=400, Kd=40) plus a sagittal-plane ankle "
          "strategy hold the standing pose. 1-4 push the spine "
          "forward/backward/right/left with 50 N for 100 steps.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
