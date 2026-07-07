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

// Ported from examples/vehicle: a car skeleton (free root + steering +
// wheel-spin dofs) driven by a position servo on the steering dofs and a
// velocity servo on the wheel-spin dofs.
//
// Deviations from the original: vehicle.skel is authored Y-up (gravity
// (0,-9.81,0), camera up (0,1,0)) and, unlike every other B4/B1-B3 scene, is
// deliberately NOT reoriented to Z-up here. The vehicle's DOF layout (car
// dofs 6-11) is hardcoded to vehicle.skel's own joint ordering, which is
// tied to the asset's authored (Y-up) root-joint axes; ZUp.hpp only rotates
// each skeleton's root-joint transform-from-parent and gravity; it does not
// touch which local joint axis is "steering" vs "wheel-spin" for a
// multi-dof root FreeJoint, so reorienting here would risk silently
// scrambling which dof each hardcoded index (6, 7, 8, ...) refers to. This
// mirrors parity-b4.json's own guidance to treat this as "a parity gap to
// document rather than replicate" (the same call already made for this
// scene during Phase 0 recon) rather than risk an unverified behavior
// change. The original's fixed 640x480 window (it does not parse
// --gui-scale, unlike every other example) and its own instruction-text
// block are host chrome now.

#include "Scenes.hpp"

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

//==============================================================================
/// Per-instance state captured by this scene's preStep/key-action lambdas.
struct VehicleState
{
  dart::dynamics::SkeletonPtr vehicle;

  double backWheelVelocity = 0.0;
  double steeringWheelAngle = 0.0;

  const double k = 0.01;
  const double d = 0.005;
};

} // namespace

//==============================================================================
DemoScene makeVehicleScene()
{
  DemoScene scene;
  scene.id = "vehicle";
  scene.title = "Vehicle";
  scene.category = "Robots";
  scene.summary = "Drive a steerable car past obstacles.";

  scene.factory = [] {
    auto world
        = dart::utils::SkelParser::readWorld("dart://sample/skel/vehicle.skel");
    if (!world)
      throw std::runtime_error(
          "failed to load dart://sample/skel/vehicle.skel");
    // Y-up, deliberately not reoriented -- see the file comment.
    world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

    auto vehicle = world->getSkeleton("car_skeleton");
    if (!vehicle)
      throw std::runtime_error("vehicle.skel: missing skeleton 'car_skeleton'");
    if (vehicle->getNumDofs() < 12)
      throw std::runtime_error(
          "vehicle.skel: expected >= 12 dofs (car_skeleton layout)");

    auto state = std::make_shared<VehicleState>();
    state->vehicle = vehicle;

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.0, 3.0, 3.0),
        ::osg::Vec3d(0.0, 0.0, 0.0),
        ::osg::Vec3d(0.0, 1.0, 0.0)};

    setup.preStep = [state] {
      auto& vehicle = state->vehicle;
      const Eigen::VectorXd q = vehicle->getPositions();
      const Eigen::VectorXd dq = vehicle->getVelocities();
      Eigen::VectorXd tau = Eigen::VectorXd::Zero(vehicle->getNumDofs());

      tau[6]
          = -state->k * (q[6] - state->steeringWheelAngle) - state->d * dq[6];
      tau[8]
          = -state->k * (q[8] - state->steeringWheelAngle) - state->d * dq[8];
      tau[7] = -state->d * (dq[7] - state->backWheelVelocity);
      tau[9] = -state->d * (dq[9] - state->backWheelVelocity);
      tau[10] = -state->d * (dq[10] - state->backWheelVelocity);
      tau[11] = -state->d * (dq[11] - state->backWheelVelocity);

      vehicle->setForces(tau);
    };

    using dart::math::constantsd;
    const double degToRad = constantsd::pi() / 180.0;

    setup.keyActions.push_back(
        KeyAction{'w', "Move forward", [state, degToRad] {
                    state->backWheelVelocity = -420.0 * degToRad;
                  }});
    setup.keyActions.push_back(KeyAction{'s', "Stop", [state] {
                                           state->backWheelVelocity = 0.0;
                                         }});
    setup.keyActions.push_back(
        KeyAction{'x', "Move backward", [state, degToRad] {
                    state->backWheelVelocity = +420.0 * degToRad;
                  }});
    setup.keyActions.push_back(
        KeyAction{'a', "Steer left", [state, degToRad] {
                    state->steeringWheelAngle += 10.0 * degToRad;
                    if (state->steeringWheelAngle > 30.0 * degToRad)
                      state->steeringWheelAngle = 30.0 * degToRad;
                  }});
    setup.keyActions.push_back(
        KeyAction{'d', "Steer right", [state, degToRad] {
                    state->steeringWheelAngle -= 10.0 * degToRad;
                    if (state->steeringWheelAngle < -30.0 * degToRad)
                      state->steeringWheelAngle = -30.0 * degToRad;
                  }});

    setup.renderPanel = [state, degToRad] {
      ImGui::Text(
          "Steering angle: %.1f deg", state->steeringWheelAngle / degToRad);
      ImGui::Text(
          "Back wheel target velocity: %.1f deg/s",
          state->backWheelVelocity / degToRad);
      ImGui::TextWrapped(
          "w/s/x drive the back wheels forward/stop/backward; a/d steer "
          "left/right in 10-degree steps (clamped to +-30 deg). This scene "
          "keeps vehicle.skel's original Y-up orientation (see file "
          "comment) rather than the host's usual Z-up convention.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
