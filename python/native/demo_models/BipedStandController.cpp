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

#include "BipedStandController.hpp"

#include <dart/dart.hpp>

#include <stdexcept>

namespace dart_demos {

using dart::dynamics::SkeletonPtr;

namespace {
const char* const kBipedStandLeftHeelName = "h_heel_left";
} // namespace

//==============================================================================
BipedStandController makeBipedStandController(
    const SkeletonPtr& skel, double timestep)
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

  controller.leftHeel = skel->getBodyNode(kBipedStandLeftHeelName);
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

  // World X is invariant under the scene's Y-up to Z-up reorientation.
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

} // namespace dart_demos
