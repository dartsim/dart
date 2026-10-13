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

#include "JointConstraintsController.hpp"

#include <dart/dynamics/BodyNode.hpp>

#include <Eigen/LU>

#include <algorithm>
#include <stdexcept>

#include <cmath>

namespace dart_demos {

//==============================================================================
JointConstraintsController::JointConstraintsController(
    const dart::dynamics::SkeletonPtr& skel, double timestep)
  : timestep(timestep)
{
  if (!skel || skel->getNumDofs() == 0)
    throw std::invalid_argument("skeleton must have at least one DOF");
  if (!std::isfinite(timestep) || timestep <= 0.0)
    throw std::invalid_argument("timestep must be finite and positive");
  const int nDof = static_cast<int>(skel->getNumDofs());
  kp = Eigen::MatrixXd::Zero(nDof, nDof);
  kd = Eigen::MatrixXd::Zero(nDof, nDof);
  constrForces = Eigen::VectorXd::Zero(nDof);
  desiredDofs = skel->getPositions();

  // Root 6 dofs keep zero gains (uncontrolled); lower body + lower back get
  // stiffer gains than the upper body, exactly as the original tuned them.
  for (int i = 6; i < std::min(22, nDof); ++i)
    kp(i, i) = 200.0;
  for (int i = 22; i < nDof; ++i)
    kp(i, i) = 20.0;
  for (int i = 6; i < std::min(22, nDof); ++i)
    kd(i, i) = 100.0;
  for (int i = 22; i < nDof; ++i)
    kd(i, i) = 10.0;
}

//==============================================================================
Eigen::VectorXd JointConstraintsController::computeTorques(
    const dart::dynamics::SkeletonPtr& skel)
{
  if (!skel
      || static_cast<Eigen::Index>(skel->getNumDofs()) != desiredDofs.size())
    throw std::invalid_argument("skeleton DOFs must match the initial pose");
  if (!std::isfinite(timestep) || timestep <= 0.0)
    throw std::invalid_argument("timestep must be finite and positive");
  const Eigen::VectorXd q = skel->getPositions();
  const Eigen::VectorXd dq = skel->getVelocities();
  const double dt = timestep;

  const Eigen::MatrixXd invM = (skel->getMassMatrix() + kd * dt).inverse();
  const Eigen::VectorXd p = -kp * (q + dq * dt - desiredDofs);
  const Eigen::VectorXd d = -kd * dq;
  const Eigen::VectorXd qddot
      = invM * (-skel->getCoriolisAndGravityForces() + p + d + constrForces);
  Eigen::VectorXd torques = p + d - kd * qddot * dt;

  // World X is invariant under the scene's Y-up to Z-up reorientation.
  auto* heel = skel->getBodyNode("h_heel_left");
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
      torques[17] += -k1 * offset + kdAnkle * (preOffset - offset);
      torques[25] += -k2 * offset + kdAnkle * (preOffset - offset);
      torques[19] += -k1 * offset + kdAnkle * (preOffset - offset);
      torques[26] += -k2 * offset + kdAnkle * (preOffset - offset);
      preOffset = offset;
    }
  }

  // Illegal (uncontrolled) root torques are always zeroed, as in the original.
  for (int i = 0; i < std::min(6, static_cast<int>(torques.size())); ++i)
    torques[i] = 0.0;

  return torques;
}

} // namespace dart_demos
