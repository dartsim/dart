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

#include "OperationalSpaceController.hpp"

#include <dart/dynamics/BodyNode.hpp>

#include <stdexcept>
#include <utility>

namespace dart_demos {

//==============================================================================
OperationalSpaceController::OperationalSpaceController(
    dart::dynamics::SkeletonPtr robot, dart::dynamics::SimpleFramePtr target)
  : mRobot(std::move(robot)), mEndEffector(nullptr)
{
  if (!mRobot || mRobot->getNumBodyNodes() == 0 || mRobot->getNumDofs() == 0)
    throw std::invalid_argument(
        "robot must have bodies and degrees of freedom");
  mEndEffector = mRobot->getBodyNode(mRobot->getNumBodyNodes() - 1);
  const auto dofs = mEndEffector->getNumDependentGenCoords();
  if (dofs != mRobot->getNumDofs())
    throw std::invalid_argument("end effector must depend on all robot DOFs");
  mKd = 5.0 * Eigen::MatrixXd::Identity(dofs, dofs);
  setTarget(std::move(target));
}

//==============================================================================
Eigen::VectorXd OperationalSpaceController::computeForces() const
{
  if (mRobot->getNumBodyNodes() == 0
      || mRobot->getBodyNode(mRobot->getNumBodyNodes() - 1) != mEndEffector
      || static_cast<Eigen::Index>(mRobot->getNumDofs()) != mKd.rows()
      || mEndEffector->getNumDependentGenCoords() != mRobot->getNumDofs())
    throw std::runtime_error(
        "robot structure changed after controller construction");
  auto* robot = mRobot.get();
  auto* ee = mEndEffector;

  Eigen::MatrixXd M = robot->getMassMatrix();

  dart::math::LinearJacobian J = ee->getLinearJacobian(mOffset);
  Eigen::MatrixXd pinvJ
      = J.transpose()
        * (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity()).inverse();

  dart::math::LinearJacobian dJ = ee->getLinearJacobianDeriv(mOffset);
  Eigen::MatrixXd pinvDJ
      = dJ.transpose()
        * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
              .inverse();

  Eigen::Vector3d e = mTarget->getWorldTransform().translation()
                      - ee->getWorldTransform() * mOffset;
  Eigen::Vector3d de = -ee->getLinearVelocity(mOffset);
  Eigen::VectorXd Cg = robot->getCoriolisAndGravityForces();

  Eigen::VectorXd forces
      = M * (pinvJ * mKp * de + pinvDJ * mKp * e) + Cg + mKd * pinvJ * mKp * e;
  return forces;
}

//==============================================================================
void OperationalSpaceController::update()
{
  mRobot->setForces(computeForces());
}

//==============================================================================
const dart::dynamics::SkeletonPtr& OperationalSpaceController::getRobot() const
{
  return mRobot;
}

//==============================================================================
const dart::dynamics::SimpleFramePtr& OperationalSpaceController::getTarget()
    const
{
  return mTarget;
}

//==============================================================================
void OperationalSpaceController::setTarget(
    dart::dynamics::SimpleFramePtr target)
{
  if (!target)
    throw std::invalid_argument("target must not be null");
  mTarget = std::move(target);
}

//==============================================================================
const Eigen::Vector3d& OperationalSpaceController::getOffset() const
{
  return mOffset;
}

//==============================================================================
void OperationalSpaceController::setOffset(const Eigen::Vector3d& offset)
{
  if (!offset.allFinite())
    throw std::invalid_argument("offset must be finite");
  mOffset = offset;
}

//==============================================================================
const Eigen::Matrix3d& OperationalSpaceController::getKp() const
{
  return mKp;
}

//==============================================================================
void OperationalSpaceController::setKp(const Eigen::Matrix3d& kp)
{
  if (!kp.allFinite())
    throw std::invalid_argument("kp must be finite");
  mKp = kp;
}

//==============================================================================
const Eigen::MatrixXd& OperationalSpaceController::getKd() const
{
  return mKd;
}

//==============================================================================
void OperationalSpaceController::setKd(const Eigen::MatrixXd& kd)
{
  const auto dofs = static_cast<Eigen::Index>(mRobot->getNumDofs());
  if (kd.rows() != dofs || kd.cols() != dofs || !kd.allFinite())
    throw std::invalid_argument("kd must be a finite robot-DOF square matrix");
  mKd = kd;
}

} // namespace dart_demos
