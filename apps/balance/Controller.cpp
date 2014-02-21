/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "apps/balance/Controller.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Shape.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/collision/CollisionDetector.h"

Controller::Controller(dart::dynamics::Skeleton* _skel,
                       dart::constraint::ConstraintDynamics* _collisionHandle,
                       double _t) {
  mSkel = _skel;
  mCollisionHandle = _collisionHandle;
  mTimestep = _t;
  mFrame = 0;
  int nDof = mSkel->getNumGenCoords();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mConstrForces = Eigen::VectorXd::Zero(nDof);

  mTorques.resize(nDof);
  mDesiredDofs.resize(nDof);
  for (int i = 0; i < nDof; i++) {
    mTorques[i] = 0.0;
    mDesiredDofs[i] = mSkel->getGenCoord(i)->get_q();
  }

  // using SPD results in simple Kp coefficients
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }
  for (int i = 6; i < 22; i++)
    mKp(i, i) = 200.0;  // lower body + lower back
  for (int i = 22; i < nDof; i++)
    mKp(i, i) = 20.0;
  for (int i = 6; i < 22; i++)
    mKd(i, i) = 100.0;
  for (int i = 22; i < nDof; i++)
    mKd(i, i) = 10.0;

  mPreOffset = 0.0;
}

Controller::~Controller() {
}

Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

double Controller::getTorque(int _index) {
  return mTorques[_index];
}

void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

void Controller::computeTorques(const Eigen::VectorXd& _dof,
                                const Eigen::VectorXd& _dofVel) {
  // SPD tracking
  int nDof = mSkel->getNumGenCoords();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * _dofVel;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
  mTorques = p + d - mKd * qddot * mTimestep;

  // ankle strategy for sagital plane
  Eigen::Vector3d com = mSkel->getWorldCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getWorldTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  Eigen::Vector2d diff(com[0] - cop[0], com[2] - cop[2]);
  if (diff[0] < 0.1) {
    double offset = com[0] - cop[0];
    double k1 = 20.0;
    double k2 = 10.0;
    double kd = 100.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }

  // Just to make sure no illegal torque is used
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
  mFrame++;
}

dart::dynamics::Skeleton*Controller::getSkel() {
  return mSkel;
}

Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

Eigen::MatrixXd Controller::getKd() {
  return mKd;
}

void Controller::setConstrForces(const Eigen::VectorXd& _constrForce) {
  mConstrForces = _constrForce;
}

