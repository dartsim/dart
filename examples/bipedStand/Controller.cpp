/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "Controller.hpp"

Controller::Controller(dart::dynamics::SkeletonPtr _skel,
                       double _t) {
  mSkel = _skel;
  mLeftHeel = _skel->getBodyNode("h_heel_left");

  mLeftFoot[0] = _skel->getDof("j_heel_left_1")->getIndexInSkeleton();
  mLeftFoot[1] = _skel->getDof("j_toe_left")->getIndexInSkeleton();

  mRightFoot[0] = _skel->getDof("j_heel_right_1")->getIndexInSkeleton();
  mRightFoot[1] = _skel->getDof("j_toe_right")->getIndexInSkeleton();

  mTimestep = _t;
  mFrame = 0;
  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);

  mTorques.resize(nDof);
  mTorques.setZero();

  mDesiredDofs = mSkel->getPositions();

  // using SPD results in simple Kp coefficients
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }
  for (int i = 6; i < nDof; i++)
    mKp(i, i) = 400.0;
  for (int i = 6; i < nDof; i++)
    mKd(i, i) = 40.0;

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

void Controller::computeTorques() {
  Eigen::VectorXd _dof = mSkel->getPositions();
  Eigen::VectorXd _dofVel = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();

  // SPD tracking
  //std::size_t nDof = mSkel->getNumDofs();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * _dofVel;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques = p + d - mKd * qddot * mTimestep;

  // ankle strategy for sagital plane
  Eigen::Vector3d com = mSkel->getCOM();
  Eigen::Vector3d cop = mLeftHeel->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);

  double offset = com[0] - cop[0];
  if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[mLeftFoot[0]]  += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mLeftFoot[1]]  += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mRightFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mRightFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[mLeftFoot[0]]  += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mLeftFoot[1]]  += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mRightFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mRightFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }

  // Just to make sure no illegal torque is used
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
  mFrame++;
}

dart::dynamics::MetaSkeletonPtr Controller::getSkel() {
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
