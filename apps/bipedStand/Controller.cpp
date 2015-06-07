/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
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

#include "apps/bipedStand/Controller.h"
#include "dart/dynamics/Group.h"

void createIndexing(std::vector<size_t>&)
{
  // Do nothing
}

template <typename ...Args>
void createIndexing(std::vector<size_t>& _indexing, size_t _name,
                    Args... args)
{
  _indexing.push_back(_name);
  createIndexing(_indexing, args...);
}

static std::vector<size_t> createIndexing()
{
  // This app was made with dof indices hardcoded, but some internal DART
  // development have changed the underlying indexing for the robot, so this
  // map converts the old indexing into the new indexing as a temporary (or
  // maybe permanent) workaround.

  std::vector<size_t> indexing;
  //                        0   1   2   3   4   5   6
  createIndexing(indexing,  0,  1,  2,  3,  4,  5,  6,
  //                        7   8   9  10  11  12  13
                            7,  8, 13, 14, 15, 20, 21,
  //                       14  15  16  17  18  19  20
                            9, 16, 22, 10, 11, 17, 18,
  //                       21  22  23  24  25  26  27
                           23, 24, 25, 31, 12, 19, 26,
  //                       28  29  30  31  32  33  34
                           27, 28, 32, 33, 34, 29, 35,
  //                       35  36
                           30, 36);
  return indexing;
}

static std::vector<dart::dynamics::DegreeOfFreedom*> getDofs(
    const dart::dynamics::SkeletonPtr& _skel)
{
  std::vector<dart::dynamics::DegreeOfFreedom*> dofs;
  const std::vector<size_t>& indexing = createIndexing();
  dofs.reserve(indexing.size());

  for(size_t index : indexing)
    dofs.push_back(_skel->getDof(index));

  return dofs;
}

Controller::Controller(dart::dynamics::SkeletonPtr _skel,
                       double _t) {
  mSkel = dart::dynamics::Group::create("Group", getDofs(_skel));
  mLeftHeel = _skel->getBodyNode("h_heel_left");
  mTimestep = _t;
  mFrame = 0;
  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);

  mTorques.resize(nDof);
  mTorques.setZero();

  resetDesiredDofs();

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

void Controller::resetDesiredDofs() {
  mDesiredDofs = mSkel->getPositions();
}

void Controller::computeTorques() {
  Eigen::VectorXd _dof = mSkel->getPositions();
  Eigen::VectorXd _dofVel = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();

  // SPD tracking
  //size_t nDof = mSkel->getNumDofs();
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
  Eigen::Vector2d diff(com[0] - cop[0], com[2] - cop[2]);
  double offset = com[0] - cop[0];
  if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
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
