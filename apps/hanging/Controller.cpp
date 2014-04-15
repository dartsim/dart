/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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

#include "Controller.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Shape.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace dynamics;
using namespace math;

Controller::Controller(dynamics::Skeleton* _skel,
                       dart::constraint::ConstraintSolver* _constraintSolver,
                       double _t) {
  mSkel = _skel;
  mConstraintSolver = _constraintSolver;
  mTimestep = _t;
  mFrame = 0;
  int nDof = mSkel->getNumGenCoords();
  mKp = MatrixXd::Identity(nDof, nDof);
  mKd = MatrixXd::Identity(nDof, nDof);
  mConstrForces = VectorXd::Zero(nDof);

  mTorques.resize(nDof);
  mDesiredDofs.resize(nDof);
  for (int i = 0; i < nDof; i++){
    mTorques[i] = 0.0;
    mDesiredDofs[i] = mSkel->getGenCoord(i)->getPos();
  }

  // using SPD results in simple Kp coefficients
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }
  for (int i = 6; i < 22; i++)
    mKp(i, i) = 200.0; // lower body + lower back
  for (int i = 22; i < nDof; i++)
    mKp(i, i) = 20.0;
  for (int i = 6; i < 22; i++)
    mKd(i, i) = 100.0;
  for (int i = 22; i < nDof; i++)
    mKd(i, i) = 10.0;

  mPreOffset = 0.0;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
  // SPD tracking
  int nDof = mSkel->getNumGenCoords();
  MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
  VectorXd d = -mKd * _dofVel;
  VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
  mTorques = p + d - mKd * qddot * mTimestep;

  for (int i = 0; i < 6; i++){
    mTorques[i] = 0.0;
  }
  mFrame++;
}
