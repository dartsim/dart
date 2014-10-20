/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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

#include "Controller.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/constraint/BallJointConstraint.h"

Controller::Controller(dart::dynamics::Skeleton* _skel, dart::constraint::ConstraintSolver* _constrSolver,
                       double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;
  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);

  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the current pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

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
  mLeadingHand = NULL;
  mTrailingHand = NULL;
  mFootContact = true;
  mLeftHandContact = false;
  mRightHandContact = false;
  mLeftHandGrabbing = false;
  mRightHandGrabbing = false;
  mTimer = 300;
  mAccumulation = 0.0;
  mState = "STAND";
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

void Controller::computeTorques(int _currentFrame) {
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB1") {
    grabOneBar();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else if (mState == "LEAD") {
    lead();
  } else if (mState == "GRAB2") {
    grabTwoBars();
  } else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

void Controller::checkContactState() {
  mFootContact = false;
  mLeftHandContact = false;
  mRightHandContact = false;
  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNode* body1 = cd->getContact(i).bodyNode1;
    dart::dynamics::BodyNode* body2 = cd->getContact(i).bodyNode2;
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_heel_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = true;
    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_heel_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = true;
    if ((body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left")) || (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left")))
      mLeftHandContact = true;
    if ((body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right")) || (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right")))
      mRightHandContact = true;
  }
}

void Controller::stand() {
  // change to standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;
  // switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
  }
}

void Controller::crouch() {
  // change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.7;
  mDesiredDofs[9] = 0.7;
  mDesiredDofs[14] = -1.1;
  mDesiredDofs[15] = -1.1;
  mDesiredDofs[17] = 0.5;
  mDesiredDofs[19] = 0.5;
  mDesiredDofs[13] = -0.2;
  stablePD();
  ankleStrategy();
  mTimer --;
  if (mTimer == 0) {
    mState = "JUMP";
  }
}

void Controller::jump() {
  // change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.3;
  mDesiredDofs[28] = -1.0;
  mDesiredDofs[30] = 0.3;
  mDesiredDofs[31] = 1.0;
  mDesiredDofs[33] = 0.5;
  mDesiredDofs[34] = 0.5;
  stablePD();

  Eigen::Vector3d vf(-1100.0, -2600, 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);
  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (!mFootContact) {
    mState = "REACH";
  }
}

void Controller::reach() {
  // change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.7;
  mDesiredDofs[34] = 0.7;
  stablePD();

  checkContactState();
  if (mFootContact) {
    mState = "JUMP";
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB1";
    mTimer = 20;
  } else {
    mState = "REACH";
  }
}

void Controller::grabOneBar() {  
// create a ball joint constraint
  if (!mLeftHandGrabbing) {
    dart::dynamics::BodyNode *bd1 = mSkel->getBodyNode("h_hand_left");
    Eigen::Vector3d offset(0.0, -0.1, 0.0);
    Eigen::Vector3d jointPos1 = bd1->getTransform() * offset;
    dart::constraint::BallJointConstraint *hold1 = new dart::constraint::BallJointConstraint(bd1, jointPos1);
    mConstraintSolver->addConstraint(hold1);
    bd1->setCollidable(false);
    mLeftHandGrabbing = true;
    mLeadingHand = hold1;
  }

  if (!mRightHandGrabbing) {
    dart::dynamics::BodyNode *bd2 = mSkel->getBodyNode("h_hand_right");
    Eigen::Vector3d offset(0.0, -0.1, 0.0);
    Eigen::Vector3d jointPos2 = bd2->getTransform() * offset;
    dart::constraint::BallJointConstraint *hold2 = new dart::constraint::BallJointConstraint(bd2, jointPos2);
    mConstraintSolver->addConstraint(hold2);
    bd2->setCollidable(false);
    mRightHandGrabbing = true;
    mTrailingHand = hold2;
  }

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.7;
  mDesiredDofs[34] = 0.7;
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mAccumulation = 0.0;
    mState = "SWING";
  }
}  

void Controller::swing() {
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.7;
  mDesiredDofs[34] = 0.7;
  stablePD();
  mAccumulation += fabs(mSkel->getWorldCOMVelocity()[0]);
    
  if (mAccumulation > 40.0) {
    mState = "RELEASE";
  }
}

void Controller::release() {
  // delete ball joint constraint
  if (mTrailingHand->getBodyNode1() == mSkel->getBodyNode("h_hand_left")) {
      mConstraintSolver->removeConstraint(mTrailingHand);
      mLeftHandGrabbing = false;
      mSkel->getBodyNode("h_hand_left")->setCollidable(true);
      mTrailingHand = NULL;
  } else {
      mConstraintSolver->removeConstraint(mTrailingHand);
      mRightHandGrabbing = false;
      mSkel->getBodyNode("h_hand_right")->setCollidable(true);
      mTrailingHand = NULL;
  }
  
  if (mAccumulation > 40.0) {
    mState = "LEAD";
    mTimer = 20;
  } else {
    mState = "TRAIL";
    mTimer = 20;
  }
}

void Controller::lead() {
  mDesiredDofs = mDefaultPose;  
  if (!mLeftHandGrabbing) {
    mDesiredDofs[27] = 0.7;
    mDesiredDofs[28] = -2.3;
    mDesiredDofs[30] = 0.7;
    mDesiredDofs[31] = 2.3;
    mDesiredDofs[33] = 0.0;
    mDesiredDofs[34] = 0.7;
  } else {
    mDesiredDofs[27] = 0.7;
    mDesiredDofs[28] = -2.3;
    mDesiredDofs[30] = 0.7;
    mDesiredDofs[31] = 2.3;
    mDesiredDofs[33] = 0.7;
    mDesiredDofs[34] = 0.0;
  }
  stablePD();
  mTimer--;
  if (mTimer <= 0) {
    checkContactState();
    if (mRightHandContact || mLeftHandContact) {
      mState = "GRAB2";
      mTimer = 100;
      mAccumulation = 0.0;
    }
  }
}

void Controller::grabTwoBars() {
  // create ball joints on trailing hand
  if (!mLeftHandGrabbing) {
    dart::dynamics::BodyNode *bd1 = mSkel->getBodyNode("h_hand_left");
    Eigen::Vector3d offset(0.0, -0.1, 0.0);
    Eigen::Vector3d jointPos1 = bd1->getTransform() * offset;
    dart::constraint::BallJointConstraint *hold1 = new dart::constraint::BallJointConstraint(bd1, jointPos1);
    mConstraintSolver->addConstraint(hold1);
    bd1->setCollidable(false);
    mLeftHandGrabbing = true;
    mTrailingHand = mLeadingHand;
    mLeadingHand = hold1;
  }

  if (!mRightHandGrabbing) {
    dart::dynamics::BodyNode *bd2 = mSkel->getBodyNode("h_hand_right");
    Eigen::Vector3d offset(0.0, -0.1, 0.0);
    Eigen::Vector3d jointPos2 = bd2->getTransform() * offset;
    dart::constraint::BallJointConstraint *hold2 = new dart::constraint::BallJointConstraint(bd2, jointPos2);
    mConstraintSolver->addConstraint(hold2);
    bd2->setCollidable(false);
    mRightHandGrabbing = true;
    mTrailingHand = mLeadingHand;
    mLeadingHand = hold2;
  }
  
  mAccumulation += fabs(mSkel->getWorldCOMVelocity()[0]);
  mTimer--;
  if (mTimer == 0) {
    mState = "RELEASE";
  }
}

// void Controller:trail() {
//   // create pose by IK

//   checkContactState();
//   if (mRightHandContact || mLeftHandContact) {
//     mState = "GRAB1";
//     mTimer = 20;
//   }

  
void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getWorldCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
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
}

void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getJacobian(_bodyNode, _offset);
  mTorques += jacobian.transpose() * _force;
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

// 0-5 pelvis
// 6-8 left hip
// 9-11 right hip
// 12,13 abdomen
// 14 left knee
// 15 right knee
// 16 spine
// 17,18 left ankle
// 19,20 right ankle
// 21,22 head
// 23 left scapula
// 24 right scapula
// 25 left toe
// 26 right toe
// 27-29 left shoulder
// 30-32 right shoulder
// 33 left elbow
// 34 right elbow
// 35 left wrist
// 36 right wrist
