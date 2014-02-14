/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "apps/harnessTest/MyWindow.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/simulation/World.h"
#include "dart/gui/GLFuncs.h"
#include "dart/constraint/BallJointConstraint.h"
#include "dart/constraint/RevoluteJointConstraint.h"


using namespace dart;
using namespace math;
using namespace dynamics;
using namespace gui;
using namespace dart::constraint;


void MyWindow::timeStepping()
{
    mWorld->getSkeleton(1)->getBodyNode("h_spine")->addExtForce(Eigen::Vector3d(0.0, 0.0, 0.0), mForce);

    mController->setConstrForces(mWorld->getConstraintHandler()->getTotalConstraintForce(1));
    mController->computeTorques(mWorld->getSkeleton(1)->get_q(), mWorld->getSkeleton(1)->get_dq());
    mWorld->getSkeleton(1)->setInternalForceVector(mController->getTorques());

    mWorld->step();

    // for perturbation test
    mImpulseDuration--;
    if (mImpulseDuration <= 0) {
        mImpulseDuration = 0;
        mForce.setZero();
    }    
}

void MyWindow::drawSkels()
{
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

    // draw arrow
    if (mImpulseDuration > 0) {
        Eigen::Vector3d poa = mWorld->getSkeleton(1)->getBodyNode("h_spine")->getWorldTransform() * Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d start = poa - mForce / 10.0;
        double len = mForce.norm() / 10.0;
        drawArrow3D(start, mForce, len, 0.05, 0.1);
    }
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    case '1':
        mForce[0] = 20;
        mImpulseDuration = 100.0;
        std::cout << "push forward" << std::endl;
        break;
    case '2':
        mForce[0] = -10;
        mImpulseDuration = 100.0;
        std::cout << "push backward" << std::endl;
        break;
    case '3':
        mForce[2] = 50;
        mImpulseDuration = 100.0;
        std::cout << "push right" << std::endl;
        break;
    case '4':
        mForce[2] = -50;
        mImpulseDuration = 100.0;
        std::cout << "push left" << std::endl;
        break;

    case 'h':
        if (mHarnessOn) {
            mWorld->getConstraintHandler()->deleteConstraint();
            mHarnessOn = false;
        } else {
            addWeldConstraint();
        }
        break;

    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

void MyWindow::addWeldConstraint() {
    BodyNode *bd = mWorld->getSkeleton(1)->getBodyNode("h_pelvis");
    Eigen::Vector3d offset(0.0, 0.0, 0.0);
    Eigen::Vector3d target = bd->getWorldTransform() * offset;
    BallJointConstraint *bj = new BallJointConstraint(bd, offset, target);
    mWorld->getConstraintHandler()->addConstraint(bj);

    Eigen::Vector3d axis1(0.0, 1.0, 0.0);
    Eigen::Vector3d globalAxis1 = bd->getWorldTransform() * axis1 - target;
    RevoluteJointConstraint *rj = new RevoluteJointConstraint(bd, axis1, globalAxis1);
      mWorld->getConstraintHandler()->addConstraint(rj);

    mHarnessOn = true;
}
