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

#include "apps/ballJointConstraintTest/MyWindow.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/simulation/World.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/constraint/BallJointConstraint.h"

using namespace dart::dynamics;
using namespace dart::constraint;

void MyWindow::timeStepping()
{
    //    Eigen::VectorXd damping = computeDamping();
    //mWorld->getSkeleton(0)->setInternalForces(damping);
    mWorld->step();
}

Eigen::VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumGenCoords();
    Eigen::VectorXd damping = Eigen::VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(0)->get_dq();
    for (int i = 0; i < nDof; i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
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

    case 'h':
        if (mHeadConstraint) {
            mWorld->getConstraintHandler()->deleteConstraint(mHeadConstraint);
            mHeadConstraint = NULL;
        } else {
            mHeadConstraint = addHeadConstraint();
        }
        break;

    case 't':
        if (mTailConstraint) {
            mWorld->getConstraintHandler()->deleteConstraint(mTailConstraint);
            mTailConstraint = NULL;
        } else {
            mTailConstraint = addTailConstraint();
        }
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

Constraint* MyWindow::addHeadConstraint() {
    BodyNode *bd = mWorld->getSkeleton(0)->getBodyNode("link 1");
    Eigen::Vector3d offset(0.0, 0.025, 0.0);
    Eigen::Vector3d target = bd->getWorldTransform() * offset;
    BallJointConstraint *bj = new BallJointConstraint(bd, offset, target);
    mWorld->getConstraintHandler()->addConstraint(bj);
    return bj;
}

Constraint* MyWindow::addTailConstraint() {
    BodyNode *bd = mWorld->getSkeleton(0)->getBodyNode("link 10");
    Eigen::Vector3d offset(0.0, -0.025, 0.0);
    Eigen::Vector3d target = bd->getWorldTransform() * offset;
    BallJointConstraint *bj = new BallJointConstraint(bd, offset, target);
    mWorld->getConstraintHandler()->addConstraint(bj);
    return bj;
}
