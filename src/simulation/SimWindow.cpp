/* Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include <cstdio>

#include "SimWindow.h"
#include "simulation/World.h"
#include "dynamics/Skeleton.h"
#include "constraint/ConstraintDynamics.h"
#include "collision/CollisionDetector.h"
#include "yui/GLFuncs.h"

namespace dart {
namespace simulation {

SimWindow::SimWindow()
    : Win3D()
{
    mBackground[0] = 1.0;
    mBackground[1] = 1.0;
    mBackground[2] = 1.0;
    mBackground[3] = 1.0;

    mPlay = false;
    mSimulating = false;
    mPlayFrame = 0;
    mShowMarkers = true;
    mPersp = 45.f;
    mTrans[1] = 300.f;
}

SimWindow::~SimWindow()
{
}

void SimWindow::drawSkels()
{
    for (int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);
}

void SimWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mPlay) {
        mPlayFrame += 16;
        if (mPlayFrame >= mBakedStates.size())
            mPlayFrame = 0;
    }else if (mSimulating) {
        for (int i = 0; i < numIter; i++) {
            timeStepping();
            bake();
        }
    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void SimWindow::draw()
{
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    if (!mSimulating) {
        if (mPlayFrame < mBakedStates.size()) {
            int nSkels = mWorld->getNumSkeletons();
            for (unsigned int i = 0; i < nSkels; i++) {
                int start = mWorld->getIndex(i);
                //int size = mWorld->getDofs(i).size();
                int size = mWorld->getSkeleton(i)->getNumGenCoords();
                mWorld->getSkeleton(i)->setConfig(mBakedStates[mPlayFrame].segment(start, size), true, false);
            }
            if (mShowMarkers) {
                int sumDofs = mWorld->getIndex(nSkels);
                int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
                for (int i = 0; i < nContact; i++) {
                    Eigen::Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
                    Eigen::Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
                    glBegin(GL_LINES);
                    glVertex3f(v[0], v[1], v[2]);
                    glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                    glEnd();
                    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
                    mRI->pushMatrix();
                    glTranslated(v[0], v[1], v[2]);
                    mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
                    mRI->popMatrix();
                }
            }
        }
    }else{
        if (mShowMarkers) {
            for (int k = 0; k < mWorld->getConstraintHandler()->getCollisionDetector()->getNumContacts(); k++) {
                Eigen::Vector3d v = mWorld->getConstraintHandler()->getCollisionDetector()->getContact(k).point;
                Eigen::Vector3d f = mWorld->getConstraintHandler()->getCollisionDetector()->getContact(k).force / 10.0;
                glBegin(GL_LINES);
                glVertex3f(v[0], v[1], v[2]);
                glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                glEnd();
                mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
                mRI->pushMatrix();
                glTranslated(v[0], v[1], v[2]);
                mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
                mRI->popMatrix();
            }
        }
    }
    drawSkels();

    // display the frame count in 2D text
    char buff[64];
    if (!mSimulating)
        sprintf(buff, "%d", mPlayFrame);
    else
        sprintf(buff, "%d", mWorld->getSimFrames());
    std::string frame(buff);
    glColor3f(0.0, 0.0, 0.0);
    yui::drawStringOnScreen(0.02f, 0.02f, frame);
    glEnable(GL_LIGHTING);
}

void SimWindow::keyboard(unsigned char key, int x, int y)
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
        default:
            Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void SimWindow::bake()
{
    int nContact = mWorld->getConstraintHandler()->getCollisionDetector()->getNumContacts();
    Eigen::VectorXd state(mWorld->getIndex(mWorld->getNumSkeletons()) + 6 * nContact);
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        state.segment(mWorld->getIndex(i), mWorld->getSkeleton(i)->getNumGenCoords()) = mWorld->getSkeleton(i)->get_q();
    for (int i = 0; i < nContact; i++) {
        int begin = mWorld->getIndex(mWorld->getNumSkeletons()) + i * 6;
        state.segment(begin, 3) = mWorld->getConstraintHandler()->getCollisionDetector()->getContact(i).point;
        state.segment(begin + 3, 3) = mWorld->getConstraintHandler()->getCollisionDetector()->getContact(i).force;
    }
    mBakedStates.push_back(state);
}

} // namespace simulation
} // namespace dart
