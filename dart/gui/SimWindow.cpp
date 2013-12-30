/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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

#include "dart/gui/SimWindow.h"

#include <cstdio>
#include <string>

#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/gui/GLFuncs.h"

namespace dart {
namespace gui {

SimWindow::SimWindow()
  : Win3D() {
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

SimWindow::~SimWindow() {
}

void SimWindow::timeStepping() {
  mWorld->step();
}

void SimWindow::drawSkels() {
  for (int i = 0; i < mWorld->getNumSkeletons(); i++)
    mWorld->getSkeleton(i)->draw(mRI);
}

void SimWindow::displayTimer(int _val) {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (mPlay) {
    mPlayFrame += 16;
    if (mPlayFrame >= mBakedStates.size())
      mPlayFrame = 0;
  } else if (mSimulating) {
    for (int i = 0; i < numIter; i++) {
      timeStepping();
      bake();
    }
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void SimWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!mSimulating) {
    if (mPlayFrame < mBakedStates.size()) {
      int nSkels = mWorld->getNumSkeletons();
      for (unsigned int i = 0; i < nSkels; i++) {
        int start = mWorld->getIndex(i);
        int size = mWorld->getSkeleton(i)->getNumGenCoords();
        mWorld->getSkeleton(i)->setConfig(
              mBakedStates[mPlayFrame].segment(start, size));
      }
      if (mShowMarkers) {
        int sumDofs = mWorld->getIndex(nSkels);
        int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
        for (int i = 0; i < nContact; i++) {
          Eigen::Vector3d v =
              mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
          Eigen::Vector3d f =
              mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
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
  } else {
    if (mShowMarkers) {
      collision::CollisionDetector* cd =
          mWorld->getConstraintHandler()->getCollisionDetector();
      for (int k = 0; k < cd->getNumContacts(); k++) {
        Eigen::Vector3d v = cd->getContact(k).point;
        Eigen::Vector3d f = cd->getContact(k).force / 10.0;
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
#ifdef WIN32
    _snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#else
    std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#endif
  else
#ifdef WIN32
    _snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#else
    std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#endif
  std::string frame(buff);
  glColor3f(0.0, 0.0, 0.0);
  gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}

void SimWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating) {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay) {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating) {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if (mPlayFrame >= mBakedStates.size())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void SimWindow::setWorld(simulation::World* _world) {
  mWorld = _world;
}

void SimWindow::bake() {
  collision::CollisionDetector* cd =
      mWorld->getConstraintHandler()->getCollisionDetector();
  int nContacts  = cd->getNumContacts();
  int nSkeletons = mWorld->getNumSkeletons();
  Eigen::VectorXd state(mWorld->getIndex(nSkeletons) + 6 * nContacts);
  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
    state.segment(mWorld->getIndex(i),
                  mWorld->getSkeleton(i)->getNumGenCoords()) =
        mWorld->getSkeleton(i)->get_q();
  }
  for (int i = 0; i < nContacts; i++) {
    int begin = mWorld->getIndex(nSkeletons) + i * 6;
    state.segment(begin, 3)     = cd->getContact(i).point;
    state.segment(begin + 3, 3) = cd->getContact(i).force;
  }
  mBakedStates.push_back(state);
}

}  // namespace gui
}  // namespace dart
