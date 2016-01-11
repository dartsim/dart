/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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
#include <iostream>
#include <string>

#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/gui/LoadGlut.h"
#include "dart/gui/GLFuncs.h"
#include "dart/utils/FileInfoWorld.h"
#include "dart/gui/GraphWindow.h"

namespace kido {
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
  for (const auto& graphWindow : mGraphWindows)
    delete graphWindow;
}

void SimWindow::timeStepping() {
  mWorld->step();
}

//==============================================================================
void SimWindow::drawSkels()
{
  for (size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
  {
    mWorld->getSkeleton(i)->draw(mRI);

    if (mShowMarkers)
      mWorld->getSkeleton(i)->drawMarkers(mRI);
  }
}

void SimWindow::drawEntities()
{
  for (size_t i = 0; i < mWorld->getNumSimpleFrames(); ++i)
    mWorld->getSimpleFrame(i)->draw(mRI);
}

void SimWindow::displayTimer(int _val) {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (mPlay) {
    mPlayFrame += 16;
    if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
      mPlayFrame = 0;
  } else if (mSimulating) {
    for (int i = 0; i < numIter; i++) {
      timeStepping();
      mWorld->bake();
    }
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void SimWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!mSimulating) {
      if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
      size_t nSkels = mWorld->getNumSkeletons();
      for (size_t i = 0; i < nSkels; i++) {
        // size_t start = mWorld->getIndex(i);
        // size_t size = mWorld->getSkeleton(i)->getNumDofs();
        mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
      }
      if (mShowMarkers) {
        // size_t sumDofs = mWorld->getIndex(nSkels);
        int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
        for (int i = 0; i < nContact; i++) {
            Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
            Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

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
          mWorld->getConstraintSolver()->getCollisionDetector();
      for (size_t k = 0; k < cd->getNumContacts(); k++) {
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
  drawEntities();

  // display the frame count in 2D text
  char buff[64];
  if (!mSimulating)
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#else
    std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#endif
  else
#ifdef _WIN32
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
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
  case 's':
      saveWorld();
      std::cout << "World Saved in 'tempWorld.txt'" << std::endl;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void SimWindow::setWorld(simulation::WorldPtr _world) {
  mWorld = _world;
}

void SimWindow::saveWorld() {
  if (!mWorld)
    return;
  kido::utils::FileInfoWorld worldFile;
  worldFile.saveFile("tempWorld.txt", mWorld->getRecording());
}

void SimWindow::plot(Eigen::VectorXd& _data) {
  GraphWindow* figure = new GraphWindow();
  figure->setData(_data);
  figure->initWindow(480, 240, "figure");
  mGraphWindows.push_back(figure);
}

}  // namespace gui
}  // namespace kido
