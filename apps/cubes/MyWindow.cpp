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

#include "apps/cubes/MyWindow.h"

#include "dart/math/Helpers.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"

MyWindow::MyWindow()
  : SimWindow() {
  mForce = Eigen::Vector3d::Zero();
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  mWorld->getSkeleton(1)->getBodyNode(0)->addExtForce(
        Eigen::Vector3d(0.0, 0.0, 0.0), mForce);
  mWorld->step();
  mForce /= 2.0;
}

void MyWindow::drawSkels() {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
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
    case '1':  // upper right force
      mForce[0] = -500;
      break;
    case '2':  // upper right force
      mForce[0] = 500;
      break;
    case '3':  // upper right force
      mForce[2] = -500;
      break;
    case '4':  // upper right force
      mForce[2] = 500;
      break;
    case 'q':  // Spawn a cube
    case 'Q': {  // Spawn a cube
      Eigen::Vector3d position = Eigen::Vector3d(dart::math::random(-1.0, 1.0),
                                                 dart::math::random(-1.0, 1.0),
                                                 dart::math::random(0.5, 1.0));
      Eigen::Vector3d size = Eigen::Vector3d(dart::math::random(0.01, 0.2),
                                             dart::math::random(0.01, 0.2),
                                             dart::math::random(0.01, 0.2));
      spawnCube(position, size);
      break;
    }
    case 'w':    // Spawn a cube
    case 'W': {  // Spawn a cube
      if (mWorld->getNumSkeletons() > 4)
        mWorld->removeSkeleton(mWorld->getSkeleton(4));
      break;
    }
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void MyWindow::spawnCube(const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _size,
                         double _mass) {
  dart::dynamics::Skeleton*  newCubeSkeleton =
      new dart::dynamics::Skeleton();
  dart::dynamics::BodyNode*  newBodyNode     =
      new dart::dynamics::BodyNode("cube_link");
  dart::dynamics::FreeJoint* newFreeJoint    =
      new dart::dynamics::FreeJoint("cube_joint");
  dart::dynamics::BoxShape*  newBoxShape     =
      new dart::dynamics::BoxShape(_size);

  newBodyNode->addVisualizationShape(newBoxShape);
  newBodyNode->addCollisionShape(newBoxShape);
  newBodyNode->setMass(_mass);
  newBodyNode->setParentJoint(newFreeJoint);
  newFreeJoint->setTransformFromParentBodyNode(
        Eigen::Isometry3d(Eigen::Translation3d(_position)));
  newBoxShape->setColor(Eigen::Vector3d(dart::math::random(0.0, 1.0),
                                        dart::math::random(0.0, 1.0),
                                        dart::math::random(0.0, 1.0)));
  newCubeSkeleton->addBodyNode(newBodyNode);
  mWorld->addSkeleton(newCubeSkeleton);
}
