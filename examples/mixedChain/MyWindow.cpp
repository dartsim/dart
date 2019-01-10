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

#include "MyWindow.hpp"

#include <dart/gui/GLFuncs.hpp>

#define FORCE_ON_RIGIDBODY 500.0;
#define FORCE_ON_VERTEX 1.00;

MyWindow::MyWindow()
  : SoftSimWindow()
{
  mForceOnRigidBody = Eigen::Vector3d::Zero();
  mForceOnVertex = Eigen::Vector3d::Zero();
  mImpulseDuration = 0.0;
}

MyWindow::~MyWindow()
{
}

void MyWindow::timeStepping()
{
//  dart::dynamics::SkeletonPtr Skeleton =
//      static_cast<dart::dynamics::SkeletonPtr>(mWorld->getSkeleton(0));
//  dart::dynamics::SoftBodyNode* softBodyNode = Skeleton->getSoftBodyNode(0);
//  softBodyNode->addExtForce(mForceOnRigidBody);

  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0)
  {
    mImpulseDuration = 0;
    mForceOnRigidBody.setZero();
  }

  mForceOnVertex /= 2.0;
}

void MyWindow::drawWorld() const
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  Eigen::Vector4d color;
//  color << 0.5, 0.8, 0.6, 1.0;
//  mWorld->getSkeleton(0)->draw(mRI, color, false);

  // draw arrow
  if (mImpulseDuration > 0)
  {
    dart::dynamics::SkeletonPtr Skeleton = mWorld->getSkeleton(1);
    dart::dynamics::SoftBodyNode* softBodyNode
        = Skeleton->getSoftBodyNode(3);
    softBodyNode->addExtForce(mForceOnRigidBody);
    Eigen::Vector3d poa
        = softBodyNode->getTransform() * Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d start = poa - mForceOnRigidBody / 25.0;
    double len = mForceOnRigidBody.norm() / 25.0;
    dart::gui::drawArrow3D(start, mForceOnRigidBody, len, 0.025, 0.05);
  }

  SimWindow::drawWorld();
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
  switch (key)
  {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay)
        mSimulating = false;
      break;
    case '[':  // step backward
      if (!mSimulating)
      {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating)
      {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'n':
      mShowPointMasses = !mShowPointMasses;
      break;
    case 'm':
      mShowMeshs = !mShowMeshs;
      break;
    case 'q':  // upper right force
      mForceOnRigidBody[0] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'w':  // upper right force
      mForceOnRigidBody[0] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'e':  // upper right force
      mForceOnRigidBody[1] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'r':  // upper right force
      mForceOnRigidBody[1] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 't':  // upper right force
      mForceOnRigidBody[2] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'y':  // upper right force
      mForceOnRigidBody[2] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    default:
      Win3D::keyboard(key, x, y);
  }
  glutPostRedisplay();
}
