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

#include "MyWindow.h"

MyWindow::MyWindow()
  : SimWindow() {
}

MyWindow::~MyWindow() {
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
      if (mSimulating)
        mPlay = false;
      break;
    case 'q':  // Spawn a cube
    case 'Q': {  // Spawn a cube
      Eigen::Vector3d position = Eigen::Vector3d(dart::math::random(-1.0, 1.0),
                                                 dart::math::random( 0.5, 1.0),
                                                 dart::math::random(-1.0, 1.0));
      Eigen::Vector3d size = Eigen::Vector3d(dart::math::random(0.1, 0.5),
                                             dart::math::random(0.1, 0.5),
                                             dart::math::random(0.1, 0.5));
      spawnCube(position, size);
      break;
    }
    case 'w':    // Spawn a cube
    case 'W': {  // Spawn a cube
      if (mWorld->getNumSkeletons() > 1)
        mWorld->removeSkeleton(mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
      break;
    }
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void MyWindow::spawnCube(const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _size,
                         double _mass)
{
  dart::dynamics::SkeletonPtr newCubeSkeleton =
      dart::dynamics::Skeleton::create();

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(_mass);
  dart::dynamics::ShapePtr newBoxShape(new dart::dynamics::BoxShape(_size));
  body.mVizShapes.push_back(newBoxShape);
  body.mColShapes.push_back(newBoxShape);
  newBoxShape->setColor(dart::math::randomVector<3>(0.0, 1.0));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(_position);

  newCubeSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr, joint, body);

  mWorld->addSkeleton(newCubeSkeleton);
}
