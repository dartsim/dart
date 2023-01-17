/*
 * Copyright (c) 2011-2023, The DART development contributors
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

using namespace dart;
using namespace dart::math;
using namespace dart::dynamics;

MyWindow::MyWindow() : SimWindow() {}

MyWindow::~MyWindow() {}

void MyWindow::drawWorld() const
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawWorld();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  switch (_key) {
    case ' ': // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'q':   // Spawn a cube
    case 'Q': { // Spawn a cube
      Vector3d position
          = Vector3d(Uniform(-1.0, 1.0), Uniform(0.5, 1.0), Uniform(-1.0, 1.0));
      Vector3d size
          = Vector3d(Uniform(0.1, 0.5), Uniform(0.1, 0.5), Uniform(0.1, 0.5));
      spawnCube(position, size);
      break;
    }
    case 'w':   // Spawn a cube
    case 'W': { // Spawn a cube
      if (mWorld->getNumSkeletons() > 1)
        mWorld->removeSkeleton(
            mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
      break;
    }
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void MyWindow::spawnCube(
    const Vector3d& _position, const Vector3d& _size, double _mass)
{
  SkeletonPtr newCubeSkeleton = Skeleton::create();

  BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(_mass);
  body.mInertia.setMoment(BoxShape::computeInertia(_size, _mass));
  ShapePtr newBoxShape(new BoxShape(_size));

  FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Translation3d(_position);

  auto pair = newCubeSkeleton->createJointAndBodyNodePair<FreeJoint>(
      nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(newBoxShape);
  shapeNode->getVisualAspect()->setColor(Uniform<Vector3d>(0.0, 1.0));

  mWorld->addSkeleton(newCubeSkeleton);
}
