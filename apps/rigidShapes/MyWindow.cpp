/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "MyWindow.h"

using namespace std;
using namespace Eigen;
using namespace dart;

//==============================================================================
MyWindow::MyWindow()
  : SimWindow()
{
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  mWorld->step();
}

//==============================================================================
void MyWindow::drawSkels()
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawSkels();
}

//==============================================================================
Isometry3d getRandomTransform()
{
  Isometry3d T = Isometry3d::Identity();

  const Vector3d rotation = math::randomVector<3>(-DART_PI, DART_PI);
  const Vector3d position = Vector3d(math::random(-1.0, 1.0),
                                     math::random( 0.5, 1.0),
                                     math::random(-1.0, 1.0));

  T.translation() = position;
  T.linear() = math::expMapRot(rotation);

  return T;
}

//==============================================================================
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
    case 'q':  // Spawn a cube
    case 'Q':
    {
      spawnBox(getRandomTransform(), math::randomVector<3>(0.05, 0.25));
      break;
    }
    case 'w':  // Spawn an ellipsoid
    case 'W':
    {
      spawnEllipsoid(getRandomTransform(), math::randomVector<3>(0.05, 0.25));
      break;
    }
    case 'e':  // Spawn an cylinder
    case 'E':
    {
      const double radius = math::random(0.05, 0.25);
      const double height = math::random(0.1, 0.5);
      spawnCylinder(getRandomTransform(), radius, height);
      break;
    }
    case 'a':    // Remove the skeleton added at last
    case 'A':
    {
      if (mWorld->getNumSkeletons() > 1)
      {
        mWorld->removeSkeleton(
              mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
      }
      break;
    }
    default:
      Win3D::keyboard(key, x, y);
  }
  glutPostRedisplay();
}

//==============================================================================
void MyWindow::spawnBox(const Eigen::Isometry3d& _T,
                        const Eigen::Vector3d& _size,
                        double _mass)
{
  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();

  dynamics::ShapePtr newShape(new dynamics::BoxShape(_size));
  newShape->setColor(math::randomVector<3>(0.0, 1.0));

  dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "box_link";
  bodyProp.mInertia.setMass(_mass);
  bodyProp.mVizShapes.push_back(newShape);
  bodyProp.mColShapes.push_back(newShape);

  dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = "box_joint";
  jointProp.mT_ParentBodyToJoint = _T;

  newSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        nullptr, jointProp, bodyProp);

  mWorld->addSkeleton(newSkeleton);
}

//==============================================================================
void MyWindow::spawnEllipsoid(const Eigen::Isometry3d& _T,
                              const Eigen::Vector3d& _radii,
                              double _mass)
{
  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();

  dynamics::ShapePtr newShape(new dynamics::EllipsoidShape(_radii));
  newShape->setColor(math::randomVector<3>(0.0, 1.0));

  dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "ellipsoid_link";
  bodyProp.mInertia.setMass(_mass);
  bodyProp.mVizShapes.push_back(newShape);
  bodyProp.mColShapes.push_back(newShape);

  dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = "ellipsoid_joint";
  jointProp.mT_ParentBodyToJoint = _T;

  newSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        nullptr, jointProp, bodyProp);

  mWorld->addSkeleton(newSkeleton);
}

//==============================================================================
void MyWindow::spawnCylinder(const Eigen::Isometry3d& _T,
                             double _radius,
                             double _height,
                             double _mass)
{
  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();

  dynamics::ShapePtr newShape(new dynamics::CylinderShape(_radius, _height));
  newShape->setColor(math::randomVector<3>(0.0, 1.0));

  dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = "cylinder_link";
  bodyProp.mInertia.setMass(_mass);
  bodyProp.mVizShapes.push_back(newShape);
  bodyProp.mColShapes.push_back(newShape);

  dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = "cylinder_joint";
  jointProp.mT_ParentBodyToJoint = _T;

  newSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        nullptr, jointProp, bodyProp);

  mWorld->addSkeleton(newSkeleton);
}

