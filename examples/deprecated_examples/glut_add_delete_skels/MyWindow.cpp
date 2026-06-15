/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <chrono>

#include <cstdio>

MyWindow::MyWindow() : SimWindow() {}

MyWindow::~MyWindow() {}

void MyWindow::draw()
{
  // Render the world plus the base class's 2D simulation-frame counter.
  SimWindow::draw();

  // --- Update the HUD timing readouts -------------------------------------
  const auto now = std::chrono::steady_clock::now();
  const double simTime = mWorld->getTime();
  if (mHudInitialized) {
    const double wallDt
        = std::chrono::duration<double>(now - mLastDrawWall).count();
    if (wallDt > 1e-6) {
      // Exponential moving averages keep both readouts steady frame-to-frame.
      const double fps = 1.0 / wallDt;
      mFpsEma = (mFpsEma < 0.0) ? fps : 0.9 * mFpsEma + 0.1 * fps;
      if (mSimulating) {
        // Real-time factor: simulated seconds advanced per wall-clock second.
        // < 1 means the physics cannot keep up with real time.
        const double rtf = (simTime - mLastSimTime) / wallDt;
        mRtfEma = (mRtfEma < 0.0) ? rtf : 0.9 * mRtfEma + 0.1 * rtf;
      }
    }
  }
  mLastDrawWall = now;
  mLastSimTime = simTime;
  mHudInitialized = true;

  // Dynamic cubes = every skeleton except the static ground.
  const std::size_t numSkeletons = mWorld->getNumSkeletons();
  const std::size_t numCubes = (numSkeletons > 0) ? numSkeletons - 1 : 0;

  // --- Draw the HUD in the top-left corner --------------------------------
  // SimWindow::draw() re-enables lighting, so disable it for flat text.
  glDisable(GL_LIGHTING);
  glColor3f(0.0f, 0.0f, 0.0f);

  char line[96];
  std::snprintf(line, sizeof(line), "FPS: %.1f", mFpsEma < 0.0 ? 0.0 : mFpsEma);
  dart::gui::glut::drawStringOnScreen(0.02f, 0.95f, line);

  if (mSimulating)
    std::snprintf(
        line, sizeof(line), "Physics RTF: %.2fx", mRtfEma < 0.0 ? 0.0 : mRtfEma);
  else
    std::snprintf(line, sizeof(line), "Physics RTF: paused");
  dart::gui::glut::drawStringOnScreen(0.02f, 0.91f, line);

  std::snprintf(line, sizeof(line), "Cubes: %zu", numCubes);
  dart::gui::glut::drawStringOnScreen(0.02f, 0.87f, line);

  glEnable(GL_LIGHTING);
}

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
      Eigen::Vector3d position = Eigen::Vector3d(
          dart::math::Random::uniform(-1.0, 1.0),
          dart::math::Random::uniform(0.5, 1.0),
          dart::math::Random::uniform(-1.0, 1.0));
      Eigen::Vector3d size = Eigen::Vector3d(
          dart::math::Random::uniform(0.1, 0.5),
          dart::math::Random::uniform(0.1, 0.5),
          dart::math::Random::uniform(0.1, 0.5));
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
    const Eigen::Vector3d& _position,
    const Eigen::Vector3d& _size,
    double _mass)
{
  dart::dynamics::SkeletonPtr newCubeSkeleton
      = dart::dynamics::Skeleton::create();

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(_mass);
  body.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(_size, _mass));
  dart::dynamics::ShapePtr newBoxShape(new dart::dynamics::BoxShape(_size));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(_position);

  auto pair
      = newCubeSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
          nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(newBoxShape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  mWorld->addSkeleton(newCubeSkeleton);
}
