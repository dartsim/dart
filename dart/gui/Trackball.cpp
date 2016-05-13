/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/gui/Trackball.hpp"

#include "dart/gui/LoadOpengl.hpp"

namespace dart {
namespace gui {

Trackball::Trackball()
  : mCenter(0.0, 0.0),
    mRadius(1.0),
    mStartPos(0.0, 0.0, 0.0),
    mCurrQuat(1.0, 0.0, 0.0, 0.0) {
}

Trackball::Trackball(const Eigen::Vector2d& _center, double _radius)
  : mCenter(_center),
    mRadius(_radius),
    mStartPos(0.0, 0.0, 0.0),
    mCurrQuat(1.0, 0.0, 0.0, 0.0) {
}

void Trackball::startBall(double _x, double _y) {
  mStartPos = mouseOnSphere(_x, _y);
}

void Trackball::updateBall(double _x, double _y) {
  Eigen::Vector3d toPos = mouseOnSphere(_x, _y);
  Eigen::Quaterniond newQuat(quatFromVectors(mStartPos, toPos));
  mStartPos = toPos;
  mCurrQuat = newQuat*mCurrQuat;
}

void Trackball::applyGLRotation() {
  Eigen::Transform<double, 3, Eigen::Affine> t(mCurrQuat);
  glMultMatrixd(t.data());
}

void Trackball::draw(int _winWidth, int _winHeight) {
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);

  glPushMatrix();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, _winWidth, _winHeight);
  gluOrtho2D(0.0, (GLdouble)_winWidth, 0.0, (GLdouble)_winHeight);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glLineWidth(4.0);
  glColor3f(1.0f, 1.0f, 0.0f);
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < 360; i += 4) {
    double theta = i / 180.0 * M_PI;
    double x = mRadius * cos(theta);
    double y = mRadius * sin(theta);
    glVertex2d(static_cast<GLdouble>((_winWidth >> 1) + x),
               static_cast<GLdouble>((_winHeight >> 1) + y));
  }
  glEnd();

  glPopMatrix();
}

void Trackball::setTrackball(const Eigen::Vector2d& _center,
                             const double _radius) {
  mCenter = _center;
  mRadius = _radius;
}

void Trackball::setCenter(const Eigen::Vector2d& _center) {
  mCenter = _center;
}

void Trackball::setRadius(const double _radius) {
  mRadius = _radius;
}

void Trackball::setQuaternion(const Eigen::Quaterniond& _q) {
  mCurrQuat = _q;
}

Eigen::Quaterniond Trackball::getCurrQuat() const {
  return mCurrQuat;
}

Eigen::Matrix3d Trackball::getRotationMatrix() const {
  return mCurrQuat.toRotationMatrix();
}

Eigen::Vector2d Trackball::getCenter() const {
  return mCenter;
}

double Trackball::getRadius() const {
  return mRadius;
}

Eigen::Vector3d Trackball::mouseOnSphere(double _mouseX, double _mouseY) const {
  double mag;
  Eigen::Vector3d pointOnSphere;

  pointOnSphere(0) = (_mouseX - mCenter(0)) / mRadius;
  pointOnSphere(1) = (_mouseY - mCenter(1)) / mRadius;

  mag = pointOnSphere(0) * pointOnSphere(0) + pointOnSphere(1)*pointOnSphere(1);
  if (mag > 1.0) {
    double scale = 1.0/sqrt(mag);
    pointOnSphere(0) *= scale;
    pointOnSphere(1) *= scale;
    pointOnSphere(2) = 0.0;
  } else {
    pointOnSphere(2) = sqrt(1 - mag);
  }

  return pointOnSphere;
}

Eigen::Quaterniond Trackball::quatFromVectors(
    const Eigen::Vector3d& _from, const Eigen::Vector3d& _to) const {
  Eigen::Quaterniond quat;
  quat.x() = _from(1)*_to(2) - _from(2)*_to(1);
  quat.y() = _from(2)*_to(0) - _from(0)*_to(2);
  quat.z() = _from(0)*_to(1) - _from(1)*_to(0);
  quat.w() = _from(0)*_to(0) + _from(1)*_to(1) + _from(2)*_to(2);
  return quat;
}

}  // namespace gui
}  // namespace dart
