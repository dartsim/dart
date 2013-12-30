/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Tobias Kunz <tobias@gatech.edu>
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

#include "dart/dynamics/CylinderShape.h"

#include "dart/renderer/RenderInterface.h"

namespace dart {
namespace dynamics {

CylinderShape::CylinderShape(double _radius, double _height)
  : Shape(CYLINDER),
    mRadius(_radius),
    mHeight(_height) {
  assert(0.0 < _radius);
  assert(0.0 < _height);
  _updateBoundingBoxDim();
  initMeshes();
  computeVolume();
}

double CylinderShape::getRadius() const {
  return mRadius;
}

void CylinderShape::setRadius(double _radius) {
  assert(0.0 < _radius);
  mRadius = _radius;
  _updateBoundingBoxDim();
  computeVolume();
}

double CylinderShape::getHeight() const {
  return mHeight;
}

void CylinderShape::setHeight(double _height) {
  assert(0.0 < _height);
  mHeight = _height;
  _updateBoundingBoxDim();
  computeVolume();
}

void CylinderShape::draw(renderer::RenderInterface* _ri,
                         const Eigen::Vector4d& _color,
                         bool _useDefaultColor) const {
  if (!_ri) return;
  if (!_useDefaultColor)
    _ri->setPenColor(_color);
  else
    _ri->setPenColor(mColor);
  _ri->pushMatrix();
  _ri->transform(mTransform);
  _ri->drawCylinder(mRadius, mHeight);
  _ri->popMatrix();
}

void CylinderShape::computeVolume() {
  mVolume = DART_PI * mRadius * mRadius * mHeight;
}

Eigen::Matrix3d CylinderShape::computeInertia(double _mass) const {
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = _mass * (3.0 * mRadius * mRadius + mHeight * mHeight) / 12.0;
  inertia(1, 1) = inertia(0, 0);
  inertia(2, 2) = 0.5 * _mass * mRadius * mRadius;

  return inertia;
}

void CylinderShape::_updateBoundingBoxDim() {
  mBoundingBoxDim[0] = mRadius * 2.0;
  mBoundingBoxDim[1] = mRadius * 2.0;
  mBoundingBoxDim[2] = mHeight;
}

}  // namespace dynamics
}  // namespace dart
