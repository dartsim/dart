/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "dart/dynamics/PlaneShape.h"

#include "dart/renderer/RenderInterface.h"

namespace dart {
namespace dynamics {

PlaneShape::PlaneShape(const Eigen::Vector3d& _normal,
                       const Eigen::Vector3d& _point)
  : Shape(PLANE),
    mNormal(_normal),
    mPoint(_point) {
}

void PlaneShape::draw(renderer::RenderInterface* _ri,
                      const Eigen::Vector4d& _color,
                      bool _useDefaultColor) const {
  // TODO(JS): Not implemented yet
  if (!_ri) return;
  if (!_useDefaultColor)
    _ri->setPenColor(_color);
  else
    _ri->setPenColor(mColor);
  _ri->pushMatrix();
  _ri->transform(mTransform);
//  _ri->drawCube(mDim);
  _ri->popMatrix();
}

Eigen::Matrix3d PlaneShape::computeInertia(double _mass) const {
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  return inertia;
}

void PlaneShape::setNormal(const Eigen::Vector3d& _normal) {
  mNormal = _normal.normalized();
}

const Eigen::Vector3d& PlaneShape::getNormal() const {
  return mNormal;
}

void PlaneShape::setPoint(const Eigen::Vector3d& _point) {
  mPoint = _point;
}

const Eigen::Vector3d& PlaneShape::getPoint() const {
  return mPoint;
}

void PlaneShape::computeVolume() {
  mVolume = 0.0;
}

}  // namespace dynamics
}  // namespace dart
