/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
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

#include "dart/dynamics/EllipsoidShape.hpp"

#include "dart/renderer/RenderInterface.hpp"

namespace dart {
namespace dynamics {

EllipsoidShape::EllipsoidShape(const Eigen::Vector3d& _size)
  : Shape(ELLIPSOID) {
  setSize(_size);
}

EllipsoidShape::~EllipsoidShape() {
}

void EllipsoidShape::setSize(const Eigen::Vector3d& _size) {
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
  mSize = _size;
  mBoundingBox.setMin(-_size * 0.5);
  mBoundingBox.setMax(_size * 0.5);
  updateVolume();
}

const Eigen::Vector3d&EllipsoidShape::getSize() const {
  return mSize;
}

//==============================================================================
void EllipsoidShape::draw(renderer::RenderInterface* ri,
                          const Eigen::Vector4d& col) const
{
  if (!ri)
    return;

  ri->setPenColor(col);
  ri->drawEllipsoid(mBoundingBox.computeFullExtents());
}

//==============================================================================
double EllipsoidShape::computeVolume(const Eigen::Vector3d& size)
{
  // 4/3* Pi* a/2* b/2* c/2
  return DART_PI * size[0] * size[1] * size[2] / 6.0;
}

//==============================================================================
Eigen::Matrix3d EllipsoidShape::computeInertia(
    const Eigen::Vector3d& size, double mass)
{
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();

  inertia(0, 0) = mass / 20.0 * (std::pow(size[1], 2) + std::pow(size[2], 2));
  inertia(1, 1) = mass / 20.0 * (std::pow(size[0], 2) + std::pow(size[2], 2));
  inertia(2, 2) = mass / 20.0 * (std::pow(size[0], 2) + std::pow(size[1], 2));

  return inertia;
}

//==============================================================================
Eigen::Matrix3d EllipsoidShape::computeInertia(double mass) const
{
  return computeInertia(mSize, mass);
}

bool EllipsoidShape::isSphere() const {
  if (mSize[0] == mSize[1] && mSize[1] == mSize[2])
    return true;
  else
    return false;
}

//==============================================================================
void EllipsoidShape::updateVolume()
{
  mVolume = computeVolume(mSize);
}

}  // namespace dynamics
}  // namespace dart
