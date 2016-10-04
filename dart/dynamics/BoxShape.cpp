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

#include <cmath>
#include "dart/dynamics/BoxShape.hpp"

namespace dart {
namespace dynamics {

BoxShape::BoxShape(const Eigen::Vector3d& _size)
  : Shape(BOX),
    mSize(_size) {
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
  mBoundingBox.setMin(-_size * 0.5);
  mBoundingBox.setMax(_size * 0.5);
  updateVolume();
}

BoxShape::~BoxShape() {
}

//==============================================================================
const std::string& BoxShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& BoxShape::getStaticType()
{
  static const std::string type("BoxShape");
  return type;
}

//==============================================================================
double BoxShape::computeVolume(const Eigen::Vector3d& size)
{
  return size[0] * size[1] * size[2];
}

//==============================================================================
Eigen::Matrix3d BoxShape::computeInertia(const Eigen::Vector3d& size,
                                         double mass)
{
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();

  inertia(0, 0) = mass / 12.0 * (std::pow(size[1], 2) + std::pow(size[2], 2));
  inertia(1, 1) = mass / 12.0 * (std::pow(size[0], 2) + std::pow(size[2], 2));
  inertia(2, 2) = mass / 12.0 * (std::pow(size[0], 2) + std::pow(size[1], 2));

  return inertia;
}

void BoxShape::setSize(const Eigen::Vector3d& _size) {
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
  mSize = _size;
  mBoundingBox.setMin(-_size * 0.5);
  mBoundingBox.setMax(_size * 0.5);
  updateVolume();
}

const Eigen::Vector3d& BoxShape::getSize() const {
  return mSize;
}

//==============================================================================
Eigen::Matrix3d BoxShape::computeInertia(double mass) const
{
  return computeInertia(mSize, mass);
}

//==============================================================================
void BoxShape::updateVolume()
{
  mVolume = computeVolume(mSize);
}

}  // namespace dynamics
}  // namespace dart
