/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "dart/dynamics/SphereShape.hpp"

#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
SphereShape::SphereShape(double radius)
  : Shape(SPHERE)
{
  setRadius(radius);
}

//==============================================================================
SphereShape::~SphereShape()
{
  // Do nothing
}

//==============================================================================
const std::string& SphereShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& SphereShape::getStaticType()
{
  static const std::string type("SphereShape");
  return type;
}

//==============================================================================
void SphereShape::setRadius(double radius)
{
  assert(radius > 0.0);

  mRadius = radius;

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
double SphereShape::getRadius() const
{
  return mRadius;
}

//==============================================================================
double SphereShape::computeVolume(double radius)
{
  return math::constantsd::pi() * 4.0 / 3.0 * std::pow(radius, 3) ;
}

//==============================================================================
Eigen::Matrix3d SphereShape::computeInertia(double radius, double mass)
{
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();

  inertia(0, 0) = 2.0 / 5.0 * mass * std::pow(radius, 2);
  inertia(1, 1) = inertia(0, 0);
  inertia(2, 2) = inertia(0, 0);

  return inertia;
}

//==============================================================================
Eigen::Matrix3d SphereShape::computeInertia(double mass) const
{
  return computeInertia(mRadius, mass);
}

//==============================================================================
void SphereShape::updateBoundingBox() const
{
  mBoundingBox.setMin(Eigen::Vector3d::Constant(-mRadius));
  mBoundingBox.setMax(Eigen::Vector3d::Constant(mRadius));
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void SphereShape::updateVolume() const
{
  mVolume = computeVolume(mRadius);
  mIsVolumeDirty = false;
}

}  // namespace dynamics
}  // namespace dart
