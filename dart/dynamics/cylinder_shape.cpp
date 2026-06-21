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

#include "dart/dynamics/cylinder_shape.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/math/helpers.hpp"

#include <cmath>

namespace dart {
namespace dynamics {

//==============================================================================
CylinderShape::CylinderShape(double _radius, double _height) : Shape(CYLINDER)
{
  setRadius(_radius);
  setHeight(_height);
}

//==============================================================================
std::string_view CylinderShape::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view CylinderShape::getStaticType()
{
  static constexpr std::string_view type = "CylinderShape";
  return type;
}

//==============================================================================
double CylinderShape::getRadius() const
{
  return mRadius;
}

//==============================================================================
void CylinderShape::setRadius(double _radius)
{
  if (!std::isfinite(_radius) || _radius <= 0.0) {
    DART_WARN(
        "CylinderShape::setRadius: Invalid radius '{}'. Radius must be a "
        "positive finite value. Ignoring request.",
        _radius);
    return;
  }

  mRadius = _radius;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
double CylinderShape::getHeight() const
{
  return mHeight;
}

//==============================================================================
void CylinderShape::setHeight(double _height)
{
  if (!std::isfinite(_height) || _height <= 0.0) {
    DART_WARN(
        "CylinderShape::setHeight: Invalid height '{}'. Height must be a "
        "positive finite value. Ignoring request.",
        _height);
    return;
  }

  mHeight = _height;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
double CylinderShape::computeVolume(double radius, double height)
{
  return math::pi * radius * radius * height;
}

//==============================================================================
Eigen::Matrix3d CylinderShape::computeInertia(
    double radius, double height, double mass)
{
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();

  const auto radiusSquared = radius * radius;
  const auto heightSquared = height * height;
  inertia(0, 0) = mass * (3.0 * radiusSquared + heightSquared) / 12.0;
  inertia(1, 1) = inertia(0, 0);
  inertia(2, 2) = 0.5 * mass * radiusSquared;

  return inertia;
}

//==============================================================================
ShapePtr CylinderShape::clone() const
{
  return std::make_shared<CylinderShape>(mRadius, mHeight);
}

//==============================================================================
void CylinderShape::updateBoundingBox() const
{
  mBoundingBox.setMin(Eigen::Vector3d(-mRadius, -mRadius, -mHeight * 0.5));
  mBoundingBox.setMax(Eigen::Vector3d(mRadius, mRadius, mHeight * 0.5));
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void CylinderShape::updateVolume() const
{
  mVolume = computeVolume(mRadius, mHeight);
  mIsVolumeDirty = false;
}

//==============================================================================
Eigen::Matrix3d CylinderShape::computeInertia(double mass) const
{
  return computeInertia(mRadius, mHeight, mass);
}

} // namespace dynamics
} // namespace dart
