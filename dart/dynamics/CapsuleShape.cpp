/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/CapsuleShape.hpp"

#include <cmath>
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
CapsuleShape::CapsuleShape(double radius, double height)
  : Shape(CAPSULE),
    mRadius(radius),
    mHeight(height)
{
  assert(0.0 < radius);
  assert(0.0 < height);
  updateBoundingBoxDim();
  updateVolume();
}

//==============================================================================
const std::string& CapsuleShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& CapsuleShape::getStaticType()
{
  static const std::string type("CapsuleShape");
  return type;
}

//==============================================================================
double CapsuleShape::getRadius() const
{
  return mRadius;
}

//==============================================================================
void CapsuleShape::setRadius(double radius)
{
  assert(0.0 < radius);
  mRadius = radius;
  updateBoundingBoxDim();
  updateVolume();
}

//==============================================================================
double CapsuleShape::getHeight() const
{
  return mHeight;
}

//==============================================================================
void CapsuleShape::setHeight(double height)
{
  assert(0.0 < height);
  mHeight = height;
  updateBoundingBoxDim();
  updateVolume();
}

//==============================================================================
double CapsuleShape::computeVolume(double radius, double height)
{
  return CylinderShape::computeVolume(radius, height)
      + SphereShape::computeVolume(radius);
}

//==============================================================================
Eigen::Matrix3d CapsuleShape::computeInertia(
    double radius, double height, double mass)
{
  // Reference: http://www.gamedev.net/page/resources/_/technical/
  // math-and-physics/capsule-inertia-tensor-r3856

  const auto radius2 = radius*radius;
  const auto height2 = height*height;

  const auto volumeCylinder = CylinderShape::computeVolume(radius, height);
  const auto volumeSphere = SphereShape::computeVolume(radius);

  const auto density = mass / (volumeCylinder + volumeSphere);

  const auto massCylinder = density * volumeCylinder;
  const auto massSphere = density * volumeSphere;

  const auto Ixx
      = massCylinder*(height2/12.0 + radius2/4.0)
      + massSphere*(height2 + (3.0/8.0)*height*radius + (2.0/5.0)*radius2);
  const auto Izz = massCylinder*(radius2/2.0) + massSphere*((2.0/5.0)*radius2);

  return Eigen::Vector3d(Ixx, Ixx, Izz).asDiagonal();
}

//==============================================================================
void CapsuleShape::updateVolume()
{
  mVolume = computeVolume(mRadius, mHeight);
}

//==============================================================================
Eigen::Matrix3d CapsuleShape::computeInertia(double mass) const
{
  return computeInertia(mRadius, mHeight, mass);
}

//==============================================================================
void CapsuleShape::updateBoundingBoxDim()
{
  const Eigen::Vector3d corner(mRadius, mRadius, mRadius + 0.5*mHeight);

  mBoundingBox.setMin(-corner);
  mBoundingBox.setMax(corner);
}

}  // namespace dynamics
}  // namespace dart
