/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/PlaneShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
PlaneShape::PlaneShape(const Eigen::Vector3d& _normal, double _offset)
  : Shape(PLANE),
    mNormal(_normal.normalized()),
    mOffset(_offset)
{
}

//==============================================================================
PlaneShape::PlaneShape(const Eigen::Vector3d& _normal,
                       const Eigen::Vector3d& _point)
  : Shape(),
    mNormal(_normal.normalized()),
    mOffset(mNormal.dot(_point))
{
}

//==============================================================================
const std::string& PlaneShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& PlaneShape::getStaticType()
{
  static const std::string type("PlaneShape");
  return type;
}

//==============================================================================
Eigen::Matrix3d PlaneShape::computeInertia(double /*mass*/) const
{
  return Eigen::Matrix3d::Zero();
}

//==============================================================================
void PlaneShape::setNormal(const Eigen::Vector3d& _normal)
{
  mNormal = _normal.normalized();
}

//==============================================================================
const Eigen::Vector3d& PlaneShape::getNormal() const
{
  return mNormal;
}

//==============================================================================
void PlaneShape::setOffset(double _offset)
{
  mOffset = _offset;
}

//==============================================================================
double PlaneShape::getOffset() const
{
  return mOffset;
}

//==============================================================================
void PlaneShape::setNormalAndOffset(const Eigen::Vector3d& _normal,
                                    double _offset)
{
  setNormal(_normal);
  setOffset(_offset);
}

//==============================================================================
void PlaneShape::setNormalAndPoint(const Eigen::Vector3d& _normal,
                                   const Eigen::Vector3d& _point)
{
  setNormal(_normal);
  setOffset(mNormal.dot(_point));
}

//==============================================================================
double PlaneShape::computeDistance(const Eigen::Vector3d& _point) const
{
  return std::abs(computeSignedDistance(_point));
}

//==============================================================================
double PlaneShape::computeSignedDistance(const Eigen::Vector3d& _point) const
{
  return mNormal.dot(_point) - mOffset;
}

//==============================================================================
void PlaneShape::updateVolume()
{
  mVolume = 0.0;
}

}  // namespace dynamics
}  // namespace dart
