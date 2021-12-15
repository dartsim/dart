/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/dynamics/PyramidShape.hpp"

#include <cmath>

#include "dart/common/Console.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
PyramidShape::PyramidShape(double baseWidth, double baseDepth, double height)
  : Shape(PYRAMID),
    mBaseWidth(baseWidth),
    mBaseDepth(baseDepth),
    mHeight(height)
{
  assert(0.0 < baseWidth);
  assert(0.0 < baseDepth);
  assert(0.0 < height);
}

//==============================================================================
const std::string& PyramidShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& PyramidShape::getStaticType()
{
  static const std::string type("PyramidShape");
  return type;
}

//==============================================================================
double PyramidShape::getBaseWidth() const
{
  return mBaseWidth;
}

//==============================================================================
void PyramidShape::setBaseWidth(double width)
{
  assert(0.0 < width);
  mBaseWidth = width;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
double PyramidShape::getBaseDepth() const
{
  return mBaseDepth;
}

//==============================================================================
void PyramidShape::setBaseDepth(double depth)
{
  assert(0.0 < depth);
  mBaseDepth = depth;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
double PyramidShape::getHeight() const
{
  return mHeight;
}

//==============================================================================
void PyramidShape::setHeight(double height)
{
  assert(0.0 < height);
  mHeight = height;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
double PyramidShape::computeVolume(
    double baseWidth, double baseDepth, double height)
{
  return (1.0 / 3.0) * baseWidth * baseDepth * height;
}

//==============================================================================
void PyramidShape::updateBoundingBox() const
{
  const Eigen::Vector3d corner(
      0.5 * mBaseWidth, 0.5 * mBaseDepth, 0.5 * mHeight);

  mBoundingBox.setMin(-corner);
  mBoundingBox.setMax(corner);

  mIsBoundingBoxDirty = false;
}

//==============================================================================
void PyramidShape::updateVolume() const
{
  mVolume = computeVolume(mBaseWidth, mBaseDepth, mHeight);
  mIsVolumeDirty = false;
}

//==============================================================================
Eigen::Matrix3d PyramidShape::computeInertia(double /*mass*/) const
{
  // TODO(JS): Not implemented
  dterr << "[PyramidShape] Moment of inertia computation is not implemented. "
        << "Returning identity.\n";

  return Eigen::Matrix3d::Identity();
}

//==============================================================================
ShapePtr PyramidShape::clone() const
{
  return std::make_shared<PyramidShape>(mBaseWidth, mBaseDepth, mHeight);
}

} // namespace dynamics
} // namespace dart
