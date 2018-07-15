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

#include "dart/dynamics/HeightmapShape.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include "dart/common/Console.hpp"
#include "dart/dynamics/BoxShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
HeightmapShape::HeightmapShape() : Shape(HEIGHTMAP), mScale(1, 1, 1)
{
  // Do nothing
}

//==============================================================================
HeightmapShape::~HeightmapShape()
{
  // Do nothing
}

//==============================================================================
const std::string& HeightmapShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& HeightmapShape::getStaticType()
{
  static const std::string type("HeightmapShape");
  return type;
}

//==============================================================================
void HeightmapShape::setScale(const Eigen::Vector3d& scale)
{
  assert(scale[0] > 0.0);
  assert(scale[1] > 0.0);
  assert(scale[2] > 0.0);
  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
const Eigen::Vector3d& HeightmapShape::getScale() const
{
  return mScale;
}

//==============================================================================
void HeightmapShape::setHeightField(
    const std::size_t& width,
    const std::size_t& depth,
    const std::vector<HeightType>& heights)
{
  assert(heights.size() == width * depth);
  if ((width * depth) != heights.size())
  {
    dterr << "Size of height field needs to be width*depth=" << width * depth
          << std::endl;
    return;
  }
  if (heights.empty())
  {
    dtwarn << "Empty height field makes no sense." << std::endl;
    return;
  }
  mHeights = HeightField(depth, width);
  for (size_t r = 0; r < depth; ++r)
  {
    for (size_t c = 0; c < width; ++c)
    {
      mHeights(r, c) = heights[r * width + c];
    }
  }

  // compute minimum and maximum height
  mMinHeight = std::numeric_limits<HeightType>::max();
  mMaxHeight = -std::numeric_limits<HeightType>::max();
  for (auto it = heights.begin(); it != heights.end(); ++it)
  {
    if (*it < mMinHeight)
      mMinHeight = *it;
    if (*it > mMaxHeight)
      mMaxHeight = *it;
  }
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
const HeightmapShape::HeightField& HeightmapShape::getHeightField() const
{
  return mHeights;
}

//==============================================================================
HeightmapShape::HeightField& HeightmapShape::getHeightFieldModifiable() const
{
  return mHeights;
}

//==============================================================================
void HeightmapShape::flipY() const
{
  mHeights = mHeights.colwise().reverse().eval();
}

//==============================================================================
HeightmapShape::HeightType HeightmapShape::getMaxHeight() const
{
  return mMaxHeight;
}

//==============================================================================
HeightmapShape::HeightType HeightmapShape::getMinHeight() const
{
  return mMinHeight;
}

//==============================================================================
std::size_t HeightmapShape::getWidth() const
{
  return mHeights.cols();
}

//==============================================================================
std::size_t HeightmapShape::getDepth() const
{
  return mHeights.rows();
}

//==============================================================================
Eigen::Matrix3d HeightmapShape::computeInertia(double mass) const
{
  if (mIsBoundingBoxDirty)
  {
    updateBoundingBox();
  }
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), mass);
}

//==============================================================================
void HeightmapShape::computeBoundingBox(
    Eigen::Vector3d& min, Eigen::Vector3d& max) const
{
  const double dimX = getWidth() * mScale.x();
  const double dimY = getDepth() * mScale.y();
  const double dimZ = (mMaxHeight - mMinHeight) * mScale.z();
  min = Eigen::Vector3d(-dimX * 0.5, -dimY * 0.5, mMinHeight * mScale.z());
  max = min + Eigen::Vector3d(dimX, dimY, dimZ);
}

//==============================================================================
void HeightmapShape::updateBoundingBox() const
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  computeBoundingBox(min, max);
  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void HeightmapShape::updateVolume() const
{
  updateBoundingBox();
  const Eigen::Vector3d size = mBoundingBox.getMax() - mBoundingBox.getMin();
  mVolume = size.x() * size.y() * size.z();
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart
