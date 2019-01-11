/*
 * Copyright (c) 2011-2019, The DART development contributors
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
template <typename S>
HeightmapShape<S>::HeightmapShape() : Shape(HEIGHTMAP), mScale(1, 1, 1)
{
  static_assert(
      std::is_same<S, float>::value || std::is_same<S, double>::value,
      "Height field needs to be double or float");
}

//==============================================================================
template <typename S>
const std::string& HeightmapShape<S>::getType() const
{
  return getStaticType();
}

//==============================================================================
template <typename S>
const std::string& HeightmapShape<S>::getStaticType()
{
  static const std::string type
      = "HeightmapShape (" + std::string(typeid(S).name()) + ")";
  return type;
}

//==============================================================================
template <typename S>
void HeightmapShape<S>::setScale(const Eigen::Vector3d& scale)
{
  assert(scale[0] > 0.0);
  assert(scale[1] > 0.0);
  assert(scale[2] > 0.0);
  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
template <typename S>
const Eigen::Vector3d& HeightmapShape<S>::getScale() const
{
  return mScale;
}

//==============================================================================
template <typename S>
void HeightmapShape<S>::setHeightField(
    const std::size_t& width,
    const std::size_t& depth,
    const std::vector<S>& heights)
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
  mMinHeight = std::numeric_limits<S>::max();
  mMaxHeight = -std::numeric_limits<S>::max();
  for (auto it = heights.begin(); it != heights.end(); ++it)
  {
    if (*it < mMinHeight)
      mMinHeight = *it;
    if (*it > mMaxHeight)
      mMaxHeight = *it;
  }
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
template <typename S>
auto HeightmapShape<S>::getHeightField() const -> const HeightField&
{
  return mHeights;
}

//==============================================================================
template <typename S>
auto HeightmapShape<S>::getHeightFieldModifiable() const -> HeightField&
{
  return mHeights;
}

//==============================================================================
template <typename S>
void HeightmapShape<S>::flipY() const
{
  mHeights = mHeights.colwise().reverse().eval();
}

//==============================================================================
template <typename S>
auto HeightmapShape<S>::getMaxHeight() const -> S
{
  return mMaxHeight;
}

//==============================================================================
template <typename S>
auto HeightmapShape<S>::getMinHeight() const -> S
{
  return mMinHeight;
}

//==============================================================================
template <typename S>
std::size_t HeightmapShape<S>::getWidth() const
{
  return mHeights.cols();
}

//==============================================================================
template <typename S>
std::size_t HeightmapShape<S>::getDepth() const
{
  return mHeights.rows();
}

//==============================================================================
template <typename S>
Eigen::Matrix3d HeightmapShape<S>::computeInertia(double mass) const
{
  if (mIsBoundingBoxDirty)
  {
    updateBoundingBox();
  }
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), mass);
}

//==============================================================================
template <typename S>
void HeightmapShape<S>::computeBoundingBox(
    Eigen::Vector3d& min, Eigen::Vector3d& max) const
{
  const double dimX = getWidth() * mScale.x();
  const double dimY = getDepth() * mScale.y();
  const double dimZ = (mMaxHeight - mMinHeight) * mScale.z();
  min = Eigen::Vector3d(-dimX * 0.5, -dimY * 0.5, mMinHeight * mScale.z());
  max = min + Eigen::Vector3d(dimX, dimY, dimZ);
}

//==============================================================================
template <typename S>
void HeightmapShape<S>::updateBoundingBox() const
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  computeBoundingBox(min, max);
  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);
  mIsBoundingBoxDirty = false;
}

//==============================================================================
template <typename S>
void HeightmapShape<S>::updateVolume() const
{
  updateBoundingBox();
  const Eigen::Vector3d size = mBoundingBox.getMax() - mBoundingBox.getMin();
  mVolume = size.x() * size.y() * size.z();
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart
