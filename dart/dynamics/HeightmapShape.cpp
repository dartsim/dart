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

#include <cmath>
#include "dart/dynamics/HeightmapShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/common/Console.hpp"
#include <limits>
#include <algorithm>

namespace dart {
namespace dynamics {

//==============================================================================
HeightmapShape::HeightmapShape()
  : Shape(HEIGHTMAP),
    mScale(1, 1, 1)
{
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
void HeightmapShape::setHeightField(const size_t& width, const size_t& height,
                                    const std::vector<HeightType>& heights)
{
  // dtdbg << "Setting height field " << width << "*" << height << std::endl;
  assert(heights.size() == width*height);
  if ((width*height) != heights.size())
  {
    dterr << "Size of height field needs to be width*height="
          << width*height << std::endl;
    return;
  }
  if (heights.empty())
  {
    dtwarn << "Empty height field makes no sense." << std::endl;
    return;
  }
  mHeights = heights;
  mWidth = width;
  mHeight = height;  

  // compute minimum and maximum height
  mMinHeight = std::numeric_limits<HeightType>::max();
  mMaxHeight = -std::numeric_limits<HeightType>::max();
  for (auto it = heights.begin(); it != heights.end(); ++it)
  {
    if (*it < mMinHeight) mMinHeight = *it;
    if (*it > mMaxHeight) mMaxHeight = *it;
  }
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
const std::vector<HeightmapShape::HeightType>&
  HeightmapShape::getHeightField() const
{
  return mHeights;
}

//==============================================================================
std::vector<HeightmapShape::HeightType>&
  HeightmapShape::getHeightFieldModifiable() const
{
  return mHeights;
}

//==============================================================================
void HeightmapShape::flipY() const
{
  assert(mHeights.size() > mWidth);
  assert(mHeights.size() == mWidth*mHeight);
  std::vector<HeightType>::iterator it1 = mHeights.begin();
  std::vector<HeightType>::iterator it2 = mHeights.end() - mWidth;
  // int division mHeight/2 ensures odd row counts skip middle row
  for (size_t r = 0; r < mHeight / 2; ++r) 
  {
    it1 = std::swap_ranges(it2, it2+mWidth, it1);
    it2 -= mWidth;
  }
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
size_t HeightmapShape::getWidth() const
{
  return mWidth;
}

//==============================================================================
size_t HeightmapShape::getHeight() const
{
  return mHeight;
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
void HeightmapShape::computeBoundingBox(Eigen::Vector3d& min,
                                        Eigen::Vector3d& max) const
{
  const double dimX = mWidth * mScale.x();
  const double dimY = mHeight * mScale.y();
  const double dimZ = (mMaxHeight - mMinHeight) * mScale.z();
  min = Eigen::Vector3d(-dimX*0.5, -dimY*0.5, mMinHeight*mScale.z());
  max = min + Eigen::Vector3d(dimX, dimY, dimZ);
  dtdbg << "Bounding box of height map: min = {"
        << min.x() << ", " << min.y() << ", " << min.z() << "}, max = {"
        << max.x() << ", " << max.y() << ", " << max.z() << "}" << std::endl;
}

//==============================================================================
void HeightmapShape::updateBoundingBox() const
{
  Eigen::Vector3d min, max;
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

}  // namespace dynamics
}  // namespace dart
