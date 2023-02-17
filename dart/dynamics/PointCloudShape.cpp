/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/dynamics/PointCloudShape.hpp"

#include "dart/common/Console.hpp"

namespace dart {
namespace dynamics {

#if DART_HAS_OCTOMAP

namespace {

//==============================================================================
math::Vector3d toVector3d(const octomap::point3d& point)
{
  return math::Vector3f(point.x(), point.y(), point.z()).cast<double>();
}

} // namespace

#endif // DART_HAS_OCTOMAP

//==============================================================================
PointCloudShape::PointCloudShape(double visualSize)
  : Shape(),
    mPointShapeType(BOX),
    mColorMode(USE_SHAPE_COLOR),
    mVisualSize(visualSize)
{
  // Do nothing
}

//==============================================================================
const std::string& PointCloudShape::getType() const
{
  return getStaticType();
}

//==============================================================================
math::Matrix3d PointCloudShape::computeInertia(double /*mass*/) const
{
  return math::Matrix3d::Identity();
}

//==============================================================================
const std::string& PointCloudShape::getStaticType()
{
  static const std::string type("PointCloudShape");
  return type;
}

//==============================================================================
void PointCloudShape::reserve(std::size_t size)
{
  mPoints.reserve(size);
}

//==============================================================================
void PointCloudShape::addPoint(const math::Vector3d& point)
{
  mPoints.emplace_back(point);
  incrementVersion();
}

//==============================================================================
void PointCloudShape::addPoint(const std::vector<math::Vector3d>& points)
{
  mPoints.reserve(mPoints.size() + points.size());
  for (const auto& point : points)
    mPoints.emplace_back(point);
  incrementVersion();
}

//==============================================================================
void PointCloudShape::setPoint(const std::vector<math::Vector3d>& points)
{
  mPoints = points;
  incrementVersion();
}

#if DART_HAS_OCTOMAP
//==============================================================================
void PointCloudShape::setPoints(const ::octomap::Pointcloud& pointCloud)
{
  mPoints.resize(pointCloud.size());
  for (auto i = 0u; i < mPoints.size(); ++i)
    mPoints[i] = toVector3d(pointCloud[i]);
  incrementVersion();
}

//==============================================================================
void PointCloudShape::addPoints(const ::octomap::Pointcloud& pointCloud)
{
  mPoints.reserve(mPoints.size() + pointCloud.size());
  for (const auto& point : pointCloud)
    mPoints.emplace_back(toVector3d(point));
  incrementVersion();
}
#endif

//==============================================================================
const std::vector<math::Vector3d>& PointCloudShape::getPoints() const
{
  return mPoints;
}

//==============================================================================
std::size_t PointCloudShape::getNumPoints() const
{
  return mPoints.size();
}

//==============================================================================
void PointCloudShape::removeAllPoints()
{
  mPoints.clear();
}

//==============================================================================
void PointCloudShape::setPointShapeType(PointCloudShape::PointShapeType type)
{
  if (mPointShapeType == type)
    return;

  mPointShapeType = type;
  incrementVersion();
}

//==============================================================================
PointCloudShape::PointShapeType PointCloudShape::getPointShapeType() const
{
  return mPointShapeType;
}

//==============================================================================
void PointCloudShape::setColorMode(PointCloudShape::ColorMode mode)
{
  mColorMode = mode;
}

//==============================================================================
PointCloudShape::ColorMode PointCloudShape::getColorMode() const
{
  return mColorMode;
}

//==============================================================================
void PointCloudShape::setOverallColor(const math::Vector4d& color)
{
  mColors.resize(1);
  mColors[0] = color;
}

//==============================================================================
math::Vector4d PointCloudShape::getOverallColor() const
{
  if (mColors.empty()) {
    dtwarn << "[PointCloudShape] Attempt to get the overall color when the "
           << "color array is empty. Returning (RGBA: [0.5, 0.5, 0.5, 0.5]) "
           << "color\n";
    return math::Vector4d(0.5, 0.5, 0.5, 0.5);
  }

  if (mColors.size() > 1) {
    dtwarn << "[PointCloudShape] Attempting to get the overal color when the "
           << "color array contains more than one color. This is potentially "
           << "an error. Returning the first color in the color array.\n";
  }

  return mColors[0];
}

//==============================================================================
void PointCloudShape::setColors(const std::vector<math::Vector4d>& colors)
{
  mColors = colors;
}

//==============================================================================
const std::vector<math::Vector4d>& PointCloudShape::getColors() const
{
  return mColors;
}

//==============================================================================
void PointCloudShape::setVisualSize(double size)
{
  mVisualSize = size;
  incrementVersion();
}

//==============================================================================
double PointCloudShape::getVisualSize() const
{
  return mVisualSize;
}

//==============================================================================
void PointCloudShape::notifyColorUpdated(const math::Vector4d& /*color*/)
{
  incrementVersion();
}

//==============================================================================
ShapePtr PointCloudShape::clone() const
{
  auto new_shape = std::make_shared<PointCloudShape>(mVisualSize);
  new_shape->mPoints = mPoints;
  new_shape->mPointShapeType = mPointShapeType;
  new_shape->mColorMode = mColorMode;
  new_shape->mColors = mColors;

  return new_shape;
}

//==============================================================================
void PointCloudShape::updateVolume() const
{
  mVolume = 0.0;
  mIsVolumeDirty = false;
}

//==============================================================================
void PointCloudShape::updateBoundingBox() const
{
  if (mPoints.empty()) {
    mBoundingBox.setMin(math::Vector3d::Zero());
    mBoundingBox.setMax(math::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  math::Vector3d min = math::Vector3d::Constant(math::inf<double>());
  math::Vector3d max = math::Vector3d::Constant(-math::inf<double>());

  for (const auto& vertex : mPoints) {
    min = min.cwiseMin(vertex);
    max = max.cwiseMax(vertex);
  }

  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);

  mIsBoundingBoxDirty = false;
}

} // namespace dynamics
} // namespace dart
