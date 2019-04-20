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

#include "dart/dynamics/PointCloudShape.hpp"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
Eigen::Vector3d toVector3d(const octomap::point3d& point)
{
  return Eigen::Vector3f(point.x(), point.y(), point.z()).cast<double>();
}

} // namespace

//==============================================================================
PointCloudShape::PointCloudShape(double visualSize)
  : Shape(), mVisualSize(visualSize)
{
  mVariance = DYNAMIC_ELEMENTS;
}

//==============================================================================
const std::string& PointCloudShape::getType() const
{
  return getStaticType();
}

//==============================================================================
Eigen::Matrix3d PointCloudShape::computeInertia(double /*mass*/) const
{
  return Eigen::Matrix3d::Identity();
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
void PointCloudShape::addPoint(const Eigen::Vector3d& point)
{
  mPoints.emplace_back(point);
  incrementVersion();
}

//==============================================================================
void PointCloudShape::addPoint(const std::vector<Eigen::Vector3d>& points)
{
  mPoints.reserve(mPoints.size() + points.size());
  for (const auto& point : points)
    mPoints.emplace_back(point);
  incrementVersion();
}

//==============================================================================
void PointCloudShape::setPoint(const std::vector<Eigen::Vector3d>& points)
{
  mPoints = points;
  incrementVersion();
}

#if HAVE_OCTOMAP
//==============================================================================
void PointCloudShape::setPoints(octomap::Pointcloud& pointCloud)
{
  mPoints.resize(pointCloud.size());
  for (auto i = 0u; i < mPoints.size(); ++i)
    mPoints[i] = toVector3d(pointCloud[i]);
  incrementVersion();
}

//==============================================================================
void PointCloudShape::addPoints(octomap::Pointcloud& pointCloud)
{
  mPoints.reserve(mPoints.size() + pointCloud.size());
  for (const auto& point : pointCloud)
    mPoints.emplace_back(toVector3d(point));
  incrementVersion();
}
#endif

//==============================================================================
const std::vector<Eigen::Vector3d>& PointCloudShape::getPoints() const
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
void PointCloudShape::setVisualSize(double size)
{
  mVisualSize = size;
}

//==============================================================================
double PointCloudShape::getVisualSize() const
{
  return mVisualSize;
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
  if (mPoints.empty())
  {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  for (const auto& vertex : mPoints)
  {
    min = min.cwiseMin(vertex);
    max = max.cwiseMax(vertex);
  }

  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);

  mIsBoundingBoxDirty = false;
}

} // namespace dynamics
} // namespace dart
