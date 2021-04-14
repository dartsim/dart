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

#include "dart/dynamics/VoxelGridShape.hpp"

#if DART_HAVE_OCTOMAP

#  include "dart/common/Console.hpp"
#  include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
octomap::point3d toPoint3f(const Eigen::Vector3f& point)
{
  return octomap::point3d(point.x(), point.y(), point.z());
}

//==============================================================================
octomap::point3d toPoint3d(const Eigen::Vector3d& point)
{
  return toPoint3f(point.cast<float>());
}

//==============================================================================
octomath::Quaternion toQuaternionf(const Eigen::Matrix3f& rotation)
{
  Eigen::Quaternionf quat(rotation);
  return octomath::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
}

//==============================================================================
octomath::Quaternion toQuaterniond(const Eigen::Matrix3d& rotation)
{
  return toQuaternionf(rotation.cast<float>());
}

//==============================================================================
octomap::pose6d toPose6d(const Eigen::Isometry3d& frame)
{
  return octomap::pose6d(
      toPoint3d(frame.translation()), toQuaterniond(frame.linear()));
}

} // namespace

//==============================================================================
VoxelGridShape::VoxelGridShape(double resolution) : Shape()
{
  setOctree(fcl_make_shared<octomap::OcTree>(resolution));

  mVariance = DYNAMIC_ELEMENTS;
}

//==============================================================================
VoxelGridShape::VoxelGridShape(fcl_shared_ptr<octomap::OcTree> octree) : Shape()
{
  if (!octree)
  {
    dtwarn << "[VoxelGridShape] Attempting to assign null octree. Creating an "
           << "empty octree with resolution 0.01 instead.\n";
    setOctree(fcl_make_shared<octomap::OcTree>(0.01));
    return;
  }

  setOctree(std::move(octree));
}

//==============================================================================
const std::string& VoxelGridShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& VoxelGridShape::getStaticType()
{
  static const std::string type("VoxelGridShape");
  return type;
}

//==============================================================================
void VoxelGridShape::setOctree(fcl_shared_ptr<octomap::OcTree> octree)
{
  if (!octree)
  {
    dtwarn
        << "[VoxelGridShape] Attempting to assign null octree. Ignoring this "
        << "query.\n";
    return;
  }

  if (octree == mOctree)
    return;

  mOctree = std::move(octree);

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
fcl_shared_ptr<octomap::OcTree> VoxelGridShape::getOctree()
{
  return mOctree;
}

//==============================================================================
fcl_shared_ptr<const octomap::OcTree> VoxelGridShape::getOctree() const
{
  return mOctree;
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const Eigen::Vector3d& point, bool occupied)
{
  mOctree->updateNode(toPoint3d(point), occupied);

  incrementVersion();
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  mOctree->insertRay(toPoint3d(from), toPoint3d(to));

  incrementVersion();
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const octomap::Pointcloud& pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Frame* relativeTo)
{
  if (relativeTo == Frame::World())
  {
    mOctree->insertPointCloud(pointCloud, toPoint3d(sensorOrigin));
    incrementVersion();
  }
  else
  {
    updateOccupancy(pointCloud, sensorOrigin, relativeTo->getWorldTransform());
  }
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const octomap::Pointcloud& pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Eigen::Isometry3d& relativeTo)
{
  mOctree->insertPointCloud(
      pointCloud, toPoint3d(sensorOrigin), toPose6d(relativeTo));

  incrementVersion();
}

//==============================================================================
double VoxelGridShape::getOccupancy(const Eigen::Vector3d& point) const
{
  const auto node = mOctree->search(point.x(), point.y(), point.z());
  if (node)
    return node->getOccupancy();
  else
    return 0.0;
}

//==============================================================================
Eigen::Matrix3d VoxelGridShape::computeInertia(double /*mass*/) const
{
  // TODO(JS): Not implemented. Do we really want to compute inertia out of
  // voxels?
  return Eigen::Matrix3d::Identity();
}

//==============================================================================
void VoxelGridShape::notifyColorUpdated(const Eigen::Vector4d& /*color*/)
{
  incrementVersion();
}

//==============================================================================
void VoxelGridShape::updateBoundingBox() const
{
  // TODO(JS): Not implemented.
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void VoxelGridShape::updateVolume() const
{
  // TODO(JS): Not implemented.
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart

#endif // DART_HAVE_OCTOMAP
