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

#include "dart/dynamics/VoxelGridShape.hpp"

#if HAVE_OCTOMAP

  #include "dart/common/Console.hpp"
  #include "dart/math/Helpers.hpp"

  #include <cmath>

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

//==============================================================================
bool rayFitsKeyBuffer(
    const octomap::OcTree& octree,
    const octomap::point3d& from,
    const octomap::point3d& to)
{
  const double factor = 1.0 / octree.getResolution();
  double cells = 0.0;
  for (unsigned int i = 0; i < 3; ++i) {
    cells
        += std::abs(std::floor(factor * to(i)) - std::floor(factor * from(i)));
  }

  // OctoMap 1.10.1 computeRayKeys advances one axis per key, includes the
  // origin and excludes the endpoint. Leave eight keys for up to three extra
  // keys from float rounding, its strict size() < sizeMax() - 1 assertion,
  // and spare capacity (KeyRay holds 100,000 keys).
  // https://github.com/OctoMap/octomap/blob/v1.10.1/octomap/include/octomap/OcTreeBaseImpl.hxx#L542-L646
  // https://github.com/OctoMap/octomap/blob/v1.10.1/octomap/include/octomap/OcTreeKey.h#L139-L186
  static const std::size_t maxKeys = octomap::KeyRay().sizeMax();
  if (!(cells + 8.0 <= static_cast<double>(maxKeys)))
    return false;
  if (cells == 0.0)
    return true;

  // Float normalization must remain finite for the traversal bound to hold.
  const float length = static_cast<float>((to - from).norm());
  return std::isfinite(length) && length > 0.0f;
}

//==============================================================================
void insertPointCloud(
    octomap::OcTree& octree,
    const octomap::Pointcloud& pointCloud,
    const octomap::point3d& sensorOrigin,
    const octomap::pose6d* frameOrigin = nullptr)
{
  const auto origin
      = frameOrigin ? frameOrigin->transform(sensorOrigin) : sensorOrigin;
  const auto fits = [&](const octomap::point3d& point) {
    return rayFitsKeyBuffer(
        octree, origin, frameOrigin ? frameOrigin->transform(point) : point);
  };
  std::size_t skipped = 0;
  for (const auto& point : pointCloud) {
    if (!fits(point))
      ++skipped;
  }

  octomap::Pointcloud filtered;
  if (skipped > 0) {
    filtered.reserve(pointCloud.size() - skipped);
    for (const auto& point : pointCloud) {
      if (fits(point))
        filtered.push_back(point);
    }
    dtwarn << "[VoxelGridShape] Skipped " << skipped << " of "
           << pointCloud.size()
           << " rays that cannot be traced safely in OctoMap's key buffer.\n";
  }
  const auto& scan = skipped > 0 ? filtered : pointCloud;
  if (frameOrigin)
    octree.insertPointCloud(scan, sensorOrigin, *frameOrigin);
  else
    octree.insertPointCloud(scan, sensorOrigin);
}

} // namespace

//==============================================================================
VoxelGridShape::VoxelGridShape(double resolution) : Shape()
{
  setOctree(std::make_shared<octomap::OcTree>(resolution));

  mVariance = DYNAMIC_ELEMENTS;
}

//==============================================================================
VoxelGridShape::VoxelGridShape(std::shared_ptr<octomap::OcTree> octree)
  : Shape()
{
  if (!octree) {
    dtwarn << "[VoxelGridShape] Attempting to assign null octree. Creating an "
           << "empty octree with resolution 0.01 instead.\n";
    setOctree(std::make_shared<octomap::OcTree>(0.01));
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
void VoxelGridShape::setOctree(std::shared_ptr<octomap::OcTree> octree)
{
  if (!octree) {
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
std::shared_ptr<octomap::OcTree> VoxelGridShape::getOctree()
{
  return mOctree;
}

//==============================================================================
std::shared_ptr<const octomap::OcTree> VoxelGridShape::getOctree() const
{
  return mOctree;
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const Eigen::Vector3d& point, bool occupied)
{
  // OctoMap's double overload addresses the cell that getOccupancy() reads.
  mOctree->updateNode(point.x(), point.y(), point.z(), occupied);

  incrementVersion();
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  const auto origin = toPoint3d(from);
  const auto end = toPoint3d(to);
  if (rayFitsKeyBuffer(*mOctree, origin, end))
    mOctree->insertRay(origin, end);
  else
    dtwarn << "[VoxelGridShape] Skipping a ray that cannot be traced safely "
           << "in OctoMap's key buffer.\n";

  incrementVersion();
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const octomap::Pointcloud& pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Frame* relativeTo)
{
  if (relativeTo == Frame::World()) {
    insertPointCloud(*mOctree, pointCloud, toPoint3d(sensorOrigin));
    incrementVersion();
  } else {
    updateOccupancy(pointCloud, sensorOrigin, relativeTo->getWorldTransform());
  }
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const octomap::Pointcloud& pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Eigen::Isometry3d& relativeTo)
{
  const auto pose = toPose6d(relativeTo);
  insertPointCloud(*mOctree, pointCloud, toPoint3d(sensorOrigin), &pose);

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
  incrementVisualVersion();
}

//==============================================================================
ShapePtr VoxelGridShape::clone() const
{
  return std::make_shared<VoxelGridShape>(
      std::make_shared<octomap::OcTree>(*mOctree));
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

#endif // HAVE_OCTOMAP
