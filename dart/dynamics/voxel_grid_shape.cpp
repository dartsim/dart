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

#include "dart/dynamics/voxel_grid_shape.hpp"

#include "dart/common/logging.hpp"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
std::shared_ptr<SparseOccupancyGrid> makeDefaultGrid()
{
  return std::make_shared<SparseOccupancyGrid>(0.01);
}

#if DART_HAVE_OCTOMAP

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
Eigen::Vector3d toVector3d(const octomap::point3d& point)
{
  return Eigen::Vector3f(point.x(), point.y(), point.z()).cast<double>();
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
std::vector<Eigen::Vector3d> toVector3d(const octomap::Pointcloud& pointCloud)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(pointCloud.size());
  for (const auto& point : pointCloud) {
    points.emplace_back(toVector3d(point));
  }

  return points;
}

//==============================================================================
void importOctree(SparseOccupancyGrid& grid, const octomap::OcTree& octree)
{
  grid.clear();
  for (auto it = octree.begin_leafs(), end = octree.end_leafs(); it != end;
       ++it) {
    const Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
    grid.setOccupancy(grid.worldToCell(center), it->getOccupancy());
  }
}

#endif // DART_HAVE_OCTOMAP

} // namespace

//==============================================================================
VoxelGridShape::VoxelGridShape(double resolution) : Shape()
{
  setOccupancyGrid(std::make_shared<SparseOccupancyGrid>(resolution));

  mVariance = DYNAMIC_ELEMENTS;
}

//==============================================================================
VoxelGridShape::VoxelGridShape(std::shared_ptr<SparseOccupancyGrid> grid)
  : Shape()
{
  if (!grid) {
    DART_WARN(
        "[VoxelGridShape] Attempting to assign null occupancy grid. Creating "
        "an empty grid with resolution 0.01 instead.");
    grid = makeDefaultGrid();
  }

  setOccupancyGrid(std::move(grid));

  mVariance = DYNAMIC_ELEMENTS;
}

#if DART_HAVE_OCTOMAP
//==============================================================================
VoxelGridShape::VoxelGridShape(std::shared_ptr<octomap::OcTree> octree)
  : Shape()
{
  if (!octree) {
    DART_WARN(
        "[VoxelGridShape] Attempting to assign null octree. Creating an empty "
        "octree with resolution 0.01 instead.");
    octree = std::make_shared<octomap::OcTree>(0.01);
  }

  setOctree(std::move(octree));

  mVariance = DYNAMIC_ELEMENTS;
}
#endif

//==============================================================================
std::string_view VoxelGridShape::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view VoxelGridShape::getStaticType()
{
  static constexpr std::string_view type = "VoxelGridShape";
  return type;
}

//==============================================================================
void VoxelGridShape::setOccupancyGrid(std::shared_ptr<SparseOccupancyGrid> grid)
{
  if (!grid) {
    DART_WARN(
        "[VoxelGridShape] Attempting to assign null occupancy grid. Ignoring "
        "this query.");
    return;
  }

  if (grid == mOccupancyGrid) {
    return;
  }

  mOccupancyGrid = std::move(grid);

#if DART_HAVE_OCTOMAP
  rebuildOctreeFromOccupancyGrid();
#endif

  markDataDirty();
}

//==============================================================================
std::shared_ptr<SparseOccupancyGrid> VoxelGridShape::getOccupancyGrid()
{
  return mOccupancyGrid;
}

//==============================================================================
std::shared_ptr<const SparseOccupancyGrid> VoxelGridShape::getOccupancyGrid()
    const
{
  return mOccupancyGrid;
}

//==============================================================================
void VoxelGridShape::notifyOccupancyGridUpdated()
{
#if DART_HAVE_OCTOMAP
  rebuildOctreeFromOccupancyGrid();
#endif

  markDataDirty();
}

#if DART_HAVE_OCTOMAP
//==============================================================================
void VoxelGridShape::setOctree(std::shared_ptr<octomap::OcTree> octree)
{
  if (!octree) {
    DART_WARN(
        "[VoxelGridShape] Attempting to assign null octree. Ignoring this "
        "query.");
    return;
  }

  if (octree == mOctree) {
    return;
  }

  mOctree = std::move(octree);
  mOccupancyGrid
      = std::make_shared<SparseOccupancyGrid>(mOctree->getResolution());
  importOctree(*mOccupancyGrid, *mOctree);

  markDataDirty();
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
#endif

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const Eigen::Vector3d& point, bool occupied)
{
  mOccupancyGrid->updateOccupancy(point, occupied);

#if DART_HAVE_OCTOMAP
  if (mOctree) {
    mOctree->updateNode(toPoint3d(point), occupied);
  }
#endif

  markDataDirty();
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  mOccupancyGrid->insertRay(from, to);

#if DART_HAVE_OCTOMAP
  if (mOctree) {
    mOctree->insertRay(toPoint3d(from), toPoint3d(to));
  }
#endif

  markDataDirty();
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    std::span<const Eigen::Vector3d> pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Frame* relativeTo,
    std::size_t numThreads)
{
  if (!relativeTo || relativeTo == Frame::World()) {
    updateOccupancy(
        pointCloud, sensorOrigin, Eigen::Isometry3d::Identity(), numThreads);
    return;
  }

  updateOccupancy(
      pointCloud, sensorOrigin, relativeTo->getWorldTransform(), numThreads);
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    std::span<const Eigen::Vector3d> pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Eigen::Isometry3d& relativeTo,
    std::size_t numThreads)
{
  mOccupancyGrid->insertPointCloud(
      pointCloud, sensorOrigin, relativeTo, numThreads);

#if DART_HAVE_OCTOMAP
  if (mOctree) {
    octomap::Pointcloud octomapPointCloud;
    for (const auto& point : pointCloud) {
      octomapPointCloud.push_back(
          static_cast<float>(point.x()),
          static_cast<float>(point.y()),
          static_cast<float>(point.z()));
    }

    mOctree->insertPointCloud(
        octomapPointCloud, toPoint3d(sensorOrigin), toPose6d(relativeTo));
  }
#endif

  markDataDirty();
}

#if DART_HAVE_OCTOMAP
//==============================================================================
void VoxelGridShape::updateOccupancy(
    const octomap::Pointcloud& pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Frame* relativeTo)
{
  if (!relativeTo || relativeTo == Frame::World()) {
    updateOccupancy(pointCloud, sensorOrigin, Eigen::Isometry3d::Identity());
    return;
  }

  updateOccupancy(pointCloud, sensorOrigin, relativeTo->getWorldTransform());
}

//==============================================================================
void VoxelGridShape::updateOccupancy(
    const octomap::Pointcloud& pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Eigen::Isometry3d& relativeTo)
{
  const auto points = toVector3d(pointCloud);
  mOccupancyGrid->insertPointCloud(points, sensorOrigin, relativeTo);

  if (mOctree) {
    mOctree->insertPointCloud(
        pointCloud, toPoint3d(sensorOrigin), toPose6d(relativeTo));
  }

  markDataDirty();
}
#endif

//==============================================================================
double VoxelGridShape::getOccupancy(const Eigen::Vector3d& point) const
{
  return mOccupancyGrid->getOccupancy(point);
}

//==============================================================================
std::vector<SparseOccupancyGrid::OccupiedCell>
VoxelGridShape::getOccupiedCells() const
{
  return mOccupancyGrid->getOccupiedCells();
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
ShapePtr VoxelGridShape::clone() const
{
  auto clone = std::make_shared<VoxelGridShape>(
      std::make_shared<SparseOccupancyGrid>(*mOccupancyGrid));

#if DART_HAVE_OCTOMAP
  if (mOctree) {
    clone->mOctree = std::make_shared<octomap::OcTree>(*mOctree);
  }
#endif

  return clone;
}

//==============================================================================
void VoxelGridShape::updateBoundingBox() const
{
  const auto cells = mOccupancyGrid->getOccupiedCells();
  if (cells.empty()) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  for (const auto& cell : cells) {
    const Eigen::Vector3d halfSize = Eigen::Vector3d::Constant(cell.size * 0.5);
    min = min.cwiseMin(cell.center - halfSize);
    max = max.cwiseMax(cell.center + halfSize);
  }

  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void VoxelGridShape::updateVolume() const
{
  mVolume = 0.0;
  for (const auto& cell : mOccupancyGrid->getOccupiedCells()) {
    mVolume += cell.size * cell.size * cell.size;
  }

  mIsVolumeDirty = false;
}

//==============================================================================
void VoxelGridShape::markDataDirty()
{
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

#if DART_HAVE_OCTOMAP
//==============================================================================
void VoxelGridShape::rebuildOctreeFromOccupancyGrid()
{
  mOctree = std::make_shared<octomap::OcTree>(mOccupancyGrid->getResolution());

  for (const auto& cell : mOccupancyGrid->getOccupiedCells()) {
    mOctree->updateNode(toPoint3d(cell.center), true);
  }
}
#endif

} // namespace dynamics
} // namespace dart
