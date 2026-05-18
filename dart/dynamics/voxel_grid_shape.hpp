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

#ifndef DART_DYNAMICS_VOXELGRIDSHAPE_HPP_
#define DART_DYNAMICS_VOXELGRIDSHAPE_HPP_

#include <dart/config.hpp>

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/sparse_occupancy_grid.hpp>

#include <dart/common/diagnostics.hpp>

#include <memory>
#include <span>
#include <vector>

#if DART_HAVE_OCTOMAP
DART_SUPPRESS_CPP_WARNING_BEGIN
  #include <octomap/octomap.h>
DART_SUPPRESS_CPP_WARNING_END
#endif // DART_HAVE_OCTOMAP

namespace dart {
namespace dynamics {

/// VoxelGridShape represents a probabilistic 3D occupancy voxel grid.
class DART_API VoxelGridShape : public Shape
{
public:
  /// Constructor.
  /// @param[in] resolution Size of voxel. Default is 0.01.
  explicit VoxelGridShape(double resolution = 0.01);

  /// Constructs from native sparse occupancy-grid storage.
  explicit VoxelGridShape(std::shared_ptr<SparseOccupancyGrid> grid);

#if DART_HAVE_OCTOMAP
  /// Constructs from an octomap::OcTree.
  /// @param[in] octree Octree.
  explicit VoxelGridShape(std::shared_ptr<octomap::OcTree> octree);
#endif

  /// Destructor.
  ~VoxelGridShape() override = default;

  // Documentation inherited.
  std::string_view getType() const override;

  /// Returns shape type for this class
  static std::string_view getStaticType();

  /// Sets native sparse occupancy-grid storage.
  void setOccupancyGrid(std::shared_ptr<SparseOccupancyGrid> grid);

  /// Returns native sparse occupancy-grid storage.
  std::shared_ptr<SparseOccupancyGrid> getOccupancyGrid();

  /// Returns native sparse occupancy-grid storage.
  std::shared_ptr<const SparseOccupancyGrid> getOccupancyGrid() const;

#if DART_HAVE_OCTOMAP
  /// Sets octree and imports its leaves into native storage.
  void setOctree(std::shared_ptr<octomap::OcTree> octree);

  /// Returns octree compatibility storage.
  std::shared_ptr<octomap::OcTree> getOctree();

  /// Returns octree compatibility storage.
  std::shared_ptr<const octomap::OcTree> getOctree() const;
#endif

  /// Updates the occupancy probability of the voxel containing @c point.
  ///
  /// Note that the probability is computed from both the current probability
  /// and the new sensor measurement.
  ///
  /// @param[in] point Location of the sensor measurement.
  /// @param[in] occupied True if the location was measured occupied.
  void updateOccupancy(const Eigen::Vector3d& point, bool occupied = true);

  /// Updates occupancy from a single sensor ray.
  ///
  /// Traversed voxels are updated as free space, and the endpoint voxel is
  /// updated as occupied.
  ///
  /// @param from Origin of sensor in global coordinates.
  /// @param to Endpoint of measurement in global coordinates.
  void updateOccupancy(const Eigen::Vector3d& from, const Eigen::Vector3d& to);

  /// Updates occupancy from a point cloud.
  ///
  /// @param[in] pointCloud Point cloud relative to frame. Points represent the
  /// end points of the rays from the sensor origin.
  /// @param[in] sensorOrigin Origin of sensor relative to frame.
  /// @param[in] relativeTo Reference frame, determines transform to be
  /// applied to point cloud and sensor origin.
  void updateOccupancy(
      std::span<const Eigen::Vector3d> pointCloud,
      const Eigen::Vector3d& sensorOrigin = Eigen::Vector3d::Zero(),
      const Frame* relativeTo = Frame::World());

  /// Updates occupancy from a point cloud.
  ///
  /// @param[in] pointCloud Point cloud relative to frame. Points represent the
  /// end points of the rays from the sensor origin.
  /// @param[in] sensorOrigin Origin of sensor relative to frame.
  /// @param[in] relativeTo Reference transform applied to point cloud and
  /// sensor origin.
  void updateOccupancy(
      std::span<const Eigen::Vector3d> pointCloud,
      const Eigen::Vector3d& sensorOrigin,
      const Eigen::Isometry3d& relativeTo);

#if DART_HAVE_OCTOMAP
  /// Updates occupancy from an octomap point cloud.
  ///
  /// @param[in] pointCloud Point cloud relative to frame. Points represent the
  /// end points of the rays from the sensor origin.
  /// @param[in] sensorOrigin Origin of sensor relative to frame.
  /// @param[in] relativeTo Reference frame, determines transform to be
  /// applied to point cloud and sensor origin.
  void updateOccupancy(
      const octomap::Pointcloud& pointCloud,
      const Eigen::Vector3d& sensorOrigin = Eigen::Vector3d::Zero(),
      const Frame* relativeTo = Frame::World());

  /// Updates occupancy from an octomap point cloud.
  ///
  /// @param[in] pointCloud Point cloud relative to frame. Points represent the
  /// end points of the rays from the sensor origin.
  /// @param[in] sensorOrigin Origin of sensor relative to frame.
  /// @param[in] relativeTo Reference transform applied to point cloud and
  /// sensor origin.
  void updateOccupancy(
      const octomap::Pointcloud& pointCloud,
      const Eigen::Vector3d& sensorOrigin,
      const Eigen::Isometry3d& relativeTo);
#endif

  /// Returns occupancy probability of the voxel that contains @c point.
  double getOccupancy(const Eigen::Vector3d& point) const;

  /// Returns occupied cells with centers and probabilities.
  std::vector<SparseOccupancyGrid::OccupiedCell> getOccupiedCells() const;

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

  // Documentation inherited.
  void notifyColorUpdated(const Eigen::Vector4d& color) override;

  // Documentation inherited.
  ShapePtr clone() const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  void markDataDirty();

#if DART_HAVE_OCTOMAP
  void rebuildOctreeFromOccupancyGrid();
#endif

  /// Native sparse occupancy-grid storage.
  std::shared_ptr<SparseOccupancyGrid> mOccupancyGrid;

#if DART_HAVE_OCTOMAP
  /// Octree compatibility storage.
  std::shared_ptr<octomap::OcTree> mOctree;
#endif
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_VOXELGRIDSHAPE_HPP_
