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

#ifndef DART_DYNAMICS_VOXELGRIDSHAPE_HPP_
#define DART_DYNAMICS_VOXELGRIDSHAPE_HPP_

#include "dart/config.hpp"

#if HAVE_OCTOMAP

#include <octomap/octomap.h>
#include "dart/collision/fcl/BackwardCompatibility.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

/// VoxelGridShape represents a probabilistic 3D occupancy voxel grid.
class VoxelGridShape : public Shape
{
public:
  /// Constructor.
  /// \param[in] resolution Size of voxel. Default is 0.01.
  explicit VoxelGridShape(double resolution = 0.01);

  /// Constructs from a octomap::OcTree.
  /// \param[in] octree Octree.
  explicit VoxelGridShape(fcl_shared_ptr<octomap::OcTree> octree);

  /// Destructor.
  ~VoxelGridShape() override = default;

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  /// Sets octree.
  void setOctree(fcl_shared_ptr<octomap::OcTree> octree);

  /// Returns octree.
  fcl_shared_ptr<octomap::OcTree> getOctree();

  /// Returns octree.
  fcl_shared_ptr<const octomap::OcTree> getOctree() const;

  /// Updates the occupancy probability of the voxels where \c point is located.
  ///
  /// Note that the probability is computed by both of the current probability
  /// and the new sensor measurement.
  ///
  /// \param[in] point Location of the sensor measurement.
  /// \param[in] occupied True if the location was measured occupied.
  void updateOccupancy(const Eigen::Vector3d& point, bool occupied = true);

  /// Updates the occupancy probability of the voxels given sensor measurement,
  /// which is a single ray.
  ///
  /// Note that the probability is computed by both of the current probability
  /// and the new sensor measurement.
  ///
  /// If you have a ray cloud, then using updateOccupancy(PointClout, ...) is
  /// more efficient.
  ///
  /// \param from Origin of sensor in global coordinates.
  /// \param to Endpoint of measurement in global coordinates.
  void updateOccupancy(const Eigen::Vector3d& from, const Eigen::Vector3d& to);

  /// Updates the occupancy probability of the voxels given sensor measurement.
  ///
  /// Note that the probability is computed by both of the current probability
  /// and the new sensor measurement.
  ///
  /// The voxels where the ray endpoints are located will increase the
  /// probabilities because that’s where the rays hit objects. On the other
  /// hand, the voxels that the rays pass through will decrease the
  /// probabilities because it means there are no objects.
  ///
  /// \param[in] pointCloud Point cloud relative to frame. Points represent the
  /// end points of the rays from the sensor origin.
  /// \param[in] sensorOrigin Origin of sensor relative to frame.
  /// \param[in] relativeTo Reference frame, determines transform to be
  /// applied to point cloud and sensor origin.
  void updateOccupancy(
      const octomap::Pointcloud& pointCloud,
      const Eigen::Vector3d& sensorOrigin = Eigen::Vector3d::Zero(),
      const Frame* relativeTo = Frame::World());

  /// Updates the occupancy probability of the voxels given sensor measurement.
  ///
  /// Note that the probability is computed by both of the current probability
  /// and the new sensor measurement.
  ///
  /// The voxels where the ray endpoints are located will increase the
  /// probabilities because that’s where the rays hit objects. On the other
  /// hand, the voxels that the rays pass through will decrease the
  /// probabilities because it means there are no objects.
  ///
  /// \param[in] pointCloud Point cloud relative to frame. Points represent the
  /// end points of the rays from the sensor origin.
  /// \param[in] sensorOrigin Origin of sensor relative to frame.
  /// \param[in] relativeTo Reference frame, determines transform to be
  /// applied to point cloud and sensor origin.
  void updateOccupancy(
      const octomap::Pointcloud& pointCloud,
      const Eigen::Vector3d& sensorOrigin,
      const Eigen::Isometry3d& relativeTo);

  /// Returns occupancy probability of a node that contains \c point.
  double getOccupancy(const Eigen::Vector3d& point) const;

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

  /// Octree.
  fcl_shared_ptr<octomap::OcTree> mOctree;
  // TODO(JS): Use std::shared_ptr once we drop supporting FCL (< 0.5)
};

} // namespace dynamics
} // namespace dart

#endif // HAVE_OCTOMAP

#endif // DART_DYNAMICS_VOXELGRIDSHAPE_HPP_
