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

#ifndef DART_DYNAMICS_OCTREESHAPE_HPP_
#define DART_DYNAMICS_OCTREESHAPE_HPP_

#include <octomap/octomap.h>
#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

class OctreeShape : public Shape
{
public:
  /// Constructor.
  /// \param[in] resolution Size of voxel. Default is 0.01.
  explicit OctreeShape(double resolution = 0.01);

  /// Constructor.
  /// \param[in] octree Octree.
  explicit OctreeShape(std::shared_ptr<octomap::OcTree> octree);

  /// Destructor.
  virtual ~OctreeShape() = default;

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  /// Sets octree.
  void setOctree(std::shared_ptr<octomap::OcTree> octree);

  /// Returns octree.
  std::shared_ptr<octomap::OcTree> getOctree();

  /// Returns octree.
  std::shared_ptr<const octomap::OcTree> getOctree() const;

  /// Sets occupany at a point.
  void setOccupancy(const Eigen::Vector3d& point, bool occupy);

  // TODO(JS): Enable to take point cloud.

  /// Occupy a point.
  void occupy(const Eigen::Vector3d& point);

  /// Unoccupy a point.
  void unoccupy(const Eigen::Vector3d& point);

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

  /// Octree.
  std::shared_ptr<octomap::OcTree> mOctree;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_OCTREESHAPE_HPP_
