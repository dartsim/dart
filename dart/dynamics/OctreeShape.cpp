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

#include "dart/dynamics/OctreeShape.hpp"

#include "dart/math/Helpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
OctreeShape::OctreeShape(double resolution) : Shape()
{
  setOctree(std::make_shared<octomap::OcTree>(resolution));
}

//==============================================================================
OctreeShape::OctreeShape(std::shared_ptr<octomap::OcTree> octree) : Shape()
{
  setOctree(std::move(octree));
}

//==============================================================================
const std::string& OctreeShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& OctreeShape::getStaticType()
{
  static const std::string type("OctreeShape");
  return type;
}

//==============================================================================
void OctreeShape::setOctree(std::shared_ptr<octomap::OcTree> octree)
{
  if (octree == mOctree)
    return;

  mOctree = std::move(octree);

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
std::shared_ptr<octomap::OcTree> OctreeShape::getOctree()
{
  return mOctree;
}

//==============================================================================
std::shared_ptr<const octomap::OcTree> OctreeShape::getOctree() const
{
  return mOctree;
}

//==============================================================================
void OctreeShape::setOccupancy(const Eigen::Vector3d& point, bool occupy)
{
  if (!mOctree)
    return;

  mOctree->updateNode(point.x(), point.y(), point.z(), occupy);
}

//==============================================================================
void OctreeShape::occupy(const Eigen::Vector3d& point)
{
  setOccupancy(point, true);
}

//==============================================================================
void OctreeShape::unoccupy(const Eigen::Vector3d& point)
{
  setOccupancy(point, false);
}

//==============================================================================
Eigen::Matrix3d OctreeShape::computeInertia(double /*mass*/) const
{
  // TODO(JS): Not implemented. Do we really want to compute inertia out of
  // voxels?
  return Eigen::Matrix3d::Identity();
}

//==============================================================================
void OctreeShape::updateBoundingBox() const
{
  // TODO(JS): Not implemented.
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void OctreeShape::updateVolume() const
{
  // TODO(JS): Not implemented.
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart
