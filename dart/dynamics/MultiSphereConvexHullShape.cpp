/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include "dart/dynamics/MultiSphereConvexHullShape.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BoxShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
MultiSphereConvexHullShape::MultiSphereConvexHullShape(const Spheres& spheres)
  : Shape(MULTISPHERE)
{
  addSpheres(spheres);
}

//==============================================================================
MultiSphereConvexHullShape::~MultiSphereConvexHullShape()
{
  // Do nothing
}

//==============================================================================
const std::string& MultiSphereConvexHullShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& MultiSphereConvexHullShape::getStaticType()
{
  static const std::string type("MultiSphereConvexHullShape");
  return type;
}

//==============================================================================
void MultiSphereConvexHullShape::addSpheres(const MultiSphereConvexHullShape::Spheres& spheres)
{
  mSpheres.insert(mSpheres.end(), spheres.begin(), spheres.end());

  updateBoundingBoxDim();
  mIsVolumeDirty = true;
}

//==============================================================================
void MultiSphereConvexHullShape::addSphere(const MultiSphereConvexHullShape::Sphere& sphere)
{
  mSpheres.push_back(sphere);

  updateBoundingBoxDim();
  mIsVolumeDirty = true;
}

//==============================================================================
void MultiSphereConvexHullShape::addSphere(double radius, const Eigen::Vector3d& position)
{
  addSphere(std::make_pair(radius, position));
}

//==============================================================================
void MultiSphereConvexHullShape::removeAllSpheres()
{
  mSpheres.clear();

  updateBoundingBoxDim();
  mIsVolumeDirty = true;
}

//==============================================================================
std::size_t MultiSphereConvexHullShape::getNumSpheres() const
{
  return mSpheres.size();
}

//==============================================================================
const MultiSphereConvexHullShape::Spheres& MultiSphereConvexHullShape::getSpheres() const
{
  return mSpheres;
}

//==============================================================================
Eigen::Matrix3d MultiSphereConvexHullShape::computeInertia(double mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(mBoundingBox.computeFullExtents(), mass);
}

//==============================================================================
void MultiSphereConvexHullShape::updateVolume() const
{
  mVolume = BoxShape::computeVolume(mBoundingBox.computeFullExtents());
  mIsVolumeDirty = false;
}

//==============================================================================
void MultiSphereConvexHullShape::updateBoundingBoxDim()
{
  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d max = -min;

  for (const auto& sphere : mSpheres)
  {
    const auto& radius = sphere.first;
    const Eigen::Vector3d& pos = sphere.second;
    const Eigen::Vector3d extent = Eigen::Vector3d::Constant(radius);

    min = min.cwiseMin(pos - extent);
    max = max.cwiseMax(pos + extent);
  }

  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);
}

}  // namespace dynamics
}  // namespace dart
