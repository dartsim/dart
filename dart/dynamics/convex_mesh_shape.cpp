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

#include "dart/dynamics/convex_mesh_shape.hpp"

#include "dart/dynamics/box_shape.hpp"

#include <limits>

namespace dart {
namespace dynamics {

//==============================================================================
ConvexMeshShape::ConvexMeshShape(const std::shared_ptr<TriMeshType>& mesh)
  : Shape(CONVEX_MESH), mMesh(mesh ? mesh : std::make_shared<TriMeshType>())
{
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
ConvexMeshShape::ConvexMeshShape(
    const Vertices& vertices, const Triangles& triangles)
  : Shape(CONVEX_MESH)
{
  auto mesh = std::make_shared<TriMeshType>();
  mesh->setTriangles(vertices, triangles);
  mMesh = mesh;

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
}

//==============================================================================
std::string_view ConvexMeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view ConvexMeshShape::getStaticType()
{
  static constexpr std::string_view type = "ConvexMeshShape";
  return type;
}

//==============================================================================
const std::shared_ptr<ConvexMeshShape::TriMeshType>& ConvexMeshShape::getMesh()
    const
{
  return mMesh;
}

//==============================================================================
void ConvexMeshShape::setMesh(const std::shared_ptr<TriMeshType>& mesh)
{
  mMesh = mesh;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
  incrementVersion();
}

//==============================================================================
std::shared_ptr<ConvexMeshShape> ConvexMeshShape::fromMesh(
    const std::shared_ptr<TriMeshType>& mesh, bool computeHull)
{
  if (!mesh)
    return std::make_shared<ConvexMeshShape>(std::make_shared<TriMeshType>());

  if (computeHull) {
    auto hull = mesh->generateConvexHull(true);
    return std::make_shared<ConvexMeshShape>(hull);
  }

  return std::make_shared<ConvexMeshShape>(mesh);
}

//==============================================================================
ShapePtr ConvexMeshShape::clone() const
{
  std::shared_ptr<TriMeshType> clonedMesh;
  if (mMesh) {
    clonedMesh = std::make_shared<TriMeshType>(*mMesh);
  }
  return std::make_shared<ConvexMeshShape>(clonedMesh);
}

//==============================================================================
Eigen::Matrix3d ConvexMeshShape::computeInertia(double mass) const
{
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), mass);
}

//==============================================================================
void ConvexMeshShape::updateBoundingBox() const
{
  if (!mMesh || !mMesh->hasVertices()) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d minPoint
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maxPoint = -minPoint;

  for (const auto& vertex : mMesh->getVertices()) {
    minPoint = minPoint.cwiseMin(vertex);
    maxPoint = maxPoint.cwiseMax(vertex);
  }

  mBoundingBox.setMin(minPoint);
  mBoundingBox.setMax(maxPoint);
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void ConvexMeshShape::updateVolume() const
{
  const Eigen::Vector3d bounds = getBoundingBox().computeFullExtents();
  mVolume = bounds.x() * bounds.y() * bounds.z();
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart
