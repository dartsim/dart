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

#ifndef DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/dart/Aabb.hpp>
#include <dart/collision/dart/shapes/Shape.hpp>

#include <Eigen/Dense>

#include <limits>
#include <memory>
#include <vector>

#include <cstddef>

namespace dart {
namespace collision {

class DARTCollisionDetector;
class DARTCollisionGroup;

class DARTCollisionObject : public CollisionObject
{
public:
  friend class DARTCollisionDetector;
  friend class DARTCollisionGroup;

  const native::Shape* getNativeShape() const;

  const Eigen::Isometry3d& getNativeTransform() const;

  const native::Aabb& getNativeAabb() const;

  struct CachedSoftFace
  {
    Eigen::Vector3i indices{Eigen::Vector3i::Constant(-1)};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
    Eigen::Vector3d edge0{Eigen::Vector3d::Zero()};
    Eigen::Vector3d edge1{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    Eigen::Vector3d boundsMin{Eigen::Vector3d::Zero()};
    Eigen::Vector3d boundsMax{Eigen::Vector3d::Zero()};
    double planeOffset{0.0};
    double d00{0.0};
    double d01{0.0};
    double d11{0.0};
    double denom{0.0};
    bool valid{false};
  };

  struct CachedSoftFaceBvhNode
  {
    Eigen::Vector3d boundsMin{Eigen::Vector3d::Zero()};
    Eigen::Vector3d boundsMax{Eigen::Vector3d::Zero()};
    int left{-1};
    int right{-1};
    int first{0};
    int count{0};
  };

  bool isSoftMeshShape() const;

  const std::vector<Eigen::Vector3d>& getCachedSoftLocalVertices() const;

  const std::vector<int>& getCachedSoftFirstFaceByPointMass() const;

  const std::vector<CachedSoftFace>& getCachedSoftFaces() const;

  const std::vector<CachedSoftFaceBvhNode>& getCachedSoftFaceBvhNodes() const;

  const std::vector<int>& getCachedSoftFaceBvhIndices() const;

protected:
  DARTCollisionObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame);

  // Documentation inherited
  void updateEngineData() override;

private:
  void rebuildNativeShape();

  void refreshSoftMeshCache();

  void refreshSoftFaceCacheIfNeeded();

  void refreshSoftFaceBvhCache();

  void refreshSoftFaceBvhBounds();

  std::unique_ptr<native::Shape> mNativeShape;
  Eigen::Isometry3d mNativeTransform{Eigen::Isometry3d::Identity()};
  /// Local-space AABB used by the broadphase. Rigid native shapes refresh this
  /// only when the native shape is rebuilt; soft meshes refresh it from point
  /// masses every engine update.
  native::Aabb mNativeLocalAabb;
  native::Aabb mNativeAabb;
  std::size_t mLastKnownShapeId{std::numeric_limits<std::size_t>::max()};
  std::size_t mLastKnownShapeVersion{0u};
  bool mHasNativeAabb{false};
  bool mIsSoftMeshShape{false};
  std::size_t mCachedSoftBodyNodeVersion{
      std::numeric_limits<std::size_t>::max()};
  std::vector<Eigen::Vector3d> mCachedSoftLocalVertices;
  std::vector<int> mCachedSoftFirstFaceByPointMass;
  std::vector<CachedSoftFace> mCachedSoftFaces;
  std::vector<CachedSoftFaceBvhNode> mCachedSoftFaceBvhNodes;
  std::vector<int> mCachedSoftFaceBvhIndices;
  bool mCachedSoftFacesDirty{false};
  bool mCachedSoftFaceTopologyDirty{false};
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_
