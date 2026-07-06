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

#include <dart/dynamics/SmartPointer.hpp>

#include <Eigen/Dense>

#include <limits>
#include <string>
#include <vector>

#include <cstddef>

namespace dart {
namespace collision {

class CollisionObject;

class DARTCollisionObject : public CollisionObject
{
public:
  friend class DARTCollisionDetector;
  friend class DARTCollisionGroup;

  enum class CachedShapeKind
  {
    Unknown,
    Sphere,
    SphereEllipsoid,
    Box,
    Cylinder,
    Capsule,
    SoftMesh,
    Plane,
  };

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

protected:
  /// Constructor
  DARTCollisionObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame);

public:
  const dynamics::Shape* getCachedShape() const;

  const std::string& getCachedShapeType() const;

  CachedShapeKind getCachedShapeKind() const;

  const Eigen::Vector3d& getCachedLocalBoundsMin() const;

  const Eigen::Vector3d& getCachedLocalBoundsMax() const;

  /// Returns the center of the cached local bounding box.
  const Eigen::Vector3d& getCachedLocalBoundsCenter() const;

  /// Returns the half-extents of the cached local bounding box.
  const Eigen::Vector3d& getCachedLocalBoundsHalfExtents() const;

  bool hasFiniteCachedLocalBounds() const;

  bool isCachedPlaneShape() const;

  /// Return the world transform path used by the DART-native collision backend.
  const Eigen::Isometry3d& getWorldTransformForCollision() const;

  const std::vector<Eigen::Vector3d>& getCachedSoftLocalVertices() const;

  const std::vector<int>& getCachedSoftFirstFaceByPointMass() const;

  const std::vector<CachedSoftFace>& getCachedSoftFaces() const;

  const std::vector<CachedSoftFaceBvhNode>& getCachedSoftFaceBvhNodes() const;

  const std::vector<int>& getCachedSoftFaceBvhIndices() const;

protected:
  // Documentation inherited
  void updateEngineData() override;

private:
  void refreshShapeCache();

  void refreshSoftMeshCache();

  void refreshSoftFaceBvhCache();

  dynamics::ConstShapePtr mCachedShape;
  const std::string* mCachedShapeType{nullptr};
  std::size_t mCachedShapeFrameVersion{std::numeric_limits<std::size_t>::max()};
  CachedShapeKind mCachedShapeKind{CachedShapeKind::Unknown};
  Eigen::Vector3d mCachedLocalBoundsMin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mCachedLocalBoundsMax{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mCachedLocalBoundsCenter{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mCachedLocalBoundsHalfExtents{Eigen::Vector3d::Zero()};
  bool mHasFiniteCachedLocalBounds{false};
  bool mIsCachedPlaneShape{false};
  bool mUseBodyNodeWorldTransform{false};
  std::vector<Eigen::Vector3d> mCachedSoftLocalVertices;
  std::vector<int> mCachedSoftFirstFaceByPointMass;
  std::vector<CachedSoftFace> mCachedSoftFaces;
  std::vector<CachedSoftFaceBvhNode> mCachedSoftFaceBvhNodes;
  std::vector<int> mCachedSoftFaceBvhIndices;
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_
