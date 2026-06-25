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

#include <cstddef>

namespace dart {
namespace collision {

class CollisionObject;

class DARTCollisionObject : public CollisionObject
{
public:
  friend class DARTCollisionDetector;

  enum class CachedShapeKind
  {
    Unknown,
    Sphere,
    SphereEllipsoid,
    Box,
    Cylinder,
    Capsule,
    Plane,
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

  /// Refresh shape metadata used by the DART-native collision backend.
  void refreshShapeCacheForCollision();

protected:
  // Documentation inherited
  void updateEngineData() override;

private:
  void refreshShapeCache();

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
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONOBJECT_HPP_
