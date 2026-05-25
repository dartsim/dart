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

#ifndef DART_COLLISION_DART_DARTCOLLISIONGROUP_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONGROUP_HPP_

#include <dart/collision/collision_group.hpp>

#include <memory>

#include <cstddef>

namespace dart {
namespace collision {

namespace native {
class PersistentManifoldCache;
} // namespace native

class DartCollisionScene;

class DART_API DartCollisionGroup : public CollisionGroup
{
public:
  friend class DartCollisionDetector;

  /// Constructor
  DartCollisionGroup(const CollisionDetectorPtr& collisionDetector);

  /// Destructor
  ~DartCollisionGroup() override;

protected:
  // Documentation inherited
  void initializeEngineData() override;

  // Documentation inherited
  void addCollisionObjectToEngine(CollisionObject* object) override;

  // Documentation inherited
  void addCollisionObjectsToEngine(
      std::span<CollisionObject* const> collObjects) override;

  // Documentation inherited
  void removeCollisionObjectFromEngine(CollisionObject* object) override;

  // Documentation inherited
  void removeAllCollisionObjectsFromEngine() override;

  // Documentation inherited
  void updateCollisionGroupEngineData() override;

  bool collideSelf(
      const CollisionOption& option,
      CollisionResult* result,
      native::PersistentManifoldCache* manifoldCache);

  bool collideWith(
      DartCollisionGroup& other,
      const CollisionOption& option,
      CollisionResult* result,
      native::PersistentManifoldCache* manifoldCache);

  double distanceSelf(const DistanceOption& option, DistanceResult* result);

  double distanceWith(
      DartCollisionGroup& other,
      const DistanceOption& option,
      DistanceResult* result);

  bool raycast(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option,
      RaycastResult* result);

  bool sphereCast(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      double radius,
      const ContinuousCollisionOption& option,
      ContinuousCollisionResult* result);

  bool capsuleCast(
      const Eigen::Isometry3d& capsuleStart,
      const Eigen::Isometry3d& capsuleEnd,
      double radius,
      double height,
      const ContinuousCollisionOption& option,
      ContinuousCollisionResult* result);

  std::size_t getManifoldCacheId(CollisionObject* object) const;

protected:
  /// CollisionObjects added to this DartCollisionGroup
  std::vector<CollisionObject*> mCollisionObjects;

  std::unique_ptr<DartCollisionScene> mScene;
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONGROUP_HPP_
