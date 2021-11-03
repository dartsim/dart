/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_HPP_
#define DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_HPP_

#include <vector>

#include <assimp/scene.h>

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/bullet/BulletCollisionGroup.hpp"
#include "dart/collision/bullet/BulletCollisionShape.hpp"
#include "dart/collision/bullet/BulletInclude.hpp"

namespace dart {
namespace collision {

class BulletCollisionDetector : public CollisionDetector
{
public:
  using CollisionDetector::createCollisionGroup;

  friend class CollisionDetector;

  static std::shared_ptr<BulletCollisionDetector> create();

  /// Constructor
  ~BulletCollisionDetector() override;

  // Documentation inherited
  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override;

  // Documentation inherited
  const std::string& getType() const override;

  /// Get collision detector type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  std::unique_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  double distance(
      CollisionGroup* group,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  // Documentation inherited
  double distance(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  // Documentation inherited
  bool raycast(
      CollisionGroup* group,
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option = RaycastOption(),
      RaycastResult* result = nullptr) override;

protected:
  /// Constructor
  BulletCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  // Documentation inherited
  void refreshCollisionObject(CollisionObject* object) override;

  // Documentation inherited
  void notifyCollisionObjectDestroying(CollisionObject* object) override;

private:
  std::shared_ptr<BulletCollisionShape> claimBulletCollisionShape(
      const dynamics::ConstShapePtr& shape);

  void reclaimBulletCollisionShape(const dynamics::ConstShapePtr& shape);

  std::unique_ptr<BulletCollisionShape> createBulletCollisionShape(
      const dynamics::ConstShapePtr& shape);

  /// This deleter is responsible for deleting BulletCollsionShape objects and
  /// removing them from mShapeMap when they are not shared by any
  /// CollisionObjects.
  class BulletCollisionShapeDeleter final
  {
  public:
    BulletCollisionShapeDeleter(
        BulletCollisionDetector* cd, const dynamics::ConstShapePtr& shape);

    void operator()(BulletCollisionShape* shape) const;

  private:
    BulletCollisionDetector* mBulletCollisionDetector;

    dynamics::ConstShapePtr mShape;
  };

  /// Information for a shape that was generated by this collision detector
  struct ShapeInfo final
  {
    /// A weak reference to the shape
    std::weak_ptr<BulletCollisionShape> mShape;

    /// The last version of the shape, as known by this collision detector
    std::size_t mLastKnownVersion;
  };

private:
  std::map<dynamics::ConstShapePtr, ShapeInfo> mShapeMap;

  std::unique_ptr<BulletCollisionGroup> mGroupForFiltering;

  static Registrar<BulletCollisionDetector> mRegistrar;
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_HPP_
