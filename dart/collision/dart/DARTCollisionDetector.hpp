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

#ifndef DART_COLLISION_DART_DARTCOLLISIONDETECTOR_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONDETECTOR_HPP_

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/dart/Fwd.hpp>

namespace dart {
namespace collision {

class DARTCollisionObject;
class DARTCollisionGroup;

namespace detail {
struct DARTCollisionDetectorAccessor;
struct DARTCollisionGroupEngineData;
struct DARTCollisionObjectEngineData;
} // namespace detail

class DARTCollisionDetector : public CollisionDetector
{
public:
  using CollisionDetector::createCollisionGroup;

  static std::shared_ptr<DARTCollisionDetector> create();

  // Documentation inherited
  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override;

  // Documentation inherited
  const std::string& getType() const override;

  /// Get collision detector type for this class.
  static const std::string& getStaticType();

  /// Sets the number of worker participants for parallel collision queries.
  /// A value of 0 maps to hardware concurrency.
  void setNumCollisionThreads(std::size_t numThreads);

  /// Returns the number of worker participants for parallel collision queries.
  std::size_t getNumCollisionThreads() const;

  /// Enables contacts for primitive penetration through soft triangle
  /// interiors when no vertex contact covers that face. This is disabled by
  /// default to preserve legacy contact results. The
  /// DART_SOFT_FACE_INTERIOR_CONTACTS environment variable overrides the
  /// construction default when set to a non-empty value other than "0".
  void setSoftFaceInteriorContactsEnabled(bool enabled);

  /// Returns whether soft face-interior contacts are enabled.
  bool getSoftFaceInteriorContactsEnabled() const;

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

  [[nodiscard]] native::CachedContact* getCachedContact(
      const DARTCollisionObject* object1,
      const DARTCollisionObject* object2,
      void* userData) const;

protected:
  DARTCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  // Documentation inherited
  void refreshCollisionObject(CollisionObject* object) override;

  // Documentation inherited
  void notifyCollisionObjectDestroying(CollisionObject* object) override;

private:
  friend class DARTCollisionGroup;
  friend class DARTCollisionObject;
  friend struct detail::DARTCollisionDetectorAccessor;

  class DARTCollisionObjectManager;

  CollisionDetectorPtr attachCollisionGroupEngineData(
      const DARTCollisionGroup* group,
      const CollisionDetectorPtr& collisionDetector);

  void removeCollisionGroupEngineData(const DARTCollisionGroup* group);

  detail::DARTCollisionGroupEngineData& getCollisionGroupEngineData(
      const DARTCollisionGroup* group);

  dynamics::ConstShapePtr attachCollisionObjectEngineData(
      const DARTCollisionObject* object, const dynamics::ConstShapePtr& shape);

  void removeCollisionObjectEngineData(const DARTCollisionObject* object);

  detail::DARTCollisionObjectEngineData& getCollisionObjectEngineData(
      const DARTCollisionObject* object);

  const detail::DARTCollisionObjectEngineData& getCollisionObjectEngineData(
      const DARTCollisionObject* object) const;

  std::size_t getNumCollisionGroupEngineData() const;

  std::size_t getNumCollisionObjectEngineData() const;

  static Registrar<DARTCollisionDetector> mRegistrar;
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONDETECTOR_HPP_
