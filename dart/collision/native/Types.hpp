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

#pragma once

#include <dart/collision/native/ContactManifold.hpp>
#include <dart/collision/native/ContactPoint.hpp>
#include <dart/collision/native/Export.hpp>
#include <dart/collision/native/Fwd.hpp>
#include <dart/collision/native/detail/Span.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <cstddef>

namespace dart::collision::native {

class DART_COLLISION_NATIVE_API CollisionResult
{
public:
  CollisionResult() = default;
  CollisionResult(CollisionResult&& other) noexcept;
  CollisionResult& operator=(CollisionResult&& other) noexcept;

  CollisionResult(const CollisionResult&) = delete;
  CollisionResult& operator=(const CollisionResult&) = delete;

  void addContact(const ContactPoint& contact)
  {
    if (manifoldCount_ == 0u) {
      firstContact_ = contact;
      firstEntryIsContact_ = true;
      manifoldCount_ = 1u;
      contactCount_ = 1u;
      invalidateCache();
      return;
    }

    auto& manifold = nextExtraManifold();
    manifold.setSingleContact(contact, ContactType::Point);
    ++contactCount_;
    invalidateCache();
  }

  void addContact(
      const Eigen::Vector3d& position,
      const Eigen::Vector3d& normal,
      double depth)
  {
    if (manifoldCount_ == 0u) {
      firstContact_.position = position;
      firstContact_.normal = normal;
      firstContact_.depth = depth;
      firstContact_.object1 = nullptr;
      firstContact_.object2 = nullptr;
      firstContact_.featureIndex1 = -1;
      firstContact_.featureIndex2 = -1;
      firstEntryIsContact_ = true;
      manifoldCount_ = 1u;
      contactCount_ = 1u;
      invalidateCache();
      return;
    }

    auto& manifold = nextExtraManifold();
    manifold.setSingleContact(position, normal, depth, ContactType::Point);
    ++contactCount_;
    invalidateCache();
  }

  void addContact(
      double positionX,
      double positionY,
      double positionZ,
      double normalX,
      double normalY,
      double normalZ,
      double depth)
  {
    if (manifoldCount_ == 0u) {
      firstContact_.position.x() = positionX;
      firstContact_.position.y() = positionY;
      firstContact_.position.z() = positionZ;
      firstContact_.normal.x() = normalX;
      firstContact_.normal.y() = normalY;
      firstContact_.normal.z() = normalZ;
      firstContact_.depth = depth;
      firstContact_.object1 = nullptr;
      firstContact_.object2 = nullptr;
      firstContact_.featureIndex1 = -1;
      firstContact_.featureIndex2 = -1;
      firstEntryIsContact_ = true;
      manifoldCount_ = 1u;
      contactCount_ = 1u;
      invalidateCache();
      return;
    }

    auto& manifold = nextExtraManifold();
    manifold.setSingleContact(
        Eigen::Vector3d(positionX, positionY, positionZ),
        Eigen::Vector3d(normalX, normalY, normalZ),
        depth,
        ContactType::Point);
    ++contactCount_;
    invalidateCache();
  }

  void addManifold(ContactManifold manifold);
  void clear()
  {
    manifoldCount_ = 0;
    contactCount_ = 0;
    invalidateCache();
  }

  [[nodiscard]] bool isCollision() const
  {
    return manifoldCount_ > 0;
  }

  [[nodiscard]] explicit operator bool() const
  {
    return isCollision();
  }

  [[nodiscard]] std::size_t numContacts() const
  {
    return contactCount_;
  }

  [[nodiscard]] std::size_t numManifolds() const
  {
    return manifoldCount_;
  }

  [[nodiscard]] const ContactManifold& getManifold(std::size_t i) const;
  [[nodiscard]] span<const ContactManifold> getManifolds() const;

  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;

  template <typename Callback>
  void forEachContact(Callback&& callback) const
  {
    if (manifoldCount_ == 0u) {
      return;
    }

    if (firstEntryIsContact_) {
      callback(firstContact_);
    } else {
      for (const auto& contact : firstManifold_->getContacts()) {
        callback(contact);
      }
    }

    for (std::size_t i = 1u; i < manifoldCount_; ++i) {
      for (const auto& contact : extraManifolds_[i - 1u].getContacts()) {
        callback(contact);
      }
    }
  }

private:
  ContactPoint firstContact_;
  std::unique_ptr<ContactManifold> firstManifold_;
  std::vector<ContactManifold> extraManifolds_;
  std::size_t manifoldCount_ = 0;
  std::size_t contactCount_ = 0;
  bool firstEntryIsContact_ = false;

  mutable std::vector<ContactManifold> manifoldsCache_;
  mutable std::vector<const ContactPoint*> flatContactsCache_;
  mutable bool manifoldsCacheValid_ = false;
  mutable bool flatCacheValid_ = false;

  ContactManifold& nextExtraManifold()
  {
    const std::size_t extraIndex = manifoldCount_ - 1u;
    if (extraIndex == extraManifolds_.size()) {
      extraManifolds_.emplace_back();
    }
    ++manifoldCount_;
    return extraManifolds_[extraIndex];
  }

  [[nodiscard]] const ContactManifold& manifoldAt(std::size_t i) const;
  void updateManifoldsCache() const;
  void invalidateCache()
  {
    manifoldsCacheValid_ = false;
    flatCacheValid_ = false;
  }

  void updateFlatCache() const;
};

struct DART_COLLISION_NATIVE_API CollisionOption
{
  bool enableContact = true;

  std::size_t maxNumContacts = 1000;

  const CollisionFilter* collisionFilter = nullptr;

  [[nodiscard]] static CollisionOption binaryCheck()
  {
    return {false, 1, nullptr};
  }

  [[nodiscard]] static CollisionOption fullContacts(std::size_t max = 1000)
  {
    return {true, max, nullptr};
  }

  [[nodiscard]] static CollisionOption withFilter(const CollisionFilter* filter)
  {
    return {true, 1000, filter};
  }
};

struct DART_COLLISION_NATIVE_API DistanceResult
{
  double distance = std::numeric_limits<double>::max();

  Eigen::Vector3d pointOnObject1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointOnObject2 = Eigen::Vector3d::Zero();

  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  const CollisionObject* object1 = nullptr;
  const CollisionObject* object2 = nullptr;

  [[nodiscard]] bool isValid() const
  {
    return distance < std::numeric_limits<double>::max();
  }

  void clear()
  {
    distance = std::numeric_limits<double>::max();
    pointOnObject1 = Eigen::Vector3d::Zero();
    pointOnObject2 = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::UnitZ();
    object1 = nullptr;
    object2 = nullptr;
  }
};

struct DART_COLLISION_NATIVE_API DistanceOption
{
  double upperBound = std::numeric_limits<double>::max();

  bool enableNearestPoints = true;

  [[nodiscard]] static DistanceOption unlimited()
  {
    return {std::numeric_limits<double>::max(), true};
  }

  [[nodiscard]] static DistanceOption withUpperBound(double bound)
  {
    return {bound, true};
  }
};

struct DART_COLLISION_NATIVE_API Ray
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();

  double maxDistance = std::numeric_limits<double>::max();

  Ray() = default;

  Ray(const Eigen::Vector3d& origin,
      const Eigen::Vector3d& direction,
      double maxDistance = std::numeric_limits<double>::max())
    : origin(origin),
      direction(direction.normalized()),
      maxDistance(maxDistance)
  {
  }

  [[nodiscard]] Eigen::Vector3d pointAt(double t) const
  {
    return origin + t * direction;
  }
};

struct DART_COLLISION_NATIVE_API RaycastResult
{
  bool hit = false;

  double distance = std::numeric_limits<double>::max();

  Eigen::Vector3d point = Eigen::Vector3d::Zero();

  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  const CollisionObject* object = nullptr;

  [[nodiscard]] bool isHit() const
  {
    return hit;
  }

  void clear()
  {
    hit = false;
    distance = std::numeric_limits<double>::max();
    point = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::UnitZ();
    object = nullptr;
  }
};

struct DART_COLLISION_NATIVE_API RaycastOption
{
  double maxDistance = std::numeric_limits<double>::max();

  bool backfaceCulling = true;

  [[nodiscard]] static RaycastOption unlimited()
  {
    return {std::numeric_limits<double>::max(), true};
  }

  [[nodiscard]] static RaycastOption withMaxDistance(double maxDist)
  {
    return {maxDist, true};
  }
};

struct DART_COLLISION_NATIVE_API CcdResult
{
  bool hit = false;

  double timeOfImpact = 1.0;

  Eigen::Vector3d point = Eigen::Vector3d::Zero();

  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  const CollisionObject* object = nullptr;

  [[nodiscard]] bool isHit() const
  {
    return hit;
  }

  void clear()
  {
    hit = false;
    timeOfImpact = 1.0;
    point = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::UnitZ();
    object = nullptr;
  }
};

/// Advancement strategy for the continuous queries that iterate (the convex and
/// spline casts).
enum class CcdAdvancement
{
  /// Never overshoots a true contact: the reported time of impact is a lower
  /// bound on the first contact, so geometry cannot tunnel. Required by
  /// barrier/IPC-style solvers. Default.
  Conservative,

  /// Larger displacement-based steps: fewer iterations, but may step over a
  /// grazing or sharply curved first contact and report a later one. For
  /// rigid-body use where first-contact precision is not required and raw speed
  /// matters more than the no-tunnelling guarantee.
  Fast
};

struct DART_COLLISION_NATIVE_API CcdOption
{
  double tolerance = 1e-4;

  int maxIterations = 32;

  /// Minimum separation (gap) the query keeps between primitives. A positive
  /// value reports the time of impact at which the primitives close to this
  /// distance rather than to touching, which barrier/IPC-style solvers require.
  /// Ignored by the rigid shape casts; used by the primitive CCD queries.
  double minSeparation = 0.0;

  /// Whether iterative casts trade the no-tunnelling guarantee for fewer
  /// iterations. Defaults to the conservative (safe) strategy.
  CcdAdvancement advancement = CcdAdvancement::Conservative;

  [[nodiscard]] static CcdOption standard()
  {
    return {1e-4, 32};
  }

  [[nodiscard]] static CcdOption precise()
  {
    return {1e-6, 64};
  }
};

enum class CcdPrimitiveStatus
{
  Unknown,
  Hit,
  Miss,
  Indeterminate,
};

/// Result of a primitive-level continuous collision query (point-triangle,
/// edge-edge). Reports only a conservative time of impact in [0, 1]; the
/// contact configuration can be reconstructed by the caller from the input
/// trajectories.
struct DART_COLLISION_NATIVE_API CcdPrimitiveResult
{
  bool hit = false;
  CcdPrimitiveStatus status = CcdPrimitiveStatus::Unknown;

  /// Conservative time of impact in [0, 1]: a lower bound on the true first
  /// contact time. Never overshoots a real collision.
  double timeOfImpact = 1.0;

  [[nodiscard]] bool isHit() const
  {
    return hit;
  }

  void clear()
  {
    hit = false;
    status = CcdPrimitiveStatus::Unknown;
    timeOfImpact = 1.0;
  }
};

} // namespace dart::collision::native
