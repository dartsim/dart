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

#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/fwd.hpp>

#include <functional>

#include <cstdint>

namespace dart::collision::experimental {

/// Default collision group: all bits set (collides with everything)
constexpr std::uint32_t kCollisionGroupAll = 0xFFFFFFFF;

/// Default collision mask: all bits set (accepts collisions from everything)
constexpr std::uint32_t kCollisionMaskAll = 0xFFFFFFFF;

/// No collision group: no bits set (collides with nothing)
constexpr std::uint32_t kCollisionGroupNone = 0x00000000;

/// No collision mask: no bits set (accepts collisions from nothing)
constexpr std::uint32_t kCollisionMaskNone = 0x00000000;

/// Predefined collision filter group constants for common use cases.
/// Users can define their own groups using any bit pattern.
///
/// Example usage:
///   obj.setCollisionGroup(FilterGroup::Dynamic);
///   obj.setCollisionMask(FilterGroup::Static | FilterGroup::Dynamic);
namespace FilterGroup {

constexpr std::uint32_t Default = 0x0001;
constexpr std::uint32_t Static = 0x0002;
constexpr std::uint32_t Kinematic = 0x0004;
constexpr std::uint32_t Dynamic = 0x0008;
constexpr std::uint32_t Sensor = 0x0010;
constexpr std::uint32_t Character = 0x0020;
constexpr std::uint32_t Debris = 0x0040;
constexpr std::uint32_t Trigger = 0x0080;

// User-defined groups (bits 8-31 available)
constexpr std::uint32_t User1 = 0x0100;
constexpr std::uint32_t User2 = 0x0200;
constexpr std::uint32_t User3 = 0x0400;
constexpr std::uint32_t User4 = 0x0800;

} // namespace FilterGroup

/// ECS component storing collision filter data for an object.
/// Uses bitmask-based filtering for O(1) collision pair filtering.
///
/// Two objects A and B collide if:
///   (A.group & B.mask) != 0 && (B.group & A.mask) != 0
///
/// This is the standard approach used by Bullet, ODE, and most physics engines.
struct DART_COLLISION_EXPERIMENTAL_API CollisionFilterData
{
  /// Collision group bitmask. Defines which group(s) this object belongs to.
  /// Default: kCollisionGroupAll (object belongs to all groups)
  std::uint32_t collisionGroup = kCollisionGroupAll;

  /// Collision mask bitmask. Defines which groups this object can collide with.
  /// Default: kCollisionMaskAll (object can collide with all groups)
  std::uint32_t collisionMask = kCollisionMaskAll;

  /// Check if this object can collide with another based on bitmasks.
  /// Returns true if collision should be tested, false to skip.
  [[nodiscard]] bool canCollideWith(const CollisionFilterData& other) const
  {
    return (collisionGroup & other.collisionMask) != 0
           && (other.collisionGroup & collisionMask) != 0;
  }

  /// Factory: Create filter data that collides with everything (default)
  [[nodiscard]] static CollisionFilterData all()
  {
    return {kCollisionGroupAll, kCollisionMaskAll};
  }

  /// Factory: Create filter data that collides with nothing
  [[nodiscard]] static CollisionFilterData none()
  {
    return {kCollisionGroupNone, kCollisionMaskNone};
  }

  /// Factory: Create filter data for a specific group that collides with all
  [[nodiscard]] static CollisionFilterData group(std::uint32_t group)
  {
    return {group, kCollisionMaskAll};
  }

  /// Factory: Create filter data with specific group and mask
  [[nodiscard]] static CollisionFilterData groupAndMask(
      std::uint32_t group, std::uint32_t mask)
  {
    return {group, mask};
  }
};

/// Abstract base class for custom collision filtering.
///
/// For simple bitmask-based filtering, use CollisionFilterData directly.
/// This interface is for advanced cases requiring custom logic
/// (e.g., filtering based on object properties, game state, etc.)
class DART_COLLISION_EXPERIMENTAL_API CollisionFilter
{
public:
  virtual ~CollisionFilter() = default;

  /// Returns true if the collision between object1 and object2 should be
  /// IGNORED (skipped). Returns false if they should be tested for collision.
  ///
  /// Note: This is called AFTER bitmask filtering. If bitmask filtering
  /// already rejected the pair, this callback is not invoked.
  ///
  /// @param object1 First collision object
  /// @param object2 Second collision object
  /// @return true to IGNORE collision, false to test collision
  [[nodiscard]] virtual bool ignoresCollision(
      const CollisionObject& object1, const CollisionObject& object2) const
      = 0;
};

/// Callback-based collision filter using std::function.
/// Convenient for simple filtering logic without subclassing.
class DART_COLLISION_EXPERIMENTAL_API CallbackCollisionFilter
  : public CollisionFilter
{
public:
  using FilterCallback = std::function<bool(
      const CollisionObject& obj1, const CollisionObject& obj2)>;

  explicit CallbackCollisionFilter(FilterCallback callback)
    : callback_(std::move(callback))
  {
  }

  [[nodiscard]] bool ignoresCollision(
      const CollisionObject& object1,
      const CollisionObject& object2) const override
  {
    return callback_ ? callback_(object1, object2) : false;
  }

private:
  FilterCallback callback_;
};

/// Composite filter that combines multiple filters.
/// Returns true (ignore) if ANY filter returns true.
class DART_COLLISION_EXPERIMENTAL_API CompositeCollisionFilter
  : public CollisionFilter
{
public:
  void addFilter(const CollisionFilter* filter);
  void removeFilter(const CollisionFilter* filter);
  void clearFilters();

  [[nodiscard]] std::size_t numFilters() const
  {
    return filters_.size();
  }

  [[nodiscard]] bool ignoresCollision(
      const CollisionObject& object1,
      const CollisionObject& object2) const override;

private:
  std::vector<const CollisionFilter*> filters_;
};

/// Check if two objects should collide based on their filter data.
/// This is the fast-path bitmask check used in broadphase and narrowphase.
///
/// @param data1 Filter data for first object
/// @param data2 Filter data for second object
/// @return true if objects should be tested for collision, false to skip
[[nodiscard]] inline bool shouldCollide(
    const CollisionFilterData& data1, const CollisionFilterData& data2)
{
  return data1.canCollideWith(data2);
}

/// Check if two objects should collide, considering both bitmask and
/// optional callback filter.
///
/// @param data1 Filter data for first object
/// @param data2 Filter data for second object
/// @param object1 First collision object (for callback)
/// @param object2 Second collision object (for callback)
/// @param filter Optional custom filter callback (may be nullptr)
/// @return true if objects should be tested for collision, false to skip
[[nodiscard]] inline bool shouldCollide(
    const CollisionFilterData& data1,
    const CollisionFilterData& data2,
    const CollisionObject& object1,
    const CollisionObject& object2,
    const CollisionFilter* filter)
{
  // Fast-path: bitmask check first
  if (!data1.canCollideWith(data2)) {
    return false;
  }

  // Slow-path: callback filter
  if (filter && filter->ignoresCollision(object1, object2)) {
    return false;
  }

  return true;
}

} // namespace dart::collision::experimental
