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

#include <dart/collision/experimental/contact_manifold.hpp>
#include <dart/collision/experimental/contact_point.hpp>
#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/fwd.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <span>
#include <vector>

namespace dart::collision::experimental {

class DART_COLLISION_EXPERIMENTAL_API CollisionResult
{
public:
  CollisionResult() = default;

  void addContact(const ContactPoint& contact);
  void addManifold(ContactManifold manifold);
  void clear();

  [[nodiscard]] bool isCollision() const;
  [[nodiscard]] explicit operator bool() const;

  [[nodiscard]] std::size_t numContacts() const;
  [[nodiscard]] std::size_t numManifolds() const;

  [[nodiscard]] const ContactManifold& getManifold(std::size_t i) const;
  [[nodiscard]] std::span<const ContactManifold> getManifolds() const;

  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;

private:
  std::vector<ContactManifold> manifolds_;

  mutable std::vector<const ContactPoint*> flatContactsCache_;
  mutable bool flatCacheValid_ = false;

  void invalidateCache();
  void updateFlatCache() const;
};

struct DART_COLLISION_EXPERIMENTAL_API CollisionOption
{
  bool enableContact = true;

  std::size_t maxNumContacts = 1000;

  [[nodiscard]] static CollisionOption binaryCheck()
  {
    return {false, 1};
  }

  [[nodiscard]] static CollisionOption fullContacts(std::size_t max = 1000)
  {
    return {true, max};
  }
};

struct DART_COLLISION_EXPERIMENTAL_API DistanceResult
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

struct DART_COLLISION_EXPERIMENTAL_API DistanceOption
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

struct DART_COLLISION_EXPERIMENTAL_API Ray
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();

  double maxDistance = std::numeric_limits<double>::max();

  Ray() = default;

  Ray(
      const Eigen::Vector3d& origin,
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

struct DART_COLLISION_EXPERIMENTAL_API RaycastResult
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

struct DART_COLLISION_EXPERIMENTAL_API RaycastOption
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

struct DART_COLLISION_EXPERIMENTAL_API CcdResult
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

struct DART_COLLISION_EXPERIMENTAL_API CcdOption
{
  double tolerance = 1e-4;

  int maxIterations = 32;

  [[nodiscard]] static CcdOption standard()
  {
    return {1e-4, 32};
  }

  [[nodiscard]] static CcdOption precise()
  {
    return {1e-6, 64};
  }
};

}
