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

#include <Eigen/Core>

#include <cstddef>
#include <span>
#include <vector>

namespace dart::collision::experimental {

struct DART_COLLISION_EXPERIMENTAL_API ContactPoint
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  double depth = 0.0;

  const CollisionObject* object1 = nullptr;
  const CollisionObject* object2 = nullptr;

  int featureIndex1 = -1;
  int featureIndex2 = -1;

  static constexpr double kNormalEpsilon = 1e-6;

  [[nodiscard]] static bool isZeroNormal(const Eigen::Vector3d& n)
  {
    return n.squaredNorm() < kNormalEpsilon * kNormalEpsilon;
  }

  [[nodiscard]] static bool isNonZeroNormal(const Eigen::Vector3d& n)
  {
    return !isZeroNormal(n);
  }
};

enum class ContactType
{
  Point,
  Edge,
  Face,
  Patch,
  Unknown
};

class DART_COLLISION_EXPERIMENTAL_API ContactManifold
{
public:
  ContactManifold() = default;

  void addContact(const ContactPoint& contact);
  void clear();

  [[nodiscard]] std::size_t numContacts() const;
  [[nodiscard]] bool hasContacts() const;
  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;
  [[nodiscard]] std::span<const ContactPoint> getContacts() const;

  [[nodiscard]] ContactType getType() const;
  void setType(ContactType type);

  [[nodiscard]] Eigen::Vector3d getSharedNormal() const;

  [[nodiscard]] const CollisionObject* getObject1() const;
  [[nodiscard]] const CollisionObject* getObject2() const;
  void setObjects(const CollisionObject* o1, const CollisionObject* o2);

private:
  std::vector<ContactPoint> contacts_;
  ContactType type_ = ContactType::Unknown;
  const CollisionObject* object1_ = nullptr;
  const CollisionObject* object2_ = nullptr;
};

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

}
