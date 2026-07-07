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

#include <dart/collision/native/ContactPoint.hpp>
#include <dart/collision/native/detail/Span.hpp>
#include <dart/collision/native/Export.hpp>
#include <dart/collision/native/Fwd.hpp>

#include <array>

#include <cstddef>

namespace dart::collision::native {

enum class ContactType
{
  Point,
  Edge,
  Face,
  Patch,
  Unknown
};

class DART_COLLISION_NATIVE_API ContactManifold
{
public:
  // Contact count is a solver cap; a 4-point manifold still represents a
  // surface patch, not a volumetric overlap.
  static constexpr std::size_t kMaxContacts = 4;

  ContactManifold() = default;

  void addContact(const ContactPoint& contact);
  void clear();

  [[nodiscard]] std::size_t numContacts() const;
  [[nodiscard]] bool hasContacts() const;
  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;
  [[nodiscard]] span<const ContactPoint> getContacts() const;

  [[nodiscard]] ContactType getType() const;
  void setType(ContactType type);
  [[nodiscard]] bool isTypeCompatible() const;

  [[nodiscard]] Eigen::Vector3d getSharedNormal() const;

  [[nodiscard]] const CollisionObject* getObject1() const;
  [[nodiscard]] const CollisionObject* getObject2() const;
  void setObjects(const CollisionObject* o1, const CollisionObject* o2);

private:
  friend class CollisionResult;

  void setSingleContact(const ContactPoint& contact, ContactType type)
  {
    contacts_[0] = contact;
    numContacts_ = 1;
    type_ = type;
    object1_ = contact.object1;
    object2_ = contact.object2;
  }

  void setSingleContact(
      const Eigen::Vector3d& position,
      const Eigen::Vector3d& normal,
      double depth,
      ContactType type)
  {
    auto& contact = contacts_[0];
    contact.position = position;
    contact.normal = normal;
    contact.depth = depth;
    contact.object1 = nullptr;
    contact.object2 = nullptr;
    contact.featureIndex1 = -1;
    contact.featureIndex2 = -1;
    numContacts_ = 1;
    type_ = type;
    object1_ = nullptr;
    object2_ = nullptr;
  }

  std::array<ContactPoint, kMaxContacts> contacts_{};
  std::size_t numContacts_ = 0;
  ContactType type_ = ContactType::Unknown;
  const CollisionObject* object1_ = nullptr;
  const CollisionObject* object2_ = nullptr;
};

} // namespace dart::collision::native
