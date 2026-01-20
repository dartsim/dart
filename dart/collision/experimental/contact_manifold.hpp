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

#include <dart/collision/experimental/contact_point.hpp>
#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/fwd.hpp>

#include <array>
#include <cstddef>
#include <span>

namespace dart::collision::experimental {

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
  // Contact count is a solver cap; a 4-point manifold still represents a
  // surface patch, not a volumetric overlap.
  static constexpr std::size_t kMaxContacts = 4;

  ContactManifold() = default;

  void addContact(const ContactPoint& contact);
  void clear();

  [[nodiscard]] std::size_t numContacts() const;
  [[nodiscard]] bool hasContacts() const;
  [[nodiscard]] const ContactPoint& getContact(std::size_t i) const;
  [[nodiscard]] std::span<const ContactPoint> getContacts() const;

  [[nodiscard]] ContactType getType() const;
  void setType(ContactType type);
  [[nodiscard]] bool isTypeCompatible() const;

  [[nodiscard]] Eigen::Vector3d getSharedNormal() const;

  [[nodiscard]] const CollisionObject* getObject1() const;
  [[nodiscard]] const CollisionObject* getObject2() const;
  void setObjects(const CollisionObject* o1, const CollisionObject* o2);

private:
  std::array<ContactPoint, kMaxContacts> contacts_{};
  std::size_t numContacts_ = 0;
  ContactType type_ = ContactType::Unknown;
  const CollisionObject* object1_ = nullptr;
  const CollisionObject* object2_ = nullptr;
};

} // namespace dart::collision::experimental
