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

#include <dart/collision/experimental/types.hpp>

namespace dart::collision::experimental {
void CollisionResult::addContact(const ContactPoint& contact)
{
  ContactManifold manifold;
  manifold.addContact(contact);
  manifold.setType(ContactType::Point);
  if (contact.object1 != nullptr || contact.object2 != nullptr) {
    manifold.setObjects(contact.object1, contact.object2);
  }
  manifolds_.push_back(std::move(manifold));
  invalidateCache();
}

void CollisionResult::addManifold(ContactManifold manifold)
{
  manifolds_.push_back(std::move(manifold));
  invalidateCache();
}

void CollisionResult::clear()
{
  manifolds_.clear();
  invalidateCache();
}

bool CollisionResult::isCollision() const
{
  return !manifolds_.empty();
}

CollisionResult::operator bool() const
{
  return isCollision();
}

std::size_t CollisionResult::numContacts() const
{
  std::size_t count = 0;
  for (const auto& manifold : manifolds_) {
    count += manifold.numContacts();
  }
  return count;
}

std::size_t CollisionResult::numManifolds() const
{
  return manifolds_.size();
}

const ContactManifold& CollisionResult::getManifold(std::size_t i) const
{
  return manifolds_.at(i);
}

std::span<const ContactManifold> CollisionResult::getManifolds() const
{
  return manifolds_;
}

const ContactPoint& CollisionResult::getContact(std::size_t i) const
{
  updateFlatCache();
  return *flatContactsCache_.at(i);
}

void CollisionResult::invalidateCache()
{
  flatCacheValid_ = false;
}

void CollisionResult::updateFlatCache() const
{
  if (flatCacheValid_) {
    return;
  }

  flatContactsCache_.clear();
  for (const auto& manifold : manifolds_) {
    for (const auto& contact : manifold.getContacts()) {
      flatContactsCache_.push_back(&contact);
    }
  }
  flatCacheValid_ = true;
}

} // namespace dart::collision::experimental
