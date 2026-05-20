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

#include <dart/collision/native/types.hpp>

#include <stdexcept>

namespace dart::collision::native {

ContactManifold& CollisionResult::nextManifold()
{
  if (manifoldCount_ == manifolds_.size()) {
    manifolds_.emplace_back();
  }

  auto& manifold = manifolds_[manifoldCount_++];
  manifold.clear();
  return manifold;
}

void CollisionResult::addContact(const ContactPoint& contact)
{
  auto& manifold = nextManifold();
  manifold.setSingleContact(contact, ContactType::Point);
  ++contactCount_;
  invalidateCache();
}

void CollisionResult::addManifold(ContactManifold manifold)
{
  contactCount_ += manifold.numContacts();
  nextManifold() = std::move(manifold);
  invalidateCache();
}

void CollisionResult::clear()
{
  manifoldCount_ = 0;
  contactCount_ = 0;
  invalidateCache();
}

bool CollisionResult::isCollision() const
{
  return manifoldCount_ > 0;
}

CollisionResult::operator bool() const
{
  return isCollision();
}

std::size_t CollisionResult::numContacts() const
{
  return contactCount_;
}

std::size_t CollisionResult::numManifolds() const
{
  return manifoldCount_;
}

const ContactManifold& CollisionResult::getManifold(std::size_t i) const
{
  if (i >= manifoldCount_) {
    throw std::out_of_range("CollisionResult::getManifold");
  }
  return manifolds_[i];
}

std::span<const ContactManifold> CollisionResult::getManifolds() const
{
  return {manifolds_.data(), manifoldCount_};
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
  for (std::size_t i = 0; i < manifoldCount_; ++i) {
    const auto& manifold = manifolds_[i];
    for (const auto& contact : manifold.getContacts()) {
      flatContactsCache_.push_back(&contact);
    }
  }
  flatCacheValid_ = true;
}

} // namespace dart::collision::native
