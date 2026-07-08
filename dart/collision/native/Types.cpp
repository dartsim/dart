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

#include <dart/collision/native/Types.hpp>

#include <stdexcept>
#include <utility>

namespace dart::collision::native {

CollisionResult::CollisionResult(CollisionResult&& other) noexcept
  : firstContact_(std::move(other.firstContact_)),
    firstManifold_(std::move(other.firstManifold_)),
    extraManifolds_(std::move(other.extraManifolds_)),
    manifoldCount_(other.manifoldCount_),
    contactCount_(other.contactCount_),
    firstEntryIsContact_(other.firstEntryIsContact_)
{
  manifoldsCache_.clear();
  flatContactsCache_.clear();
  invalidateCache();

  other.manifoldCount_ = 0u;
  other.contactCount_ = 0u;
  other.firstEntryIsContact_ = false;
  other.firstManifold_.reset();
  other.extraManifolds_.clear();
  other.manifoldsCache_.clear();
  other.flatContactsCache_.clear();
  other.invalidateCache();
}

CollisionResult& CollisionResult::operator=(CollisionResult&& other) noexcept
{
  if (this == &other) {
    return *this;
  }

  firstContact_ = std::move(other.firstContact_);
  firstManifold_ = std::move(other.firstManifold_);
  extraManifolds_ = std::move(other.extraManifolds_);
  manifoldCount_ = other.manifoldCount_;
  contactCount_ = other.contactCount_;
  firstEntryIsContact_ = other.firstEntryIsContact_;
  manifoldsCache_.clear();
  flatContactsCache_.clear();
  invalidateCache();

  other.manifoldCount_ = 0u;
  other.contactCount_ = 0u;
  other.firstEntryIsContact_ = false;
  other.firstManifold_.reset();
  other.extraManifolds_.clear();
  other.manifoldsCache_.clear();
  other.flatContactsCache_.clear();
  other.invalidateCache();

  return *this;
}

void CollisionResult::addManifold(ContactManifold manifold)
{
  contactCount_ += manifold.numContacts();
  if (manifoldCount_ == 0u) {
    if (firstManifold_ != nullptr) {
      *firstManifold_ = std::move(manifold);
    } else {
      firstManifold_ = std::make_unique<ContactManifold>(std::move(manifold));
    }
    firstEntryIsContact_ = false;
    manifoldCount_ = 1u;
  } else {
    nextExtraManifold() = std::move(manifold);
  }
  invalidateCache();
}

const ContactManifold& CollisionResult::getManifold(std::size_t i) const
{
  if (i >= manifoldCount_) {
    throw std::out_of_range("CollisionResult::getManifold");
  }
  return manifoldAt(i);
}

span<const ContactManifold> CollisionResult::getManifolds() const
{
  if (manifoldCount_ == 0u) {
    return {};
  }

  updateManifoldsCache();
  return {manifoldsCache_.data(), manifoldsCache_.size()};
}

const ContactPoint& CollisionResult::getContact(std::size_t i) const
{
  if (i >= contactCount_) {
    throw std::out_of_range("CollisionResult::getContact");
  }

  if (firstEntryIsContact_ && i == 0u) {
    return firstContact_;
  }

  updateFlatCache();
  return *flatContactsCache_.at(i);
}

const ContactManifold& CollisionResult::manifoldAt(std::size_t i) const
{
  if (i == 0u) {
    if (firstEntryIsContact_) {
      updateManifoldsCache();
      return manifoldsCache_[0];
    }
    return *firstManifold_;
  }
  return extraManifolds_[i - 1u];
}

void CollisionResult::updateManifoldsCache() const
{
  if (manifoldsCacheValid_) {
    return;
  }

  manifoldsCache_.clear();
  manifoldsCache_.reserve(manifoldCount_);
  if (firstEntryIsContact_) {
    ContactManifold manifold;
    manifold.setSingleContact(firstContact_, ContactType::Point);
    manifoldsCache_.push_back(manifold);
  } else {
    manifoldsCache_.push_back(*firstManifold_);
  }

  for (std::size_t i = 1u; i < manifoldCount_; ++i) {
    manifoldsCache_.push_back(extraManifolds_[i - 1u]);
  }
  manifoldsCacheValid_ = true;
}

void CollisionResult::updateFlatCache() const
{
  if (flatCacheValid_) {
    return;
  }

  flatContactsCache_.clear();
  if (firstEntryIsContact_) {
    flatContactsCache_.push_back(&firstContact_);
  } else {
    const auto& manifold = *firstManifold_;
    for (const auto& contact : manifold.getContacts()) {
      flatContactsCache_.push_back(&contact);
    }
  }

  for (std::size_t i = 1u; i < manifoldCount_; ++i) {
    const auto& manifold = extraManifolds_[i - 1u];
    for (const auto& contact : manifold.getContacts()) {
      flatContactsCache_.push_back(&contact);
    }
  }
  flatCacheValid_ = true;
}

} // namespace dart::collision::native
