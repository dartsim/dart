/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/common/Fwd.hpp>
#include <dart/common/ecs/entity_manager.hpp>

#include <cstdint>

namespace dart::common {

template <typename... ComponentTypes>
class View
{
public:
  using EntityType = typename EntityManager::EntityType;

  explicit View(EntityManager& em)
    : m_em(em)
  {
    // Empty
  }

//  [[nodiscard]] auto begin() const
//  {
//  }

//  [[nodiscard]] auto end() const
//  {
//  }

private:
  EntityManager& m_em;
//  ComponentTypeMask componentMasks_;

  class Iterator
  {
  public:
    Iterator() = default;

//    explicit Iterator(
//        const EntityManager& manager,
//        const ComponentEntityMap& componentEntityMap,
//        ComponentTypeMask componentMasks)
//      : entityIter_(manager.getEntityData().begin()),
//        componentEntityMap_(componentEntityMap),
//        componentMasks_(componentMasks)
//    {
//      if (entityIter_ != manager.getEntityData().end()) {
//        update();
//      }
//    }

//    auto& operator*() const
//    {
//      return entity_;
//    }

//    auto& operator++()
//    {
//      ++entityIter_;
//      if (entityIter_ != componentEntityMap_.end()) {
//        update();
//      }
//      return *this;
//    }

//    auto operator==(const Iterator& other) const
//    {
//      return entityIter_ == other.entityIter_;
//    }

//    auto operator!=(const Iterator& other) const
//    {
//      return !(*this == other);
//    }

//  private:
//    EntityManager::EntityData::Iterator entityIter_;
//    const ComponentEntityMap& componentEntityMap_;
//    ComponentTypeMask componentMasks_;
    EntityType m_entity{};
//    int index_{};

//    void update()
//    {
//      const auto& componentVector = componentEntityMap_.at(entityIter_->id());
//      index_ = 0;
//      while (index_ < componentVector.size()) {
//        if ((componentVector[index_].first & componentMasks_)
//            == componentMasks_) {
//          entity_ = entityIter_->id();
//          return;
//        }
//        ++index_;
//      }
//      operator++();
//    }
  };
};

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

namespace dart::common {

//==============================================================================

} // namespace dart::common
