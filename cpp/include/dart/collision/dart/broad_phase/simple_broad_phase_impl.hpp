/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/collision/dart/broad_phase/simple_broad_phase.hpp"
#include "dart/common/limit.hpp"
#include "dart/common/stl_util.hpp"

namespace dart::collision::detail {

//==============================================================================
template <typename Scalar>
SimpleBroadPhaseAlgorithm<Scalar>::SimpleBroadPhaseAlgorithm(
    common::MemoryAllocator& allocator)
  : m_objects(allocator)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool SimpleBroadPhaseAlgorithm<Scalar>::add_object(DartObject<Scalar>* object)
{
  DART_ASSERT(static_cast<int>(m_objects.size()) <= common::max<int>());

  if (!object) {
    DART_DEBUG("Cannot add null object to broad phase algorithm.");
    return false;
  }

  if (common::contains(m_objects, object)) {
    DART_DEBUG("Cannot add the same object twice to broad phase algorithm.");
    return false;
  }

  m_objects.push_back(object);
  return true;
}

//==============================================================================
template <typename Scalar>
void SimpleBroadPhaseAlgorithm<Scalar>::remove_object(
    DartObject<Scalar>* object)
{
  if (!object) {
    return;
  }

  common::erase(m_objects, object);
}

//==============================================================================
template <typename Scalar>
int SimpleBroadPhaseAlgorithm<Scalar>::get_object_count() const
{
  return m_objects.size();
}

//==============================================================================
template <typename Scalar>
void SimpleBroadPhaseAlgorithm<Scalar>::clear()
{
  return m_objects.clear();
}

//==============================================================================
template <typename Scalar>
void SimpleBroadPhaseAlgorithm<Scalar>::compute_overlapping_pairs(
    Scalar time_step, BroadPhaseCallback<Scalar>&& callback)
{
  DART_UNUSED(time_step);

  for (auto it_a = m_objects.cbegin(); it_a != m_objects.cend(); ++it_a) {
    DartObject<Scalar>* object_a = *it_a;
    auto it_b = it_a;
    it_b++;
    for (; it_b != m_objects.cend(); ++it_b) {
      DartObject<Scalar>* object_b = *it_b;
      if (object_a->get_aabb().overlaps(object_b->get_aabb())) {
        if (callback.add_pair(object_a, object_b)) {
          return;
        }
      } else {
        if (callback.remove_pair(object_a, object_b)) {
          return;
        }
      }
    }
  }
}

//==============================================================================
template <typename Scalar>
void SimpleBroadPhaseAlgorithm<Scalar>::compute_overlapping_pairs(
    Scalar time_step,
    std::function<void(DartObject<Scalar>*, DartObject<Scalar>*)>&& callback)
{
  DART_UNUSED(time_step);

  for (auto it_a = m_objects.cbegin(); it_a != m_objects.cend(); ++it_a) {
    DartObject<Scalar>* object_a = *it_a;
    auto it_b = it_a;
    it_b++;
    for (; it_b != m_objects.cend(); ++it_b) {
      DartObject<Scalar>* object_b = *it_b;
      if (object_a->get_aabb().overlaps(object_b->get_aabb())) {
        callback(object_a, object_b);
      }
    }
  }
}

} // namespace dart::collision::detail
