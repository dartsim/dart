/*
 * Copyright (c) 2011-2021, The DART development contributors
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

namespace dart::collision::detail {

//==============================================================================
template <typename Scalar>
bool SimpleBroadPhaseAlgorithm<Scalar>::add_object(DartObjectPtr<Scalar> object)
{
  if (!object) {
    DART_DEBUG("Cannot add null object to broad phase algorithm.");
    return false;
  }

  if (std::find(m_objects.begin(), m_objects.end(), object.get())
      != m_objects.end()) {
    DART_DEBUG("Cannot add the same object twice to broad phase algorithm.");
    return false;
  }

  m_objects.push_back(object.get());
  return true;
}

//==============================================================================
template <typename Scalar>
void SimpleBroadPhaseAlgorithm<Scalar>::compute_overlapping_pairs(
    Scalar time_step, BroadPhaseCallback<Scalar>&& callback)
{
  (void)time_step;
  (void)callback;
  DART_NOT_IMPLEMENTED;

  // TODO(JS): Update AABB (or any BV)

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

} // namespace dart::collision::detail
