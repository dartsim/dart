/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/collision/dart/broad_phase/broad_phase_overlapping_pairs.hpp"
#include "dart/collision/dart/dart_object.hpp"
#include "dart/collision/export.hpp"

namespace dart::collision::detail {

//==============================================================================
template <typename Scalar>
bool BroadPhaseOverlappingPairs<Scalar>::add(
    DartObject<Scalar>* object_a, DartObject<Scalar>* object_b)
{
  // Not allowed to add the same objects as a pair
  if (object_a == object_b) {
    DART_DEBUG("Cannot add the same collision objects as a pair.");
    return false;
  }

  // Sort the objects by the IDs
  if (object_a->get_id() > object_b->get_id()) {
    std::swap(object_a, object_b);
  }

  const ObjectId id_a = object_a->get_id();
  const ObjectId id_b = object_b->get_id();

  // IDs should be sorted.
  DART_ASSERT(id_a < id_b);

  // Two distinct collision objects shouldn't have the same ID.
  DART_ASSERT(id_a != id_b);

  if (m_pairs.find({id_a, id_b}) != m_pairs.end()) {
    DART_DEBUG("Cannot add the same pair twice.");
    return false;
  }

  auto result = m_pairs.insert({{id_a, id_b}, {}});
  ObjectPair<Scalar>& object_pair = result->first;
  object_pair.object_a = object_a;
  object_pair.object_b = object_b;
}

//==============================================================================
template <typename Scalar>
bool BroadPhaseOverlappingPairs<Scalar>::remove(
    DartObject<Scalar>* object_a, DartObject<Scalar>* object_b)
{
  DART_NOT_IMPLEMENTED;
  DART_UNUSED(object_a, object_b);
  return false;
}

} // namespace dart::collision::detail
