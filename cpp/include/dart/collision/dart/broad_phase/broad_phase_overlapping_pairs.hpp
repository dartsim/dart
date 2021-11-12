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

#include <unordered_map>

#include "dart/collision/dart/broad_phase/object_pair.hpp"
#include "dart/collision/dart/dart_type.hpp"
#include "dart/common/hash.hpp"

namespace dart::collision::detail {

struct ObjectPairHash
{
  std::size_t operator()(const std::pair<ObjectId, ObjectId>& key) const
  {
    return common::hash_pair_szudzik(key.first, key.second);
  }
};

/// Container specialized for storing collision object pairs that are usually
/// detected by the broad phase collision detection
template <typename Scalar_>
class BroadPhaseOverlappingPairs
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Constructor
  BroadPhaseOverlappingPairs() = default;

  /// Destructor
  virtual ~BroadPhaseOverlappingPairs() = default;

  bool add(DartObject<Scalar>* object_a, DartObject<Scalar>* object_b);

  bool remove(DartObject<Scalar>* object_a, DartObject<Scalar>* object_b);

protected:
private:
  std::unordered_map<
      std::pair<ObjectId, ObjectId>,
      ObjectPair<Scalar>,
      ObjectPairHash>
      m_pairs;
};

} // namespace dart::collision::detail

#include "dart/collision/dart/broad_phase/broad_phase_overlapping_pairs.hpp"
