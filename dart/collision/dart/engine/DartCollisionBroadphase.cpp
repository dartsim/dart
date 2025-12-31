/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/dart/engine/DartCollisionBroadphase.hpp"

#include <algorithm>

namespace dart {
namespace collision {

//==============================================================================
void computeSweepPairs(
    const std::vector<CoreBroadphaseEntry>& entries,
    std::uint8_t mask1,
    std::uint8_t mask2,
    std::vector<CoreBroadphasePair>* pairs)
{
  if (!pairs)
    return;

  pairs->clear();

  if (entries.size() < 2u)
    return;

  struct SweepEntry
  {
    double minX;
    double maxX;
    std::size_t index;
  };

  std::vector<SweepEntry> sweep;
  sweep.reserve(entries.size());
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto* object = entries[i].object;
    if (!object)
      continue;

    sweep.push_back({object->worldAabbMin.x(), object->worldAabbMax.x(), i});
  }

  if (sweep.size() < 2u)
    return;

  std::stable_sort(
      sweep.begin(), sweep.end(), [](const SweepEntry& a, const SweepEntry& b) {
        return a.minX < b.minX;
      });

  for (std::size_t i = 0; i + 1u < sweep.size(); ++i) {
    const auto& entry = sweep[i];
    const auto maxX = entry.maxX;
    const auto& current = entries[entry.index];

    for (std::size_t j = i + 1u; j < sweep.size(); ++j) {
      if (sweep[j].minX > maxX)
        break;

      const auto& other = entries[sweep[j].index];
      const bool validPair
          = ((current.mask & mask1) && (other.mask & mask2))
            || ((current.mask & mask2) && (other.mask & mask1));

      if (!validPair)
        continue;

      pairs->push_back({entry.index, sweep[j].index});
    }
  }
}

} // namespace collision
} // namespace dart
