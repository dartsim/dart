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

#include <dart/collision/native/broad_phase/BroadPhase.hpp>

#include <unordered_map>

#include <cstring>

namespace dart::collision::native {

class DART_COLLISION_NATIVE_API BruteForceBroadPhase : public BroadPhase
{
public:
  BruteForceBroadPhase() = default;

  void clear() override;
  void add(std::size_t id, const Aabb& aabb) override;
  void update(std::size_t id, const Aabb& aabb) override;
  void updateRange(
      span<const std::size_t> ids, span<const Aabb> aabbs) override;
  void remove(std::size_t id) override;

  [[nodiscard]] std::vector<BroadPhasePair> queryPairs() const override;
  [[nodiscard]] std::vector<std::size_t> queryOverlapping(
      const Aabb& aabb) const override;
  [[nodiscard]] std::size_t size() const override;

  bool visitPairs(const BroadPhasePairVisitor& visitor) const override;

  template <typename Visitor>
  bool visitPairsInline(Visitor&& visitor) const
  {
    const std::size_t count = entryCount();
    for (std::size_t i = 0u; i < count; ++i) {
      const Entry entry1 = loadEntry(i);
      for (std::size_t j = i + 1u; j < count; ++j) {
        const Entry entry2 = loadEntry(j);
        if (overlapsFast(entry1, entry2) && !visitor(entry1.id, entry2.id))
          return false;
      }
    }

    return true;
  }

  using BroadPhase::build;
  using BroadPhase::queryPairs;
  using BroadPhase::updateRange;

private:
  struct Entry
  {
    std::size_t id;
    double minX{0.0};
    double minY{0.0};
    double minZ{0.0};
    double maxX{0.0};
    double maxY{0.0};
    double maxZ{0.0};
  };

  static Entry makeEntry(std::size_t id, const Aabb& aabb);
  static bool overlapsFast(const Entry& a, const Entry& b);
  static bool overlapsFast(const Entry& entry, const Aabb& aabb);

  static constexpr std::size_t kDoubleWordCount
      = (sizeof(double) + sizeof(std::size_t) - 1u) / sizeof(std::size_t);
  static constexpr std::size_t kEntryWordCount = 1u + 6u * kDoubleWordCount;

  [[nodiscard]] std::size_t entryCount() const
  {
    return orderedIds_.size() / kEntryWordCount;
  }

  [[nodiscard]] Entry loadEntry(std::size_t index) const
  {
    const std::size_t offset = index * kEntryWordCount;
    Entry entry{};
    entry.id = orderedIds_[offset];
    std::memcpy(
        &entry.minX, orderedIds_.data() + offset + 1u, sizeof(entry.minX));
    std::memcpy(
        &entry.minY,
        orderedIds_.data() + offset + 1u + kDoubleWordCount,
        sizeof(entry.minY));
    std::memcpy(
        &entry.minZ,
        orderedIds_.data() + offset + 1u + 2u * kDoubleWordCount,
        sizeof(entry.minZ));
    std::memcpy(
        &entry.maxX,
        orderedIds_.data() + offset + 1u + 3u * kDoubleWordCount,
        sizeof(entry.maxX));
    std::memcpy(
        &entry.maxY,
        orderedIds_.data() + offset + 1u + 4u * kDoubleWordCount,
        sizeof(entry.maxY));
    std::memcpy(
        &entry.maxZ,
        orderedIds_.data() + offset + 1u + 5u * kDoubleWordCount,
        sizeof(entry.maxZ));
    return entry;
  }

  void storeEntry(std::size_t index, const Entry& entry);
  [[nodiscard]] std::size_t findEntryIndex(std::size_t id) const;
  void rebuildEntries();
  void rebuildOrderedIds();

  // Keep these members identical to the original 6.20 layout. orderedIds_
  // stores packed, sorted Entry records so pair walks stay contiguous without
  // changing the exported class ABI.
  std::unordered_map<std::size_t, Aabb> objects_;
  std::vector<std::size_t> orderedIds_;
};

} // namespace dart::collision::native
