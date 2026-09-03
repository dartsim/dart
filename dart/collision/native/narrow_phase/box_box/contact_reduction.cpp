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

#include <dart/collision/native/narrow_phase/box_box/contact_reduction.hpp>

#include <algorithm>
#include <limits>

namespace dart::collision::native::box_box {

namespace {

constexpr double kDuplicateDistanceSq = 1e-14;

[[nodiscard]] FixedContactCandidates uniqueCandidatesFixed(
    const FixedContactCandidates& candidates)
{
  FixedContactCandidates unique;

  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (std::size_t i = 0; i < unique.count; ++i) {
      auto& accepted = unique.values[i];
      if ((candidate.position - accepted.position).squaredNorm()
          <= kDuplicateDistanceSq) {
        accepted.depth = std::max(accepted.depth, candidate.depth);
        duplicate = true;
        break;
      }
    }

    if (!duplicate) {
      unique.values[unique.count++] = candidate;
    }
  }

  return unique;
}

[[nodiscard]] std::size_t deepestCandidateIndexFixed(
    const FixedContactCandidates& candidates)
{
  std::size_t index = 0;
  double depth = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.count; ++i) {
    if (candidates.values[i].depth > depth) {
      depth = candidates.values[i].depth;
      index = i;
    }
  }
  return index;
}

void eraseCandidate(FixedContactCandidates& candidates, std::size_t index)
{
  for (std::size_t i = index + 1u; i < candidates.count; ++i) {
    candidates.values[i - 1u] = candidates.values[i];
  }
  --candidates.count;
}

[[nodiscard]] std::vector<ContactCandidate> uniqueCandidates(
    const std::vector<ContactCandidate>& candidates)
{
  std::vector<ContactCandidate> unique;
  unique.reserve(candidates.size());

  for (const auto& candidate : candidates) {
    bool duplicate = false;
    for (auto& accepted : unique) {
      if ((candidate.position - accepted.position).squaredNorm()
          <= kDuplicateDistanceSq) {
        accepted.depth = std::max(accepted.depth, candidate.depth);
        duplicate = true;
        break;
      }
    }

    if (!duplicate) {
      unique.push_back(candidate);
    }
  }

  return unique;
}

[[nodiscard]] std::size_t deepestCandidateIndex(
    const std::vector<ContactCandidate>& candidates)
{
  std::size_t index = 0;
  double depth = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (candidates[i].depth > depth) {
      depth = candidates[i].depth;
      index = i;
    }
  }
  return index;
}

} // namespace

FixedContactCandidates reduceContactCandidatesFixed(
    const FixedContactCandidates& candidates, std::size_t maxContacts)
{
  if (maxContacts == 0u || candidates.empty()) {
    return {};
  }

  FixedContactCandidates remaining = uniqueCandidatesFixed(candidates);
  if (remaining.size() <= maxContacts) {
    return remaining;
  }

  FixedContactCandidates selected;
  const std::size_t deepestIndex = deepestCandidateIndexFixed(remaining);
  selected.values[selected.count++] = remaining.values[deepestIndex];
  eraseCandidate(remaining, deepestIndex);

  while (selected.size() < maxContacts && !remaining.empty()) {
    std::size_t bestIndex = 0;
    double bestDistanceSq = -1.0;
    double bestDepth = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < remaining.count; ++i) {
      double minDistanceSq = std::numeric_limits<double>::infinity();
      for (const auto& accepted : selected) {
        minDistanceSq = std::min(
            minDistanceSq,
            (remaining.values[i].position - accepted.position).squaredNorm());
      }

      if (minDistanceSq > bestDistanceSq
          || (minDistanceSq == bestDistanceSq
              && remaining.values[i].depth > bestDepth)) {
        bestDistanceSq = minDistanceSq;
        bestDepth = remaining.values[i].depth;
        bestIndex = i;
      }
    }

    selected.values[selected.count++] = remaining.values[bestIndex];
    eraseCandidate(remaining, bestIndex);
  }

  return selected;
}

std::vector<ContactCandidate> reduceContactCandidates(
    const std::vector<ContactCandidate>& candidates, std::size_t maxContacts)
{
  if (maxContacts == 0 || candidates.empty()) {
    return {};
  }

  std::vector<ContactCandidate> remaining = uniqueCandidates(candidates);
  if (remaining.size() <= maxContacts) {
    return remaining;
  }

  std::vector<ContactCandidate> selected;
  selected.reserve(maxContacts);
  const std::size_t deepestIndex = deepestCandidateIndex(remaining);
  selected.push_back(remaining[deepestIndex]);
  remaining.erase(
      remaining.begin() + static_cast<std::ptrdiff_t>(deepestIndex));

  while (selected.size() < maxContacts && !remaining.empty()) {
    std::size_t bestIndex = 0;
    double bestDistanceSq = -1.0;
    double bestDepth = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < remaining.size(); ++i) {
      double minDistanceSq = std::numeric_limits<double>::infinity();
      for (const auto& accepted : selected) {
        minDistanceSq = std::min(
            minDistanceSq,
            (remaining[i].position - accepted.position).squaredNorm());
      }

      if (minDistanceSq > bestDistanceSq
          || (minDistanceSq == bestDistanceSq
              && remaining[i].depth > bestDepth)) {
        bestDistanceSq = minDistanceSq;
        bestDepth = remaining[i].depth;
        bestIndex = i;
      }
    }

    selected.push_back(remaining[bestIndex]);
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(bestIndex));
  }

  return selected;
}

} // namespace dart::collision::native::box_box
