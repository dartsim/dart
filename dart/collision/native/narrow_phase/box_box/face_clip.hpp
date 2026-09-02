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

#include <dart/collision/native/export.hpp>
#include <dart/collision/native/narrow_phase/box_box/sat.hpp>

#include <Eigen/Core>

#include <array>
#include <vector>

#include <cstddef>

namespace dart::collision::native::box_box {

struct ContactCandidate
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double depth = 0.0;
};

/// Stack-bounded candidate buffer for the intersection of two box faces.
///
/// The size bound is a theorem, not a guess, because this buffer sits on the
/// collision hot path and is filled from worker threads that cannot report a
/// failure. Sutherland-Hodgman clipping of a convex n-gon against one
/// half-space emits at most one crossing vertex per sign change; a convex
/// polygon changes sign at most twice around its boundary, and two crossings
/// require at least one vertex to fall outside, so the result has at most
/// (n - 1) + 2 = n + 1 vertices. Starting from the four vertices of the
/// incident box face and applying the five reference-face half-spaces in turn
/// gives 4 -> 5 -> 6 -> 7 -> 8 -> 9. The capacity keeps a further margin above
/// that bound without putting heap storage on the hot path.
struct FixedContactCandidates
{
  /// The incident face of a box is a quadrilateral.
  static constexpr std::size_t kIncidentFaceVertexCount = 4u;
  /// The reference face contributes its four side half-spaces plus its own
  /// plane.
  static constexpr std::size_t kClipPlaneCount = 5u;
  /// Proven maximum vertex count of the clipped incident face.
  static constexpr std::size_t kMaxClippedFaceVertexCount
      = kIncidentFaceVertexCount + kClipPlaneCount;

  static constexpr std::size_t kCapacity = 12u;

  static_assert(
      kMaxClippedFaceVertexCount <= kCapacity,
      "Box-box face clipping must fit its proven vertex bound; raise kCapacity "
      "if the clip plane count or incident face changes");

  [[nodiscard]] bool empty() const noexcept
  {
    return count == 0u;
  }

  [[nodiscard]] std::size_t size() const noexcept
  {
    return count;
  }

  [[nodiscard]] const ContactCandidate* begin() const noexcept
  {
    return values.data();
  }

  [[nodiscard]] const ContactCandidate* end() const noexcept
  {
    return values.data() + count;
  }

  [[nodiscard]] const ContactCandidate& operator[](
      std::size_t index) const noexcept
  {
    return values[index];
  }

  std::array<ContactCandidate, kCapacity> values{};
  std::size_t count = 0u;
};

[[nodiscard]] DART_COLLISION_NATIVE_API FixedContactCandidates
computeBoxBoxContactCandidatesFixed(
    const BoxData& box1, const BoxData& box2, const SatResult& sat);

[[nodiscard]] DART_COLLISION_NATIVE_API std::vector<ContactCandidate>
computeBoxBoxContactCandidates(
    const BoxData& box1, const BoxData& box2, const SatResult& sat);

} // namespace dart::collision::native::box_box
