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
/// Clipping a four-vertex convex face against the five reference-face
/// half-spaces produces at most nine vertices; the larger bound leaves a
/// defensive margin without putting heap storage on the collision hot path.
struct FixedContactCandidates
{
  static constexpr std::size_t kCapacity = 12u;

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
