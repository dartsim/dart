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

#ifndef DART_COLLISION_CONTINUOUSCOLLISIONOPTION_HPP_
#define DART_COLLISION_CONTINUOUSCOLLISIONOPTION_HPP_

#include <dart/export.hpp>

#include <functional>

namespace dart {
namespace collision {

class CollisionObject;

/// Advancement strategy for iterative continuous collision queries.
enum class ContinuousCollisionAdvancement
{
  /// Conservative steps never overshoot the first contact.
  Conservative,

  /// Larger steps may report a later contact in exchange for speed.
  Fast
};

struct DART_API ContinuousCollisionOption
{
  using Filter = std::function<bool(const CollisionObject*)>;

  /// Constructor.
  ContinuousCollisionOption(
      bool enableAllHits = false,
      bool sortByTimeOfImpact = false,
      double tolerance = 1e-4,
      int maxIterations = 32,
      ContinuousCollisionAdvancement advancement
      = ContinuousCollisionAdvancement::Conservative,
      Filter filter = nullptr);

  /// Returns true when the filter is not set or allows the object.
  bool passesFilter(const CollisionObject* object) const;

  /// Report every hit instead of only the earliest hit.
  bool mEnableAllHits;

  /// Sort all-hit results by increasing time of impact.
  bool mSortByTimeOfImpact;

  /// Iterative solver convergence tolerance.
  double mTolerance;

  /// Maximum number of iterative solver steps.
  int mMaxIterations;

  /// Advancement strategy for iterative casts.
  ContinuousCollisionAdvancement mAdvancement;

  /// Optional filter to reject hits from specific collision objects.
  Filter mFilter;
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_CONTINUOUSCOLLISIONOPTION_HPP_
