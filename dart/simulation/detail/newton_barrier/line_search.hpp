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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 *   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *   SUCH DAMAGE.
 */

#pragma once

#include <dart/collision/native/types.hpp>

#include <algorithm>

#include <cmath>
#include <cstddef>

namespace dart::simulation::detail::newton_barrier {

inline constexpr double kDefaultSufficientDecreaseFactor = 1e-4;
inline constexpr double kDefaultBacktrackingScale = 0.5;

struct LineSearchOptions
{
  double minSeparation = 0.0;
  double tolerance = 1e-6;
  int maxIterations = 64;
};

struct LineSearchStats
{
  std::size_t pointPointChecks = 0;
  std::size_t pointEdgeChecks = 0;
  std::size_t edgeEdgeChecks = 0;
  std::size_t pointTriangleChecks = 0;
  std::size_t hits = 0;
  std::size_t misses = 0;
  std::size_t indeterminate = 0;
  std::size_t zeroStepCount = 0;
};

struct LineSearchCcdOutcome
{
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;
};

[[nodiscard]] inline collision::native::CcdOption makeLineSearchCcdOption(
    const LineSearchOptions& options)
{
  collision::native::CcdOption ccdOption;
  ccdOption.minSeparation = std::max(0.0, options.minSeparation);
  ccdOption.tolerance = std::max(0.0, options.tolerance);
  ccdOption.maxIterations = std::max(1, options.maxIterations);
  ccdOption.advancement = collision::native::CcdAdvancement::Conservative;
  return ccdOption;
}

[[nodiscard]] inline bool allowsPositiveLineSearchStep(
    const double stepBound, const bool indeterminate) noexcept
{
  return stepBound > 0.0 && !indeterminate;
}

[[nodiscard]] inline bool allowsFullLineSearchStep(
    const double stepBound,
    const bool limited,
    const bool indeterminate) noexcept
{
  return allowsPositiveLineSearchStep(stepBound, indeterminate)
         && (!limited || stepBound >= 1.0);
}

[[nodiscard]] inline double makeLineSearchStepScale(
    const double stepBound, const double safetyScale = 1.0) noexcept
{
  const double clampedStepBound = std::clamp(stepBound, 0.0, 1.0);
  const double clampedSafetyScale = std::clamp(safetyScale, 0.0, 1.0);
  if (!std::isfinite(clampedStepBound) || !std::isfinite(clampedSafetyScale)) {
    return 0.0;
  }

  const double stepScale
      = std::clamp(clampedStepBound * clampedSafetyScale, 0.0, 1.0);
  return std::isfinite(stepScale) ? stepScale : 0.0;
}

[[nodiscard]] inline double makeInteriorLineSearchStepScale(
    const double stepBound) noexcept
{
  const double stepScale = makeLineSearchStepScale(stepBound);
  if (stepScale <= 0.0) {
    return 0.0;
  }

  const double interiorStepScale = std::nextafter(stepScale, 0.0);
  if (interiorStepScale <= 0.0 || !std::isfinite(interiorStepScale)) {
    return 0.0;
  }

  return interiorStepScale;
}

[[nodiscard]] inline double sanitizeSufficientDecreaseFactor(
    const double factor) noexcept
{
  if (!std::isfinite(factor)) {
    return kDefaultSufficientDecreaseFactor;
  }

  return std::clamp(factor, 0.0, std::nextafter(1.0, 0.0));
}

[[nodiscard]] inline double sanitizeBacktrackingScale(
    const double scale) noexcept
{
  if (!(std::isfinite(scale) && scale > 0.0 && scale < 1.0)) {
    return kDefaultBacktrackingScale;
  }

  return scale;
}

[[nodiscard]] inline double sufficientDecreaseThreshold(
    const double currentValue,
    const double scaledDirectionalDerivative,
    const double factor = kDefaultSufficientDecreaseFactor) noexcept
{
  return currentValue
         + sanitizeSufficientDecreaseFactor(factor)
               * scaledDirectionalDerivative;
}

[[nodiscard]] inline bool satisfiesSufficientDecrease(
    const double currentValue,
    const double candidateValue,
    const double scaledDirectionalDerivative,
    const double factor = kDefaultSufficientDecreaseFactor) noexcept
{
  if (!std::isfinite(currentValue) || !std::isfinite(candidateValue)
      || !std::isfinite(scaledDirectionalDerivative)) {
    return false;
  }

  return candidateValue <= sufficientDecreaseThreshold(
             currentValue, scaledDirectionalDerivative, factor);
}

[[nodiscard]] inline double recordLineSearchHit(
    LineSearchStats& stats, const double timeOfImpact) noexcept
{
  ++stats.hits;
  const double stepBound = std::clamp(timeOfImpact, 0.0, 1.0);
  if (stepBound <= 0.0) {
    ++stats.zeroStepCount;
  }
  return stepBound;
}

[[nodiscard]] inline LineSearchCcdOutcome recordLineSearchCcdOutcome(
    LineSearchStats& stats,
    const collision::native::CcdPrimitiveResult& primitive) noexcept
{
  LineSearchCcdOutcome outcome;
  if (primitive.hit) {
    outcome.hit = true;
    outcome.stepBound = recordLineSearchHit(stats, primitive.timeOfImpact);
    return outcome;
  }

  if (primitive.status
      == collision::native::CcdPrimitiveStatus::Indeterminate) {
    ++stats.indeterminate;
    outcome.indeterminate = true;
    outcome.stepBound = std::clamp(primitive.timeOfImpact, 0.0, 1.0);
    return outcome;
  }

  ++stats.misses;
  return outcome;
}

inline void accumulateLineSearchStats(
    LineSearchStats& total, const LineSearchStats& addend)
{
  total.pointPointChecks += addend.pointPointChecks;
  total.pointEdgeChecks += addend.pointEdgeChecks;
  total.edgeEdgeChecks += addend.edgeEdgeChecks;
  total.pointTriangleChecks += addend.pointTriangleChecks;
  total.hits += addend.hits;
  total.misses += addend.misses;
  total.indeterminate += addend.indeterminate;
  total.zeroStepCount += addend.zeroStepCount;
}

} // namespace dart::simulation::detail::newton_barrier
