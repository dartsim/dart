/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
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
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <algorithm>

#include <cmath>

namespace dart::simulation::detail::newton_barrier {

[[nodiscard]] inline double sanitizeProjectedNewtonTolerance(
    const double tolerance) noexcept
{
  return std::isfinite(tolerance) ? std::max(0.0, tolerance) : 0.0;
}

[[nodiscard]] inline double projectedNewtonResidualNormFromSquared(
    const double squaredResidual) noexcept
{
  if (!(std::isfinite(squaredResidual) && squaredResidual > 0.0)) {
    return 0.0;
  }

  return std::sqrt(squaredResidual);
}

[[nodiscard]] inline bool projectedNewtonResidualConverged(
    const double residualNorm, const double tolerance) noexcept
{
  return std::isfinite(residualNorm)
         && residualNorm <= sanitizeProjectedNewtonTolerance(tolerance);
}

[[nodiscard]] inline bool projectedNewtonSquaredResidualConverged(
    const double squaredResidual, const double tolerance) noexcept
{
  const double sanitizedTolerance = sanitizeProjectedNewtonTolerance(tolerance);
  return std::isfinite(squaredResidual)
         && squaredResidual <= sanitizedTolerance * sanitizedTolerance;
}

[[nodiscard]] inline double projectedNewtonEffectiveGradientTolerance(
    const double absoluteTolerance,
    const double relativeTolerance,
    const double initialGradientNorm) noexcept
{
  const double absolute = sanitizeProjectedNewtonTolerance(absoluteTolerance);
  const double relative = sanitizeProjectedNewtonTolerance(relativeTolerance);
  const double initial = std::isfinite(initialGradientNorm)
                             ? std::max(0.0, initialGradientNorm)
                             : 0.0;
  return std::max(absolute, relative * initial);
}

} // namespace dart::simulation::detail::newton_barrier
