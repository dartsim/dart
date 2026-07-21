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

#ifndef DART_EXAMPLES_DEMOS_HEADLESSEXACTFBFFAILFAST_HPP_
#define DART_EXAMPLES_DEMOS_HEADLESSEXACTFBFFAILFAST_HPP_

#include <limits>

#include <cmath>
#include <cstddef>

namespace dart_demos {
namespace detail {

constexpr double kHeadlessExactFbfResidualTolerance = 1e-6;

//==============================================================================
/// Minimal exact-FBF diagnostics needed by the opt-in headless fail-fast gate.
struct HeadlessExactFbfFailFastDiagnostics
{
  std::size_t exactAttempts = 0u;
  std::size_t acceptedAtCap = 0u;
  std::size_t exactFailures = 0u;
  std::size_t boxedFallbacks = 0u;
  double residual = std::numeric_limits<double>::quiet_NaN();
  double worstResidual = std::numeric_limits<double>::quiet_NaN();
};

//==============================================================================
/// Result of evaluating one completed-step diagnostics record.
struct HeadlessExactFbfFailFastDecision
{
  bool triggered = false;
  const char* reason = nullptr;
};

//==============================================================================
/// Evaluates one completed-step diagnostics record in stable priority order.
///
/// Non-finite residuals are expected before the first exact solve, so they are
/// accepted while `exactAttempts` is zero. Once an attempt has occurred, both
/// the current and worst-to-date residual must be finite and within tolerance.
inline HeadlessExactFbfFailFastDecision evaluateHeadlessExactFbfFailFast(
    const HeadlessExactFbfFailFastDiagnostics& diagnostics,
    double residualTolerance = kHeadlessExactFbfResidualTolerance)
{
  if (diagnostics.boxedFallbacks > 0u)
    return {true, "boxed_fallback"};
  if (diagnostics.exactFailures > 0u)
    return {true, "exact_failure"};
  if (diagnostics.acceptedAtCap > 0u)
    return {true, "iteration_cap"};
  if (diagnostics.exactAttempts == 0u)
    return {};
  if (!std::isfinite(diagnostics.residual)
      || !std::isfinite(diagnostics.worstResidual)) {
    return {true, "nonfinite_residual"};
  }
  if (diagnostics.residual > residualTolerance
      || diagnostics.worstResidual > residualTolerance) {
    return {true, "residual_tolerance_exceeded"};
  }
  return {};
}

} // namespace detail
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_HEADLESSEXACTFBFFAILFAST_HPP_
