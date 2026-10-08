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

#ifndef DART_CONSTRAINT_FRICTIONSOLVESTATS_HPP_
#define DART_CONSTRAINT_FRICTIONSOLVESTATS_HPP_

#include <cstdint>

namespace dart {
namespace constraint {

/// Cumulative friction-solver statistics. Difference snapshots around a step,
/// or reset the counters while no solves are running.
struct FrictionSolveStats
{
  std::uint64_t numSolves = 0;
  std::uint64_t numConverged = 0;
  /// Solves that returned their best finite iterate at the iteration cap.
  std::uint64_t numAcceptedAtCap = 0;
  /// Solves that returned false, requesting the secondary solver.
  std::uint64_t numFailed = 0;
  std::uint64_t numContacts = 0;
  /// Contacts solved with the box law.
  std::uint64_t numBoxContacts = 0;
  /// Local contact QPs that needed the slow certificate path.
  std::uint64_t numLocalFallbacks = 0;
  /// Gauss-Seidel sweeps.
  std::uint64_t numIterations = 0;
  /// Largest final law violation [m/s].
  double maxViolation = 0.0;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_FRICTIONSOLVESTATS_HPP_
