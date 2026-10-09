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

/// Cumulative friction-solver statistics. Counters can be differenced around a
/// step. maxViolation is a maximum since resetStats(); reset while no solves
/// are running to obtain a per-step maximum.
///
/// New in DART 6.20: no released version has this struct, so fields may still
/// be added before 6.20.0. From 6.20.0 on, its layout is part of the ABI.
struct FrictionSolveStats
{
  std::uint64_t numSolves = 0;
  std::uint64_t numConverged = 0;
  /// Solves that accepted their best finite iterate at the iteration cap or
  /// after an unsuccessful iteration with no secondary available.
  std::uint64_t numAcceptedAtCap = 0;
  /// Solves that returned false, requesting the secondary solver.
  std::uint64_t numFailed = 0;
  std::uint64_t numContacts = 0;
  /// Contacts solved with the box law.
  std::uint64_t numBoxContacts = 0;
  /// Local contact QPs that needed the slow certificate path.
  std::uint64_t numLocalFallbacks = 0;
  /// Gauss-Seidel sweeps (NSGS) or outer iterations (FBF; groups without a
  /// frictional cone contact count one per plain Gauss-Seidel sweep).
  std::uint64_t numIterations = 0;
  /// FBF Gauss-Seidel sweeps, including sweeps for rejected step sizes and the
  /// plain sweeps of groups without a frictional cone contact.
  std::uint64_t numInnerIterations = 0;
  /// FBF step-size reductions.
  std::uint64_t numStepShrinks = 0;
  /// FBF inner solves stopped by maxInnerSweeps above their target.
  std::uint64_t numInnerCaps = 0;
  /// Largest final law violation [m/s] among accepted solves (converged or at
  /// the cap) since resetStats(). Failed solves do not contribute.
  double maxViolation = 0.0;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_FRICTIONSOLVESTATS_HPP_
