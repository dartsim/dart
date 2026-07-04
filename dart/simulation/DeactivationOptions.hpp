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
 *
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

#ifndef DART_SIMULATION_DEACTIVATIONOPTIONS_HPP_
#define DART_SIMULATION_DEACTIVATIONOPTIONS_HPP_

namespace dart {
namespace simulation {

/// Options controlling automatic body deactivation ("sleeping").
///
/// A solver island whose every skeleton has come to rest is "frozen": its
/// forward dynamics (including gravity), its constraint solve, and its position
/// integration are all skipped together each step until the island is
/// disturbed. This yields a large speedup for big resting scenes while adding
/// negligible overhead to small or active scenes.
///
/// Deactivation is decided per solver island and is a deterministic function of
/// stable state (cached body speeds, dwell time, and the thresholds below), so
/// it does not depend on iteration or container order.
struct DeactivationOptions
{
  /// Whether automatic deactivation is enabled. Defaults to true so resting
  /// scenes benefit from the performance path without application changes.
  /// Downstream integrations that require legacy always-active stepping can
  /// set this to false; when disabled, no skeleton is ever flagged as resting
  /// and behavior is byte-identical to having the feature absent.
  bool mEnabled = true;

  /// Maximum linear speed (m/s) of any body in a skeleton for that skeleton to
  /// be considered quiet enough to begin sleeping. Becoming a sleep candidate
  /// additionally requires the smoothed speed to drop below 10% of this value
  /// (the final-quiet gate), so both bounds scale together when the threshold
  /// is tuned for scenes with a higher contact-solver jitter floor.
  double mLinearSpeedThreshold = 0.01;

  /// Maximum angular speed (rad/s) of any body in a skeleton for that skeleton
  /// to be considered quiet enough to begin sleeping. Becoming a sleep
  /// candidate additionally requires the smoothed speed to drop below 20% of
  /// this value (the final-quiet gate), so both bounds scale together when the
  /// threshold is tuned.
  double mAngularSpeedThreshold = 0.05;

  /// Duration (seconds) of sustained sub-threshold motion that must elapse
  /// before a skeleton is allowed to sleep.
  double mTimeUntilSleep = 0.5;

  /// Hysteresis factor applied to the sleep thresholds to obtain the wake
  /// thresholds (wake threshold = scale * sleep threshold). Using a larger wake
  /// band than sleep band prevents repeated sleep/wake thrashing near the
  /// thresholds.
  double mWakeThresholdScale = 2.0;
};

} // namespace simulation
} // namespace dart

#endif // DART_SIMULATION_DEACTIVATIONOPTIONS_HPP_
