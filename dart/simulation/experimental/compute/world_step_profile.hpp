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

#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <chrono>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::compute {

/// Wall-clock timing for one stage of a profiled World step.
struct DART_EXPERIMENTAL_API WorldStepStageProfile
{
  using Duration = std::chrono::steady_clock::duration;

  /// Stage name (see `WorldStepStage::getName`), e.g. "rigid_body_contact".
  std::string name;

  /// Broad domain the stage participates in (see
  /// `ComputeStageMetadata::domain`).
  ComputeStageDomain domain = ComputeStageDomain::Generic;

  /// Wall-clock time spent in this stage's `execute` during the step.
  Duration duration{};
};

/// Per-stage wall-clock profile of the most recent profiled World step.
///
/// This is the experimental World's text-first profiling surface: an opt-in,
/// allocation-light breakdown of where one `World::step` spent its time, stage
/// by stage. It is produced only when DART is built with `DART_BUILD_PROFILE`
/// and step profiling is enabled at runtime (`World::setStepProfilingEnabled`);
/// when the build option is off, the runtime toggle is a no-op and the hot step
/// path compiles back to the normal pipeline. The profile is intended for
/// non-GUI performance analysis -- including by AI agents -- through
/// `toSummaryText()` or the individual fields. It is empty before the first
/// profiled step.
///
/// The stage breakdown is the same domain-neutral pipeline surface the executor
/// runs (rigid-body, multibody, deformable, kinematics, ...), so the report is
/// stable across solver choices and reads the same from C++, Python, or a log.
struct DART_EXPERIMENTAL_API WorldStepProfile
{
  using Duration = std::chrono::steady_clock::duration;

  /// Number of steps captured into this snapshot. The `step` overloads
  /// overwrite the profile each step, so this is 1 after a single step and
  /// remains 1 after `step(count)` (the snapshot reflects the last step).
  std::size_t stepCount = 0;

  /// End-to-end wall time of the profiled step, including any per-stage
  /// scheduling overhead not attributed to an individual stage.
  Duration wallTime{};

  /// Per-stage timings, in pipeline execution order.
  std::vector<WorldStepStageProfile> stages;

  /// True before the first profiled step (no stages recorded).
  [[nodiscard]] bool isEmpty() const noexcept;

  /// Sum of all per-stage durations. `wallTime - totalStageTime()` is the
  /// unattributed step overhead (validation, scratch reset, bookkeeping).
  [[nodiscard]] Duration totalStageTime() const noexcept;

  /// Returns the stage profile with the given name, or nullptr if absent.
  [[nodiscard]] const WorldStepStageProfile* getStage(
      std::string_view name) const;

  /// Renders a compact, sorted, human- and agent-readable timing table.
  ///
  /// Stages are listed slowest-first with their domain, duration in
  /// milliseconds, and share of wall time. A trailing row reports the
  /// unattributed step overhead. Returns a short notice when empty.
  [[nodiscard]] std::string toSummaryText() const;

  /// Clears the profile back to the empty state.
  void reset() noexcept;
};

} // namespace dart::simulation::experimental::compute
