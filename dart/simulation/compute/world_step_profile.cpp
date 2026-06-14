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

#include "dart/simulation/compute/world_step_profile.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace dart::simulation::compute {

namespace {

//==============================================================================
double toMilliseconds(WorldStepProfile::Duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

//==============================================================================
bool hasExecutionDetails(const WorldStepStageProfile& stage)
{
  return stage.acceleration != toMask(ComputeStageAcceleration::None)
         || stage.acceleratedBackendEnabled || !stage.graphProfiles.empty();
}

} // namespace

//==============================================================================
WorldStepStageProfile::Duration WorldStepStageProfile::totalGraphWallTime()
    const noexcept
{
  Duration total{};
  for (const auto& profile : graphProfiles) {
    total += profile.wallTime;
  }
  return total;
}

//==============================================================================
std::size_t WorldStepStageProfile::maxGraphWorkerCount() const noexcept
{
  std::size_t count = 0;
  for (const auto& profile : graphProfiles) {
    count = std::max(count, profile.workerCount);
  }
  return count;
}

//==============================================================================
std::size_t WorldStepStageProfile::maxGraphParallelism() const noexcept
{
  std::size_t parallelism = 0;
  for (const auto& profile : graphProfiles) {
    parallelism = std::max(parallelism, profile.maxParallelism);
  }
  return parallelism;
}

//==============================================================================
bool WorldStepProfile::isEmpty() const noexcept
{
  return stages.empty();
}

//==============================================================================
WorldStepProfile::Duration WorldStepProfile::totalStageTime() const noexcept
{
  Duration total{};
  for (const auto& stage : stages) {
    total += stage.duration;
  }
  return total;
}

//==============================================================================
const WorldStepStageProfile* WorldStepProfile::getStage(
    std::string_view name) const
{
  const auto it = std::ranges::find(stages, name, &WorldStepStageProfile::name);
  return it == stages.end() ? nullptr : &*it;
}

//==============================================================================
std::string WorldStepProfile::toSummaryText() const
{
  if (stages.empty()) {
    return "No World step profile captured. Enable profiling with "
           "World::setStepProfilingEnabled(true) and run a step.\n";
  }

  const double wallMs = toMilliseconds(wallTime);
  const auto sharePercent = [wallMs](double ms) {
    return wallMs > 0.0 ? 100.0 * ms / wallMs : 0.0;
  };

  // Sort a view of the stages slowest-first; the stored order (pipeline order)
  // is left untouched so callers can still read stages in execution sequence.
  std::vector<const WorldStepStageProfile*> ordered;
  ordered.reserve(stages.size());
  for (const auto& stage : stages) {
    ordered.push_back(&stage);
  }
  std::ranges::sort(ordered, [](const auto* lhs, const auto* rhs) {
    return lhs->duration > rhs->duration;
  });

  std::ostringstream out;
  out << "=== World Step Profile ===\n";
  out << "steps=" << stepCount << "  wall=" << std::fixed
      << std::setprecision(3) << wallMs << " ms\n\n";
  out << std::left << std::setw(34) << "Stage" << std::setw(16) << "Domain"
      << std::right << std::setw(14) << "Time (ms)" << std::setw(10) << "% wall"
      << '\n';
  out << std::string(74, '-') << '\n';
  for (const auto* stage : ordered) {
    const double ms = toMilliseconds(stage->duration);
    out << std::left << std::setw(34) << stage->name << std::setw(16)
        << std::string(toString(stage->domain)) << std::right << std::setw(14)
        << std::fixed << std::setprecision(3) << ms << std::setw(9)
        << std::fixed << std::setprecision(1) << sharePercent(ms) << '%'
        << '\n';
  }
  out << std::string(74, '-') << '\n';
  const auto stageTime = totalStageTime();
  const double overheadMs = toMilliseconds(
      wallTime > stageTime ? wallTime - stageTime : Duration{});
  out << std::left << std::setw(34) << "(unattributed overhead)"
      << std::setw(16) << "" << std::right << std::setw(14) << std::fixed
      << std::setprecision(3) << overheadMs << std::setw(9) << std::fixed
      << std::setprecision(1) << sharePercent(overheadMs) << '%' << '\n';

  bool printedDetailsHeader = false;
  for (const auto* stage : ordered) {
    if (!hasExecutionDetails(*stage)) {
      continue;
    }
    if (!printedDetailsHeader) {
      out << "\nExecution details:\n";
      printedDetailsHeader = true;
    }

    out << "- " << stage->name
        << ": acceleration=" << formatAccelerationMask(stage->acceleration);
    if (stage->acceleratedBackendEnabled) {
      out << ", backend=accelerated";
    }
    if (!stage->graphProfiles.empty()) {
      out << ", graph_profiles=" << stage->graphProfiles.size()
          << ", graph_wall=" << std::fixed << std::setprecision(3)
          << toMilliseconds(stage->totalGraphWallTime()) << " ms"
          << ", max_workers=" << stage->maxGraphWorkerCount()
          << ", max_parallelism=" << stage->maxGraphParallelism();
    }
    out << '\n';
  }
  return out.str();
}

//==============================================================================
void WorldStepProfile::reset() noexcept
{
  stepCount = 0;
  wallTime = Duration{};
  stages.clear();
}

//==============================================================================
bool ResolvedSolverConfiguration::hasSubstitution() const noexcept
{
  return std::ranges::any_of(notes, [](const ResolvedConfigurationNote& note) {
    return note.isSubstitution();
  });
}

//==============================================================================
std::string ResolvedSolverConfiguration::toSummaryText() const
{
  if (notes.empty()) {
    return "resolved solver configuration: (none recorded)\n";
  }
  std::ostringstream out;
  out << "resolved solver configuration:\n";
  for (const ResolvedConfigurationNote& note : notes) {
    out << "  " << note.domain << ": " << note.requested << " -> "
        << note.resolved << " (" << note.reason << ")\n";
  }
  return out.str();
}

} // namespace dart::simulation::compute
