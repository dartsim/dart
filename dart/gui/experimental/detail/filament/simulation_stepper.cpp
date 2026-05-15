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

#include <dart/gui/experimental/detail/filament/simulation_stepper.hpp>

#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/simulation/world.hpp>

#include <algorithm>
#include <chrono>

namespace dart::gui::experimental::filament {

using dart::gui::experimental::ProfileAccumulator;
using dart::gui::experimental::RunOptions;
using dart::gui::experimental::ViewerLifecycleState;
using dart::gui::experimental::elapsedMs;
using dart::gui::experimental::markSimulationAdvanced;
using dart::gui::experimental::shouldAdvanceSimulation;

std::size_t SimulationStepper::stepsToRun(
    const RunOptions& options,
    const ViewerLifecycleState& lifecycle,
    double timeStep)
{
  if (!shouldAdvanceSimulation(lifecycle)) {
    mLastSimulationClock = ProfileAccumulator::Clock::now();
    mSimulationAccumulator = 0.0;
    return 0;
  }

  if (options.headless || lifecycle.stepOnce) {
    return 1;
  }

  constexpr std::size_t kMaxSimulationStepsPerRenderedFrame = 64;
  std::size_t simulationStepsToRun = 0;
  const auto now = ProfileAccumulator::Clock::now();
  mSimulationAccumulator
      += std::chrono::duration<double>(now - mLastSimulationClock).count();
  mLastSimulationClock = now;
  if (timeStep > 0.0) {
    mSimulationAccumulator = std::min(
        mSimulationAccumulator,
        timeStep * static_cast<double>(kMaxSimulationStepsPerRenderedFrame));
    while (simulationStepsToRun < kMaxSimulationStepsPerRenderedFrame
           && mSimulationAccumulator + 1e-12 >= timeStep) {
      ++simulationStepsToRun;
      mSimulationAccumulator -= timeStep;
    }
  }

  return simulationStepsToRun;
}

bool advanceSimulationSteps(
    DartScene& scene,
    std::size_t simulationStepsToRun,
    ViewerLifecycleState& lifecycle,
    ProfileAccumulator& profile)
{
  if (simulationStepsToRun == 0) {
    return false;
  }

  const auto phaseStart = ProfileAccumulator::Clock::now();
  for (std::size_t i = 0; i < simulationStepsToRun; ++i) {
    const double timeStep = scene.world->getTimeStep();
    if (scene.preStep) {
      scene.preStep();
    }
    scene.world->step();
    profile.simulatedMs += timeStep * 1000.0;
  }
  markSimulationAdvanced(lifecycle);
  profile.simulationSteps += simulationStepsToRun;
  profile.simulationMs += elapsedMs(phaseStart);
  return true;
}

} // namespace dart::gui::experimental::filament
