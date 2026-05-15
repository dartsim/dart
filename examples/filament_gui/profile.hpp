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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#ifndef EXAMPLES_FILAMENT_GUI_PROFILE_HPP_
#define EXAMPLES_FILAMENT_GUI_PROFILE_HPP_

#include <chrono>

#include <cstddef>

namespace dart::examples::filament_gui {

struct ProfileAccumulator
{
  using Clock = std::chrono::steady_clock;

  std::size_t frames = 0;
  std::size_t renderedFrames = 0;
  std::size_t skippedFrames = 0;
  std::size_t simulationSteps = 0;
  double frameMs = 0.0;
  double simulatedMs = 0.0;
  double inputMs = 0.0;
  double viewportCameraMs = 0.0;
  double simulationMs = 0.0;
  double contactDebugMs = 0.0;
  double extractionMs = 0.0;
  double syncMs = 0.0;
  double interactionMs = 0.0;
  double selectionDebugMs = 0.0;
  double uiMs = 0.0;
  double beginFrameMs = 0.0;
  double renderMs = 0.0;
  double screenshotWaitMs = 0.0;
  double screenshotSaveMs = 0.0;
  double maxFrameMs = 0.0;
  double maxRenderMs = 0.0;
};

double elapsedMs(ProfileAccumulator::Clock::time_point start);

void printProfile(const ProfileAccumulator& profile);

} // namespace dart::examples::filament_gui

#endif // EXAMPLES_FILAMENT_GUI_PROFILE_HPP_
