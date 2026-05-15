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

#include <dart/gui/experimental/profile.hpp>

#include <algorithm>
#include <iostream>

namespace dart::gui::experimental {

double elapsedMs(ProfileAccumulator::Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(
             ProfileAccumulator::Clock::now() - start)
      .count();
}

void printProfile(const ProfileAccumulator& profile)
{
  if (profile.frames == 0) {
    return;
  }

  const double frames = static_cast<double>(profile.frames);
  const double renderedFrames
      = static_cast<double>(std::max<std::size_t>(profile.renderedFrames, 1));
  const double averageFrameMs = profile.frameMs / frames;
  const double fps = averageFrameMs > 0.0 ? 1000.0 / averageFrameMs : 0.0;
  const double renderedFps
      = profile.frameMs > 0.0
            ? 1000.0 * static_cast<double>(profile.renderedFrames)
                  / profile.frameMs
            : 0.0;
  const auto avg = [&](double value) {
    return value / frames;
  };
  const auto avgRendered = [&](double value) {
    return value / renderedFrames;
  };

  std::cout << "Profile frames: " << profile.frames
            << " rendered=" << profile.renderedFrames
            << " skipped=" << profile.skippedFrames << "\n";
  std::cout << "Profile average: frame=" << averageFrameMs
            << " ms loop_fps=" << fps << " rendered_fps=" << renderedFps
            << " max_frame=" << profile.maxFrameMs
            << " ms max_render=" << profile.maxRenderMs << " ms\n";
  std::cout << "Profile phases (ms/frame): input=" << avg(profile.inputMs)
            << " viewport_camera=" << avg(profile.viewportCameraMs)
            << " simulation=" << avg(profile.simulationMs)
            << " contact_debug=" << avg(profile.contactDebugMs)
            << " extraction=" << avg(profile.extractionMs)
            << " sync=" << avg(profile.syncMs)
            << " interaction=" << avg(profile.interactionMs)
            << " selection_debug=" << avg(profile.selectionDebugMs)
            << " ui=" << avg(profile.uiMs)
            << " begin_frame=" << avg(profile.beginFrameMs)
            << " render=" << avg(profile.renderMs) << "\n";
  std::cout << "Profile per rendered frame (ms): elapsed="
            << avgRendered(profile.frameMs)
            << " begin_frame=" << avgRendered(profile.beginFrameMs)
            << " simulation=" << avgRendered(profile.simulationMs)
            << " extraction=" << avgRendered(profile.extractionMs)
            << " sync=" << avgRendered(profile.syncMs)
            << " render=" << avgRendered(profile.renderMs) << "\n";
  const double realTimeFactor
      = profile.frameMs > 0.0 ? profile.simulatedMs / profile.frameMs : 0.0;
  std::cout << "Profile simulation: steps=" << profile.simulationSteps
            << " simulated=" << profile.simulatedMs
            << " ms real_time_factor=" << realTimeFactor << "\n";
  if (profile.screenshotWaitMs > 0.0 || profile.screenshotSaveMs > 0.0) {
    std::cout << "Profile screenshot: wait=" << profile.screenshotWaitMs
              << " ms save=" << profile.screenshotSaveMs << " ms\n";
  }
}

} // namespace dart::gui::experimental
