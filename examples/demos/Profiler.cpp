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

#include "Profiler.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/common/Profile.hpp>

namespace dart_demos {

//==============================================================================
void Profiler::render()
{
  if (!dart::common::profile::isTextProfilingEnabled()) {
    ImGui::TextDisabled("Built without profiling (DART_BUILD_PROFILE is off).");
    return;
  }

  if (ImGui::Checkbox("Record profile", &mRecording)) {
    if (mRecording) {
      dart::common::profile::resetProfile();
      mSummaryText.clear();
      mFramesSinceRefresh = 0;
    }
  }

  if (!mRecording) {
    ImGui::TextDisabled("Recording is off.");
    return;
  }

  // Re-formatting the full summary text is real work (it walks every
  // recorded scope); throttle it to roughly every 30 rendered frames rather
  // than doing it unconditionally every single frame.
  if (mSummaryText.empty() || mFramesSinceRefresh >= kRefreshEveryNFrames) {
    mSummaryText = dart::common::profile::getProfileSummaryText();
    mFramesSinceRefresh = 0;
  }
  ++mFramesSinceRefresh;

  // Height is font-relative so it tracks the GUI-scale slider (io.FontGlobal
  // scale) like the sibling Scene Tree / Log console child regions.
  ImGui::BeginChild(
      "##profile_summary", ImVec2(0.0f, ImGui::GetFontSize() * 8.0f), true);
  ImGui::TextUnformatted(
      mSummaryText.empty() ? "(no samples yet)" : mSummaryText.c_str());
  ImGui::EndChild();
}

//==============================================================================
void Profiler::setRecordingForTest(bool on)
{
  if (on && !mRecording) {
    dart::common::profile::resetProfile();
    mSummaryText.clear();
    mFramesSinceRefresh = 0;
  }
  mRecording = on;
}

} // namespace dart_demos
