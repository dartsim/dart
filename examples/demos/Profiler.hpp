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

#ifndef DART_EXAMPLES_DEMOS_PROFILER_HPP_
#define DART_EXAMPLES_DEMOS_PROFILER_HPP_

#include <string>

namespace dart_demos {

//==============================================================================
/// Diagnostics-panel live profiler. Wraps dart::common::profile's built-in
/// text backend (dart/common/Profile.hpp), which compiles to real
/// functionality when DART_BUILD_PROFILE is on (pixi's config task turns it
/// on) and to inert no-ops otherwise -- isTextProfilingEnabled() tells us
/// which, so this class needs no #if of its own. Recording state persists
/// across scene switches (it profiles the app/World::step machinery in
/// general, not any one scene).
class Profiler
{
public:
  /// Renders the "Record profile" toggle and, while recording, the
  /// monospaced summary text (refreshed every ~30 calls rather than every
  /// frame, since re-formatting the full summary on every single frame would
  /// itself be a needless cost). Call once per rendered frame from
  /// DemoHost::renderDiagnostics.
  void render();

  /// Test/debug-only hook: forces the recording state without going through
  /// the checkbox (used by main.cpp's hidden --debug-record-profile flag so
  /// a headless capture can show the profiler actually recording). Mirrors
  /// the checkbox handler's own resetProfile()-on-enable behavior.
  void setRecordingForTest(bool on);

private:
  bool mRecording = false;
  std::string mSummaryText;
  int mFramesSinceRefresh = 0;

  static constexpr int kRefreshEveryNFrames = 30;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_PROFILER_HPP_
