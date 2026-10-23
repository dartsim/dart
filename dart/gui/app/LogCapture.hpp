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

#ifndef DART_GUI_APP_LOGCAPTURE_HPP_
#define DART_GUI_APP_LOGCAPTURE_HPP_

#include <dart/common/Logging.hpp>

#include <functional>
#include <memory>
#include <streambuf>
#include <string>

namespace dart::gui::app {

//==============================================================================
enum class CapturedLogLevel
{
  Info,
  Warning,
  Error
};

//==============================================================================
/// Observes callback-consumed DART records, wraps sink output and tees other
/// stdout/stderr lines. Restores stream buffers on destruction and the previous
/// sink only if this capture's wrapper is still installed. Stream buffers
/// require reverse construction order among captures; other sink wrappers may
/// be disabled in either order. Construction, drain and destruction belong to
/// one owner thread; direct stdout/stderr writers must be quiescent during
/// stream-buffer installation/removal. Native record producers may keep
/// running. The newest 2000 records wait for drain; output forwarding is
/// immediate.
class LogCapture
{
public:
  using LineSink = std::function<void(CapturedLogLevel, const std::string&)>;

  explicit LogCapture(LineSink sink);
  ~LogCapture();

  /// Deliver the queued snapshot on the owner thread. Panel callbacks may log,
  /// throw, or destroy this capture; queued records are discarded on
  /// destruction.
  void drain();

  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

private:
  class TeeBuf;
  struct SinkState;

  std::shared_ptr<SinkState> mSinkState;
  std::shared_ptr<common::LogSink> mInstalledSink;

  std::unique_ptr<TeeBuf> mCoutBuf;
  std::unique_ptr<TeeBuf> mCerrBuf;
  std::streambuf* mOldCoutBuf;
  std::streambuf* mOldCerrBuf;
};

} // namespace dart::gui::app

#endif // DART_GUI_APP_LOGCAPTURE_HPP_
