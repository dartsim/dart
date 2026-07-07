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

#ifndef DART_EXAMPLES_DEMOS_LOGCAPTURE_HPP_
#define DART_EXAMPLES_DEMOS_LOGCAPTURE_HPP_

#include <functional>
#include <memory>
#include <streambuf>
#include <string>

namespace dart_demos {

//==============================================================================
enum class CapturedLogLevel
{
  Info,
  Warning,
  Error
};

//==============================================================================
/// RAII redirect of std::cout/std::cerr into a line sink. Every character is
/// forwarded to the original stream buffer unchanged (so console output is
/// completely unaffected), while complete lines -- with ANSI color escapes
/// stripped -- are also collected and handed to `sink`. This is how
/// dart-demos captures dart::common's dtmsg/dtwarn/dterr output into its log
/// console without any dart/ changes: those macros
/// (dart/common/Console.hpp) write straight to std::cout/std::cerr with an
/// SGR color prefix (see colorMsg/colorErr in dart/common/Console.cpp), which
/// this class also uses to recover the severity DART itself assigned.
/// Restores the original stream buffers on destruction; construct at most
/// one instance for the process lifetime.
class LogCapture
{
public:
  using LineSink = std::function<void(CapturedLogLevel, const std::string&)>;

  explicit LogCapture(LineSink sink);
  ~LogCapture();

  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

private:
  class TeeBuf;

  std::unique_ptr<TeeBuf> mCoutBuf;
  std::unique_ptr<TeeBuf> mCerrBuf;
  std::streambuf* mOldCoutBuf;
  std::streambuf* mOldCerrBuf;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_LOGCAPTURE_HPP_
