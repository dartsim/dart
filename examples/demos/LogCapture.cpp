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

#include "LogCapture.hpp"

#include <iostream>

namespace dart_demos {

namespace {

//==============================================================================
/// Strips ANSI SGR escape sequences ("\033[...m") from `text`.
std::string stripAnsi(const std::string& text)
{
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] == '\033' && i + 1 < text.size() && text[i + 1] == '[') {
      std::size_t j = i + 2;
      while (j < text.size() && text[j] != 'm')
        ++j;
      i = j; // the loop's ++i lands just past the terminating 'm'
      continue;
    }
    out.push_back(text[i]);
  }
  return out;
}

//==============================================================================
/// dtmsg/dtwarn/dterr (dart/common/Console.hpp) each open their line with a
/// distinct SGR color code -- "\033[1;32m" (msg), "\033[1;33m" (warn),
/// "\033[1;31m" (error); see colorMsg/colorErr in Console.cpp -- before any
/// user text. Sniff that prefix to recover the severity DART itself
/// assigned; uncolored lines from other code fall back to a per-stream
/// guess (stdout -> Info, stderr -> Warning).
CapturedLogLevel sniffSeverity(
    const std::string& raw, CapturedLogLevel fallback)
{
  const std::size_t pos = raw.find("\033[1;");
  if (pos == std::string::npos || pos + 6 > raw.size())
    return fallback;
  const std::string code = raw.substr(pos + 4, 2);
  if (code == "31")
    return CapturedLogLevel::Error;
  if (code == "33")
    return CapturedLogLevel::Warning;
  if (code == "32")
    return CapturedLogLevel::Info;
  return fallback;
}

} // namespace

//==============================================================================
/// Forwards every character to the stream's original buffer (so console
/// output is byte-for-byte unaffected), while accumulating complete lines to
/// hand to `mSink`. Never buffers internally beyond the current partial
/// line, so no data is lost if the process exits mid-write.
class LogCapture::TeeBuf : public std::streambuf
{
public:
  TeeBuf(
      std::streambuf* original, CapturedLogLevel fallbackLevel, LineSink sink)
    : mOriginal(original), mFallbackLevel(fallbackLevel), mSink(std::move(sink))
  {
  }

protected:
  int_type overflow(int_type c) override
  {
    if (mOriginal)
      mOriginal->sputc(traits_type::to_char_type(c));
    if (traits_type::eq_int_type(c, traits_type::eof()))
      return traits_type::not_eof(c);

    const char ch = traits_type::to_char_type(c);
    if (ch == '\n')
      flushLine();
    else
      mLineBuf.push_back(ch);
    return c;
  }

  std::streamsize xsputn(const char* s, std::streamsize n) override
  {
    if (mOriginal)
      mOriginal->sputn(s, n);
    for (std::streamsize i = 0; i < n; ++i) {
      if (s[i] == '\n')
        flushLine();
      else
        mLineBuf.push_back(s[i]);
    }
    return n;
  }

private:
  void flushLine()
  {
    if (mLineBuf.empty())
      return;

    try {
      if (mSink) {
        const CapturedLogLevel level = sniffSeverity(mLineBuf, mFallbackLevel);
        mSink(level, stripAnsi(mLineBuf));
      }
    } catch (...) {
      // The log console must never crash the app, even if the sink throws.
    }
    mLineBuf.clear();
  }

  std::streambuf* mOriginal;
  CapturedLogLevel mFallbackLevel;
  LineSink mSink;
  std::string mLineBuf;
};

//==============================================================================
LogCapture::LogCapture(LineSink sink)
{
  mCoutBuf = std::make_unique<TeeBuf>(
      std::cout.rdbuf(), CapturedLogLevel::Info, sink);
  mCerrBuf = std::make_unique<TeeBuf>(
      std::cerr.rdbuf(), CapturedLogLevel::Warning, sink);
  mOldCoutBuf = std::cout.rdbuf(mCoutBuf.get());
  mOldCerrBuf = std::cerr.rdbuf(mCerrBuf.get());
}

//==============================================================================
LogCapture::~LogCapture()
{
  std::cout.rdbuf(mOldCoutBuf);
  std::cerr.rdbuf(mOldCerrBuf);
}

} // namespace dart_demos
