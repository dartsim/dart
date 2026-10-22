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

#include "dart/common/Console.hpp"

#include <dart/common/Logging.hpp>

#include <exception>
#include <iostream>
#include <streambuf>
#include <utility>

namespace dart {
namespace common {

namespace {

thread_local bool insideConsoleEmit = false;

// Each thread assembles its own lines before entering the serialized sink.
class ConsoleBuffer : public std::streambuf
{
public:
  explicit ConsoleBuffer(LogLevel level) : mLevel(level) {}

  void markPrefix(int color)
  {
    mPrefixLength = mText.size();
    mColor = color;
  }

protected:
  int_type overflow(int_type c) override
  {
    if (traits_type::eq_int_type(c, traits_type::eof()))
      return traits_type::not_eof(c);
    const auto ch = traits_type::to_char_type(c);
    if (ch == '\n')
      emit(true);
    else {
      try {
        mText += ch;
      } catch (const std::exception&) {
        // Drop the failed line without poisoning the ostream with badbit.
        mText.clear();
        mPrefixLength = 0;
      }
    }
    return c;
  }

  int sync() override
  {
    if (!mText.empty())
      emit(false);
    return 0;
  }

private:
  void emit(bool newline) noexcept
  {
    const bool wasEmitting = insideConsoleEmit;
    insideConsoleEmit = true;
    try {
      auto text = std::move(mText);
      mText.clear();
      const auto prefix = mPrefixLength;
      mPrefixLength = 0;
      detail::logConsoleRecord(mLevel, text, prefix, mColor, newline);
    } catch (...) {
      mText.clear();
      mPrefixLength = 0;
    }
    insideConsoleEmit = wasEmitting;
  }

  LogLevel mLevel;
  std::string mText;
  std::size_t mPrefixLength = 0;
  int mColor = 0;
};

//==============================================================================
std::ostream& consoleStream(LogLevel level)
{
  auto& fallback = level >= LogLevel::Warn ? std::cerr : std::cout;
  thread_local bool cleaned = false;
  if (insideConsoleEmit || cleaned)
    return fallback;
  struct Stream
  {
    ConsoleBuffer buffer;
    std::ostream stream;

    explicit Stream(LogLevel severity) : buffer(severity), stream(&buffer) {}
  };
  struct Cleanup
  {
    Stream** streams;
    bool& cleaned;

    ~Cleanup()
    {
      cleaned = true;
      for (int i = 0; i < 4; ++i) {
        if (streams[i]) {
          try {
            streams[i]->stream.clear();
            streams[i]->stream.flush();
          } catch (...) {
          }
        }
      }
      for (int i = 0; i < 4; ++i) {
        delete streams[i];
        streams[i] = nullptr;
      }
    }
  };
  thread_local Stream* streams[4]{};
  thread_local bool hasStreams = false;
  if (!hasStreams && !detail::hasCustomLogSink())
    return fallback;
  for (auto* stream : streams) {
    if (stream) {
      try {
        stream->stream.clear();
        stream->stream.flush();
      } catch (...) {
      }
    }
  }
  if (!detail::hasCustomLogSink())
    return fallback;

  const auto index = level == LogLevel::Debug   ? 0
                     : level == LogLevel::Warn  ? 2
                     : level == LogLevel::Error ? 3
                                                : 1;
  auto& stream = streams[index];
  if (!stream) {
    try {
      stream = new Stream(level);
      thread_local Cleanup cleanup{streams, cleaned};
      hasStreams = true;
    } catch (const std::exception&) {
      return fallback;
    }
  }
  stream->stream.clear();
  return stream->stream;
}

} // namespace

//==============================================================================
std::ostream& colorMsg(const std::string& _msg, int _color)
{
  auto& stream = consoleStream(_color == 36 ? LogLevel::Debug : LogLevel::Info);
  if (&stream == &std::cout)
    stream << "\033[1;" << _color << "m" << _msg << "\033[0m ";
  else {
    stream << _msg;
    static_cast<ConsoleBuffer*>(stream.rdbuf())->markPrefix(_color);
    stream << ' ';
  }
  return stream;
}

//==============================================================================
std::ostream& colorErr(
    const std::string& _msg,
    const std::string& _file,
    unsigned int _line,
    int _color)
{
  const auto index = _file.find_last_of("/") + 1;
  auto& stream = consoleStream(_color == 31 ? LogLevel::Error : LogLevel::Warn);
  if (&stream == &std::cerr)
    stream << "\033[1;" << _color << "m";
  stream << _msg << " [" << _file.substr(index) << ":" << _line << "]";
  if (&stream == &std::cerr)
    stream << "\033[0m";
  else
    static_cast<ConsoleBuffer*>(stream.rdbuf())->markPrefix(_color);
  stream << ' ';
  return stream;
}

} // namespace common
} // namespace dart
