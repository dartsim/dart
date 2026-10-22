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

#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

#include <cassert>

namespace dart::gui::app {

namespace {

// Both stream buffers share recursion state only within the current thread.
thread_local bool inLogCaptureSink = false;

struct CaptureScope
{
  bool previous = inLogCaptureSink;
  CaptureScope()
  {
    inLogCaptureSink = true;
  }
  ~CaptureScope()
  {
    inLogCaptureSink = previous;
  }
};

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
/// Recover severity for other code that writes prefixes or ANSI colors to
/// stdout/stderr. DART's own sink supplies its severity directly.
CapturedLogLevel sniffSeverity(
    const std::string& raw, const std::string& text, CapturedLogLevel fallback)
{
  if (text.compare(0, 7, "[error]") == 0 || text.compare(0, 7, "[fatal]") == 0)
    return CapturedLogLevel::Error;
  if (text.compare(0, 6, "[warn]") == 0)
    return CapturedLogLevel::Warning;
  if (text.compare(0, 7, "[trace]") == 0 || text.compare(0, 7, "[debug]") == 0
      || text.compare(0, 6, "[info]") == 0)
    return CapturedLogLevel::Info;

  const std::size_t pos = raw.find("\033[1;");
  if (pos == std::string::npos || pos + 6 > raw.size())
    return fallback;
  const std::string code = raw.substr(pos + 4, 2);
  if (code == "31" || code == "35")
    return CapturedLogLevel::Error;
  if (code == "33")
    return CapturedLogLevel::Warning;
  if (code == "32" || code == "36" || code == "37")
    return CapturedLogLevel::Info;
  return fallback;
}

} // namespace

//==============================================================================
struct LogCapture::SinkState
{
  std::shared_ptr<common::LogSink> previous;
  std::shared_ptr<LineSink> panel;
  std::mutex mutex;
  std::deque<std::pair<CapturedLogLevel, std::string>> records;
  bool alive = true;
  const std::thread::id owner = std::this_thread::get_id();
  std::shared_ptr<common::LogSink> observer;
  common::detail::LogObserver* token = nullptr;

  void enqueue(CapturedLogLevel level, const std::string& message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!alive)
      return;
    // Match the App log's bound while the GUI owner is between frames.
    constexpr std::size_t capacity = 2000;
    if (records.size() == capacity)
      records.pop_front();
    records.emplace_back(level, stripAnsi(message));
  }
};

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
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (traits_type::eq_int_type(c, traits_type::eof()))
      return traits_type::not_eof(c);
    if (mOriginal)
      mOriginal->sputc(traits_type::to_char_type(c));

    // Writes the sink makes are forwarded but never captured again.
    if (inLogCaptureSink)
      return c;

    const char ch = traits_type::to_char_type(c);
    if (ch == '\n')
      flushLine();
    else
      mLineBuf.push_back(ch);
    return c;
  }

  std::streamsize xsputn(const char* s, std::streamsize n) override
  {
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mOriginal)
      mOriginal->sputn(s, n);
    if (inLogCaptureSink)
      return n;
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

    CaptureScope scope;
    try {
      if (mSink) {
        const auto text = stripAnsi(mLineBuf);
        const CapturedLogLevel level
            = sniffSeverity(mLineBuf, text, mFallbackLevel);
        mSink(level, text);
      }
#if defined(__GLIBCXX__)
    } catch (const abi::__forced_unwind&) {
      throw;
#endif
    } catch (...) {
      // The log console must never crash the app, even if the sink throws.
    }
    mLineBuf.clear();
  }

  std::recursive_mutex mMutex;
  std::streambuf* mOriginal;
  CapturedLogLevel mFallbackLevel;
  LineSink mSink;
  std::string mLineBuf;
};

//==============================================================================
LogCapture::LogCapture(LineSink sink)
{
  mSinkState = std::make_shared<SinkState>();
  mSinkState->panel = std::make_shared<LineSink>(std::move(sink));
  auto enqueue
      = [state = mSinkState](CapturedLogLevel level, const std::string& text) {
          state->enqueue(level, text);
        };
  mSinkState->observer = std::make_shared<common::LogSink>(
      [state = std::weak_ptr<SinkState>(mSinkState)](
          common::LogLevel level, const std::string& message) {
        if (inLogCaptureSink)
          return;
        if (auto current = state.lock()) {
          const auto captured
              = level >= common::LogLevel::Error  ? CapturedLogLevel::Error
                : level == common::LogLevel::Warn ? CapturedLogLevel::Warning
                                                  : CapturedLogLevel::Info;
          current->enqueue(captured, message);
        }
      });
  mCoutBuf = std::make_unique<TeeBuf>(
      std::cout.rdbuf(), CapturedLogLevel::Info, enqueue);
  mCerrBuf = std::make_unique<TeeBuf>(
      std::cerr.rdbuf(), CapturedLogLevel::Warning, enqueue);
  mOldCoutBuf = std::cout.rdbuf(mCoutBuf.get());
  mOldCerrBuf = std::cerr.rdbuf(mCerrBuf.get());
  try {
    mInstalledSink = common::detail::wrapLogSink(
        [state = mSinkState](std::shared_ptr<common::LogSink> previous) {
          state->previous = std::move(previous);
          return [state](common::LogLevel level, const std::string& message) {
            const bool wasCapturing = inLogCaptureSink;
            {
              CaptureScope scope;
              try {
                if (state->previous)
                  (*state->previous)(level, message);
                else
                  common::detail::defaultLogSink(level, message);
#if defined(__GLIBCXX__)
              } catch (const abi::__forced_unwind&) {
                throw;
#endif
              } catch (...) {
                // A failing chained sink must not hide the panel record.
              }
            }
            if (!wasCapturing)
              (*state->observer)(level, message);
          };
        });
    mSinkState->token
        = common::detail::registerLogObserver(mSinkState->observer);
  } catch (...) {
    common::detail::restoreLogSinkIf(mInstalledSink, mSinkState->previous);
    std::cout.rdbuf(mOldCoutBuf);
    std::cerr.rdbuf(mOldCerrBuf);
    throw;
  }
}

//==============================================================================
void LogCapture::drain()
{
  // Local ownership also permits a panel callback to destroy this capture.
  auto state = mSinkState;
  assert(state->owner == std::this_thread::get_id());
  if (inLogCaptureSink)
    return;
  std::deque<std::pair<CapturedLogLevel, std::string>> records;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    records.swap(state->records);
  }
  CaptureScope scope;
  for (const auto& record : records) {
    std::shared_ptr<LineSink> panel;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->alive)
        break;
      panel = state->panel;
    }
    try {
      if (panel && *panel)
        (*panel)(record.first, record.second);
#if defined(__GLIBCXX__)
    } catch (const abi::__forced_unwind&) {
      throw;
#endif
    } catch (...) {
      // One failed panel notification must not lose the following records.
    }
  }
}

//==============================================================================
LogCapture::~LogCapture()
{
  assert(mSinkState->owner == std::this_thread::get_id());
  std::shared_ptr<LineSink> panel;
  {
    std::lock_guard<std::mutex> lock(mSinkState->mutex);
    mSinkState->alive = false;
    panel = std::move(mSinkState->panel);
    mSinkState->records.clear();
  }
  common::detail::unregisterLogObserver(mSinkState->token);
  try {
    common::detail::restoreLogSinkIf(mInstalledSink, mSinkState->previous);
  } catch (...) {
  }
  std::cout.rdbuf(mOldCoutBuf);
  std::cerr.rdbuf(mOldCerrBuf);
}

} // namespace dart::gui::app
