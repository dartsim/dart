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

#include <dart/common/Logging.hpp>

#include <fmt/args.h>
#include <fmt/format.h>

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
  #include <io.h>
#else
  #include <unistd.h>
#endif

namespace dart::common {
namespace {

std::atomic<LogLevel> loggingLevel{
    DART_ACTIVE_LOG_LEVEL < DART_LOG_LEVEL_INFO
        ? LogLevel::Info
        : static_cast<LogLevel>(DART_ACTIVE_LOG_LEVEL)};

// Flush at exit; stream objects survive Init destruction for late logging.
std::ios_base::Init streams;

struct LoggingState
{
  std::recursive_mutex mutex;
  std::shared_ptr<LogSink> sink;
};

//==============================================================================
LoggingState& loggingState()
{
  // Process lifetime: logging is also used by static destructors.
  static auto* state = new LoggingState;
  return *state;
}

// Sinks currently executing on this thread, innermost first.
struct ActiveSink
{
  const LogSink* sink;
  const ActiveSink* outer;
};

thread_local const ActiveSink* activeSinks = nullptr;

struct ActiveSinkScope : ActiveSink
{
  explicit ActiveSinkScope(const LogSink* s) : ActiveSink{s, activeSinks}
  {
    activeSinks = this;
  }

  ~ActiveSinkScope()
  {
    activeSinks = outer;
  }
};

bool isSinkActive(const LogSink* sink)
{
  for (auto* active = activeSinks; active; active = active->outer) {
    if (active->sink == sink)
      return true;
  }
  return false;
}

//==============================================================================
bool useLogColor(std::FILE* stream)
{
  const auto* term = std::getenv("TERM");
  if (std::getenv("NO_COLOR") || (term && std::strcmp(term, "dumb") == 0))
    return false;
#ifdef _WIN32
  return _isatty(_fileno(stream)) != 0;
#else
  return isatty(fileno(stream)) != 0;
#endif
}

//==============================================================================
void defaultLogSink(LogLevel level, const std::string& message)
{
  static std::ios_base::Init init;
  if (level < LogLevel::Trace || level >= LogLevel::Off)
    return;

  static const char* const names[]{
      "trace", "debug", "info", "warn", "error", "fatal"};
  static const int colors[]{37, 36, 32, 33, 31, 35};
  static const bool colorStdout = useLogColor(stdout);
  static const bool colorStderr = useLogColor(stderr);
  const auto index = static_cast<unsigned>(level);
  auto& stream = level >= LogLevel::Warn ? std::cerr : std::cout;
  const bool color = level >= LogLevel::Warn ? colorStderr : colorStdout;
  if (color)
    stream << "\033[1;" << colors[index] << 'm';
  stream << '[' << names[index] << ']';
  if (color)
    stream << "\033[0m";
  stream << ' ' << message << std::endl;
}

} // namespace

//==============================================================================
LogSink setLogSink(LogSink sink)
{
  auto replacement
      = sink ? std::make_shared<LogSink>(std::move(sink)) : nullptr;
  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  auto previous = state.sink ? *state.sink : LogSink{};
  state.sink.swap(replacement);
  return previous;
}

//==============================================================================
void resetLogSink()
{
  setLogSink({});
}

//==============================================================================
void setLogLevel(LogLevel level)
{
  loggingLevel.store(level, std::memory_order_relaxed);
}

//==============================================================================
LogLevel getLogLevel()
{
  return loggingLevel.load(std::memory_order_relaxed);
}

namespace detail {

//==============================================================================
LogArgument::LogArgument(const std::string& value)
  : type(Type::String), pointerValue(nullptr), stringValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(const char* value)
  : type(Type::String),
    pointerValue(nullptr),
    stringValue(value ? value : "(null)")
{
}

//==============================================================================
LogArgument::LogArgument(bool value) : type(Type::Bool), boolValue(value) {}

//==============================================================================
LogArgument::LogArgument(char value) : type(Type::Char), charValue(value) {}

//==============================================================================
LogArgument::LogArgument(float value) : type(Type::Float), floatValue(value) {}

//==============================================================================
LogArgument::LogArgument(double value) : type(Type::Double), doubleValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(long double value)
  : type(Type::LongDouble), longDoubleValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(const void* value)
  : type(Type::Pointer), pointerValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(std::nullptr_t)
  : LogArgument(static_cast<const void*>(nullptr))
{
}

//==============================================================================
void logMessage(
    LogLevel level,
    const std::string& format,
    const LogArgument* arguments,
    std::size_t count)
{
  if (level < LogLevel::Trace || level >= LogLevel::Off
      || level < getLogLevel())
    return;

  fmt::dynamic_format_arg_store<fmt::format_context> store;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& argument = arguments[i];
    switch (argument.type) {
      case LogArgument::Type::String:
        store.push_back(argument.stringValue);
        break;
      case LogArgument::Type::Signed:
        store.push_back(argument.signedValue);
        break;
      case LogArgument::Type::Unsigned:
        store.push_back(argument.unsignedValue);
        break;
      case LogArgument::Type::Float:
        store.push_back(argument.floatValue);
        break;
      case LogArgument::Type::Double:
        store.push_back(argument.doubleValue);
        break;
      case LogArgument::Type::LongDouble:
        store.push_back(argument.longDoubleValue);
        break;
      case LogArgument::Type::Bool:
        store.push_back(argument.boolValue);
        break;
      case LogArgument::Type::Char:
        store.push_back(argument.charValue);
        break;
      case LogArgument::Type::Pointer:
        store.push_back(argument.pointerValue);
        break;
    }
  }

  std::string message;
  try {
    message = fmt::vformat(format, store);
  } catch (const fmt::format_error& error) {
    message = "[log format error: " + std::string(error.what()) + "] " + format;
  }

  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  // A callback may replace itself. Keep it alive until this invocation returns.
  auto sink = state.sink;
  try {
    // Re-entry into an already running sink goes to the default output.
    if (sink && !isSinkActive(sink.get())) {
      ActiveSinkScope scope(sink.get());
      (*sink)(level, message);
    } else
      defaultLogSink(level, message);
  } catch (...) {
    // Logging must not interrupt application error handling.
  }
}

} // namespace detail
} // namespace dart::common
