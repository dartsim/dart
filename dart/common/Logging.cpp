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

#include <algorithm>
#include <atomic>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
  #include <io.h>
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
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
  std::map<
      std::weak_ptr<LogSink>,
      std::weak_ptr<LogSink>,
      std::owner_less<std::weak_ptr<LogSink>>>
      retired;
  std::atomic<bool> hasSink{false};
};

struct ConsoleOrigin
{
  std::size_t prefixLength;
  int color;
  bool newline;
};

thread_local const ConsoleOrigin* consoleOrigin = nullptr;

//==============================================================================
LoggingState& loggingState()
{
  // Process lifetime: logging is also used by static destructors.
  static auto* state = new LoggingState;
  return *state;
}

//==============================================================================
bool useLogColor(std::FILE* stream)
{
  const auto* term = std::getenv("TERM");
  if (std::getenv("NO_COLOR") || (term && std::strcmp(term, "dumb") == 0))
    return false;
#ifdef _WIN32
  if (!_isatty(_fileno(stream)))
    return false;
  const auto handle = reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(stream)));
  return detail::enableVirtualTerminal(
      [handle](unsigned long& mode) {
        return GetConsoleMode(handle, &mode) != 0;
      },
      [handle](unsigned long mode) {
        return SetConsoleMode(handle, mode) != 0;
      });
#else
  return isatty(fileno(stream)) != 0;
#endif
}

//==============================================================================
void writeDefaultLog(LogLevel level, const std::string& message)
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
  if (consoleOrigin) {
    const auto prefix = consoleOrigin->prefixLength;
    if (prefix)
      stream << "\033[1;" << consoleOrigin->color << 'm';
    stream.write(message.data(), prefix);
    if (prefix)
      stream << "\033[0m";
    stream.write(message.data() + prefix, message.size() - prefix);
    if (consoleOrigin->newline)
      stream << '\n';
    stream.flush();
    return;
  }
  const bool color = level >= LogLevel::Warn ? colorStderr : colorStdout;
  if (color)
    stream << "\033[1;" << colors[index] << 'm';
  stream << '[' << names[index] << ']';
  if (color)
    stream << "\033[0m";
  stream << ' ' << message << std::endl;
}

//==============================================================================
void dispatchLog(
    LogLevel level,
    const std::string& message,
    const ConsoleOrigin* origin) noexcept
{
  const auto* previousOrigin = consoleOrigin;
  consoleOrigin = origin;
  try {
    auto& state = loggingState();
    std::lock_guard<std::recursive_mutex> lock(state.mutex);
    // A callback may replace itself. Keep it alive until it returns.
    auto sink = state.sink;
    // Prefix-only statements retain terminal bytes without an empty record.
    const bool prefixOnly = origin && !origin->newline && origin->prefixLength
                            && message.size() == origin->prefixLength + 1;
    if (sink && !prefixOnly)
      (*sink)(level, message);
    else
      writeDefaultLog(level, message);
  } catch (...) {
    // Logging must not interrupt application error handling or thread cleanup.
  }
  consoleOrigin = previousOrigin;
}

//==============================================================================
template <typename T>
void appendNumber(std::string& message, T value, int base = 10)
{
  char buffer[32];
  const auto result
      = std::to_chars(buffer, buffer + sizeof(buffer), value, base);
  if (result.ec != std::errc{})
    throw std::invalid_argument("number conversion failed");
  message.append(buffer, result.ptr);
}

//==============================================================================
template <typename T>
bool appendNonFinite(std::string& message, T value)
{
  if (std::isfinite(value))
    return false;
  if (std::signbit(value))
    message += '-';
  message += std::isnan(value) ? "nan" : "inf";
  return true;
}

//==============================================================================
template <typename T>
void appendFloat(
    std::string& message, T value, char presentation, int precision)
{
  if (appendNonFinite(message, value))
    return;

  if (!presentation) {
    char buffer[64];
    const auto result = std::to_chars(
        buffer, buffer + sizeof(buffer), value, std::chars_format::scientific);
    if (result.ec != std::errc{})
      throw std::invalid_argument("number conversion failed");
    std::string_view number(buffer, result.ptr - buffer);
    const auto exponentPosition = number.find('e');
    if (exponentPosition == std::string_view::npos) {
      message += number;
      return;
    }
    int exponent = 0;
    const char* digits = buffer + exponentPosition + 1;
    // from_chars does not accept a leading plus sign.
    if (digits != result.ptr && *digits == '+')
      ++digits;
    if (digits == result.ptr)
      throw std::invalid_argument("number conversion failed");
    const auto exponentResult = std::from_chars(digits, result.ptr, exponent);
    if (exponentResult.ec != std::errc{} || exponentResult.ptr != result.ptr)
      throw std::invalid_argument("number conversion failed");
    // Keep the shortest digits from to_chars, with fmt's default layout.
    if (exponent < -4
        || exponent >= std::min(16, std::numeric_limits<T>::digits10 + 1)) {
      message += number;
      return;
    }
    number = number.substr(0, exponentPosition);
    if (number.front() == '-') {
      message += '-';
      number.remove_prefix(1);
    }
    if (exponent < 0) {
      message += "0.";
      message.append(-exponent - 1, '0');
    }
    int written = 0;
    const int digitCount = static_cast<int>(number.size())
                           - (number.find('.') != std::string_view::npos);
    for (const char digit : number) {
      if (digit == '.')
        continue;
      message += digit;
      ++written;
      if (written == exponent + 1 && written < digitCount)
        message += '.';
    }
    if (written < exponent + 1)
      message.append(exponent + 1 - written, '0');
    return;
  }

  const auto style = presentation == 'e'   ? std::chars_format::scientific
                     : presentation == 'f' ? std::chars_format::fixed
                                           : std::chars_format::general;
  const auto convert = [&](char* begin, char* end) {
    return std::to_chars(begin, end, value, style, precision);
  };
  char buffer[128];
  auto result = convert(buffer, buffer + sizeof(buffer));
  if (result.ec == std::errc{}) {
    message.append(buffer, result.ptr);
    return;
  }
  if (result.ec != std::errc::value_too_large)
    throw std::invalid_argument("number conversion failed");

  std::string largeBuffer(sizeof(buffer) * 2, '\0');
  for (;;) {
    result
        = convert(largeBuffer.data(), largeBuffer.data() + largeBuffer.size());
    if (result.ec == std::errc{}) {
      message.append(largeBuffer.data(), result.ptr);
      return;
    }
    if (result.ec != std::errc::value_too_large || largeBuffer.size() >= 8192)
      throw std::invalid_argument("number conversion failed");
    largeBuffer.resize(largeBuffer.size() * 2);
  }
}

//==============================================================================
void appendArgument(
    std::string& message,
    const detail::LogArgument& argument,
    std::string_view field)
{
  using Type = detail::LogArgument::Type;
  char presentation = 0;
  int precision = 0;
  if (!field.empty()) {
    if (field.size() < 4 || field.substr(0, 2) != ":."
        || field.find_first_not_of("0123456789", 2) != field.size() - 1
        || (field.back() != 'e' && field.back() != 'f' && field.back() != 'g'))
      throw std::invalid_argument("unsupported format field");
    const auto result = std::from_chars(
        field.data() + 2, field.data() + field.size() - 1, precision);
    if (result.ec != std::errc{}
        || result.ptr != field.data() + field.size() - 1 || precision > 1000)
      throw std::invalid_argument("invalid precision");
    if (argument.type != Type::Float && argument.type != Type::Double
        && argument.type != Type::LongDouble)
      throw std::invalid_argument("precision requires floating-point argument");
    presentation = field.back();
    if (presentation == 'g' && precision == 0)
      precision = 1;
  }

  switch (argument.type) {
    case Type::String:
      message += argument.stringValue;
      break;
    case Type::Signed:
      appendNumber(message, argument.signedValue);
      break;
    case Type::Unsigned:
      appendNumber(message, argument.unsignedValue);
      break;
    case Type::Float:
      appendFloat(message, argument.floatValue, presentation, precision);
      break;
    case Type::Double:
      appendFloat(message, argument.doubleValue, presentation, precision);
      break;
    case Type::LongDouble: {
      if (appendNonFinite(message, argument.longDoubleValue))
        break;
      // Classic streams also support platforms without long double to_chars.
      // Default precision preserves round trips; layout may differ from fmt.
      std::ostringstream stream;
      stream.imbue(std::locale::classic());
      if (presentation == 'e')
        stream << std::scientific;
      else if (presentation == 'f')
        stream << std::fixed;
      stream << std::setprecision(
          presentation ? precision
                       : std::numeric_limits<long double>::max_digits10)
             << argument.longDoubleValue;
      message += stream.str();
      break;
    }
    case Type::Bool:
      message += argument.boolValue ? "true" : "false";
      break;
    case Type::Char:
      message += argument.charValue;
      break;
    case Type::Pointer:
      message += "0x";
      appendNumber(
          message, reinterpret_cast<std::uintptr_t>(argument.pointerValue), 16);
      break;
  }
}

//==============================================================================
std::string formatMessage(
    const std::string& format,
    const detail::LogArgument* arguments,
    std::size_t count)
{
  std::string message;
  message.reserve(format.size());
  std::size_t argument = 0;
  for (std::size_t i = 0; i < format.size(); ++i) {
    const char character = format[i];
    if (character != '{' && character != '}') {
      message += character;
      continue;
    }
    if (i + 1 < format.size() && format[i + 1] == character) {
      message += character;
      ++i;
      continue;
    }
    if (character == '}')
      throw std::invalid_argument("unmatched '}'");
    const auto end = format.find('}', i + 1);
    if (end == std::string::npos)
      throw std::invalid_argument("unmatched '{'");
    if (argument == count)
      throw std::invalid_argument("too few arguments");
    appendArgument(
        message,
        arguments[argument++],
        std::string_view(format.data() + i + 1, end - i - 1));
    i = end;
  }
  return message;
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
  state.retired.clear();
  state.hasSink.store(static_cast<bool>(state.sink), std::memory_order_relaxed);
  return previous;
}

//==============================================================================
void resetLogSink()
{
  std::shared_ptr<LogSink> previous;
  auto& state = loggingState();
  {
    std::lock_guard<std::recursive_mutex> lock(state.mutex);
    previous.swap(state.sink);
    state.retired.clear();
    state.hasSink.store(false, std::memory_order_relaxed);
  }
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
std::shared_ptr<LogSink> wrapLogSink(
    const std::function<LogSink(std::shared_ptr<LogSink>)>& wrap)
{
  std::shared_ptr<LogSink> replacement;
  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  auto sink = wrap(state.sink);
  if (sink)
    replacement = std::make_shared<LogSink>(std::move(sink));
  state.sink.swap(replacement);
  state.hasSink.store(static_cast<bool>(state.sink), std::memory_order_relaxed);
  return state.sink;
}

//==============================================================================
void restoreLogSinkIf(
    std::shared_ptr<LogSink> installed, std::shared_ptr<LogSink> previous)
{
  if (!installed)
    return;
  std::vector<std::shared_ptr<LogSink>> removed;
  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  for (auto it = state.retired.begin(); it != state.retired.end();) {
    if (it->first.expired())
      it = state.retired.erase(it);
    else
      ++it;
  }
  if (state.sink == installed) {
    state.sink.swap(previous);
    while (state.sink) {
      const auto retired = state.retired.find(state.sink);
      if (retired == state.retired.end())
        break;
      // Keep the wrapper (and its predecessor) alive until after unlocking.
      removed.push_back(std::move(state.sink));
      state.sink = retired->second.lock();
      state.retired.erase(retired);
    }
    state.hasSink.store(
        static_cast<bool>(state.sink), std::memory_order_relaxed);
  } else {
    state.retired[installed] = previous;
  }
}

//==============================================================================
bool hasCustomLogSink()
{
  return loggingState().hasSink.load(std::memory_order_relaxed);
}

//==============================================================================
void logRecord(LogLevel level, const std::string& message) noexcept
{
  dispatchLog(level, message, nullptr);
}

//==============================================================================
void logConsoleRecord(
    LogLevel level,
    const std::string& message,
    std::size_t prefixLength,
    int color,
    bool newline) noexcept
{
  const ConsoleOrigin origin{prefixLength, color, newline};
  dispatchLog(level, message, &origin);
}

//==============================================================================
void defaultLogSink(LogLevel level, const std::string& message)
{
  writeDefaultLog(level, message);
}

//==============================================================================
LogArgument::LogArgument(const std::string& value)
  : type(Type::String), pointerValue(nullptr), stringValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(std::string&& value)
  : type(Type::String), pointerValue(nullptr), stringValue(std::move(value))
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

  std::string message;
  try {
    message = formatMessage(format, arguments, count);
  } catch (const std::invalid_argument& error) {
    message = "[log format error: " + std::string(error.what()) + "] " + format;
  }

  logRecord(level, message);
}

} // namespace detail
} // namespace dart::common
