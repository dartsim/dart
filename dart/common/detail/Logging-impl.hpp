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

#ifndef DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
#define DART_COMMON_DETAIL_LOGGING_IMPL_HPP_

#include <dart/common/Logging.hpp>

#include <array>
#include <memory>
#include <type_traits>
#include <utility>

#include <cstddef>

#if defined(__GLIBCXX__)
  #include <cxxabi.h>
#endif

namespace dart {
namespace common {
namespace detail {

/// Forward a record to the console without reentering the installed sink.
void defaultLogSink(LogLevel level, const std::string& message);

// Capture the previous sink and publish its wrapper under the logging mutex.
// The wrapper callback must be non-empty; an empty result resets to default.
std::shared_ptr<LogSink> wrapLogSink(
    const std::function<LogSink(std::shared_ptr<LogSink>)>& wrap);
void restoreLogSinkIf(
    std::shared_ptr<LogSink> installed, std::shared_ptr<LogSink> previous);

// Scoped GUI observation after callback consumption; registration is
// owner-thread only. Weak payloads never retain captures or their previous
// output sinks.
struct LogObserver;
LogObserver* registerLogObserver(const std::shared_ptr<LogSink>& observer);
void unregisterLogObserver(LogObserver* observer);

// Legacy console streams bypass the formatter and runtime threshold.
bool hasCustomLogSink();
void logRecord(LogLevel level, const std::string& message);
void logConsoleRecord(
    LogLevel level,
    const std::string& message,
    std::size_t prefixLength,
    int color,
    bool newline) noexcept;

// The Windows console API is injected so the color decision is testable
// without changing the process console or requiring a Windows host.
template <typename GetMode, typename SetMode>
bool enableVirtualTerminal(GetMode getMode, SetMode setMode)
{
  constexpr unsigned long virtualTerminalProcessing = 0x0004;
  unsigned long mode = 0;
  return getMode(mode)
         && ((mode & virtualTerminalProcessing)
             || setMode(mode | virtualTerminalProcessing));
}

// Keep formatter types out of installed headers and preserve scalar types until
// formatting in the library. This boundary must also compile as C++14.
struct LogArgument
{
  enum class Type
  {
    String,
    Signed,
    Unsigned,
    Float,
    Double,
    LongDouble,
    Bool,
    Char,
    Pointer
  };

  Type type;
  union
  {
    long long signedValue;
    unsigned long long unsignedValue;
    float floatValue;
    double doubleValue;
    long double longDoubleValue;
    bool boolValue;
    char charValue;
    const void* pointerValue;
  };
  std::string stringValue;

  LogArgument(const std::string& value);
  LogArgument(std::string&& value);
  LogArgument(const char* value);
  LogArgument(bool value);
  LogArgument(char value);
  LogArgument(float value);
  LogArgument(double value);
  LogArgument(long double value);
  LogArgument(const void* value);
  LogArgument(std::nullptr_t);

  // Mutable char arrays use this overload; string literals use const char*.
  template <std::size_t N>
  LogArgument(const char (&value)[N])
    : LogArgument(static_cast<const char*>(value))
  {
  }

  template <
      typename T,
      typename std::enable_if<
          !std::is_scalar<T>::value
              && std::is_constructible<std::string, const T&>::value,
          int>::type
      = 0>
  LogArgument(const T& value) : LogArgument(std::string(value))
  {
  }

  template <
      typename T,
      typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
  LogArgument(T value)
    // Promote char and bool enum bases so they format as integers.
    : LogArgument(+static_cast<typename std::underlying_type<T>::type>(value))
  {
  }

  template <
      typename T,
      typename std::enable_if<std::is_function<T>::value, int>::type = 0>
  LogArgument(T*) = delete;

  template <typename T, typename C>
  LogArgument(T C::*) = delete;

  // volatile pointers would otherwise convert to bool. The address is only
  // stored, never dereferenced.
  template <
      typename T,
      typename std::enable_if<
          !std::is_function<T>::value && std::is_volatile<T>::value,
          int>::type
      = 0>
  LogArgument(T* value)
    : LogArgument(static_cast<const void*>(
        const_cast<typename std::remove_cv<T>::type*>(value)))
  {
  }

  // Only scalar, string-like and enum arguments cross this lean boundary.
  template <
      typename UnsupportedLogArgument,
      typename std::enable_if<
          !std::is_scalar<UnsupportedLogArgument>::value
              && !std::is_constructible<
                  std::string,
                  const UnsupportedLogArgument&>::value,
          int>::type
      = 0>
  LogArgument(const UnsupportedLogArgument&) = delete;

  template <
      typename T,
      typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  LogArgument(T value)
    : type(std::is_signed<T>::value ? Type::Signed : Type::Unsigned)
  {
    if (std::is_signed<T>::value)
      signedValue = static_cast<long long>(value);
    else
      unsignedValue = static_cast<unsigned long long>(value);
  }
};

void logMessage(
    LogLevel level,
    const std::string& format,
    const LogArgument* arguments,
    std::size_t count);

//==============================================================================
template <typename S>
void log(LogLevel level, const S& format)
{
  if (level < getLogLevel() && !hasLogCallback())
    return;

  try {
    logMessage(level, std::string(format), nullptr, 0);
#if defined(__GLIBCXX__)
  } catch (const abi::__forced_unwind&) {
    throw;
#endif
  } catch (...) {
    // Logging must not interrupt application error handling.
  }
}

//==============================================================================
template <typename S, typename... Args>
void log(LogLevel level, const S& format, Args&&... args)
{
  // In particular, disabled logging must not allocate or initialize the sink.
  if (level < getLogLevel() && !hasLogCallback())
    return;

  try {
    const std::array<LogArgument, sizeof...(Args)> arguments{
        {LogArgument(std::forward<Args>(args))...}};
    logMessage(level, std::string(format), arguments.data(), arguments.size());
#if defined(__GLIBCXX__)
  } catch (const abi::__forced_unwind&) {
    throw;
#endif
  } catch (...) {
    // Logging must not interrupt application error handling.
  }
}

} // namespace detail

//==============================================================================
template <typename S, typename... Args>
void trace(const S& format_str, Args&&... args)
{
  detail::log(LogLevel::Trace, format_str, std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void debug(const S& format_str, Args&&... args)
{
  detail::log(LogLevel::Debug, format_str, std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void info(const S& format_str, Args&&... args)
{
  detail::log(LogLevel::Info, format_str, std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void warn(const S& format_str, Args&&... args)
{
  detail::log(LogLevel::Warn, format_str, std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void error(const S& format_str, Args&&... args)
{
  detail::log(LogLevel::Error, format_str, std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, Args&&... args)
{
  detail::log(LogLevel::Fatal, format_str, std::forward<Args>(args)...);
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
