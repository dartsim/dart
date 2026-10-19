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

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>

#if DART_HAVE_spdlog
  #include <fmt/format.h>
  #include <spdlog/spdlog.h>

  #include <exception>
  #include <ostream>
  #include <string>
#else
  #include <sstream>
  #include <string>
#endif

namespace dart::common {

#if DART_HAVE_spdlog
namespace detail {

//==============================================================================
inline spdlog::level::level_enum toSpdlogLevel(LogLevel level)
{
  switch (level) {
    case LogLevel::Trace:
      return spdlog::level::trace;
    case LogLevel::Debug:
      return spdlog::level::debug;
    case LogLevel::Info:
      return spdlog::level::info;
    case LogLevel::Warn:
      return spdlog::level::warn;
    case LogLevel::Error:
      return spdlog::level::err;
    default:
      return spdlog::level::critical;
  }
}

//==============================================================================
/// Formats a runtime-formatted message without handing spdlog the format
/// string. spdlog's variadic log() funnels the format string through
/// spdlog::details::to_string_view(), whose implicit fmt::format_string ->
/// string_view conversion was deprecated in fmt 12.2.0 ("use
/// format_string::get() instead"). Because DART's logging templates are
/// header-only and instantiated by downstreams, that deprecation warning
/// surfaces in every consumer built against fmt >= 12.2.0 (e.g. gz-physics,
/// see https://github.com/gazebosim/gz-physics/issues/1018). Pre-formatting
/// with fmt and handing spdlog a ready-made string sidesteps the deprecated
/// conversion entirely.
///
/// A logging call must never throw, so a malformed format string (only caught
/// at runtime, since the format is not a compile-time constant) falls back to
/// the raw format string instead of propagating the fmt error. fmt::vformat
/// takes a runtime format string directly, so it works across every fmt
/// version DART supports (unlike fmt::runtime, which is fmt 8+ only) and
/// independent of spdlog's formatting backend.
template <typename S, typename... Args>
std::string formatMessage(const S& format_str, Args&&... args)
{
  try {
    return fmt::vformat(format_str, fmt::make_format_args(args...));
  } catch (const std::exception& e) {
    return fmt::format("[log format error: {}] {}", e.what(), format_str);
  }
}

//==============================================================================
template <typename S, typename... Args>
void logToSpdlog(LogLevel level, const S& format_str, Args&&... args)
{
  const spdlog::level::level_enum spdlogLevel = toSpdlogLevel(level);

  // Skip all work for a message that spdlog would discard, unless a callback
  // from setLogCallback() wants it: the callback receives every message
  // regardless of spdlog's runtime level. should_backtrace() preserves the
  // behaviour of the prior spdlog::*() calls, which still recorded
  // below-level messages into spdlog's backtrace ring when one is enabled.
  // spdlog::drop_all() leaves no default logger; such a message goes on.
  if (!hasLogCallback()) {
    auto* const logger = spdlog::default_logger_raw();
    if (logger && !logger->should_log(spdlogLevel)
        && !logger->should_backtrace()) {
      return;
    }
  }

  // Formatted once, then delivered in order: to the callback, which consults
  // no spdlog registry and takes no spdlog lock, so it reaches the same
  // callback from every library that instantiates it and works after
  // spdlog::drop_all(); when the callback declines the message (it was
  // removed meanwhile, this thread is inside it, or it threw), to spdlog's
  // default logger, which applies its own runtime level; and with no default
  // logger, to the console streams.
  const std::string message
      = formatMessage(format_str, std::forward<Args>(args)...);
  if (forwardToLogCallback(level, message.data(), message.size()))
    return;
  if (auto* const logger = spdlog::default_logger_raw())
    logger->log(spdlogLevel, message);
  else
    (level < LogLevel::Warn ? consoleOut() : consoleErr()) << message << '\n';
}

} // namespace detail
#endif

#if !DART_HAVE_spdlog
namespace detail {

//==============================================================================
template <typename S>
void printMessage(std::ostream& os, const S& format_str)
{
  os << format_str;
}

//==============================================================================
template <typename S, typename Arg, typename... Args>
void printMessage(
    std::ostream& os, const S& format_str, Arg&& arg, Args&&... args)
{
  os << format_str << " [args]: " << std::forward<Arg>(arg);
  ((os << ", " << std::forward<Args>(args)), ...);
}

//==============================================================================
template <typename S, typename... Args>
void print(
    LogLevel level,
    ConsoleStream stream,
    const char* header,
    int color,
    const S& format_str,
    Args&&... args)
{
  // Formatted once, into a fresh stream so that neither a manipulator among
  // the arguments nor an operator<< that logs can leak into another message,
  // then delivered to a callback from setLogCallback() or, when it declines
  // the message (it was removed meanwhile, this thread is inside it, or it
  // threw), to the console stream.
  std::ostringstream text;
  printMessage(text, format_str, std::forward<Args>(args)...);
  const std::string message = text.str();
  if (forwardToLogCallback(level, message.data(), message.size()))
    return;
  stream() << "\033[1;" << color << "m" << header << "\033[0m " << message
           << std::endl;
}

} // namespace detail
#endif

//==============================================================================
template <typename S, typename... Args>
void trace(const S& format_str, Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(LogLevel::Trace, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      LogLevel::Trace,
      &consoleOut,
      "[trace]",
      38,
      format_str,
      std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void debug(const S& format_str, Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(LogLevel::Debug, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      LogLevel::Debug,
      &consoleOut,
      "[debug]",
      36,
      format_str,
      std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void info(const S& format_str, Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(LogLevel::Info, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      LogLevel::Info,
      &consoleOut,
      "[info]",
      32,
      format_str,
      std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void warn(const S& format_str, Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(LogLevel::Warn, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      LogLevel::Warn,
      &consoleErr,
      "[warn]",
      33,
      format_str,
      std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void error(const S& format_str, Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(LogLevel::Error, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      LogLevel::Error,
      &consoleErr,
      "[error]",
      31,
      format_str,
      std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(LogLevel::Fatal, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      LogLevel::Fatal,
      &consoleErr,
      "[fatal]",
      35,
      format_str,
      std::forward<Args>(args)...);
#endif
}

} // namespace dart::common

#endif // DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
