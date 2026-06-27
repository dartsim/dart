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

#include <dart/common/logging.hpp>

#include <fmt/format.h>

#if DART_HAVE_spdlog
  #include <spdlog/spdlog.h>

  #include <concepts>
  #include <type_traits>
  #include <utility>
  // These fmt helpers back normalize() below and are needed for the fmt
  // backend, which is the configuration DART builds against. spdlog's
  // std::format backend is not currently supported here (normalize() is
  // referenced unconditionally at the call site below).
  #if !defined(SPDLOG_USE_STD_FORMAT)
    #include <fmt/ostream.h>
    #include <spdlog/fmt/fmt.h>
    #include <spdlog/fmt/ostr.h>

    #include <ostream>

namespace detail_log_arg {

template <typename Stream, typename T>
concept StreamInsertable
    = requires(Stream& stream, const T& value) { stream << value; };

template <typename T>
concept PointerLikeGet = requires(T& value) {
  value.get();
  requires std::is_pointer_v<std::remove_reference_t<decltype(value.get())>>;
};

template <typename T>
auto normalize(T&& arg)
{
  using Decayed = std::decay_t<T>;
  if constexpr (std::is_pointer_v<Decayed>) {
    using Pointee = std::remove_cv_t<std::remove_pointer_t<Decayed>>;
    if constexpr (
        !std::same_as<Pointee, char> && !std::same_as<Pointee, signed char>
        && !std::same_as<Pointee, unsigned char>) {
      return fmt::ptr(arg);
    } else {
      return std::forward<T>(arg);
    }
  } else if constexpr (PointerLikeGet<Decayed>) {
    using RawPointer
        = std::remove_reference_t<decltype(std::declval<Decayed&>().get())>;
    using Pointee = std::remove_cv_t<std::remove_pointer_t<RawPointer>>;
    if constexpr (
        !std::same_as<Pointee, char> && !std::same_as<Pointee, signed char>
        && !std::same_as<Pointee, unsigned char>) {
      return fmt::ptr(arg.get());
    } else {
      return std::forward<T>(arg);
    }
  } else {
    using Char = typename fmt::format_context::char_type;
    constexpr bool kHasFormatter = fmt::detail::has_formatter<Decayed, Char>();
    if constexpr (std::is_enum_v<Decayed> && !kHasFormatter) {
      return static_cast<std::underlying_type_t<Decayed>>(arg);
    } else if constexpr (kHasFormatter) {
      return std::forward<T>(arg);
    } else if constexpr (StreamInsertable<std::ostream, Decayed>) {
      return fmt::streamed(std::forward<T>(arg));
    } else {
      return std::forward<T>(arg);
    }
  }
}

} // namespace detail_log_arg

  #endif
#else
  #include <iostream>
  #include <string>
#endif

#include <filesystem>
#include <string>

namespace dart::common {

#if !DART_HAVE_spdlog
namespace detail {

//==============================================================================
template <typename S1, typename S2>
void print(std::ostream& os, const S1& header, const S2& format_str, int color)
{
  os << "\033[1;" << color << "m" << header << "\033[0m " << format_str
     << std::endl;
}

//==============================================================================
template <typename S, typename Arg, typename... Args>
void print(
    std::ostream& os,
    const S& header,
    std::string format_str,
    int color,
    Arg&& arg,
    Args&&... args)
{
  os << "\033[1;" << color << "m" << header << "\033[0m " << format_str
     << " [args]: ";
  os << std::forward<Arg>(arg);
  ((os << ", " << std::forward<Args>(args)), ...);
  os << std::endl;
}

} // namespace detail
#endif

namespace detail {

inline std::string makeLogPrefix(
    const char* file, int line, const char* function)
{
#ifndef NDEBUG
  std::string prefix;
  bool has_component = false;
  if (file != nullptr && *file != '\0') {
    prefix += std::filesystem::path(file).filename().string();
    has_component = true;
  }
  if (line > 0) {
    if (has_component) {
      prefix += ':';
    }
    prefix += std::to_string(line);
    has_component = true;
  }
  if (function != nullptr && *function != '\0') {
    if (has_component) {
      prefix += "::";
    }
    prefix += function;
    has_component = true;
  }
  if (has_component) {
    prefix.insert(prefix.begin(), '[');
    prefix.append("] ");
    return prefix;
  }
#else
  (void)file;
  (void)line;
  (void)function;
#endif

  return {};
}

inline std::string makeLogPrefix(const std::source_location& location)
{
  return makeLogPrefix(
      location.file_name(),
      static_cast<int>(location.line()),
      location.function_name());
}

#if DART_HAVE_spdlog
inline constexpr spdlog::level::level_enum toSpdlogLevel(LogLevel level)
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
    case LogLevel::Fatal:
      return spdlog::level::critical;
  }
  return spdlog::level::info;
}
#endif

inline constexpr int toAnsiColor(LogLevel level)
{
  switch (level) {
    case LogLevel::Trace:
      return 38;
    case LogLevel::Debug:
      return 36;
    case LogLevel::Info:
      return 32;
    case LogLevel::Warn:
      return 33;
    case LogLevel::Error:
      return 31;
    case LogLevel::Fatal:
      return 35;
  }
  return 37;
}

inline constexpr const char* toLabel(LogLevel level)
{
  switch (level) {
    case LogLevel::Trace:
      return "[trace]";
    case LogLevel::Debug:
      return "[debug]";
    case LogLevel::Info:
      return "[info]";
    case LogLevel::Warn:
      return "[warn]";
    case LogLevel::Error:
      return "[error]";
    case LogLevel::Fatal:
      return "[fatal]";
  }
  return "[info]";
}

template <typename S, typename... Args>
void log(
    LogLevel level,
    const char* file,
    int line,
    const char* function,
    const S& format_str,
    Args&&... args)
{
#if DART_HAVE_spdlog
  // Hand spdlog a ready-made string rather than a format string: format with
  // fmt up front. spdlog's variadic log() funnels the format string through
  // spdlog::details::to_string_view(), whose implicit fmt::format_string ->
  // string_view conversion was deprecated in fmt 12.2.0 ("use
  // format_string::get() instead"). Routing through it makes that deprecation
  // warning surface in every downstream that instantiates DART's logging
  // templates against fmt >= 12.2.0 (e.g. gz-physics, see
  // https://github.com/gazebosim/gz-physics/issues/1018). Pre-formatting
  // sidesteps the deprecated conversion entirely.
  //
  // Skip all work for a fully disabled message: no prefix, no allocation and
  // no formatting (cheaper than the previous code path, which always built the
  // format string before spdlog's internal level check). should_backtrace()
  // preserves the behaviour of the prior spdlog::*() calls, which still
  // recorded below-level messages into spdlog's backtrace ring when one is
  // enabled.
  auto* const logger = spdlog::default_logger_raw();
  const spdlog::level::level_enum spdlog_level = toSpdlogLevel(level);
  if (!logger->should_log(spdlog_level) && !logger->should_backtrace()) {
    return;
  }
#endif

  const std::string prefix = makeLogPrefix(file, line, function);
  fmt::basic_string_view<char> fmt_view(format_str);
  std::string final_format;
  if (!prefix.empty()) {
    final_format.reserve(prefix.size() + fmt_view.size());
    final_format += prefix;
    final_format.append(fmt_view.data(), fmt_view.size());
  } else {
    final_format.assign(fmt_view.data(), fmt_view.size());
  }

#if DART_HAVE_spdlog
  // A logging call must never throw, so a malformed format string (only caught
  // at runtime, since the format is not a compile-time constant) falls back to
  // logging the raw format string instead of propagating the fmt error.
  try {
    logger->log(
        spdlog_level,
        fmt::format(
            fmt::runtime(final_format),
            detail_log_arg::normalize(std::forward<Args>(args))...));
  } catch (const std::exception& e) {
    logger->log(
        spdlog_level,
        fmt::format("[log format error: {}] {}", e.what(), final_format));
  }
#else
  auto& stream = (level == LogLevel::Error || level == LogLevel::Fatal)
                     ? std::cerr
                     : std::cout;
  detail::print(
      stream,
      toLabel(level),
      final_format,
      toAnsiColor(level),
      std::forward<Args>(args)...);
#endif
}

template <typename S, typename... Args>
void log(
    LogLevel level,
    const std::source_location& location,
    const S& format_str,
    Args&&... args)
{
  log(level,
      location.file_name(),
      static_cast<int>(location.line()),
      location.function_name(),
      format_str,
      std::forward<Args>(args)...);
}

} // namespace detail

//==============================================================================
template <typename S, typename... Args>
void trace(const S& format_str, [[maybe_unused]] Args&&... args)
{
  detail::log(
      detail::LogLevel::Trace,
      nullptr,
      0,
      nullptr,
      format_str,
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void debug(const S& format_str, [[maybe_unused]] Args&&... args)
{
  detail::log(
      detail::LogLevel::Debug,
      nullptr,
      0,
      nullptr,
      format_str,
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void info(const S& format_str, [[maybe_unused]] Args&&... args)
{
  detail::log(
      detail::LogLevel::Info,
      nullptr,
      0,
      nullptr,
      format_str,
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void warn(const S& format_str, [[maybe_unused]] Args&&... args)
{
  detail::log(
      detail::LogLevel::Warn,
      nullptr,
      0,
      nullptr,
      format_str,
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void error(const S& format_str, [[maybe_unused]] Args&&... args)
{
  detail::log(
      detail::LogLevel::Error,
      nullptr,
      0,
      nullptr,
      format_str,
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, [[maybe_unused]] Args&&... args)
{
  detail::log(
      detail::LogLevel::Fatal,
      nullptr,
      0,
      nullptr,
      format_str,
      std::forward<Args>(args)...);
}

} // namespace dart::common

#endif // DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
