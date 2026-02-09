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

  #include <type_traits>
  #include <utility>
  // Check if spdlog is using fmt or std::format backend
  #if defined(SPDLOG_USE_STD_FORMAT)
    // spdlog uses std::format - no need for runtime wrapper
    #define DART_SPDLOG_RUNTIME(str) str
  #else
    // spdlog uses fmt - need fmt::runtime for non-compile-time strings
    #include <fmt/ostream.h>
    #include <spdlog/fmt/fmt.h>
    #include <spdlog/fmt/ostr.h>

    #include <ostream>

namespace detail_log_arg {

template <typename Stream, typename T, typename = void>
struct is_stream_insertable : std::false_type
{
};

template <typename Stream, typename T>
struct is_stream_insertable<
    Stream,
    T,
    std::void_t<decltype(std::declval<Stream&>() << std::declval<const T&>())>>
  : std::true_type
{
};

template <typename T, typename = void>
struct has_pointer_get : std::false_type
{
};

template <typename T>
struct has_pointer_get<T, std::void_t<decltype(std::declval<T&>().get())>>
  : std::bool_constant<std::is_pointer_v<
        std::remove_reference_t<decltype(std::declval<T&>().get())>>>
{
};

template <typename T>
auto normalize(T&& arg)
{
  using Decayed = std::decay_t<T>;
  if constexpr (std::is_pointer_v<Decayed>) {
    using Pointee = std::remove_cv_t<std::remove_pointer_t<Decayed>>;
    if constexpr (
        !std::is_same_v<Pointee, char> && !std::is_same_v<Pointee, signed char>
        && !std::is_same_v<Pointee, unsigned char>) {
      return fmt::ptr(arg);
    } else {
      return std::forward<T>(arg);
    }
  } else if constexpr (has_pointer_get<Decayed>::value) {
    using RawPointer
        = std::remove_reference_t<decltype(std::declval<Decayed&>().get())>;
    using Pointee = std::remove_cv_t<std::remove_pointer_t<RawPointer>>;
    if constexpr (
        !std::is_same_v<Pointee, char> && !std::is_same_v<Pointee, signed char>
        && !std::is_same_v<Pointee, unsigned char>) {
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
    } else if constexpr (is_stream_insertable<std::ostream, Decayed>::value) {
      return fmt::streamed(std::forward<T>(arg));
    } else {
      return std::forward<T>(arg);
    }
  }
}

} // namespace detail_log_arg

    #define DART_SPDLOG_RUNTIME(str) fmt::runtime(str)
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
  spdlog::log(
      toSpdlogLevel(level),
      DART_SPDLOG_RUNTIME(final_format),
      detail_log_arg::normalize(std::forward<Args>(args))...);
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
