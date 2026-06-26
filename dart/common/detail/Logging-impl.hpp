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

#if DART_HAVE_spdlog
  #include <fmt/format.h>
  #include <spdlog/spdlog.h>

  #include <exception>
#else
  #include <iostream>
  #include <string>
#endif

namespace dart::common {

#if DART_HAVE_spdlog
namespace detail {

//==============================================================================
/// Logs a runtime-formatted message through spdlog without handing spdlog the
/// format string. spdlog's variadic log() funnels the format string through
/// spdlog::details::to_string_view(), whose implicit fmt::format_string ->
/// string_view conversion was deprecated in fmt 12.2.0 ("use
/// format_string::get() instead"). Because DART's logging templates are
/// header-only and instantiated by downstreams, that deprecation warning
/// surfaces in every consumer built against fmt >= 12.2.0 (e.g. gz-physics,
/// see https://github.com/gazebosim/gz-physics/issues/1018). Pre-formatting
/// with fmt and handing spdlog a ready-made string sidesteps the deprecated
/// conversion entirely.
template <typename S, typename... Args>
void logToSpdlog(
    spdlog::level::level_enum level, const S& format_str, Args&&... args)
{
  auto* const logger = spdlog::default_logger_raw();

  // Skip all work for a fully disabled message. should_backtrace() preserves
  // the behaviour of the prior spdlog::*() calls, which still recorded
  // below-level messages into spdlog's backtrace ring when one is enabled.
  if (!logger->should_log(level) && !logger->should_backtrace()) {
    return;
  }

  // A logging call must never throw, so a malformed format string (only caught
  // at runtime, since the format is not a compile-time constant) falls back to
  // logging the raw format string instead of propagating the fmt error.
  try {
    logger->log(
        level,
        fmt::format(fmt::runtime(format_str), std::forward<Args>(args)...));
  } catch (const std::exception& e) {
    logger->log(
        level, fmt::format("[log format error: {}] {}", e.what(), format_str));
  }
}

} // namespace detail
#endif

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

//==============================================================================
template <typename S, typename... Args>
void trace(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(
      spdlog::level::trace, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[trace]", format_str, 38, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void debug(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(
      spdlog::level::debug, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[debug]", format_str, 36, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void info(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(
      spdlog::level::info, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[info]", format_str, 32, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void warn(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(
      spdlog::level::warn, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[warn]", format_str, 33, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void error(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(
      spdlog::level::err, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[error]", format_str, 31, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  detail::logToSpdlog(
      spdlog::level::critical, format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[fatal]", format_str, 35, std::forward<Args>(args)...);
#endif
}

} // namespace dart::common

#endif // DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
