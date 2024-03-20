/*
 * Copyright (c) The DART development contributors
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

#pragma once

#include <dart/config.hpp>

// clang-format off
#define DART_LOG_LEVEL_TRACE 0
#define DART_LOG_LEVEL_DEBUG 1
#define DART_LOG_LEVEL_INFO  2
#define DART_LOG_LEVEL_WARN  3
#define DART_LOG_LEVEL_ERROR 4
#define DART_LOG_LEVEL_FATAL 5
#define DART_LOG_LEVEL_OFF   6
// clang-format on

// Default active log level
#if !defined(DART_ACTIVE_LOG_LEVEL)
  #define DART_ACTIVE_LOG_LEVEL DART_LOG_LEVEL_INFO
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_TRACE
  #define DART_TRACE(...) ::dart::common::trace(__VA_ARGS__)
#else
  #define DART_TRACE(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_DEBUG
  #define DART_DEBUG(...) ::dart::common::debug(__VA_ARGS__)
#else
  #define DART_DEBUG(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_INFO
  #define DART_INFO(...) ::dart::common::info(__VA_ARGS__)
#else
  #define DART_INFO(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_WARN
  #define DART_WARN(...) ::dart::common::warn(__VA_ARGS__)
#else
  #define DART_WARN(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_ERROR
  #define DART_ERROR(...) ::dart::common::error(__VA_ARGS__)
#else
  #define DART_ERROR(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_FATAL
  #define DART_FATAL(...) ::dart::common::fatal(__VA_ARGS__)
#else
  #define DART_FATAL(...) (void)0
#endif

#define DART_TRACE_IF(condition, ...)                                          \
  if (condition) {                                                             \
    DART_TRACE(__VA_ARGS__);                                                   \
  }

#define DART_DEBUG_IF(condition, ...)                                          \
  if (condition) {                                                             \
    DART_DEBUG(__VA_ARGS__);                                                   \
  }

#define DART_INFO_IF(condition, ...)                                           \
  if (condition) {                                                             \
    DART_INFO(__VA_ARGS__);                                                    \
  }

#define DART_WARN_IF(condition, ...)                                           \
  if (condition) {                                                             \
    DART_WARN(__VA_ARGS__);                                                    \
  }

#define DART_ERROR_IF(condition, ...)                                          \
  if (condition) {                                                             \
    DART_ERROR(__VA_ARGS__);                                                   \
  }

#define DART_FATAL_IF(condition, ...)                                          \
  if (condition) {                                                             \
    DART_FATAL(__VA_ARGS__);                                                   \
  }

namespace dart::common {

/// \brief Logs for a trace message
///
/// Logs for the most fine-grained information than any other log levels.
///
/// \sa info
template <typename S, typename... Args>
void trace(const S& format_str, Args&&... args);

/// \brief Logs for a debug message
///
/// Logs for fine-grained information that is useful for debugging.
///
/// \sa info
template <typename S, typename... Args>
void debug(const S& format_str, Args&&... args);

/// \brief Logs for a information message
///
/// Logs for useful information from normal operations.
///
/// You can use a Python like formatting API as
/// \code
/// dart::common::info("Hello {}!", "World");  // logged as "Hello World!"
/// \endcode
template <typename S, typename... Args>
void info(const S& format_str, Args&&... args);

/// \brief Logs for a warning message
///
/// Logs for warning information that is potentially harmful.
///
/// \sa info
template <typename S, typename... Args>
void warn(const S& format_str, Args&&... args);

/// \brief Logs for a error message
///
/// Logs for errors that might still allow the application to continue running,
/// but the application might lead to unexpected behavior.
///
/// \sa info
template <typename S, typename... Args>
void error(const S& format_str, Args&&... args);

/// \brief Logs for a fatal error message
///
/// Logs for highly sever errors that will presumably lead the application to
/// crash.
///
/// \sa info
template <typename S, typename... Args>
void fatal(const S& format_str, Args&&... args);

} // namespace dart::common

//==============================================================================
//
// Implementation
//
//==============================================================================

#if DART_HAVE_SPDLOG
  #include <spdlog/spdlog.h>
#else
  #include <iostream>
  #include <string>
#endif

namespace dart::common {

#if !DART_HAVE_SPDLOG
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
#if DART_HAVE_SPDLOG
  spdlog::trace(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[trace]", format_str, 38, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void debug(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::debug(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[debug]", format_str, 36, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void info(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::info(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[info]", format_str, 32, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void warn(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::warn(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[warn]", format_str, 33, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void error(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::error(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[error]", format_str, 31, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::critical(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[fatal]", format_str, 35, std::forward<Args>(args)...);
#endif
}

} // namespace dart::common
