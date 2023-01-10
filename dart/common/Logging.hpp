/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_COMMON_LOGGING_HPP_
#define DART_COMMON_LOGGING_HPP_

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

#include <dart/common/detail/Logging-impl.hpp>

#endif // DART_COMMON_LOGGING_HPP_
