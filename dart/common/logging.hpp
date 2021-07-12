/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#define DART_LOG_LEVEL_TRACE 0
#define DART_LOG_LEVEL_DEBUG 1
#define DART_LOG_LEVEL_INFO 2
#define DART_LOG_LEVEL_WARN 3
#define DART_LOG_LEVEL_ERROR 4
#define DART_LOG_LEVEL_FATAL 5
#define DART_LOG_LEVEL_OFF 6

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

namespace dart {
namespace common {

/// Log level
enum class LogLevel {
  /// The TRACE level designates finer-grained information events than the
  /// DEBUG level.
  ///
  /// This level is intended to use for showing logic-level program flow,
  /// without performance concerns. Not encouraged to use in production.
  LL_TRACE,

  /// The DEBUG level designates fine-grained informational events that are most
  /// useful to debug.
  ///
  /// This level is intended to use for dumping variable state, specific error
  /// codes, etc, which don't affect the performance significantly.
  LL_DEBUG,

  /// The INFO level designates informational messages that highlight the
  /// progress of the application at coarse-grained level.
  ///
  /// Assumed to be used in production.
  LL_INFO,

  /// The WARN level designates potentially harmful situations.
  LL_WARN,

  /// The ERROR level designates error events that might still allow the
  /// application to continue running, but the application might not be able to
  /// achieve the goals.
  LL_ERROR,

  /// The FATAL level designates very sever error events that will presumably
  /// lead the application to abort.
  LL_FATAL,

  /// Use to completely turn off logging.
  LL_OFF,
};
// Informative discussion: https://github.com/dartsim/dart/issues/428

/// Sets log level of the global logger.
///
/// Use LogLevel::OFF to completely turn off logging.
void set_log_level(LogLevel level);

/// Logs for a trace message
template <typename S, typename... Args>
void trace(const S& format_str, Args&&... args);

/// Logs for a debug message
template <typename S, typename... Args>
void debug(const S& format_str, Args&&... args);

/// Logs for a info message
template <typename S, typename... Args>
void info(const S& format_str, Args&&... args);

/// Logs for a warn message
template <typename S, typename... Args>
void warn(const S& format_str, Args&&... args);

/// Logs for a error message
template <typename S, typename... Args>
void error(const S& format_str, Args&&... args);

/// Logs for a fatal message
template <typename S, typename... Args>
void fatal(const S& format_str, Args&&... args);

} // namespace common
} // namespace dart

#include "dart/common/detail/logging_impl.hpp"
