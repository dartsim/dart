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

#include <cstddef>

namespace dart::common {

/// Severity of a message from trace() through fatal().
enum class LogLevel
{
  Trace,
  Debug,
  Info,
  Warn,
  Error,
  Fatal
};

/// Receives one formatted message from trace() through fatal(). It runs on the
/// logging thread, so it must be thread-safe. \p message holds \p length
/// bytes, has no terminating newline and is valid only during the call. A
/// message logged from inside the callback goes to the default output instead
/// of re-entering it. An ordinary exception thrown by the callback is
/// swallowed: DART reports the first one through the error console stream,
/// unfiltered by spdlog's runtime level. The failed message goes to the default
/// output, which still applies its own runtime level. Forced thread unwinding
/// propagates. Do not fork() from inside the callback: the child would inherit
/// the forking thread's re-entry state.
using LogCallback = void (*)(
    LogLevel level, const char* message, std::size_t length, void* userData);

/// Routes every message from trace() through fatal() to \p callback instead of
/// spdlog, or of the console streams in a build without spdlog, whichever
/// library instantiated the logging call. Messages below the compile-time
/// DART_ACTIVE_LOG_LEVEL are still compiled out. The callback receives every
/// remaining message regardless of spdlog's runtime level, so the receiver
/// filters by level itself.
///
/// Each message is formatted once and offered first to the callback, which
/// consults no spdlog registry and takes no lock, so it works after
/// spdlog::drop_all() and in the child of a fork() taken from a
/// single-threaded process. A message the callback declines goes to spdlog's
/// default logger if one exists, or else to the console streams. Each call
/// publishes an immutable
/// {callback, userData} pair through one atomic pointer; a logging thread
/// loads that pointer once per message, which adds one acquire load to the
/// path without a callback. Old pairs are never freed, so \p userData must
/// outlive every in-flight call: unset the callback only while no thread
/// logs, or keep \p userData alive for the rest of the process. A null
/// callback restores the default output. Call setLogCallback() from one
/// thread at a time.
void setLogCallback(LogCallback callback, void* userData = nullptr);

namespace detail {

/// Whether setLogCallback() installed a callback. The answer can be stale by
/// the time the caller acts on it, so a caller must not drop a message on its
/// account; it only decides whether to format a message that spdlog's runtime
/// level would otherwise discard.
bool hasLogCallback();

/// Hands \p message to the installed callback. Returns false, without
/// touching the message, when no callback is installed, when this thread is
/// already inside the callback, or when the callback threw; the caller then
/// sends the message to the default output, whose runtime level still applies.
bool forwardToLogCallback(
    LogLevel level, const char* message, std::size_t length);

} // namespace detail

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
