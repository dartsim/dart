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

#include <dart/config.hpp>

#include <functional>
#include <string>

#include <cstddef>

// clang-format off
#define DART_LOG_LEVEL_TRACE 0
#define DART_LOG_LEVEL_DEBUG 1
#define DART_LOG_LEVEL_INFO  2
#define DART_LOG_LEVEL_WARN  3
#define DART_LOG_LEVEL_ERROR 4
#define DART_LOG_LEVEL_FATAL 5
#define DART_LOG_LEVEL_OFF   6
// clang-format on

// Defining DART_ACTIVE_LOG_LEVEL in a consumer changes only which DART_* macros
// compile. Use setLogLevel() to change the library's runtime threshold.
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

/// Logging severity, ordered from most verbose to disabled.
enum class LogLevel
{
  Trace = DART_LOG_LEVEL_TRACE,
  Debug = DART_LOG_LEVEL_DEBUG,
  Info = DART_LOG_LEVEL_INFO,
  Warn = DART_LOG_LEVEL_WARN,
  Error = DART_LOG_LEVEL_ERROR,
  Fatal = DART_LOG_LEVEL_FATAL,
  Off = DART_LOG_LEVEL_OFF
};

using LogSink = std::function<void(LogLevel, const std::string&)>;

/// Replace the process-wide sink. An empty sink restores the default sink.
/// Return a copy of the previous sink for restoration, or empty for the
/// default. Callbacks run synchronously and are serialized, including across
/// threads. They may log or replace the sink; a message logged from inside a
/// callback that reaches the same running sink uses default output instead.
/// Ordinary exceptions are ignored; forced thread unwinding propagates.
/// The message reference is valid only
/// for the duration of the callback. Captured resources must remain alive until
/// the sink is replaced or reset.
LogSink setLogSink(LogSink sink);

/// Restore console output: warn and above to stderr, others to stdout. Color
/// requires a terminal, NO_COLOR unset and TERM other than "dumb"; this is
/// checked once per stream.
void resetLogSink();

/// Set the runtime threshold, initially the more restrictive of Info and the
/// library's DART_ACTIVE_LOG_LEVEL. Lower it to enable debug or trace messages.
/// Off disables sink output; an installed LogCallback still receives messages.
/// Compile-time disabled DART_* macros cannot be enabled at runtime.
void setLogLevel(LogLevel level);

/// Return the runtime threshold without allocating; lock-free on typical
/// platforms.
LogLevel getLogLevel();

/// Receives one formatted message from trace() through fatal(). It runs on the
/// logging thread, so it must be thread-safe. \p message holds \p length
/// bytes, has no terminating newline and is valid only during the call. A
/// message logged from inside the callback goes to the default output instead
/// of re-entering it. An ordinary exception thrown by the callback is
/// swallowed: DART reports the first one through the error console stream,
/// unfiltered by the sink's runtime level. The failed message goes to the
/// selected sink or default output, which still applies its own runtime level.
/// Forced thread unwinding propagates. Do not fork() from inside the callback:
/// the child would inherit the forking thread's re-entry state.
using LogCallback = void (*)(
    LogLevel level, const char* message, std::size_t length, void* userData);

/// Routes every message from trace() through fatal() to \p callback before the
/// selected sink. Compile-time disabled DART_* macros remain disabled. The
/// callback receives messages regardless of setLogLevel(), including Off, so
/// the receiver filters by level itself.
///
/// Each message is formatted once and offered first to the callback, before
/// acquiring or initializing sink state, so the successful callback path works
/// in the child of a fork() taken from a single-threaded process. A message
/// the callback declines goes to the selected sink or console streams, subject
/// to the runtime threshold. Each call publishes an immutable
/// {callback, userData} pair through one atomic pointer; a logging thread
/// loads that pointer once per delivery attempt. Old pairs are never freed, so
/// \p userData must outlive every in-flight call: unset the callback only while
/// no thread logs, or keep \p userData alive for the rest of the process. A
/// null callback restores the default output. Call setLogCallback() from one
/// thread at a time.
void setLogCallback(LogCallback callback, void* userData = nullptr);

namespace detail {

/// Whether setLogCallback() installed a callback. The answer can be stale by
/// the time the caller acts on it, so a caller must not drop a message on its
/// account; it only decides whether to format a message that the sink's runtime
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

} // namespace common
} // namespace dart

#include <dart/common/detail/Logging-impl.hpp>

#endif // DART_COMMON_LOGGING_HPP_
