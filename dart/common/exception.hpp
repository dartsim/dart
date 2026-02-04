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

#pragma once

#include <dart/common/export.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <exception>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>

namespace dart::common {

//==============================================================================
/// @brief Base exception class for DART with source location tracking.
///
/// All DART exceptions derive from this class and include:
/// - Original error message
/// - Source location (file, line, function) where exception was thrown
/// - Formatted what() message for debugging
///
/// @note This exception system can be disabled at compile-time by defining
/// DART_DISABLE_EXCEPTIONS. In that mode, errors are logged and std::terminate
/// is called (or a custom error handler is invoked).
///
/// Example usage:
/// @code
/// if (index >= size) {
///     throw dart::common::OutOfRangeException(
///         fmt::format("Index {} out of range [0, {})", index, size));
/// }
/// @endcode
///
/// Or using macros:
/// @code
/// DART_THROW_T_IF(index >= size, OutOfRangeException,
///     "Index {} out of range [0, {})", index, size);
/// @endcode
class Exception : public std::runtime_error
{
public:
  /// @brief Construct an exception with a message and source location.
  /// @param[in] message The error message describing the problem.
  /// @param[in] location Source location where the exception was created.
  explicit Exception(
      const std::string& message,
      const std::source_location& location = std::source_location::current())
    : std::runtime_error(formatMessage(message, location)),
      m_message(message),
      m_location(location)
  {
  }

  /// @brief Get the original message without source location formatting.
  [[nodiscard]] const std::string& message() const noexcept
  {
    return m_message;
  }

  /// @brief Get the source location where the exception was thrown.
  [[nodiscard]] const std::source_location& location() const noexcept
  {
    return m_location;
  }

private:
  /// Format the exception message with source location for display.
  static std::string formatMessage(
      const std::string& message, const std::source_location& location)
  {
    std::string_view filepath = location.file_name();
    std::string_view func = location.function_name();
    std::string_view funcName = func;

    // Extract simple function name from signature
    // e.g., "void foo::bar(int)" -> "bar"
    auto paren = func.find('(');
    if (paren != std::string_view::npos) {
      auto space = func.rfind(' ', paren);
      auto colon = func.rfind("::", paren);
      auto start = std::max(
          space == std::string_view::npos ? 0 : space + 1,
          colon == std::string_view::npos ? 0 : colon + 2);
      funcName = func.substr(start, paren - start);
    }

    return fmt::format(
        "\n[DART Exception]\n"
        "  Message:  {}\n"
        "  Location: {}:{} in {}()",
        message,
        filepath,
        location.line(),
        funcName);
  }

  std::string m_message;
  std::source_location m_location;
};

//==============================================================================
// Derived Exception Types
//==============================================================================

/// @brief Exception for invalid arguments passed to functions.
/// @note Use this for programmer errors where the caller passed bad data.
class InvalidArgumentException : public Exception
{
public:
  using Exception::Exception;
};

/// @brief Exception for index/iterator out of valid range.
class OutOfRangeException : public Exception
{
public:
  using Exception::Exception;
};

/// @brief Exception for null pointer where non-null was required.
class NullPointerException : public Exception
{
public:
  using Exception::Exception;
};

/// @brief Exception for calling functionality not yet implemented.
class NotImplementedException : public Exception
{
public:
  using Exception::Exception;
};

/// @brief Exception for operations invalid in the current state.
/// @note Use this when an operation is valid in general but not in the
///       current object state (e.g., modifying a finalized object).
class InvalidOperationException : public Exception
{
public:
  using Exception::Exception;
};

/// @brief Exception for file/resource not found.
class FileNotFoundException : public Exception
{
public:
  using Exception::Exception;
};

/// @brief Exception for parsing errors in file formats (URDF, SDF, etc.).
class ParseException : public Exception
{
public:
  using Exception::Exception;
};

//==============================================================================
// Error Handler for Exception-Free Mode
//==============================================================================

/// @brief Function signature for custom error handlers.
///
/// Error handlers receive:
/// - exceptionType: Name of the exception type (e.g.,
/// "InvalidArgumentException")
/// - message: The error message
/// - location: Source location where error occurred
///
/// @note Only called when DART_DISABLE_EXCEPTIONS is defined.
using ErrorHandler = void (*)(
    const char* exceptionType,
    const char* message,
    const std::source_location& location);

/// @brief Set a custom error handler for exception-free mode.
///
/// When DART_DISABLE_EXCEPTIONS is defined, this handler is called BEFORE
/// std::terminate(). Use it for custom logging or cleanup. The handler cannot
/// prevent termination - DART_THROW always terminates in exception-free mode.
///
/// @param[in] handler Function pointer to custom handler, or nullptr to reset.
///
/// Example:
/// @code
/// dart::common::setErrorHandler([](const char* type, const char* msg, auto
/// loc) {
///     myLogger.fatal("{} at {}:{}: {}", type, loc.file_name(), loc.line(),
///     msg);
///     // Log to telemetry, flush buffers, etc. before terminate
/// });
/// @endcode
DART_DLL_EXPORT void setErrorHandler(ErrorHandler handler);

/// @brief Get the current error handler.
/// @return Current handler, or nullptr if using default behavior.
DART_DLL_EXPORT ErrorHandler getErrorHandler();

} // namespace dart::common

//==============================================================================
// Exception Throwing Macros
//==============================================================================
//
// These macros provide a unified interface for error handling that can be
// configured at compile-time:
//
// - Default (exceptions enabled): Throws the specified exception type.
// - DART_DISABLE_EXCEPTIONS defined: Logs error and calls std::terminate
//   (or custom error handler if set).
//
// Macro Naming Convention:
// - DART_THROW(msg, ...): Throw default Exception type
// - DART_THROW_IF(cond, msg, ...): Throw if condition is TRUE
// - DART_THROW_T(Type, msg, ...): Throw specific exception Type
// - DART_THROW_T_IF(cond, Type, msg, ...): Throw Type if condition is TRUE
//
// All message arguments support fmt::format() style formatting.
//
// Examples:
//   DART_THROW("Unexpected state");
//   DART_THROW_IF(ptr == nullptr, "Null pointer");
//   DART_THROW_T(InvalidArgumentException, "Invalid value: {}", value);
//   DART_THROW_T_IF(index >= size, OutOfRangeException,
//       "Index {} >= size {}", index, size);
//==============================================================================

#ifndef DART_DISABLE_EXCEPTIONS

// Exception mode (DEFAULT): throw exceptions as normal

  #define DART_THROW_T(ExceptionType, ...)                                     \
    throw ExceptionType(                                                       \
        fmt::format(__VA_ARGS__), std::source_location::current())

#else

// Exception-free mode: log error, call handler if set, then terminate.
// The handler is for logging/cleanup before termination, not for continuation.
// DART_THROW is always [[noreturn]] - it either throws or terminates.

  #include <cstdio>
  #include <cstdlib>

  #define DART_THROW_T(ExceptionType, ...)                                     \
    do {                                                                       \
      auto loc = std::source_location::current();                              \
      auto msg = fmt::format(__VA_ARGS__);                                     \
      auto handler = ::dart::common::getErrorHandler();                        \
      if (handler) {                                                           \
        handler(#ExceptionType, msg.c_str(), loc);                             \
      }                                                                        \
      std::fprintf(                                                            \
          stderr,                                                              \
          "\n[DART Fatal Error: %s]\n"                                         \
          "  Message: %s\n"                                                    \
          "  File: %s:%u\n"                                                    \
          "  Function: %s\n\n",                                                \
          #ExceptionType,                                                      \
          msg.c_str(),                                                         \
          loc.file_name(),                                                     \
          loc.line(),                                                          \
          loc.function_name());                                                \
      std::terminate();                                                        \
    } while (false)

#endif // DART_DISABLE_EXCEPTIONS

/// @brief Throw exception of ExceptionType if condition is TRUE.
/// @note Supports fmt::format() style message arguments.
#define DART_THROW_T_IF(condition, ExceptionType, ...)                         \
  do {                                                                         \
    if (condition) {                                                           \
      DART_THROW_T(ExceptionType, __VA_ARGS__);                                \
    }                                                                          \
  } while (false)

/// @brief Unconditionally throw dart::common::Exception.
/// @note Supports fmt::format() style message arguments.
#define DART_THROW(...) DART_THROW_T(::dart::common::Exception, __VA_ARGS__)

/// @brief Throw dart::common::Exception if condition is TRUE.
/// @note Supports fmt::format() style message arguments.
#define DART_THROW_IF(condition, ...)                                          \
  DART_THROW_T_IF(condition, ::dart::common::Exception, __VA_ARGS__)
