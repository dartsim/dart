/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart8/export.hpp>

#include <exception>
#include <format>
#include <source_location>
#include <stdexcept>
#include <string>

//===============================================================================
// Exception Macros - Put at top for easy access
//===============================================================================

namespace dart8 {

/// Base exception class for DART8
class Exception : public std::runtime_error
{
public:
  explicit Exception(
      const std::string& message,
      const std::source_location& location = std::source_location::current())
    : std::runtime_error(formatMessage(message, location)),
      m_message(message),
      m_location(location)
  {
  }

  /// Get the original message without formatting
  [[nodiscard]] const std::string& message() const noexcept
  {
    return m_message;
  }

  /// Get the source location where the exception was thrown
  [[nodiscard]] const std::source_location& location() const noexcept
  {
    return m_location;
  }

private:
  static std::string formatMessage(
      const std::string& message, const std::source_location& location)
  {
    // Use full file path for clickable links in VS Code terminal
    std::string_view filepath = location.file_name();

    // Simplify function signature - extract just function name
    std::string_view func = location.function_name();
    std::string_view funcName = func;

    // Try to extract simple function name from signature
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

    return std::format(
        "\n[DART8 Exception]\n"
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

/// Invalid argument exception
class InvalidArgumentException : public Exception
{
public:
  using Exception::Exception;
};

/// Out of range exception
class OutOfRangeException : public Exception
{
public:
  using Exception::Exception;
};

/// Null pointer exception
class NullPointerException : public Exception
{
public:
  using Exception::Exception;
};

/// Not implemented exception
class NotImplementedException : public Exception
{
public:
  using Exception::Exception;
};

/// Invalid operation exception
class InvalidOperationException : public Exception
{
public:
  using Exception::Exception;
};

/// File not found exception
class FileNotFoundException : public Exception
{
public:
  using Exception::Exception;
};

//==============================================================================
// Custom Error Handler Support
//==============================================================================

/// Function signature for custom error handlers
///
/// Error handlers receive:
/// - exceptionType: Name of the exception type (e.g.,
/// "InvalidArgumentException")
/// - message: Formatted error message
/// - location: Source location where error occurred
using ErrorHandler = void (*)(
    const char* exceptionType,
    const char* message,
    const std::source_location& location);

/// Set a custom error handler for exception-free mode
///
/// When DART8_DISABLE_EXCEPTIONS is defined, this handler will be called
/// instead of the default behavior (print to stderr + std::terminate).
///
/// \param[in] handler Function pointer to custom handler, or nullptr to reset
///
/// Example:
/// \code
///   dart8::setErrorHandler([](const char* type, const char* msg, auto loc) {
///     myLogger.fatal("{} at {}:{}: {}", type, loc.file_name(), loc.line(),
///     msg);
///     // Can attempt recovery or exit gracefully
///   });
/// \endcode
DART8_API void setErrorHandler(ErrorHandler handler);

/// Get the current error handler
///
/// \return Current handler, or nullptr if using default behavior
DART8_API ErrorHandler getErrorHandler();

} // namespace dart8

//===============================================================================
// Exception Handling Macros - CMake Controlled
//===============================================================================
//
// DART8 provides two sets of macros with concise, clear naming:
//
// 1. Default exception type (convenience - shortest):
//    - DART8_THROW(msg, ...)
//        Throws dart8::Exception with source location
//    - DART8_THROW_IF(condition, msg, ...)
//        Throws dart8::Exception if condition is TRUE
//
// 2. Explicit exception type (precision - _T suffix):
//    - DART8_THROW_T(Type, msg, ...)
//        Throws Type with source location
//    - DART8_THROW_T_IF(condition, Type, msg, ...)
//        Throws Type if condition is TRUE
//
// Exception-free mode (opt-in via DART8_DISABLE_EXCEPTIONS):
//    - Prints error message with source location to stderr
//    - Calls custom error handler (if set) or std::terminate()
//    - Suitable for embedded systems, real-time systems, or when
//    -fno-exceptions
//
// Control via CMake:
//   cmake -DDART8_DISABLE_EXCEPTIONS=ON ...
//
// Examples:
//   // Default type (quick checks/assertions)
//   DART8_THROW("Unexpected state");
//   DART8_THROW_IF(ptr == nullptr, "Null pointer");
//   DART8_THROW_IF(value < 0, "Value {} must be non-negative", value);
//
//   // Explicit type (API validation, specific errors)
//   DART8_THROW_T(InvalidArgumentException, "Invalid value: {}", value);
//   DART8_THROW_T_IF(index >= size, OutOfRangeException, "Index {} >= size {}",
//   index, size);
//
//===============================================================================

#ifndef DART8_DISABLE_EXCEPTIONS

  // Exception mode (DEFAULT): throw exceptions as normal
  #define DART8_THROW_T(ExceptionType, ...)                                    \
    throw ExceptionType(                                                       \
        ::std::format(__VA_ARGS__), std::source_location::current())

#else

  // Exception-free mode: print error and terminate
  #include <cstdio>
  #include <cstdlib>

  #define DART8_THROW_T(ExceptionType, ...)                                    \
    do {                                                                       \
      auto loc = std::source_location::current();                              \
      std::fprintf(                                                            \
          stderr,                                                              \
          "\n[DART8 Fatal Error: %s]\n"                                        \
          "  Message: %s\n"                                                    \
          "  File: %s:%u\n"                                                    \
          "  Function: %s\n\n",                                                \
          #ExceptionType,                                                      \
          ::std::format(__VA_ARGS__).c_str(),                                  \
          loc.file_name(),                                                     \
          loc.line(),                                                          \
          loc.function_name());                                                \
      std::terminate();                                                        \
    } while (false)

#endif // DART8_DISABLE_EXCEPTIONS

// DART8_THROW_T_IF: Throw exception if condition is TRUE (explicit type)
// Supports std::format() style arguments
//
// Usage:
//   DART8_THROW_T_IF(index >= size, OutOfRangeException, "Index {} >= size {}",
//   index, size) DART8_THROW_T_IF(ptr == nullptr, NullPointerException,
//   "Pointer is null")
#define DART8_THROW_T_IF(condition, ExceptionType, ...)                        \
  do {                                                                         \
    if (condition) {                                                           \
      DART8_THROW_T(ExceptionType, __VA_ARGS__);                               \
    }                                                                          \
  } while (false)

//===============================================================================
// Default exception type (dart8::Exception) - Shortest names
//===============================================================================

// DART8_THROW: Unconditionally throw dart8::Exception
// Supports std::format() style arguments
//
// Usage:
//   DART8_THROW("Unexpected state");
//   DART8_THROW("Invalid value: {}", value);
#define DART8_THROW(...) DART8_THROW_T(dart8::Exception, __VA_ARGS__)

// DART8_THROW_IF: Throw dart8::Exception if condition is TRUE
// Supports std::format() style arguments
//
// Usage:
//   DART8_THROW_IF(ptr == nullptr, "Null pointer");
//   DART8_THROW_IF(value < 0, "Value {} must be non-negative", value);
#define DART8_THROW_IF(condition, ...)                                         \
  DART8_THROW_T_IF(condition, dart8::Exception, __VA_ARGS__)
