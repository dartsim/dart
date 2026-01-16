/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <iostream>
#include <source_location>

#include <cassert>
#include <cstdlib>

//===============================================================================
// Assertion Macros - Put at top for easy access
//===============================================================================
//
// DART_EXPERIMENTAL_ASSERT vs DART_EXPERIMENTAL_VERIFY - When to use which?
//
// DART_EXPERIMENTAL_ASSERT(condition)
//   - Only active in DEBUG builds (disabled when NDEBUG is defined)
//   - Use for checking preconditions, postconditions, and invariants
//   - Should NOT have side effects (condition won't be evaluated in release)
//   - Example: DART_EXPERIMENTAL_ASSERT(index < size);
//
// DART_EXPERIMENTAL_VERIFY(condition)
//   - Always active in ALL builds (debug and release)
//   - Use when the condition has side effects that must always execute
//   - Returns bool result (same as condition) so it can be used in expressions
//   - Example: if (!DART_EXPERIMENTAL_VERIFY(file.open())) { handle_error(); }
//
// Recommendation: Prefer DART_EXPERIMENTAL_ASSERT for most checks. Only use
// DART_EXPERIMENTAL_VERIFY when the condition expression has necessary side
// effects.
//===============================================================================

namespace dart::simulation::experimental::common {

/// Print assertion failure message and abort
inline void assertionFailed(
    const char* condition,
    const std::source_location& location = std::source_location::current())
{
  std::cerr << "\n[SIMULATION-EXPERIMENTAL ASSERTION FAILED]\n"
            << "  Condition: " << condition << "\n"
            << "  File: " << location.file_name() << "\n"
            << "  Line: " << location.line() << "\n"
            << "  Function: " << location.function_name() << "\n"
            << std::endl;
  std::abort();
}

/// Print assertion failure message with custom message and abort
inline void assertionFailed(
    const char* condition,
    const char* message,
    const std::source_location& location = std::source_location::current())
{
  std::cerr << "\n[SIMULATION-EXPERIMENTAL ASSERTION FAILED]\n"
            << "  Condition: " << condition << "\n"
            << "  Message: " << message << "\n"
            << "  File: " << location.file_name() << "\n"
            << "  Line: " << location.line() << "\n"
            << "  Function: " << location.function_name() << "\n"
            << std::endl;
  std::abort();
}

} // namespace dart::simulation::experimental::common

// DART_EXPERIMENTAL_ASSERT: Assertion only enabled in debug builds
#ifndef NDEBUG
  #define DART_EXPERIMENTAL_ASSERT(condition)                                  \
    do {                                                                       \
      if (!(condition)) {                                                      \
        ::dart::simulation::experimental::common::assertionFailed(#condition); \
      }                                                                        \
    } while (false)

  #define DART_EXPERIMENTAL_ASSERT_MSG(condition, message)                     \
    do {                                                                       \
      if (!(condition)) {                                                      \
        ::dart::simulation::experimental::common::assertionFailed(             \
            #condition, message);                                              \
      }                                                                        \
    } while (false)
#else
  #define DART_EXPERIMENTAL_ASSERT(condition) ((void)0)
  #define DART_EXPERIMENTAL_ASSERT_MSG(condition, message) ((void)0)
#endif

// DART_EXPERIMENTAL_DEBUG_ASSERT: Alias for DART_EXPERIMENTAL_ASSERT (for
// clarity)
#define DART_EXPERIMENTAL_DEBUG_ASSERT(condition)                              \
  DART_EXPERIMENTAL_ASSERT(condition)
#define DART_EXPERIMENTAL_DEBUG_ASSERT_MSG(condition, message)                 \
  DART_EXPERIMENTAL_ASSERT_MSG(condition, message)

// DART_EXPERIMENTAL_VERIFY: Always evaluates condition in ALL builds
// Returns the bool result so it can be used in if statements
#define DART_EXPERIMENTAL_VERIFY(condition)                                    \
  ([](bool result) {                                                           \
    if (!result) {                                                             \
      ::dart::simulation::experimental::common::assertionFailed(#condition);   \
    }                                                                          \
    return result;                                                             \
  }(static_cast<bool>(condition)))

#define DART_EXPERIMENTAL_VERIFY_MSG(condition, message)                       \
  ([](bool result) {                                                           \
    if (!result) {                                                             \
      ::dart::simulation::experimental::common::assertionFailed(               \
          #condition, message);                                                \
    }                                                                          \
    return result;                                                             \
  }(static_cast<bool>(condition)))

// DART_EXPERIMENTAL_NOT_REACHED: Mark code that should never be reached
#define DART_EXPERIMENTAL_NOT_REACHED()                                        \
  ::dart::simulation::experimental::common::assertionFailed(                   \
      "Code should not be reached")

// DART_EXPERIMENTAL_NOT_IMPLEMENTED: Mark unimplemented functionality
#define DART_EXPERIMENTAL_NOT_IMPLEMENTED()                                    \
  ::dart::simulation::experimental::common::assertionFailed(                   \
      "Not yet implemented")
