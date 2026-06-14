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

#ifndef DART_COMMON_MACROS_HPP_
#define DART_COMMON_MACROS_HPP_

#include <dart/common/logging.hpp>

#include <utility>

#include <cassert>
#include <cstdlib>

// DART_NUM_ARGS(<arg1> [, <arg2> [, ...]])
#define DETAIL_DART_NUM_ARGS(z, a, b, c, d, e, f, cnt, ...) cnt
#define DART_NUM_ARGS(...)                                                     \
  DETAIL_DART_NUM_ARGS(, ##__VA_ARGS__, 6, 5, 4, 3, 2, 1, 0)

// DART_CONCAT(a, b)
#define DETAIL_DART_CONCAT(a, b) a##b
#define DART_CONCAT(a, b) DETAIL_DART_CONCAT(a, b)

// Macro to suppress -Wunused-parameter and -Wunused-variable warnings in
// release mode when a variable is only used in assertions.
//
// Usage: DART_UNUSED(<variable1> [, <variable2> [, ...]])
#define DETAIL_DART_UNUSED_0()
#define DETAIL_DART_UNUSED_1(a) (void)(a)
#define DETAIL_DART_UNUSED_2(a, b) (void)(a), DETAIL_DART_UNUSED_1(b)
#define DETAIL_DART_UNUSED_3(a, b, c) (void)(a), DETAIL_DART_UNUSED_2(b, c)
#define DETAIL_DART_UNUSED_4(a, b, c, d)                                       \
  (void)(a), DETAIL_DART_UNUSED_3(b, c, d)
#define DETAIL_DART_UNUSED_5(a, b, c, d, e)                                    \
  (void)(a), DETAIL_DART_UNUSED_4(b, c, d, e)
#define DETAIL_DART_UNUSED_6(a, b, c, d, e, f)                                 \
  (void)(a), DETAIL_DART_UNUSED_5(b, c, d, e, f)
#define DART_UNUSED(...)                                                       \
  DART_CONCAT(DETAIL_DART_UNUSED_, DART_NUM_ARGS(__VA_ARGS__))(__VA_ARGS__)

// DART_ASSERT(<expression> [, <message>])
#define DETAIL_DART_ASSERT_1(condition) assert(condition)
#define DETAIL_DART_ASSERT_2(condition, message) assert((condition) && #message)
#define DART_ASSERT(...)                                                       \
  DART_CONCAT(DETAIL_DART_ASSERT_, DART_NUM_ARGS(__VA_ARGS__))                 \
  (__VA_ARGS__)

// DART_UNREACHABLE()
//
// Marks a program point the author guarantees is never reached (for example,
// the end of a function whose switch already handles every enumerator).
//
//   - Debug builds: logs a fatal diagnostic and aborts, so a logic error that
//     does reach the point is caught loudly instead of silently triggering
//     undefined behavior.
//   - Release builds (NDEBUG): lowers to std::unreachable() -- or the compiler
//     builtin on toolchains that predate it -- letting the optimizer drop the
//     dead branch and surrounding bounds checks.
//
// Only use this where the branch is genuinely unreachable for every valid
// input. Paths reachable on malformed user input must return an error or keep
// an assertion instead: reaching a DART_UNREACHABLE() is undefined behavior.
#if defined(__cpp_lib_unreachable) && __cpp_lib_unreachable >= 202202L
  #define DETAIL_DART_UNREACHABLE_BUILTIN() ::std::unreachable()
#elif defined(__GNUC__) || defined(__clang__)
  #define DETAIL_DART_UNREACHABLE_BUILTIN() __builtin_unreachable()
#elif defined(_MSC_VER)
  #define DETAIL_DART_UNREACHABLE_BUILTIN() __assume(false)
#else
  #define DETAIL_DART_UNREACHABLE_BUILTIN() ((void)0)
#endif

#if defined(NDEBUG)
  #define DART_UNREACHABLE() DETAIL_DART_UNREACHABLE_BUILTIN()
#else
  #define DART_UNREACHABLE()                                                   \
    do {                                                                       \
      DART_FATAL("Unreachable code reached at {}:{}", __FILE__, __LINE__);     \
      ::std::abort();                                                          \
    } while (false)
#endif

// Macro to mark the function is not implemented
#define DART_NOT_IMPLEMENTED                                                   \
  DART_FATAL("Not implemented: {}:{}", __FILE__, __LINE__);                    \
  void(0)

#endif
