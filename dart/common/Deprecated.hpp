/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_DEPRECATED_HPP_
#define DART_COMMON_DEPRECATED_HPP_

#include "dart/config.hpp"

//==============================================================================
// Deprecated is used for backward compatibility between different minor
// versions of DART. Every deprecated function should be removed for every major
// version up.
//==============================================================================

#if defined(__GNUC__) || defined(__clang__)
  #define DART_DEPRECATED(version) __attribute__ ((deprecated))
  #define DART_FORCEINLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
  #define DART_DEPRECATED(version) __declspec(deprecated)
  #define DART_FORCEINLINE _forceinline
#else
  #define DART_DEPRECATED(version) ()
  #define DART_FORCEINLINE
#endif

// We define two convenient macros that can be used to suppress
// deprecated-warnings for a specific code block rather than a whole project.
// This macros would be useful when you need to call deprecated function for
// some reason (e.g., for backward compatibility) but don't want warnings.
//
// Example code:
//
// deprecated_function()  // warning
//
// DART_SUPPRESS_DEPRECATED_BEGIN
// deprecated_function()  // okay, no warning
// DART_SUPPRESS_DEPRECATED_END
//
#if defined (DART_COMPILER_GCC)

  #define DART_SUPPRESS_DEPRECATED_BEGIN                            \
    _Pragma("GCC diagnostic push")                                  \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")

  #define DART_SUPPRESS_DEPRECATED_END \
    _Pragma("GCC diagnostic pop")

#elif defined (DART_COMPILER_CLANG)

  #define DART_SUPPRESS_DEPRECATED_BEGIN                              \
    _Pragma("clang diagnostic push")                                  \
    _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")

  #define DART_SUPPRESS_DEPRECATED_END \
    _Pragma("clang diagnostic pop")

#elif defined (DART_COMPILER_MSVC)

  #define DART_SUPPRESS_DEPRECATED_BEGIN \
    __pragma(warning(push))              \
    __pragma(warning(disable:4996))

  #define DART_SUPPRESS_DEPRECATED_END \
    __pragma(warning(pop))

#endif

#endif  // DART_COMMON_DEPRECATED_HPP_
