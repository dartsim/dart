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

#ifndef DART_COMMON_PLATFORM_HPP_
#define DART_COMMON_PLATFORM_HPP_

// Operating systems and architectures
#if defined(__linux__)

#define DART_OS_LINUX      1
#if __x86_64__ || __ppc64__
#define DART_ARCH_64BITS   1
#else
#define DART_ARCH_32BITS   1
#endif

#elif defined(__APPLE__)

#define DART_OS_MACOS      1
#if __LP64__
#define DART_ARCH_64BITS   1
#else
#define DART_ARCH_32BITS   1
#endif

#elif defined(_WIN32)

#define DART_OS_WINDOWS    1
#define DART_ARCH_32BITS   1

#elif defined(_WIN64)

#define DART_OS_WINDOWS    1
#define DART_ARCH_64BITS   1

#else

#error Unsupported platform.

#endif // if defined(__linux__)

// Define undefined preprocessors as 0
#ifndef DART_OS_WINDOWS
  #define DART_OS_WINDOWS  0
#endif

#ifndef DART_OS_LINUX
  #define DART_OS_LINUX    0
#endif

#ifndef DART_OS_MACOS
  #define DART_OS_MACOS   0
#endif

#ifndef DART_ARCH_32BITS
  #define DART_ARCH_32BITS 0
#endif

#ifndef DART_ARCH_64BITS
  #define DART_ARCH_64BITS 0
#endif

#endif // DART_COMMON_PLATFORM_HPP_
