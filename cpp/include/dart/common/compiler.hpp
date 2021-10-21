/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// Note: Don't change the order of clang and gcc because clang has definition of
// __GNUC__ as well.
#if defined(__clang__)
  #define DART_COMPILER_CLANG 1
#elif defined(__GNUC__) || defined(__GNUG__)
  #define DART_COMPILER_GCC 1
#elif defined(_MSC_VER)
  #define DART_COMPILER_MSVC _MSC_VER
#elif defined(__EMSCRIPTEN__)
  #define DART_COMPILER_EMSCRIPTEN 1
#elif defined(__llvm__)
  #define DART_COMPILER_LLVM 1
#elif defined(__INTEL_COMPILER)
  #define DART_COMPILER_ICC 1
#elif defined(__MINGW32__)
  #define DART_COMPILER_MINGW32 1
#elif defined(__SUNPRO_CC)
  #define DART_COMPILER_SUNPRO 1
#elif defined(__IBMCPP__)
  #define DART_COMPILER_IBM 1
#elif defined(__PGI)
  #define DART_COMPILER_PGI 1
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
  #define DART_COMPILER_ARM 1
#else
  #error Unknown compiler
#endif

// Define undefined preprocessors as 0
#ifndef DART_COMPILER_CLANG
  #define DART_COMPILER_CLANG 0
#endif

#ifndef DART_COMPILER_GCC
  #define DART_COMPILER_GCC 0
#endif

#ifndef DART_COMPILER_MSVC
  #define DART_COMPILER_MSVC 0
#endif

#ifndef DART_COMPILER_EMSCRIPTEN
  #define DART_COMPILER_EMSCRIPTEN 0
#endif

#ifndef DART_COMPILER_LLVM
  #define DART_COMPILER_LLVM 0
#endif

#ifndef DART_COMPILER_ICC
  #define DART_COMPILER_ICC 0
#endif

#ifndef DART_COMPILER_MINGW32
  #define DART_COMPILER_MINGW32 0
#endif

#ifndef DART_COMPILER_SUNPRO
  #define DART_COMPILER_SUNPRO 0
#endif

#ifndef DART_COMPILER_IBM
  #define DART_COMPILER_IBM 0
#endif

#ifndef DART_COMPILER_PGI
  #define DART_COMPILER_PGI 0
#endif

#ifndef DART_COMPILER_ARM
  #define DART_COMPILER_ARM 0
#endif

#define DART_GCC_VERSION_GE(x, y, z)                                           \
  __GNUC__                                                                     \
  &&(__GNUC__ > x                                                              \
     || (__GNUC__ == x                                                         \
         && (__GNUC_MINOR__ > y                                                \
             || (__GNUC_MINOR__ == y && __GNUC_PATCHLEVEL__ >= z))))
