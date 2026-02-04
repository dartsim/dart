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

#include <dart/common/platform.hpp>

#ifndef DART_SYMBOL_EXPORT
  #if DART_OS_WINDOWS
    #define DART_SYMBOL_EXPORT __declspec(dllexport)
  #elif defined(__GNUC__) && (__GNUC__ >= 4)
    #define DART_SYMBOL_EXPORT __attribute__((visibility("default")))
  #else
    #define DART_SYMBOL_EXPORT
  #endif
#endif

#ifndef DART_SYMBOL_IMPORT
  #if DART_OS_WINDOWS
    #define DART_SYMBOL_IMPORT __declspec(dllimport)
  #elif defined(__GNUC__) && (__GNUC__ >= 4)
    #define DART_SYMBOL_IMPORT __attribute__((visibility("default")))
  #else
    #define DART_SYMBOL_IMPORT
  #endif
#endif

#ifndef DART_SYMBOL_LOCAL
  #if DART_OS_WINDOWS
    #define DART_SYMBOL_LOCAL
  #elif defined(__GNUC__) && (__GNUC__ >= 4)
    #define DART_SYMBOL_LOCAL __attribute__((visibility("hidden")))
  #else
    #define DART_SYMBOL_LOCAL
  #endif
#endif

#ifndef DART_BUILD_SHARED
  #define DART_BUILD_SHARED 0
#endif

#ifndef DART_DLL_EXPORT
  #if DART_BUILD_SHARED
    #define DART_DLL_EXPORT DART_SYMBOL_EXPORT
    #define DART_DLL_IMPORT DART_SYMBOL_IMPORT
    #define DART_DLL_LOCAL DART_SYMBOL_LOCAL
  #else
    #define DART_DLL_EXPORT
    #define DART_DLL_IMPORT
    #define DART_DLL_LOCAL
  #endif
#endif

#ifndef DART_DLL_IMPORT
  #define DART_DLL_IMPORT
#endif

#ifndef DART_DLL_LOCAL
  #define DART_DLL_LOCAL
#endif
