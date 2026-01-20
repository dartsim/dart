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

#include <dart/config.hpp>

#if DART_BUILD_PROFILE

  //-------------------------------------------------------------------------
  // Backend selection
  //-------------------------------------------------------------------------

  // Defaults can be overridden via compile definitions or CMake options:
  // -D DART_PROFILE_ENABLE_TEXT=0 to disable the text backend
  // -D DART_PROFILE_ENABLE_TRACY=0 to skip Tracy (even if headers are present)
  #if !defined(DART_PROFILE_ENABLE_TEXT)
    #define DART_PROFILE_ENABLE_TEXT 1
  #endif
  #if !defined(DART_PROFILE_ENABLE_TRACY)
    #define DART_PROFILE_ENABLE_TRACY 1
  #endif

  // Tracy (optional GUI backend)
  #if DART_PROFILE_ENABLE_TRACY
    #if defined(__has_include)
      #if __has_include(<tracy/Tracy.hpp>)
        #define DART_PROFILE_HAS_TRACY 1
      #else
        #define DART_PROFILE_HAS_TRACY 0
      #endif
    #else
      #define DART_PROFILE_HAS_TRACY 1
    #endif
  #else
    #define DART_PROFILE_HAS_TRACY 0
  #endif

  #if DART_PROFILE_HAS_TRACY
    #include <tracy/Tracy.hpp>
  #endif

  // Built-in text backend (readable console summaries)
  #if DART_PROFILE_ENABLE_TEXT
    #include <dart/common/detail/profiler.hpp>
  #endif

  #if defined(__COUNTER__)
    #define DART_PROFILE_UNIQUE_ID __COUNTER__
  #else
    #define DART_PROFILE_UNIQUE_ID __LINE__
  #endif

  #define DART_PROFILE_CONCAT_IMPL(a, b) a##b
  #define DART_PROFILE_CONCAT(a, b) DART_PROFILE_CONCAT_IMPL(a, b)
  #define DART_PROFILE_SCOPE_NAME(prefix)                                      \
    DART_PROFILE_CONCAT(prefix, DART_PROFILE_UNIQUE_ID)

  #if DART_PROFILE_HAS_TRACY
    #define DART_PROFILE_TRACY_FRAME FrameMark
    #define DART_PROFILE_TRACY_SCOPED ZoneScoped
    #define DART_PROFILE_TRACY_SCOPED_N(name_literal) ZoneScopedN(name_literal)
  #else
    #define DART_PROFILE_TRACY_FRAME
    #define DART_PROFILE_TRACY_SCOPED
    #define DART_PROFILE_TRACY_SCOPED_N(name_literal)
  #endif

  #if DART_PROFILE_ENABLE_TEXT
    #define DART_PROFILE_TEXT_FRAME                                            \
      ::dart::common::profile::Profiler::instance().markFrame()
    #define DART_PROFILE_TEXT_SCOPED(name_literal)                             \
      ::dart::common::profile::ProfileScope DART_PROFILE_SCOPE_NAME(           \
          _dart_profile_scope_)(                                               \
          name_literal, __FILE__, static_cast<int>(__LINE__))
    #define DART_PROFILE_TEXT_SCOPED_F()                                       \
      ::dart::common::profile::ProfileScope DART_PROFILE_SCOPE_NAME(           \
          _dart_profile_scope_func_)(                                          \
          __func__, __FILE__, static_cast<int>(__LINE__))
    #define DART_PROFILE_TEXT_DUMP()                                           \
      ::dart::common::profile::Profiler::instance().printSummary()
  #else
    #define DART_PROFILE_TEXT_FRAME
    #define DART_PROFILE_TEXT_SCOPED(name_literal)
    #define DART_PROFILE_TEXT_SCOPED_F()
    #define DART_PROFILE_TEXT_DUMP()
  #endif

  #define DART_PROFILE_FRAME                                                   \
    do {                                                                       \
      DART_PROFILE_TRACY_FRAME;                                                \
      DART_PROFILE_TEXT_FRAME;                                                 \
    } while (false)

  #define DART_PROFILE_SCOPED                                                  \
    do {                                                                       \
      DART_PROFILE_TRACY_SCOPED;                                               \
      DART_PROFILE_TEXT_SCOPED_F();                                            \
    } while (false)

  #define DART_PROFILE_SCOPED_N(name_literal)                                  \
    do {                                                                       \
      DART_PROFILE_TRACY_SCOPED_N(name_literal);                               \
      DART_PROFILE_TEXT_SCOPED(name_literal);                                  \
    } while (false)

#else // no-op

  #define DART_PROFILE_FRAME
  #define DART_PROFILE_SCOPED
  #define DART_PROFILE_SCOPED_N(name_literal)
  #define DART_PROFILE_TEXT_DUMP()

#endif
