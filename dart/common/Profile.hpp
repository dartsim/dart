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

// DART 6 profiling front-end, backported from DART 7 (dart/common/profile.hpp).
//
// DART_PROFILE_* macros compile to nothing unless DART_BUILD_PROFILE is on. When
// enabled, two backends can be active independently:
//   * a built-in text backend (DART_PROFILE_ENABLE_TEXT, default 1) that needs
//     no GUI and no special kernel permissions -- ideal for headless,
//     evidence-based performance work, and
//   * the optional Tracy GUI backend (DART_PROFILE_ENABLE_TRACY, only used if
//     the Tracy headers are available).
// The selector macros default sensibly and can be overridden by compile
// definitions wired from CMake (see DART_PROFILE_BUILTIN / DART_PROFILE_TRACY).

#include <dart/config.hpp>

#include <iosfwd>
#include <string>

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

    #include <cstdint>
    #include <cstring>
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
  #define DART_PROFILE_SCOPE_NAME(prefix)                                       \
    DART_PROFILE_CONCAT(prefix, DART_PROFILE_UNIQUE_ID)

  #if DART_PROFILE_HAS_TRACY
    #define DART_PROFILE_TRACY_FRAME FrameMark
  #else
    #define DART_PROFILE_TRACY_FRAME
  #endif

  #if DART_PROFILE_ENABLE_TEXT
    #define DART_PROFILE_TEXT_FRAME                                             \
      ::dart::common::profile::Profiler::instance().markFrame()
    #define DART_PROFILE_TEXT_SCOPED(name_literal)                             \
      ::dart::common::profile::ProfileScope DART_PROFILE_SCOPE_NAME(            \
          _dart_profile_scope_)(name_literal, __FILE__, __LINE__)
    #define DART_PROFILE_TEXT_SCOPED_F()                                        \
      ::dart::common::profile::ProfileScope DART_PROFILE_SCOPE_NAME(            \
          _dart_profile_scope_func_)(__func__, __FILE__, __LINE__)
    #define DART_PROFILE_TEXT_DUMP()                                            \
      ::dart::common::profile::Profiler::instance().printSummary()
    #define DART_PROFILE_TEXT_SUMMARY()                                         \
      ::dart::common::profile::Profiler::instance().toSummaryText()
  #else
    #define DART_PROFILE_TEXT_FRAME
    #define DART_PROFILE_TEXT_SCOPED(name_literal)
    #define DART_PROFILE_TEXT_SCOPED_F()
    #define DART_PROFILE_TEXT_DUMP()
    #define DART_PROFILE_TEXT_SUMMARY()                                         \
      ::std::string {}
  #endif

  #if DART_PROFILE_HAS_TRACY || DART_PROFILE_ENABLE_TEXT

namespace dart::common::profile::detail {

class ScopedProfile
{
public:
  ScopedProfile(
      const char* name, const char* file, int line, const char* function)
    #if DART_PROFILE_HAS_TRACY && DART_PROFILE_ENABLE_TEXT
    : m_tracy(
          static_cast<std::uint32_t>(line),
          file,
          std::strlen(file),
          function,
          std::strlen(function),
          name,
          std::strlen(name),
          TRACY_CALLSTACK,
          true),
      m_text(name, file, line)
    #elif DART_PROFILE_HAS_TRACY
    : m_tracy(
          static_cast<std::uint32_t>(line),
          file,
          std::strlen(file),
          function,
          std::strlen(function),
          name,
          std::strlen(name),
          TRACY_CALLSTACK,
          true)
    #elif DART_PROFILE_ENABLE_TEXT
    : m_text(name, file, line)
    #endif
  {
    #if !DART_PROFILE_HAS_TRACY
    (void)function;
    #endif
  }

  ScopedProfile(const ScopedProfile&) = delete;
  ScopedProfile& operator=(const ScopedProfile&) = delete;

private:
    #if DART_PROFILE_HAS_TRACY
  tracy::ScopedZone m_tracy;
    #endif
    #if DART_PROFILE_ENABLE_TEXT
  ProfileScope m_text;
    #endif
};

} // namespace dart::common::profile::detail

  #endif

  #define DART_PROFILE_FRAME                                                    \
    do {                                                                        \
      DART_PROFILE_TRACY_FRAME;                                                 \
      DART_PROFILE_TEXT_FRAME;                                                  \
    } while (false)

  #if DART_PROFILE_HAS_TRACY || DART_PROFILE_ENABLE_TEXT

    #define DART_PROFILE_SCOPED                                                 \
      ::dart::common::profile::detail::ScopedProfile DART_PROFILE_SCOPE_NAME(   \
          _dart_profile_scope_)(__func__, __FILE__, __LINE__, __func__)

    #define DART_PROFILE_SCOPED_N(name_literal)                                \
      ::dart::common::profile::detail::ScopedProfile DART_PROFILE_SCOPE_NAME(   \
          _dart_profile_scope_)(name_literal, __FILE__, __LINE__, __func__)

  #else

    #define DART_PROFILE_SCOPED
    #define DART_PROFILE_SCOPED_N(name_literal)

  #endif

#else // no-op

  #define DART_PROFILE_FRAME
  #define DART_PROFILE_SCOPED
  #define DART_PROFILE_SCOPED_N(name_literal)
  #define DART_PROFILE_TEXT_DUMP()
  #define DART_PROFILE_TEXT_SUMMARY()                                           \
    ::std::string {}

#endif

namespace dart::common::profile {

/// Return whether DART was built with profiling support.
[[nodiscard]] constexpr bool isProfilingEnabled() noexcept
{
#if DART_BUILD_PROFILE
  return true;
#else
  return false;
#endif
}

/// Return whether the built-in text profiling backend is available.
[[nodiscard]] constexpr bool isTextProfilingEnabled() noexcept
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  return true;
#else
  return false;
#endif
}

/// Mark a frame in the built-in text profiler.
inline void markProfileFrame()
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  Profiler::instance().markFrame();
#endif
}

/// Clear the built-in text profiler state.
inline void resetProfile()
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  Profiler::instance().reset();
#endif
}

/// Write the built-in text profiler summary to a stream.
inline void printProfileSummary(std::ostream& os)
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  Profiler::instance().printSummary(os);
#else
  (void)os;
#endif
}

/// Return the built-in text profiler summary.
[[nodiscard]] inline std::string getProfileSummaryText()
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  return Profiler::instance().toSummaryText();
#else
  return {};
#endif
}

} // namespace dart::common::profile
