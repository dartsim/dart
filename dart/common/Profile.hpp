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
// DART_PROFILE_* macros compile to nothing unless DART_BUILD_PROFILE is on.
// When enabled, two backends can be active independently:
//   * a built-in text backend (DART_PROFILE_ENABLE_TEXT, default 1) that needs
//     no GUI and no special kernel permissions -- ideal for headless,
//     evidence-based performance work, and
//   * the optional Tracy GUI backend (DART_PROFILE_ENABLE_TRACY, only used if
//     the Tracy headers are available).
// The selector macros default sensibly and can be overridden by compile
// definitions wired from CMake (see DART_PROFILE_BUILTIN / DART_PROFILE_TRACY).

#include <dart/config.hpp>

#include <atomic>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>

#include <cstdint>

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

    #if defined(TRACY_HAS_CALLSTACK) && defined(TRACY_CALLSTACK)
      #define DART_PROFILE_TRACY_CALLSTACK_ARG , TRACY_CALLSTACK
    #else
      #define DART_PROFILE_TRACY_CALLSTACK_ARG
    #endif
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
  #else
    #define DART_PROFILE_TRACY_FRAME
  #endif

  #if DART_PROFILE_ENABLE_TEXT
    #define DART_PROFILE_TEXT_FRAME                                            \
      ::dart::common::profile::Profiler::instance().markFrame()
    #define DART_PROFILE_TEXT_SCOPED(name_literal)                             \
      ::dart::common::profile::ProfileScope DART_PROFILE_SCOPE_NAME(           \
          _dart_profile_scope_)(name_literal, __FILE__, __LINE__)
    #define DART_PROFILE_TEXT_SCOPED_F()                                       \
      ::dart::common::profile::ProfileScope DART_PROFILE_SCOPE_NAME(           \
          _dart_profile_scope_func_)(__func__, __FILE__, __LINE__)
    #define DART_PROFILE_TEXT_DUMP()                                           \
      ::dart::common::profile::Profiler::instance().printSummary()
    #define DART_PROFILE_TEXT_SUMMARY()                                        \
      ::dart::common::profile::Profiler::instance().toSummaryText()
    #define DART_PROFILE_TEXT_COUNTER(name_literal, value)                     \
      do {                                                                     \
        ::dart::common::profile::Profiler::instance().recordCounter(           \
            name_literal,                                                      \
            __FILE__,                                                          \
            __LINE__,                                                          \
            static_cast<::std::uint64_t>(value));                              \
      } while (false)
  #else
    #define DART_PROFILE_TEXT_FRAME
    #define DART_PROFILE_TEXT_SCOPED(name_literal)
    #define DART_PROFILE_TEXT_SCOPED_F()
    #define DART_PROFILE_TEXT_DUMP()
    #define DART_PROFILE_TEXT_SUMMARY()                                        \
      ::std::string {}
    #define DART_PROFILE_TEXT_COUNTER(name_literal, value)                     \
      do {                                                                     \
        (void)sizeof(value);                                                   \
      } while (false)
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
        std::strlen(name) DART_PROFILE_TRACY_CALLSTACK_ARG,
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
        std::strlen(name) DART_PROFILE_TRACY_CALLSTACK_ARG,
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

class ConditionalScopedProfile
{
public:
  ConditionalScopedProfile(
      bool enabled,
      const char* name,
      const char* file,
      int line,
      const char* function)
  {
    if (enabled)
      mScope.emplace(name, file, line, function);
  }

  ConditionalScopedProfile(const ConditionalScopedProfile&) = delete;
  ConditionalScopedProfile& operator=(const ConditionalScopedProfile&) = delete;

private:
  std::optional<ScopedProfile> mScope;
};

} // namespace dart::common::profile::detail

    #if DART_PROFILE_HAS_TRACY
      #undef DART_PROFILE_TRACY_CALLSTACK_ARG
    #endif

  #endif

  #define DART_PROFILE_FRAME                                                   \
    do {                                                                       \
      DART_PROFILE_TRACY_FRAME;                                                \
      DART_PROFILE_TEXT_FRAME;                                                 \
    } while (false)

  #define DART_PROFILE_COUNTER_N(name_literal, value)                          \
    DART_PROFILE_TEXT_COUNTER(name_literal, value)
  #define DART_PROFILE_COUNTER_IF_N(enabled, name_literal, value)              \
    do {                                                                       \
      if (enabled)                                                             \
        DART_PROFILE_TEXT_COUNTER(name_literal, value);                        \
    } while (false)

  #if DART_PROFILE_HAS_TRACY || DART_PROFILE_ENABLE_TEXT

    #define DART_PROFILE_SCOPED                                                \
      ::dart::common::profile::detail::ScopedProfile DART_PROFILE_SCOPE_NAME(  \
          _dart_profile_scope_)(__func__, __FILE__, __LINE__, __func__)

    #define DART_PROFILE_SCOPED_N(name_literal)                                \
      ::dart::common::profile::detail::ScopedProfile DART_PROFILE_SCOPE_NAME(  \
          _dart_profile_scope_)(name_literal, __FILE__, __LINE__, __func__)

    #define DART_PROFILE_SCOPED_IF_N(enabled, name_literal)                    \
      ::dart::common::profile::detail::ConditionalScopedProfile                \
      DART_PROFILE_SCOPE_NAME(_dart_profile_scope_)(                           \
          enabled, name_literal, __FILE__, __LINE__, __func__)

  #else

    #define DART_PROFILE_SCOPED
    #define DART_PROFILE_SCOPED_N(name_literal)
    #define DART_PROFILE_SCOPED_IF_N(enabled, name_literal)                    \
      do {                                                                     \
        (void)sizeof(enabled);                                                 \
      } while (false)

  #endif

#else // no-op

  #define DART_PROFILE_FRAME
  #define DART_PROFILE_SCOPED
  #define DART_PROFILE_SCOPED_N(name_literal)
  #define DART_PROFILE_SCOPED_IF_N(enabled, name_literal)                      \
    do {                                                                       \
      (void)sizeof(enabled);                                                   \
    } while (false)
  #define DART_PROFILE_COUNTER_N(name_literal, value)                          \
    do {                                                                       \
      (void)sizeof(value);                                                     \
    } while (false)
  #define DART_PROFILE_COUNTER_IF_N(enabled, name_literal, value)              \
    do {                                                                       \
      (void)sizeof(enabled);                                                   \
      (void)sizeof(value);                                                     \
    } while (false)
  #define DART_PROFILE_TEXT_DUMP()
  #define DART_PROFILE_TEXT_SUMMARY()                                          \
    ::std::string {}
  #define DART_PROFILE_TEXT_COUNTER(name_literal, value)                       \
    do {                                                                       \
      (void)sizeof(value);                                                     \
    } while (false)

#endif

namespace dart::common::profile {

namespace detail {

#if DART_BUILD_PROFILE && !DART_PROFILE_ENABLE_TEXT && DART_PROFILE_HAS_TRACY

[[nodiscard]] inline std::atomic<bool>& profileRecordingEnabledFlag() noexcept
{
  static std::atomic<bool> enabled{false};
  return enabled;
}

#endif

} // namespace detail

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

/// Return whether conditional profile instrumentation is currently recording.
[[nodiscard]] inline bool isProfileRecordingEnabled() noexcept
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  return Profiler::instance().isRecordingEnabled();
#elif DART_BUILD_PROFILE && DART_PROFILE_HAS_TRACY
  return detail::profileRecordingEnabledFlag().load(std::memory_order_relaxed);
#else
  return false;
#endif
}

/// Enable or disable conditional profile instrumentation and return the
/// previous state.
inline bool setProfileRecordingEnabled(bool enabled) noexcept
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  return Profiler::instance().setRecordingEnabled(enabled);
#elif DART_BUILD_PROFILE && DART_PROFILE_HAS_TRACY
  return detail::profileRecordingEnabledFlag().exchange(
      enabled, std::memory_order_relaxed);
#else
  (void)enabled;
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

/// Record an integer counter sample in the built-in text profiler.
inline void recordProfileCounter(
    std::string_view label,
    std::uint64_t value,
    std::string_view file,
    int line)
{
#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  Profiler::instance().recordCounter(label, file, line, value);
#else
  (void)label;
  (void)value;
  (void)file;
  (void)line;
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
