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

#include <dart/simulation/next/export.hpp>

#include <chrono>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>

namespace dart::simulation::next::common {

/// Simple RAII timer for profiling
class DART8_API ScopedTimer
{
public:
  explicit ScopedTimer(std::string_view name)
    : m_name(name), m_start(std::chrono::high_resolution_clock::now())
  {
  }

  ~ScopedTimer()
  {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration
        = std::chrono::duration_cast<std::chrono::microseconds>(end - m_start);
    reportDuration(m_name, duration.count());
  }

  /// Get elapsed time in microseconds
  [[nodiscard]] long long elapsed() const
  {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - m_start)
        .count();
  }

private:
  static void reportDuration(std::string_view name, long long microseconds);

  std::string m_name;
  std::chrono::high_resolution_clock::time_point m_start;
};

/// Simple stopwatch for manual timing measurements
///
/// Example usage:
/// @code
///   Stopwatch sw;
///   sw.tic();
///   // ... code to measure ...
///   double elapsed_ms = sw.tock();
///   std::cout << "Elapsed: " << elapsed_ms << " ms\n";
/// @endcode
class DART8_API Stopwatch
{
public:
  Stopwatch() = default;

  /// Start timing
  void tic()
  {
    m_start = std::chrono::high_resolution_clock::now();
  }

  /// Stop timing and return elapsed time in milliseconds
  [[nodiscard]] double tock() const
  {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration
        = std::chrono::duration_cast<std::chrono::nanoseconds>(end - m_start);
    return duration.count() / 1e6; // Convert to milliseconds
  }

  /// Get elapsed time in milliseconds without stopping
  [[nodiscard]] double elapsed_ms() const
  {
    return tock();
  }

  /// Get elapsed time in seconds without stopping
  [[nodiscard]] double elapsed_sec() const
  {
    return tock() / 1000.0;
  }

  /// Get elapsed time in microseconds without stopping
  [[nodiscard]] long long elapsed_us() const
  {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - m_start)
        .count();
  }

private:
  std::chrono::high_resolution_clock::time_point m_start;
};

/// Profile statistics aggregator
class DART8_API ProfileStats
{
public:
  struct Entry
  {
    std::string name;
    long long totalMicroseconds{0};
    long long count{0};
    long long minMicroseconds{std::numeric_limits<long long>::max()};
    long long maxMicroseconds{0};

    [[nodiscard]] double averageMicroseconds() const
    {
      return count > 0 ? static_cast<double>(totalMicroseconds) / count : 0.0;
    }
  };

  /// Record a timing measurement
  static void record(std::string_view name, long long microseconds);

  /// Get all profile entries
  static const std::unordered_map<std::string, Entry>& entries();

  /// Reset all statistics
  static void reset();

  /// Print summary to stdout
  static void printSummary();

private:
  static std::unordered_map<std::string, Entry>& entriesInternal();
};

} // namespace dart::simulation::next::common

//===============================================================================
// Profiling Backend Selection
//===============================================================================

// DART8 supports two profiling backends:
// 1. Built-in profiling (controlled by DART8_ENABLE_PROFILING)
// 2. Tracy Profiler (controlled by TRACY_ENABLE)
//
// You can enable one, both, or none:
// - DART8_ENABLE_PROFILING: Enables built-in lightweight profiling
// - TRACY_ENABLE: Enables Tracy profiler integration (requires Tracy library)

#ifdef TRACY_ENABLE
  #include <tracy/Tracy.hpp>
#endif

// Profiling macros

#if defined(DART8_ENABLE_PROFILING) && !defined(TRACY_ENABLE)
// Built-in profiling only

  #define DART8_PROFILE_SCOPE(name)                                            \
    ::dart::simulation::next::common::ScopedTimer _dart_profile_timer##__LINE__(name)

  #define DART8_PROFILE_FUNCTION() DART8_PROFILE_SCOPE(__PRETTY_FUNCTION__)

  #define DART8_PROFILE_BEGIN(name)                                            \
    auto _dart_profile_start_##name = std::chrono::high_resolution_clock::now()

  #define DART8_PROFILE_END(name)                                              \
    do {                                                                       \
      auto _dart_profile_end = std::chrono::high_resolution_clock::now();      \
      auto _dart_profile_duration                                              \
          = std::chrono::duration_cast<std::chrono::microseconds>(             \
              _dart_profile_end - _dart_profile_start_##name);                 \
      ::dart::simulation::next::common::ProfileStats::record(                                   \
          #name, _dart_profile_duration.count());                              \
    } while (false)

  #define DART8_PROFILE_FRAME() ((void)0)

#elif defined(TRACY_ENABLE)
// Tracy profiling (with or without built-in profiling)

  #define DART8_PROFILE_SCOPE(name) ZoneScopedN(name)
  #define DART8_PROFILE_FUNCTION() ZoneScoped
  #define DART8_PROFILE_FRAME() FrameMark

  // For manual begin/end, Tracy uses a different approach
  #define DART8_PROFILE_BEGIN(name)                                            \
    [[maybe_unused]] auto _dart_tracy_zone_##name = tracy::ScopedZone(         \
        __LINE__, __FILE__, strlen(__FILE__), #name, strlen(#name))

  #define DART8_PROFILE_END(name) ((void)0) // Tracy zones end automatically

  // If both are enabled, also record in built-in profiler
  #ifdef DART8_ENABLE_PROFILING
    #define DART8_PROFILE_SCOPE_DUAL(name)                                     \
      ZoneScopedN(name);                                                       \
      ::dart::simulation::next::common::ScopedTimer _dart_profile_timer##__LINE__(name)
  #else
    #define DART8_PROFILE_SCOPE_DUAL(name) DART8_PROFILE_SCOPE(name)
  #endif

#else
// No profiling - all no-ops

  #define DART8_PROFILE_SCOPE(name) ((void)0)
  #define DART8_PROFILE_FUNCTION() ((void)0)
  #define DART8_PROFILE_BEGIN(name) ((void)0)
  #define DART8_PROFILE_END(name) ((void)0)
  #define DART8_PROFILE_FRAME() ((void)0)
  #define DART8_PROFILE_SCOPE_DUAL(name) ((void)0)

#endif
