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

// Hierarchical, dependency-free text profiler backported from DART 7
// (dart/common/detail/profiler.hpp). It captures scoped timings into a
// per-thread call tree and prints an easy-to-skim, hotspot-focused summary for
// headless, text-based inspection -- no GUI and no special kernel permissions
// required. DART 6 targets C++17, so the DART 7 C++20 conveniences
// (std::source_location, std::ranges) are intentionally omitted here; the
// macros in dart/common/Profile.hpp always pass file/line explicitly.

#include <atomic>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace dart::common::profile {

/// Hierarchical text profiler: captures scoped timings and prints an
/// easy-to-skim, hotspot-focused summary for text-based inspection.
/// Collection is thread-local; call printSummary() after worker threads
/// settle to avoid reading while they are still updating their stacks.
class Profiler
{
public:
  static Profiler& instance();

  void markFrame();

  /// Enable or disable runtime recording for conditional instrumentation.
  /// Returns the previous state.
  bool setRecordingEnabled(bool enabled) noexcept;

  /// Return whether conditional instrumentation should record samples.
  [[nodiscard]] bool isRecordingEnabled() const noexcept;

  /// Record an integer counter sample in the current thread's profiler record.
  void recordCounter(
      std::string_view label,
      std::string_view file,
      int line,
      std::uint64_t value);

  /// Dump a summary suitable for quick textual review (hotspots + per-thread
  /// tree).
  void printSummary(std::ostream& os = std::cout);

  /// Return the same summary text produced by printSummary().
  [[nodiscard]] std::string toSummaryText();

  /// Clear collected statistics while keeping thread registrations intact.
  void reset();

private:
  struct ProfileNode;
  struct CounterNode;
  struct ActiveScope;
  struct ThreadRecord;
  struct Flattened;

  friend class ProfileScope;
  friend class ProfilerTestAccess;

  static constexpr std::uint64_t kUnsetDuration
      = std::numeric_limits<std::uint64_t>::max();
  static constexpr std::size_t kMaxFrameSamples = 4096;
  static constexpr std::size_t kInitialThreadNodeCapacity = 4096;

  Profiler();

  static std::shared_ptr<ThreadRecord> threadRecord();

  std::shared_ptr<ThreadRecord> registerThread();
  void pushScope(
      ThreadRecord& record,
      std::string_view label,
      std::string_view file,
      int line);
  void popScope(ThreadRecord& record);

  ProfileNode* findOrCreateChild(
      ThreadRecord& record,
      ProfileNode& parent,
      std::string_view label,
      std::string_view file,
      int line);
  ProfileNode* allocateNode(
      ThreadRecord& record,
      std::string_view label,
      std::string_view file,
      int line);
  CounterNode* findOrCreateCounter(
      ThreadRecord& record,
      std::string_view label,
      std::string_view file,
      int line);
  static std::string sourceText(const ProfileNode& node);
  static std::string sourceText(const CounterNode& node);
  static std::string padRight(std::string_view text, std::size_t width);
  static bool useColor();
  static std::string colorize(std::string_view text, const char* code);
  static const char* heatColor(double pct);
  static std::string formatDurationAligned(std::uint64_t ns);
  static std::string formatFps(double fps);
  static std::string formatCount(std::uint64_t v);
  static std::size_t maxLabelWidth(
      const ProfileNode& node, std::size_t minWidth, std::size_t maxWidth);

  static std::uint64_t sumInclusiveChildren(const ProfileNode& node);
  static bool hasRecordedScopes(const ProfileNode& node);
  static bool hasRecordedCounters(const ThreadRecord& record);
  static void clearNode(ProfileNode& node);
  static std::string formatDuration(std::uint64_t ns);
  static std::string formatPercent(double pct);
  static double percentage(std::uint64_t part, std::uint64_t total);

  void collectHotspots(
      const ProfileNode& node,
      const std::string& path,
      const std::string& threadLabel,
      std::vector<Flattened>& out) const;

  void printThreadTree(
      std::ostream& os,
      const ThreadRecord& record,
      std::uint64_t threadTotalNs,
      double minPercent,
      std::size_t labelWidth) const;

  void printThreadCounters(std::ostream& os, const ThreadRecord& record) const;

  void printNode(
      std::ostream& os,
      const ProfileNode& node,
      std::uint64_t threadTotalNs,
      const std::string& indent,
      double minPercent,
      std::size_t labelWidth) const;

  std::mutex m_threadRegistryMutex;
  std::vector<std::shared_ptr<ThreadRecord>> m_threads;
  std::atomic<bool> m_recordingEnabled;
  std::atomic<std::uint64_t> m_frameCount;
  std::atomic<std::uint64_t> m_frameTimeSumNs;
  std::vector<std::uint64_t> m_frameSamplesNs;
  std::size_t m_frameSampleCursor{0};
  std::chrono::steady_clock::time_point m_lastFrameTime{};
};

class ProfileScope
{
public:
  explicit ProfileScope(std::string_view name, std::string_view file, int line);
  ~ProfileScope();

  ProfileScope(const ProfileScope&) = delete;
  ProfileScope& operator=(const ProfileScope&) = delete;

private:
  std::shared_ptr<Profiler::ThreadRecord> m_record;
};

} // namespace dart::common::profile
