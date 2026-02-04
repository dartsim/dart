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

#include <dart/export.hpp>

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
class DART_API Profiler
{
public:
  static Profiler& instance();

  void markFrame();

  /// Dump a summary suitable for quick textual review (hotspots + per-thread
  /// tree).
  void printSummary(std::ostream& os = std::cout);

  /// Clear collected statistics while keeping thread registrations intact.
  void reset();

private:
  struct ProfileNode;
  struct ActiveScope;
  struct ThreadRecord;
  struct Flattened;

  friend class ProfileScope;
  friend class ProfilerTestAccess;

  static constexpr std::uint64_t kUnsetDuration
      = std::numeric_limits<std::uint64_t>::max();

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
      ProfileNode& parent, std::string_view label, std::string_view source);
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

  void printNode(
      std::ostream& os,
      const ProfileNode& node,
      std::uint64_t threadTotalNs,
      const std::string& indent,
      double minPercent,
      std::size_t labelWidth) const;

  std::mutex m_threadRegistryMutex;
  std::vector<std::shared_ptr<ThreadRecord>> m_threads;
  std::atomic<std::uint64_t> m_frameCount;
  std::atomic<std::uint64_t> m_frameTimeSumNs;
  std::vector<std::uint64_t> m_frameSamplesNs;
  std::chrono::steady_clock::time_point m_lastFrameTime{};
};

class DART_API ProfileScope
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
