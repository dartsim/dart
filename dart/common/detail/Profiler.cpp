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

#include "dart/common/detail/profiler.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace dart::common::profile {

struct Profiler::ProfileNode
{
  std::string label;
  std::string source;
  std::uint64_t callCount{0};
  std::uint64_t inclusiveNs{0};
  std::uint64_t selfNs{0};
  std::uint64_t minNs{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t maxNs{0};
  std::unordered_map<std::string, std::unique_ptr<ProfileNode>> children;
};

struct Profiler::ActiveScope
{
  ProfileNode* node{nullptr};
  std::chrono::steady_clock::time_point start{};
  std::uint64_t childTimeNs{0};
};

struct Profiler::ThreadRecord
{
  std::string label;
  std::thread::id id;
  ProfileNode root{};
  std::vector<ActiveScope> stack;
};

struct Profiler::Flattened
{
  std::string path;
  std::string threadLabel;
  std::string source;
  std::uint64_t inclusiveNs{0};
  std::uint64_t selfNs{0};
  std::uint64_t callCount{0};
};

Profiler::Profiler() : m_frameCount(0), m_frameTimeSumNs(0) {}

Profiler& Profiler::instance()
{
  static Profiler profiler;
  return profiler;
}

std::shared_ptr<Profiler::ThreadRecord> Profiler::threadRecord()
{
  thread_local std::shared_ptr<ThreadRecord> record
      = Profiler::instance().registerThread();
  return record;
}

std::shared_ptr<Profiler::ThreadRecord> Profiler::registerThread()
{
  auto record = std::make_shared<ThreadRecord>();
  record->id = std::this_thread::get_id();
  {
    std::ostringstream oss;
    oss << record->id;
    record->label = oss.str();
    record->root.label = "thread " + record->label;
  }

  {
    std::lock_guard<std::mutex> lock(m_threadRegistryMutex);
    m_threads.push_back(record);
  }

  return record;
}

Profiler::ProfileNode* Profiler::findOrCreateChild(
    ProfileNode& parent, std::string_view label, std::string_view source)
{
  const std::string key = std::string(label) + " @ " + std::string(source);
  auto it = parent.children.find(key);
  if (it == parent.children.end()) {
    auto child = std::make_unique<ProfileNode>();
    child->label = std::string(label);
    child->source = std::string(source);
    it = parent.children.emplace(key, std::move(child)).first;
  }
  return it->second.get();
}

void Profiler::pushScope(
    Profiler::ThreadRecord& record,
    std::string_view label,
    std::string_view file,
    int line)
{
  auto* parent = record.stack.empty() ? &record.root : record.stack.back().node;
  const std::string source
      = std::string(file) + ":" + std::to_string(static_cast<long long>(line));
  auto* node = findOrCreateChild(*parent, label, source);
  record.stack.push_back(
      {node, std::chrono::steady_clock::now(), /*childTimeNs=*/0});
}

void Profiler::popScope(Profiler::ThreadRecord& record)
{
  if (record.stack.empty()) {
    return;
  }

  const auto active = record.stack.back();
  record.stack.pop_back();

  const auto end = std::chrono::steady_clock::now();
  const auto durationNs = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - active.start)
          .count());

  auto* node = active.node;
  ++node->callCount;
  node->inclusiveNs += durationNs;

  const auto selfNs
      = durationNs > active.childTimeNs ? durationNs - active.childTimeNs : 0;

  node->selfNs += selfNs;
  node->minNs = std::min(node->minNs, durationNs);
  node->maxNs = std::max(node->maxNs, durationNs);

  if (!record.stack.empty()) {
    record.stack.back().childTimeNs += durationNs;
  }
}

std::uint64_t Profiler::sumInclusiveChildren(const ProfileNode& node)
{
  std::uint64_t total = 0;
  for (const auto& [_, child] : node.children) {
    total += child->inclusiveNs;
  }
  return total;
}

void Profiler::markFrame()
{
  const auto now = std::chrono::steady_clock::now();
  const auto count = m_frameCount.load(std::memory_order_relaxed);
  if (count == 0) {
    m_lastFrameTime = now;
  } else {
    const auto deltaNs = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now - m_lastFrameTime)
            .count());
    m_lastFrameTime = now;
    m_frameTimeSumNs.fetch_add(deltaNs, std::memory_order_relaxed);
    m_frameSamplesNs.push_back(deltaNs);
  }
  m_frameCount.fetch_add(1, std::memory_order_relaxed);
}

std::string Profiler::formatDuration(std::uint64_t ns)
{
  std::ostringstream oss;
  if (ns >= 1'000'000'000ULL) {
    const double s = static_cast<double>(ns) / 1'000'000'000.0;
    oss << std::fixed << std::setprecision(s >= 10.0 ? 1 : 3) << s << " s";
  } else if (ns >= 1'000'000ULL) {
    const double ms = static_cast<double>(ns) / 1'000'000.0;
    oss << std::fixed << std::setprecision(ms >= 10.0 ? 2 : 3) << ms << " ms";
  } else {
    const double us = static_cast<double>(ns) / 1'000.0;
    oss << std::fixed << std::setprecision(us >= 10.0 ? 2 : 3) << us << " µs";
  }
  return oss.str();
}

std::string Profiler::formatDurationAligned(std::uint64_t ns)
{
  std::ostringstream oss;
  if (ns >= 1'000'000'000ULL) {
    const double s = static_cast<double>(ns) / 1'000'000'000.0;
    oss << std::setw(8) << std::fixed << std::setprecision(s >= 10.0 ? 2 : 3)
        << s << " s ";
  } else if (ns >= 1'000'000ULL) {
    const double ms = static_cast<double>(ns) / 1'000'000.0;
    oss << std::setw(8) << std::fixed << std::setprecision(ms >= 10.0 ? 2 : 3)
        << ms << " ms";
  } else if (ns >= 1'000ULL) {
    const double us = static_cast<double>(ns) / 1'000.0;
    oss << std::setw(8) << std::fixed << std::setprecision(us >= 10.0 ? 2 : 3)
        << us << " µs";
  } else {
    oss << std::setw(8) << ns << " ns";
  }
  return oss.str();
}

std::string Profiler::formatFps(double fps)
{
  std::ostringstream oss;
  if (fps >= 1000000.0) {
    oss << std::fixed << std::setprecision(1) << fps / 1000000.0 << "M";
  } else if (fps >= 1000.0) {
    oss << std::fixed << std::setprecision(1) << fps / 1000.0 << "k";
  } else {
    oss << std::fixed << std::setprecision(fps >= 100.0 ? 0 : 1) << fps;
  }
  return oss.str();
}

std::string Profiler::formatCount(std::uint64_t v)
{
  std::ostringstream oss;
  if (v >= 1'000'000'000ULL) {
    oss << std::fixed << std::setprecision(1)
        << static_cast<double>(v) / 1'000'000'000.0 << "B";
  } else if (v >= 1'000'000ULL) {
    oss << std::fixed << std::setprecision(1)
        << static_cast<double>(v) / 1'000'000.0 << "M";
  } else if (v >= 1'000ULL) {
    oss << std::fixed << std::setprecision(1)
        << static_cast<double>(v) / 1'000.0 << "K";
  } else {
    oss << v;
  }
  return oss.str();
}

std::size_t Profiler::maxLabelWidth(
    const ProfileNode& node, std::size_t minWidth, std::size_t maxWidth)
{
  std::size_t width = node.label.size();
  for (const auto& [_, child] : node.children) {
    width = std::max(width, maxLabelWidth(*child, minWidth, maxWidth));
  }
  return std::clamp<std::size_t>(width, minWidth, maxWidth);
}

double Profiler::percentage(std::uint64_t part, std::uint64_t total)
{
  if (total == 0) {
    return 0.0;
  }
  return (static_cast<double>(part) / static_cast<double>(total)) * 100.0;
}

std::string Profiler::formatPercent(double pct)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(pct >= 10.0 ? 1 : 2) << pct << '%';
  return oss.str();
}

std::string Profiler::padRight(std::string_view text, std::size_t width)
{
  if (text.size() >= width) {
    return std::string(text);
  }
  std::string padded;
  padded.reserve(width);
  padded.append(text);
  padded.append(width - text.size(), ' ');
  return padded;
}

bool Profiler::useColor()
{
  const char* env = std::getenv("DART_PROFILE_COLOR");
  if (!env) {
    return true; // enabled by default
  }
  std::string val(env);
  for (auto& c : val) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return (val == "1" || val == "ON" || val == "TRUE" || val == "YES");
}

std::string Profiler::colorize(std::string_view text, const char* code)
{
  if (!useColor()) {
    return std::string(text);
  }
  return std::string(code) + std::string(text) + "\033[0m";
}

const char* Profiler::heatColor(double pct)
{
  if (!useColor()) {
    return "";
  }
  if (pct >= 20.0) {
    return "\033[31m"; // red
  }
  if (pct >= 5.0) {
    return "\033[33m"; // yellow
  }
  return "\033[36m"; // cyan for low-but-present
}

void Profiler::collectHotspots(
    const ProfileNode& node,
    const std::string& path,
    const std::string& threadLabel,
    std::vector<Flattened>& out) const
{
  out.push_back(
      {path,
       threadLabel,
       node.source,
       node.inclusiveNs,
       node.selfNs,
       node.callCount});
  for (const auto& [_, child] : node.children) {
    const auto childPath
        = path.empty() ? child->label : (path + " > " + child->label);
    collectHotspots(*child, childPath, threadLabel, out);
  }
}

void Profiler::printNode(
    std::ostream& os,
    const ProfileNode& node,
    std::uint64_t threadTotalNs,
    const std::string& indent,
    double minPercent,
    std::size_t labelWidth) const
{
  std::vector<const ProfileNode*> children;
  children.reserve(node.children.size());
  for (const auto& [_, child] : node.children) {
    children.push_back(child.get());
  }

  std::ranges::sort(
      children, [](const ProfileNode* lhs, const ProfileNode* rhs) {
        return lhs->inclusiveNs > rhs->inclusiveNs;
      });

  for (std::size_t idx = 0; idx < children.size(); ++idx) {
    const auto* child = children[idx];
    const auto pct = percentage(child->inclusiveNs, threadTotalNs);
    if (pct < minPercent && child->inclusiveNs < 1'000'000) {
      continue; // Skip insignificant nodes (<minPercent and <1ms total).
    }

    const auto avgNs
        = child->callCount > 0 ? child->inclusiveNs / child->callCount : 0;

    const bool isLast = (idx + 1 == children.size());
    const std::string connector = isLast ? "`-- " : "|-- ";
    const std::string childIndent = indent + (isLast ? "    " : "|   ");
    const auto color = heatColor(pct);

    std::ostringstream line;
    line << indent << connector
         << colorize(padRight(child->label, labelWidth), color) << ' '
         << "total " << formatDurationAligned(child->inclusiveNs) << ' '
         << "self " << formatDurationAligned(child->selfNs) << ' '
         << "per-call " << formatDurationAligned(avgNs) << ' ' << "calls "
         << std::setw(8) << child->callCount << ' ' << "share "
         << padRight(colorize(formatPercent(pct), color), 6);
    if (!child->source.empty()) {
      line << " src " << child->source;
    }

    os << line.str() << '\n';

    printNode(
        os, *child, threadTotalNs, childIndent, minPercent * 0.65, labelWidth);
  }
}

void Profiler::printThreadTree(
    std::ostream& os,
    const ThreadRecord& record,
    std::uint64_t threadTotalNs,
    double minPercent,
    std::size_t labelWidth) const
{
  printNode(os, record.root, threadTotalNs, "", minPercent, labelWidth);
}

void Profiler::printSummary(std::ostream& os)
{
  std::vector<std::shared_ptr<ThreadRecord>> threads;
  {
    std::lock_guard<std::mutex> lock(m_threadRegistryMutex);
    threads = m_threads;
  }

  if (threads.empty()) {
    os << "DART profiler (text): no scoped regions were recorded.\n";
    return;
  }

  std::uint64_t totalNs = 0;
  for (const auto& record : threads) {
    totalNs += sumInclusiveChildren(record->root);
  }

  const auto frameCount = m_frameCount.load(std::memory_order_relaxed);

  os << "\nDART profiler (text backend)\n";
  os << "Threads: " << threads.size() << " | Frames marked: " << frameCount
     << " | Total scoped time: " << formatDuration(totalNs);

  const auto frameSumNs = m_frameTimeSumNs.load(std::memory_order_relaxed);
  if (frameCount > 1 && frameSumNs > 0 && !m_frameSamplesNs.empty()) {
    const double avgFps
        = (static_cast<double>(frameCount - 1) * 1e9) / frameSumNs;
    std::vector<std::uint64_t> samples = m_frameSamplesNs;
    std::ranges::sort(samples);
    const auto pickIndex = [&](double pct) -> std::size_t {
      if (samples.empty()) {
        return 0;
      }
      const auto maxIndex = std::ssize(samples) - 1;
      const double pos = pct * static_cast<double>(maxIndex);
      return static_cast<std::size_t>(std::clamp(
          static_cast<std::ptrdiff_t>(std::lround(pos)),
          static_cast<std::ptrdiff_t>(0),
          maxIndex));
    };
    const auto bestIdx = pickIndex(0.1);  // 10th percentile (short frames)
    const auto worstIdx = pickIndex(0.9); // 90th percentile (long frames)
    const double bestFps = 1e9 / static_cast<double>(samples[bestIdx]);
    const double worstFps = 1e9 / static_cast<double>(samples[worstIdx]);
    os << " | Avg FPS: " << formatFps(avgFps)
       << " | Best 10%: " << formatFps(bestFps)
       << " | Worst 10%: " << formatFps(worstFps);
  }
  os << '\n';

  // Build hotspot list.
  std::vector<Flattened> hotspots;
  for (const auto& record : threads) {
    for (const auto& [_, child] : record->root.children) {
      collectHotspots(*child, child->label, record->label, hotspots);
    }
  }

  std::ranges::sort(hotspots, [](const Flattened& lhs, const Flattened& rhs) {
    return lhs.inclusiveNs > rhs.inclusiveNs;
  });

  const std::size_t hotspotCount = std::min<std::size_t>(hotspots.size(), 10);

  os << "Hotspots (inclusive time):\n";
  if (hotspotCount == 0) {
    os << "  (no measured scopes)\n";
  } else {
    std::size_t maxHotLabel = 0;
    for (const auto& entry : hotspots) {
      maxHotLabel = std::max(maxHotLabel, entry.path.size());
    }
    const std::size_t hotLabelWidth
        = std::clamp<std::size_t>(maxHotLabel, 16, 48);

    for (std::size_t i = 0; i < hotspotCount; ++i) {
      const auto& entry = hotspots[i];
      const auto pct = percentage(entry.inclusiveNs, totalNs);
      const bool isHot = (i < 3) || (pct >= 20.0);

      const auto avgNs
          = entry.callCount > 0 ? entry.inclusiveNs / entry.callCount : 0;

      const auto color = heatColor(pct);
      const std::string tag = isHot ? colorize("[HOT]", color) : "     ";
      os << "  " << tag << " "
         << colorize(padRight(entry.path, hotLabelWidth), color) << " "
         << padRight(("thr " + entry.threadLabel), 12) << " total "
         << formatDurationAligned(entry.inclusiveNs) << " self "
         << formatDurationAligned(entry.selfNs) << " per-call "
         << formatDurationAligned(avgNs) << " calls " << entry.callCount
         << " share " << colorize(formatPercent(pct), color);
      if (!entry.source.empty()) {
        os << " src " << entry.source;
      }
      os << '\n';
    }
  }

  if (threads.size() > 1) {
    os << "Per-thread breakdown (inclusive):\n";
  }

  for (const auto& record : threads) {
    const auto threadTotal = sumInclusiveChildren(record->root);
    if (threadTotal == 0) {
      continue;
    }

    const auto labelWidth
        = maxLabelWidth(record->root, /*minWidth=*/16, /*maxWidth=*/48);

    os << "- thread " << record->label << " total "
       << formatDuration(threadTotal) << '\n';
    printThreadTree(os, *record, threadTotal, /*minPercent=*/0.25, labelWidth);
  }
  os << std::flush;
}

void Profiler::reset()
{
  std::lock_guard<std::mutex> lock(m_threadRegistryMutex);
  for (auto& record : m_threads) {
    clearNode(record->root);
  }
  m_frameCount.store(0, std::memory_order_relaxed);
  m_frameTimeSumNs.store(0, std::memory_order_relaxed);
  m_frameSamplesNs.clear();
  m_lastFrameTime = {};
}

void Profiler::clearNode(ProfileNode& node)
{
  node.callCount = 0;
  node.inclusiveNs = 0;
  node.selfNs = 0;
  node.minNs = std::numeric_limits<std::uint64_t>::max();
  node.maxNs = 0;
  for (auto& [_, child] : node.children) {
    clearNode(*child);
  }
  node.children.clear();
}

ProfileScope::ProfileScope(
    std::string_view name, std::string_view file, int line)
  : m_record(Profiler::threadRecord())
{
  Profiler::instance().pushScope(*m_record, name, file, line);
}

ProfileScope::~ProfileScope()
{
  if (m_record) {
    Profiler::instance().popScope(*m_record);
  }
}

} // namespace dart::common::profile
