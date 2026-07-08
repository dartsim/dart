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

#include "dart/common/detail/ProfileRecording.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>

namespace dart::common::profile {

namespace {

template <std::size_t Capacity>
struct InlineString
{
  void assign(std::string_view value)
  {
    length = std::min<std::size_t>(value.size(), Capacity - 1u);
    std::copy_n(value.data(), length, data.data());
    data[length] = '\0';
  }

  [[nodiscard]] std::string_view view() const
  {
    return {data.data(), length};
  }

  [[nodiscard]] bool matches(std::string_view value) const
  {
    const auto comparableLength
        = std::min<std::size_t>(value.size(), Capacity - 1u);
    return view() == value.substr(0u, comparableLength);
  }

  [[nodiscard]] bool empty() const
  {
    return length == 0u;
  }

  [[nodiscard]] std::size_t size() const
  {
    return length;
  }

  std::array<char, Capacity> data{};
  std::size_t length{0};
};

} // namespace

struct Profiler::ProfileNode
{
  InlineString<192> label;
  InlineString<320> file;
  int line{0};
  std::uint64_t callCount{0};
  std::uint64_t inclusiveNs{0};
  std::uint64_t selfNs{0};
  std::uint64_t minNs{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t maxNs{0};
  ProfileNode* firstChild{nullptr};
  ProfileNode* nextSibling{nullptr};
};

struct Profiler::CounterNode
{
  InlineString<192> label;
  InlineString<320> file;
  int line{0};
  std::uint64_t sampleCount{0};
  std::uint64_t sum{0};
  std::uint64_t min{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t max{0};
  std::uint64_t last{0};
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
  std::string rootLabel;
  std::thread::id id;
  ProfileNode root{};
  std::vector<ProfileNode> nodes;
  std::vector<CounterNode> counters;
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

Profiler::Profiler() : m_frameCount(0), m_frameTimeSumNs(0)
{
  m_frameSamplesNs.reserve(kMaxFrameSamples);
}

Profiler& Profiler::instance()
{
  static Profiler profiler;
  return profiler;
}

bool Profiler::setRecordingEnabled(bool enabled) noexcept
{
  return detail::setProfileRecordingEnabledFlag(enabled);
}

bool Profiler::isRecordingEnabled() const noexcept
{
  return detail::isProfileRecordingEnabledFlag();
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
    record->rootLabel = "thread " + record->label;
    record->root.label.assign(record->rootLabel);
  }
  record->nodes.reserve(kInitialThreadNodeCapacity);
  record->counters.reserve(kInitialThreadNodeCapacity);

  {
    std::lock_guard<std::mutex> lock(m_threadRegistryMutex);
    m_threads.push_back(record);
  }

  return record;
}

Profiler::ProfileNode* Profiler::findOrCreateChild(
    Profiler::ThreadRecord& record,
    ProfileNode& parent,
    std::string_view label,
    std::string_view file,
    int line)
{
  for (auto* child = parent.firstChild; child != nullptr;
       child = child->nextSibling) {
    if (child->line == line && child->label.matches(label)
        && child->file.matches(file)) {
      return child;
    }
  }

  auto* child = allocateNode(record, label, file, line);
  if (child == nullptr)
    return &parent;

  child->nextSibling = parent.firstChild;
  parent.firstChild = child;
  return child;
}

Profiler::ProfileNode* Profiler::allocateNode(
    Profiler::ThreadRecord& record,
    std::string_view label,
    std::string_view file,
    int line)
{
  if (record.nodes.size() == record.nodes.capacity())
    return nullptr;

  record.nodes.emplace_back();
  auto& node = record.nodes.back();
  node.label.assign(label);
  node.file.assign(file);
  node.line = line;
  return &node;
}

Profiler::CounterNode* Profiler::findOrCreateCounter(
    Profiler::ThreadRecord& record,
    std::string_view label,
    std::string_view file,
    int line)
{
  for (auto& counter : record.counters) {
    if (counter.line == line && counter.label.matches(label)
        && counter.file.matches(file)) {
      return &counter;
    }
  }

  if (record.counters.size() == record.counters.capacity())
    return nullptr;

  record.counters.emplace_back();
  auto& counter = record.counters.back();
  counter.label.assign(label);
  counter.file.assign(file);
  counter.line = line;
  return &counter;
}

std::string Profiler::sourceText(const ProfileNode& node)
{
  if (node.file.empty())
    return "";

  const auto file = node.file.view();
  std::string source;
  source.reserve(file.size() + 1u + 16u);
  source.append(file);
  source.push_back(':');
  source.append(std::to_string(static_cast<long long>(node.line)));
  return source;
}

std::string Profiler::sourceText(const CounterNode& node)
{
  if (node.file.empty())
    return "";

  const auto file = node.file.view();
  std::string source;
  source.reserve(file.size() + 1u + 16u);
  source.append(file);
  source.push_back(':');
  source.append(std::to_string(static_cast<long long>(node.line)));
  return source;
}

void Profiler::pushScope(
    Profiler::ThreadRecord& record,
    std::string_view label,
    std::string_view file,
    int line)
{
  auto* parent = record.stack.empty() ? &record.root : record.stack.back().node;
  auto* node = findOrCreateChild(record, *parent, label, file, line);
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

void Profiler::recordCounter(
    std::string_view label,
    std::string_view file,
    int line,
    std::uint64_t value)
{
  auto record = threadRecord();
  auto* counter = findOrCreateCounter(*record, label, file, line);
  if (counter == nullptr)
    return;

  ++counter->sampleCount;
  counter->sum += value;
  counter->min = std::min(counter->min, value);
  counter->max = std::max(counter->max, value);
  counter->last = value;
}

void Profiler::recordScopeForTesting(
    std::string_view label,
    std::string_view file,
    int line,
    std::uint64_t durationNs)
{
  auto record = threadRecord();
  auto* node = findOrCreateChild(*record, record->root, label, file, line);

  ++node->callCount;
  node->inclusiveNs += durationNs;
  node->selfNs += durationNs;
  node->minNs = std::min(node->minNs, durationNs);
  node->maxNs = std::max(node->maxNs, durationNs);
}

std::uint64_t Profiler::sumInclusiveChildren(const ProfileNode& node)
{
  std::uint64_t total = 0;
  for (auto* child = node.firstChild; child != nullptr;
       child = child->nextSibling) {
    total += child->inclusiveNs;
  }
  return total;
}

bool Profiler::hasRecordedScopes(const ProfileNode& node)
{
  if (node.callCount > 0) {
    return true;
  }

  for (auto* child = node.firstChild; child != nullptr;
       child = child->nextSibling) {
    if (hasRecordedScopes(*child)) {
      return true;
    }
  }

  return false;
}

bool Profiler::hasRecordedCounters(const ThreadRecord& record)
{
  for (const auto& counter : record.counters) {
    if (counter.sampleCount > 0)
      return true;
  }

  return false;
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
    if (m_frameSamplesNs.size() < kMaxFrameSamples) {
      m_frameSamplesNs.push_back(deltaNs);
    } else {
      m_frameSamplesNs[m_frameSampleCursor] = deltaNs;
      m_frameSampleCursor = (m_frameSampleCursor + 1u) % kMaxFrameSamples;
    }
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
  for (auto* child = node.firstChild; child != nullptr;
       child = child->nextSibling) {
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
  if (node.callCount == 0)
    return;

  out.push_back(
      {path,
       threadLabel,
       sourceText(node),
       node.inclusiveNs,
       node.selfNs,
       node.callCount});
  for (auto* child = node.firstChild; child != nullptr;
       child = child->nextSibling) {
    const auto childPath
        = path.empty() ? std::string(child->label.view())
                       : (path + " > " + std::string(child->label.view()));
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
  for (auto* child = node.firstChild; child != nullptr;
       child = child->nextSibling) {
    children.push_back(child);
  }

  std::sort(
      children.begin(),
      children.end(),
      [](const ProfileNode* lhs, const ProfileNode* rhs) {
        return lhs->inclusiveNs > rhs->inclusiveNs;
      });

  for (std::size_t idx = 0; idx < children.size(); ++idx) {
    const auto* child = children[idx];
    const auto pct = percentage(child->inclusiveNs, threadTotalNs);
    const bool shouldShowZeroDurationCall
        = threadTotalNs == 0 && child->callCount > 0;
    if (!shouldShowZeroDurationCall && pct < minPercent
        && child->inclusiveNs < 1'000'000) {
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
         << colorize(padRight(child->label.view(), labelWidth), color) << ' '
         << "total " << formatDurationAligned(child->inclusiveNs) << ' '
         << "self " << formatDurationAligned(child->selfNs) << ' '
         << "per-call " << formatDurationAligned(avgNs) << ' ' << "calls "
         << std::setw(8) << child->callCount << ' ' << "share "
         << padRight(colorize(formatPercent(pct), color), 6);
    const auto source = sourceText(*child);
    if (!source.empty()) {
      line << " src " << source;
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

void Profiler::printThreadCounters(
    std::ostream& os, const ThreadRecord& record) const
{
  std::vector<const CounterNode*> counters;
  counters.reserve(record.counters.size());
  for (const auto& counter : record.counters) {
    if (counter.sampleCount > 0)
      counters.push_back(&counter);
  }

  if (counters.empty())
    return;

  std::sort(
      counters.begin(),
      counters.end(),
      [](const CounterNode* lhs, const CounterNode* rhs) {
        const auto lhsLabel = lhs->label.view();
        const auto rhsLabel = rhs->label.view();
        if (lhsLabel == rhsLabel)
          return lhs->line < rhs->line;
        return lhsLabel < rhsLabel;
      });

  std::size_t labelWidth = 0;
  for (const auto* counter : counters)
    labelWidth = std::max(labelWidth, counter->label.view().size());
  labelWidth = std::clamp<std::size_t>(labelWidth, 16, 56);

  os << "  Counters:\n";
  for (const auto* counter : counters) {
    const double mean = counter->sampleCount > 0
                            ? static_cast<double>(counter->sum)
                                  / static_cast<double>(counter->sampleCount)
                            : 0.0;
    std::ostringstream meanText;
    meanText << std::fixed << std::setprecision(mean >= 100.0 ? 1 : 2) << mean;

    os << "    " << padRight(counter->label.view(), labelWidth) << " samples "
       << std::setw(8) << counter->sampleCount << " sum " << std::setw(8)
       << formatCount(counter->sum) << " mean " << std::setw(8)
       << meanText.str() << " min " << std::setw(8) << formatCount(counter->min)
       << " max " << std::setw(8) << formatCount(counter->max) << " last "
       << std::setw(8) << formatCount(counter->last);
    const auto source = sourceText(*counter);
    if (!source.empty())
      os << " src " << source;
    os << '\n';
  }
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
  bool anyScopes = false;
  bool anyCounters = false;
  for (const auto& record : threads) {
    totalNs += sumInclusiveChildren(record->root);
    anyScopes = anyScopes || hasRecordedScopes(record->root);
    anyCounters = anyCounters || hasRecordedCounters(*record);
  }

  if (!anyScopes && !anyCounters) {
    os << "DART profiler (text): no scoped regions were recorded.\n";
    return;
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
    std::sort(samples.begin(), samples.end());
    const auto pickIndex = [&](double pct) -> std::size_t {
      if (samples.empty()) {
        return 0;
      }
      const auto maxIndex = static_cast<std::ptrdiff_t>(samples.size()) - 1;
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
    for (auto* child = record->root.firstChild; child != nullptr;
         child = child->nextSibling) {
      collectHotspots(
          *child, std::string(child->label.view()), record->label, hotspots);
    }
  }

  std::sort(
      hotspots.begin(),
      hotspots.end(),
      [](const Flattened& lhs, const Flattened& rhs) {
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
    const bool threadHasCounters = hasRecordedCounters(*record);
    if (threadTotal == 0 && !hasRecordedScopes(record->root)
        && !threadHasCounters) {
      continue;
    }

    const auto labelWidth
        = maxLabelWidth(record->root, /*minWidth=*/16, /*maxWidth=*/48);

    os << "- thread " << record->label << " total "
       << formatDuration(threadTotal) << '\n';
    printThreadTree(os, *record, threadTotal, /*minPercent=*/0.25, labelWidth);
    printThreadCounters(os, *record);
  }
  os << std::flush;
}

std::string Profiler::toSummaryText()
{
  std::ostringstream os;
  printSummary(os);
  return os.str();
}

void Profiler::reset()
{
  std::lock_guard<std::mutex> lock(m_threadRegistryMutex);
  for (auto& record : m_threads) {
    clearNode(record->root);
    record->root.firstChild = nullptr;
    record->root.nextSibling = nullptr;
    record->nodes.clear();
    record->counters.clear();
    record->stack.clear();
  }
  m_frameCount.store(0, std::memory_order_relaxed);
  m_frameTimeSumNs.store(0, std::memory_order_relaxed);
  m_frameSamplesNs.clear();
  m_frameSampleCursor = 0;
  m_lastFrameTime = {};
}

void Profiler::clearNode(ProfileNode& node)
{
  node.callCount = 0;
  node.inclusiveNs = 0;
  node.selfNs = 0;
  node.minNs = Profiler::kUnsetDuration;
  node.maxNs = 0;
  for (auto* child = node.firstChild; child != nullptr;
       child = child->nextSibling) {
    clearNode(*child);
  }
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
