/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "memory_diagnostics_model.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <iterator>
#include <limits>
#include <utility>

#include <cmath>

#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
  #if !defined(PSAPI_VERSION)
    #define PSAPI_VERSION 1
  #endif
  #include <psapi.h>
#elif defined(__APPLE__)
  #include <mach/mach.h>
#endif

namespace dart::examples::demos {

namespace {

constexpr std::string_view kAsciiWhitespace{" \t\r\n\f\v"};

std::string_view trim(std::string_view value) noexcept
{
  const std::size_t first = value.find_first_not_of(kAsciiWhitespace);
  if (first == std::string_view::npos) {
    return {};
  }
  const std::size_t last = value.find_last_not_of(kAsciiWhitespace);
  return value.substr(first, last - first + 1);
}

std::optional<std::uint64_t> parseKilobytes(std::string_view value) noexcept
{
  value = trim(value);
  if (value.empty() || value.front() < '0' || value.front() > '9') {
    return std::nullopt;
  }

  std::uint64_t kilobytes = 0;
  const char* const begin = value.data();
  const char* const end = begin + value.size();
  const auto parsed = std::from_chars(begin, end, kilobytes);
  if (parsed.ec != std::errc{}) {
    return std::nullopt;
  }

  const std::string_view suffix = trim(
      std::string_view(parsed.ptr, static_cast<std::size_t>(end - parsed.ptr)));
  if (suffix != "kB") {
    return std::nullopt;
  }
  if (kilobytes > std::numeric_limits<std::uint64_t>::max() / 1024u) {
    return std::nullopt;
  }
  return kilobytes * 1024u;
}

void addGuidanceOnce(DiagnosticSnapshot& snapshot, std::string guidance)
{
  if (std::find(snapshot.guidance.begin(), snapshot.guidance.end(), guidance)
      == snapshot.guidance.end()) {
    snapshot.guidance.push_back(std::move(guidance));
  }
}

bool compatibleMetrics(
    const DiagnosticMetric& baseline, const DiagnosticMetric& current) noexcept
{
  return baseline.key == current.key && baseline.unit == current.unit
         && baseline.quality == current.quality
         && baseline.scope == current.scope
         && baseline.source == current.source;
}

DiagnosticSnapshot makeBaselineSnapshot(const DiagnosticSnapshot& snapshot)
{
  DiagnosticSnapshot baseline;
  baseline.schema = snapshot.schema;
  baseline.engine = snapshot.engine;
  baseline.platform = snapshot.platform;
  baseline.generation = snapshot.generation;
  baseline.monotonicTimeSeconds = snapshot.monotonicTimeSeconds;
  baseline.frame = snapshot.frame;
  baseline.simulationTimeSeconds = snapshot.simulationTimeSeconds;
  baseline.metrics = snapshot.metrics;
  return baseline;
}

} // namespace

const char* metricQualityLabel(MetricQuality quality) noexcept
{
  switch (quality) {
    case MetricQuality::Measured:
      return "measured";
    case MetricQuality::Estimate:
      return "estimate";
    case MetricQuality::Proxy:
      return "proxy";
  }
  return "unknown";
}

const DiagnosticMetric* findMetric(
    const DiagnosticSnapshot& snapshot, std::string_view key) noexcept
{
  const auto found = std::find_if(
      snapshot.metrics.begin(),
      snapshot.metrics.end(),
      [key](const DiagnosticMetric& metric) { return metric.key == key; });
  return found == snapshot.metrics.end() ? nullptr : &*found;
}

LinuxProcStatusMemory parseLinuxProcStatus(std::string_view contents) noexcept
{
  LinuxProcStatusMemory result;
  bool residentSeen = false;
  bool peakSeen = false;
  bool residentDuplicate = false;
  bool peakDuplicate = false;

  std::size_t position = 0;
  while (position <= contents.size()) {
    const std::size_t end = contents.find('\n', position);
    const std::string_view line = trim(contents.substr(
        position,
        end == std::string_view::npos ? std::string_view::npos
                                      : end - position));

    const auto parseField = [](std::string_view lineValue,
                               std::string_view prefix,
                               bool& seen,
                               bool& duplicate,
                               std::optional<std::uint64_t>& target) {
      if (lineValue.substr(0, prefix.size()) != prefix) {
        return;
      }
      if (seen) {
        duplicate = true;
        target.reset();
        return;
      }
      seen = true;
      target = parseKilobytes(lineValue.substr(prefix.size()));
    };

    parseField(
        line, "VmRSS:", residentSeen, residentDuplicate, result.residentBytes);
    parseField(
        line, "VmHWM:", peakSeen, peakDuplicate, result.peakResidentBytes);

    if (end == std::string_view::npos) {
      break;
    }
    position = end + 1;
  }

  if (residentDuplicate) {
    result.residentBytes.reset();
  }
  if (peakDuplicate) {
    result.peakResidentBytes.reset();
  }
  return result;
}

std::string currentProcessPlatform()
{
#if defined(__linux__)
  return "linux";
#elif defined(_WIN32)
  return "windows";
#elif defined(__APPLE__)
  return "macos";
#else
  return "unsupported";
#endif
}

ProcessMemoryReading collectProcessMemory()
{
  ProcessMemoryReading reading;
  reading.platform = currentProcessPlatform();

#if defined(__linux__)
  reading.source = "/proc/self/status VmRSS/VmHWM";
  reading.limitation
      = "OS-accounted whole-process resident memory can lag and includes the "
        "renderer, libraries, assets, allocator-retained pages, and the "
        "diagnostics buffers themselves; it is not DART-owned heap usage.";

  std::ifstream status("/proc/self/status");
  if (!status) {
    return reading;
  }
  const std::string contents{
      std::istreambuf_iterator<char>(status), std::istreambuf_iterator<char>()};
  const LinuxProcStatusMemory parsed = parseLinuxProcStatus(contents);
  reading.residentBytes = parsed.residentBytes;
  reading.peakResidentBytes = parsed.peakResidentBytes;
#elif defined(_WIN32)
  reading.source = "GetProcessMemoryInfo working set";
  reading.limitation
      = "The working set is whole-process OS accounting, not DART-owned heap "
        "usage, and includes renderer, library, asset, and diagnostic pages.";

  PROCESS_MEMORY_COUNTERS counters{};
  counters.cb = sizeof(counters);
  if (GetProcessMemoryInfo(
          GetCurrentProcess(),
          &counters,
          static_cast<DWORD>(sizeof(counters)))) {
    reading.residentBytes = static_cast<std::uint64_t>(counters.WorkingSetSize);
    reading.peakResidentBytes
        = static_cast<std::uint64_t>(counters.PeakWorkingSetSize);
  }
#elif defined(__APPLE__)
  reading.source = "task_info TASK_VM_INFO resident size";
  reading.limitation
      = "Resident size is whole-process Mach accounting, not DART-owned heap "
        "usage, and includes renderer, library, asset, and diagnostic pages.";

  task_vm_info_data_t info{};
  mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
  const kern_return_t status = task_info(
      mach_task_self(),
      TASK_VM_INFO,
      reinterpret_cast<task_info_t>(&info),
      &count);
  if (status == KERN_SUCCESS) {
    reading.residentBytes = static_cast<std::uint64_t>(info.resident_size);
  #if defined(TASK_VM_INFO_REV0_COUNT)
    if (count >= TASK_VM_INFO_REV0_COUNT) {
      reading.peakResidentBytes
          = static_cast<std::uint64_t>(info.resident_size_peak);
    }
  #endif
  }
#else
  reading.source = "unsupported platform";
  reading.limitation
      = "Whole-process resident memory is unavailable on this platform.";
#endif

  return reading;
}

std::vector<DiagnosticMetric> makeProcessMemoryMetrics(
    const ProcessMemoryReading& reading)
{
  DiagnosticMetric resident;
  resident.key = kProcessResidentBytesKey;
  resident.label = "Current process resident memory";
  resident.unit = "bytes";
  if (reading.residentBytes) {
    resident.value = static_cast<double>(*reading.residentBytes);
  }
  resident.quality = MetricQuality::Measured;
  resident.scope = "whole process";
  resident.source = reading.source;
  resident.limitation = reading.limitation;
  resident.includeInHistory = true;

  DiagnosticMetric peak = resident;
  peak.key = kProcessPeakResidentBytesKey;
  peak.label = "Process-lifetime peak resident memory";
  peak.value.reset();
  if (reading.peakResidentBytes) {
    peak.value = static_cast<double>(*reading.peakResidentBytes);
  }
  peak.includeInHistory = false;

  std::vector<DiagnosticMetric> metrics;
  metrics.reserve(2);
  metrics.push_back(std::move(resident));
  metrics.push_back(std::move(peak));
  return metrics;
}

void appendGenericMemoryGuidance(DiagnosticSnapshot& snapshot)
{
  bool hasEstimate = false;
  bool hasProxy = false;
  for (const DiagnosticMetric& metric : snapshot.metrics) {
    hasEstimate = hasEstimate || metric.quality == MetricQuality::Estimate;
    hasProxy = hasProxy || metric.quality == MetricQuality::Proxy;
  }

  if (hasEstimate) {
    addGuidanceOnce(
        snapshot,
        "Estimated byte totals may omit nested, third-party, allocator, or "
        "driver allocations; use them for bounded comparisons, not ownership "
        "accounting.");
  }
  if (hasProxy) {
    addGuidanceOnce(
        snapshot,
        "Layout proxies describe address or storage patterns only; they do "
        "not measure cache misses or prove a performance improvement.");
  }

  const DiagnosticMetric* resident
      = findMetric(snapshot, kProcessResidentBytesKey);
  if (resident != nullptr && !resident->value) {
    addGuidanceOnce(
        snapshot,
        "Current process resident memory is unavailable from this platform "
        "source; unavailable values are intentionally not shown as zero.");
  }
}

SnapshotComparison compareSnapshots(
    const DiagnosticSnapshot& baseline, const DiagnosticSnapshot& current)
{
  SnapshotComparison comparison;
  comparison.schemaMatches = baseline.schema == current.schema;
  comparison.generationMatches = baseline.generation == current.generation;
  comparison.generation = current.generation;
  if (!comparison.schemaMatches || !comparison.generationMatches) {
    return comparison;
  }

  comparison.metrics.reserve(current.metrics.size());
  for (const DiagnosticMetric& currentMetric : current.metrics) {
    if (!currentMetric.value) {
      continue;
    }
    const DiagnosticMetric* baselineMetric
        = findMetric(baseline, currentMetric.key);
    if (baselineMetric == nullptr || !baselineMetric->value
        || !compatibleMetrics(*baselineMetric, currentMetric)) {
      continue;
    }

    MetricDelta delta;
    delta.metric = currentMetric;
    delta.baselineValue = *baselineMetric->value;
    delta.currentValue = *currentMetric.value;
    delta.delta = delta.currentValue - delta.baselineValue;
    comparison.metrics.push_back(std::move(delta));
  }
  return comparison;
}

AddressLocalitySummary summarizeAddressLocality(
    const std::vector<std::uintptr_t>& addresses, std::size_t pageSizeBytes)
{
  AddressLocalitySummary summary;
  summary.addressCount = addresses.size();
  summary.pageSizeBytes = pageSizeBytes;
  summary.source = "iteration-order virtual addresses grouped by page";
  summary.limitation
      = "Distinct pages, page transitions, and adjacent address gaps are "
        "layout proxies. They do not measure physical placement, cache misses, "
        "prefetch behavior, or execution speed.";
  if (pageSizeBytes == 0) {
    return summary;
  }
  summary.available = true;

  std::vector<std::uintptr_t> pages;
  pages.reserve(addresses.size());
  std::vector<std::uintptr_t> gaps;
  if (addresses.size() > 1) {
    gaps.reserve(addresses.size() - 1);
  }

  for (std::size_t i = 0; i < addresses.size(); ++i) {
    const std::uintptr_t page = addresses[i] / pageSizeBytes;
    pages.push_back(page);
    if (i > 0) {
      const std::uintptr_t previousPage = addresses[i - 1] / pageSizeBytes;
      if (page != previousPage) {
        ++summary.pageTransitions;
      }
      gaps.push_back(
          addresses[i] >= addresses[i - 1] ? addresses[i] - addresses[i - 1]
                                           : addresses[i - 1] - addresses[i]);
    }
  }

  std::sort(pages.begin(), pages.end());
  summary.distinctPageCount = static_cast<std::size_t>(
      std::distance(pages.begin(), std::unique(pages.begin(), pages.end())));

  if (!gaps.empty()) {
    std::sort(gaps.begin(), gaps.end());
    const std::size_t middle = gaps.size() / 2;
    if (gaps.size() % 2 == 0) {
      summary.medianGapBytes = (static_cast<double>(gaps[middle - 1])
                                + static_cast<double>(gaps[middle]))
                               / 2.0;
    } else {
      summary.medianGapBytes = static_cast<double>(gaps[middle]);
    }
    const std::size_t p95Index = static_cast<std::size_t>(std::ceil(
                                     0.95 * static_cast<double>(gaps.size())))
                                 - 1;
    summary.p95GapBytes = static_cast<double>(gaps[p95Index]);
  }
  return summary;
}

DiagnosticSession::DiagnosticSession(
    Collector collector,
    std::size_t historyCapacity,
    double sampleIntervalSeconds)
  : mCollector(std::move(collector)),
    mSampleIntervalSeconds(
        std::isfinite(sampleIntervalSeconds) && sampleIntervalSeconds >= 0.0
            ? sampleIntervalSeconds
            : 0.5),
    mHistoryLimit(historyCapacity)
{
  // Collection and history allocation are deliberately deferred until the
  // user explicitly enables diagnostics and a sample is actually collected.
}

bool DiagnosticSession::isEnabled() const noexcept
{
  return mEnabled;
}

void DiagnosticSession::setEnabled(bool enabled) noexcept
{
  if (mEnabled == enabled) {
    return;
  }
  mEnabled = enabled;
  if (enabled) {
    mLastCollectionTimeSeconds.reset();
  }
}

double DiagnosticSession::sampleIntervalSeconds() const noexcept
{
  return mSampleIntervalSeconds;
}

void DiagnosticSession::setSampleIntervalSeconds(double seconds) noexcept
{
  if (std::isfinite(seconds) && seconds >= 0.0) {
    mSampleIntervalSeconds = seconds;
  }
}

std::size_t DiagnosticSession::addressMapCellLimit() const noexcept
{
  return mAddressMapCellLimit;
}

void DiagnosticSession::setAddressMapCellLimit(std::size_t maximumCells)
{
  constexpr std::size_t minimumCells = 32u;
  constexpr std::size_t maximumAllowedCells = 4096u;
  const std::size_t bounded
      = std::clamp(maximumCells, minimumCells, maximumAllowedCells);
  if (bounded == mAddressMapCellLimit) {
    return;
  }
  mAddressMapCellLimit = bounded;
  if (mLatest) {
    for (MemoryAddressMapRow& row : mLatest->addressMaps) {
      rebinMemoryAddressMapRow(row, mAddressMapCellLimit);
    }
  }
}

bool DiagnosticSession::update(double monotonicNowSeconds)
{
  if (!mEnabled || !mCollector || !std::isfinite(monotonicNowSeconds)) {
    return false;
  }
  if (mLastCollectionTimeSeconds
      && monotonicNowSeconds >= *mLastCollectionTimeSeconds
      && monotonicNowSeconds - *mLastCollectionTimeSeconds
             < mSampleIntervalSeconds) {
    return false;
  }
  return collect(monotonicNowSeconds, false);
}

bool DiagnosticSession::captureNow(double monotonicNowSeconds)
{
  if (!mEnabled || !mCollector || !std::isfinite(monotonicNowSeconds)) {
    return false;
  }
  return collect(monotonicNowSeconds, true);
}

bool DiagnosticSession::captureLatestAsBaseline()
{
  if (!mEnabled || !mLatest) {
    return false;
  }
  mBaseline = makeBaselineSnapshot(*mLatest);
  return true;
}

void DiagnosticSession::clearBaseline() noexcept
{
  mBaseline.reset();
}

void DiagnosticSession::reset() noexcept
{
  clearGenerationLocalState();
  mLastCollectionTimeSeconds.reset();
}

const std::optional<DiagnosticSnapshot>& DiagnosticSession::latest()
    const noexcept
{
  return mLatest;
}

const std::optional<DiagnosticSnapshot>& DiagnosticSession::baseline()
    const noexcept
{
  return mBaseline;
}

std::optional<SnapshotComparison> DiagnosticSession::comparison() const
{
  if (!mBaseline || !mLatest) {
    return std::nullopt;
  }
  return compareSnapshots(*mBaseline, *mLatest);
}

std::optional<double> DiagnosticSession::observedResidentPeakBytes()
    const noexcept
{
  return mObservedResidentPeakBytes;
}

std::size_t DiagnosticSession::historySize() const noexcept
{
  return mHistoryCount;
}

std::size_t DiagnosticSession::historyCapacity() const noexcept
{
  return mHistoryLimit;
}

std::size_t DiagnosticSession::historyStorageCapacity() const noexcept
{
  return mHistorySlots.capacity();
}

const DiagnosticSnapshot* DiagnosticSession::historyAt(
    std::size_t index) const noexcept
{
  if (index >= mHistoryCount || mHistorySlots.empty()) {
    return nullptr;
  }
  return &mHistorySlots[(mHistoryStart + index) % mHistorySlots.size()];
}

std::optional<HistorySeries> DiagnosticSession::historySeries(
    std::string_view key) const
{
  const DiagnosticMetric* descriptor = nullptr;
  for (std::size_t offset = mHistoryCount; offset > 0; --offset) {
    const DiagnosticSnapshot* snapshot = historyAt(offset - 1);
    const DiagnosticMetric* metric
        = snapshot == nullptr ? nullptr : findMetric(*snapshot, key);
    if (metric != nullptr && metric->includeInHistory && metric->value) {
      descriptor = metric;
      break;
    }
  }
  if (descriptor == nullptr) {
    return std::nullopt;
  }

  HistorySeries series;
  series.key = descriptor->key;
  series.label = descriptor->label;
  series.unit = descriptor->unit;
  series.quality = descriptor->quality;
  series.scope = descriptor->scope;
  series.source = descriptor->source;
  series.monotonicTimeSeconds.reserve(mHistoryCount);
  series.values.reserve(mHistoryCount);
  for (std::size_t i = 0; i < mHistoryCount; ++i) {
    const DiagnosticSnapshot* snapshot = historyAt(i);
    const DiagnosticMetric* metric
        = snapshot == nullptr ? nullptr : findMetric(*snapshot, key);
    if (metric == nullptr || !metric->includeInHistory || !metric->value
        || !compatibleMetrics(*descriptor, *metric)) {
      continue;
    }
    series.monotonicTimeSeconds.push_back(snapshot->monotonicTimeSeconds);
    series.values.push_back(*metric->value);
  }
  return series;
}

bool DiagnosticSession::collect(double monotonicNowSeconds, bool makeBaseline)
{
  DiagnosticSnapshot snapshot = mCollector();
  snapshot.monotonicTimeSeconds = monotonicNowSeconds;
  if (snapshot.schema.empty()) {
    snapshot.schema = kMemoryDiagnosticsSchema;
  }
  appendGenericMemoryGuidance(snapshot);
  for (MemoryAddressMapRow& row : snapshot.addressMaps) {
    rebinMemoryAddressMapRow(row, mAddressMapCellLimit);
  }

  if (mGeneration && *mGeneration != snapshot.generation) {
    clearGenerationLocalState();
  }
  mGeneration = snapshot.generation;

  const DiagnosticMetric* resident
      = findMetric(snapshot, kProcessResidentBytesKey);
  if (resident != nullptr && resident->value && resident->unit == "bytes"
      && resident->quality == MetricQuality::Measured
      && std::isfinite(*resident->value) && *resident->value >= 0.0) {
    if (!mObservedResidentPeakBytes
        || *resident->value > *mObservedResidentPeakBytes) {
      mObservedResidentPeakBytes = *resident->value;
    }
  }
  if (resident != nullptr) {
    DiagnosticMetric sessionPeak = *resident;
    sessionPeak.key = kSessionPeakResidentBytesKey;
    sessionPeak.label = "Sampled-session peak resident memory";
    sessionPeak.value = mObservedResidentPeakBytes;
    sessionPeak.limitation
        = "This is the maximum among diagnostics samples since the most "
          "recent session reset or scene generation change. Resetting it does "
          "not reset the OS process-lifetime peak. "
          + resident->limitation;
    sessionPeak.includeInHistory = false;
    const auto existing = std::find_if(
        snapshot.metrics.begin(),
        snapshot.metrics.end(),
        [](const DiagnosticMetric& metric) {
          return metric.key == kSessionPeakResidentBytesKey;
        });
    if (existing == snapshot.metrics.end()) {
      snapshot.metrics.push_back(std::move(sessionPeak));
    } else {
      *existing = std::move(sessionPeak);
    }
  }

  recordHistory(snapshot);
  mLatest = std::move(snapshot);
  if (makeBaseline) {
    mBaseline = makeBaselineSnapshot(*mLatest);
  }
  mLastCollectionTimeSeconds = monotonicNowSeconds;
  return true;
}

void DiagnosticSession::recordHistory(const DiagnosticSnapshot& snapshot)
{
  if (mHistoryLimit == 0) {
    return;
  }
  if (mHistorySlots.empty()) {
    mHistorySlots.resize(mHistoryLimit);
  }

  std::size_t slotIndex = 0;
  if (mHistoryCount < mHistorySlots.size()) {
    slotIndex = (mHistoryStart + mHistoryCount) % mHistorySlots.size();
    ++mHistoryCount;
  } else {
    slotIndex = mHistoryStart;
    mHistoryStart = (mHistoryStart + 1) % mHistorySlots.size();
  }

  DiagnosticSnapshot& slot = mHistorySlots[slotIndex];
  slot.schema = snapshot.schema;
  slot.engine = snapshot.engine;
  slot.platform = snapshot.platform;
  slot.generation = snapshot.generation;
  slot.monotonicTimeSeconds = snapshot.monotonicTimeSeconds;
  slot.frame = snapshot.frame;
  slot.simulationTimeSeconds = snapshot.simulationTimeSeconds;
  slot.metrics.clear();
  for (const DiagnosticMetric& metric : snapshot.metrics) {
    if (metric.includeInHistory) {
      slot.metrics.push_back(metric);
    }
  }
  slot.addressMaps.clear();
  slot.memoryMaps.clear();
  slot.guidance.clear();
}

void DiagnosticSession::clearGenerationLocalState() noexcept
{
  mHistoryStart = 0;
  mHistoryCount = 0;
  mLatest.reset();
  mBaseline.reset();
  mObservedResidentPeakBytes.reset();
}

} // namespace dart::examples::demos
