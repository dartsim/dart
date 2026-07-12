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
  #include <unistd.h>
#elif defined(__linux__) || defined(__unix__)
  #include <unistd.h>
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

const char* memoryStorageStateLabel(MemoryStorageState state) noexcept
{
  switch (state) {
    case MemoryStorageState::Metadata:
      return "metadata";
    case MemoryStorageState::Allocated:
      return "allocated";
    case MemoryStorageState::Free:
      return "free";
    case MemoryStorageState::Reserved:
      return "reserved";
    case MemoryStorageState::Padding:
      return "padding";
    case MemoryStorageState::Observed:
      return "observed typed mark";
    case MemoryStorageState::Unobserved:
      return "unobserved address range";
  }
  return "unknown";
}

const char* memoryDataCategoryLabel(MemoryDataCategory category) noexcept
{
  switch (category) {
    case MemoryDataCategory::None:
      return "none";
    case MemoryDataCategory::Unknown:
      return "unknown payload";
    case MemoryDataCategory::Metadata:
      return "allocator metadata";
    case MemoryDataCategory::Model:
      return "model / topology";
    case MemoryDataCategory::State:
      return "simulation state";
    case MemoryDataCategory::Geometry:
      return "geometry";
    case MemoryDataCategory::ConstraintSolver:
      return "constraint / solver";
    case MemoryDataCategory::Scratch:
      return "scratch";
  }
  return "unknown";
}

const char* memoryExtentKindLabel(MemoryExtentKind kind) noexcept
{
  switch (kind) {
    case MemoryExtentKind::ExactByteRange:
      return "exact byte range";
    case MemoryExtentKind::ShallowLowerBound:
      return "shallow-size lower bound";
    case MemoryExtentKind::AddressPoint:
      return "address point only";
    case MemoryExtentKind::UnobservedRange:
      return "unobserved range";
  }
  return "unknown";
}

std::vector<MemoryMapCell> composeMemoryMapCells(
    const MemoryMapRegion& region, std::size_t bytesPerCell)
{
  std::vector<MemoryMapCell> cells;
  if (region.sizeBytes == 0 || bytesPerCell == 0) {
    return cells;
  }

  for (std::size_t offset = 0; offset < region.sizeBytes;) {
    MemoryMapCell cell;
    cell.offsetBytes = offset;
    cell.sizeBytes = std::min(bytesPerCell, region.sizeBytes - offset);
    cells.push_back(std::move(cell));
    offset += cells.back().sizeBytes;
  }

  const auto appendOverlaps = [&region, bytesPerCell, &cells](
                                  const std::vector<MemoryMapSpan>& spans,
                                  MemoryMapLayer layer) {
    for (std::size_t spanIndex = 0; spanIndex < spans.size(); ++spanIndex) {
      const auto& span = spans[spanIndex];
      const std::size_t spanBegin
          = std::min(span.offsetBytes, region.sizeBytes);
      const std::size_t spanSize
          = std::min(span.sizeBytes, region.sizeBytes - spanBegin);
      if (spanSize == 0) {
        continue;
      }
      const std::size_t spanEnd = spanBegin + spanSize;
      const std::size_t firstCell = spanBegin / bytesPerCell;
      const std::size_t lastCell = (spanEnd - 1u) / bytesPerCell;
      for (std::size_t cellIndex = firstCell; cellIndex <= lastCell;
           ++cellIndex) {
        auto& cell = cells[cellIndex];
        const std::size_t cellEnd = cell.offsetBytes + cell.sizeBytes;
        const std::size_t overlapBegin = std::max(spanBegin, cell.offsetBytes);
        const std::size_t overlapEnd = std::min(spanEnd, cellEnd);
        cell.segments.push_back(MemoryMapCellSegment{
            layer, spanIndex, overlapBegin, overlapEnd - overlapBegin});
      }
    }
  };
  appendOverlaps(region.spans, MemoryMapLayer::Partition);
  std::vector<std::size_t> partitionSegmentCounts;
  partitionSegmentCounts.reserve(cells.size());
  for (const auto& cell : cells) {
    partitionSegmentCounts.push_back(cell.segments.size());
  }
  appendOverlaps(region.observations, MemoryMapLayer::Observation);

  const auto sourceSpan
      = [&region](const MemoryMapCellSegment& segment) -> const MemoryMapSpan& {
    return segment.layer == MemoryMapLayer::Partition
               ? region.spans[segment.spanIndex]
               : region.observations[segment.spanIndex];
  };
  const auto segmentLess
      = [&sourceSpan](
            const MemoryMapCellSegment& lhs, const MemoryMapCellSegment& rhs) {
          const auto& lhsSpan = sourceSpan(lhs);
          const auto& rhsSpan = sourceSpan(rhs);
          if (lhsSpan.offsetBytes != rhsSpan.offsetBytes) {
            return lhsSpan.offsetBytes < rhsSpan.offsetBytes;
          }
          if (lhs.layer != rhs.layer) {
            return lhs.layer == MemoryMapLayer::Partition;
          }
          if (lhsSpan.dataCategory != rhsSpan.dataCategory) {
            return static_cast<int>(lhsSpan.dataCategory)
                   < static_cast<int>(rhsSpan.dataCategory);
          }
          if (lhsSpan.extentKind != rhsSpan.extentKind) {
            return static_cast<int>(lhsSpan.extentKind)
                   < static_cast<int>(rhsSpan.extentKind);
          }
          return lhsSpan.label < rhsSpan.label;
        };
  for (std::size_t cellIndex = 0; cellIndex < cells.size(); ++cellIndex) {
    auto& segments = cells[cellIndex].segments;
    const auto middle
        = segments.begin()
          + static_cast<std::ptrdiff_t>(partitionSegmentCounts[cellIndex]);
    if (middle == segments.begin() || middle == segments.end()) {
      continue;
    }
    std::vector<MemoryMapCellSegment> merged;
    merged.reserve(segments.size());
    std::merge(
        segments.begin(),
        middle,
        middle,
        segments.end(),
        std::back_inserter(merged),
        segmentLess);
    segments = std::move(merged);
  }
  return cells;
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

HostPageSizeReading collectHostPageSize()
{
  HostPageSizeReading reading;
#if defined(_WIN32)
  SYSTEM_INFO systemInfo{};
  GetSystemInfo(&systemInfo);
  reading.source = "GetSystemInfo dwPageSize";
  if (systemInfo.dwPageSize != 0) {
    reading.bytes = static_cast<std::size_t>(systemInfo.dwPageSize);
  } else {
    reading.limitation = "The operating system reported a zero page size.";
  }
#elif defined(_SC_PAGESIZE)
  reading.source = "sysconf(_SC_PAGESIZE)";
  const long pageSize = sysconf(_SC_PAGESIZE);
  if (pageSize > 0
      && static_cast<unsigned long>(pageSize)
             <= std::numeric_limits<std::size_t>::max()) {
    reading.bytes = static_cast<std::size_t>(pageSize);
  } else {
    reading.limitation
        = "The host page-size query failed; address-page maps are unavailable.";
  }
#else
  reading.source = "unsupported host page-size query";
  reading.limitation
      = "The host page size is unavailable on this platform; address-page "
        "maps are unavailable rather than assuming a page size.";
#endif
  return reading;
}

std::optional<std::uintptr_t> checkedAddressRangeEnd(
    std::uintptr_t begin, std::size_t sizeBytes) noexcept
{
  if (sizeBytes > std::numeric_limits<std::uintptr_t>::max() - begin) {
    return std::nullopt;
  }
  return begin + sizeBytes;
}

std::optional<std::uintptr_t> checkedRoundAddressUp(
    std::uintptr_t address, std::size_t alignmentBytes) noexcept
{
  if (alignmentBytes == 0) {
    return std::nullopt;
  }
  const std::size_t remainder = address % alignmentBytes;
  if (remainder == 0) {
    return address;
  }
  const std::size_t increment = alignmentBytes - remainder;
  if (increment > std::numeric_limits<std::uintptr_t>::max() - address) {
    return std::nullopt;
  }
  return address + increment;
}

std::vector<MemoryMapRegion> makeObjectAddressAtlas(
    std::vector<AddressAtlasExtent> extents,
    const HostPageSizeReading& pageSizeReading)
{
  std::vector<MemoryMapRegion> regions;
  if (extents.empty() || !pageSizeReading.bytes
      || *pageSizeReading.bytes == 0) {
    return regions;
  }
  const std::size_t pageSizeBytes = *pageSizeReading.bytes;

  std::sort(
      extents.begin(),
      extents.end(),
      [](const AddressAtlasExtent& lhs, const AddressAtlasExtent& rhs) {
        if (lhs.address != rhs.address) {
          return lhs.address < rhs.address;
        }
        if (lhs.dataCategory != rhs.dataCategory) {
          return static_cast<int>(lhs.dataCategory)
                 < static_cast<int>(rhs.dataCategory);
        }
        if (lhs.extentKind != rhs.extentKind) {
          return static_cast<int>(lhs.extentKind)
                 < static_cast<int>(rhs.extentKind);
        }
        if (lhs.label != rhs.label) {
          return lhs.label < rhs.label;
        }
        return lhs.sizeBytes < rhs.sizeBytes;
      });

  struct ValidatedExtent
  {
    AddressAtlasExtent extent;
    std::uintptr_t endAddress{0};
  };
  struct PageRun
  {
    std::uintptr_t begin{0};
    std::uintptr_t end{0};
    std::vector<ValidatedExtent> extents;
  };
  std::vector<PageRun> runs;

  for (auto& extent : extents) {
    if (extent.sizeBytes == 0) {
      continue;
    }
    const auto endAddress
        = checkedAddressRangeEnd(extent.address, extent.sizeBytes);
    if (!endAddress) {
      continue;
    }
    const std::uintptr_t pageBegin
        = (extent.address / pageSizeBytes) * pageSizeBytes;
    const auto pageEnd = checkedRoundAddressUp(*endAddress, pageSizeBytes);
    if (!pageEnd || *pageEnd <= pageBegin) {
      continue;
    }

    if (runs.empty() || pageBegin > runs.back().end) {
      runs.push_back(PageRun{pageBegin, *pageEnd, {}});
    } else {
      runs.back().end = std::max(runs.back().end, *pageEnd);
    }
    runs.back().extents.push_back(
        ValidatedExtent{std::move(extent), *endAddress});
  }

  regions.reserve(runs.size());
  for (std::size_t runIndex = 0; runIndex < runs.size(); ++runIndex) {
    auto& run = runs[runIndex];
    const std::uintptr_t runSize = run.end - run.begin;
    if (runSize > std::numeric_limits<std::size_t>::max()) {
      continue;
    }

    MemoryMapRegion region;
    region.id = "object-atlas-" + std::to_string(runIndex);
    region.label = "Observed object page run " + std::to_string(runIndex + 1u)
                   + " (" + std::to_string(pageSizeBytes) + " B host pages)";
    region.sizeBytes = static_cast<std::size_t>(runSize);
    region.quality = MetricQuality::Proxy;
    region.scope = "sampled classic DART 6 object virtual-address pages";
    region.source
        = "World graph traversal; exact object addresses plus evidence-bounded "
          "point or concrete-type sizeof observations";
    region.limitation
        = "Typed marks are address points unless the collector proves the "
          "concrete counted type; concrete sizeof spans remain shallow-size "
          "lower bounds, not allocator ownership or full object sizes. Gray "
          "spans are unobserved address ranges, not proven free memory. Page "
          "runs show virtual, not physical, contiguity.";
    region.pageSizeBytes = pageSizeBytes;
    region.pageSizeEvidence = pageSizeReading.source;

    region.spans.push_back(MemoryMapSpan{
        0,
        region.sizeBytes,
        MemoryStorageState::Unobserved,
        MemoryDataCategory::None,
        MemoryExtentKind::UnobservedRange,
        "Page-run address-space background",
        MetricQuality::Proxy,
        "non-overlapping background partition; typed observations are separate "
        "overlays"});

    std::size_t overlapConflictCount = 0;
    std::uintptr_t maximumObservedEnd = run.begin;
    for (const auto& validated : run.extents) {
      const auto& extent = validated.extent;
      if (extent.address < maximumObservedEnd) {
        ++overlapConflictCount;
      }
      maximumObservedEnd = std::max(maximumObservedEnd, validated.endAddress);
      region.observations.push_back(MemoryMapSpan{
          static_cast<std::size_t>(extent.address - run.begin),
          extent.sizeBytes,
          MemoryStorageState::Observed,
          extent.dataCategory,
          extent.extentKind,
          extent.label,
          extent.quality,
          extent.evidence});
    }
    if (overlapConflictCount != 0) {
      region.label += " | overlapping observations: "
                      + std::to_string(overlapConflictCount);
      region.limitation += " " + std::to_string(overlapConflictCount)
                           + " sampled observations overlap earlier evidence; "
                             "all observations are preserved independently.";
    }
    regions.push_back(std::move(region));
  }
  return regions;
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
  summary.addressStatisticsAvailable = true;
  summary.addressCount = addresses.size();
  summary.pageSizeBytes = pageSizeBytes;
  summary.source
      = "iteration-order virtual addresses and optional page buckets";
  summary.limitation
      = "Distinct pages, page transitions, and adjacent address gaps are "
        "layout proxies. They do not measure physical placement, cache misses, "
        "prefetch behavior, or execution speed.";

  std::vector<std::uintptr_t> gaps;
  if (addresses.size() > 1) {
    gaps.reserve(addresses.size() - 1);
  }

  for (std::size_t i = 0; i < addresses.size(); ++i) {
    if (i > 0) {
      gaps.push_back(
          addresses[i] >= addresses[i - 1] ? addresses[i] - addresses[i - 1]
                                           : addresses[i - 1] - addresses[i]);
    }
  }

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

  if (pageSizeBytes == 0) {
    summary.limitation
        += " Host page size is unavailable, so only page-bucket metrics are "
           "unavailable; address count and adjacent gaps remain valid.";
    return summary;
  }

  summary.pageStatisticsAvailable = true;
  std::vector<std::uintptr_t> pages;
  pages.reserve(addresses.size());
  for (std::size_t i = 0; i < addresses.size(); ++i) {
    const std::uintptr_t page = addresses[i] / pageSizeBytes;
    pages.push_back(page);
    if (i > 0 && page != addresses[i - 1] / pageSizeBytes) {
      ++summary.pageTransitions;
    }
  }
  std::sort(pages.begin(), pages.end());
  summary.distinctPageCount = static_cast<std::size_t>(
      std::distance(pages.begin(), std::unique(pages.begin(), pages.end())));
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
  mBaseline = *mLatest;
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
    mBaseline = *mLatest;
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
