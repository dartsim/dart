/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_MEMORY_DIAGNOSTICS_MODEL_HPP_
#define DART_EXAMPLES_DEMOS_MEMORY_DIAGNOSTICS_MODEL_HPP_

#include "memory_map_model.hpp"

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::examples::demos {

inline constexpr char kMemoryDiagnosticsSchema[] = "dart.memory-diagnostics.v1";
inline constexpr char kProcessResidentBytesKey[] = "process.resident_bytes";
inline constexpr char kProcessPeakResidentBytesKey[]
    = "process.peak_resident_bytes";
inline constexpr char kSessionPeakResidentBytesKey[]
    = "process.session_peak_resident_bytes";

/// How directly a diagnostic value describes the thing named by its label.
enum class MetricQuality
{
  Measured,
  Estimate,
  Proxy,
};

const char* metricQualityLabel(MetricQuality quality) noexcept;

/// One renderer-neutral diagnostic value. A missing value means unavailable.
struct DiagnosticMetric
{
  std::string key;
  std::string label;
  std::string unit;
  std::optional<double> value;
  MetricQuality quality{MetricQuality::Measured};
  std::string scope;
  std::string source;
  std::string limitation;
  bool includeInHistory{false};
};

/// A complete sample from one scene/World generation.
struct DiagnosticSnapshot
{
  std::string schema{kMemoryDiagnosticsSchema};
  std::string engine;
  std::string platform;
  std::uint64_t generation{0};
  double monotonicTimeSeconds{0.0};
  std::uint64_t frame{0};
  double simulationTimeSeconds{0.0};
  std::vector<DiagnosticMetric> metrics;
  /// Current-sample logical maps. History and baseline snapshots omit them.
  std::vector<MemoryMapRow> memoryMaps;
  std::vector<std::string> guidance;
};

const DiagnosticMetric* findMetric(
    const DiagnosticSnapshot& snapshot, std::string_view key) noexcept;

/// Checked result of parsing the Linux /proc/self/status memory fields.
struct LinuxProcStatusMemory
{
  std::optional<std::uint64_t> residentBytes;
  std::optional<std::uint64_t> peakResidentBytes;
};

LinuxProcStatusMemory parseLinuxProcStatus(std::string_view contents) noexcept;

/// Whole-process resident-memory readings, not DART-owned heap attribution.
struct ProcessMemoryReading
{
  std::optional<std::uint64_t> residentBytes;
  std::optional<std::uint64_t> peakResidentBytes;
  std::string platform;
  std::string source;
  std::string limitation;
};

std::string currentProcessPlatform();
ProcessMemoryReading collectProcessMemory();
std::vector<DiagnosticMetric> makeProcessMemoryMetrics(
    const ProcessMemoryReading& reading);

/// Adds non-duplicated, cautious quality/availability guidance.
void appendGenericMemoryGuidance(DiagnosticSnapshot& snapshot);

struct MetricDelta
{
  DiagnosticMetric metric;
  double baselineValue{0.0};
  double currentValue{0.0};
  double delta{0.0};
};

struct SnapshotComparison
{
  bool generationMatches{false};
  std::uint64_t generation{0};
  std::vector<MetricDelta> metrics;
};

/// Compares only values with matching key/unit/quality/scope/source/generation.
SnapshotComparison compareSnapshots(
    const DiagnosticSnapshot& baseline, const DiagnosticSnapshot& current);

struct HistorySeries
{
  std::string key;
  std::string label;
  std::string unit;
  MetricQuality quality{MetricQuality::Measured};
  std::string scope;
  std::string source;
  std::vector<double> monotonicTimeSeconds;
  std::vector<double> values;
};

/// Address-order statistics are layout proxies, not cache measurements.
struct AddressLocalitySummary
{
  MetricQuality quality{MetricQuality::Proxy};
  bool available{false};
  std::size_t addressCount{0};
  std::size_t pageSizeBytes{0};
  std::size_t distinctPageCount{0};
  std::size_t pageTransitions{0};
  std::optional<double> medianGapBytes;
  std::optional<double> p95GapBytes;
  std::string source;
  std::string limitation;
};

AddressLocalitySummary summarizeAddressLocality(
    const std::vector<std::uintptr_t>& addresses,
    std::size_t pageSizeBytes = 4096);

/// Opt-in sampling state shared by the DART 6 and DART 7 demo panels.
class DiagnosticSession
{
public:
  using Collector = std::function<DiagnosticSnapshot()>;

  explicit DiagnosticSession(
      Collector collector,
      std::size_t historyCapacity = 180,
      double sampleIntervalSeconds = 0.5);

  bool isEnabled() const noexcept;
  void setEnabled(bool enabled) noexcept;

  double sampleIntervalSeconds() const noexcept;
  void setSampleIntervalSeconds(double seconds) noexcept;

  /// Collects when enabled and the cadence is due. Returns true on collection.
  bool update(double monotonicNowSeconds);

  /// Collects immediately and stores its comparison fields as the baseline.
  bool captureNow(double monotonicNowSeconds);

  /// Stores the latest sample's comparison fields as the baseline.
  bool captureLatestAsBaseline();

  void clearBaseline() noexcept;

  /// Clears session-local observations but retains the history ring storage.
  void reset() noexcept;

  const std::optional<DiagnosticSnapshot>& latest() const noexcept;
  const std::optional<DiagnosticSnapshot>& baseline() const noexcept;
  std::optional<SnapshotComparison> comparison() const;

  std::optional<double> observedResidentPeakBytes() const noexcept;

  std::size_t historySize() const noexcept;
  std::size_t historyCapacity() const noexcept;
  std::size_t historyStorageCapacity() const noexcept;
  const DiagnosticSnapshot* historyAt(std::size_t index) const noexcept;
  std::optional<HistorySeries> historySeries(std::string_view key) const;

private:
  bool collect(double monotonicNowSeconds, bool makeBaseline);
  void recordHistory(const DiagnosticSnapshot& snapshot);
  void clearGenerationLocalState() noexcept;

  Collector mCollector;
  bool mEnabled{false};
  double mSampleIntervalSeconds{0.5};
  std::optional<double> mLastCollectionTimeSeconds;
  std::optional<std::uint64_t> mGeneration;
  std::optional<DiagnosticSnapshot> mLatest;
  std::optional<DiagnosticSnapshot> mBaseline;
  std::optional<double> mObservedResidentPeakBytes;
  std::size_t mHistoryLimit{0};
  std::vector<DiagnosticSnapshot> mHistorySlots;
  std::size_t mHistoryStart{0};
  std::size_t mHistoryCount{0};
};

} // namespace dart::examples::demos

#endif // DART_EXAMPLES_DEMOS_MEMORY_DIAGNOSTICS_MODEL_HPP_
