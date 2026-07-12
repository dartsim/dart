/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../memory_diagnostics_model.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::examples::demos {
namespace {

DiagnosticMetric makeMetric(
    std::string key,
    double value,
    bool includeInHistory = true,
    MetricQuality quality = MetricQuality::Measured,
    std::string unit = "bytes",
    std::string scope = "test scope",
    std::string source = "test source")
{
  DiagnosticMetric metric;
  metric.key = std::move(key);
  metric.label = "Test metric";
  metric.unit = std::move(unit);
  metric.value = value;
  metric.quality = quality;
  metric.scope = std::move(scope);
  metric.source = std::move(source);
  metric.limitation = "test limitation";
  metric.includeInHistory = includeInHistory;
  return metric;
}

DiagnosticSnapshot makeSnapshot(
    std::uint64_t generation,
    std::uint64_t frame,
    double value,
    std::string key = "test.bytes")
{
  DiagnosticSnapshot snapshot;
  snapshot.engine = "test engine";
  snapshot.platform = "test platform";
  snapshot.generation = generation;
  snapshot.frame = frame;
  snapshot.simulationTimeSeconds = static_cast<double>(frame) * 0.01;
  snapshot.metrics.push_back(makeMetric(std::move(key), value));
  return snapshot;
}

TEST(MemoryDiagnosticsModel, DisabledSessionDoesNoWorkOrMutation)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        return makeSnapshot(1, collectorCalls, 1024.0);
      },
      4,
      0.5);

  const std::size_t storageCapacity = session.historyStorageCapacity();
  EXPECT_EQ(storageCapacity, 0u);
  EXPECT_EQ(session.historyCapacity(), 4u);
  EXPECT_FALSE(session.isEnabled());
  EXPECT_FALSE(session.update(0.0));
  EXPECT_FALSE(session.captureNow(0.0));
  EXPECT_FALSE(session.captureLatestAsBaseline());

  EXPECT_EQ(collectorCalls, 0u);
  EXPECT_FALSE(session.latest());
  EXPECT_FALSE(session.baseline());
  EXPECT_FALSE(session.observedResidentPeakBytes());
  EXPECT_EQ(session.historySize(), 0u);
  EXPECT_EQ(session.historyStorageCapacity(), storageCapacity);
}

TEST(MemoryDiagnosticsModel, UpdateUsesCadenceAndCaptureBypassesIt)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        return makeSnapshot(1, collectorCalls, 100.0 + collectorCalls);
      },
      8,
      0.5);
  session.setEnabled(true);

  EXPECT_TRUE(session.update(10.0));
  EXPECT_FALSE(session.update(10.49));
  EXPECT_EQ(collectorCalls, 1u);
  EXPECT_TRUE(session.update(10.5));
  EXPECT_EQ(collectorCalls, 2u);

  EXPECT_TRUE(session.captureNow(10.6));
  ASSERT_TRUE(session.baseline());
  EXPECT_EQ(session.baseline()->frame, 3u);
  EXPECT_FALSE(session.update(10.6));
  EXPECT_EQ(collectorCalls, 3u);
  EXPECT_FALSE(session.update(10.99));
  EXPECT_EQ(collectorCalls, 3u);
  EXPECT_TRUE(session.update(11.1));
  EXPECT_EQ(collectorCalls, 4u);
}

TEST(MemoryDiagnosticsModel, HistoryIsBoundedAndSeriesIsChronological)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        DiagnosticSnapshot snapshot
            = makeSnapshot(7, collectorCalls, collectorCalls * 10.0);
        snapshot.metrics.push_back(makeMetric("not.history", 999.0, false));
        snapshot.addressMaps.push_back(makeMemoryAddressMapRow(
            "address.region.0",
            "Region",
            64u,
            {MemoryAddressSpan{
                .offsetBytes = 0u,
                .sizeBytes = 64u,
                .state = MemoryAddressState::Allocated,
                .category = MemoryAddressCategory::Unknown,
                .label = "unknown"}},
            "scope",
            "source",
            "limitation"));
        snapshot.memoryMaps.push_back(makeMemoryMapRow(
            "test.map",
            "Test map",
            "slots",
            2u,
            1u,
            1u,
            "test scope",
            "test source",
            "test limitation"));
        return snapshot;
      },
      3,
      0.0);
  session.setEnabled(true);

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(session.update(static_cast<double>(i)));
  }

  ASSERT_EQ(session.historySize(), 3u);
  ASSERT_NE(session.historyAt(0), nullptr);
  ASSERT_NE(session.historyAt(2), nullptr);
  EXPECT_EQ(session.historyAt(0)->frame, 3u);
  EXPECT_EQ(session.historyAt(2)->frame, 5u);
  EXPECT_EQ(findMetric(*session.historyAt(0), "not.history"), nullptr);
  EXPECT_TRUE(session.historyAt(0)->addressMaps.empty());
  EXPECT_TRUE(session.historyAt(0)->memoryMaps.empty());
  ASSERT_TRUE(session.latest());
  EXPECT_EQ(session.latest()->addressMaps.size(), 1u);
  ASSERT_EQ(session.latest()->addressMaps[0].cells.size(), 64u);
  session.setAddressMapCellLimit(32u);
  EXPECT_EQ(session.addressMapCellLimit(), 32u);
  ASSERT_TRUE(session.latest());
  EXPECT_EQ(session.latest()->addressMaps[0].cells.size(), 32u);
  session.setAddressMapCellLimit(5000u);
  EXPECT_EQ(session.addressMapCellLimit(), 4096u);
  EXPECT_EQ(session.latest()->addressMaps[0].cells.size(), 64u);
  EXPECT_EQ(session.latest()->memoryMaps.size(), 1u);

  const auto series = session.historySeries("test.bytes");
  ASSERT_TRUE(series);
  EXPECT_EQ(series->monotonicTimeSeconds, (std::vector<double>{2.0, 3.0, 4.0}));
  EXPECT_EQ(series->values, (std::vector<double>{30.0, 40.0, 50.0}));
  EXPECT_FALSE(session.historySeries("not.history"));
}

TEST(MemoryDiagnosticsModel, ResetClearsLocalStateAndRetainsStorage)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        DiagnosticSnapshot snapshot
            = makeSnapshot(1, collectorCalls, collectorCalls * 100.0);
        snapshot.metrics.clear();
        snapshot.metrics.push_back(
            makeMetric(kProcessResidentBytesKey, collectorCalls * 100.0));
        return snapshot;
      },
      5,
      10.0);
  session.setEnabled(true);
  ASSERT_TRUE(session.captureNow(1.0));
  ASSERT_TRUE(session.baseline());
  ASSERT_TRUE(session.observedResidentPeakBytes());
  ASSERT_EQ(session.historySize(), 1u);
  const DiagnosticMetric* sampledPeak
      = findMetric(*session.latest(), kSessionPeakResidentBytesKey);
  ASSERT_NE(sampledPeak, nullptr);
  ASSERT_TRUE(sampledPeak->value);
  EXPECT_DOUBLE_EQ(*sampledPeak->value, 100.0);
  EXPECT_EQ(sampledPeak->source, "test source");
  EXPECT_NE(
      sampledPeak->limitation.find("does not reset the OS"), std::string::npos);
  const std::size_t storageCapacity = session.historyStorageCapacity();

  session.reset();

  EXPECT_EQ(session.historySize(), 0u);
  EXPECT_EQ(session.historyStorageCapacity(), storageCapacity);
  EXPECT_FALSE(session.latest());
  EXPECT_FALSE(session.baseline());
  EXPECT_FALSE(session.observedResidentPeakBytes());
  EXPECT_TRUE(session.update(1.1));
  EXPECT_EQ(collectorCalls, 2u);
}

TEST(MemoryDiagnosticsModel, CaptureCreatesCompatibleSignedDelta)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        return makeSnapshot(
            3, collectorCalls, collectorCalls == 1 ? 20.0 : 15.0);
      },
      4,
      0.0);
  session.setEnabled(true);

  ASSERT_TRUE(session.captureNow(0.0));
  ASSERT_TRUE(session.update(1.0));
  const auto comparison = session.comparison();
  ASSERT_TRUE(comparison);
  ASSERT_TRUE(comparison->generationMatches);
  ASSERT_EQ(comparison->metrics.size(), 1u);
  EXPECT_DOUBLE_EQ(comparison->metrics[0].baselineValue, 20.0);
  EXPECT_DOUBLE_EQ(comparison->metrics[0].currentValue, 15.0);
  EXPECT_DOUBLE_EQ(comparison->metrics[0].delta, -5.0);
}

TEST(MemoryDiagnosticsModel, BaselinesOmitCurrentSamplePresentationPayload)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        DiagnosticSnapshot snapshot
            = makeSnapshot(3, collectorCalls, collectorCalls * 10.0);
        snapshot.addressMaps.push_back(makeMemoryAddressMapRow(
            "address.region.0",
            "Region",
            64u,
            {MemoryAddressSpan{
                .offsetBytes = 0u,
                .sizeBytes = 64u,
                .state = MemoryAddressState::Allocated,
                .category = MemoryAddressCategory::Unknown,
                .label = "unknown"}},
            "scope",
            "source",
            "limitation"));
        snapshot.memoryMaps.push_back(makeMemoryMapRow(
            "test.map",
            "Test map",
            "slots",
            2u,
            1u,
            1u,
            "test scope",
            "test source",
            "test limitation"));
        snapshot.guidance.push_back("current-sample guidance");
        return snapshot;
      },
      4,
      0.0);
  session.setEnabled(true);

  ASSERT_TRUE(session.update(0.0));
  ASSERT_TRUE(session.latest());
  ASSERT_EQ(session.latest()->addressMaps.size(), 1u);
  ASSERT_EQ(session.latest()->memoryMaps.size(), 1u);
  ASSERT_EQ(session.latest()->guidance.size(), 1u);
  ASSERT_TRUE(session.captureLatestAsBaseline());
  ASSERT_TRUE(session.baseline());
  EXPECT_TRUE(session.baseline()->addressMaps.empty());
  EXPECT_TRUE(session.baseline()->memoryMaps.empty());
  EXPECT_TRUE(session.baseline()->guidance.empty());
  EXPECT_EQ(session.baseline()->metrics.size(), 1u);

  session.clearBaseline();
  ASSERT_TRUE(session.captureNow(1.0));
  ASSERT_TRUE(session.latest());
  ASSERT_EQ(session.latest()->addressMaps.size(), 1u);
  ASSERT_EQ(session.latest()->memoryMaps.size(), 1u);
  ASSERT_EQ(session.latest()->guidance.size(), 1u);
  ASSERT_TRUE(session.baseline());
  EXPECT_TRUE(session.baseline()->addressMaps.empty());
  EXPECT_TRUE(session.baseline()->memoryMaps.empty());
  EXPECT_TRUE(session.baseline()->guidance.empty());
  EXPECT_EQ(session.baseline()->metrics.size(), 1u);
}

TEST(MemoryDiagnosticsModel, ComparisonRequiresAllSemanticFieldsToMatch)
{
  const DiagnosticSnapshot baseline = makeSnapshot(4, 1, 10.0);
  DiagnosticSnapshot current = makeSnapshot(4, 2, 12.0);
  ASSERT_EQ(compareSnapshots(baseline, current).metrics.size(), 1u);

  const auto expectNoDelta = [&baseline](DiagnosticSnapshot candidate) {
    const SnapshotComparison comparison = compareSnapshots(baseline, candidate);
    EXPECT_TRUE(comparison.generationMatches);
    EXPECT_TRUE(comparison.metrics.empty());
  };

  current.metrics[0].unit = "count";
  expectNoDelta(current);
  current = makeSnapshot(4, 2, 12.0);
  current.metrics[0].quality = MetricQuality::Estimate;
  expectNoDelta(current);
  current = makeSnapshot(4, 2, 12.0);
  current.metrics[0].scope = "different scope";
  expectNoDelta(current);
  current = makeSnapshot(4, 2, 12.0);
  current.metrics[0].source = "different source";
  expectNoDelta(current);
  current = makeSnapshot(4, 2, 12.0);
  current.metrics[0].value.reset();
  expectNoDelta(current);

  current = makeSnapshot(5, 2, 12.0);
  const SnapshotComparison differentGeneration
      = compareSnapshots(baseline, current);
  EXPECT_FALSE(differentGeneration.generationMatches);
  EXPECT_TRUE(differentGeneration.metrics.empty());
}

TEST(MemoryDiagnosticsModel, SchemaV2DoesNotCompareAgainstGroupedV1)
{
  DiagnosticSnapshot baseline = makeSnapshot(4, 1, 10.0);
  DiagnosticSnapshot current = makeSnapshot(4, 2, 12.0);
  EXPECT_EQ(current.schema, "dart.memory-diagnostics.v2");
  baseline.schema = "dart.memory-diagnostics.v1";

  const SnapshotComparison comparison = compareSnapshots(baseline, current);
  EXPECT_FALSE(comparison.schemaMatches);
  EXPECT_TRUE(comparison.generationMatches);
  EXPECT_TRUE(comparison.metrics.empty());
}

TEST(MemoryDiagnosticsModel, GenerationChangeStartsANewLocalSegment)
{
  std::size_t collectorCalls = 0;
  DiagnosticSession session(
      [&collectorCalls]() {
        ++collectorCalls;
        const std::uint64_t generation = collectorCalls < 3 ? 1 : 2;
        DiagnosticSnapshot snapshot
            = makeSnapshot(generation, collectorCalls, 0.0);
        snapshot.metrics.clear();
        snapshot.metrics.push_back(makeMetric(
            kProcessResidentBytesKey,
            generation == 1 ? collectorCalls * 100.0 : 50.0));
        return snapshot;
      },
      8,
      0.0);
  session.setEnabled(true);
  ASSERT_TRUE(session.captureNow(0.0));
  ASSERT_TRUE(session.update(1.0));
  ASSERT_EQ(session.historySize(), 2u);
  ASSERT_DOUBLE_EQ(*session.observedResidentPeakBytes(), 200.0);

  ASSERT_TRUE(session.update(2.0));
  EXPECT_EQ(session.historySize(), 1u);
  EXPECT_FALSE(session.baseline());
  ASSERT_TRUE(session.observedResidentPeakBytes());
  EXPECT_DOUBLE_EQ(*session.observedResidentPeakBytes(), 50.0);
  ASSERT_NE(session.historyAt(0), nullptr);
  EXPECT_EQ(session.historyAt(0)->generation, 2u);
}

TEST(MemoryDiagnosticsModel, LinuxStatusParserChecksUnitsAndOverflow)
{
  const LinuxProcStatusMemory valid = parseLinuxProcStatus(
      "Name:\tdart-demos\nVmHWM:\t42 kB\nVmRSS:\t17 kB\n");
  ASSERT_TRUE(valid.residentBytes);
  ASSERT_TRUE(valid.peakResidentBytes);
  EXPECT_EQ(*valid.residentBytes, 17u * 1024u);
  EXPECT_EQ(*valid.peakResidentBytes, 42u * 1024u);

  const LinuxProcStatusMemory missing
      = parseLinuxProcStatus("Name:\tdart-demos\nThreads:\t1\n");
  EXPECT_FALSE(missing.residentBytes);
  EXPECT_FALSE(missing.peakResidentBytes);

  const LinuxProcStatusMemory malformed
      = parseLinuxProcStatus("VmRSS: 10 MB\nVmHWM: -1 kB\n");
  EXPECT_FALSE(malformed.residentBytes);
  EXPECT_FALSE(malformed.peakResidentBytes);

  const LinuxProcStatusMemory overflow = parseLinuxProcStatus(
      "VmRSS: 18014398509481984 kB\nVmHWM: 999999999999999999999 kB\n");
  EXPECT_FALSE(overflow.residentBytes);
  EXPECT_FALSE(overflow.peakResidentBytes);

  const LinuxProcStatusMemory duplicate
      = parseLinuxProcStatus("VmRSS: 1 kB\nVmRSS: 2 kB\nVmHWM: 3 kB\n");
  EXPECT_FALSE(duplicate.residentBytes);
  ASSERT_TRUE(duplicate.peakResidentBytes);
  EXPECT_EQ(*duplicate.peakResidentBytes, 3u * 1024u);
}

TEST(MemoryDiagnosticsModel, LiveProcessProbeUsesAvailabilityNotFakeZero)
{
  const ProcessMemoryReading reading = collectProcessMemory();
  EXPECT_EQ(reading.platform, currentProcessPlatform());
  EXPECT_FALSE(reading.source.empty());
  EXPECT_FALSE(reading.limitation.empty());

#if defined(__linux__) || defined(_WIN32) || defined(__APPLE__)
  ASSERT_TRUE(reading.residentBytes);
  EXPECT_GT(*reading.residentBytes, 0u);
  if (reading.peakResidentBytes) {
    EXPECT_GE(*reading.peakResidentBytes, *reading.residentBytes);
  }
#else
  EXPECT_FALSE(reading.residentBytes);
  EXPECT_FALSE(reading.peakResidentBytes);
#endif

  const std::vector<DiagnosticMetric> metrics
      = makeProcessMemoryMetrics(reading);
  ASSERT_EQ(metrics.size(), 2u);
  EXPECT_EQ(metrics[0].key, kProcessResidentBytesKey);
  EXPECT_EQ(metrics[1].key, kProcessPeakResidentBytesKey);
  EXPECT_EQ(metrics[0].value.has_value(), reading.residentBytes.has_value());
  EXPECT_EQ(
      metrics[1].value.has_value(), reading.peakResidentBytes.has_value());
  EXPECT_TRUE(metrics[0].includeInHistory);
  EXPECT_FALSE(metrics[1].includeInHistory);
}

TEST(MemoryDiagnosticsModel, LocalitySummaryUsesIterationOrderMath)
{
  const std::vector<std::uintptr_t> addresses{
      0x1000u, 0x1010u, 0x3000u, 0x3020u, 0x2000u};
  const AddressLocalitySummary summary
      = summarizeAddressLocality(addresses, 4096);

  EXPECT_TRUE(summary.available);
  EXPECT_EQ(summary.quality, MetricQuality::Proxy);
  EXPECT_EQ(summary.addressCount, 5u);
  EXPECT_EQ(summary.distinctPageCount, 3u);
  EXPECT_EQ(summary.pageTransitions, 2u);
  ASSERT_TRUE(summary.medianGapBytes);
  ASSERT_TRUE(summary.p95GapBytes);
  EXPECT_DOUBLE_EQ(*summary.medianGapBytes, 2080.0);
  EXPECT_DOUBLE_EQ(*summary.p95GapBytes, 8176.0);
  EXPECT_NE(summary.limitation.find("cache misses"), std::string::npos);

  const AddressLocalitySummary singleton
      = summarizeAddressLocality({0x1234u}, 4096);
  EXPECT_TRUE(singleton.available);
  EXPECT_EQ(singleton.distinctPageCount, 1u);
  EXPECT_EQ(singleton.pageTransitions, 0u);
  EXPECT_FALSE(singleton.medianGapBytes);
  EXPECT_FALSE(singleton.p95GapBytes);

  const AddressLocalitySummary invalid = summarizeAddressLocality(addresses, 0);
  EXPECT_FALSE(invalid.available);
}

TEST(MemoryDiagnosticsModel, GenericGuidanceKeepsQualityClaimsCautious)
{
  DiagnosticSnapshot snapshot;
  DiagnosticMetric unavailable = makeMetric(kProcessResidentBytesKey, 0.0);
  unavailable.value.reset();
  snapshot.metrics.push_back(std::move(unavailable));
  snapshot.metrics.push_back(
      makeMetric("estimated.bytes", 10.0, false, MetricQuality::Estimate));
  snapshot.metrics.push_back(
      makeMetric("proxy.pages", 2.0, false, MetricQuality::Proxy, "count"));

  appendGenericMemoryGuidance(snapshot);
  const std::size_t guidanceCount = snapshot.guidance.size();
  appendGenericMemoryGuidance(snapshot);

  EXPECT_EQ(snapshot.guidance.size(), guidanceCount);
  ASSERT_EQ(snapshot.guidance.size(), 3u);
}

} // namespace
} // namespace dart::examples::demos
