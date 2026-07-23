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

#include <algorithm>
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

  current = makeSnapshot(4, 2, 12.0);
  current.schema = "dart.memory-diagnostics.v1";
  const SnapshotComparison differentSchema
      = compareSnapshots(baseline, current);
  EXPECT_FALSE(differentSchema.schemaMatches);
  EXPECT_TRUE(differentSchema.generationMatches);
  EXPECT_TRUE(differentSchema.metrics.empty());
}

TEST(MemoryDiagnosticsModel, HostPageSizeProbeIsGuardedAndExplicit)
{
  const HostPageSizeReading reading = collectHostPageSize();
  EXPECT_FALSE(reading.source.empty());
  if (reading.bytes) {
    EXPECT_GT(*reading.bytes, 0u);
  } else {
    EXPECT_FALSE(reading.limitation.empty());
  }
}

TEST(MemoryDiagnosticsModel, AddressRangeAndPageRoundingRejectOverflow)
{
  constexpr std::uintptr_t maximum = std::numeric_limits<std::uintptr_t>::max();
  EXPECT_EQ(checkedAddressRangeEnd(100u, 24u), 124u);
  EXPECT_FALSE(checkedAddressRangeEnd(maximum - 3u, 4u));

  EXPECT_EQ(checkedRoundAddressUp(4097u, 4096u), 8192u);
  EXPECT_EQ(checkedRoundAddressUp(8192u, 4096u), 8192u);
  EXPECT_FALSE(checkedRoundAddressUp(100u, 0u));
  EXPECT_FALSE(checkedRoundAddressUp(maximum - 3u, 8u));
}

TEST(MemoryDiagnosticsModel, ObjectAtlasIsDeterministicAndExplicitAboutOverlap)
{
  HostPageSizeReading pageSize;
  pageSize.bytes = 64u;
  pageSize.source = "synthetic 64-byte host page";

  const auto makeExtent
      = [](std::uintptr_t address,
           std::size_t sizeBytes,
           MemoryDataCategory category,
           std::string label,
           MemoryExtentKind extentKind = MemoryExtentKind::ShallowLowerBound) {
          AddressAtlasExtent extent;
          extent.address = address;
          extent.sizeBytes = sizeBytes;
          extent.dataCategory = category;
          extent.extentKind = extentKind;
          extent.quality = extentKind == MemoryExtentKind::AddressPoint
                               ? MetricQuality::Proxy
                               : MetricQuality::Estimate;
          extent.label = std::move(label);
          extent.evidence = "synthetic exact address";
          return extent;
        };

  const std::vector<AddressAtlasExtent> extents{
      makeExtent(0x1080u, 16u, MemoryDataCategory::Geometry, "geometry extent"),
      makeExtent(0x1010u, 32u, MemoryDataCategory::State, "state extent"),
      makeExtent(0x1010u, 32u, MemoryDataCategory::Model, "model extent"),
      makeExtent(
          0x1018u,
          1u,
          MemoryDataCategory::ConstraintSolver,
          "constraint point",
          MemoryExtentKind::AddressPoint),
      makeExtent(
          0x1018u,
          1u,
          MemoryDataCategory::Geometry,
          "geometry point",
          MemoryExtentKind::AddressPoint)};

  const auto forward = makeObjectAddressAtlas(extents, pageSize);
  std::vector<AddressAtlasExtent> reversed = extents;
  std::reverse(reversed.begin(), reversed.end());
  const auto backward = makeObjectAddressAtlas(std::move(reversed), pageSize);

  ASSERT_EQ(forward.size(), 2u);
  ASSERT_EQ(backward.size(), forward.size());
  for (std::size_t regionIndex = 0; regionIndex < forward.size();
       ++regionIndex) {
    const auto& lhs = forward[regionIndex];
    const auto& rhs = backward[regionIndex];
    EXPECT_EQ(lhs.id, rhs.id);
    EXPECT_EQ(lhs.label, rhs.label);
    EXPECT_EQ(lhs.sizeBytes, rhs.sizeBytes);
    EXPECT_EQ(lhs.pageSizeBytes, rhs.pageSizeBytes);
    ASSERT_EQ(lhs.spans.size(), rhs.spans.size());
    ASSERT_EQ(lhs.observations.size(), rhs.observations.size());

    std::size_t cursor = 0;
    for (std::size_t spanIndex = 0; spanIndex < lhs.spans.size(); ++spanIndex) {
      const auto& lhsSpan = lhs.spans[spanIndex];
      const auto& rhsSpan = rhs.spans[spanIndex];
      EXPECT_EQ(lhsSpan.offsetBytes, cursor);
      cursor += lhsSpan.sizeBytes;
      EXPECT_EQ(lhsSpan.offsetBytes, rhsSpan.offsetBytes);
      EXPECT_EQ(lhsSpan.sizeBytes, rhsSpan.sizeBytes);
      EXPECT_EQ(lhsSpan.storageState, rhsSpan.storageState);
      EXPECT_EQ(lhsSpan.dataCategory, rhsSpan.dataCategory);
      EXPECT_EQ(lhsSpan.extentKind, rhsSpan.extentKind);
      EXPECT_EQ(lhsSpan.label, rhsSpan.label);
    }
    EXPECT_EQ(cursor, lhs.sizeBytes);
    for (std::size_t observationIndex = 0;
         observationIndex < lhs.observations.size();
         ++observationIndex) {
      const auto& lhsObservation = lhs.observations[observationIndex];
      const auto& rhsObservation = rhs.observations[observationIndex];
      EXPECT_EQ(lhsObservation.offsetBytes, rhsObservation.offsetBytes);
      EXPECT_EQ(lhsObservation.sizeBytes, rhsObservation.sizeBytes);
      EXPECT_EQ(lhsObservation.storageState, rhsObservation.storageState);
      EXPECT_EQ(lhsObservation.dataCategory, rhsObservation.dataCategory);
      EXPECT_EQ(lhsObservation.extentKind, rhsObservation.extentKind);
      EXPECT_EQ(lhsObservation.label, rhsObservation.label);
      ASSERT_LE(lhsObservation.offsetBytes, lhs.sizeBytes);
      EXPECT_LE(
          lhsObservation.sizeBytes, lhs.sizeBytes - lhsObservation.offsetBytes);
    }
    EXPECT_EQ(lhs.id.find("0x"), std::string::npos);
    EXPECT_EQ(lhs.label.find("0x"), std::string::npos);
    EXPECT_EQ(lhs.source.find("0x"), std::string::npos);
  }
  ASSERT_EQ(forward[0].spans.size(), 1u);
  EXPECT_EQ(forward[0].spans[0].offsetBytes, 0u);
  EXPECT_EQ(forward[0].spans[0].sizeBytes, forward[0].sizeBytes);
  EXPECT_EQ(forward[0].spans[0].extentKind, MemoryExtentKind::UnobservedRange);
  ASSERT_EQ(forward[0].observations.size(), 4u);
  EXPECT_NE(
      forward[0].label.find("overlapping observations:"), std::string::npos);

  const auto findObservation = [&forward](const std::string& label) {
    return std::find_if(
        forward[0].observations.begin(),
        forward[0].observations.end(),
        [&label](const MemoryMapSpan& span) { return span.label == label; });
  };
  const auto model = findObservation("model extent");
  const auto state = findObservation("state extent");
  const auto constraintPoint = findObservation("constraint point");
  const auto geometryPoint = findObservation("geometry point");
  ASSERT_NE(model, forward[0].observations.end());
  ASSERT_NE(state, forward[0].observations.end());
  ASSERT_NE(constraintPoint, forward[0].observations.end());
  ASSERT_NE(geometryPoint, forward[0].observations.end());
  EXPECT_EQ(model->offsetBytes, state->offsetBytes);
  EXPECT_EQ(model->sizeBytes, state->sizeBytes);
  EXPECT_EQ(constraintPoint->offsetBytes, geometryPoint->offsetBytes);
  EXPECT_EQ(constraintPoint->sizeBytes, 1u);
  EXPECT_EQ(constraintPoint->extentKind, MemoryExtentKind::AddressPoint);
  EXPECT_EQ(constraintPoint->quality, MetricQuality::Proxy);
  EXPECT_GE(constraintPoint->offsetBytes, model->offsetBytes);
  EXPECT_LT(
      constraintPoint->offsetBytes, model->offsetBytes + model->sizeBytes);

  HostPageSizeReading unavailable;
  unavailable.source = "synthetic unavailable query";
  unavailable.limitation = "unavailable by test";
  EXPECT_TRUE(makeObjectAddressAtlas(extents, unavailable).empty());

  const std::vector<AddressAtlasExtent> overflowing{makeExtent(
      std::numeric_limits<std::uintptr_t>::max() - 3u,
      8u,
      MemoryDataCategory::Model,
      "overflowing extent")};
  EXPECT_TRUE(makeObjectAddressAtlas(overflowing, pageSize).empty());
}

TEST(MemoryDiagnosticsModel, CellCompositionPreservesDenseBoundariesAndOverlays)
{
  MemoryMapRegion region;
  region.sizeBytes = 64u;
  const auto makeSpan = [](std::size_t offset,
                           std::size_t size,
                           MemoryStorageState state,
                           MemoryDataCategory category,
                           MemoryExtentKind extent,
                           std::string label) {
    return MemoryMapSpan{
        offset,
        size,
        state,
        category,
        extent,
        std::move(label),
        MetricQuality::Measured,
        "synthetic evidence"};
  };
  region.spans
      = {makeSpan(
             0u,
             3u,
             MemoryStorageState::Metadata,
             MemoryDataCategory::Metadata,
             MemoryExtentKind::ExactByteRange,
             "metadata"),
         makeSpan(
             3u,
             4u,
             MemoryStorageState::Allocated,
             MemoryDataCategory::Unknown,
             MemoryExtentKind::ExactByteRange,
             "allocated"),
         makeSpan(
             7u,
             9u,
             MemoryStorageState::Free,
             MemoryDataCategory::None,
             MemoryExtentKind::ExactByteRange,
             "free"),
         makeSpan(
             16u,
             1u,
             MemoryStorageState::Padding,
             MemoryDataCategory::None,
             MemoryExtentKind::ExactByteRange,
             "padding"),
         makeSpan(
             17u,
             47u,
             MemoryStorageState::Reserved,
             MemoryDataCategory::Scratch,
             MemoryExtentKind::ExactByteRange,
             "reserved")};
  region.observations
      = {makeSpan(
             2u,
             12u,
             MemoryStorageState::Observed,
             MemoryDataCategory::State,
             MemoryExtentKind::ShallowLowerBound,
             "state overlay"),
         makeSpan(
             3u,
             1u,
             MemoryStorageState::Observed,
             MemoryDataCategory::Model,
             MemoryExtentKind::AddressPoint,
             "model point"),
         makeSpan(
             3u,
             1u,
             MemoryStorageState::Observed,
             MemoryDataCategory::Geometry,
             MemoryExtentKind::AddressPoint,
             "geometry point")};

  const auto oneCell = composeMemoryMapCells(region, 64u);
  ASSERT_EQ(oneCell.size(), 1u);
  ASSERT_EQ(oneCell[0].segments.size(), 8u);
  EXPECT_TRUE(std::is_sorted(
      oneCell[0].segments.begin(),
      oneCell[0].segments.end(),
      [](const MemoryMapCellSegment& lhs, const MemoryMapCellSegment& rhs) {
        return lhs.offsetBytes < rhs.offsetBytes;
      }));
  EXPECT_EQ(
      std::count_if(
          oneCell[0].segments.begin(),
          oneCell[0].segments.end(),
          [](const MemoryMapCellSegment& segment) {
            return segment.layer == MemoryMapLayer::Partition;
          }),
      5);
  EXPECT_EQ(
      std::count_if(
          oneCell[0].segments.begin(),
          oneCell[0].segments.end(),
          [](const MemoryMapCellSegment& segment) {
            return segment.layer == MemoryMapLayer::Observation;
          }),
      3);

  const auto cells = composeMemoryMapCells(region, 8u);
  ASSERT_EQ(cells.size(), 8u);
  std::vector<std::size_t> partitionCoverage(region.spans.size(), 0u);
  std::vector<std::size_t> observationCoverage(region.observations.size(), 0u);
  for (const auto& cell : cells) {
    for (const auto& segment : cell.segments) {
      auto& coverage = segment.layer == MemoryMapLayer::Partition
                           ? partitionCoverage[segment.spanIndex]
                           : observationCoverage[segment.spanIndex];
      coverage += segment.sizeBytes;
    }
  }
  for (std::size_t index = 0; index < region.spans.size(); ++index) {
    EXPECT_EQ(partitionCoverage[index], region.spans[index].sizeBytes);
  }
  for (std::size_t index = 0; index < region.observations.size(); ++index) {
    EXPECT_EQ(observationCoverage[index], region.observations[index].sizeBytes);
  }
}

TEST(MemoryDiagnosticsModel, CellCompositionScalesAcrossDenseOverlays)
{
  constexpr std::size_t kRegionBytes = 4096u;
  constexpr std::size_t kObservationCount = 512u;
  MemoryMapRegion region;
  region.sizeBytes = kRegionBytes;
  region.spans.push_back(MemoryMapSpan{
      0u,
      kRegionBytes,
      MemoryStorageState::Unobserved,
      MemoryDataCategory::None,
      MemoryExtentKind::UnobservedRange,
      "background",
      MetricQuality::Proxy,
      "synthetic background"});
  region.observations.assign(
      kObservationCount,
      MemoryMapSpan{
          0u,
          kRegionBytes,
          MemoryStorageState::Observed,
          MemoryDataCategory::Model,
          MemoryExtentKind::ShallowLowerBound,
          "overlapping observation",
          MetricQuality::Estimate,
          "synthetic overlap"});

  const auto cells = composeMemoryMapCells(region, 64u);
  ASSERT_EQ(cells.size(), 64u);
  for (const auto& cell : cells) {
    ASSERT_EQ(cell.segments.size(), kObservationCount + 1u);
    EXPECT_EQ(cell.segments.front().layer, MemoryMapLayer::Partition);
    EXPECT_EQ(
        std::count_if(
            cell.segments.begin(),
            cell.segments.end(),
            [](const MemoryMapCellSegment& segment) {
              return segment.layer == MemoryMapLayer::Observation;
            }),
        static_cast<std::ptrdiff_t>(kObservationCount));
  }
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

  EXPECT_TRUE(summary.addressStatisticsAvailable);
  EXPECT_TRUE(summary.pageStatisticsAvailable);
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
  EXPECT_TRUE(singleton.addressStatisticsAvailable);
  EXPECT_TRUE(singleton.pageStatisticsAvailable);
  EXPECT_EQ(singleton.distinctPageCount, 1u);
  EXPECT_EQ(singleton.pageTransitions, 0u);
  EXPECT_FALSE(singleton.medianGapBytes);
  EXPECT_FALSE(singleton.p95GapBytes);

  const AddressLocalitySummary invalid = summarizeAddressLocality(addresses, 0);
  EXPECT_TRUE(invalid.addressStatisticsAvailable);
  EXPECT_FALSE(invalid.pageStatisticsAvailable);
  EXPECT_EQ(invalid.addressCount, addresses.size());
  EXPECT_EQ(invalid.distinctPageCount, 0u);
  EXPECT_EQ(invalid.pageTransitions, 0u);
  EXPECT_EQ(invalid.medianGapBytes, summary.medianGapBytes);
  EXPECT_EQ(invalid.p95GapBytes, summary.p95GapBytes);
  EXPECT_NE(
      invalid.limitation.find("page-bucket metrics are unavailable"),
      std::string::npos);
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
