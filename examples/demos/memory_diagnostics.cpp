/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "memory_diagnostics.hpp"

#include <dart/simulation/world.hpp>

#include <dart/common/memory_manager.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdlib>

namespace dart::examples::demos {

namespace {

constexpr double kBytesPerKibibyte = 1024.0;
constexpr double kBytesPerMebibyte = 1024.0 * kBytesPerKibibyte;
constexpr double kBytesPerGibibyte = 1024.0 * kBytesPerMebibyte;

double monotonicNowSeconds()
{
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

bool diagnosticsEnabledFromEnvironment()
{
  const char* const value = std::getenv("DART_DEMOS_MEMORY_DIAGNOSTICS");
  if (value == nullptr) {
    return false;
  }
  const std::string_view setting(value);
  return setting == "1" || setting == "true" || setting == "TRUE"
         || setting == "on" || setting == "ON";
}

void addMetric(
    DiagnosticSnapshot& snapshot,
    std::string key,
    std::string label,
    std::string unit,
    std::optional<double> value,
    MetricQuality quality,
    std::string scope,
    std::string source,
    std::string limitation,
    bool includeInHistory = false)
{
  DiagnosticMetric metric;
  metric.key = std::move(key);
  metric.label = std::move(label);
  metric.unit = std::move(unit);
  metric.value = value;
  metric.quality = quality;
  metric.scope = std::move(scope);
  metric.source = std::move(source);
  metric.limitation = std::move(limitation);
  metric.includeInHistory = includeInHistory;
  snapshot.metrics.push_back(std::move(metric));
}

void addCountMetric(
    DiagnosticSnapshot& snapshot,
    std::string key,
    std::string label,
    std::size_t value,
    std::string scope,
    std::string source,
    std::string limitation,
    bool includeInHistory = false,
    MetricQuality quality = MetricQuality::Measured)
{
  addMetric(
      snapshot,
      std::move(key),
      std::move(label),
      "count",
      static_cast<double>(value),
      quality,
      std::move(scope),
      std::move(source),
      std::move(limitation),
      includeInHistory);
}

void addBytesMetric(
    DiagnosticSnapshot& snapshot,
    std::string key,
    std::string label,
    std::optional<std::size_t> value,
    std::string scope,
    std::string source,
    std::string limitation,
    bool includeInHistory = false)
{
  addMetric(
      snapshot,
      std::move(key),
      std::move(label),
      "bytes",
      value ? std::optional<double>(static_cast<double>(*value)) : std::nullopt,
      MetricQuality::Measured,
      std::move(scope),
      std::move(source),
      std::move(limitation),
      includeInHistory);
}

void addRatioMetric(
    DiagnosticSnapshot& snapshot,
    std::string key,
    std::string label,
    std::size_t numerator,
    std::size_t denominator,
    MetricQuality quality,
    std::string scope,
    std::string source,
    std::string limitation,
    bool includeInHistory = false)
{
  const std::optional<double> value
      = denominator == 0u ? std::nullopt
                          : std::optional<double>(
                                static_cast<double>(numerator)
                                / static_cast<double>(denominator));
  addMetric(
      snapshot,
      std::move(key),
      std::move(label),
      "ratio",
      value,
      quality,
      std::move(scope),
      std::move(source),
      std::move(limitation),
      includeInHistory);
}

std::string storagePrefix(std::size_t storageId)
{
  return "ecs.storage." + std::to_string(storageId) + ".";
}

std::string storageLabel(
    const simulation::WorldEcsStorageDiagnostics& storage,
    std::string_view metric)
{
  return storage.diagnosticLabel + " [" + std::to_string(storage.storageId)
         + "] " + std::string(metric);
}

std::string formatNumber(double value, int precision)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string formatBytes(double value)
{
  const double magnitude = std::abs(value);
  if (magnitude >= kBytesPerGibibyte) {
    return formatNumber(value / kBytesPerGibibyte, 2) + " GiB";
  }
  if (magnitude >= kBytesPerMebibyte) {
    return formatNumber(value / kBytesPerMebibyte, 2) + " MiB";
  }
  if (magnitude >= kBytesPerKibibyte) {
    return formatNumber(value / kBytesPerKibibyte, 2) + " KiB";
  }
  return formatNumber(value, 0) + " B";
}

std::string formatValue(double value, std::string_view unit)
{
  if (unit == "bytes") {
    return formatBytes(value);
  }
  if (unit == "ratio") {
    return formatNumber(100.0 * value, 1) + "%";
  }
  if (unit == "boolean") {
    return value != 0.0 ? "yes" : "no";
  }
  if (unit == "seconds") {
    return formatNumber(value, 4) + " s";
  }
  if (unit == "count") {
    return formatNumber(value, 0);
  }
  return formatNumber(value, 3) + " " + std::string(unit);
}

std::string formatDelta(double delta, std::string_view unit)
{
  const char* const sign = delta > 0.0 ? "+" : "";
  if (unit == "ratio") {
    return std::string(sign) + formatNumber(100.0 * delta, 1) + " pp";
  }
  return std::string(sign) + formatValue(delta, unit);
}

const MetricDelta* findDelta(
    const SnapshotComparison* comparison, std::string_view key)
{
  if (comparison == nullptr || !comparison->generationMatches) {
    return nullptr;
  }
  const auto found = std::find_if(
      comparison->metrics.begin(),
      comparison->metrics.end(),
      [key](const MetricDelta& delta) { return delta.metric.key == key; });
  return found == comparison->metrics.end() ? nullptr : &*found;
}

const Eigen::Vector4d& memoryMapColor(MemoryMapCellKind kind)
{
  static const Eigen::Vector4d active(0.20, 0.68, 0.52, 1.0);
  static const Eigen::Vector4d hole(0.95, 0.55, 0.16, 1.0);
  static const Eigen::Vector4d reserved(0.30, 0.34, 0.40, 1.0);
  static const Eigen::Vector4d mixed(0.57, 0.44, 0.82, 1.0);
  switch (kind) {
    case MemoryMapCellKind::Active:
      return active;
    case MemoryMapCellKind::Hole:
      return hole;
    case MemoryMapCellKind::Reserved:
      return reserved;
    case MemoryMapCellKind::Mixed:
      return mixed;
  }
  return reserved;
}

std::string formatMapUnits(std::size_t value, std::string_view unit)
{
  if (unit == "bytes") {
    return formatBytes(static_cast<double>(value));
  }
  return std::to_string(value) + " " + std::string(unit);
}

void renderMemoryMaps(
    gui::PanelBuilder& builder, const DiagnosticSnapshot& snapshot)
{
  if (!builder.collapsingHeader("2D memory maps", true)) {
    return;
  }

  builder.text(
      "Logical capacity composition only; these blocks are not a process heap "
      "or address map.");
  builder.colorSwatch(
      "active / used", memoryMapColor(MemoryMapCellKind::Active));
  builder.sameLine();
  builder.colorSwatch("hole", memoryMapColor(MemoryMapCellKind::Hole));
  builder.sameLine();
  builder.colorSwatch(
      "reserved / unused", memoryMapColor(MemoryMapCellKind::Reserved));
  builder.sameLine();
  builder.colorSwatch("mixed bin", memoryMapColor(MemoryMapCellKind::Mixed));

  bool rendered = false;
  for (const MemoryMapRow& row : snapshot.memoryMaps) {
    if (row.cells.empty()) {
      continue;
    }

    rendered = true;
    const std::string summary
        = row.label + ": " + formatMapUnits(row.activeUnits, row.unit)
          + " live/used | " + formatMapUnits(row.holeUnits, row.unit)
          + " holes | " + formatMapUnits(row.reservedUnits, row.unit)
          + " reserved";
    std::vector<gui::PanelBlock> blocks;
    blocks.reserve(row.cells.size());
    for (const MemoryMapCell& cell : row.cells) {
      blocks.push_back(
          gui::PanelBlock{
              .rgba = memoryMapColor(cell.kind), .tooltip = cell.tooltip});
    }
    builder.blockGrid(summary + "##" + row.key, blocks, 32);
  }

  if (!rendered) {
    builder.text("No non-empty logical capacity maps are available.");
  }
}

void renderMetricSection(
    gui::PanelBuilder& builder,
    const DiagnosticSnapshot& snapshot,
    const SnapshotComparison* comparison,
    std::string_view heading,
    std::string_view keyPrefix,
    bool defaultOpen)
{
  if (!builder.collapsingHeader(heading, defaultOpen)) {
    return;
  }

  constexpr std::array<std::string_view, 4> columns{
      "Metric", "Value", "Evidence", "Baseline delta"};
  const std::string tableId = "memory_metrics_" + std::string(keyPrefix) + "##"
                              + std::string(heading);
  const bool tableOpen = builder.beginTable(tableId, columns);
  for (const DiagnosticMetric& metric : snapshot.metrics) {
    if (!metric.key.starts_with(keyPrefix)) {
      continue;
    }

    if (!tableOpen) {
      const std::string value = metric.value
                                    ? formatValue(*metric.value, metric.unit)
                                    : "unavailable";
      builder.text(metric.label + ": " + value);
      continue;
    }

    builder.tableNextRow();
    builder.tableNextColumn();
    builder.text(metric.label);
    builder.itemTooltip(metric.scope);

    builder.tableNextColumn();
    builder.text(
        metric.value ? formatValue(*metric.value, metric.unit) : "unavailable");

    builder.tableNextColumn();
    builder.text(
        metric.value ? metricQualityLabel(metric.quality) : "unavailable");
    builder.itemTooltip(metric.source + "\n" + metric.limitation);

    builder.tableNextColumn();
    const MetricDelta* const delta = findDelta(comparison, metric.key);
    builder.text(
        delta == nullptr ? "--" : formatDelta(delta->delta, metric.unit));
  }
  if (tableOpen) {
    builder.endTable();
  }
}

void renderHistory(gui::PanelBuilder& builder, const DiagnosticSession& session)
{
  if (!builder.collapsingHeader("History", true)) {
    return;
  }

  const auto renderSeries
      = [&builder, &session](
            std::string_view key, std::string_view label, double scale) {
          auto series = session.historySeries(key);
          if (!series || series->values.empty()) {
            return;
          }
          for (double& value : series->values) {
            value *= scale;
          }
          builder.plotLines(label, series->values);
        };

  renderSeries(
      kProcessResidentBytesKey, "Process RSS (MiB)", 1.0 / kBytesPerMebibyte);
  renderSeries(
      "scratch.used_bytes",
      "Frame scratch used (KiB)",
      1.0 / kBytesPerKibibyte);
  renderSeries(
      "ecs.aggregate.component_utilization",
      "ECS component-slot utilization (%)",
      100.0);
}

void renderPanelContents(
    gui::PanelBuilder& builder,
    DiagnosticSession& session,
    std::string_view sceneLabel)
{
  builder.text("Scene: " + std::string(sceneLabel));
  builder.text(
      "Whole-process and current-World diagnostics; categories overlap.");

  bool enabled = session.isEnabled();
  if (builder.checkbox("Enable memory diagnostics", enabled)) {
    session.setEnabled(enabled);
  }
  if (!session.isEnabled()) {
    builder.text(
        "Disabled: no OS query, World walk, history, or formatting sample.");
    builder.text("Enable to sample at a bounded cadence (default 0.5 s).");
    return;
  }

  double interval = session.sampleIntervalSeconds();
  if (builder.slider("Sample interval (s)", interval, 0.1, 5.0)) {
    session.setSampleIntervalSeconds(interval);
  }

  const double now = monotonicNowSeconds();
  bool sampledManually = false;
  bool reset = false;
  if (builder.button("Sample + set baseline")) {
    sampledManually = session.captureNow(now);
  }
  builder.sameLine();
  if (builder.button("Baseline = latest")) {
    session.captureLatestAsBaseline();
  }
  if (builder.button("Clear baseline")) {
    session.clearBaseline();
  }
  builder.sameLine();
  if (builder.button("Reset session")) {
    session.reset();
    reset = true;
  }

  if (!sampledManually && !reset) {
    session.update(now);
  }
  if (!session.latest()) {
    builder.text("Waiting for the first sample.");
    return;
  }

  const DiagnosticSnapshot& snapshot = *session.latest();
  const auto comparison = session.comparison();
  const SnapshotComparison* comparisonPtr = comparison ? &*comparison : nullptr;

  builder.separator();
  builder.text(
      "Frame " + std::to_string(snapshot.frame) + " | simulation time "
      + formatNumber(snapshot.simulationTimeSeconds, 4) + " s | samples "
      + std::to_string(session.historySize()) + "/"
      + std::to_string(session.historyCapacity()));
  if (session.observedResidentPeakBytes()) {
    builder.text(
        "Session-observed RSS peak: "
        + formatBytes(*session.observedResidentPeakBytes()));
  }
  if (comparisonPtr != nullptr && !comparisonPtr->generationMatches) {
    builder.text(
        "Baseline belongs to another World generation; deltas hidden.");
  } else if (!session.baseline()) {
    builder.text("No baseline selected; delta column is inactive.");
  }

  renderMemoryMaps(builder, snapshot);
  renderHistory(builder, session);
  renderMetricSection(
      builder, snapshot, comparisonPtr, "Process memory", "process.", true);
  renderMetricSection(
      builder,
      snapshot,
      comparisonPtr,
      "World allocator categories",
      "allocator.",
      true);
  renderMetricSection(
      builder, snapshot, comparisonPtr, "Frame scratch", "scratch.", true);
  renderMetricSection(
      builder, snapshot, comparisonPtr, "World object counts", "world.", false);
  renderMetricSection(
      builder,
      snapshot,
      comparisonPtr,
      "ECS aggregate",
      "ecs.aggregate.",
      true);
  renderMetricSection(
      builder,
      snapshot,
      comparisonPtr,
      "EnTT storage layout",
      "ecs.storage.",
      false);

  if (builder.collapsingHeader("Interpretation and guidance", true)) {
    for (const std::string& guidance : snapshot.guidance) {
      builder.text("- " + guidance);
    }
  }
}

} // namespace

DiagnosticSnapshot collectMemoryDiagnostics(
    const simulation::World& world, std::uint64_t generation)
{
  DiagnosticSnapshot snapshot;
  snapshot.engine = "DART 7 EnTT World";
  snapshot.generation = generation;
  snapshot.frame = world.getFrame();
  snapshot.simulationTimeSeconds = world.getTime();

  const ProcessMemoryReading process = collectProcessMemory();
  snapshot.platform = process.platform;
  snapshot.metrics = makeProcessMemoryMetrics(process);

  constexpr std::string_view objectSource = "DART 7 World public count APIs";
  constexpr std::string_view objectLimitation
      = "A live object count is not a byte estimate or allocation-ownership "
        "measurement.";
  addCountMetric(
      snapshot,
      "world.rigid_bodies",
      "Rigid bodies",
      world.getRigidBodyCount(),
      "current World",
      std::string(objectSource),
      std::string(objectLimitation));
  addCountMetric(
      snapshot,
      "world.deformable_bodies",
      "Deformable bodies",
      world.getDeformableBodyCount(),
      "current World",
      std::string(objectSource),
      std::string(objectLimitation));
  addCountMetric(
      snapshot,
      "world.multibodies",
      "Multibodies",
      world.getMultibodyCount(),
      "current World",
      std::string(objectSource),
      std::string(objectLimitation));
  addCountMetric(
      snapshot,
      "world.joints",
      "World-added non-topology joints",
      world.getJointCount(),
      "current World",
      std::string(objectSource),
      "Counts joints created through World::addJoint; multibody tree parent "
      "joints are excluded. A live object count is not byte attribution.");
  addCountMetric(
      snapshot,
      "world.loop_closures",
      "Loop closures",
      world.getLoopClosureCount(),
      "current World",
      std::string(objectSource),
      std::string(objectLimitation));
  const simulation::WorldMemoryDiagnostics memory = world.getMemoryDiagnostics(
      simulation::WorldMemoryDiagnosticsOptions{
          .includeStorageLayoutDetails = true});
  const common::MemoryManager& manager = world.getMemoryManager();
  const common::FreeListAllocator& freeAllocator
      = manager.getFreeListAllocator();
  const common::PoolAllocator& poolAllocator = manager.getPoolAllocator();

  constexpr std::string_view freeSource
      = "World MemoryManager FreeListAllocator counters";
  constexpr std::string_view freeLimitation
      = "User-requested bytes exclude allocator metadata and "
        "reserved-but-unused "
        "backing space. Child allocator backing can overlap other rows; do not "
        "sum this with process RSS.";
  addBytesMetric(
      snapshot,
      "allocator.free.live_bytes",
      "Free-list live requested bytes",
      freeAllocator.getAllocatedSize(),
      "current World free-list hierarchy",
      std::string(freeSource),
      std::string(freeLimitation),
      true);
  addBytesMetric(
      snapshot,
      "allocator.free.peak_bytes",
      "Free-list peak requested bytes",
      freeAllocator.getPeakAllocatedSize(),
      "current World free-list lifetime",
      std::string(freeSource),
      std::string(freeLimitation));
  addCountMetric(
      snapshot,
      "allocator.free.live_allocations",
      "Free-list live allocations",
      freeAllocator.getAllocationCount(),
      "current World free-list hierarchy",
      std::string(freeSource),
      std::string(freeLimitation));

  constexpr std::string_view poolSource
      = "World MemoryManager PoolAllocator counters";
  const std::string poolLimitation
      = poolAllocator.isDiagnosticsEnabled()
            ? "User-requested bytes exclude pool metadata and unused units; "
              "the pool can overlap free-list backing and process RSS."
            : "Live-byte and live-allocation counters are disabled for this "
              "pool configuration to avoid hot-path instrumentation overhead.";
  const auto optionalPoolValue = [&poolAllocator](std::size_t value) {
    return poolAllocator.isDiagnosticsEnabled()
               ? std::optional<std::size_t>(value)
               : std::nullopt;
  };
  addBytesMetric(
      snapshot,
      "allocator.pool.live_bytes",
      "Pool live requested bytes",
      optionalPoolValue(poolAllocator.getAllocatedSize()),
      "current World pool",
      std::string(poolSource),
      poolLimitation,
      true);
  addBytesMetric(
      snapshot,
      "allocator.pool.peak_bytes",
      "Pool peak requested bytes",
      optionalPoolValue(poolAllocator.getPeakAllocatedSize()),
      "current World pool lifetime",
      std::string(poolSource),
      poolLimitation);
  addMetric(
      snapshot,
      "allocator.pool.live_allocations",
      "Pool live allocations",
      "count",
      poolAllocator.isDiagnosticsEnabled()
          ? std::optional<double>(
                static_cast<double>(poolAllocator.getAllocationCount()))
          : std::nullopt,
      MetricQuality::Measured,
      "current World pool",
      std::string(poolSource),
      poolLimitation);
  addCountMetric(
      snapshot,
      "allocator.pool.backing_blocks",
      "Pool allocated backing blocks",
      static_cast<std::size_t>(poolAllocator.getNumAllocatedMemoryBlocks()),
      "current World pool",
      std::string(poolSource),
      "Backing-block count is not a byte total and does not report unused "
      "units.");

  constexpr std::string_view scratchSource
      = "WorldMemoryDiagnostics frame allocator counters";
  constexpr std::string_view scratchLimitation
      = "Frame scratch is one World-owned arena category; it can overlap "
        "allocator backing rows and does not include all transient process "
        "memory.";
  addBytesMetric(
      snapshot,
      "scratch.used_bytes",
      "Current frame scratch used",
      memory.frameScratchUsedBytes,
      "current World frame scratch",
      std::string(scratchSource),
      std::string(scratchLimitation),
      true);
  addBytesMetric(
      snapshot,
      "scratch.capacity_bytes",
      "Frame scratch arena capacity",
      memory.frameScratchCapacityBytes,
      "current World frame scratch",
      std::string(scratchSource),
      std::string(scratchLimitation));
  addBytesMetric(
      snapshot,
      "scratch.peak_used_bytes",
      "Peak frame scratch used",
      memory.frameScratchPeakUsedBytes,
      "current World since construction or clear",
      std::string(scratchSource),
      std::string(scratchLimitation));
  addCountMetric(
      snapshot,
      "scratch.overflow_count",
      "Current overflow allocations",
      memory.frameScratchOverflowCount,
      "current World frame scratch",
      std::string(scratchSource),
      std::string(scratchLimitation),
      true);
  addBytesMetric(
      snapshot,
      "scratch.overflow_bytes",
      "Current overflow bytes",
      memory.frameScratchOverflowBytes,
      "current World frame scratch",
      std::string(scratchSource),
      std::string(scratchLimitation),
      true);
  addCountMetric(
      snapshot,
      "scratch.reset_count",
      "Frame scratch resets",
      memory.frameScratchResetCount,
      "current World lifetime",
      std::string(scratchSource),
      "Reset count is a lifetime event counter, not allocated memory.");
  const std::size_t scratchArenaUsed
      = memory.frameScratchUsedBytes > memory.frameScratchOverflowBytes
            ? memory.frameScratchUsedBytes - memory.frameScratchOverflowBytes
            : 0u;
  const std::size_t boundedScratchArenaUsed
      = std::min(scratchArenaUsed, memory.frameScratchCapacityBytes);
  if (memory.frameScratchCapacityBytes > 0u) {
    snapshot.memoryMaps.push_back(makeMemoryMapRow(
        "scratch.arena",
        "Frame scratch reservation arena",
        "bytes",
        boundedScratchArenaUsed,
        0u,
        memory.frameScratchCapacityBytes - boundedScratchArenaUsed,
        "current World frame-scratch reservation arena",
        std::string(scratchSource),
        "Blocks aggregate arena bytes into bounded display bins. They do not "
        "show individual allocations or addresses; overflow bytes are excluded "
        "from this row and reported separately."));
  }

  const simulation::WorldEcsDiagnostics& ecs = memory.ecsDiagnostics;
  constexpr std::string_view ecsSource
      = "WorldMemoryDiagnostics type-erased EnTT storage counters";
  constexpr std::string_view ecsLimitation
      = "Capacities are slots, not bytes. Component payload types and sizes "
        "remain behind the World implementation boundary.";
  addCountMetric(
      snapshot,
      "ecs.aggregate.entities_live",
      "Live entities",
      ecs.entityCount,
      "current World entity storage",
      std::string(ecsSource),
      std::string(ecsLimitation),
      true);
  addCountMetric(
      snapshot,
      "ecs.aggregate.entity_capacity",
      "Entity-slot capacity",
      ecs.entityCapacity,
      "current World entity storage",
      std::string(ecsSource),
      std::string(ecsLimitation));
  addCountMetric(
      snapshot,
      "ecs.aggregate.entity_spare_slots",
      "Spare entity slots",
      ecs.entityCapacity > ecs.entityCount
          ? ecs.entityCapacity - ecs.entityCount
          : 0u,
      "current World entity storage",
      std::string(ecsSource),
      std::string(ecsLimitation));
  addRatioMetric(
      snapshot,
      "ecs.aggregate.entity_utilization",
      "Entity-slot utilization",
      ecs.entityCount,
      ecs.entityCapacity,
      MetricQuality::Measured,
      "current World entity storage",
      std::string(ecsSource),
      std::string(ecsLimitation),
      true);
  addCountMetric(
      snapshot,
      "ecs.aggregate.storages",
      "Materialized component storages",
      ecs.storageCount,
      "current World component storages",
      std::string(ecsSource),
      std::string(ecsLimitation));
  addCountMetric(
      snapshot,
      "ecs.aggregate.components_live",
      "Live components",
      ecs.componentCount,
      "current World component storages",
      std::string(ecsSource),
      std::string(ecsLimitation),
      true);
  addCountMetric(
      snapshot,
      "ecs.aggregate.component_capacity",
      "Component-slot capacity",
      ecs.componentCapacity,
      "current World component storages",
      std::string(ecsSource),
      std::string(ecsLimitation));
  addCountMetric(
      snapshot,
      "ecs.aggregate.component_spare_slots",
      "Spare component slots",
      ecs.componentCapacity > ecs.componentCount
          ? ecs.componentCapacity - ecs.componentCount
          : 0u,
      "current World component storages",
      std::string(ecsSource),
      std::string(ecsLimitation));
  addRatioMetric(
      snapshot,
      "ecs.aggregate.component_utilization",
      "Component-slot utilization",
      ecs.componentCount,
      ecs.componentCapacity,
      MetricQuality::Measured,
      "current World component storages",
      std::string(ecsSource),
      std::string(ecsLimitation),
      true);

  std::vector<simulation::WorldEcsStorageDiagnostics> storages = ecs.storages;
  std::sort(
      storages.begin(), storages.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.storageId < rhs.storageId;
      });
  std::size_t totalHoles = 0;
  for (const simulation::WorldEcsStorageDiagnostics& storage : storages) {
    totalHoles += storage.holeCount;
    const std::string prefix = storagePrefix(storage.storageId);
    const std::string scope
        = "current World storage token " + std::to_string(storage.storageId);
    const std::string limitation
        = "The storage token is internal and not stable across builds. Slot "
          "layout does not expose component bytes or measure cache behavior.";
    if (ecs.storageLayoutDetailsIncluded && storage.capacity > 0u) {
      snapshot.memoryMaps.push_back(makeMemoryMapRow(
          prefix + "capacity_map",
          storage.diagnosticLabel,
          "slots",
          storage.size,
          storage.holeCount,
          storage.unusedCapacity,
          scope,
          std::string(ecsSource),
          "Cells are grouped by live, hole, and reserved state. They do not "
          "reproduce packed-slot order, component payload adjacency, virtual "
          "addresses, or physical allocation layout."));
    }
    addCountMetric(
        snapshot,
        prefix + "live_slots",
        storageLabel(storage, "live slots"),
        storage.size,
        scope,
        std::string(ecsSource),
        limitation);
    addCountMetric(
        snapshot,
        prefix + "packed_slots",
        storageLabel(storage, "materialized packed slots"),
        storage.packedSlotCount,
        scope,
        std::string(ecsSource),
        limitation);
    addCountMetric(
        snapshot,
        prefix + "capacity",
        storageLabel(storage, "capacity"),
        storage.capacity,
        scope,
        std::string(ecsSource),
        limitation);
    addCountMetric(
        snapshot,
        prefix + "unused_capacity",
        storageLabel(storage, "unused reserved slots"),
        storage.unusedCapacity,
        scope,
        std::string(ecsSource),
        limitation);
    addCountMetric(
        snapshot,
        prefix + "holes",
        storageLabel(storage, "holes"),
        storage.holeCount,
        scope,
        std::string(ecsSource),
        limitation);
    addCountMetric(
        snapshot,
        prefix + "live_regions",
        storageLabel(storage, "live packed regions"),
        storage.livePackedRegionCount,
        scope,
        std::string(ecsSource),
        limitation,
        false,
        MetricQuality::Proxy);
    addCountMetric(
        snapshot,
        prefix + "sparse_extent",
        storageLabel(storage, "sparse index extent"),
        storage.sparseExtent,
        scope,
        std::string(ecsSource),
        "Sparse extent is entity index space, not allocated sparse pages or "
        "bytes.");
    addRatioMetric(
        snapshot,
        prefix + "capacity_utilization",
        storageLabel(storage, "capacity utilization"),
        storage.size,
        storage.capacity,
        MetricQuality::Measured,
        scope,
        std::string(ecsSource),
        limitation);
    addRatioMetric(
        snapshot,
        prefix + "packed_density",
        storageLabel(storage, "live packed-slot density"),
        storage.size,
        storage.packedSlotCount,
        MetricQuality::Proxy,
        scope,
        std::string(ecsSource),
        limitation);
    addMetric(
        snapshot,
        prefix + "packed_contiguous",
        storageLabel(storage, "hole-free packed slots"),
        "boolean",
        storage.packedContiguous ? 1.0 : 0.0,
        MetricQuality::Proxy,
        scope,
        std::string(ecsSource),
        limitation);
  }

  snapshot.guidance.push_back(
      "Allocator, scratch, ECS-slot, and process rows have different scopes "
      "and can overlap; do not add them into a total.");
  snapshot.guidance.push_back(
      "Process peak is OS process-lifetime state. Reset clears only the "
      "diagnostics session, baseline, history, and session-observed peak.");
  snapshot.guidance.push_back(
      "EnTT storage tokens are for grouping the current sample only; packed "
      "slot layout does not establish component-payload adjacency.");
  if (!poolAllocator.isDiagnosticsEnabled()) {
    snapshot.guidance.push_back(
        "Pool live-byte counters are unavailable because pool diagnostics are "
        "disabled; enable instrumentation in an appropriate diagnostic build "
        "instead of interpreting unavailable as zero.");
  }
  if (memory.frameScratchOverflowCount > 0u) {
    snapshot.guidance.push_back(
        "Frame scratch overflowed its arena in this frame. Confirm the pattern "
        "over several samples before changing the reserved capacity.");
  }
  if (totalHoles > 0u) {
    snapshot.guidance.push_back(
        "One or more EnTT packed entity arrays contain holes. Treat this as a "
        "layout lead and profile the affected workload before considering "
        "compaction or deletion-policy changes.");
  }
  if (ecs.componentCapacity > 0u
      && 2u * ecs.componentCount < ecs.componentCapacity) {
    snapshot.guidance.push_back(
        "Less than half of reserved component slots are live. Compare "
        "snapshots "
        "around scene rebuilds before deciding whether retained capacity "
        "matters.");
  }
  appendGenericMemoryGuidance(snapshot);
  return snapshot;
}

gui::Panel createMemoryDiagnosticsPanel(
    std::string sceneLabel,
    std::function<const simulation::World*()> worldProvider)
{
  static std::atomic<std::uint64_t> nextGeneration{1};
  const std::uint64_t generation
      = nextGeneration.fetch_add(1, std::memory_order_relaxed);

  auto session = std::make_shared<DiagnosticSession>([worldProvider = std::move(
                                                          worldProvider),
                                                      generation]() {
    const simulation::World* const world = worldProvider();
    if (world != nullptr) {
      return collectMemoryDiagnostics(*world, generation);
    }
    DiagnosticSnapshot unavailable;
    unavailable.engine = "DART 7 EnTT World";
    unavailable.platform = currentProcessPlatform();
    unavailable.generation = generation;
    unavailable.guidance.push_back(
        "The scene World is unavailable, so no World metrics were sampled.");
    return unavailable;
  });
  session->setEnabled(diagnosticsEnabledFromEnvironment());

  gui::Panel panel;
  panel.title = "Memory Diagnostics";
  panel.initialSize = std::array<double, 2>{640.0, 640.0};
  panel.autoResize = false;
  panel.horizontalScrollbar = true;
  panel.dockSide = gui::DockSide::Right;
  panel.build = [session, sceneLabel = std::move(sceneLabel)](
                    gui::PanelBuilder& builder) {
    renderPanelContents(builder, *session, sceneLabel);
  };
  return panel;
}

} // namespace dart::examples::demos
