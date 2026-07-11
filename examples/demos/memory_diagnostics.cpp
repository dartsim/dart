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
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 *   IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "memory_diagnostics.hpp"

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionGroup.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>

#include <dart/common/FrameAllocator.hpp>
#include <dart/common/MemoryManager.hpp>
#include <dart/common/PoolAllocator.hpp>

#include <imgui.h>

#include <algorithm>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::examples::demos {

namespace {

//==============================================================================
void addMetric(
    DiagnosticSnapshot& snapshot,
    std::string key,
    std::string label,
    std::string unit,
    std::optional<double> value,
    MetricQuality quality,
    std::string scope,
    std::string source,
    std::string limitation = {},
    bool includeInHistory = false)
{
  snapshot.metrics.push_back(DiagnosticMetric{
      std::move(key),
      std::move(label),
      std::move(unit),
      value,
      quality,
      std::move(scope),
      std::move(source),
      std::move(limitation),
      includeInHistory});
}

//==============================================================================
void addUnavailableAllocationMetrics(DiagnosticSnapshot& snapshot)
{
  const std::string source = "No global allocation instrumentation";
  const std::string limitation
      = "DART 6 does not interpose global new/delete or malloc in the normal "
        "demos executable.";
  const std::string scope = "whole process and legacy DART object graph";

  addMetric(
      snapshot,
      "allocation.active_count",
      "Active allocations",
      "count",
      std::nullopt,
      MetricQuality::Measured,
      scope,
      source,
      limitation);
  addMetric(
      snapshot,
      "allocation.total_call_count",
      "Allocation calls",
      "count",
      std::nullopt,
      MetricQuality::Measured,
      scope,
      source,
      limitation);
  addMetric(
      snapshot,
      "allocation.active_bytes",
      "DART-owned active heap bytes",
      "bytes",
      std::nullopt,
      MetricQuality::Measured,
      scope,
      source,
      "Whole-process RSS cannot be attributed to DART-owned live allocations "
      "without intrusive allocation tracking.");
}

//==============================================================================
void addCountMetric(
    DiagnosticSnapshot& snapshot,
    const char* key,
    const char* label,
    std::size_t value,
    const char* source)
{
  addMetric(
      snapshot,
      key,
      label,
      "count",
      static_cast<double>(value),
      MetricQuality::Measured,
      "active DART 6 World",
      source);
}

//==============================================================================
void addAddress(
    std::vector<std::uintptr_t>& addresses, const void* address) noexcept
{
  if (address) {
    addresses.push_back(reinterpret_cast<std::uintptr_t>(address));
  }
}

} // namespace

//==============================================================================
DiagnosticSnapshot collectMemoryDiagnostics(
    const simulation::WorldPtr& world,
    std::uint64_t generation,
    const ProcessMemoryReading& processMemory)
{
  DiagnosticSnapshot snapshot;
  snapshot.engine = "DART 6";
  snapshot.platform = processMemory.platform.empty() ? currentProcessPlatform()
                                                     : processMemory.platform;
  snapshot.generation = generation;

  auto processMetrics = makeProcessMemoryMetrics(processMemory);
  snapshot.metrics.insert(
      snapshot.metrics.end(),
      std::make_move_iterator(processMetrics.begin()),
      std::make_move_iterator(processMetrics.end()));

  addCountMetric(
      snapshot,
      "world.count",
      "Worlds sampled",
      world ? 1u : 0u,
      "active demo host");

  if (!world) {
    addUnavailableAllocationMetrics(snapshot);
    snapshot.guidance.push_back(
        "No active DART 6 World is installed; only whole-process metrics are "
        "available.");
    appendGenericMemoryGuidance(snapshot);
    return snapshot;
  }

  const int simFrames = world->getSimFrames();
  snapshot.frame = simFrames > 0 ? static_cast<std::uint64_t>(simFrames) : 0u;
  snapshot.simulationTimeSeconds = world->getTime();

  addMetric(
      snapshot,
      "world.frame",
      "World frame",
      "count",
      static_cast<double>(snapshot.frame),
      MetricQuality::Measured,
      "active DART 6 World",
      "World::getSimFrames");
  addMetric(
      snapshot,
      "world.simulation_time",
      "Simulation time",
      "seconds",
      snapshot.simulationTimeSeconds,
      MetricQuality::Measured,
      "active DART 6 World",
      "World::getTime");

  std::size_t skeletonCount = world->getNumSkeletons();
  std::size_t bodyNodeCount = 0;
  std::size_t softBodyCount = 0;
  std::size_t pointMassCount = 0;
  std::size_t jointCount = 0;
  std::size_t dofCount = 0;
  std::size_t shapeNodeCount = 0;
  const std::size_t simpleFrameCount = world->getNumSimpleFrames();
  std::unordered_set<const dynamics::Shape*> uniqueShapes;
  std::vector<std::uintptr_t> objectAddresses;
  double shallowSizeFloorBytes = 0.0;

  objectAddresses.reserve(skeletonCount * 8u);
  for (std::size_t skeletonIndex = 0; skeletonIndex < skeletonCount;
       ++skeletonIndex) {
    const auto& skeleton = world->getSkeleton(skeletonIndex);
    if (!skeleton) {
      continue;
    }

    addAddress(objectAddresses, skeleton.get());
    shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::Skeleton));

    bodyNodeCount += skeleton->getNumBodyNodes();
    jointCount += skeleton->getNumJoints();
    dofCount += skeleton->getNumDofs();

    for (std::size_t jointIndex = 0; jointIndex < skeleton->getNumJoints();
         ++jointIndex) {
      const auto* joint = skeleton->getJoint(jointIndex);
      addAddress(objectAddresses, joint);
      if (joint) {
        shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::Joint));
      }
    }

    for (std::size_t dofIndex = 0; dofIndex < skeleton->getNumDofs();
         ++dofIndex) {
      const auto* dof = skeleton->getDof(dofIndex);
      addAddress(objectAddresses, dof);
      if (dof) {
        shallowSizeFloorBytes
            += static_cast<double>(sizeof(dynamics::DegreeOfFreedom));
      }
    }

    for (std::size_t bodyIndex = 0; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      const auto* body = skeleton->getBodyNode(bodyIndex);
      addAddress(objectAddresses, body);
      if (!body) {
        continue;
      }

      shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::BodyNode));
      const auto* softBody = body->asSoftBodyNode();
      if (softBody) {
        ++softBodyCount;
        if (sizeof(dynamics::SoftBodyNode) > sizeof(dynamics::BodyNode)) {
          shallowSizeFloorBytes += static_cast<double>(
              sizeof(dynamics::SoftBodyNode) - sizeof(dynamics::BodyNode));
        }
        pointMassCount += softBody->getNumPointMasses();
        for (std::size_t pointIndex = 0;
             pointIndex < softBody->getNumPointMasses();
             ++pointIndex) {
          const auto* pointMass = softBody->getPointMass(pointIndex);
          addAddress(objectAddresses, pointMass);
          if (pointMass) {
            shallowSizeFloorBytes
                += static_cast<double>(sizeof(dynamics::PointMass));
          }
        }
      }

      shapeNodeCount += body->getNumShapeNodes();
      for (std::size_t shapeIndex = 0; shapeIndex < body->getNumShapeNodes();
           ++shapeIndex) {
        const auto* shapeNode = body->getShapeNode(shapeIndex);
        addAddress(objectAddresses, shapeNode);
        if (!shapeNode) {
          continue;
        }

        shallowSizeFloorBytes
            += static_cast<double>(sizeof(dynamics::ShapeNode));
        const auto shape = shapeNode->getShape();
        if (shape && uniqueShapes.insert(shape.get()).second) {
          addAddress(objectAddresses, shape.get());
          shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::Shape));
        }
      }
    }
  }

  for (std::size_t frameIndex = 0; frameIndex < simpleFrameCount;
       ++frameIndex) {
    const auto& frame = world->getSimpleFrame(frameIndex);
    if (!frame) {
      continue;
    }
    addAddress(objectAddresses, frame.get());
    shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::SimpleFrame));
    const auto shape = frame->getShape();
    if (shape && uniqueShapes.insert(shape.get()).second) {
      addAddress(objectAddresses, shape.get());
      shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::Shape));
    }
  }

  const std::size_t rigidBodyCount = bodyNodeCount - softBodyCount;
  addCountMetric(
      snapshot,
      "world.skeleton_count",
      "Skeletons",
      skeletonCount,
      "World/Skeleton traversal");
  addCountMetric(
      snapshot,
      "world.body_node_count",
      "BodyNodes",
      bodyNodeCount,
      "World/Skeleton traversal");
  addCountMetric(
      snapshot,
      "world.rigid_body_node_count",
      "Rigid BodyNodes",
      rigidBodyCount,
      "BodyNodes excluding SoftBodyNodes");
  addCountMetric(
      snapshot,
      "world.soft_body_node_count",
      "SoftBodyNodes",
      softBodyCount,
      "World/Skeleton traversal");
  addCountMetric(
      snapshot,
      "world.point_mass_count",
      "PointMasses",
      pointMassCount,
      "SoftBodyNode traversal");
  addCountMetric(
      snapshot,
      "world.joint_count",
      "Joints",
      jointCount,
      "Skeleton traversal");
  addCountMetric(
      snapshot,
      "world.dof_count",
      "Degrees of freedom",
      dofCount,
      "Skeleton traversal");
  addCountMetric(
      snapshot,
      "world.shape_node_count",
      "ShapeNodes",
      shapeNodeCount,
      "BodyNode traversal");
  addCountMetric(
      snapshot,
      "world.simple_frame_count",
      "SimpleFrames",
      simpleFrameCount,
      "World::getNumSimpleFrames");
  addCountMetric(
      snapshot,
      "world.unique_shape_count",
      "Unique Shape objects",
      uniqueShapes.size(),
      "Shape pointer identity within this sample");

  const auto* constraintSolver = world->getConstraintSolver();
  const std::size_t contactCount
      = world->getLastCollisionResult().getNumContacts();
  addCountMetric(
      snapshot,
      "world.last_contact_count",
      "Last-step contacts",
      contactCount,
      "World::getLastCollisionResult");

  if (constraintSolver) {
    addCountMetric(
        snapshot,
        "world.manual_constraint_count",
        "Manually registered constraints",
        constraintSolver->getNumConstraints(),
        "ConstraintSolver::getNumConstraints");

    const auto collisionGroup = constraintSolver->getCollisionGroup();
    addMetric(
        snapshot,
        "world.collision_shape_frame_count",
        "Collision-group ShapeFrames",
        "count",
        collisionGroup ? std::optional<double>(
            static_cast<double>(collisionGroup->getNumShapeFrames()))
                       : std::nullopt,
        MetricQuality::Measured,
        "active constraint solver collision group",
        "CollisionGroup::getNumShapeFrames",
        "This is collision-group membership, not the total ShapeNode count.");
  } else {
    addMetric(
        snapshot,
        "world.manual_constraint_count",
        "Manually registered constraints",
        "count",
        std::nullopt,
        MetricQuality::Measured,
        "active DART 6 World",
        "ConstraintSolver",
        "The World has no constraint solver.");
    addMetric(
        snapshot,
        "world.collision_shape_frame_count",
        "Collision-group ShapeFrames",
        "count",
        std::nullopt,
        MetricQuality::Measured,
        "active DART 6 World",
        "ConstraintSolver collision group",
        "The World has no constraint solver.");
  }

  auto& memoryManager = world->getMemoryManager();
  const auto& frameAllocator = memoryManager.getFrameAllocator();
  const auto& poolAllocator = memoryManager.getPoolAllocator();
  const std::string frameScope = "active World MemoryManager reservation arena";
  const std::string frameSource = "World MemoryManager FrameAllocator";
  const std::string frameLimitation
      = "World reserves and resets this arena, but classic DART 6 solver "
        "scratch does not allocate from it; these counters do not measure "
        "legacy solver temporaries.";
  addMetric(
      snapshot,
      "world.scratch.frame_capacity_bytes",
      "Reserved frame arena capacity",
      "bytes",
      static_cast<double>(frameAllocator.capacity()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      frameLimitation,
      true);
  addMetric(
      snapshot,
      "world.scratch.frame_usable_bytes",
      "Reserved frame arena usable capacity",
      "bytes",
      static_cast<double>(frameAllocator.usableCapacity()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      frameLimitation);
  addMetric(
      snapshot,
      "world.scratch.frame_used_bytes",
      "Reserved frame arena used",
      "bytes",
      static_cast<double>(frameAllocator.used()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      "The arena resets at the start of each World::step; this is the current "
      "post-reset interval, not a lifetime peak. "
          + frameLimitation,
      true);
  addMetric(
      snapshot,
      "world.scratch.frame_overflow_count",
      "Reserved frame arena overflow allocations",
      "count",
      static_cast<double>(frameAllocator.overflowCount()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      frameLimitation);
  addMetric(
      snapshot,
      "world.scratch.frame_overflow_bytes",
      "Reserved frame arena overflow bytes",
      "bytes",
      static_cast<double>(frameAllocator.overflowBytes()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      frameLimitation,
      true);
  addMetric(
      snapshot,
      "world.scratch.pool_block_count",
      "MemoryManager pool backing blocks",
      "count",
      static_cast<double>(poolAllocator.getNumAllocatedMemoryBlocks()),
      MetricQuality::Measured,
      "active World MemoryManager pool reservation",
      "World MemoryManager PoolAllocator",
      "This allocator does not own Skeleton, BodyNode, Joint, or Shape "
      "objects and does not measure classic DART 6 solver scratch.");

  addMetric(
      snapshot,
      "world.object_shallow_floor_bytes",
      "Known object shallow-size floor",
      "bytes",
      shallowSizeFloorBytes,
      MetricQuality::Estimate,
      "traversed DART 6 object graph",
      "object counts multiplied by sizeof(base or known concrete type)",
      "Excludes derived-class tails beyond the counted base, container "
      "capacity, shared-pointer control blocks, collision-backend objects, "
      "alignment, and other nested allocations.",
      true);

  const auto locality = summarizeAddressLocality(objectAddresses);
  const std::string localityScope = "traversed DART 6 object addresses";
  addMetric(
      snapshot,
      "world.locality.address_count",
      "Object addresses sampled",
      "count",
      locality.available
          ? std::optional<double>(static_cast<double>(locality.addressCount))
          : std::nullopt,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.address_page_count",
      "Distinct 4 KiB address buckets",
      "count",
      locality.available ? std::optional<double>(
          static_cast<double>(locality.distinctPageCount))
                         : std::nullopt,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.page_transitions",
      "Address-order page transitions",
      "count",
      locality.available
          ? std::optional<double>(static_cast<double>(locality.pageTransitions))
          : std::nullopt,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.median_gap_bytes",
      "Median adjacent address gap",
      "bytes",
      locality.medianGapBytes,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.p95_gap_bytes",
      "P95 adjacent address gap",
      "bytes",
      locality.p95GapBytes,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);

  addUnavailableAllocationMetrics(snapshot);
  snapshot.guidance.push_back(
      "DART 6 graph counts are measured from the active World. Its "
      "MemoryManager rows cover a reserved/reset arena, not the legacy object "
      "graph or classic solver scratch.");
  snapshot.guidance.push_back(
      "Manually registered constraints exclude contact and other constraints "
      "generated internally for the current solve.");
  snapshot.guidance.push_back(
      "Address buckets and gaps are allocation-layout proxies. They do not "
      "measure physical-page residency, cache misses, or iteration speed.");
  appendGenericMemoryGuidance(snapshot);
  return snapshot;
}

} // namespace dart::examples::demos

namespace dart::examples::demos {

namespace {

//==============================================================================
std::string formatBytes(double bytes, bool signedValue)
{
  constexpr double kKibibyte = 1024.0;
  constexpr double kMebibyte = 1024.0 * kKibibyte;
  constexpr double kGibibyte = 1024.0 * kMebibyte;

  const double magnitude = std::abs(bytes);
  double divisor = 1.0;
  const char* suffix = "B";
  int precision = 0;
  if (magnitude >= kGibibyte) {
    divisor = kGibibyte;
    suffix = "GiB";
    precision = 2;
  } else if (magnitude >= kMebibyte) {
    divisor = kMebibyte;
    suffix = "MiB";
    precision = 2;
  } else if (magnitude >= kKibibyte) {
    divisor = kKibibyte;
    suffix = "KiB";
    precision = 1;
  }

  std::ostringstream stream;
  if (signedValue && bytes >= 0.0) {
    stream << '+';
  }
  stream << std::fixed << std::setprecision(precision) << bytes / divisor << ' '
         << suffix;
  return stream.str();
}

//==============================================================================
std::string formatMetricValue(
    const DiagnosticMetric& metric, double value, bool signedValue)
{
  if (metric.unit == "bytes") {
    return formatBytes(value, signedValue);
  }

  std::ostringstream stream;
  if (signedValue && value >= 0.0) {
    stream << '+';
  }
  if (metric.unit == "count") {
    stream << std::fixed << std::setprecision(0) << value;
  } else if (metric.unit == "seconds") {
    stream << std::fixed << std::setprecision(3) << value << " s";
  } else {
    stream << std::fixed << std::setprecision(3) << value;
    if (!metric.unit.empty()) {
      stream << ' ' << metric.unit;
    }
  }
  return stream.str();
}

//==============================================================================
const MetricDelta* findDelta(
    const std::optional<SnapshotComparison>& comparison,
    const std::string& key) noexcept
{
  if (!comparison) {
    return nullptr;
  }

  for (const auto& delta : comparison->metrics) {
    if (delta.metric.key == key) {
      return &delta;
    }
  }
  return nullptr;
}

//==============================================================================
void renderMetricTooltip(const DiagnosticMetric& metric)
{
  if (!ImGui::IsItemHovered()) {
    return;
  }

  ImGui::BeginTooltip();
  ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
  ImGui::Text("Key: %s", metric.key.c_str());
  ImGui::Text(
      "Quality: %s",
      metric.value ? metricQualityLabel(metric.quality) : "unavailable");
  if (!metric.scope.empty()) {
    ImGui::TextWrapped("Scope: %s", metric.scope.c_str());
  }
  if (!metric.source.empty()) {
    ImGui::TextWrapped("Source: %s", metric.source.c_str());
  }
  if (!metric.limitation.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Limitation: %s", metric.limitation.c_str());
  }
  if (!metric.value) {
    ImGui::Separator();
    ImGui::TextDisabled("Unavailable in this build or sample.");
  }
  ImGui::PopTextWrapPos();
  ImGui::EndTooltip();
}

//==============================================================================
void renderByteHistory(
    const DiagnosticSession& session,
    const char* key,
    double guiScale,
    bool showObservedPeak)
{
  const auto series = session.historySeries(key);
  if (!series || series->values.empty()) {
    ImGui::TextDisabled("RSS history will appear after the first sample.");
    return;
  }

  constexpr double kMebibyte = 1024.0 * 1024.0;
  std::vector<float> values;
  values.reserve(series->values.size());
  float graphMax = 1.0f;
  for (const double value : series->values) {
    const double mib = value / kMebibyte;
    const float plotted = static_cast<float>(
        std::min(mib, static_cast<double>(std::numeric_limits<float>::max())));
    values.push_back(plotted);
    graphMax = std::max(graphMax, plotted);
  }

  std::string overlay = formatBytes(series->values.back(), false);
  if (showObservedPeak) {
    const auto observedPeak = session.observedResidentPeakBytes();
    if (observedPeak) {
      overlay += " current, ";
      overlay += formatBytes(*observedPeak, false);
      overlay += " observed peak";
    }
  }

  const std::string plotId = "##memory_history_" + series->key;
  ImGui::PlotLines(
      plotId.c_str(),
      values.data(),
      static_cast<int>(values.size()),
      0,
      overlay.c_str(),
      0.0f,
      graphMax * 1.05f,
      ImVec2(
          ImGui::GetContentRegionAvail().x,
          std::max(54.0f, 78.0f * static_cast<float>(guiScale))));
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "History is bounded to %zu samples.", session.historyCapacity());
  }
}

//==============================================================================
void renderMetricTable(
    const DiagnosticSnapshot& snapshot,
    const std::optional<SnapshotComparison>& comparison,
    double guiScale)
{
  constexpr ImGuiTableFlags flags
      = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg
        | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY
        | ImGuiTableFlags_SizingStretchProp;
  const float availableHeight = ImGui::GetContentRegionAvail().y;
  const float tableHeight = std::max(
      150.0f,
      std::min(360.0f * static_cast<float>(guiScale), availableHeight * 0.65f));
  if (!ImGui::BeginTable(
          "##dart6_memory_metrics", 4, flags, ImVec2(0.0f, tableHeight))) {
    return;
  }

  ImGui::TableSetupScrollFreeze(0, 1);
  ImGui::TableSetupColumn("Metric", ImGuiTableColumnFlags_WidthStretch, 2.2f);
  ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch, 1.2f);
  ImGui::TableSetupColumn("Quality", ImGuiTableColumnFlags_WidthFixed, 64.0f);
  ImGui::TableSetupColumn("Delta", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableHeadersRow();

  for (const auto& metric : snapshot.metrics) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(metric.label.c_str());
    renderMetricTooltip(metric);

    ImGui::TableSetColumnIndex(1);
    if (metric.value) {
      const std::string value = formatMetricValue(metric, *metric.value, false);
      ImGui::TextUnformatted(value.c_str());
    } else {
      ImGui::TextDisabled("Unavailable");
    }
    renderMetricTooltip(metric);

    ImGui::TableSetColumnIndex(2);
    if (metric.value) {
      ImGui::TextUnformatted(metricQualityLabel(metric.quality));
    } else {
      ImGui::TextDisabled("unavailable");
    }
    renderMetricTooltip(metric);

    ImGui::TableSetColumnIndex(3);
    const auto* delta = findDelta(comparison, metric.key);
    if (delta) {
      const std::string formatted
          = formatMetricValue(metric, delta->delta, true);
      ImGui::TextUnformatted(formatted.c_str());
    } else {
      ImGui::TextDisabled("--");
    }
    renderMetricTooltip(metric);
  }

  ImGui::EndTable();
}

} // namespace

//==============================================================================
void renderMemoryDiagnostics(
    DiagnosticSession& session, double monotonicNowSeconds, double guiScale)
{
  bool enabled = session.isEnabled();
  if (ImGui::Checkbox("Enable collection", &enabled)) {
    session.setEnabled(enabled);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Opt-in sampling at %.1f s cadence. Disabled collection performs no "
        "process or World traversal.",
        session.sampleIntervalSeconds());
  }

  float sampleInterval = static_cast<float>(session.sampleIntervalSeconds());
  ImGui::SetNextItemWidth(std::min(
      180.0f * static_cast<float>(guiScale), ImGui::GetContentRegionAvail().x));
  if (ImGui::SliderFloat(
          "Sample interval",
          &sampleInterval,
          0.1f,
          5.0f,
          "%.1f s",
          ImGuiSliderFlags_AlwaysClamp)) {
    session.setSampleIntervalSeconds(
        static_cast<double>(std::clamp(sampleInterval, 0.1f, 5.0f)));
  }

  if (!session.isEnabled()) {
    ImGui::TextWrapped(
        "Memory diagnostics are off by default. Enable collection to sample "
        "whole-process RSS and the active DART 6 World every %.1f seconds.",
        session.sampleIntervalSeconds());
    return;
  }

  if (ImGui::Button("Capture now")) {
    session.captureNow(monotonicNowSeconds);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Collect immediately and use that sample as baseline.");
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!session.latest());
  if (ImGui::Button("Set baseline")) {
    session.captureLatestAsBaseline();
  }
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Use the latest collected sample as the comparison base.");
  }
  ImGui::SameLine();
  bool resetSession = false;
  if (ImGui::Button("Reset")) {
    session.reset();
    resetSession = true;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Clear panel history, baseline, and observed peak. Process-lifetime "
        "peak RSS is not reset.");
  }

  // Process manual actions before the cadence tick. Capture updates the last
  // collection time, so a due frame cannot traverse twice; reset intentionally
  // leaves the panel empty until the next frame.
  if (!resetSession) {
    session.update(monotonicNowSeconds);
  }

  if (!session.latest()) {
    ImGui::TextDisabled("Waiting for the first sample.");
    return;
  }

  const auto& snapshot = *session.latest();
  ImGui::Text(
      "Generation %llu   Frame %llu   Sim %.3f s",
      static_cast<unsigned long long>(snapshot.generation),
      static_cast<unsigned long long>(snapshot.frame),
      snapshot.simulationTimeSeconds);

  ImGui::TextUnformatted("Whole-process resident memory");
  renderByteHistory(session, kProcessResidentBytesKey, guiScale, true);
  ImGui::TextUnformatted("Known object shallow-size floor");
  renderByteHistory(
      session, "world.object_shallow_floor_bytes", guiScale, false);

  const auto comparison = session.comparison();
  if (session.baseline()) {
    if (comparison && comparison->generationMatches) {
      ImGui::TextDisabled("Delta is current minus baseline.");
    } else {
      ImGui::TextDisabled("Baseline is not comparable with this generation.");
    }
  } else {
    ImGui::TextDisabled("Set a baseline to show deltas.");
  }

  renderMetricTable(snapshot, comparison, guiScale);

  if (ImGui::CollapsingHeader(
          "How to read these metrics", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (const auto& guidance : snapshot.guidance) {
      ImGui::BulletText("%s", guidance.c_str());
    }
  }
}

} // namespace dart::examples::demos
