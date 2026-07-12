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

#include <dart/constraint/ConstraintBase.hpp>
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
#include <dart/common/detail/AllocatorMemoryLayout.hpp>

#include <imgui.h>

#include <algorithm>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeinfo>
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
void addObjectAddressSample(
    std::vector<std::uintptr_t>& addresses,
    std::vector<AddressAtlasExtent>& samples,
    const void* address,
    std::size_t sizeBytes,
    MemoryDataCategory dataCategory,
    MemoryExtentKind extentKind,
    std::string label,
    std::string evidence)
{
  if (address == nullptr || sizeBytes == 0) {
    return;
  }

  const auto rawAddress = reinterpret_cast<std::uintptr_t>(address);
  const std::size_t observedSize
      = extentKind == MemoryExtentKind::AddressPoint ? 1u : sizeBytes;
  const auto endAddress = checkedAddressRangeEnd(rawAddress, observedSize);
  if (!endAddress) {
    return;
  }
  addresses.push_back(rawAddress);
  AddressAtlasExtent sample;
  sample.address = rawAddress;
  sample.sizeBytes = observedSize;
  sample.dataCategory = dataCategory;
  sample.extentKind = extentKind;
  sample.quality = extentKind == MemoryExtentKind::AddressPoint
                       ? MetricQuality::Proxy
                       : MetricQuality::Estimate;
  sample.label = std::move(label);
  sample.evidence = std::move(evidence);
  samples.push_back(std::move(sample));
}

template <typename T>
MemoryExtentKind concreteExtentKind(const T* pointer)
{
  if constexpr (std::is_polymorphic<T>::value) {
    return pointer != nullptr && typeid(*pointer) == typeid(T)
               ? MemoryExtentKind::ShallowLowerBound
               : MemoryExtentKind::AddressPoint;
  }
  return MemoryExtentKind::AddressPoint;
}

std::string observationLabel(const char* typeName, MemoryExtentKind kind)
{
  return std::string(typeName)
         + (kind == MemoryExtentKind::ShallowLowerBound
                ? " shallow lower-bound extent"
                : " address point");
}

std::string observationEvidence(const char* typeName, MemoryExtentKind kind)
{
  if (kind == MemoryExtentKind::ShallowLowerBound) {
    return std::string("exact traversed address; runtime type exactly ")
           + typeName + "; sizeof concrete type is a shallow lower bound";
  }
  return std::string("exact traversed address; concrete dynamic type is not ")
         + typeName + "; point observation only";
}

//==============================================================================
MemoryStorageState convertAllocatorStorageState(
    common::detail::AllocatorMemorySpanKind kind) noexcept
{
  using Kind = common::detail::AllocatorMemorySpanKind;
  switch (kind) {
    case Kind::AllocatorMetadata:
      return MemoryStorageState::Metadata;
    case Kind::AllocatedPayload:
      return MemoryStorageState::Allocated;
    case Kind::FreePayload:
      return MemoryStorageState::Free;
    case Kind::AlignmentPadding:
      return MemoryStorageState::Padding;
    case Kind::FrameCursorConsumed:
      return MemoryStorageState::Allocated;
    case Kind::FrameReservedAvailable:
      return MemoryStorageState::Reserved;
    case Kind::FrameOverflowBacking:
      return MemoryStorageState::Allocated;
  }
  return MemoryStorageState::Unobserved;
}

//==============================================================================
MemoryDataCategory convertAllocatorDataCategory(
    common::detail::AllocatorMemorySpanKind kind) noexcept
{
  using Kind = common::detail::AllocatorMemorySpanKind;
  switch (kind) {
    case Kind::AllocatorMetadata:
      return MemoryDataCategory::Metadata;
    case Kind::AllocatedPayload:
      return MemoryDataCategory::Unknown;
    case Kind::FreePayload:
    case Kind::AlignmentPadding:
    case Kind::FrameReservedAvailable:
      return MemoryDataCategory::None;
    case Kind::FrameCursorConsumed:
    case Kind::FrameOverflowBacking:
      return MemoryDataCategory::Scratch;
  }
  return MemoryDataCategory::None;
}

//==============================================================================
const char* allocatorSpanLabel(
    common::detail::AllocatorMemorySpanKind kind) noexcept
{
  using Kind = common::detail::AllocatorMemorySpanKind;
  switch (kind) {
    case Kind::AllocatorMetadata:
      return "allocator block header";
    case Kind::AllocatedPayload:
      return "allocated free-list block extent";
    case Kind::FreePayload:
      return "available free-list block extent";
    case Kind::AlignmentPadding:
      return "alignment or unusable tail padding";
    case Kind::FrameCursorConsumed:
      return "frame cursor-consumed backing extent";
    case Kind::FrameReservedAvailable:
      return "frame reserved available backing extent";
    case Kind::FrameOverflowBacking:
      return "frame overflow backing allocation";
  }
  return "allocator backing extent";
}

//==============================================================================
const char* allocatorSpanEvidence(
    common::detail::AllocatorMemorySpanKind kind) noexcept
{
  using Kind = common::detail::AllocatorMemorySpanKind;
  switch (kind) {
    case Kind::AllocatorMetadata:
      return "exact allocator-owned header extent";
    case Kind::AllocatedPayload:
      return "exact allocated block extent; an unsplittable remainder can make "
             "this larger than the caller's requested byte count";
    case Kind::FreePayload:
      return "exact currently available block extent";
    case Kind::AlignmentPadding:
      return "exact backing bytes outside the aligned usable window";
    case Kind::FrameCursorConsumed:
      return "exact cursor-consumed backing extent; includes requested storage "
             "plus alignment, cache-color, or internal padding and is not "
             "requested-payload accounting";
    case Kind::FrameReservedAvailable:
      return "exact reserved backing extent between the cursor and usable end";
    case Kind::FrameOverflowBacking:
      return "exact live overflow backing extent; requested payload and "
             "alignment padding are not retained separately";
  }
  return "allocator backing extent";
}

//==============================================================================
void appendAllocatorRegions(
    DiagnosticSnapshot& snapshot,
    const common::detail::AllocatorMemoryLayout& layout)
{
  using RegionKind = common::detail::AllocatorMemoryRegionKind;
  std::size_t freeListIndex = 0;
  std::size_t framePrimaryIndex = 0;
  std::size_t frameOverflowIndex = 0;

  for (const auto& captured : layout.regions) {
    MemoryMapRegion region;
    region.sizeBytes = captured.sizeBytes;
    region.quality = MetricQuality::Measured;

    switch (captured.kind) {
      case RegionKind::FreeListBacking:
        region.id = "free-list-" + std::to_string(freeListIndex);
        region.label
            = "Free-list backing region " + std::to_string(++freeListIndex);
        region.scope = "World MemoryManager free-list backing allocation";
        region.source
            = "FreeListAllocator linked block metadata captured under its "
              "mutex";
        region.limitation
            = "Exact allocator-owned headers and payload state. The classic "
              "DART 6 object graph and solver scratch are not allocated here.";
        break;
      case RegionKind::FramePrimary:
        region.id = "frame-primary-" + std::to_string(framePrimaryIndex);
        region.label = "Frame arena backing region "
                       + std::to_string(++framePrimaryIndex);
        region.scope = "World MemoryManager primary frame backing allocation";
        region.source = "FrameAllocator buffer, cursor, and usable window";
        region.limitation
            = "Exact arena backing and cursor state. Cursor-consumed bytes can "
              "include alignment, cache-color, or internal padding and are not "
              "an exact requested-payload total. Current classic DART 6 solver "
              "scratch does not allocate from this arena.";
        break;
      case RegionKind::FrameOverflow:
        region.id = "frame-overflow-" + std::to_string(frameOverflowIndex);
        region.label = "Frame overflow backing region "
                       + std::to_string(++frameOverflowIndex);
        region.scope = "World MemoryManager frame overflow allocation";
        region.source = "FrameAllocator live overflow entry";
        region.limitation
            = "The whole base allocation is exact; requested payload and "
              "alignment padding are not retained separately.";
        break;
    }

    region.spans.reserve(captured.spans.size());
    for (const auto& span : captured.spans) {
      const MemoryStorageState storageState
          = convertAllocatorStorageState(span.kind);
      const MemoryDataCategory dataCategory
          = convertAllocatorDataCategory(span.kind);
      region.spans.push_back(MemoryMapSpan{
          span.offsetBytes,
          span.sizeBytes,
          storageState,
          dataCategory,
          MemoryExtentKind::ExactByteRange,
          allocatorSpanLabel(span.kind),
          MetricQuality::Measured,
          allocatorSpanEvidence(span.kind)});
    }
    snapshot.allocatorMemoryMap.push_back(std::move(region));
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
  std::vector<AddressAtlasExtent> objectAddressSamples;
  double shallowSizeFloorBytes = 0.0;

  objectAddresses.reserve(skeletonCount * 8u);
  objectAddressSamples.reserve(skeletonCount * 8u);
  for (std::size_t skeletonIndex = 0; skeletonIndex < skeletonCount;
       ++skeletonIndex) {
    const auto& skeleton = world->getSkeleton(skeletonIndex);
    if (!skeleton) {
      continue;
    }

    const MemoryExtentKind skeletonExtentKind
        = concreteExtentKind(skeleton.get());
    addObjectAddressSample(
        objectAddresses,
        objectAddressSamples,
        skeleton.get(),
        sizeof(dynamics::Skeleton),
        MemoryDataCategory::Model,
        skeletonExtentKind,
        observationLabel("Skeleton", skeletonExtentKind),
        observationEvidence("Skeleton", skeletonExtentKind));
    if (skeletonExtentKind == MemoryExtentKind::ShallowLowerBound) {
      shallowSizeFloorBytes += static_cast<double>(sizeof(dynamics::Skeleton));
    }

    bodyNodeCount += skeleton->getNumBodyNodes();
    jointCount += skeleton->getNumJoints();
    dofCount += skeleton->getNumDofs();

    for (std::size_t jointIndex = 0; jointIndex < skeleton->getNumJoints();
         ++jointIndex) {
      const auto* joint = skeleton->getJoint(jointIndex);
      constexpr MemoryExtentKind jointExtentKind
          = MemoryExtentKind::AddressPoint;
      addObjectAddressSample(
          objectAddresses,
          objectAddressSamples,
          joint,
          sizeof(dynamics::Joint),
          MemoryDataCategory::Model,
          jointExtentKind,
          observationLabel("Joint", jointExtentKind),
          observationEvidence("Joint", jointExtentKind));
    }

    for (std::size_t dofIndex = 0; dofIndex < skeleton->getNumDofs();
         ++dofIndex) {
      const auto* dof = skeleton->getDof(dofIndex);
      const MemoryExtentKind dofExtentKind = concreteExtentKind(dof);
      addObjectAddressSample(
          objectAddresses,
          objectAddressSamples,
          dof,
          sizeof(dynamics::DegreeOfFreedom),
          MemoryDataCategory::State,
          dofExtentKind,
          observationLabel("DegreeOfFreedom", dofExtentKind),
          observationEvidence("DegreeOfFreedom", dofExtentKind));
      if (dofExtentKind == MemoryExtentKind::ShallowLowerBound) {
        shallowSizeFloorBytes
            += static_cast<double>(sizeof(dynamics::DegreeOfFreedom));
      }
    }

    for (std::size_t bodyIndex = 0; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      const auto* body = skeleton->getBodyNode(bodyIndex);
      if (!body) {
        continue;
      }

      const auto* softBody = body->asSoftBodyNode();
      const std::size_t bodyShallowBytes = softBody
                                               ? sizeof(dynamics::SoftBodyNode)
                                               : sizeof(dynamics::BodyNode);
      const MemoryExtentKind bodyExtentKind
          = softBody ? concreteExtentKind(softBody) : concreteExtentKind(body);
      const char* const bodyTypeName = softBody ? "SoftBodyNode" : "BodyNode";
      addObjectAddressSample(
          objectAddresses,
          objectAddressSamples,
          softBody ? static_cast<const void*>(softBody)
                   : static_cast<const void*>(body),
          bodyShallowBytes,
          MemoryDataCategory::Model,
          bodyExtentKind,
          observationLabel(bodyTypeName, bodyExtentKind),
          observationEvidence(bodyTypeName, bodyExtentKind));
      if (bodyExtentKind == MemoryExtentKind::ShallowLowerBound) {
        shallowSizeFloorBytes += static_cast<double>(bodyShallowBytes);
      }
      if (softBody) {
        ++softBodyCount;
        pointMassCount += softBody->getNumPointMasses();
        for (std::size_t pointIndex = 0;
             pointIndex < softBody->getNumPointMasses();
             ++pointIndex) {
          const auto* pointMass = softBody->getPointMass(pointIndex);
          const MemoryExtentKind pointMassExtentKind
              = concreteExtentKind(pointMass);
          addObjectAddressSample(
              objectAddresses,
              objectAddressSamples,
              pointMass,
              sizeof(dynamics::PointMass),
              MemoryDataCategory::State,
              pointMassExtentKind,
              observationLabel("PointMass", pointMassExtentKind),
              observationEvidence("PointMass", pointMassExtentKind));
          if (pointMassExtentKind == MemoryExtentKind::ShallowLowerBound) {
            shallowSizeFloorBytes
                += static_cast<double>(sizeof(dynamics::PointMass));
          }
        }
      }

      shapeNodeCount += body->getNumShapeNodes();
      for (std::size_t shapeIndex = 0; shapeIndex < body->getNumShapeNodes();
           ++shapeIndex) {
        const auto* shapeNode = body->getShapeNode(shapeIndex);
        if (!shapeNode) {
          continue;
        }

        const MemoryExtentKind shapeNodeExtentKind
            = concreteExtentKind(shapeNode);
        addObjectAddressSample(
            objectAddresses,
            objectAddressSamples,
            shapeNode,
            sizeof(dynamics::ShapeNode),
            MemoryDataCategory::Geometry,
            shapeNodeExtentKind,
            observationLabel("ShapeNode", shapeNodeExtentKind),
            observationEvidence("ShapeNode", shapeNodeExtentKind));
        if (shapeNodeExtentKind == MemoryExtentKind::ShallowLowerBound) {
          shallowSizeFloorBytes
              += static_cast<double>(sizeof(dynamics::ShapeNode));
        }
        const auto shape = shapeNode->getShape();
        if (shape && uniqueShapes.insert(shape.get()).second) {
          constexpr MemoryExtentKind shapeExtentKind
              = MemoryExtentKind::AddressPoint;
          addObjectAddressSample(
              objectAddresses,
              objectAddressSamples,
              shape.get(),
              sizeof(dynamics::Shape),
              MemoryDataCategory::Geometry,
              shapeExtentKind,
              observationLabel("Shape", shapeExtentKind),
              observationEvidence("Shape", shapeExtentKind));
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
    const MemoryExtentKind frameExtentKind = concreteExtentKind(frame.get());
    addObjectAddressSample(
        objectAddresses,
        objectAddressSamples,
        frame.get(),
        sizeof(dynamics::SimpleFrame),
        MemoryDataCategory::Geometry,
        frameExtentKind,
        observationLabel("SimpleFrame", frameExtentKind),
        observationEvidence("SimpleFrame", frameExtentKind));
    if (frameExtentKind == MemoryExtentKind::ShallowLowerBound) {
      shallowSizeFloorBytes
          += static_cast<double>(sizeof(dynamics::SimpleFrame));
    }
    const auto shape = frame->getShape();
    if (shape && uniqueShapes.insert(shape.get()).second) {
      constexpr MemoryExtentKind shapeExtentKind
          = MemoryExtentKind::AddressPoint;
      addObjectAddressSample(
          objectAddresses,
          objectAddressSamples,
          shape.get(),
          sizeof(dynamics::Shape),
          MemoryDataCategory::Geometry,
          shapeExtentKind,
          observationLabel("Shape", shapeExtentKind),
          observationEvidence("Shape", shapeExtentKind));
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
    for (std::size_t constraintIndex = 0;
         constraintIndex < constraintSolver->getNumConstraints();
         ++constraintIndex) {
      const auto constraint = constraintSolver->getConstraint(constraintIndex);
      constexpr MemoryExtentKind constraintExtentKind
          = MemoryExtentKind::AddressPoint;
      addObjectAddressSample(
          objectAddresses,
          objectAddressSamples,
          constraint.get(),
          sizeof(constraint::ConstraintBase),
          MemoryDataCategory::ConstraintSolver,
          constraintExtentKind,
          observationLabel("Manual ConstraintBase", constraintExtentKind),
          "exact registered-constraint address; concrete dynamic type is not "
          "ConstraintBase; point observation only");
    }

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
  appendAllocatorRegions(
      snapshot,
      common::detail::captureAllocatorMemoryLayout(
          memoryManager.getFreeListAllocator()));
  const auto& frameAllocator = memoryManager.getFrameAllocator();
  appendAllocatorRegions(
      snapshot, common::detail::captureAllocatorMemoryLayout(frameAllocator));
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
      "Frame arena cursor-consumed backing bytes",
      "bytes",
      static_cast<double>(frameAllocator.used()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      "This cursor distance includes requested storage plus alignment, "
      "cache-color, or internal padding; it is not exact requested-payload "
      "accounting. The arena resets at the start of each World::step, so this "
      "is the current post-reset interval, not a lifetime peak. "
          + frameLimitation,
      true);
  addMetric(
      snapshot,
      "world.scratch.frame_overflow_count",
      "Live frame overflow backing allocations",
      "count",
      static_cast<double>(frameAllocator.overflowCount()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      frameLimitation);
  addMetric(
      snapshot,
      "world.scratch.frame_overflow_bytes",
      "Live frame overflow backing bytes",
      "bytes",
      static_cast<double>(frameAllocator.overflowBytes()),
      MetricQuality::Measured,
      frameScope,
      frameSource,
      "These are backing-allocation extents; requested payload and alignment "
      "padding are not retained separately. "
          + frameLimitation,
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
      "Known concrete object shallow-size floor",
      "bytes",
      shallowSizeFloorBytes,
      MetricQuality::Estimate,
      "traversed DART 6 object graph",
      "exact runtime concrete-type matches multiplied by sizeof(concrete type)",
      "Excludes point-only dynamic observations, container capacity, "
      "shared-pointer control blocks, collision-backend objects, alignment, "
      "and other nested allocations.",
      true);

  const HostPageSizeReading pageSizeReading = collectHostPageSize();
  snapshot.objectAddressAtlas = makeObjectAddressAtlas(
      std::move(objectAddressSamples), pageSizeReading);

  auto locality = summarizeAddressLocality(
      objectAddresses, pageSizeReading.bytes.value_or(0u));
  locality.source += "; page size source: " + pageSizeReading.source;
  if (!pageSizeReading.limitation.empty()) {
    locality.limitation += " " + pageSizeReading.limitation;
  }
  const std::string localityScope = "traversed DART 6 object addresses";
  const std::string pageBucketLabel
      = pageSizeReading.bytes
            ? "Distinct object-address "
                  + std::to_string(*pageSizeReading.bytes) + " B page buckets"
            : "Distinct object-address host-page buckets";
  addMetric(
      snapshot,
      "world.locality.address_count",
      "Traversed object addresses sampled",
      "count",
      locality.addressStatisticsAvailable
          ? std::optional<double>(static_cast<double>(locality.addressCount))
          : std::nullopt,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.address_page_count",
      pageBucketLabel,
      "count",
      locality.pageStatisticsAvailable ? std::optional<double>(
          static_cast<double>(locality.distinctPageCount))
                                       : std::nullopt,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.page_transitions",
      "Traversal-order virtual-page transitions",
      "count",
      locality.pageStatisticsAvailable
          ? std::optional<double>(static_cast<double>(locality.pageTransitions))
          : std::nullopt,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.median_gap_bytes",
      "Median traversal-adjacent virtual-address gap",
      "bytes",
      locality.medianGapBytes,
      MetricQuality::Proxy,
      localityScope,
      locality.source,
      locality.limitation);
  addMetric(
      snapshot,
      "world.locality.p95_gap_bytes",
      "P95 traversal-adjacent virtual-address gap",
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
      "Allocator map rows are exact contiguous backing allocations with "
      "region-relative byte offsets; separate rows are not assumed "
      "contiguous.");
  snapshot.guidance.push_back(
      "The typed object atlas uses exact virtual-address points and only draws "
      "a sizeof shallow lower bound when runtime type identity proves the "
      "concrete counted type. It is visually separate because classic DART 6 "
      "simulation objects are not owned by the World MemoryManager arenas.");
  if (!pageSizeReading.bytes) {
    snapshot.guidance.push_back(
        "The host page-size query was unavailable, so the typed object atlas "
        "and page-bucket locality values are unavailable rather than assuming "
        "a page size. "
        + pageSizeReading.limitation);
  }
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
ImVec4 memoryDataColor(MemoryDataCategory category) noexcept
{
  switch (category) {
    case MemoryDataCategory::None:
      return ImVec4(0.40f, 0.42f, 0.46f, 1.0f);
    case MemoryDataCategory::Unknown:
      return ImVec4(0.95f, 0.68f, 0.16f, 1.0f);
    case MemoryDataCategory::Metadata:
      return ImVec4(0.35f, 0.38f, 0.43f, 1.0f);
    case MemoryDataCategory::Model:
      return ImVec4(0.00f, 0.45f, 0.70f, 1.0f);
    case MemoryDataCategory::State:
      return ImVec4(0.80f, 0.47f, 0.65f, 1.0f);
    case MemoryDataCategory::Geometry:
      return ImVec4(0.00f, 0.62f, 0.45f, 1.0f);
    case MemoryDataCategory::ConstraintSolver:
      return ImVec4(0.73f, 0.30f, 0.74f, 1.0f);
    case MemoryDataCategory::Scratch:
      return ImVec4(0.90f, 0.38f, 0.12f, 1.0f);
  }
  return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
}

//==============================================================================
ImVec4 memoryMapColor(
    MemoryDataCategory category, MemoryStorageState state) noexcept
{
  ImVec4 color = memoryDataColor(category);
  switch (state) {
    case MemoryStorageState::Free:
      color.w = 0.60f;
      break;
    case MemoryStorageState::Reserved:
      color.w = 0.50f;
      break;
    case MemoryStorageState::Padding:
      color.w = 0.42f;
      break;
    case MemoryStorageState::Unobserved:
      color.w = 0.34f;
      break;
    case MemoryStorageState::Metadata:
    case MemoryStorageState::Allocated:
    case MemoryStorageState::Observed:
      color.w = 1.0f;
      break;
  }
  return color;
}

//==============================================================================
bool memoryStorageStateUsesPattern(MemoryStorageState state) noexcept
{
  return state == MemoryStorageState::Free
         || state == MemoryStorageState::Reserved
         || state == MemoryStorageState::Padding
         || state == MemoryStorageState::Unobserved;
}

//==============================================================================
ImU32 memoryStorageStateBorder(MemoryStorageState state) noexcept
{
  switch (state) {
    case MemoryStorageState::Metadata:
      return IM_COL32(245, 247, 250, 210);
    case MemoryStorageState::Allocated:
    case MemoryStorageState::Observed:
      return IM_COL32(245, 247, 250, 90);
    case MemoryStorageState::Free:
    case MemoryStorageState::Reserved:
    case MemoryStorageState::Padding:
    case MemoryStorageState::Unobserved:
      return IM_COL32(245, 247, 250, 50);
  }
  return IM_COL32(245, 247, 250, 50);
}

//==============================================================================
std::size_t ceilDivide(std::size_t numerator, std::size_t denominator)
{
  return denominator == 0
             ? 0
             : numerator / denominator
                   + static_cast<std::size_t>(numerator % denominator != 0);
}

//==============================================================================
std::size_t chooseBytesPerRow(std::size_t sizeBytes, std::size_t maxRows)
{
  if (sizeBytes == 0 || maxRows == 0) {
    return 1;
  }
  const std::size_t target = ceilDivide(sizeBytes, maxRows);
  std::size_t bytesPerRow = 64;
  while (bytesPerRow < target
         && bytesPerRow <= std::numeric_limits<std::size_t>::max() / 2u) {
    bytesPerRow *= 2u;
  }
  return std::max<std::size_t>(bytesPerRow, target);
}

//==============================================================================
void drawMemoryMapPattern(
    ImDrawList* drawList,
    const ImVec2& minimum,
    const ImVec2& maximum,
    MemoryStorageState state)
{
  drawList->PushClipRect(minimum, maximum, true);
  const ImU32 patternColor = IM_COL32(235, 239, 245, 72);
  const float height = maximum.y - minimum.y;
  constexpr float kHatchSpacing = 11.0f;
  const auto drawForwardHatch = [&] {
    for (float x = minimum.x - height; x < maximum.x; x += kHatchSpacing) {
      drawList->AddLine(
          ImVec2(x, maximum.y),
          ImVec2(x + height, minimum.y),
          patternColor,
          1.0f);
    }
  };
  const auto drawBackwardHatch = [&] {
    for (float x = minimum.x; x < maximum.x + height; x += kHatchSpacing) {
      drawList->AddLine(
          ImVec2(x, minimum.y),
          ImVec2(x - height, maximum.y),
          patternColor,
          1.0f);
    }
  };

  switch (state) {
    case MemoryStorageState::Free:
      drawForwardHatch();
      break;
    case MemoryStorageState::Reserved:
      drawBackwardHatch();
      break;
    case MemoryStorageState::Padding:
      drawForwardHatch();
      drawBackwardHatch();
      break;
    case MemoryStorageState::Unobserved: {
      const float width = maximum.x - minimum.x;
      const float firstY = minimum.y + std::min(4.0f, height * 0.5f);
      const float firstX = minimum.x + std::min(4.0f, width * 0.5f);
      const float radius = std::max(0.25f, std::min(0.8f, height * 0.35f));
      for (float y = firstY; y < maximum.y; y += 9.0f) {
        for (float x = firstX; x < maximum.x; x += 9.0f) {
          drawList->AddCircleFilled(ImVec2(x, y), radius, patternColor);
        }
      }
    } break;
    case MemoryStorageState::Metadata:
    case MemoryStorageState::Allocated:
    case MemoryStorageState::Observed:
      break;
  }
  drawList->PopClipRect();
}

//==============================================================================
void renderMemoryMapLegendSwatch(
    MemoryDataCategory dataCategory, MemoryStorageState storageState)
{
  const ImVec2 size(ImGui::GetTextLineHeight(), ImGui::GetTextLineHeight());
  ImGui::InvisibleButton("##legend_swatch", size);
  const ImVec2 minimum = ImGui::GetItemRectMin();
  const ImVec2 maximum = ImGui::GetItemRectMax();
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(
      minimum,
      maximum,
      ImGui::ColorConvertFloat4ToU32(
          memoryMapColor(dataCategory, storageState)));
  if (memoryStorageStateUsesPattern(storageState)) {
    drawMemoryMapPattern(drawList, minimum, maximum, storageState);
  }
  drawList->AddRect(
      minimum,
      maximum,
      memoryStorageStateBorder(storageState),
      0.0f,
      0,
      storageState == MemoryStorageState::Metadata ? 2.0f : 1.0f);
}

//==============================================================================
void renderMemoryMapLegend(bool objectAtlas)
{
  struct DataLegendEntry
  {
    MemoryDataCategory category;
    const char* label;
  };
  const std::vector<DataLegendEntry> dataEntries
      = objectAtlas
            ? std::vector<DataLegendEntry>{
                  {MemoryDataCategory::Model,
                   "Model: Skeleton / BodyNode / Joint"},
                  {MemoryDataCategory::State,
                   "State: DegreeOfFreedom / PointMass"},
                  {MemoryDataCategory::Geometry,
                   "Geometry: ShapeNode / Shape / SimpleFrame"},
                  {MemoryDataCategory::ConstraintSolver,
                   "Constraint/solver: manual constraints"},
                  {MemoryDataCategory::None, "No observed typed extent"}}
            : std::vector<DataLegendEntry>{
                  {MemoryDataCategory::Metadata, "Allocator metadata"},
                  {MemoryDataCategory::Unknown, "Untyped allocator payload"},
                  {MemoryDataCategory::Scratch, "Frame scratch"},
                  {MemoryDataCategory::None, "No data category"}};

  struct StateLegendEntry
  {
    MemoryStorageState state;
    const char* label;
  };
  const std::vector<StateLegendEntry> stateEntries
      = objectAtlas
            ? std::vector<StateLegendEntry>{
                  {MemoryStorageState::Observed, "Observed typed mark"},
                  {MemoryStorageState::Unobserved, "Unobserved (dots)"}}
            : std::vector<StateLegendEntry>{
                  {MemoryStorageState::Metadata, "Metadata (heavy border)"},
                  {MemoryStorageState::Allocated, "Allocated (solid)"},
                  {MemoryStorageState::Free, "Free (forward hatch)"},
                  {MemoryStorageState::Reserved, "Reserved (back hatch)"},
                  {MemoryStorageState::Padding, "Padding (crosshatch)"}};

  const int columnCount = ImGui::GetContentRegionAvail().x >= 560.0f ? 2 : 1;
  ImGui::TextDisabled("Hue = data category");
  if (!ImGui::BeginTable(
          objectAtlas ? "##object_atlas_data_legend"
                      : "##allocator_map_data_legend",
          columnCount,
          ImGuiTableFlags_SizingStretchSame)) {
    return;
  }
  for (std::size_t index = 0; index < dataEntries.size(); ++index) {
    ImGui::TableNextColumn();
    ImGui::PushID(static_cast<int>(index));
    ImGui::ColorButton(
        "##legend",
        memoryDataColor(dataEntries[index].category),
        ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
        ImVec2(ImGui::GetTextLineHeight(), ImGui::GetTextLineHeight()));
    ImGui::SameLine();
    ImGui::TextWrapped("%s", dataEntries[index].label);
    ImGui::PopID();
  }
  ImGui::EndTable();

  ImGui::TextDisabled("Pattern / border / opacity = storage state");
  if (!ImGui::BeginTable(
          objectAtlas ? "##object_atlas_state_legend"
                      : "##allocator_map_state_legend",
          columnCount,
          ImGuiTableFlags_SizingStretchSame)) {
    return;
  }
  for (std::size_t index = 0; index < stateEntries.size(); ++index) {
    ImGui::TableNextColumn();
    ImGui::PushID(static_cast<int>(index));
    renderMemoryMapLegendSwatch(
        objectAtlas ? MemoryDataCategory::Model : MemoryDataCategory::Unknown,
        stateEntries[index].state);
    ImGui::SameLine();
    ImGui::TextWrapped("%s", stateEntries[index].label);
    ImGui::PopID();
  }
  ImGui::EndTable();
  if (objectAtlas) {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled(
        "Filled bar = concrete shallow lower bound; 3 px white marker = "
        "address "
        "point only");
    ImGui::PopTextWrapPos();
  }
}

//==============================================================================
void renderMemoryMapRegion(
    const MemoryMapRegion& region,
    double guiScale,
    bool objectAtlas,
    std::size_t maxRows)
{
  if (region.sizeBytes == 0) {
    return;
  }

  constexpr std::size_t kColumns = 64;
  const std::size_t bytesPerRow = chooseBytesPerRow(region.sizeBytes, maxRows);
  const std::size_t rowCount = ceilDivide(region.sizeBytes, bytesPerRow);
  const std::size_t bytesPerCell
      = std::max<std::size_t>(1u, ceilDivide(bytesPerRow, kColumns));
  const std::vector<MemoryMapCell> cells
      = composeMemoryMapCells(region, bytesPerCell);
  const float rowHeight = std::max(13.0f, 17.0f * static_cast<float>(guiScale));

  ImGui::PushID(region.id.c_str());
  ImGui::Text(
      "%s  |  %s",
      region.label.c_str(),
      formatBytes(static_cast<double>(region.sizeBytes), false).c_str());
  ImGui::TextDisabled("%s evidence", metricQualityLabel(region.quality));
  ImGui::TextWrapped("Source: %s", region.source.c_str());
  if (region.pageSizeBytes) {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled(
        "Host page size: %s | %s",
        formatBytes(static_cast<double>(*region.pageSizeBytes), false).c_str(),
        region.pageSizeEvidence.c_str());
    ImGui::PopTextWrapPos();
  }
  ImGui::PushTextWrapPos(0.0f);
  ImGui::TextDisabled(
      "%s/row | %s/cell | offsets are relative to this region",
      formatBytes(static_cast<double>(bytesPerRow), false).c_str(),
      formatBytes(static_cast<double>(bytesPerCell), false).c_str());
  ImGui::TextDisabled(
      "Each cell uses ordered horizontal lanes for every intersecting range "
      "and observation; raise detail rows to resolve byte boundaries.");
  ImGui::PopTextWrapPos();

  const ImVec2 canvasSize(
      std::max(1.0f, ImGui::GetContentRegionAvail().x),
      rowHeight * static_cast<float>(rowCount));
  ImGui::InvisibleButton("##map", canvasSize);
  const ImVec2 origin = ImGui::GetItemRectMin();
  const ImVec2 canvasEnd = ImGui::GetItemRectMax();
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(origin, canvasEnd, IM_COL32(27, 30, 37, 255));

  const auto sourceSpan
      = [&region](const MemoryMapCellSegment& segment) -> const MemoryMapSpan& {
    return segment.layer == MemoryMapLayer::Partition
               ? region.spans[segment.spanIndex]
               : region.observations[segment.spanIndex];
  };
  const auto segmentRectangle = [&](const MemoryMapCell& cell,
                                    std::size_t segmentIndex) {
    const auto& segment = cell.segments[segmentIndex];
    const std::size_t row = cell.offsetBytes / bytesPerRow;
    const std::size_t rowBegin = row * bytesPerRow;
    const float x0 = origin.x
                     + canvasSize.x
                           * static_cast<float>(segment.offsetBytes - rowBegin)
                           / static_cast<float>(bytesPerRow);
    float x1 = origin.x
               + canvasSize.x
                     * static_cast<float>(
                         segment.offsetBytes + segment.sizeBytes - rowBegin)
                     / static_cast<float>(bytesPerRow);
    const float laneHeight
        = rowHeight / static_cast<float>(cell.segments.size());
    const float y0 = origin.y + static_cast<float>(row) * rowHeight
                     + static_cast<float>(segmentIndex) * laneHeight;
    const float y1 = origin.y + static_cast<float>(row + 1u) * rowHeight;
    x1 = std::min(canvasEnd.x, std::max(x1, x0 + 1.0f));
    return std::pair<ImVec2, ImVec2>{
        ImVec2(x0, y0),
        ImVec2(
            x1,
            segmentIndex + 1u == cell.segments.size() ? y1 : y0 + laneHeight)};
  };

  // A source span gets its own lane within each raster cell. Partition
  // boundaries and overlapping typed observations therefore remain visible as
  // an ordered composition instead of later spans overwriting earlier ones.
  for (const auto& cell : cells) {
    for (std::size_t segmentIndex = 0; segmentIndex < cell.segments.size();
         ++segmentIndex) {
      const auto& span = sourceSpan(cell.segments[segmentIndex]);
      const auto [minimum, maximum] = segmentRectangle(cell, segmentIndex);
      drawList->AddRectFilled(
          minimum,
          maximum,
          ImGui::ColorConvertFloat4ToU32(
              memoryMapColor(span.dataCategory, span.storageState)));
      if (memoryStorageStateUsesPattern(span.storageState)) {
        drawMemoryMapPattern(drawList, minimum, maximum, span.storageState);
      }
      drawList->AddRect(
          minimum,
          maximum,
          memoryStorageStateBorder(span.storageState),
          0.0f,
          0,
          span.storageState == MemoryStorageState::Metadata ? 2.0f : 1.0f);
    }
  }

  const ImU32 gridColor = IM_COL32(225, 231, 239, 40);
  for (std::size_t row = 0; row <= rowCount; ++row) {
    const float y = origin.y + static_cast<float>(row) * rowHeight;
    drawList->AddLine(ImVec2(origin.x, y), ImVec2(canvasEnd.x, y), gridColor);
  }
  for (std::size_t column = 0; column <= kColumns; ++column) {
    const float x = origin.x
                    + canvasSize.x * static_cast<float>(column)
                          / static_cast<float>(kColumns);
    drawList->AddLine(ImVec2(x, origin.y), ImVec2(x, canvasEnd.y), gridColor);
  }
  for (const auto& cell : cells) {
    for (std::size_t segmentIndex = 0; segmentIndex < cell.segments.size();
         ++segmentIndex) {
      const auto& span = sourceSpan(cell.segments[segmentIndex]);
      if (span.extentKind == MemoryExtentKind::AddressPoint) {
        const auto [minimum, maximum] = segmentRectangle(cell, segmentIndex);
        drawList->AddLine(
            ImVec2(minimum.x, minimum.y),
            ImVec2(minimum.x, maximum.y),
            IM_COL32(255, 255, 255, 245),
            3.0f);
        drawList->AddCircleFilled(
            ImVec2(minimum.x, (minimum.y + maximum.y) * 0.5f),
            2.0f,
            IM_COL32(255, 255, 255, 245));
      }
    }
  }
  drawList->AddRect(origin, canvasEnd, IM_COL32(225, 231, 239, 100));

  if (ImGui::IsItemHovered()) {
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const std::size_t row = std::min<std::size_t>(
        rowCount - 1u,
        static_cast<std::size_t>((mouse.y - origin.y) / rowHeight));
    const float fraction
        = std::clamp((mouse.x - origin.x) / canvasSize.x, 0.0f, 0.999999f);
    const std::size_t offset = std::min(
        region.sizeBytes - 1u,
        row * bytesPerRow + static_cast<std::size_t>(fraction * bytesPerRow));
    const std::size_t cellIndex
        = std::min(cells.size() - 1u, offset / bytesPerCell);
    const auto& cell = cells[cellIndex];

    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 36.0f);
    ImGui::TextUnformatted(region.label.c_str());
    ImGui::Text(
        "Relative offset: +%s",
        formatBytes(static_cast<double>(offset), false).c_str());
    ImGui::Text(
        "Cell range: +%s .. +%s | %zu composed evidence lanes",
        formatBytes(static_cast<double>(cell.offsetBytes), false).c_str(),
        formatBytes(
            static_cast<double>(cell.offsetBytes + cell.sizeBytes), false)
            .c_str(),
        cell.segments.size());
    for (const auto& segment : cell.segments) {
      const auto& span = sourceSpan(segment);
      ImGui::Separator();
      ImGui::Text(
          "%s layer | %s",
          segment.layer == MemoryMapLayer::Partition ? "partition"
                                                     : "observation",
          span.label.c_str());
      ImGui::Text(
          "Storage state: %s", memoryStorageStateLabel(span.storageState));
      ImGui::Text(
          "Data category: %s", memoryDataCategoryLabel(span.dataCategory));
      ImGui::Text(
          "Extent evidence: %s", memoryExtentKindLabel(span.extentKind));
      if (span.extentKind == MemoryExtentKind::AddressPoint) {
        ImGui::Text(
            "Point: +%s (display marker only; no byte extent)",
            formatBytes(static_cast<double>(span.offsetBytes), false).c_str());
      } else {
        ImGui::Text(
            "Range: +%s .. +%s (%s)",
            formatBytes(static_cast<double>(span.offsetBytes), false).c_str(),
            formatBytes(
                static_cast<double>(span.offsetBytes + span.sizeBytes), false)
                .c_str(),
            formatBytes(static_cast<double>(span.sizeBytes), false).c_str());
      }
      ImGui::Text("Evidence: %s", metricQualityLabel(span.quality));
      ImGui::TextWrapped("Source: %s", span.evidence.c_str());
    }
    ImGui::Separator();
    ImGui::TextWrapped("Scope: %s", region.scope.c_str());
    if (region.pageSizeBytes) {
      ImGui::TextWrapped(
          "Host page size: %s (%s)",
          formatBytes(static_cast<double>(*region.pageSizeBytes), false)
              .c_str(),
          region.pageSizeEvidence.c_str());
    }
    ImGui::TextWrapped("Limitation: %s", region.limitation.c_str());
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }

  if (objectAtlas) {
    ImGui::TextDisabled(
        "Hue = semantic data category; each cell stacks all typed observations "
        "over its dotted unobserved partition; white mark = point only.");
  }
  ImGui::PopID();
}

//==============================================================================
void renderExactMemoryRangeTable(
    const std::vector<MemoryMapRegion>& regions,
    double guiScale,
    bool objectAtlas)
{
  constexpr std::size_t kRowLimit = 128;
  std::size_t totalRows = 0;
  for (const auto& region : regions) {
    const auto addRows = [&totalRows](std::size_t count) {
      totalRows = count > std::numeric_limits<std::size_t>::max() - totalRows
                      ? std::numeric_limits<std::size_t>::max()
                      : totalRows + count;
    };
    addRows(region.spans.size());
    addRows(region.observations.size());
  }

  ImGui::PushID(objectAtlas ? "object_exact_ranges" : "allocator_exact_ranges");
  if (!ImGui::CollapsingHeader("Relative range evidence (keyboard-readable)")) {
    ImGui::PopID();
    return;
  }
  ImGui::TextWrapped(
      "Bounded text view of region-relative evidence. Raw process addresses "
      "are intentionally omitted; at most %zu rows are shown.",
      kRowLimit);

  constexpr ImGuiTableFlags flags
      = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
        | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX
        | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;
  const float tableHeight
      = std::max(160.0f, 260.0f * static_cast<float>(guiScale));
  const float tableWidth = 1220.0f * static_cast<float>(guiScale);
  if (!ImGui::BeginTable(
          "##ranges", 7, flags, ImVec2(0.0f, tableHeight), tableWidth)) {
    ImGui::PopID();
    return;
  }
  ImGui::TableSetupScrollFreeze(1, 1);
  const float scale = static_cast<float>(guiScale);
  ImGui::TableSetupColumn(
      "Region", ImGuiTableColumnFlags_WidthFixed, 180.0f * scale);
  ImGui::TableSetupColumn(
      "Layer", ImGuiTableColumnFlags_WidthFixed, 90.0f * scale);
  ImGui::TableSetupColumn(
      "Relative range", ImGuiTableColumnFlags_WidthFixed, 180.0f * scale);
  ImGui::TableSetupColumn(
      "Storage", ImGuiTableColumnFlags_WidthFixed, 110.0f * scale);
  ImGui::TableSetupColumn(
      "Extent", ImGuiTableColumnFlags_WidthFixed, 150.0f * scale);
  ImGui::TableSetupColumn(
      "Data", ImGuiTableColumnFlags_WidthFixed, 130.0f * scale);
  ImGui::TableSetupColumn(
      "Evidence", ImGuiTableColumnFlags_WidthFixed, 380.0f * scale);
  ImGui::TableHeadersRow();

  std::size_t shownRows = 0;
  for (const auto& region : regions) {
    const auto renderSpans = [&](const std::vector<MemoryMapSpan>& spans,
                                 const char* layerLabel) {
      for (const auto& span : spans) {
        if (shownRows == kRowLimit) {
          return;
        }
        ++shownRows;
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextWrapped("%s", region.label.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(layerLabel);
        ImGui::TableSetColumnIndex(2);
        if (span.extentKind == MemoryExtentKind::AddressPoint) {
          ImGui::TextWrapped(
              "+%s (point marker; no byte extent)",
              formatBytes(static_cast<double>(span.offsetBytes), false)
                  .c_str());
        } else {
          ImGui::TextWrapped(
              "+%s .. +%s (%s)",
              formatBytes(static_cast<double>(span.offsetBytes), false).c_str(),
              formatBytes(
                  static_cast<double>(span.offsetBytes + span.sizeBytes), false)
                  .c_str(),
              formatBytes(static_cast<double>(span.sizeBytes), false).c_str());
        }
        ImGui::TableSetColumnIndex(3);
        ImGui::TextWrapped("%s", memoryStorageStateLabel(span.storageState));
        ImGui::TableSetColumnIndex(4);
        ImGui::TextWrapped("%s", memoryExtentKindLabel(span.extentKind));
        ImGui::TableSetColumnIndex(5);
        ImGui::TextWrapped("%s", memoryDataCategoryLabel(span.dataCategory));
        ImGui::TableSetColumnIndex(6);
        ImGui::TextWrapped(
            "%s | %s | %s | region source: %s | limitation: %s",
            metricQualityLabel(span.quality),
            span.label.c_str(),
            span.evidence.c_str(),
            region.source.c_str(),
            region.limitation.c_str());
      }
    };
    renderSpans(region.spans, "partition");
    renderSpans(region.observations, "observation");
    if (shownRows == kRowLimit) {
      break;
    }
  }
  ImGui::EndTable();
  ImGui::TextDisabled(
      "Showing first %zu of %zu evidence rows.", shownRows, totalRows);
  ImGui::PopID();
}

//==============================================================================
void renderMemoryMapSection(
    const std::vector<MemoryMapRegion>& regions,
    double guiScale,
    bool objectAtlas,
    std::size_t maxRows)
{
  if (regions.empty()) {
    ImGui::TextDisabled(
        objectAtlas ? "No classic object addresses were sampled."
                    : "No allocator backing regions are available.");
    return;
  }

  renderMemoryMapLegend(objectAtlas);
  for (const auto& region : regions) {
    renderMemoryMapRegion(region, guiScale, objectAtlas, maxRows);
    ImGui::Spacing();
  }
  renderExactMemoryRangeTable(regions, guiScale, objectAtlas);
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

  static int addressMapDetailRows = 8;
  ImGui::SetNextItemWidth(std::min(
      260.0f * static_cast<float>(guiScale), ImGui::GetContentRegionAvail().x));
  ImGui::SliderInt(
      "Address-map detail rows / region",
      &addressMapDetailRows,
      4,
      48,
      "%d",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::PushTextWrapPos(0.0f);
  ImGui::TextDisabled(
      "More rows zoom the byte raster while preserving region-relative order.");
  ImGui::PopTextWrapPos();

  if (ImGui::CollapsingHeader(
          "Address map - exact World allocator regions",
          ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextWrapped(
        "Each row group is one real contiguous backing allocation. Byte order "
        "is preserved within the row group; separate groups are explicit "
        "address discontinuities. These arenas do not own the classic object "
        "graph. Cache-line and physical-page overlays are unavailable in this "
        "slice.");
    renderMemoryMapSection(
        snapshot.allocatorMemoryMap,
        guiScale,
        false,
        static_cast<std::size_t>(addressMapDetailRows));
  }

  if (ImGui::CollapsingHeader(
          "Typed classic-object address atlas - points and shallow bounds",
          ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextWrapped(
        "Exact traversed virtual addresses are grouped into contiguous host-"
        "page runs using the runtime page-size query shown on each row. Typed "
        "marks are address points unless runtime identity proves a concrete "
        "type, in which case sizeof supplies a shallow lower bound. Dotted "
        "spans are unobserved, not proven free. This is "
        "not a cache-miss measurement or an allocator ownership map. Host page "
        "size is used for grouping, not as a physical-page boundary overlay.");
    renderMemoryMapSection(
        snapshot.objectAddressAtlas,
        guiScale,
        true,
        static_cast<std::size_t>(addressMapDetailRows));
  }

  ImGui::TextUnformatted("Whole-process resident memory");
  renderByteHistory(session, kProcessResidentBytesKey, guiScale, true);
  ImGui::TextUnformatted("Known concrete object shallow-size floor");
  renderByteHistory(
      session, "world.object_shallow_floor_bytes", guiScale, false);

  const auto comparison = session.comparison();
  if (session.baseline()) {
    if (comparison && comparison->schemaMatches
        && comparison->generationMatches) {
      ImGui::TextDisabled("Delta is current minus baseline.");
    } else {
      ImGui::TextDisabled(
          "Baseline schema or World generation is not comparable.");
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
