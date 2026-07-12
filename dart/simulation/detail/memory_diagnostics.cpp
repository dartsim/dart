/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 * https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/simulation/detail/memory_diagnostics.hpp"

#include "dart/common/detail/allocator_memory_layout.hpp"
#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/compute/variational_integration.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail {
namespace {

//==============================================================================
std::string diagnosticLabel(entt::id_type id)
{
  const auto matches = [id]<typename Component>() {
    return id == entt::type_hash<Component>::value();
  };

  if (matches.template operator()<comps::Name>()) {
    return "Names";
  }
  if (matches.template operator()<comps::FrameTag>()) {
    return "Frame tags";
  }
  if (matches.template operator()<comps::FrameState>()) {
    return "Frame state";
  }
  if (matches.template operator()<comps::FrameCache>()) {
    return "Frame transform cache";
  }
  if (matches.template operator()<comps::FixedFrameTag>()) {
    return "Fixed-frame tags";
  }
  if (matches.template operator()<comps::FixedFrameProperties>()) {
    return "Fixed-frame properties";
  }
  if (matches.template operator()<comps::FreeFrameTag>()) {
    return "Free-frame tags";
  }
  if (matches.template operator()<comps::FreeFrameProperties>()) {
    return "Free-frame properties";
  }
  if (matches.template operator()<comps::Transform>()) {
    return "Transforms";
  }
  if (matches.template operator()<comps::Velocity>()) {
    return "Velocities";
  }
  if (matches.template operator()<comps::Force>()) {
    return "Forces";
  }
  if (matches.template operator()<comps::MassProperties>()) {
    return "Mass properties";
  }
  if (matches.template operator()<comps::DeactivationState>()) {
    return "Deactivation state";
  }
  if (matches.template operator()<comps::RigidBodyTag>()) {
    return "Rigid-body tags";
  }
  if (matches.template operator()<comps::StaticBodyTag>()) {
    return "Static-body tags";
  }
  if (matches.template operator()<comps::KinematicBodyTag>()) {
    return "Kinematic-body configuration";
  }
  if (matches.template operator()<comps::KinematicBodyStepTrace>()) {
    return "Kinematic-body step trace";
  }
  if (matches.template operator()<comps::RigidAvbdContactConfig>()) {
    return "Rigid AVBD contact configuration";
  }
  if (matches.template operator()<comps::CollisionGeometry>()) {
    return "Collision geometry";
  }
  if (matches.template operator()<comps::DeformableGroundBarrierTag>()) {
    return "Deformable ground-barrier tags";
  }
  if (matches.template operator()<comps::DeformableSurfaceCcdObstacleTag>()) {
    return "Deformable surface-CCD obstacle tags";
  }
  if (matches.template operator()<comps::DeformableObstacleNoCcdTag>()) {
    return "Deformable no-CCD obstacle tags";
  }
  if (matches.template operator()<comps::ContactMaterial>()) {
    return "Contact material properties";
  }
  if (matches.template operator()<comps::DeformableBodyTag>()) {
    return "Deformable-body tags";
  }
  if (matches.template operator()<comps::DeformableNodeModel>()) {
    return "Deformable-node model";
  }
  if (matches.template operator()<comps::DeformableNodeState>()) {
    return "Deformable-node state";
  }
  if (matches.template operator()<comps::DeformableSpringModel>()) {
    return "Deformable springs";
  }
  if (matches.template operator()<comps::DeformableMeshTopology>()) {
    return "Deformable mesh topology";
  }
  if (matches.template operator()<comps::DeformableMaterial>()) {
    return "Deformable materials";
  }
  if (matches.template operator()<comps::DeformableBoundaryConditions>()) {
    return "Deformable boundary conditions";
  }
  if (matches.template operator()<comps::DeformableVbdConfig>()) {
    return "Deformable VBD configuration";
  }
  if (matches.template operator()<comps::DeformableSolverScratch>()) {
    return "Deformable solver scratch";
  }
  if (matches.template operator()<comps::MultibodyTag>()) {
    return "Multibody tags";
  }
  if (matches.template operator()<comps::MultibodyStructure>()) {
    return "Multibody structure";
  }
  if (matches.template operator()<comps::LinkModel>()) {
    return "Multibody link model";
  }
  if (matches.template operator()<comps::LinkState>()) {
    return "Multibody link state";
  }
  if (matches.template operator()<comps::LinkControl>()) {
    return "Multibody link control";
  }
  if (matches.template operator()<comps::JointModel>()) {
    return "Joint model";
  }
  if (matches.template operator()<comps::JointState>()) {
    return "Joint state";
  }
  if (matches.template operator()<comps::JointActuation>()) {
    return "Joint actuation";
  }
  if (matches.template operator()<comps::AvbdJointStiffness>()) {
    return "AVBD joint stiffness";
  }
  if (matches.template operator()<comps::LoopClosure>()) {
    return "Loop closures";
  }
  if (matches.template operator()<comps::VariationalContact>()) {
    return "Variational contacts";
  }
  if (matches.template operator()<comps::VariationalContactDualState>()) {
    return "Variational contact dual state";
  }
  if (matches.template operator()<compute::MultibodyVariationalState>()) {
    return "Multibody variational state";
  }
  if (matches.template operator()<compute::MultibodyVariationalScratch>()) {
    return "Multibody variational scratch";
  }
  if (matches.template
      operator()<deformable_vbd::AvbdRigidWorldPointJointConfig>()) {
    return "AVBD rigid point-joint configuration";
  }
  if (matches.template
      operator()<deformable_vbd::AvbdRigidWorldDistanceSpringConfig>()) {
    return "AVBD rigid distance-spring configuration";
  }
  return "Other/internal storage";
}

struct TypedMemoryOverlay
{
  std::uintptr_t address = 0;
  std::size_t sizeBytes = 0;
  WorldMemoryDataCategory category{WorldMemoryDataCategory::Unclassified};
  WorldMemoryLogicalUse logicalUse{WorldMemoryLogicalUse::NotApplicable};
  std::string diagnosticLabel;
};

const char* logicalUseLabel(WorldMemoryLogicalUse logicalUse)
{
  switch (logicalUse) {
    case WorldMemoryLogicalUse::NotApplicable:
      return "not applicable";
    case WorldMemoryLogicalUse::Live:
      return "live";
    case WorldMemoryLogicalUse::Tombstone:
      return "tombstone";
    case WorldMemoryLogicalUse::Spare:
      return "spare";
  }
  return "unknown";
}

template <typename Storage>
WorldMemoryLogicalUse logicalUseAt(
    const Storage& storage, std::size_t slotIndex)
{
  if (slotIndex >= storage.size()) {
    return WorldMemoryLogicalUse::Spare;
  }
  const auto* packedEntities = storage.data();
  return packedEntities != nullptr
                 && storage.contains(packedEntities[slotIndex])
             ? WorldMemoryLogicalUse::Live
             : WorldMemoryLogicalUse::Tombstone;
}

template <typename Component>
void appendComponentPayloadPages(
    const WorldRegistry& registry,
    WorldMemoryDataCategory category,
    std::string_view label,
    std::vector<TypedMemoryOverlay>& overlays)
{
  constexpr std::size_t pageSlots
      = entt::component_traits<Component>::page_size;
  if constexpr (pageSlots == 0u) {
    return;
  } else {
    const auto* storage = registry.template storage<Component>();
    if (storage == nullptr || storage->capacity() == 0u) {
      return;
    }

    const auto pages = storage->raw();
    const std::size_t pageCount = storage->capacity() / pageSlots;
    for (std::size_t pageIndex = 0; pageIndex < pageCount; ++pageIndex) {
      if (pages[pageIndex] == nullptr) {
        continue;
      }
      const auto pageAddress
          = reinterpret_cast<std::uintptr_t>(pages[pageIndex]);
      constexpr std::size_t pageBytes = pageSlots * sizeof(Component);
      if (pageBytes
          > std::numeric_limits<std::uintptr_t>::max() - pageAddress) {
        continue;
      }
      const std::size_t firstSlot = pageIndex * pageSlots;
      std::size_t runBegin = 0u;
      while (runBegin < pageSlots) {
        const WorldMemoryLogicalUse logicalUse
            = logicalUseAt(*storage, firstSlot + runBegin);
        std::size_t runEnd = runBegin + 1u;
        while (runEnd < pageSlots
               && logicalUseAt(*storage, firstSlot + runEnd) == logicalUse) {
          ++runEnd;
        }
        const std::size_t runOffsetBytes = runBegin * sizeof(Component);
        const std::size_t runSizeBytes
            = (runEnd - runBegin) * sizeof(Component);
        overlays.push_back(
            TypedMemoryOverlay{
                .address = pageAddress + runOffsetBytes,
                .sizeBytes = runSizeBytes,
                .category = category,
                .logicalUse = logicalUse,
                .diagnosticLabel = std::string(label) + " payload page "
                                   + std::to_string(pageIndex) + " "
                                   + logicalUseLabel(logicalUse) + " slots ["
                                   + std::to_string(firstSlot + runBegin) + ", "
                                   + std::to_string(firstSlot + runEnd) + ")"});
        runBegin = runEnd;
      }
    }
  }
}

std::vector<TypedMemoryOverlay> collectTypedMemoryOverlays(
    const WorldRegistry& registry)
{
  std::vector<TypedMemoryOverlay> overlays;

  appendComponentPayloadPages<comps::Name>(
      registry, WorldMemoryDataCategory::SimulationMetadata, "Names", overlays);
  appendComponentPayloadPages<comps::FrameTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Frame tags",
      overlays);
  appendComponentPayloadPages<comps::FrameState>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Frame state",
      overlays);
  appendComponentPayloadPages<comps::FrameCache>(
      registry,
      WorldMemoryDataCategory::SimulationCache,
      "Frame transform cache",
      overlays);
  appendComponentPayloadPages<comps::FixedFrameProperties>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Fixed-frame properties",
      overlays);
  appendComponentPayloadPages<comps::FixedFrameTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Fixed-frame tags",
      overlays);
  appendComponentPayloadPages<comps::FreeFrameProperties>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Free-frame properties",
      overlays);
  appendComponentPayloadPages<comps::FreeFrameTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Free-frame tags",
      overlays);
  appendComponentPayloadPages<comps::Transform>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Transforms",
      overlays);
  appendComponentPayloadPages<comps::Velocity>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Velocities",
      overlays);
  appendComponentPayloadPages<comps::Force>(
      registry, WorldMemoryDataCategory::SimulationControl, "Forces", overlays);
  appendComponentPayloadPages<comps::MassProperties>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Mass properties",
      overlays);
  appendComponentPayloadPages<comps::DeactivationState>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Deactivation state",
      overlays);
  appendComponentPayloadPages<comps::RigidBodyTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Rigid-body tags",
      overlays);
  appendComponentPayloadPages<comps::StaticBodyTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Static-body tags",
      overlays);
  appendComponentPayloadPages<comps::KinematicBodyTag>(
      registry,
      WorldMemoryDataCategory::SimulationControl,
      "Kinematic-body configuration",
      overlays);
  appendComponentPayloadPages<comps::KinematicBodyStepTrace>(
      registry,
      WorldMemoryDataCategory::SimulationCache,
      "Kinematic-body step trace",
      overlays);
  appendComponentPayloadPages<comps::RigidAvbdContactConfig>(
      registry,
      WorldMemoryDataCategory::ContactSolver,
      "Rigid AVBD contact configuration",
      overlays);
  appendComponentPayloadPages<comps::CollisionGeometry>(
      registry,
      WorldMemoryDataCategory::CollisionGeometry,
      "Collision geometry",
      overlays);
  appendComponentPayloadPages<comps::DeformableGroundBarrierTag>(
      registry,
      WorldMemoryDataCategory::CollisionGeometry,
      "Deformable ground-barrier tags",
      overlays);
  appendComponentPayloadPages<comps::DeformableSurfaceCcdObstacleTag>(
      registry,
      WorldMemoryDataCategory::CollisionGeometry,
      "Deformable surface-CCD obstacle tags",
      overlays);
  appendComponentPayloadPages<comps::DeformableObstacleNoCcdTag>(
      registry,
      WorldMemoryDataCategory::CollisionGeometry,
      "Deformable no-CCD obstacle tags",
      overlays);
  appendComponentPayloadPages<comps::ContactMaterial>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Contact material properties",
      overlays);
  appendComponentPayloadPages<comps::DeformableBodyTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Deformable-body tags",
      overlays);
  appendComponentPayloadPages<comps::DeformableNodeModel>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Deformable-node model",
      overlays);
  appendComponentPayloadPages<comps::DeformableNodeState>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Deformable-node state",
      overlays);
  appendComponentPayloadPages<comps::DeformableSpringModel>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Deformable springs",
      overlays);
  appendComponentPayloadPages<comps::DeformableMeshTopology>(
      registry,
      WorldMemoryDataCategory::CollisionGeometry,
      "Deformable mesh topology",
      overlays);
  appendComponentPayloadPages<comps::DeformableMaterial>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Deformable materials",
      overlays);
  appendComponentPayloadPages<comps::DeformableBoundaryConditions>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Deformable boundary conditions",
      overlays);
  appendComponentPayloadPages<comps::DeformableVbdConfig>(
      registry,
      WorldMemoryDataCategory::ContactSolver,
      "Deformable VBD configuration",
      overlays);
  appendComponentPayloadPages<comps::DeformableSolverScratch>(
      registry,
      WorldMemoryDataCategory::SimulationCache,
      "Deformable solver scratch",
      overlays);
  appendComponentPayloadPages<comps::MultibodyTag>(
      registry,
      WorldMemoryDataCategory::SimulationMetadata,
      "Multibody tags",
      overlays);
  appendComponentPayloadPages<comps::MultibodyStructure>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Multibody structure",
      overlays);
  appendComponentPayloadPages<comps::LinkModel>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Multibody link model",
      overlays);
  appendComponentPayloadPages<comps::LinkState>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Multibody link state",
      overlays);
  appendComponentPayloadPages<comps::LinkControl>(
      registry,
      WorldMemoryDataCategory::SimulationControl,
      "Multibody link control",
      overlays);
  appendComponentPayloadPages<comps::JointModel>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Joint model",
      overlays);
  appendComponentPayloadPages<comps::JointState>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Joint state",
      overlays);
  appendComponentPayloadPages<comps::JointActuation>(
      registry,
      WorldMemoryDataCategory::SimulationControl,
      "Joint actuation",
      overlays);
  appendComponentPayloadPages<comps::AvbdJointStiffness>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "AVBD joint stiffness",
      overlays);
  appendComponentPayloadPages<comps::LoopClosure>(
      registry,
      WorldMemoryDataCategory::SimulationModel,
      "Loop closures",
      overlays);
  appendComponentPayloadPages<comps::VariationalContact>(
      registry,
      WorldMemoryDataCategory::ContactSolver,
      "Variational contacts",
      overlays);
  appendComponentPayloadPages<comps::VariationalContactDualState>(
      registry,
      WorldMemoryDataCategory::ContactSolver,
      "Variational contact dual state",
      overlays);
  appendComponentPayloadPages<compute::MultibodyVariationalState>(
      registry,
      WorldMemoryDataCategory::SimulationState,
      "Multibody variational state",
      overlays);
  appendComponentPayloadPages<compute::MultibodyVariationalScratch>(
      registry,
      WorldMemoryDataCategory::SimulationCache,
      "Multibody variational scratch",
      overlays);
  appendComponentPayloadPages<deformable_vbd::AvbdRigidWorldPointJointConfig>(
      registry,
      WorldMemoryDataCategory::ContactSolver,
      "AVBD rigid point-joint configuration",
      overlays);
  appendComponentPayloadPages<
      deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(
      registry,
      WorldMemoryDataCategory::ContactSolver,
      "AVBD rigid distance-spring configuration",
      overlays);

  for (auto&& [id, storage] : registry.storage()) {
    const auto* packedEntities = storage.data();
    if (packedEntities == nullptr || storage.size() == 0u
        || storage.size() > std::numeric_limits<std::size_t>::max()
                                / sizeof(entt::entity)) {
      continue;
    }
    // EnTT's type-erased storage exposes the packed entity pointer and its
    // materialized length, but not the underlying std::vector capacity.
    // `storage.capacity()` describes component payload-page capacity and can
    // exceed the entity vector allocation, so only this initialized prefix is
    // safe to classify as an exact entity-index range.
    const std::size_t packedBytes = storage.size() * sizeof(entt::entity);
    const auto packedAddress = reinterpret_cast<std::uintptr_t>(packedEntities);
    if (packedBytes
        > std::numeric_limits<std::uintptr_t>::max() - packedAddress) {
      continue;
    }
    std::size_t runBegin = 0u;
    while (runBegin < storage.size()) {
      const WorldMemoryLogicalUse logicalUse = logicalUseAt(storage, runBegin);
      std::size_t runEnd = runBegin + 1u;
      while (runEnd < storage.size()
             && logicalUseAt(storage, runEnd) == logicalUse) {
        ++runEnd;
      }
      overlays.push_back(
          TypedMemoryOverlay{
              .address = packedAddress + runBegin * sizeof(entt::entity),
              .sizeBytes = (runEnd - runBegin) * sizeof(entt::entity),
              .category = WorldMemoryDataCategory::EntityIndex,
              .logicalUse = logicalUse,
              .diagnosticLabel = diagnosticLabel(id) + " packed entity-index "
                                 + logicalUseLabel(logicalUse) + " slots ["
                                 + std::to_string(runBegin) + ", "
                                 + std::to_string(runEnd) + ")"});
      runBegin = runEnd;
    }
  }

  std::sort(
      overlays.begin(), overlays.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.address < rhs.address;
      });
  return overlays;
}

WorldMemorySpanState publicSpanState(
    common::detail::AllocatorMemorySpanState state)
{
  using InternalState = common::detail::AllocatorMemorySpanState;
  switch (state) {
    case InternalState::Metadata:
      return WorldMemorySpanState::Metadata;
    case InternalState::Allocated:
      return WorldMemorySpanState::Allocated;
    case InternalState::Free:
      return WorldMemorySpanState::Free;
    case InternalState::Reserved:
      return WorldMemorySpanState::Reserved;
    case InternalState::Padding:
      return WorldMemorySpanState::Padding;
  }
  return WorldMemorySpanState::Reserved;
}

WorldMemoryRegionKind publicRegionKind(
    common::detail::AllocatorMemoryRegionKind kind)
{
  using InternalKind = common::detail::AllocatorMemoryRegionKind;
  switch (kind) {
    case InternalKind::FreeListBacking:
      return WorldMemoryRegionKind::FreeListBacking;
    case InternalKind::FrameArena:
      return WorldMemoryRegionKind::FrameArena;
    case InternalKind::FrameOverflow:
      return WorldMemoryRegionKind::FrameOverflow;
  }
  return WorldMemoryRegionKind::FreeListBacking;
}

WorldMemoryDataCategory defaultCategory(
    common::detail::AllocatorMemoryRegionKind regionKind,
    common::detail::AllocatorMemorySpanState state)
{
  using InternalKind = common::detail::AllocatorMemoryRegionKind;
  using InternalState = common::detail::AllocatorMemorySpanState;
  switch (state) {
    case InternalState::Metadata:
      return WorldMemoryDataCategory::AllocatorInfrastructure;
    case InternalState::Padding:
    case InternalState::Free:
    case InternalState::Reserved:
      return WorldMemoryDataCategory::None;
    case InternalState::Allocated:
      return regionKind == InternalKind::FreeListBacking
                 ? WorldMemoryDataCategory::Unclassified
                 : WorldMemoryDataCategory::FrameScratch;
  }
  return WorldMemoryDataCategory::Unclassified;
}

std::string defaultSpanLabel(
    common::detail::AllocatorMemoryRegionKind regionKind,
    common::detail::AllocatorMemorySpanState state)
{
  using InternalKind = common::detail::AllocatorMemoryRegionKind;
  using InternalState = common::detail::AllocatorMemorySpanState;
  switch (state) {
    case InternalState::Metadata:
      return "Free-list block header";
    case InternalState::Free:
      return "Available free-list payload";
    case InternalState::Reserved:
      return "Reserved frame-scratch capacity";
    case InternalState::Padding:
      return "Backing allocation alignment padding";
    case InternalState::Allocated:
      if (regionKind == InternalKind::FrameArena) {
        return "Consumed frame-scratch arena (payload and padding)";
      }
      if (regionKind == InternalKind::FrameOverflow) {
        return "Frame-scratch overflow backing allocation";
      }
      return "Allocator-consumed block payload; requested payload and "
             "aligned-allocation overhead are not separately observable";
  }
  return "Unclassified allocator range";
}

std::string regionLabel(
    common::detail::AllocatorMemoryRegionKind kind, std::size_t kindIndex)
{
  using InternalKind = common::detail::AllocatorMemoryRegionKind;
  switch (kind) {
    case InternalKind::FreeListBacking:
      return "Free-list backing region " + std::to_string(kindIndex);
    case InternalKind::FrameArena:
      return "Frame-scratch arena";
    case InternalKind::FrameOverflow:
      return "Frame-scratch overflow region " + std::to_string(kindIndex);
  }
  return "Allocator backing region";
}

void appendPublicSpan(
    WorldMemoryRegionDiagnostics& destination,
    std::uintptr_t regionAddress,
    std::uintptr_t address,
    std::size_t sizeBytes,
    WorldMemorySpanState state,
    WorldMemoryDataCategory category,
    WorldMemoryEvidenceKind evidence,
    WorldMemoryLogicalUse logicalUse,
    std::string diagnosticLabel)
{
  if (sizeBytes == 0u) {
    return;
  }
  if (address < regionAddress) {
    return;
  }
  const std::size_t offsetBytes
      = static_cast<std::size_t>(address - regionAddress);
  if (offsetBytes > destination.sizeBytes
      || sizeBytes > destination.sizeBytes - offsetBytes) {
    return;
  }
  destination.spans.push_back(
      WorldMemorySpanDiagnostics{
          .offsetBytes = offsetBytes,
          .sizeBytes = sizeBytes,
          .state = state,
          .category = category,
          .evidence = evidence,
          .logicalUse = logicalUse,
          .diagnosticLabel = std::move(diagnosticLabel)});
}

void appendAllocatedSpanWithOverlays(
    WorldMemoryRegionDiagnostics& destination,
    const common::detail::AllocatorMemoryRegion& region,
    const common::detail::AllocatorMemorySpan& span,
    const std::vector<TypedMemoryOverlay>& overlays)
{
  if (span.sizeBytes
      > std::numeric_limits<std::uintptr_t>::max() - span.address) {
    return;
  }
  const std::uintptr_t spanEnd = span.address + span.sizeBytes;
  std::uintptr_t cursor = span.address;
  auto firstOverlay = std::lower_bound(
      overlays.begin(),
      overlays.end(),
      span.address,
      [](const TypedMemoryOverlay& overlay, std::uintptr_t address) {
        return overlay.address < address;
      });
  if (firstOverlay != overlays.begin()) {
    const auto previous = std::prev(firstOverlay);
    if (previous->sizeBytes
            <= std::numeric_limits<std::uintptr_t>::max() - previous->address
        && previous->address + previous->sizeBytes > span.address) {
      firstOverlay = previous;
    }
  }
  for (auto overlayIt = firstOverlay; overlayIt != overlays.end();
       ++overlayIt) {
    const TypedMemoryOverlay& overlay = *overlayIt;
    if (overlay.address >= spanEnd) {
      break;
    }
    if (overlay.sizeBytes
            > std::numeric_limits<std::uintptr_t>::max() - overlay.address
        || overlay.address < cursor || overlay.address < span.address
        || overlay.sizeBytes > spanEnd - overlay.address) {
      continue;
    }

    appendPublicSpan(
        destination,
        region.address,
        cursor,
        static_cast<std::size_t>(overlay.address - cursor),
        WorldMemorySpanState::Allocated,
        WorldMemoryDataCategory::Unclassified,
        WorldMemoryEvidenceKind::AllocatorBookkeeping,
        WorldMemoryLogicalUse::NotApplicable,
        "Allocator-consumed block payload; requested payload and "
        "aligned-allocation overhead are not separately observable");
    appendPublicSpan(
        destination,
        region.address,
        overlay.address,
        overlay.sizeBytes,
        WorldMemorySpanState::Allocated,
        overlay.category,
        WorldMemoryEvidenceKind::TypedPayloadOverlay,
        overlay.logicalUse,
        overlay.diagnosticLabel);
    cursor = overlay.address + overlay.sizeBytes;
  }
  appendPublicSpan(
      destination,
      region.address,
      cursor,
      static_cast<std::size_t>(spanEnd - cursor),
      WorldMemorySpanState::Allocated,
      WorldMemoryDataCategory::Unclassified,
      WorldMemoryEvidenceKind::AllocatorBookkeeping,
      WorldMemoryLogicalUse::NotApplicable,
      "Allocator-consumed block payload; requested payload and "
      "aligned-allocation overhead are not separately observable");
}

std::vector<WorldMemoryRegionDiagnostics> collectMemoryRegions(
    const common::MemoryManager& memoryManager, const WorldRegistry& registry)
{
  using Inspector = common::detail::AllocatorMemoryLayoutInspector;
  std::vector<common::detail::AllocatorMemoryRegion> rawRegions
      = Inspector::inspect(memoryManager.getFreeListAllocator());
  auto frameRegions = Inspector::inspect(memoryManager.getFrameAllocator());
  rawRegions.insert(
      rawRegions.end(),
      std::make_move_iterator(frameRegions.begin()),
      std::make_move_iterator(frameRegions.end()));
  std::sort(
      rawRegions.begin(),
      rawRegions.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.address < rhs.address;
      });

  const std::vector<TypedMemoryOverlay> overlays
      = collectTypedMemoryOverlays(registry);
  std::size_t freeListIndex = 0u;
  std::size_t overflowIndex = 0u;
  std::vector<WorldMemoryRegionDiagnostics> regions;
  regions.reserve(rawRegions.size());
  for (std::size_t addressOrder = 0; addressOrder < rawRegions.size();
       ++addressOrder) {
    const auto& rawRegion = rawRegions[addressOrder];
    std::size_t kindIndex = 0u;
    if (rawRegion.kind
        == common::detail::AllocatorMemoryRegionKind::FreeListBacking) {
      kindIndex = freeListIndex++;
    } else if (
        rawRegion.kind
        == common::detail::AllocatorMemoryRegionKind::FrameOverflow) {
      kindIndex = overflowIndex++;
    }

    WorldMemoryRegionDiagnostics region;
    region.addressOrder = addressOrder;
    region.kind = publicRegionKind(rawRegion.kind);
    region.diagnosticLabel = regionLabel(rawRegion.kind, kindIndex);
    region.sizeBytes = rawRegion.sizeBytes;
    for (const auto& rawSpan : rawRegion.spans) {
      if (rawRegion.kind
              == common::detail::AllocatorMemoryRegionKind::FreeListBacking
          && rawSpan.state
                 == common::detail::AllocatorMemorySpanState::Allocated) {
        appendAllocatedSpanWithOverlays(region, rawRegion, rawSpan, overlays);
        continue;
      }
      appendPublicSpan(
          region,
          rawRegion.address,
          rawSpan.address,
          rawSpan.sizeBytes,
          publicSpanState(rawSpan.state),
          defaultCategory(rawRegion.kind, rawSpan.state),
          WorldMemoryEvidenceKind::AllocatorBookkeeping,
          WorldMemoryLogicalUse::NotApplicable,
          defaultSpanLabel(rawRegion.kind, rawSpan.state));
    }
    regions.push_back(std::move(region));
  }
  return regions;
}

//==============================================================================
WorldEcsDiagnostics collectEcsDiagnostics(
    const WorldRegistry& registry, bool includeStorageLayoutDetails)
{
  WorldEcsDiagnostics diagnostics;
  diagnostics.storageLayoutDetailsIncluded = includeStorageLayoutDetails;

  const auto* entityStorage = registry.storage<entt::entity>();
  if (entityStorage != nullptr) {
    for (auto entity : *entityStorage) {
      if (registry.valid(entity)) {
        ++diagnostics.entityCount;
      }
    }
    diagnostics.entityCapacity = entityStorage->capacity();
  }

  for (auto&& [id, storage] : registry.storage()) {
    WorldEcsStorageDiagnostics storageDiagnostics;
    storageDiagnostics.storageId = static_cast<std::size_t>(id);
    storageDiagnostics.diagnosticLabel = diagnosticLabel(id);
    storageDiagnostics.packedSlotCount = storage.size();
    storageDiagnostics.capacity = storage.capacity();
    storageDiagnostics.sparseExtent = storage.extent();
    storageDiagnostics.unusedCapacity
        = storageDiagnostics.capacity > storageDiagnostics.packedSlotCount
              ? storageDiagnostics.capacity - storageDiagnostics.packedSlotCount
              : 0u;

    if (includeStorageLayoutDetails) {
      const auto* packedEntities = storage.data();
      bool previousSlotLive = false;
      for (std::size_t index = 0; index < storageDiagnostics.packedSlotCount;
           ++index) {
        const bool slotLive = packedEntities != nullptr
                              && storage.contains(packedEntities[index]);
        if (slotLive) {
          ++storageDiagnostics.size;
          if (!previousSlotLive) {
            ++storageDiagnostics.livePackedRegionCount;
          }
        } else {
          ++storageDiagnostics.holeCount;
        }
        previousSlotLive = slotLive;
      }
      storageDiagnostics.packedContiguous = storageDiagnostics.holeCount == 0u;
    } else {
      storageDiagnostics.size = storageDiagnostics.packedSlotCount;
      storageDiagnostics.packedContiguous = false;
    }

    diagnostics.componentCount += storageDiagnostics.size;
    diagnostics.componentCapacity += storageDiagnostics.capacity;
    diagnostics.storages.push_back(std::move(storageDiagnostics));
  }

  diagnostics.storageCount = diagnostics.storages.size();
  return diagnostics;
}

} // namespace

//==============================================================================
void MemoryDiagnosticsTracker::resetFrameScratch(
    common::MemoryManager& memoryManager)
{
  memoryManager.getFrameAllocator().reset();
  ++m_cached.frameScratchResetCount;
  recordFrameScratch(memoryManager);
}

//==============================================================================
void MemoryDiagnosticsTracker::recordFrameScratch(
    const common::MemoryManager& memoryManager)
{
  m_cached.allocatorDebugDiagnostics = memoryManager.getDebugDiagnostics();
  const auto& frameAllocator = memoryManager.getFrameAllocator();
  const auto overflowBytes = frameAllocator.overflowBytes();
  m_cached.frameScratchCapacityBytes = frameAllocator.usableCapacity();
  m_cached.frameScratchUsedBytes = frameAllocator.used() + overflowBytes;
  m_cached.frameScratchOverflowCount = frameAllocator.overflowCount();
  m_cached.frameScratchOverflowBytes = overflowBytes;
  m_cached.frameScratchPeakUsedBytes = std::max(
      m_cached.frameScratchPeakUsedBytes, m_cached.frameScratchUsedBytes);
}

//==============================================================================
WorldMemoryDiagnostics MemoryDiagnosticsTracker::collect(
    const common::MemoryManager& memoryManager,
    const WorldRegistry& registry,
    const WorldMemoryDiagnosticsOptions& options) const
{
  WorldMemoryDiagnostics diagnostics = m_cached;
  diagnostics.allocatorDebugDiagnostics = memoryManager.getDebugDiagnostics();
  diagnostics.memoryLayoutDetailsIncluded = options.includeMemoryLayoutDetails;
  diagnostics.memoryRegions.clear();
  if (options.includeMemoryLayoutDetails) {
    diagnostics.memoryRegions = collectMemoryRegions(memoryManager, registry);
  }
  diagnostics.ecsDiagnostics
      = collectEcsDiagnostics(registry, options.includeStorageLayoutDetails);

  const auto& frameAllocator = memoryManager.getFrameAllocator();
  const auto overflowBytes = frameAllocator.overflowBytes();
  diagnostics.frameScratchCapacityBytes = frameAllocator.usableCapacity();
  diagnostics.frameScratchUsedBytes = frameAllocator.used() + overflowBytes;
  diagnostics.frameScratchOverflowCount = frameAllocator.overflowCount();
  diagnostics.frameScratchOverflowBytes = overflowBytes;
  diagnostics.frameScratchPeakUsedBytes = std::max(
      diagnostics.frameScratchPeakUsedBytes, diagnostics.frameScratchUsedBytes);
  return diagnostics;
}

} // namespace dart::simulation::detail
