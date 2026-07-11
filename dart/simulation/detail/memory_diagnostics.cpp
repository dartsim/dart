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

#include "dart/simulation/comps/all.hpp"

#include <algorithm>
#include <utility>

#include <cstddef>

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
  if (matches.template operator()<comps::CollisionGeometry>()) {
    return "Collision geometry";
  }
  if (matches.template operator()<comps::ContactMaterial>()) {
    return "Contact materials";
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
  return "Other/internal storage";
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
