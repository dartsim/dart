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

#include "dart/simulation/compute/world_kinematics_graph.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/link.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <charconv>
#include <string>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute {

namespace {

//==============================================================================
Eigen::Isometry3d rotationVectorTransform(const Eigen::Vector3d& rotation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const double angle = rotation.norm();
  if (angle > 1e-12) {
    transform.linear()
        = Eigen::AngleAxisd(angle, rotation / angle).toRotationMatrix();
  }
  return transform;
}

//==============================================================================
Eigen::Isometry3d getJointTransform(
    const comps::JointModel& jointModel, const comps::JointState& jointState)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  switch (jointModel.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(jointState.position[0], jointModel.axis)
                .toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = jointModel.axis * jointState.position[0];
      return transform;
    case comps::JointType::Screw:
      transform.linear()
          = Eigen::AngleAxisd(jointState.position[0], jointModel.axis)
                .toRotationMatrix();
      transform.translation()
          = jointModel.axis * jointModel.pitch * jointState.position[0];
      return transform;
    case comps::JointType::Universal:
      transform.linear()
          = (Eigen::AngleAxisd(jointState.position[0], jointModel.axis)
             * Eigen::AngleAxisd(jointState.position[1], jointModel.axis2))
                .toRotationMatrix();
      return transform;
    case comps::JointType::Spherical:
      return rotationVectorTransform(jointState.position.head<3>());
    case comps::JointType::Planar: {
      const Eigen::Vector3d normal = jointModel.axis.normalized();
      const Eigen::Vector3d axis1 = jointModel.axis2.normalized();
      const Eigen::Vector3d axis2 = normal.cross(axis1).normalized();
      transform.translation()
          = axis1 * jointState.position[0] + axis2 * jointState.position[1];
      transform.linear() = Eigen::AngleAxisd(jointState.position[2], normal)
                               .toRotationMatrix();
      return transform;
    }
    case comps::JointType::Floating:
      transform.translation() = jointState.position.head<3>();
      transform.linear()
          = rotationVectorTransform(jointState.position.tail<3>()).linear();
      return transform;
    case comps::JointType::Custom:
      DART_SIMULATION_THROW_T(
          InvalidOperationException,
          "Custom joints require a custom kinematics stage");
  }

  return transform;
}

//==============================================================================
Eigen::Isometry3d getLocalTransform(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* link = registry.try_get<comps::LinkModel>(entity)) {
    if (link->parentJoint != entt::null) {
      const auto* jointModel
          = registry.try_get<comps::JointModel>(link->parentJoint);
      DART_SIMULATION_THROW_T_IF(
          !jointModel,
          InvalidOperationException,
          "Link parent joint is missing a Joint component");

      const auto* jointState
          = registry.try_get<comps::JointState>(link->parentJoint);
      DART_SIMULATION_THROW_T_IF(
          !jointState,
          InvalidOperationException,
          "Link parent joint is missing a Joint component");

      return link->transformFromParentToJoint
             * getJointTransform(*jointModel, *jointState)
             * link->transformFromParentJoint;
    }
  }

  if (const auto* fixed
      = registry.try_get<comps::FixedFrameProperties>(entity)) {
    return fixed->localTransform;
  }

  if (const auto* free = registry.try_get<comps::FreeFrameProperties>(entity)) {
    return free->localTransform;
  }

  return Eigen::Isometry3d::Identity();
}

//==============================================================================
struct FrameRecord
{
  entt::entity entity = entt::null;
  std::size_t level = 0;
};

//==============================================================================
void appendNumber(
    ComputeResourceString& text,
    std::size_t value,
    std::string_view failureMessage)
{
  char buffer[24];
  const auto [end, error]
      = std::to_chars(std::begin(buffer), std::end(buffer), value);
  DART_SIMULATION_THROW_T_IF(
      error != std::errc{}, InvalidOperationException, "{}", failureMessage);
  text.append(buffer, end);
}

//==============================================================================
ComputeResourceString makeFrameCacheChunkResource(
    std::size_t level,
    std::size_t chunk,
    dart::common::MemoryAllocator& allocator)
{
  ComputeResourceString resource{dart::common::StlAllocator<char>{allocator}};
  resource.reserve(40);
  resource.append("comps::FrameCacheLevel#");
  appendNumber(resource, level, "Failed to format FrameCache level id");
  resource.append(":chunk#");
  appendNumber(resource, chunk, "Failed to format FrameCache chunk id");
  return resource;
}

//==============================================================================
std::size_t findFrameRecordIndex(
    const auto& records, const entt::entity entity) noexcept
{
  const auto it = std::ranges::find_if(records, [entity](const auto& record) {
    return record.entity == entity;
  });
  if (it == records.end()) {
    return records.size();
  }
  return static_cast<std::size_t>(std::distance(records.begin(), it));
}

//==============================================================================
std::size_t computeFrameLevel(
    const detail::WorldRegistry& registry,
    auto& records,
    auto& visitState,
    const std::size_t index)
{
  if (visitState[index] == 2) {
    return records[index].level;
  }

  DART_SIMULATION_THROW_T_IF(
      visitState[index] == 1,
      InvalidOperationException,
      "Frame topology contains a parent cycle");

  visitState[index] = 1;

  const auto& frameState
      = registry.get<comps::FrameState>(records[index].entity);
  std::size_t level = 0;
  if (frameState.parentFrame != entt::null) {
    const auto parentIndex
        = findFrameRecordIndex(records, frameState.parentFrame);
    if (parentIndex != records.size()) {
      level
          = computeFrameLevel(registry, records, visitState, parentIndex) + 1u;
    }
  }

  records[index].level = level;
  visitState[index] = 2;
  return level;
}

//==============================================================================
bool hasDirtyFrameCaches(const detail::WorldRegistry& registry)
{
  const auto frameView = registry.view<comps::FrameCache>();
  for (const auto entity : frameView) {
    if (frameView.get<comps::FrameCache>(entity).needTransformUpdate) {
      return true;
    }
  }
  return false;
}

} // namespace

//==============================================================================
WorldKinematicsGraph::WorldKinematicsGraph(World& world)
  : WorldKinematicsGraph(world, dart::common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
WorldKinematicsGraph::WorldKinematicsGraph(
    World& world, dart::common::MemoryAllocator& allocator)
  : m_world(world),
    m_allocator(&allocator),
    m_graph(allocator),
    m_frameEntities(EntityAllocator{allocator}),
    m_frameLevels(FrameLevelAllocator{allocator})
{
  rebuild();
}

//==============================================================================
void WorldKinematicsGraph::rebuild()
{
  rebuildForWorkerCount(m_loweredWorkerCount);
}

//==============================================================================
void WorldKinematicsGraph::rebuildForWorkerCount(std::size_t workerCount)
{
  workerCount = std::max<std::size_t>(1u, workerCount);
  m_graph.clear();
  m_frameEntities.clear();
  m_frameLevels.clear();
  m_loweredWorkerCount = workerCount;

  auto* registry = &dart::simulation::detail::registryOf(m_world);
  auto frameView
      = registry->view<comps::FrameTag, comps::FrameState, comps::FrameCache>();

  using FrameRecordAllocator = dart::common::StlAllocator<FrameRecord>;
  std::vector<FrameRecord, FrameRecordAllocator> frameRecords{
      FrameRecordAllocator{*m_allocator}};
  frameRecords.reserve(frameView.size_hint());
  for (auto entity : frameView) {
    frameRecords.push_back({entity, 0u});
  }

  std::vector<std::uint8_t, dart::common::StlAllocator<std::uint8_t>>
      visitState{dart::common::StlAllocator<std::uint8_t>{*m_allocator}};
  visitState.assign(frameRecords.size(), 0u);

  std::size_t maxLevel = 0;
  for (std::size_t i = 0; i < frameRecords.size(); ++i) {
    maxLevel = std::max(
        maxLevel, computeFrameLevel(*registry, frameRecords, visitState, i));
  }

  m_frameLevels.reserve(maxLevel + 1u);
  m_frameEntities.reserve(frameRecords.size());
  for (std::size_t level = 0; level <= maxLevel && !frameRecords.empty();
       ++level) {
    const std::size_t begin = m_frameEntities.size();
    for (const auto& record : frameRecords) {
      if (record.level == level) {
        m_frameEntities.push_back(record.entity);
      }
    }
    m_frameLevels.push_back({begin, m_frameEntities.size()});
  }

  std::vector<ComputeNode*, dart::common::StlAllocator<ComputeNode*>>
      previousLevelNodes{
          dart::common::StlAllocator<ComputeNode*>{*m_allocator}};
  std::vector<ComputeNode*, dart::common::StlAllocator<ComputeNode*>>
      currentLevelNodes{dart::common::StlAllocator<ComputeNode*>{*m_allocator}};

  for (std::size_t level = 0; level < m_frameLevels.size(); ++level) {
    const auto [levelBegin, levelEnd] = m_frameLevels[level];
    const std::size_t levelSize = levelEnd - levelBegin;
    if (levelSize == 0u) {
      continue;
    }

    const std::size_t chunkCount = std::min(workerCount, levelSize);
    const std::size_t chunkSize = (levelSize + chunkCount - 1u) / chunkCount;
    currentLevelNodes.clear();
    currentLevelNodes.reserve(chunkCount);

    for (std::size_t chunk = 0; chunk < chunkCount; ++chunk) {
      const std::size_t begin = levelBegin + chunk * chunkSize;
      const std::size_t end = std::min(begin + chunkSize, levelEnd);
      if (begin == end) {
        continue;
      }

      ComputeStageMetadata metadata{
          ComputeStageDomain::Kinematics,
          ComputeStageAcceleration::TaskParallel
              | ComputeStageAcceleration::DataLocality,
          *m_allocator};
      metadata.resources.reserve(1);
      metadata.resources.push_back(
          {makeFrameCacheChunkResource(level, chunk, *m_allocator),
           ComputeAccessMode::Write});

      auto& node = m_graph.addNode(
          "kinematics_level_" + std::to_string(level) + "_chunk_"
              + std::to_string(chunk),
          [registry, entityList = &m_frameEntities, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
              const auto entity = (*entityList)[i];
              const auto& frameState = registry->get<comps::FrameState>(entity);
              auto& cache = registry->get<comps::FrameCache>(entity);

              const auto localTransform = getLocalTransform(*registry, entity);
              if (frameState.parentFrame == entt::null) {
                cache.worldTransform = localTransform;
              } else {
                const auto* parentCache = registry->try_get<comps::FrameCache>(
                    frameState.parentFrame);
                DART_SIMULATION_THROW_T_IF(
                    !parentCache,
                    InvalidOperationException,
                    "Frame parent is missing a FrameCache component");

                cache.worldTransform
                    = parentCache->worldTransform * localTransform;
              }

              cache.needTransformUpdate = false;
            }
          },
          metadata);
      currentLevelNodes.push_back(&node);
    }

    for (auto* parentNode : previousLevelNodes) {
      for (auto* childNode : currentLevelNodes) {
        m_graph.addDependency(*parentNode, *childNode);
      }
    }
    previousLevelNodes.swap(currentLevelNodes);
  }

  (void)m_graph.getTopologicalOrderView();
  m_frameTopologyRevision = m_world.getFrameTopologyRevision();
}

//==============================================================================
bool WorldKinematicsGraph::isTopologyCurrent() const noexcept
{
  return m_frameTopologyRevision == m_world.getFrameTopologyRevision();
}

//==============================================================================
void WorldKinematicsGraph::execute(ComputeExecutor& executor)
{
  if (!m_world.isStepProfilingEnabled()
      && !hasDirtyFrameCaches(dart::simulation::detail::registryOf(m_world))) {
    return;
  }
  const auto workerCount = std::max<std::size_t>(1u, executor.getWorkerCount());
  if (!isTopologyCurrent() || m_loweredWorkerCount != workerCount) {
    rebuildForWorkerCount(workerCount);
  }
  executor.execute(m_graph);
}

} // namespace dart::simulation::compute
