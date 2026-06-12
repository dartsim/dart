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
Eigen::Isometry3d getJointTransform(const comps::Joint& joint)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  switch (joint.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(joint.position[0], joint.axis).toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = joint.axis * joint.position[0];
      return transform;
    case comps::JointType::Screw:
      transform.linear()
          = Eigen::AngleAxisd(joint.position[0], joint.axis).toRotationMatrix();
      transform.translation() = joint.axis * joint.pitch * joint.position[0];
      return transform;
    case comps::JointType::Universal:
      transform.linear() = (Eigen::AngleAxisd(joint.position[0], joint.axis)
                            * Eigen::AngleAxisd(joint.position[1], joint.axis2))
                               .toRotationMatrix();
      return transform;
    case comps::JointType::Spherical:
      return rotationVectorTransform(joint.position.head<3>());
    case comps::JointType::Planar: {
      const Eigen::Vector3d normal = joint.axis.normalized();
      const Eigen::Vector3d axis1 = joint.axis2.normalized();
      const Eigen::Vector3d axis2 = normal.cross(axis1).normalized();
      transform.translation()
          = axis1 * joint.position[0] + axis2 * joint.position[1];
      transform.linear()
          = Eigen::AngleAxisd(joint.position[2], normal).toRotationMatrix();
      return transform;
    }
    case comps::JointType::Floating:
      transform.translation() = joint.position.head<3>();
      transform.linear()
          = rotationVectorTransform(joint.position.tail<3>()).linear();
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
  if (const auto* link = registry.try_get<comps::Link>(entity)) {
    if (link->parentJoint != entt::null) {
      const auto* joint = registry.try_get<comps::Joint>(link->parentJoint);
      DART_SIMULATION_THROW_T_IF(
          !joint,
          InvalidOperationException,
          "Link parent joint is missing a Joint component");

      return link->transformFromParentToJoint * getJointTransform(*joint)
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
ComputeResourceString makeFrameCacheResource(
    entt::entity entity, dart::common::MemoryAllocator& allocator)
{
  ComputeResourceString resource{dart::common::StlAllocator<char>{allocator}};
  resource.reserve(32);
  resource.append("comps::FrameCache#");

  char buffer[16];
  const auto id = static_cast<std::uint32_t>(entity);
  const auto [end, error]
      = std::to_chars(std::begin(buffer), std::end(buffer), id);
  DART_SIMULATION_THROW_T_IF(
      error != std::errc{},
      InvalidOperationException,
      "Failed to format FrameCache resource id");
  resource.append(buffer, end);
  return resource;
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
    m_entityNodes(EntityNodeAllocator{allocator})
{
  rebuild();
}

//==============================================================================
void WorldKinematicsGraph::rebuild()
{
  m_graph.clear();
  m_entityNodes.clear();

  auto* registry = &dart::simulation::detail::registryOf(m_world);
  auto frameView
      = registry->view<comps::FrameTag, comps::FrameState, comps::FrameCache>();

  for (auto entity : frameView) {
    const auto name = std::string("frame_")
                      + std::to_string(static_cast<std::uint32_t>(entity));

    ComputeStageMetadata metadata{
        ComputeStageDomain::Kinematics,
        ComputeStageAcceleration::TaskParallel
            | ComputeStageAcceleration::DataLocality,
        *m_allocator};
    metadata.resources.reserve(2);
    metadata.resources.push_back(
        {makeFrameCacheResource(entity, *m_allocator),
         ComputeAccessMode::Write});
    const auto buildParentFrame
        = registry->get<comps::FrameState>(entity).parentFrame;
    if (buildParentFrame != entt::null) {
      metadata.resources.push_back(
          {makeFrameCacheResource(buildParentFrame, *m_allocator),
           ComputeAccessMode::Read});
    }

    auto& node = m_graph.addNode(
        name,
        [registry, entity]() {
          const auto& frameState = registry->get<comps::FrameState>(entity);
          auto& cache = registry->get<comps::FrameCache>(entity);

          const auto localTransform = getLocalTransform(*registry, entity);
          if (frameState.parentFrame == entt::null) {
            cache.worldTransform = localTransform;
          } else {
            const auto* parentCache
                = registry->try_get<comps::FrameCache>(frameState.parentFrame);
            DART_SIMULATION_THROW_T_IF(
                !parentCache,
                InvalidOperationException,
                "Frame parent is missing a FrameCache component");

            cache.worldTransform = parentCache->worldTransform * localTransform;
          }

          cache.needTransformUpdate = false;
        },
        metadata);

    m_entityNodes.push_back({entity, &node});
  }

  for (const auto& entityNode : m_entityNodes) {
    const auto& frameState
        = registry->get<comps::FrameState>(entityNode.entity);
    auto* parentNode = findNode(frameState.parentFrame);
    if (parentNode) {
      m_graph.addDependency(*parentNode, *entityNode.node);
    }
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
  executor.execute(m_graph);
}

//==============================================================================
ComputeNode* WorldKinematicsGraph::findNode(entt::entity entity) const
{
  const auto it = std::ranges::find_if(
      m_entityNodes, [&](const auto& entry) { return entry.entity == entity; });

  if (it == m_entityNodes.end()) {
    return nullptr;
  }

  return it->node;
}

} // namespace dart::simulation::compute
