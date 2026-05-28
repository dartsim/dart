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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/simulation/experimental/body/deformable_body.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <entt/entt.hpp>

namespace {

//==============================================================================
void validateFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !value.allFinite(),
      dart::simulation::experimental::InvalidArgumentException,
      "DeformableBody {} must contain only finite values",
      fieldName);
}

//==============================================================================
entt::entity toEntity(std::uint32_t entityId)
{
  return static_cast<entt::entity>(entityId);
}

} // namespace

namespace dart::simulation::experimental {

//==============================================================================
DeformableBody::DeformableBody(std::uint32_t entityId, World* world)
  : m_entityId(entityId), m_world(world)
{
}

//==============================================================================
bool DeformableBody::isValid() const noexcept
{
  if (m_world == nullptr || m_entityId == 0xffffffffu) {
    return false;
  }

  const auto& registry = m_world->getRegistry();
  const auto entity = toEntity(m_entityId);
  return registry.valid(entity)
         && registry.all_of<comps::DeformableBodyTag>(entity);
}

//==============================================================================
std::string DeformableBody::getName() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto entity = toEntity(m_entityId);
  if (const auto* name = m_world->getRegistry().try_get<comps::Name>(entity)) {
    return name->name;
  }

  return "";
}

//==============================================================================
std::size_t DeformableBody::getNodeCount() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return m_world->getRegistry()
      .get<comps::DeformableNodeState>(toEntity(m_entityId))
      .positions.size();
}

//==============================================================================
Eigen::Vector3d DeformableBody::getPosition(std::size_t node) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& state = m_world->getRegistry().get<comps::DeformableNodeState>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      node >= state.positions.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  return state.positions[node];
}

//==============================================================================
void DeformableBody::setPosition(
    std::size_t node, const Eigen::Vector3d& position)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");
  validateFiniteVector(position, "position");

  auto& state = m_world->getRegistry().get<comps::DeformableNodeState>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      node >= state.positions.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  state.positions[node] = position;
  state.previousPositions[node] = position;
}

//==============================================================================
Eigen::Vector3d DeformableBody::getVelocity(std::size_t node) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& state = m_world->getRegistry().get<comps::DeformableNodeState>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      node >= state.velocities.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  return state.velocities[node];
}

//==============================================================================
void DeformableBody::setVelocity(
    std::size_t node, const Eigen::Vector3d& velocity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");
  validateFiniteVector(velocity, "velocity");

  auto& state = m_world->getRegistry().get<comps::DeformableNodeState>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      node >= state.velocities.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  state.velocities[node] = velocity;
}

//==============================================================================
double DeformableBody::getMass(std::size_t node) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& state = m_world->getRegistry().get<comps::DeformableNodeState>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      node >= state.masses.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  return state.masses[node];
}

//==============================================================================
bool DeformableBody::isFixedNode(std::size_t node) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& state = m_world->getRegistry().get<comps::DeformableNodeState>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      node >= state.fixed.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  return state.fixed[node] != 0u;
}

//==============================================================================
std::size_t DeformableBody::getEdgeCount() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return m_world->getRegistry()
      .get<comps::DeformableSpringModel>(toEntity(m_entityId))
      .edges.size();
}

//==============================================================================
DeformableEdge DeformableBody::getEdge(std::size_t edge) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& model = m_world->getRegistry().get<comps::DeformableSpringModel>(
      toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      edge >= model.edges.size(),
      OutOfRangeException,
      "DeformableBody edge index {} is out of range",
      edge);

  const auto& internal = model.edges[edge];
  return DeformableEdge{internal.nodeA, internal.nodeB, internal.restLength};
}

//==============================================================================
std::size_t DeformableBody::getSurfaceTriangleCount() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return m_world->getRegistry()
      .get<comps::DeformableMeshTopology>(toEntity(m_entityId))
      .surfaceTriangles.size();
}

//==============================================================================
DeformableSurfaceTriangle DeformableBody::getSurfaceTriangle(
    std::size_t triangle) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& topology
      = m_world->getRegistry().get<comps::DeformableMeshTopology>(
          toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      triangle >= topology.surfaceTriangles.size(),
      OutOfRangeException,
      "DeformableBody surface triangle index {} is out of range",
      triangle);

  const auto& internal = topology.surfaceTriangles[triangle];
  return DeformableSurfaceTriangle{
      internal.nodeA, internal.nodeB, internal.nodeC};
}

//==============================================================================
std::size_t DeformableBody::getTetrahedronCount() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return m_world->getRegistry()
      .get<comps::DeformableMeshTopology>(toEntity(m_entityId))
      .tetrahedra.size();
}

//==============================================================================
DeformableTetrahedron DeformableBody::getTetrahedron(
    std::size_t tetrahedron) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& topology
      = m_world->getRegistry().get<comps::DeformableMeshTopology>(
          toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      tetrahedron >= topology.tetrahedra.size(),
      OutOfRangeException,
      "DeformableBody tetrahedron index {} is out of range",
      tetrahedron);

  const auto& internal = topology.tetrahedra[tetrahedron];
  return DeformableTetrahedron{
      internal.nodeA, internal.nodeB, internal.nodeC, internal.nodeD};
}

//==============================================================================
double DeformableBody::getTetrahedronRestVolume(std::size_t tetrahedron) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& topology
      = m_world->getRegistry().get<comps::DeformableMeshTopology>(
          toEntity(m_entityId));
  DART_EXPERIMENTAL_THROW_T_IF(
      tetrahedron >= topology.tetrahedronRestVolumes.size(),
      OutOfRangeException,
      "DeformableBody tetrahedron index {} is out of range",
      tetrahedron);

  return topology.tetrahedronRestVolumes[tetrahedron];
}

//==============================================================================
DeformableMaterialProperties DeformableBody::getMaterialProperties() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& material = m_world->getRegistry().get<comps::DeformableMaterial>(
      toEntity(m_entityId));
  return DeformableMaterialProperties{
      material.density, material.youngsModulus, material.poissonRatio};
}

} // namespace dart::simulation::experimental
