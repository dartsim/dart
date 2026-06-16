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

#include "dart/simulation/body/deformable_body.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/world.hpp"

namespace {

//==============================================================================
void validateFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName)
{
  DART_SIMULATION_THROW_T_IF(
      !value.allFinite(),
      dart::simulation::InvalidArgumentException,
      "DeformableBody {} must contain only finite values",
      fieldName);
}

} // namespace

namespace dart::simulation {

//==============================================================================
DeformableBody::DeformableBody(Entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
bool DeformableBody::isValid() const noexcept
{
  if (m_world == nullptr || m_entity == Entity{}) {
    return false;
  }

  const auto& registry = detail::registryOf(*m_world);
  const auto entity = detail::toRegistryEntity(m_entity);
  return registry.valid(entity)
         && registry.all_of<comps::DeformableBodyTag>(entity);
}

//==============================================================================
std::string DeformableBody::getName() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto entity = detail::toRegistryEntity(m_entity);
  if (const auto* name
      = detail::registryOf(*m_world).try_get<comps::Name>(entity)) {
    return name->name;
  }

  return "";
}

//==============================================================================
std::size_t DeformableBody::getNodeCount() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return detail::registryOf(*m_world)
      .get<comps::DeformableNodeState>(detail::toRegistryEntity(m_entity))
      .positions.size();
}

//==============================================================================
Eigen::Vector3d DeformableBody::getPosition(std::size_t node) const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& state
      = detail::registryOf(*m_world).get<comps::DeformableNodeState>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");
  validateFiniteVector(position, "position");

  auto& state = detail::registryOf(*m_world).get<comps::DeformableNodeState>(
      detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& state
      = detail::registryOf(*m_world).get<comps::DeformableNodeState>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");
  validateFiniteVector(velocity, "velocity");

  auto& state = detail::registryOf(*m_world).get<comps::DeformableNodeState>(
      detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
      node >= state.velocities.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  state.velocities[node] = velocity;
}

//==============================================================================
double DeformableBody::getMass(std::size_t node) const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& model
      = detail::registryOf(*m_world).get<comps::DeformableNodeModel>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
      node >= model.masses.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  return model.masses[node];
}

//==============================================================================
bool DeformableBody::isFixedNode(std::size_t node) const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& model
      = detail::registryOf(*m_world).get<comps::DeformableNodeModel>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
      node >= model.fixed.size(),
      OutOfRangeException,
      "DeformableBody node index {} is out of range",
      node);

  return model.fixed[node] != 0u;
}

//==============================================================================
std::size_t DeformableBody::getEdgeCount() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return detail::registryOf(*m_world)
      .get<comps::DeformableSpringModel>(detail::toRegistryEntity(m_entity))
      .edges.size();
}

//==============================================================================
DeformableEdge DeformableBody::getEdge(std::size_t edge) const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& model
      = detail::registryOf(*m_world).get<comps::DeformableSpringModel>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return detail::registryOf(*m_world)
      .get<comps::DeformableMeshTopology>(detail::toRegistryEntity(m_entity))
      .surfaceTriangles.size();
}

//==============================================================================
DeformableSurfaceTriangle DeformableBody::getSurfaceTriangle(
    std::size_t triangle) const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& topology
      = detail::registryOf(*m_world).get<comps::DeformableMeshTopology>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  return detail::registryOf(*m_world)
      .get<comps::DeformableMeshTopology>(detail::toRegistryEntity(m_entity))
      .tetrahedra.size();
}

//==============================================================================
DeformableTetrahedron DeformableBody::getTetrahedron(
    std::size_t tetrahedron) const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& topology
      = detail::registryOf(*m_world).get<comps::DeformableMeshTopology>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
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
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& topology
      = detail::registryOf(*m_world).get<comps::DeformableMeshTopology>(
          detail::toRegistryEntity(m_entity));
  DART_SIMULATION_THROW_T_IF(
      tetrahedron >= topology.tetrahedronRestVolumes.size(),
      OutOfRangeException,
      "DeformableBody tetrahedron index {} is out of range",
      tetrahedron);

  return topology.tetrahedronRestVolumes[tetrahedron];
}

//==============================================================================
DeformableMaterialProperties DeformableBody::getMaterialProperties() const
{
  DART_SIMULATION_THROW_T_IF(
      !isValid(), InvalidArgumentException, "Invalid deformable body handle");

  const auto& material
      = detail::registryOf(*m_world).get<comps::DeformableMaterial>(
          detail::toRegistryEntity(m_entity));
  return DeformableMaterialProperties{
      material.density,
      material.youngsModulus,
      material.poissonRatio,
      material.frictionCoefficient,
      material.useFiniteElementElasticity,
      material.useFixedCorotationalElasticity,
      material.useAdaptiveBarrierStiffness,
      material.useIterativeLinearSolver,
      material.useMatrixFreeLinearSolver};
}

} // namespace dart::simulation
