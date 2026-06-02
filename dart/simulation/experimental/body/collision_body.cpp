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

#include "dart/simulation/experimental/body/collision_body.hpp"

#include "dart/simulation/experimental/body/rigid_body.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/detail/entity_conversion.hpp"
#include "dart/simulation/experimental/multibody/link.hpp"
#include "dart/simulation/experimental/world.hpp"

namespace dart::simulation::experimental {

//==============================================================================
CollisionBody::CollisionBody(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
entt::entity CollisionBody::getEntity() const
{
  return m_entity;
}

//==============================================================================
World* CollisionBody::getWorld() const
{
  return m_world;
}

//==============================================================================
bool CollisionBody::isValid() const
{
  return m_world != nullptr && m_world->getRegistry().valid(m_entity);
}

//==============================================================================
std::string CollisionBody::getName() const
{
  if (!isValid()) {
    return {};
  }
  const auto& registry = m_world->getRegistry();
  if (const auto* name = registry.try_get<comps::Name>(m_entity)) {
    return name->name;
  }
  return {};
}

//==============================================================================
bool CollisionBody::isRigidBody() const
{
  return isValid()
         && m_world->getRegistry().all_of<comps::RigidBodyTag>(m_entity);
}

//==============================================================================
bool CollisionBody::isLink() const
{
  return isValid() && m_world->getRegistry().all_of<comps::Link>(m_entity);
}

//==============================================================================
std::optional<RigidBody> CollisionBody::asRigidBody() const
{
  if (!isRigidBody()) {
    return std::nullopt;
  }
  return RigidBody(detail::fromRegistryEntity(m_entity), m_world);
}

//==============================================================================
std::optional<Link> CollisionBody::asLink() const
{
  if (!isLink()) {
    return std::nullopt;
  }
  return Link(detail::fromRegistryEntity(m_entity), m_world);
}

} // namespace dart::simulation::experimental
