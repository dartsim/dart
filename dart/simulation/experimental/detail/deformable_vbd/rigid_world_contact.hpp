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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/comps/contact_material.hpp>
#include <dart/simulation/experimental/comps/dynamics.hpp>
#include <dart/simulation/experimental/comps/rigid_body.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/rigid_block_kernel.hpp>

#include <entt/entt.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <span>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

struct AvbdRigidWorldContactOptions
{
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldContactSnapshot
{
  std::vector<entt::entity> entities;
  std::vector<AvbdRigidBodyState> states;
  std::vector<double> masses;
  std::vector<Eigen::Matrix3d> bodyInertias;
  std::vector<std::uint8_t> fixed;
  std::vector<AvbdRigidContactManifoldPoint> contacts;
};

//==============================================================================
inline std::uint64_t avbdRigidWorldContactObjectId(entt::entity entity) noexcept
{
  return static_cast<std::uint64_t>(entt::to_integral(entity)) + 1u;
}

namespace detail {

//==============================================================================
inline double avbdRigidWorldContactFriction(
    const entt::registry& registry, entt::entity entity)
{
  const auto* material = registry.try_get<comps::ContactMaterial>(entity);
  if (material == nullptr || !std::isfinite(material->friction)) {
    return 1.0;
  }
  return std::max(0.0, material->friction);
}

//==============================================================================
inline std::uint32_t findAvbdRigidWorldBodyIndex(
    const AvbdRigidWorldContactSnapshot& snapshot, entt::entity entity)
{
  const auto it
      = std::find(snapshot.entities.begin(), snapshot.entities.end(), entity);
  if (it == snapshot.entities.end()) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  return static_cast<std::uint32_t>(
      std::distance(snapshot.entities.begin(), it));
}

//==============================================================================
inline std::uint32_t ensureAvbdRigidWorldBodyIndex(
    const entt::registry& registry,
    AvbdRigidWorldContactSnapshot& snapshot,
    entt::entity entity)
{
  const std::uint32_t existing = findAvbdRigidWorldBodyIndex(snapshot, entity);
  if (existing != std::numeric_limits<std::uint32_t>::max()) {
    return existing;
  }

  if (!registry.all_of<
          comps::RigidBodyTag,
          comps::Transform,
          comps::MassProperties>(entity)) {
    return std::numeric_limits<std::uint32_t>::max();
  }

  const auto& transform = registry.get<comps::Transform>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);

  snapshot.entities.push_back(entity);
  snapshot.states.push_back(
      AvbdRigidBodyState{
          transform.position,
          normalizeAvbdRigidOrientation(transform.orientation)});
  snapshot.masses.push_back(mass.mass);
  snapshot.bodyInertias.push_back(mass.inertia);
  snapshot.fixed.push_back(
      registry.all_of<comps::StaticBodyTag>(entity) ? 1u : 0u);
  return static_cast<std::uint32_t>(snapshot.entities.size() - 1u);
}

} // namespace detail

//==============================================================================
inline AvbdRigidWorldContactSnapshot buildAvbdRigidWorldContactSnapshot(
    const entt::registry& registry,
    std::span<const Contact> contacts,
    const AvbdRigidWorldContactOptions& options = {})
{
  AvbdRigidWorldContactSnapshot snapshot;
  snapshot.contacts.reserve(contacts.size());

  for (std::size_t contactIndex = 0; contactIndex < contacts.size();
       ++contactIndex) {
    const Contact& contact = contacts[contactIndex];
    const entt::entity entityA = contact.bodyA.getEntity();
    const entt::entity entityB = contact.bodyB.getEntity();
    if (entityA == entt::null || entityB == entt::null || entityA == entityB
        || contact.depth <= 0.0 || !std::isfinite(contact.depth)
        || !contact.point.allFinite() || !contact.normal.allFinite()
        || contact.normal.squaredNorm() <= 0.0) {
      continue;
    }

    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const bool staticA = registry.all_of<comps::StaticBodyTag>(entityA);
    const bool staticB = registry.all_of<comps::StaticBodyTag>(entityB);
    if (staticA && staticB) {
      continue;
    }

    const std::uint32_t bodyA
        = detail::ensureAvbdRigidWorldBodyIndex(registry, snapshot, entityA);
    const std::uint32_t bodyB
        = detail::ensureAvbdRigidWorldBodyIndex(registry, snapshot, entityB);
    if (bodyA == std::numeric_limits<std::uint32_t>::max()
        || bodyB == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }

    AvbdRigidContactManifoldPoint manifoldPoint;
    manifoldPoint.bodyA = bodyA;
    manifoldPoint.bodyB = bodyB;
    manifoldPoint.endpointA = AvbdContactEndpointId{
        avbdRigidWorldContactObjectId(entityA),
        packAvbdContactFeatureId(AvbdContactFeatureKind::Body, 0)};
    manifoldPoint.endpointB = AvbdContactEndpointId{
        avbdRigidWorldContactObjectId(entityB),
        packAvbdContactFeatureId(AvbdContactFeatureKind::Body, 0)};
    manifoldPoint.point = contact.point;
    manifoldPoint.normalFromAtoB = contact.normal;
    manifoldPoint.depth = contact.depth;
    manifoldPoint.frictionCoefficient = std::sqrt(
        detail::avbdRigidWorldContactFriction(registry, entityA)
        * detail::avbdRigidWorldContactFriction(registry, entityB));
    manifoldPoint.startStiffness = options.startStiffness;
    manifoldPoint.maxStiffness = options.maxStiffness;
    manifoldPoint.row = static_cast<std::uint32_t>(std::min<std::size_t>(
        contactIndex, std::numeric_limits<std::uint32_t>::max()));
    snapshot.contacts.push_back(manifoldPoint);
  }

  return snapshot;
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
