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

#include "dart/simulation/experimental/body/rigid_body.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/world.hpp"

namespace {

void syncFrameFromTransform(entt::registry& registry, entt::entity entity)
{
  const auto& transform
      = registry.get<dart::simulation::experimental::comps::Transform>(entity);

  if (auto* props
      = registry.try_get<
          dart::simulation::experimental::comps::FreeFrameProperties>(entity)) {
    props->localTransform = Eigen::Isometry3d::Identity();
    props->localTransform.translation() = transform.position;
    props->localTransform.linear() = transform.orientation.toRotationMatrix();
  }

  if (auto* cache
      = registry.try_get<dart::simulation::experimental::comps::FrameCache>(
          entity)) {
    cache->needTransformUpdate = true;
  }
}

} // namespace

namespace dart::simulation::experimental {

//==============================================================================
RigidBody::RigidBody(entt::entity entity, World* world) : Frame(entity, world)
{
  // Frame base constructor handles entity and world
}

//==============================================================================
std::string RigidBody::getName() const
{
  const auto& registry = getWorld()->getRegistry();
  if (const auto* name = registry.try_get<comps::Name>(getEntity())) {
    return name->name;
  }
  return "";
}

//==============================================================================
const Eigen::Isometry3d& RigidBody::getLocalTransform() const
{
  const auto& props
      = getWorld()->getRegistry().get<comps::FreeFrameProperties>(getEntity());
  return props.localTransform;
}

//==============================================================================
double RigidBody::getMass() const
{
  const auto& massProps
      = getWorld()->getRegistry().get<comps::MassProperties>(getEntity());
  return massProps.mass;
}

//==============================================================================
void RigidBody::setMass(double mass)
{
  auto& massProps
      = getWorld()->getRegistry().get<comps::MassProperties>(getEntity());
  massProps.mass = mass;
}

//==============================================================================
Eigen::Matrix3d RigidBody::getInertia() const
{
  const auto& massProps
      = getWorld()->getRegistry().get<comps::MassProperties>(getEntity());
  return massProps.inertia;
}

//==============================================================================
void RigidBody::setInertia(const Eigen::Matrix3d& inertia)
{
  auto& massProps
      = getWorld()->getRegistry().get<comps::MassProperties>(getEntity());
  massProps.inertia = inertia;
}

//==============================================================================
Eigen::Vector3d RigidBody::getPosition() const
{
  const auto& transform
      = getWorld()->getRegistry().get<comps::Transform>(getEntity());
  return transform.position;
}

//==============================================================================
void RigidBody::setPosition(const Eigen::Vector3d& position)
{
  auto& registry = getWorld()->getRegistry();
  auto& transform = registry.get<comps::Transform>(getEntity());
  transform.position = position;
  syncFrameFromTransform(registry, getEntity());
}

//==============================================================================
Eigen::Quaterniond RigidBody::getOrientation() const
{
  const auto& transform
      = getWorld()->getRegistry().get<comps::Transform>(getEntity());
  return transform.orientation;
}

//==============================================================================
void RigidBody::setOrientation(const Eigen::Quaterniond& orientation)
{
  auto& registry = getWorld()->getRegistry();
  auto& transform = registry.get<comps::Transform>(getEntity());
  transform.orientation = orientation;
  syncFrameFromTransform(registry, getEntity());
}

//==============================================================================
Eigen::Vector3d RigidBody::getLinearVelocity() const
{
  const auto& velocity
      = getWorld()->getRegistry().get<comps::Velocity>(getEntity());
  return velocity.linear;
}

//==============================================================================
void RigidBody::setLinearVelocity(const Eigen::Vector3d& velocity)
{
  auto& vel = getWorld()->getRegistry().get<comps::Velocity>(getEntity());
  vel.linear = velocity;
}

//==============================================================================
Eigen::Vector3d RigidBody::getAngularVelocity() const
{
  const auto& velocity
      = getWorld()->getRegistry().get<comps::Velocity>(getEntity());
  return velocity.angular;
}

//==============================================================================
void RigidBody::setAngularVelocity(const Eigen::Vector3d& velocity)
{
  auto& vel = getWorld()->getRegistry().get<comps::Velocity>(getEntity());
  vel.angular = velocity;
}

//==============================================================================
Eigen::Vector3d RigidBody::getForce() const
{
  const auto& force = getWorld()->getRegistry().get<comps::Force>(getEntity());
  return force.force;
}

//==============================================================================
void RigidBody::addForce(const Eigen::Vector3d& force)
{
  auto& f = getWorld()->getRegistry().get<comps::Force>(getEntity());
  f.force += force;
}

//==============================================================================
Eigen::Vector3d RigidBody::getTorque() const
{
  const auto& force = getWorld()->getRegistry().get<comps::Force>(getEntity());
  return force.torque;
}

//==============================================================================
void RigidBody::addTorque(const Eigen::Vector3d& torque)
{
  auto& f = getWorld()->getRegistry().get<comps::Force>(getEntity());
  f.torque += torque;
}

//==============================================================================
void RigidBody::clearForces()
{
  auto& force = getWorld()->getRegistry().get<comps::Force>(getEntity());
  force.force.setZero();
  force.torque.setZero();
}

//==============================================================================
ShapeNode RigidBody::createShapeNode(
    const dart::dynamics::ShapePtr& shape,
    std::string_view name,
    const ShapeNodeOptions& options)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !shape, InvalidArgumentException, "ShapeNode requires a valid shape");
  return getWorld()->createShapeNode(getEntity(), shape, name, options);
}

} // namespace dart::simulation::experimental
