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

#include "dart/simulation/experimental/multi_body/link.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/kinematics/joint_transform.hpp"
#include "dart/simulation/experimental/multi_body/joint.hpp"
#include "dart/simulation/experimental/world.hpp"

namespace dart::simulation::experimental {

//==============================================================================
Link::Link(entt::entity entity, World* world) : Frame(entity, world) {}

//==============================================================================
std::string_view Link::getName() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return linkComp.name;
}

//==============================================================================
Joint Link::getParentJoint() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return Joint(linkComp.parentJoint, getWorld());
}

//==============================================================================
const Eigen::Isometry3d& Link::getLocalTransform() const
{
  auto& registry = getWorld()->getRegistry();
  auto& linkComp = registry.get<comps::Link>(getEntity());

  if (linkComp.needLocalTransformUpdate) {
    if (linkComp.parentJoint != entt::null) {
      const auto& jointComp = registry.get<comps::Joint>(linkComp.parentJoint);
      Eigen::Isometry3d jointTransform
          = kinematics::computeJointTransform(jointComp);
      linkComp.localTransformCache
          = jointTransform * linkComp.transformFromParentJoint;
    } else {
      linkComp.localTransformCache = linkComp.transformFromParentJoint;
    }
    linkComp.needLocalTransformUpdate = false;
  }

  return linkComp.localTransformCache;
}

//==============================================================================
const Eigen::Isometry3d& Link::getWorldTransform() const
{
  return Frame::getTransform();
}

//==============================================================================
double Link::getMass() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return linkComp.mass.mass;
}

//==============================================================================
void Link::setMass(double mass)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.mass.mass = mass;
}

//==============================================================================
const Eigen::Vector3d& Link::getLocalCOM() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return linkComp.mass.localCOM;
}

//==============================================================================
void Link::setLocalCOM(const Eigen::Vector3d& com)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.mass.localCOM = com;
}

//==============================================================================
const Eigen::Matrix3d& Link::getInertia() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return linkComp.mass.inertia;
}

//==============================================================================
void Link::setInertia(const Eigen::Matrix3d& inertia)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.mass.inertia = inertia;
}

//==============================================================================
Eigen::Vector3d Link::getExternalForce() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return linkComp.externalForce;
}

//==============================================================================
void Link::addExternalForce(const Eigen::Vector3d& force)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.externalForce += force;
}

//==============================================================================
void Link::setExternalForce(const Eigen::Vector3d& force)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.externalForce = force;
}

//==============================================================================
Eigen::Vector3d Link::getExternalTorque() const
{
  const auto& linkComp
      = getWorld()->getRegistry().get<comps::Link>(getEntity());
  return linkComp.externalTorque;
}

//==============================================================================
void Link::addExternalTorque(const Eigen::Vector3d& torque)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.externalTorque += torque;
}

//==============================================================================
void Link::setExternalTorque(const Eigen::Vector3d& torque)
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.externalTorque = torque;
}

//==============================================================================
void Link::clearExternalForces()
{
  auto& linkComp = getWorld()->getRegistry().get<comps::Link>(getEntity());
  linkComp.externalForce.setZero();
  linkComp.externalTorque.setZero();
}

//==============================================================================
ShapeNode Link::createShapeNode(
    const dart::dynamics::ShapePtr& shape,
    std::string_view name,
    const ShapeNodeOptions& options)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !shape, InvalidArgumentException, "ShapeNode requires a valid shape");
  return getWorld()->createShapeNode(getEntity(), shape, name, options);
}

} // namespace dart::simulation::experimental
