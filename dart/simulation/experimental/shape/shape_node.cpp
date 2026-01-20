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

#include "dart/simulation/experimental/shape/shape_node.hpp"

#include "dart/simulation/experimental/comps/name.hpp"
#include "dart/simulation/experimental/comps/shape_node.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <dart/dynamics/ShapeNode.hpp>

namespace dart::simulation::experimental {

//==============================================================================
ShapeNode::ShapeNode(entt::entity entity, World* world)
  : m_entity(entity), m_world(world)
{
}

//==============================================================================
std::string_view ShapeNode::getName() const
{
  const auto& registry = m_world->getRegistry();
  if (const auto* name = registry.try_get<comps::Name>(m_entity)) {
    return name->name;
  }
  return {};
}

//==============================================================================
entt::entity ShapeNode::getEntity() const
{
  return m_entity;
}

//==============================================================================
bool ShapeNode::isValid() const
{
  return m_world && m_world->getRegistry().valid(m_entity);
}

//==============================================================================
Frame ShapeNode::getParentFrame() const
{
  const auto& registry = m_world->getRegistry();
  const auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  return Frame(shapeComp.parentEntity, m_world);
}

//==============================================================================
Eigen::Isometry3d ShapeNode::getRelativeTransform() const
{
  const auto& registry = m_world->getRegistry();
  const auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  return shapeComp.relativeTransform;
}

//==============================================================================
void ShapeNode::setRelativeTransform(const Eigen::Isometry3d& transform)
{
  auto& registry = m_world->getRegistry();
  auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  shapeComp.relativeTransform = transform;
  if (shapeComp.classicShapeNode) {
    shapeComp.classicShapeNode->setRelativeTransform(transform);
  }
}

//==============================================================================
Eigen::Isometry3d ShapeNode::getWorldTransform() const
{
  const auto parent = getParentFrame();
  return parent.getTransform() * getRelativeTransform();
}

//==============================================================================
dart::dynamics::ConstShapePtr ShapeNode::getShape() const
{
  const auto& registry = m_world->getRegistry();
  const auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  return shapeComp.shape;
}

//==============================================================================
void ShapeNode::setShape(const dart::dynamics::ShapePtr& shape)
{
  auto& registry = m_world->getRegistry();
  auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  shapeComp.shape = shape;
  if (shapeComp.classicShapeNode) {
    shapeComp.classicShapeNode->setShape(shape);
  }
}

//==============================================================================
void ShapeNode::setCollidable(bool collidable)
{
  auto& registry = m_world->getRegistry();
  auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  shapeComp.collidable = collidable;
  if (shapeComp.classicShapeNode) {
    if (auto* collisionAspect
        = shapeComp.classicShapeNode->getCollisionAspect()) {
      collisionAspect->setCollidable(collidable);
    }
  }
}

//==============================================================================
bool ShapeNode::isCollidable() const
{
  const auto& registry = m_world->getRegistry();
  const auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  return shapeComp.collidable;
}

//==============================================================================
void ShapeNode::setFrictionCoeff(double friction)
{
  auto& registry = m_world->getRegistry();
  auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  shapeComp.frictionCoeff = friction;
  if (shapeComp.classicShapeNode) {
    if (auto* dynamicsAspect
        = shapeComp.classicShapeNode->getDynamicsAspect()) {
      dynamicsAspect->setFrictionCoeff(friction);
    }
  }
}

//==============================================================================
double ShapeNode::getFrictionCoeff() const
{
  const auto& registry = m_world->getRegistry();
  const auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  return shapeComp.frictionCoeff;
}

//==============================================================================
void ShapeNode::setRestitutionCoeff(double restitution)
{
  auto& registry = m_world->getRegistry();
  auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  shapeComp.restitutionCoeff = restitution;
  if (shapeComp.classicShapeNode) {
    if (auto* dynamicsAspect
        = shapeComp.classicShapeNode->getDynamicsAspect()) {
      dynamicsAspect->setRestitutionCoeff(restitution);
    }
  }
}

//==============================================================================
double ShapeNode::getRestitutionCoeff() const
{
  const auto& registry = m_world->getRegistry();
  const auto& shapeComp = registry.get<comps::ShapeNode>(m_entity);
  return shapeComp.restitutionCoeff;
}

} // namespace dart::simulation::experimental
