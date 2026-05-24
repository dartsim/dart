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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/multibody/joint_type.hpp>
#include <dart/simulation/experimental/multibody/link.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dartsim_engine/object_manager.hpp>

#include <algorithm>
#include <string>
#include <unordered_set>

#include <cmath>

namespace dartsim {

namespace sx = dart::simulation::experimental;

namespace {

constexpr double kMinimumTimeStep = 1e-9;

sx::JointType toJointType(JointKind kind)
{
  switch (kind) {
    case JointKind::Fixed:
      return sx::JointType::Fixed;
    case JointKind::Revolute:
      return sx::JointType::Revolute;
    case JointKind::Prismatic:
      return sx::JointType::Prismatic;
    case JointKind::Screw:
      return sx::JointType::Screw;
    case JointKind::Universal:
      return sx::JointType::Universal;
    case JointKind::Ball:
      return sx::JointType::Spherical;
    case JointKind::Planar:
      return sx::JointType::Planar;
    case JointKind::Free:
      return sx::JointType::Floating;
  }
  return sx::JointType::Revolute;
}

double sanitizeTimeStep(double timeStep)
{
  return std::isfinite(timeStep) && timeStep > 0.0 ? timeStep
                                                   : kMinimumTimeStep;
}

void collectMultiBodyLinks(
    const SceneModel& model,
    ObjectId parent,
    ObjectId multiBody,
    std::vector<const SceneObject*>& links)
{
  for (const ObjectId childId : model.childrenOf(parent)) {
    const SceneObject* child = model.find(childId);
    if (child == nullptr) {
      continue;
    }
    if (child->type == ObjectType::Link && child->multiBody == multiBody) {
      links.push_back(child);
    }
    collectMultiBodyLinks(model, childId, multiBody, links);
  }
}

} // namespace

ObjectManager::ObjectManager() : m_world(std::make_unique<sx::World>()) {}

ObjectManager::~ObjectManager() = default;

void ObjectManager::setModel(SceneModel model)
{
  m_model = std::move(model);
  rebuild();
}

void ObjectManager::rebuild()
{
  // A fresh World guarantees design mode regardless of prior simulation state.
  m_world = std::make_unique<sx::World>();
  m_model.timeStep = sanitizeTimeStep(m_model.timeStep);
  m_world->setTimeStep(m_model.timeStep);

  for (const ObjectId id : m_model.rootChildren()) {
    const SceneObject* object = m_model.find(id);
    if (object == nullptr) {
      continue;
    }
    switch (object->type) {
      case ObjectType::RigidBody: {
        sx::RigidBodyOptions opts;
        opts.mass = std::max(object->mass, 1e-9);
        opts.inertia = object->inertia;
        opts.position = object->transform.translation();
        opts.orientation = Eigen::Quaterniond(object->transform.rotation());
        opts.linearVelocity = object->linearVelocity;
        opts.angularVelocity = object->angularVelocity;
        m_world->addRigidBody(object->name, opts);
        break;
      }
      case ObjectType::MultiBody:
        buildMultiBody(*object);
        break;
      case ObjectType::FreeFrame: {
        sx::FreeFrame frame = m_world->addFreeFrame(object->name);
        frame.setLocalTransform(object->transform);
        break;
      }
      case ObjectType::FixedFrame:
        m_world->addFixedFrame(
            object->name, sx::Frame::world(), object->transform);
        break;
      case ObjectType::Link:
      case ObjectType::Joint:
        // Links/joints are created as part of their owning MultiBody.
        break;
    }
  }

  // Note: World::sync()/updateKinematics() require simulation mode, so design
  // (edit) mode relies on direct rigid-body poses and lazy frame evaluation.
  // Articulated link world transforms refresh once stepping begins.
}

void ObjectManager::buildMultiBody(const SceneObject& multiBodyObject)
{
  sx::Multibody mb = m_world->addMultibody(multiBodyObject.name);

  // Collect link children in model order.
  std::vector<const SceneObject*> links;
  collectMultiBodyLinks(m_model, multiBodyObject.id, multiBodyObject.id, links);

  // Add links parent-before-child (kinematic parent given by parentLink).
  std::unordered_set<ObjectId> added;
  bool progressed = true;
  while (added.size() < links.size() && progressed) {
    progressed = false;
    for (const SceneObject* link : links) {
      if (added.count(link->id) != 0) {
        continue;
      }
      if (link->parentLink == kNoObject) {
        mb.addLink(link->name);
        added.insert(link->id);
        progressed = true;
      } else if (added.count(link->parentLink) != 0) {
        const SceneObject* parent = m_model.find(link->parentLink);
        if (parent == nullptr) {
          added.insert(link->id); // give up on this malformed link
          progressed = true;
          continue;
        }
        std::optional<sx::Link> parentHandle = mb.getLink(parent->name);
        if (!parentHandle.has_value()) {
          added.insert(link->id);
          progressed = true;
          continue;
        }
        sx::JointSpec spec;
        spec.name = link->name + "_joint";
        spec.type = toJointType(link->jointType);
        spec.axis = link->jointAxis;
        mb.addLink(link->name, *parentHandle, spec);
        added.insert(link->id);
        progressed = true;
      }
    }
  }

  // Apply single-DOF joint positions.
  for (const SceneObject* link : links) {
    if (link->parentLink == kNoObject) {
      continue;
    }
    std::optional<sx::Link> handle = mb.getLink(link->name);
    if (!handle.has_value()) {
      continue;
    }
    sx::Joint joint = handle->getParentJoint();
    if (joint.isValid() && joint.getDOFCount() == 1) {
      Eigen::VectorXd q(1);
      q[0] = link->jointPosition;
      joint.setPosition(q);
    }
  }
}

std::optional<Eigen::Isometry3d> ObjectManager::worldTransformOf(
    ObjectId id) const
{
  const SceneObject* object = m_model.find(id);
  if (object == nullptr) {
    return std::nullopt;
  }

  if (object->type == ObjectType::RigidBody) {
    std::optional<sx::RigidBody> rb = m_world->getRigidBody(object->name);
    if (rb.has_value()) {
      return rb->getTransform();
    }
  } else if (object->type == ObjectType::Link) {
    const SceneObject* mbObject = m_model.find(object->multiBody);
    if (mbObject != nullptr) {
      std::optional<sx::Multibody> mb = m_world->getMultibody(mbObject->name);
      if (mb.has_value()) {
        std::optional<sx::Link> link = mb->getLink(object->name);
        if (link.has_value()) {
          return link->getWorldTransform();
        }
      }
    }
  }
  return std::nullopt;
}

std::vector<RenderItem> ObjectManager::computeRenderItems() const
{
  std::vector<RenderItem> items;

  // Depth-first over the whole tree so links nested under a multibody render.
  std::vector<ObjectId> stack(
      m_model.rootChildren().rbegin(), m_model.rootChildren().rend());
  while (!stack.empty()) {
    const ObjectId id = stack.back();
    stack.pop_back();
    const SceneObject* object = m_model.find(id);
    if (object == nullptr) {
      continue;
    }
    const auto& children = m_model.childrenOf(id);
    for (auto it = children.rbegin(); it != children.rend(); ++it) {
      stack.push_back(*it);
    }

    const bool hasShape = object->type == ObjectType::RigidBody
                          || object->type == ObjectType::Link;
    if (!hasShape || !object->visible) {
      continue;
    }
    std::optional<Eigen::Isometry3d> transform = worldTransformOf(id);
    if (!transform.has_value()) {
      continue;
    }
    RenderItem item;
    item.id = id;
    item.shape = object->shape.type;
    item.dimensions = object->shape.dimensions;
    item.color = object->shape.color;
    item.worldTransform = *transform;
    items.push_back(item);
  }
  return items;
}

} // namespace dartsim
