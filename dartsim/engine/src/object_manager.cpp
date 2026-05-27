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
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/multibody/joint_type.hpp>
#include <dart/simulation/experimental/multibody/link.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dartsim_engine/object_manager.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <cmath>

namespace dartsim {

namespace sx = dart::simulation::experimental;

struct ObjectManager::RebuildFrameIndex
{
  std::unordered_map<ObjectId, sx::Frame> frames;
  std::vector<ObjectId> buildQueue;

  void add(ObjectId id, const sx::Frame& frame)
  {
    frames.emplace(id, frame);
    buildQueue.push_back(id);
  }
};

namespace {

constexpr double kMinimumMass = 1e-9;
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

double sanitizeMass(double mass)
{
  return std::isfinite(mass) && mass > 0.0 ? mass : kMinimumMass;
}

Eigen::Vector3d sanitizeJointAxis(Eigen::Vector3d axis)
{
  if (!axis.allFinite() || axis.isZero()) {
    return Eigen::Vector3d::UnitZ();
  }
  return axis.normalized();
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

void ObjectManager::restoreModelSnapshot(SceneModel model)
{
  if (m_model.hasSameSceneContents(model)) {
    m_model = std::move(model);
    return;
  }

  setModel(std::move(model));
}

void ObjectManager::rebuild()
{
  // A fresh World guarantees design mode regardless of prior simulation state.
  m_world = std::make_unique<sx::World>();
  m_model.timeStep = sanitizeTimeStep(m_model.timeStep);
  m_world->setTimeStep(m_model.timeStep);

  RebuildFrameIndex frames;
  for (const ObjectId id : m_model.rootChildren()) {
    SceneObject* object = m_model.find(id);
    if (object == nullptr) {
      continue;
    }
    switch (object->type) {
      case ObjectType::RigidBody: {
        sx::RigidBodyOptions opts;
        object->mass = sanitizeMass(object->mass);
        opts.mass = object->mass;
        opts.inertia = object->inertia;
        opts.position = object->transform.translation();
        opts.orientation = Eigen::Quaterniond(object->transform.rotation());
        opts.linearVelocity = object->linearVelocity;
        opts.angularVelocity = object->angularVelocity;
        sx::RigidBody body = m_world->addRigidBody(object->name, opts);
        frames.add(object->id, body);
        break;
      }
      case ObjectType::MultiBody:
        buildMultiBody(*object, frames);
        break;
      case ObjectType::FreeFrame: {
        sx::FreeFrame frame = m_world->addFreeFrame(object->name);
        frame.setLocalTransform(object->transform);
        frames.add(object->id, frame);
        break;
      }
      case ObjectType::FixedFrame:
        // Fixed frames need a non-world parent frame in the experimental API.
        // Root-level fixed-frame records are ignored instead of crashing on
        // malformed or older project files.
        break;
      case ObjectType::Link:
      case ObjectType::Joint:
        // Links/joints are created as part of their owning MultiBody.
        break;
    }
  }

  for (std::size_t index = 0; index < frames.buildQueue.size(); ++index) {
    buildAttachedFrames(frames.buildQueue[index], frames);
  }

  // Note: World::sync()/updateKinematics() require simulation mode, so design
  // (edit) mode relies on direct rigid-body poses and lazy frame evaluation.
  // Articulated link world transforms refresh once stepping begins.
}

void ObjectManager::buildMultiBody(
    const SceneObject& multiBodyObject, RebuildFrameIndex& frames)
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
        sx::Link root = mb.addLink(link->name);
        frames.add(link->id, root);
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
        spec.axis = sanitizeJointAxis(link->jointAxis);
        sx::Link child = mb.addLink(link->name, *parentHandle, spec);
        frames.add(link->id, child);
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

void ObjectManager::buildAttachedFrames(
    ObjectId parent, RebuildFrameIndex& frames)
{
  const auto parentIt = frames.frames.find(parent);
  if (parentIt == frames.frames.end()) {
    return;
  }

  for (const ObjectId childId : m_model.childrenOf(parent)) {
    SceneObject* child = m_model.find(childId);
    if (child == nullptr) {
      continue;
    }

    switch (child->type) {
      case ObjectType::FreeFrame: {
        sx::FreeFrame frame
            = m_world->addFreeFrame(child->name, parentIt->second);
        frame.setLocalTransform(child->transform);
        frames.add(child->id, frame);
        break;
      }
      case ObjectType::FixedFrame: {
        if (child->name.empty()) {
          break;
        }
        sx::FixedFrame frame = m_world->addFixedFrame(
            child->name, parentIt->second, child->transform);
        frames.add(child->id, frame);
        break;
      }
      default:
        break;
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
  } else if (
      object->type == ObjectType::FreeFrame
      || object->type == ObjectType::FixedFrame) {
    if (object->parent == kNoObject) {
      return object->transform;
    }
    const std::optional<Eigen::Isometry3d> parentTransform
        = worldTransformOf(object->parent);
    if (parentTransform.has_value()) {
      return *parentTransform * object->transform;
    }
  }
  return std::nullopt;
}

std::vector<RenderItem> ObjectManager::computeRenderItems() const
{
  std::vector<RenderItem> items;

  struct StackEntry
  {
    ObjectId id = kNoObject;
    bool ancestorsVisible = true;
  };

  // Depth-first over the whole tree so links nested under a multibody render.
  std::vector<StackEntry> stack;
  stack.reserve(m_model.size());
  for (auto it = m_model.rootChildren().rbegin();
       it != m_model.rootChildren().rend();
       ++it) {
    stack.push_back(StackEntry{*it, true});
  }
  while (!stack.empty()) {
    const StackEntry entry = stack.back();
    stack.pop_back();
    const SceneObject* object = m_model.find(entry.id);
    if (object == nullptr) {
      continue;
    }
    const bool effectivelyVisible = entry.ancestorsVisible && object->visible;
    const auto& children = m_model.childrenOf(entry.id);
    for (auto it = children.rbegin(); it != children.rend(); ++it) {
      stack.push_back(StackEntry{*it, effectivelyVisible});
    }

    const bool hasShape = object->type == ObjectType::RigidBody
                          || object->type == ObjectType::Link;
    if (!hasShape || !effectivelyVisible) {
      continue;
    }
    std::optional<Eigen::Isometry3d> transform = worldTransformOf(entry.id);
    if (!transform.has_value()) {
      continue;
    }
    RenderItem item;
    item.id = entry.id;
    item.shape = object->shape.type;
    item.dimensions = object->shape.dimensions;
    item.color = object->shape.color;
    item.worldTransform = *transform;
    items.push_back(item);
  }
  return items;
}

} // namespace dartsim
