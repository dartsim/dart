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

#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/name_manager.hpp>
#include <dartsim_engine/object_manager.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/selection_manager.hpp>

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>

namespace dartsim::commands {

namespace {

constexpr double kMinimumMass = 1e-9;
constexpr double kMinimumTimeStep = 1e-9;

/// Collect `id` and all of its descendants in pre-order.
void collectSubtree(
    const SceneModel& model, ObjectId id, std::vector<ObjectId>& out)
{
  if (!model.contains(id)) {
    return;
  }
  out.push_back(id);
  for (const ObjectId child : model.childrenOf(id)) {
    collectSubtree(model, child, out);
  }
}

bool isLinkNameAvailableInMultiBody(
    const SceneModel& model,
    ObjectId multiBody,
    std::string_view name,
    ObjectId except = kNoObject)
{
  for (const ObjectId id : model.allIds()) {
    if (id == except) {
      continue;
    }
    const SceneObject* object = model.find(id);
    if (object != nullptr && object->type == ObjectType::Link
        && object->multiBody == multiBody && object->name == name) {
      return false;
    }
  }
  return true;
}

std::string makeUniqueLinkName(
    const SceneModel& model, ObjectId multiBody, std::string_view base)
{
  const std::string root = base.empty()
                               ? NameManager::defaultBaseName(ObjectType::Link)
                               : std::string(base);
  if (isLinkNameAvailableInMultiBody(model, multiBody, root)) {
    return root;
  }
  for (int suffix = 1;; ++suffix) {
    std::string candidate = root + " " + std::to_string(suffix);
    if (isLinkNameAvailableInMultiBody(model, multiBody, candidate)) {
      return candidate;
    }
  }
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

} // namespace

std::unique_ptr<Command> addRigidBody(
    ShapeType shape, const Eigen::Isometry3d& transform, std::string name)
{
  return std::make_unique<Command>(
      "Add Rigid Body",
      [shape, transform, name = std::move(name)](
          ObjectManager& objects, SelectionManager& selection) {
        SceneModel& model = objects.model();
        SceneObject object;
        object.type = ObjectType::RigidBody;
        object.parent = kNoObject;
        object.transform = transform;
        object.shape.type = shape;
        object.name = NameManager::makeUnique(
            model,
            kNoObject,
            name.empty() ? NameManager::defaultBaseName(ObjectType::RigidBody)
                         : name);
        const ObjectId id = model.add(std::move(object));
        objects.rebuild();
        selection.select(id);
      });
}

std::unique_ptr<Command> addMultiBody(std::string name)
{
  return std::make_unique<Command>(
      "Add MultiBody",
      [name = std::move(name)](
          ObjectManager& objects, SelectionManager& selection) {
        SceneModel& model = objects.model();
        SceneObject object;
        object.type = ObjectType::MultiBody;
        object.parent = kNoObject;
        object.name = NameManager::makeUnique(
            model,
            kNoObject,
            name.empty() ? NameManager::defaultBaseName(ObjectType::MultiBody)
                         : name);
        const ObjectId id = model.add(std::move(object));
        objects.rebuild();
        selection.select(id);
      });
}

std::unique_ptr<Command> addLink(
    ObjectId multiBody, ObjectId parentLink, JointKind joint, std::string name)
{
  return std::make_unique<Command>(
      "Add Link",
      [multiBody, parentLink, joint, name = std::move(name)](
          ObjectManager& objects, SelectionManager& selection) {
        SceneModel& model = objects.model();
        const SceneObject* mb = model.find(multiBody);
        if (mb == nullptr || mb->type != ObjectType::MultiBody) {
          return;
        }
        // A non-root parent link must be an existing Link in this multibody;
        // otherwise rebuild() cannot attach the child and silently drops it.
        if (parentLink != kNoObject) {
          const SceneObject* parent = model.find(parentLink);
          if (parent == nullptr || parent->type != ObjectType::Link
              || parent->multiBody != multiBody) {
            return;
          }
        }
        SceneObject object;
        object.type = ObjectType::Link;
        object.parent = parentLink == kNoObject ? multiBody : parentLink;
        object.multiBody = multiBody;
        object.parentLink = parentLink;
        object.jointType = joint;
        object.name = makeUniqueLinkName(model, multiBody, name);
        const ObjectId id = model.add(std::move(object));
        objects.rebuild();
        selection.select(id);
      });
}

std::unique_ptr<Command> addFreeFrame(
    const Eigen::Isometry3d& transform, std::string name)
{
  return std::make_unique<Command>(
      "Add Free Frame",
      [transform, name = std::move(name)](
          ObjectManager& objects, SelectionManager& selection) {
        SceneModel& model = objects.model();
        SceneObject object;
        object.type = ObjectType::FreeFrame;
        object.parent = kNoObject;
        object.transform = transform;
        object.name = NameManager::makeUnique(
            model,
            kNoObject,
            name.empty() ? NameManager::defaultBaseName(ObjectType::FreeFrame)
                         : name);
        const ObjectId id = model.add(std::move(object));
        objects.rebuild();
        selection.select(id);
      });
}

std::unique_ptr<Command> addFixedFrame(
    const Eigen::Isometry3d& transform, std::string name)
{
  return std::make_unique<Command>(
      "Add Fixed Frame",
      [transform, name = std::move(name)](
          ObjectManager& objects, SelectionManager& selection) {
        SceneModel& model = objects.model();
        SceneObject object;
        object.type = ObjectType::FixedFrame;
        object.parent = kNoObject;
        object.transform = transform;
        object.name = NameManager::makeUnique(
            model,
            kNoObject,
            name.empty() ? NameManager::defaultBaseName(ObjectType::FixedFrame)
                         : name);
        const ObjectId id = model.add(std::move(object));
        objects.rebuild();
        selection.select(id);
      });
}

std::unique_ptr<Command> removeObject(ObjectId id)
{
  return std::make_unique<Command>(
      "Remove", [id](ObjectManager& objects, SelectionManager& selection) {
        SceneModel& model = objects.model();
        // remove() also deletes descendants, so deselect the whole subtree;
        // otherwise a selected descendant leaves a dangling id in the
        // selection.
        std::vector<ObjectId> removed;
        collectSubtree(model, id, removed);
        if (removed.empty()) {
          return;
        }
        model.remove(id);
        objects.rebuild();
        for (const ObjectId removedId : removed) {
          selection.deselect(removedId);
        }
      });
}

std::unique_ptr<Command> setTransform(
    ObjectId id, const Eigen::Isometry3d& transform)
{
  return std::make_unique<Command>(
      "Set Transform",
      [id, transform](ObjectManager& objects, SelectionManager&) {
        if (SceneObject* object = objects.model().find(id)) {
          object->transform = transform;
          objects.rebuild();
        }
      });
}

std::unique_ptr<Command> setMass(ObjectId id, double mass)
{
  return std::make_unique<Command>(
      "Set Mass", [id, mass](ObjectManager& objects, SelectionManager&) {
        if (SceneObject* object = objects.model().find(id)) {
          object->mass = sanitizeMass(mass);
          objects.rebuild();
        }
      });
}

std::unique_ptr<Command> setJointPosition(ObjectId link, double position)
{
  return std::make_unique<Command>(
      "Set Joint Position",
      [link, position](ObjectManager& objects, SelectionManager&) {
        if (SceneObject* object = objects.model().find(link)) {
          object->jointPosition = position;
          objects.rebuild();
        }
      });
}

std::unique_ptr<Command> rename(ObjectId id, std::string name)
{
  return std::make_unique<Command>(
      "Rename",
      [id, name = std::move(name)](ObjectManager& objects, SelectionManager&) {
        SceneModel& model = objects.model();
        SceneObject* object = model.find(id);
        if (object == nullptr || name.empty()) {
          return;
        }
        if (object->type == ObjectType::Link) {
          if (!isLinkNameAvailableInMultiBody(
                  model, object->multiBody, name, id)) {
            return;
          }
          object->name = name;
          objects.rebuild();
          return;
        }
        if (!model.isNameAvailable(object->parent, name, id)) {
          return;
        }
        object->name = name;
        objects.rebuild();
      });
}

std::unique_ptr<Command> reparent(ObjectId id, ObjectId newParent)
{
  return std::make_unique<Command>(
      "Reparent", [id, newParent](ObjectManager& objects, SelectionManager&) {
        SceneModel& model = objects.model();
        const SceneObject* object = model.find(id);
        if (object == nullptr) {
          return;
        }
        // Links/joints are materialized only from their owning multibody's
        // direct child list, so moving one elsewhere in the tree would silently
        // drop it from the rebuilt world. They cannot be reparented.
        if (object->type == ObjectType::Link
            || object->type == ObjectType::Joint) {
          return;
        }
        // rebuild() instantiates only root children (links come from their
        // multibody), so nesting a rigid body/frame under another object would
        // drop it from the rebuilt world. Until hierarchical rebuild exists,
        // only moves to the world root are representable.
        if (newParent != kNoObject) {
          return;
        }
        if (model.reparent(id, newParent)) {
          objects.rebuild();
        }
      });
}

std::unique_ptr<Command> setTimeStep(double timeStep)
{
  return std::make_unique<Command>(
      "Set Time Step", [timeStep](ObjectManager& objects, SelectionManager&) {
        objects.model().timeStep = sanitizeTimeStep(timeStep);
        objects.rebuild();
      });
}

} // namespace dartsim::commands
