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
#include <dartsim_ui/relationship_actions.hpp>

#include <utility>

namespace dartsim::ui {

namespace {

RelationshipAction action(
    RelationshipActionKind kind,
    std::string label,
    bool enabled,
    std::string disabledReason = {})
{
  return {kind, std::move(label), enabled, std::move(disabledReason)};
}

RelationshipActionResult result(
    bool ok, std::string message, ObjectId object = kNoObject)
{
  return {ok, std::move(message), object};
}

bool isFrameLike(ObjectType type)
{
  return type == ObjectType::RigidBody || type == ObjectType::Link
         || type == ObjectType::FreeFrame || type == ObjectType::FixedFrame;
}

bool isAttachableFrame(ObjectType type)
{
  return type == ObjectType::FreeFrame || type == ObjectType::FixedFrame;
}

bool isLink(const SceneObject* object)
{
  return object != nullptr && object->type == ObjectType::Link;
}

const SceneObject* findPrimary(const SimEngine& engine)
{
  return engine.objects().model().find(engine.selection().primary());
}

ObjectId selectedAttachChild(const SimEngine& engine)
{
  const ObjectId parent = engine.selection().primary();
  ObjectId child = kNoObject;
  for (const ObjectId id : engine.selection().selected()) {
    if (id == parent) {
      continue;
    }
    if (child != kNoObject) {
      return kNoObject;
    }
    child = id;
  }
  return child;
}

ObjectId selectedRelationshipChild(const SimEngine& engine)
{
  const ObjectId primary = engine.selection().primary();
  ObjectId child = kNoObject;
  for (const ObjectId id : engine.selection().selected()) {
    if (id == primary) {
      continue;
    }
    if (child != kNoObject) {
      return kNoObject;
    }
    child = id;
  }
  return child;
}

bool linkParentWouldCycle(
    const SceneModel& model, ObjectId link, ObjectId parentLink)
{
  ObjectId cursor = parentLink;
  while (cursor != kNoObject) {
    if (cursor == link) {
      return true;
    }
    const SceneObject* parent = model.find(cursor);
    if (parent == nullptr || parent->type != ObjectType::Link) {
      return false;
    }
    cursor = parent->parentLink;
  }
  return false;
}

bool canAttachSelectedToPrimary(const SimEngine& engine)
{
  const SceneObject* parent = findPrimary(engine);
  const SceneObject* child
      = engine.objects().model().find(selectedAttachChild(engine));
  return parent != nullptr && child != nullptr && isFrameLike(parent->type)
         && isAttachableFrame(child->type);
}

bool canDetachPrimary(const SimEngine& engine)
{
  const SceneObject* object = findPrimary(engine);
  return object != nullptr && isAttachableFrame(object->type)
         && (object->parent != kNoObject
             || object->type == ObjectType::FixedFrame);
}

bool canReparentSelectedLinkToPrimary(const SimEngine& engine)
{
  const SceneModel& model = engine.objects().model();
  const SceneObject* parent = findPrimary(engine);
  const SceneObject* child = model.find(selectedRelationshipChild(engine));
  return isLink(parent) && isLink(child)
         && parent->multiBody == child->multiBody && parent->id != child->id
         && child->parentLink != parent->id
         && !linkParentWouldCycle(model, child->id, parent->id);
}

bool canMakePrimaryLinkRoot(const SimEngine& engine)
{
  const SceneObject* object = findPrimary(engine);
  return isLink(object) && object->parentLink != kNoObject;
}

std::string namedAction(
    std::string verb, const SceneObject* child, const SceneObject* parent)
{
  if (child == nullptr || parent == nullptr) {
    return {};
  }
  return std::move(verb) + " " + child->name + " to " + parent->name;
}

} // namespace

std::vector<RelationshipAction> buildRelationshipActions(
    const SimEngine& engine)
{
  const bool canEdit = engine.canEditScene();
  const std::string locked = canEdit ? std::string() : "Simulation Mode";
  const SceneModel& model = engine.objects().model();
  const SceneObject* primary = findPrimary(engine);
  const SceneObject* attachChild = model.find(selectedAttachChild(engine));
  const SceneObject* relationshipChild
      = model.find(selectedRelationshipChild(engine));
  const bool attachEnabled = canEdit && canAttachSelectedToPrimary(engine);
  const bool detachEnabled = canEdit && canDetachPrimary(engine);
  const bool reparentEnabled
      = canEdit && canReparentSelectedLinkToPrimary(engine);
  const bool makeRootEnabled = canEdit && canMakePrimaryLinkRoot(engine);

  std::vector<RelationshipAction> actions;
  actions.push_back(action(
      RelationshipActionKind::AttachSelectedToPrimary,
      attachEnabled ? namedAction("Attach", attachChild, primary)
                    : "Attach Selected to Primary",
      attachEnabled,
      canEdit ? "Select one frame and one parent" : locked));
  actions.push_back(action(
      RelationshipActionKind::DetachPrimaryToWorld,
      detachEnabled && primary != nullptr
          ? "Detach " + primary->name + " to World"
          : "Detach Primary to World",
      detachEnabled,
      canEdit ? "Select an attached frame" : locked));
  actions.push_back(action(
      RelationshipActionKind::ReparentSelectedLinkToPrimary,
      reparentEnabled ? namedAction("Reparent", relationshipChild, primary)
                      : "Reparent Selected Link to Primary",
      reparentEnabled,
      canEdit ? "Select one child link and one parent link" : locked));
  actions.push_back(action(
      RelationshipActionKind::MakePrimaryLinkRoot,
      makeRootEnabled && primary != nullptr
          ? "Make " + primary->name + " Root Link"
          : "Make Primary Link Root",
      makeRootEnabled,
      canEdit ? "Select a child link" : locked));
  return actions;
}

RelationshipActionResult applyRelationshipAction(
    SimEngine& engine, RelationshipActionKind kind)
{
  if (!engine.canEditScene()) {
    return result(false, "Scene locked");
  }

  switch (kind) {
    case RelationshipActionKind::AttachSelectedToPrimary: {
      const ObjectId parentId = engine.selection().primary();
      const ObjectId childId = selectedAttachChild(engine);
      const SceneObject* parent = engine.objects().model().find(parentId);
      const SceneObject* child = engine.objects().model().find(childId);
      if (parent == nullptr || child == nullptr || !isFrameLike(parent->type)
          || !isAttachableFrame(child->type)) {
        return result(false, "Select one frame and one parent");
      }
      const std::string childName = child->name;
      const std::string parentName = parent->name;
      engine.execute(commands::attachFrame(childId, parentId));
      const SceneObject* updated = engine.objects().model().find(childId);
      if (updated == nullptr || updated->parent != parentId) {
        return result(false, "Attach rejected");
      }
      return result(
          true, "Attached " + childName + " to " + parentName, childId);
    }
    case RelationshipActionKind::DetachPrimaryToWorld: {
      const ObjectId objectId = engine.selection().primary();
      const SceneObject* object = engine.objects().model().find(objectId);
      if (object == nullptr || !isAttachableFrame(object->type)
          || (object->parent == kNoObject
              && object->type != ObjectType::FixedFrame)) {
        return result(false, "Select an attached frame");
      }
      const std::string name = object->name;
      engine.execute(commands::detachFrame(objectId));
      const SceneObject* updated = engine.objects().model().find(objectId);
      if (updated == nullptr || updated->parent != kNoObject
          || updated->type != ObjectType::FreeFrame) {
        return result(false, "Detach rejected");
      }
      return result(true, "Detached " + name, objectId);
    }
    case RelationshipActionKind::ReparentSelectedLinkToPrimary: {
      const ObjectId parentId = engine.selection().primary();
      const ObjectId childId = selectedRelationshipChild(engine);
      const SceneModel& model = engine.objects().model();
      const SceneObject* parent = model.find(parentId);
      const SceneObject* child = model.find(childId);
      if (!isLink(parent) || !isLink(child)
          || parent->multiBody != child->multiBody || parentId == childId
          || child->parentLink == parentId
          || linkParentWouldCycle(model, childId, parentId)) {
        return result(false, "Select one child link and one parent link");
      }
      const std::string childName = child->name;
      const std::string parentName = parent->name;
      engine.execute(commands::setLinkParent(childId, parentId));
      const SceneObject* updated = engine.objects().model().find(childId);
      if (updated == nullptr || updated->parentLink != parentId
          || updated->parent != parentId) {
        return result(false, "Link reparent rejected");
      }
      return result(
          true, "Reparented " + childName + " to " + parentName, childId);
    }
    case RelationshipActionKind::MakePrimaryLinkRoot: {
      const ObjectId linkId = engine.selection().primary();
      const SceneObject* link = engine.objects().model().find(linkId);
      if (!isLink(link) || link->parentLink == kNoObject) {
        return result(false, "Select a child link");
      }
      const std::string name = link->name;
      engine.execute(commands::setLinkParent(linkId, kNoObject));
      const SceneObject* updated = engine.objects().model().find(linkId);
      if (updated == nullptr || updated->parentLink != kNoObject
          || updated->parent != updated->multiBody) {
        return result(false, "Link root rejected");
      }
      return result(true, "Made " + name + " a root link", linkId);
    }
  }

  return result(false, "Unknown relationship action");
}

} // namespace dartsim::ui
