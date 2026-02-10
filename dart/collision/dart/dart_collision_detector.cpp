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

#include "dart/collision/dart/dart_collision_detector.hpp"

#include "dart/collision/collision_object.hpp"
#include "dart/collision/dart/dart_collision_group.hpp"
#include "dart/collision/dart/dart_collision_object.hpp"
#include "dart/collision/dart/dart_query_helper.hpp"
#include "dart/collision/dart/shape_adapter.hpp"
#include "dart/collision/native/persistent_manifold_cache.hpp"
#include "dart/common/logging.hpp"
#include "dart/dynamics/shape_frame.hpp"

#include <optional>
#include <unordered_map>
#include <vector>

namespace dart {
namespace collision {

namespace {

bool checkGroupValidity(DartCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get()) {
    DART_ERROR(
        "Attempting to check collision for a collision group that is created "
        "from a different collision detector instance.");

    return false;
  }

  return true;
}

std::size_t objectId(const CollisionObject* object)
{
  return reinterpret_cast<std::size_t>(object);
}

void warmStartContacts(
    CollisionResult* result, native::PersistentManifoldCache* manifoldCache)
{
  if (!result || !manifoldCache) {
    return;
  }

  for (auto i = 0u; i < result->getNumContacts(); ++i) {
    auto& contact = result->getContact(i);
    auto* object1 = contact.collisionObject1;
    auto* object2 = contact.collisionObject2;
    if (!object1 || !object2) {
      continue;
    }

    const auto tf1 = object1->getTransform();
    const auto tf2 = object2->getTransform();
    const auto tf1Inv = tf1.inverse();
    const auto tf2Inv = tf2.inverse();

    native::CachedContact cached;
    cached.localPointA = tf1Inv * contact.point;
    cached.localPointB = tf2Inv * contact.point;
    cached.normal = contact.normal;
    cached.penetrationDepth = contact.penetrationDepth;

    const auto id1 = objectId(object1);
    const auto id2 = objectId(object2);
    auto& manifold = manifoldCache->getOrCreate(id1, id2);
    manifold.addOrReplace(cached);
    manifold.refresh(tf1, tf2);

    if (manifold.numContacts == 0) {
      manifoldCache->remove(id1, id2);
      continue;
    }

    const auto match = manifold.findMatch(cached.localPointA);
    const auto matchIndex = (match >= 0) ? match : 0;
    auto& manifoldContact
        = manifold.contacts[static_cast<std::size_t>(matchIndex)];
    contact.cachedNormalImpulse = manifoldContact.cachedNormalImpulse;
    contact.cachedFrictionImpulse1 = manifoldContact.cachedFrictionImpulse1;
    contact.cachedFrictionImpulse2 = manifoldContact.cachedFrictionImpulse2;
    contact.userData = &manifoldContact;
  }
}

void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects,
    native::PersistentManifoldCache* manifoldCache)
{
  if (!manifoldCache) {
    return;
  }

  std::unordered_map<std::size_t, CollisionObject*> objectsById;
  objectsById.reserve(objects.size());
  for (auto* object : objects) {
    objectsById[objectId(object)] = object;
  }

  manifoldCache->refreshAll(
      [&](std::size_t idA, std::size_t idB)
          -> std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> {
        const auto itA = objectsById.find(idA);
        const auto itB = objectsById.find(idB);
        if (itA == objectsById.end() || itB == objectsById.end()) {
          return std::nullopt;
        }
        return std::make_pair(
            itA->second->getTransform(), itB->second->getTransform());
      });
}

void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects1,
    const std::vector<CollisionObject*>& objects2,
    native::PersistentManifoldCache* manifoldCache)
{
  if (!manifoldCache) {
    return;
  }

  std::unordered_map<std::size_t, CollisionObject*> objectsById;
  objectsById.reserve(objects1.size() + objects2.size());
  for (auto* object : objects1) {
    objectsById[objectId(object)] = object;
  }
  for (auto* object : objects2) {
    objectsById[objectId(object)] = object;
  }

  manifoldCache->refreshAll(
      [&](std::size_t idA, std::size_t idB)
          -> std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> {
        const auto itA = objectsById.find(idA);
        const auto itB = objectsById.find(idB);
        if (itA == objectsById.end() || itB == objectsById.end()) {
          return std::nullopt;
        }
        return std::make_pair(
            itA->second->getTransform(), itB->second->getTransform());
      });
}

} // namespace

//==============================================================================
DartCollisionDetector::Registrar<DartCollisionDetector>
    DartCollisionDetector::mRegistrar{
        std::string(DartCollisionDetector::getStaticType()),
        []() -> std::shared_ptr<dart::collision::DartCollisionDetector> {
          return dart::collision::DartCollisionDetector::create();
        }};

// Backward compatibility: register under the old "experimental" key so that
// existing code/skel files using factory->create("experimental") still work.
DartCollisionDetector::Registrar<DartCollisionDetector>
    DartCollisionDetector::mRegistrarExperimental{
        "experimental",
        []() -> std::shared_ptr<dart::collision::DartCollisionDetector> {
          return dart::collision::DartCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DartCollisionDetector> DartCollisionDetector::create()
{
  return std::shared_ptr<DartCollisionDetector>(new DartCollisionDetector());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
DartCollisionDetector::cloneWithoutCollisionObjects() const
{
  return DartCollisionDetector::create();
}

//==============================================================================
const std::string& DartCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DartCollisionDetector::getStaticType()
{
  static const std::string type = "dart";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> DartCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DartCollisionGroup>(shared_from_this());
}

//==============================================================================
bool DartCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (!checkGroupValidity(this, group)) {
    return false;
  }

  auto* castedGroup = static_cast<DartCollisionGroup*>(group);
  const auto collision = dartCollide(
      castedGroup->mCollisionObjects,
      castedGroup->mCollisionObjects,
      option,
      result);

  if (collision && option.enableContact) {
    warmStartContacts(result, mManifoldCache.get());
  }

  refreshManifoldCache(castedGroup->mCollisionObjects, mManifoldCache.get());
  return collision;
}

//==============================================================================
bool DartCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (!checkGroupValidity(this, group1)) {
    return false;
  }

  if (!checkGroupValidity(this, group2)) {
    return false;
  }

  auto* castedGroup1 = static_cast<DartCollisionGroup*>(group1);
  auto* castedGroup2 = static_cast<DartCollisionGroup*>(group2);

  const auto collision = dartCollide(
      castedGroup1->mCollisionObjects,
      castedGroup2->mCollisionObjects,
      option,
      result);

  if (collision && option.enableContact) {
    warmStartContacts(result, mManifoldCache.get());
  }

  refreshManifoldCache(
      castedGroup1->mCollisionObjects,
      castedGroup2->mCollisionObjects,
      mManifoldCache.get());
  return collision;
}

//==============================================================================
double DartCollisionDetector::distance(
    CollisionGroup* group, const DistanceOption& option, DistanceResult* result)
{
  if (!checkGroupValidity(this, group)) {
    return 0.0;
  }

  const auto* castedGroup = static_cast<DartCollisionGroup*>(group);
  return dartDistance(
      castedGroup->mCollisionObjects,
      castedGroup->mCollisionObjects,
      option,
      result);
}

//==============================================================================
double DartCollisionDetector::distance(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (!checkGroupValidity(this, group1)) {
    return 0.0;
  }

  if (!checkGroupValidity(this, group2)) {
    return 0.0;
  }

  const auto* castedGroup1 = static_cast<DartCollisionGroup*>(group1);
  const auto* castedGroup2 = static_cast<DartCollisionGroup*>(group2);

  return dartDistance(
      castedGroup1->mCollisionObjects,
      castedGroup2->mCollisionObjects,
      option,
      result);
}

//==============================================================================
bool DartCollisionDetector::raycast(
    CollisionGroup* group,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (!checkGroupValidity(this, group)) {
    return false;
  }

  auto* castedGroup = static_cast<DartCollisionGroup*>(group);
  return dartRaycast(castedGroup->mCollisionObjects, from, to, option, result);
}

//==============================================================================
DartCollisionDetector::DartCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
  mManifoldCache = std::make_unique<native::PersistentManifoldCache>();
}

//==============================================================================
std::unique_ptr<CollisionObject> DartCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto shape = adaptShape(shapeFrame ? shapeFrame->getShape() : nullptr);
  return std::unique_ptr<DartCollisionObject>(
      new DartCollisionObject(this, shapeFrame, std::move(shape)));
}

//==============================================================================
void DartCollisionDetector::refreshCollisionObject(CollisionObject* object)
{
  if (!object) {
    return;
  }

  auto* casted = static_cast<DartCollisionObject*>(object);
  const auto* shapeFrame = casted->getShapeFrame();
  if (!shapeFrame) {
    return;
  }

  casted->setNativeShape(adaptShape(shapeFrame->getShape()));
}

} // namespace collision
} // namespace dart
