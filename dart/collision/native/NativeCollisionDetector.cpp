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

#include "dart/collision/native/NativeCollisionDetector.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/native/NativeCollisionGroup.hpp"
#include "dart/collision/native/NativeCollisionObject.hpp"
#include "dart/collision/native/narrow_phase/narrow_phase.hpp"
#include "dart/common/Console.hpp"

namespace dart {
namespace collision {

namespace {

//==============================================================================
bool checkGroupValidity(
    const NativeCollisionDetector* detector, CollisionGroup* group)
{
  if (!group) {
    dterr << "[NativeCollisionDetector::collide] Attempting to check collision "
          << "with a nullptr collision group.\n";
    return false;
  }

  if (detector != group->getCollisionDetector().get()) {
    dterr << "[NativeCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
native::CollisionOption makeNativeOption(
    const CollisionOption& option, const CollisionResult* result)
{
  native::CollisionOption nativeOption;
  nativeOption.enableContact = option.enableContact && result != nullptr;
  nativeOption.maxNumContacts = nativeOption.enableContact
                                    ? option.getEffectiveMaxNumContactsPerPair()
                                    : 1u;
  nativeOption.collisionFilter = nullptr;

  return nativeOption;
}

//==============================================================================
bool shouldSkipPair(
    const NativeCollisionObject* object1,
    const NativeCollisionObject* object2,
    const CollisionOption& option)
{
  if (!object1->getNativeShape() || !object2->getNativeShape())
    return true;

  if (option.collisionFilter
      && option.collisionFilter->ignoresCollision(object1, object2)) {
    return true;
  }

  return false;
}

//==============================================================================
void addPairOnlyContact(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    CollisionResult& result)
{
  Contact contact;
  contact.collisionObject1 = object1;
  contact.collisionObject2 = object2;
  result.addContact(contact);
}

//==============================================================================
bool emitContacts(
    const native::CollisionResult& nativeResult,
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const std::size_t maxPairContacts
      = option.getEffectiveMaxNumContactsPerPair();
  std::size_t emittedForPair = 0u;

  const std::size_t numContacts = nativeResult.numContacts();
  for (std::size_t i = 0u; i < numContacts; ++i) {
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;

    if (emittedForPair >= maxPairContacts)
      return false;

    const auto& nativeContact = nativeResult.getContact(i);
    if (nativeContact.depth < 0.0
        && !option.allowNegativePenetrationDepthContacts) {
      continue;
    }

    if (Contact::isZeroNormal(nativeContact.normal))
      continue;

    Contact contact;
    contact.point = nativeContact.position;
    contact.normal = nativeContact.normal;
    contact.penetrationDepth = nativeContact.depth;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.triID1 = nativeContact.featureIndex1;
    contact.triID2 = nativeContact.featureIndex2;
    result.addContact(contact);
    ++emittedForPair;
  }

  return result.getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
bool processNativePair(
    NativeCollisionObject* object1,
    NativeCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound)
{
  if (shouldSkipPair(object1, object2, option))
    return false;

  if (result && result->getNumContacts() >= option.maxNumContacts)
    return true;

  native::CollisionResult nativeResult;
  const native::CollisionOption nativeOption = makeNativeOption(option, result);
  const bool hit = native::NarrowPhase::collide(
      object1->getNativeShape(),
      object1->getNativeTransform(),
      object2->getNativeShape(),
      object2->getNativeTransform(),
      nativeOption,
      nativeResult);

  if (!hit)
    return false;

  collisionFound = true;

  if (!result)
    return true;

  if (!option.enableContact) {
    addPairOnlyContact(object1, object2, *result);
    return result->getNumContacts() >= option.maxNumContacts;
  }

  return emitContacts(nativeResult, object1, object2, option, *result);
}

} // namespace

//==============================================================================
NativeCollisionDetector::Registrar<NativeCollisionDetector>
    NativeCollisionDetector::mRegistrar{
        NativeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<NativeCollisionDetector> {
          return NativeCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<NativeCollisionDetector> NativeCollisionDetector::create()
{
  return std::shared_ptr<NativeCollisionDetector>(
      new NativeCollisionDetector());
}

//==============================================================================
NativeCollisionDetector::~NativeCollisionDetector() = default;

//==============================================================================
std::shared_ptr<CollisionDetector>
NativeCollisionDetector::cloneWithoutCollisionObjects() const
{
  return NativeCollisionDetector::create();
}

//==============================================================================
const std::string& NativeCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& NativeCollisionDetector::getStaticType()
{
  static const std::string type = "native";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> NativeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<NativeCollisionGroup>(shared_from_this());
}

//==============================================================================
bool NativeCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto* nativeGroup = static_cast<NativeCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  bool collisionFound = false;
  nativeGroup->mBroadPhase->visitPairs([&](std::size_t id1, std::size_t id2) {
    auto* object1 = nativeGroup->mIdToObject.at(id1);
    auto* object2 = nativeGroup->mIdToObject.at(id2);
    return !processNativePair(object1, object2, option, result, collisionFound);
  });

  return collisionFound;
}

//==============================================================================
bool NativeCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  if (group1 == group2)
    return collide(group1, option, result);

  auto* nativeGroup1 = static_cast<NativeCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<NativeCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();

  bool collisionFound = false;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativePair(
              static_cast<NativeCollisionObject*>(object1),
              static_cast<NativeCollisionObject*>(object2),
              option,
              result,
              collisionFound)) {
        return collisionFound;
      }
    }
  }

  return collisionFound;
}

//==============================================================================
double NativeCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[NativeCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
double NativeCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[NativeCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
NativeCollisionDetector::NativeCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject> NativeCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<NativeCollisionObject>(
      new NativeCollisionObject(this, shapeFrame));
}

//==============================================================================
void NativeCollisionDetector::refreshCollisionObject(
    CollisionObject* /*object*/)
{
  // Do nothing. NativeCollisionObject refreshes lazily in updateEngineData().
}

} // namespace collision
} // namespace dart
