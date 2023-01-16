/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/ode/OdeCollisionDetector.hpp"

#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/CollisionFilter.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/ode/OdeCollisionGroup.hpp"
#include "dart/dynamics/ode/OdeCollisionObject.hpp"
#include "dart/dynamics/ode/OdeTypes.hpp"

#include <ode/ode.h>

namespace dart {
namespace collision {

namespace {

void CollisionCallback(void* data, dGeomID o1, dGeomID o2);

void reportContacts(
    int numContacts,
    dContactGeom* contactGeoms,
    OdeCollisionObject* b1,
    OdeCollisionObject* b2,
    const CollisionOption& option,
    CollisionResult& result);

Contact convertContact(
    const dContactGeom& fclContact,
    OdeCollisionObject* b1,
    OdeCollisionObject* b2,
    const CollisionOption& option);

struct OdeCollisionCallbackData
{
  dContactGeom* contactGeoms;

  /// Collision option of DART
  const CollisionOption& option;

  /// Collision result of DART
  CollisionResult* result;

  /// Whether the collision iteration can stop
  bool done;

  /// The total number of contacts collected by dSpaceCollide() or
  /// dSpaceCollide2(). This field is used to determine the binary contact
  /// result.
  std::size_t numContacts;

  OdeCollisionCallbackData(
      const CollisionOption& option, CollisionResult* result)
    : option(option), result(result), done(false), numContacts(0u)
  {
    // Do nothing
  }
};

} // anonymous namespace

//==============================================================================
OdeCollisionDetector::Registrar<OdeCollisionDetector>
    OdeCollisionDetector::mRegistrar{
        OdeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::OdeCollisionDetector> {
          return dart::collision::OdeCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<OdeCollisionDetector> OdeCollisionDetector::create()
{
  return std::shared_ptr<OdeCollisionDetector>(new OdeCollisionDetector());
}

//==============================================================================
OdeCollisionDetector::~OdeCollisionDetector()
{
  dWorldDestroy(mWorldId);
  mWorldId = nullptr;

  dCloseODE();
}

//==============================================================================
std::shared_ptr<CollisionDetector>
OdeCollisionDetector::cloneWithoutCollisionObjects() const
{
  return OdeCollisionDetector::create();
}

//==============================================================================
const std::string& OdeCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& OdeCollisionDetector::getStaticType()
{
  static const std::string type = "ode";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> OdeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<OdeCollisionGroup>(shared_from_this());
}

//==============================================================================
bool OdeCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  auto odeGroup = static_cast<OdeCollisionGroup*>(group);
  odeGroup->updateEngineData();

  OdeCollisionCallbackData data(option, result);
  data.contactGeoms = contactCollisions;

  dSpaceCollide(odeGroup->getOdeSpaceId(), &data, CollisionCallback);

  return data.numContacts > 0;
}

//==============================================================================
bool OdeCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  auto odeGroup1 = static_cast<OdeCollisionGroup*>(group1);
  odeGroup1->updateEngineData();

  auto odeGroup2 = static_cast<OdeCollisionGroup*>(group2);
  odeGroup2->updateEngineData();

  OdeCollisionCallbackData data(option, result);
  data.contactGeoms = contactCollisions;

  dSpaceCollide2(
      reinterpret_cast<dGeomID>(odeGroup1->getOdeSpaceId()),
      reinterpret_cast<dGeomID>(odeGroup2->getOdeSpaceId()),
      &data,
      CollisionCallback);

  return data.numContacts > 0;
}

//==============================================================================
double OdeCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dterr << "[OdeCollisionDetector] Distance query is not supported. "
        << "Returning -1.0 instead.\n";
  return -1.0;
}

//==============================================================================
double OdeCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dterr << "[OdeCollisionDetector] Distance query is not supported. "
        << "Returning -1.0 instead.\n";
  return -1.0;
}

//==============================================================================
OdeCollisionDetector::OdeCollisionDetector()
{
  // Initialize ODE. dInitODE is deprecated.
  const auto initialized = dInitODE2(0);
  assert(initialized);
  DART_UNUSED(initialized);

  dAllocateODEDataForThread(dAllocateMaskAll);

  mWorldId = dWorldCreate();
  assert(mWorldId);
}

//==============================================================================
std::unique_ptr<CollisionObject> OdeCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<OdeCollisionObject>(
      new OdeCollisionObject(this, shapeFrame));
}

//==============================================================================
void OdeCollisionDetector::refreshCollisionObject(CollisionObject* object)
{
  OdeCollisionObject temp(this, object->getShapeFrame());

  static_cast<OdeCollisionObject&>(*object) =
      //      OdeCollisionObject(this, object->getShapeFrame());
      std::move(temp);
}

//==============================================================================
dWorldID OdeCollisionDetector::getOdeWorldId() const
{
  return mWorldId;
}

namespace {

//==============================================================================
void CollisionCallback(void* data, dGeomID o1, dGeomID o2)
{
  assert(!dGeomIsSpace(o1));
  assert(!dGeomIsSpace(o2));

  auto cdData = static_cast<OdeCollisionCallbackData*>(data);

  if (cdData->done)
    return;

  auto& odeResult = cdData->contactGeoms;
  auto* result = cdData->result;
  const auto& option = cdData->option;
  const auto& filter = option.collisionFilter;

  auto geomData1 = dGeomGetData(o1);
  auto geomData2 = dGeomGetData(o2);

  auto collObj1 = static_cast<OdeCollisionObject*>(geomData1);
  auto collObj2 = static_cast<OdeCollisionObject*>(geomData2);
  assert(collObj1);
  assert(collObj2);

  if (filter && filter->ignoresCollision(collObj1, collObj2))
    return;

  // Perform narrow-phase collision detection
  auto numc
      = dCollide(o1, o2, MAX_COLLIDE_RETURNS, odeResult, sizeof(odeResult[0]));

  cdData->numContacts += numc;

  if (result)
    reportContacts(numc, odeResult, collObj1, collObj2, option, *result);
}

//==============================================================================
void reportContacts(
    int numContacts,
    dContactGeom* contactGeoms,
    OdeCollisionObject* b1,
    OdeCollisionObject* b2,
    const CollisionOption& option,
    CollisionResult& result)
{
  if (0u == numContacts)
    return;

  // For binary check, return after adding the first contact point to the result
  // without the checkings of repeatidity and co-linearity.
  if (1u == option.maxNumContacts) {
    result.addContact(convertContact(contactGeoms[0], b1, b2, option));

    return;
  }

  for (auto i = 0; i < numContacts; ++i) {
    result.addContact(convertContact(contactGeoms[i], b1, b2, option));

    if (result.getNumContacts() >= option.maxNumContacts)
      return;
  }
}

//==============================================================================
Contact convertContact(
    const dContactGeom& odeContact,
    OdeCollisionObject* b1,
    OdeCollisionObject* b2,
    const CollisionOption& option)
{
  Contact contact;

  contact.collisionObject1 = b1;
  contact.collisionObject2 = b2;

  if (option.enableContact) {
    contact.point = OdeTypes::convertVector3(odeContact.pos);
    contact.normal = OdeTypes::convertVector3(odeContact.normal);
    contact.penetrationDepth = odeContact.depth;
  }

  return contact;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
