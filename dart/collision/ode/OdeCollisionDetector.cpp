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

#include "dart/collision/ode/OdeCollisionDetector.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/ode/OdeCollisionGroup.hpp"
#include "dart/collision/ode/OdeCollisionObject.hpp"
#include "dart/collision/ode/OdeTypes.hpp"
#include "dart/common/Macros.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"

#include <ode/ode.h>

#include <vector>

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

#if DART_ODE_HAS_LIBCCD_BOX_CYL
void alignBoxCylinderNormal(Contact& contact);
void stabilizeBoxCylinderContactPoint(Contact& contact);

bool expandBoxCylinderContact(
    const Contact& baseContact,
    const CollisionOption& option,
    CollisionResult& result);
#endif

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
  DART_ASSERT(initialized);
  DART_UNUSED(initialized);

  dAllocateODEDataForThread(dAllocateMaskAll);

  mWorldId = dWorldCreate();
  DART_ASSERT(mWorldId);
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
  DART_ASSERT(!dGeomIsSpace(o1));
  DART_ASSERT(!dGeomIsSpace(o2));

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
  DART_ASSERT(collObj1);
  DART_ASSERT(collObj2);

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
    auto contact = convertContact(contactGeoms[0], b1, b2, option);
#if DART_ODE_HAS_LIBCCD_BOX_CYL
    if (option.enableContact) {
      alignBoxCylinderNormal(contact);
      stabilizeBoxCylinderContactPoint(contact);
    }
#endif
    result.addContact(contact);

    return;
  }

  if (1 == numContacts) {
    auto baseContact = convertContact(contactGeoms[0], b1, b2, option);
#if DART_ODE_HAS_LIBCCD_BOX_CYL
    if (option.enableContact) {
      alignBoxCylinderNormal(baseContact);
      stabilizeBoxCylinderContactPoint(baseContact);
      if (expandBoxCylinderContact(baseContact, option, result))
        return;
    }
#endif
    result.addContact(baseContact);
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

  auto* odeObj1 = static_cast<OdeCollisionObject*>(dGeomGetData(odeContact.g1));
  auto* odeObj2 = static_cast<OdeCollisionObject*>(dGeomGetData(odeContact.g2));
  contact.collisionObject1 = odeObj1 ? odeObj1 : b1;
  contact.collisionObject2 = odeObj2 ? odeObj2 : b2;

  if (option.enableContact) {
    contact.point = OdeTypes::convertVector3(odeContact.pos);
    contact.normal = OdeTypes::convertVector3(odeContact.normal);
    contact.penetrationDepth = odeContact.depth;

    const double normalNorm = contact.normal.norm();
    if (normalNorm > Contact::getNormalEpsilon())
      contact.normal /= normalNorm;
  }

  return contact;
}

#if DART_ODE_HAS_LIBCCD_BOX_CYL
void alignBoxCylinderNormal(Contact& contact)
{
  const auto* shape1 = contact.collisionObject1->getShape().get();
  const auto* shape2 = contact.collisionObject2->getShape().get();

  const auto* boxShape = shape1->as<dynamics::BoxShape>();
  const auto* cylinderShape = shape2->as<dynamics::CylinderShape>();
  const collision::CollisionObject* boxObject = contact.collisionObject1;

  if (!(boxShape && cylinderShape)) {
    boxShape = shape2->as<dynamics::BoxShape>();
    cylinderShape = shape1->as<dynamics::CylinderShape>();
    boxObject = contact.collisionObject2;
  }

  if (!(boxShape && cylinderShape))
    return;

  Eigen::Vector3d normal = contact.normal;
  const double normalNorm = normal.norm();
  if (normalNorm <= 0.0)
    return;
  normal /= normalNorm;

  const Eigen::Matrix3d boxRotation = boxObject->getTransform().linear();
  double maxAbsDot = -1.0;
  Eigen::Vector3d bestAxis = Eigen::Vector3d::Zero();
  for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
    const Eigen::Vector3d axis = boxRotation.col(axisIndex);
    const double absDot = std::abs(axis.dot(normal));
    if (absDot > maxAbsDot) {
      maxAbsDot = absDot;
      bestAxis = axis;
    }
  }

  constexpr double kNormalSnapThreshold = 0.9;
  if (maxAbsDot < kNormalSnapThreshold)
    return;

  if (bestAxis.dot(normal) < 0.0)
    bestAxis = -bestAxis;

  contact.normal = bestAxis;
}

void stabilizeBoxCylinderContactPoint(Contact& contact)
{
  const auto* shape1 = contact.collisionObject1->getShape().get();
  const auto* shape2 = contact.collisionObject2->getShape().get();

  const auto* cylinderShape = shape1->as<dynamics::CylinderShape>();
  const auto* boxShape = shape2->as<dynamics::BoxShape>();
  const collision::CollisionObject* cylinderObject = contact.collisionObject1;
  const collision::CollisionObject* boxObject = contact.collisionObject2;

  if (!(cylinderShape && boxShape)) {
    cylinderShape = shape2->as<dynamics::CylinderShape>();
    boxShape = shape1->as<dynamics::BoxShape>();
    cylinderObject = contact.collisionObject2;
    boxObject = contact.collisionObject1;
  }

  if (!(cylinderShape && boxShape))
    return;

  Eigen::Vector3d normal = contact.normal;
  const double normalNorm = normal.norm();
  if (normalNorm <= 0.0)
    return;
  normal /= normalNorm;

  Eigen::Vector3d axis
      = cylinderObject->getTransform().linear() * Eigen::Vector3d::UnitZ();
  const double axisNorm = axis.norm();
  if (axisNorm <= 0.0)
    return;
  axis /= axisNorm;

  const Eigen::Matrix3d boxRotation = boxObject->getTransform().linear();
  double maxAbsDot = -1.0;
  for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
    const Eigen::Vector3d boxAxis = boxRotation.col(axisIndex);
    const double absDot = std::abs(boxAxis.dot(normal));
    if (absDot > maxAbsDot)
      maxAbsDot = absDot;
  }

  constexpr double kFaceContactThreshold = 0.9;
  if (maxAbsDot < kFaceContactThreshold)
    return;

  const Eigen::Vector3d cylinderCenter
      = cylinderObject->getTransform().translation();
  const double axisDot = std::abs(axis.dot(normal));
  const bool cylinderIsObject1 = (cylinderObject == contact.collisionObject1);
  const double directionSign = cylinderIsObject1 ? -1.0 : 1.0;

  constexpr double kAxisParallelThreshold = 0.9;
  constexpr double kAxisPerpendicularThreshold = 0.1;

  if (axisDot > kAxisParallelThreshold) {
    Eigen::Vector3d capAxis = axis;
    if (capAxis.dot(normal) < 0.0)
      capAxis = -capAxis;
    const double halfHeight = 0.5 * cylinderShape->getHeight();
    contact.point = cylinderCenter + directionSign * capAxis * halfHeight;
    return;
  }

  if (axisDot < kAxisPerpendicularThreshold) {
    const double radius = cylinderShape->getRadius();
    contact.point = cylinderCenter + directionSign * normal * radius;
  }
}
#endif

#if DART_ODE_HAS_LIBCCD_BOX_CYL
bool expandBoxCylinderContact(
    const Contact& baseContact,
    const CollisionOption& option,
    CollisionResult& result)
{
  if (option.maxNumContacts <= 1)
    return false;

  const auto* shape1 = baseContact.collisionObject1->getShape().get();
  const auto* shape2 = baseContact.collisionObject2->getShape().get();

  const auto* cylinderShape = shape1->as<dynamics::CylinderShape>();
  const auto* boxShape = shape2->as<dynamics::BoxShape>();
  const collision::CollisionObject* cylinderObj = baseContact.collisionObject1;

  if (!(cylinderShape && boxShape)) {
    cylinderShape = shape2->as<dynamics::CylinderShape>();
    boxShape = shape1->as<dynamics::BoxShape>();
    cylinderObj = baseContact.collisionObject2;
  }

  if (!(cylinderShape && boxShape))
    return false;

  const collision::CollisionObject* boxObj = (shape1->as<dynamics::BoxShape>())
                                                 ? baseContact.collisionObject1
                                                 : baseContact.collisionObject2;

  // Only expand if normal is face-aligned (same threshold as
  // alignBoxCylinderNormal)
  const Eigen::Vector3d normal = baseContact.normal;
  const Eigen::Matrix3d boxRotation = boxObj->getTransform().linear();
  double maxAbsDot = -1.0;
  for (int i = 0; i < 3; ++i) {
    const double absDot = std::abs(boxRotation.col(i).dot(normal));
    if (absDot > maxAbsDot)
      maxAbsDot = absDot;
  }

  constexpr double kNormalSnapThreshold = 0.9;
  if (maxAbsDot < kNormalSnapThreshold)
    return false;

  Eigen::Vector3d axis
      = cylinderObj->getTransform().linear() * Eigen::Vector3d::UnitZ();
  const double axisNorm = axis.norm();
  if (axisNorm <= 0.0)
    return false;
  axis /= axisNorm;

  const double axisDot = std::abs(axis.dot(normal));
  std::vector<Contact> contacts;

  if (axisDot < 0.98) {
    Eigen::Vector3d axisProjection = axis - axis.dot(normal) * normal;
    if (axisProjection.norm() <= 1e-8)
      return false;

    const double projectionLength = axisProjection.norm();
    const double offset = 0.5 * cylinderShape->getHeight() * projectionLength;
    axisProjection.normalize();

    Contact contactA = baseContact;
    contactA.point = baseContact.point + axisProjection * offset;
    contacts.push_back(contactA);

    Contact contactB = baseContact;
    contactB.point = baseContact.point - axisProjection * offset;
    contacts.push_back(contactB);
  } else {
    Eigen::Vector3d tangent1 = normal.unitOrthogonal();
    Eigen::Vector3d tangent2 = normal.cross(tangent1);
    tangent2.normalize();

    const double offset = cylinderShape->getRadius();
    Contact contactA = baseContact;
    contactA.point = baseContact.point + tangent1 * offset;
    contacts.push_back(contactA);

    Contact contactB = baseContact;
    contactB.point = baseContact.point - tangent1 * offset;
    contacts.push_back(contactB);

    Contact contactC = baseContact;
    contactC.point = baseContact.point + tangent2 * offset;
    contacts.push_back(contactC);

    Contact contactD = baseContact;
    contactD.point = baseContact.point - tangent2 * offset;
    contacts.push_back(contactD);
  }

  if (contacts.empty())
    return false;

  const double perContactDepth = baseContact.penetrationDepth;
  for (auto& contact : contacts) {
    contact.penetrationDepth = perContactDepth;
    result.addContact(contact);
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;
  }

  return true;
}
#endif

} // anonymous namespace

} // namespace collision
} // namespace dart
