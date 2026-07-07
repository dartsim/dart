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
#include "dart/collision/Contact.hpp"
#include "dart/collision/ode/OdeCollisionGroup.hpp"
#include "dart/collision/ode/OdeCollisionObject.hpp"
#include "dart/collision/ode/OdeTypes.hpp"
#include "dart/common/Macros.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <ode/ode.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <cmath>

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
    CollisionResult& result,
    std::vector<OdeCollisionDetector::ContactHistoryItem>* history);

Contact convertContact(
    const dContactGeom& fclContact,
    OdeCollisionObject* b1,
    OdeCollisionObject* b2,
    const CollisionOption& option);

std::size_t reportCylinderPlaneSupportContacts(
    const std::vector<OdeCollisionObject*>& cylinders,
    const std::vector<OdeCollisionObject*>& planes,
    const CollisionOption& option,
    CollisionResult* result,
    bool cylinderFirst = true);

#if DART_ODE_HAS_LIBCCD_BOX_CYL
void alignBoxCylinderNormal(Contact& contact);
void stabilizeBoxCylinderContactPoint(Contact& contact);

bool expandBoxCylinderContact(
    const Contact& baseContact,
    const CollisionOption& option,
    CollisionResult& result);
#endif

double computeTangentialSpeed(const Contact& contact);

bool shouldUseContactHistory(
    const CollisionObject* object1, const CollisionObject* object2);

OdeCollisionDetector::CollObjPair MakeNewPair(
    CollisionObject* o1, CollisionObject* o2)
{
  if (std::less<CollisionObject*>()(o2, o1)) {
    std::swap(o1, o2);
  }
  return std::make_pair(o1, o2);
}

bool hasTransformMoved(
    const Eigen::Isometry3d& previous, const Eigen::Isometry3d& current)
{
  constexpr double translationTolerance = 5e-2;
  constexpr double rotationTolerance = 5e-2;
  return (previous.translation() - current.translation()).norm()
             > translationTolerance
         || !previous.linear().isApprox(current.linear(), rotationTolerance);
}

void refreshHistoryTransforms(OdeCollisionDetector::ContactHistoryItem& item)
{
  const auto& transform1 = item.pair.first->getTransform();
  const auto& transform2 = item.pair.second->getTransform();

  if (!item.hasTransforms || item.history.empty()) {
    item.transform1 = transform1;
    item.transform2 = transform2;
    item.hasTransforms = true;
    return;
  }

  const bool moved = hasTransformMoved(item.transform1, transform1)
                     || hasTransformMoved(item.transform2, transform2);
  if (moved) {
    item.history.clear();
    item.transform1 = transform1;
    item.transform2 = transform2;
  }
  // Otherwise keep the previous transforms as the contact-history anchor so
  // several small kinematic pose changes still invalidate stale world points.
}

OdeCollisionDetector::ContactHistoryItem& FindPairInHist(
    std::vector<OdeCollisionDetector::ContactHistoryItem>& cache,
    const OdeCollisionDetector::CollObjPair& pair)
{
  for (auto& item : cache) {
    if (pair.first == item.pair.first && pair.second == item.pair.second) {
      return item;
    }
  }

  OdeCollisionDetector::ContactHistoryItem newItem;
  newItem.pair = pair;
  newItem.transform1 = Eigen::Isometry3d::Identity();
  newItem.transform2 = Eigen::Isometry3d::Identity();
  newItem.hasTransforms = false;
  cache.push_back(newItem);

  return cache.back();
}

void eraseHistoryForObject(
    std::vector<OdeCollisionDetector::ContactHistoryItem>& cache,
    const CollisionObject* object)
{
  if (!object)
    return;

  cache.erase(
      std::remove_if(
          cache.begin(),
          cache.end(),
          [object](const OdeCollisionDetector::ContactHistoryItem& item) {
            return item.pair.first == object || item.pair.second == object;
          }),
      cache.end());
}

struct OdeCollisionCallbackData
{
  dContactGeom* contactGeoms;

  /// Collision option of DART
  const CollisionOption& option;

  /// Collision result of DART
  CollisionResult* result;
  std::vector<OdeCollisionDetector::ContactHistoryItem>* history;

  /// Whether the collision iteration can stop
  bool done;

  /// The total number of contacts collected by dSpaceCollide() or
  /// dSpaceCollide2(). This field is used to determine the binary contact
  /// result.
  std::size_t numContacts;

  OdeCollisionCallbackData(
      const CollisionOption& option, CollisionResult* result)
    : option(option),
      result(result),
      history(nullptr),
      done(false),
      numContacts(0u)
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
  data.history = &mContactHistory;

  dSpaceCollide(odeGroup->getOdeSpaceId(), &data, CollisionCallback);
  data.numContacts += reportCylinderPlaneSupportContacts(
      odeGroup->getCylinderCollisionObjects(),
      odeGroup->getPlaneCollisionObjects(),
      option,
      result,
      true);

  if (result) {
    pruneContactHistory(*result);
  }

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
  data.history = &mContactHistory;

  dSpaceCollide2(
      reinterpret_cast<dGeomID>(odeGroup1->getOdeSpaceId()),
      reinterpret_cast<dGeomID>(odeGroup2->getOdeSpaceId()),
      &data,
      CollisionCallback);
  data.numContacts += reportCylinderPlaneSupportContacts(
      odeGroup1->getCylinderCollisionObjects(),
      odeGroup2->getPlaneCollisionObjects(),
      option,
      result,
      true);
  data.numContacts += reportCylinderPlaneSupportContacts(
      odeGroup2->getCylinderCollisionObjects(),
      odeGroup1->getPlaneCollisionObjects(),
      option,
      result,
      false);

  if (result) {
    pruneContactHistory(*result);
  }

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

//==============================================================================
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
  const auto maxContactsPerPair = std::min<std::size_t>(
      option.getEffectiveMaxNumContactsPerPair(), MAX_COLLIDE_RETURNS);
  if (maxContactsPerPair == 0u)
    return;

  auto numc = dCollide(
      o1,
      o2,
      static_cast<int>(maxContactsPerPair),
      odeResult,
      sizeof(odeResult[0]));

  cdData->numContacts += numc;

  if (result) {
    reportContacts(
        numc, odeResult, collObj1, collObj2, option, *result, cdData->history);
  }
}

//==============================================================================
void reportContacts(
    int numContacts,
    dContactGeom* contactGeoms,
    OdeCollisionObject* b1,
    OdeCollisionObject* b2,
    const CollisionOption& option,
    CollisionResult& result,
    std::vector<OdeCollisionDetector::ContactHistoryItem>* history)
{
  if (0u == numContacts)
    return;

  if (0u == option.maxNumContacts)
    return;

  if (result.getNumContacts() >= option.maxNumContacts)
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

  // Box-cylinder pairs keep release-6.17's libccd single-contact stabilization
  // and return here. Every OTHER single-contact pair (e.g. a resting capsule)
  // must fall through to the contact-history path below so it is supplemented
  // to a stable manifold; otherwise the capsule sinks through the ground.
  // See gazebosim/gz-physics#692 / dartsim/dart#1654.
#if DART_ODE_HAS_LIBCCD_BOX_CYL
  if (1 == numContacts && option.enableContact) {
    const auto* shapeA = b1->getShape().get();
    const auto* shapeB = b2->getShape().get();
    const bool boxCylinderPair = (shapeA->as<dynamics::BoxShape>()
                                  && shapeB->as<dynamics::CylinderShape>())
                                 || (shapeA->as<dynamics::CylinderShape>()
                                     && shapeB->as<dynamics::BoxShape>());
    if (boxCylinderPair) {
      auto baseContact = convertContact(contactGeoms[0], b1, b2, option);
      alignBoxCylinderNormal(baseContact);
      stabilizeBoxCylinderContactPoint(baseContact);
      if (expandBoxCylinderContact(baseContact, option, result))
        return;
      result.addContact(baseContact);
      return;
    }
  }
#endif

  const auto maxContactsPerPair = option.getEffectiveMaxNumContactsPerPair();
  const auto available = option.maxNumContacts - result.getNumContacts();
  const auto requested = static_cast<std::size_t>(numContacts);
  const auto contactsToCopy
      = static_cast<int>(std::min({requested, available, maxContactsPerPair}));

  // ODE visits a given collision-object pair at most once per collide() call
  // (each OdeCollisionObject owns exactly one non-space dGeomID), so the
  // contacts appended below for `pair` land in one contiguous span of
  // `result`'s contact vector. Recording [pairContactsBegin, pairSpanEnd)
  // lets the history logic below address this pair's current-round contacts
  // directly by index instead of copying/re-scanning every contact
  // accumulated so far across all pairs.
  const std::size_t pairContactsBegin = result.getNumContacts();

  for (auto i = 0; i < contactsToCopy; ++i) {
    result.addContact(convertContact(contactGeoms[i], b1, b2, option));
  }

  if (result.getNumContacts() >= option.maxNumContacts) {
    return;
  }

  // Skip the contact-history path entirely for binary queries
  // (enableContact == false): convertContact() leaves point/normal/depth at
  // their defaults for those, so caching them would later supplement a real
  // manifold with stale zero-depth contacts at the world origin.
  if (!history || !option.enableContact || !shouldUseContactHistory(b1, b2)) {
    return;
  }

  const auto pair = MakeNewPair(b1, b2);
  const std::size_t pairSpanEnd
      = pairContactsBegin + static_cast<std::size_t>(contactsToCopy);
  const std::size_t pairContactCount = pairSpanEnd - pairContactsBegin;
  const std::size_t pairTarget
      = std::min<std::size_t>(3u, option.getEffectiveMaxNumContactsPerPair());

  auto foundHistory = std::find_if(
      history->begin(),
      history->end(),
      [&pair](const OdeCollisionDetector::ContactHistoryItem& item) {
        return pair.first == item.pair.first && pair.second == item.pair.second;
      });
  if (pairContactCount >= pairTarget && foundHistory == history->end()) {
    return;
  }

  auto& historyItem = foundHistory != history->end()
                          ? *foundHistory
                          : FindPairInHist(*history, pair);
  refreshHistoryTransforms(historyItem);
  auto& pastContacsVec = historyItem.history;

  bool sliding = false;
  constexpr double slidingThreshold = 1e-3;
  for (std::size_t i = pairContactsBegin; i < pairSpanEnd; ++i) {
    const auto& curr_cont = result.getContact(i);
    if (computeTangentialSpeed(curr_cont) > slidingThreshold) {
      sliding = true;
      break;
    }
  }

  if (sliding) {
    // The pair is moving tangentially, so the cached world-space contacts no
    // longer describe the current contact patch. Drop them instead of leaving
    // stale points that a later resting query could resurface far from the new
    // patch.
    pastContacsVec.clear();
    return;
  }

  if (pairContactCount >= pairTarget) {
    return;
  }

  std::size_t missing = pairTarget - pairContactCount;
  const auto globalRemaining = option.maxNumContacts - result.getNumContacts();
  missing = std::min(missing, globalRemaining);
  if (missing == 0u) {
    return;
  }

  for (auto it = pastContacsVec.rbegin();
       it != pastContacsVec.rend() && missing > 0u;
       ++it) {
    auto past_cont = *it;
    bool matchesCurrentContact = false;
    for (std::size_t i = pairContactsBegin; i < pairSpanEnd; ++i) {
      const auto& curr_cont = result.getContact(i);
      auto dist_v = past_cont.point - curr_cont.point;
      const auto dist_m = (dist_v.transpose() * dist_v).coeff(0, 0);
      if (dist_m < 0.01) {
        matchesCurrentContact = true;
        break;
      }
    }

    if (matchesCurrentContact) {
      continue;
    }

    if (result.getNumContacts() >= option.maxNumContacts) {
      return;
    }
    result.addContact(past_cont);
    if (--missing == 0u)
      break;
  }
  for (std::size_t i = pairContactsBegin; i < pairSpanEnd; ++i) {
    pastContacsVec.push_back(result.getContact(i));
  }

  const auto size = pastContacsVec.size();
  if (size > 11) {
    pastContacsVec.erase(pastContacsVec.begin(), pastContacsVec.end() - 5);
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

//==============================================================================
bool resultHasContactForPair(
    const CollisionResult* result,
    const OdeCollisionObject* object1,
    const OdeCollisionObject* object2)
{
  if (!result)
    return false;

  for (const auto& contact : result->getContacts()) {
    if ((contact.collisionObject1 == object1
         && contact.collisionObject2 == object2)
        || (contact.collisionObject1 == object2
            && contact.collisionObject2 == object1)) {
      return true;
    }
  }

  return false;
}

//==============================================================================
bool reportCylinderPlaneSupportContact(
    OdeCollisionObject* cylinderObject,
    OdeCollisionObject* planeObject,
    const CollisionOption& option,
    CollisionResult* result,
    bool cylinderFirst)
{
  const auto* cylinderShape
      = cylinderObject->getShape()->as<dynamics::CylinderShape>();
  const auto* planeShape = planeObject->getShape()->as<dynamics::PlaneShape>();

  if (!(cylinderShape && planeShape))
    return false;

  const auto& filter = option.collisionFilter;
  if (filter && filter->ignoresCollision(cylinderObject, planeObject))
    return false;

  if (option.getEffectiveMaxNumContactsPerPair() == 0u)
    return false;

  if (resultHasContactForPair(result, cylinderObject, planeObject))
    return false;

  const Eigen::Vector3d rawNormal
      = planeObject->getTransform().linear() * planeShape->getNormal();
  const double normalNorm = rawNormal.norm();
  if (!std::isfinite(normalNorm)
      || normalNorm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const Eigen::Vector3d worldNormal = rawNormal / normalNorm;
  const double planeOffset
      = planeShape->getOffset()
        + planeObject->getTransform().translation().dot(worldNormal);

  const Eigen::Isometry3d& cylinderTf = cylinderObject->getTransform();
  const Eigen::Vector3d rawAxis
      = cylinderTf.linear() * Eigen::Vector3d::UnitZ();
  const double axisNorm = rawAxis.norm();
  if (!std::isfinite(axisNorm)
      || axisNorm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const Eigen::Vector3d axis = rawAxis / axisNorm;
  const Eigen::Vector3d center = cylinderTf.translation();
  const double centerDistance = worldNormal.dot(center) - planeOffset;
  const double axisProjection = worldNormal.dot(axis);
  const double perpendicularProjection
      = std::sqrt(std::max(0.0, 1.0 - axisProjection * axisProjection));
  const double radius = cylinderShape->getRadius();
  const double halfHeight = 0.5 * cylinderShape->getHeight();
  const double supportDistance = halfHeight * std::abs(axisProjection)
                                 + radius * perpendicularProjection;
  const double clearance = centerDistance - supportDistance;
  const double tolerance
      = 1e-9 * std::max({1.0, radius, cylinderShape->getHeight()});

  if (!std::isfinite(clearance) || clearance > tolerance)
    return false;

  const double axisOffset = axisProjection >= 0.0 ? -halfHeight : halfHeight;
  Eigen::Vector3d radialOffset = Eigen::Vector3d::Zero();
  if (perpendicularProjection > std::numeric_limits<double>::epsilon()) {
    const Eigen::Vector3d radialDirection
        = (worldNormal - axisProjection * axis) / perpendicularProjection;
    radialOffset = -radius * radialDirection;
  }

  if (!result || result->getNumContacts() >= option.maxNumContacts)
    return true;

  Contact contact;
  contact.collisionObject1 = cylinderFirst ? cylinderObject : planeObject;
  contact.collisionObject2 = cylinderFirst ? planeObject : cylinderObject;

  if (option.enableContact) {
    contact.point = center + axisOffset * axis + radialOffset;
    contact.normal = cylinderFirst ? worldNormal : -worldNormal;
    contact.penetrationDepth = std::max(0.0, -clearance);
  }

  result->addContact(contact);
  return true;
}

//==============================================================================
std::size_t reportCylinderPlaneSupportContacts(
    const std::vector<OdeCollisionObject*>& cylinders,
    const std::vector<OdeCollisionObject*>& planes,
    const CollisionOption& option,
    CollisionResult* result,
    bool cylinderFirst)
{
  if (cylinders.empty() || planes.empty())
    return 0u;

  std::size_t reported = 0u;
  for (auto* cylinder : cylinders) {
    for (auto* plane : planes) {
      if (reportCylinderPlaneSupportContact(
              cylinder, plane, option, result, cylinderFirst)) {
        ++reported;
      }
    }
  }

  return reported;
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
  const auto maxContactsPerPair = option.getEffectiveMaxNumContactsPerPair();
  if (maxContactsPerPair <= 1)
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
  std::size_t pairContacts = 0u;
  for (auto& contact : contacts) {
    contact.penetrationDepth = perContactDepth;
    result.addContact(contact);
    ++pairContacts;
    if (pairContacts >= maxContactsPerPair)
      return true;
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;
  }

  return true;
}
#endif

double computeTangentialSpeed(const Contact& contact)
{
  const dynamics::BodyNode* bn1 = contact.collisionObject1
                                      ? contact.collisionObject1->getBodyNode()
                                      : nullptr;
  const dynamics::BodyNode* bn2 = contact.collisionObject2
                                      ? contact.collisionObject2->getBodyNode()
                                      : nullptr;

  const Eigen::Vector3d worldPoint = contact.point;

  Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
  if (bn1) {
    const Eigen::Vector3d localPoint
        = bn1->getWorldTransform().inverse() * worldPoint;
    v1 = bn1->getLinearVelocity(
        localPoint, dynamics::Frame::World(), dynamics::Frame::World());
  }

  Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
  if (bn2) {
    const Eigen::Vector3d localPoint
        = bn2->getWorldTransform().inverse() * worldPoint;
    v2 = bn2->getLinearVelocity(
        localPoint, dynamics::Frame::World(), dynamics::Frame::World());
  }

  const Eigen::Vector3d rel = v1 - v2;
  Eigen::Vector3d normal = contact.normal;
  const double normalNorm = normal.norm();
  if (normalNorm > 0.0) {
    normal /= normalNorm;
  } else {
    normal.setZero();
  }
  const Eigen::Vector3d tangential = rel - rel.dot(normal) * normal;
  return tangential.norm();
}

bool shouldUseContactHistory(
    const CollisionObject* object1, const CollisionObject* object2)
{
  // Only persist contacts for pairs whose tangential motion we can actually
  // track: both objects must be ShapeNodes backed by a BodyNode. The
  // sliding/tangential-speed check that filters stale cache entries relies on
  // body velocities (computeTangentialSpeed), so for frames without a BodyNode
  // (e.g. a kinematically moved SimpleFrame) it always reads zero speed and
  // could resurface world-space contact points from a previous pose.
  const auto hasTrackedBody = [](const CollisionObject* object) {
    if (object == nullptr)
      return false;
    return object->getBodyNode() != nullptr;
  };
  return hasTrackedBody(object1) && hasTrackedBody(object2);
}

} // anonymous namespace

//==============================================================================
void OdeCollisionDetector::pruneContactHistory(const CollisionResult& result)
{
  if (mContactHistory.empty())
    return;

  if (result.getNumContacts() == 0u) {
    mContactHistory.clear();
    return;
  }

  const auto& contacts = result.getContacts();
  for (auto& pastContact : mContactHistory) {
    bool clear = true;
    for (const auto& current : contacts) {
      auto currentPair
          = MakeNewPair(current.collisionObject1, current.collisionObject2);
      if (pastContact.pair == currentPair) {
        clear = false;
        break;
      }
    }
    if (clear) {
      pastContact.history.clear();
    }
  }
}

//==============================================================================
void OdeCollisionDetector::clearContactHistoryFor(const CollisionObject* object)
{
  eraseHistoryForObject(mContactHistory, object);
}

//==============================================================================
void OdeCollisionDetector::clearContactHistory()
{
  mContactHistory.clear();
}

} // namespace collision
} // namespace dart
