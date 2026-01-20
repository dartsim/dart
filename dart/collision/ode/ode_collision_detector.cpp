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

#include "dart/collision/ode/ode_collision_detector.hpp"

#include "dart/collision/collision_filter.hpp"
#include "dart/collision/contact.hpp"
#include "dart/collision/ode/ode_collision_group.hpp"
#include "dart/collision/ode/ode_collision_object.hpp"
#include "dart/collision/ode/ode_types.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/frame.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/multi_sphere_convex_hull_shape.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/soft_mesh_shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"

#include <ode/ode.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <utility>

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

std::deque<Contact>& FindPairInHist(
    std::vector<OdeCollisionDetector::ContactHistoryItem>& cache,
    const OdeCollisionDetector::CollObjPair& pair)
{
  for (auto& item : cache) {
    if (pair.first == item.pair.first && pair.second == item.pair.second) {
      return item.history;
    }
  }
  auto newItem
      = OdeCollisionDetector::ContactHistoryItem{pair, std::deque<Contact>()};
  cache.push_back(newItem);

  return cache.back().history;
}

void eraseHistoryForObject(
    std::vector<OdeCollisionDetector::ContactHistoryItem>& cache,
    const CollisionObject* object)
{
  if (!object)
    return;

  std::erase_if(
      cache, [object](const OdeCollisionDetector::ContactHistoryItem& item) {
        return item.pair.first == object || item.pair.second == object;
      });
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
        std::string(OdeCollisionDetector::getStaticType()),
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
  if (0u == option.maxNumContacts) {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  auto odeGroup = static_cast<OdeCollisionGroup*>(group);
  odeGroup->updateEngineData();

  OdeCollisionCallbackData data(option, result);
  data.contactGeoms = contactCollisions;
  data.history = &mContactHistory;

  dSpaceCollide(odeGroup->getOdeSpaceId(), &data, CollisionCallback);

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
  if (0u == option.maxNumContacts) {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

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
  DART_ERROR(
      "[OdeCollisionDetector] Distance query is not supported. Returning -1.0 "
      "instead.");
  return -1.0;
}

//==============================================================================
double OdeCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_ERROR(
      "[OdeCollisionDetector] Distance query is not supported. Returning -1.0 "
      "instead.");
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
  auto numc
      = dCollide(o1, o2, MAX_COLLIDE_RETURNS, odeResult, sizeof(odeResult[0]));

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
    result.addContact(convertContact(contactGeoms[0], b1, b2, option));
    return;
  }

  const auto available = option.maxNumContacts - result.getNumContacts();
  const auto requested = static_cast<std::size_t>(numContacts);
  const auto contactsToCopy = static_cast<int>(std::min(requested, available));

  for (auto i = 0; i < contactsToCopy; ++i) {
    result.addContact(convertContact(contactGeoms[i], b1, b2, option));
  }

  if (result.getNumContacts() >= option.maxNumContacts) {
    return;
  }

  if (!history || !shouldUseContactHistory(b1, b2)) {
    return;
  }

  const auto pair = MakeNewPair(b1, b2);
  auto& pastContacsVec = FindPairInHist(*history, pair);
  auto results_vec_copy = result.getContacts();

  bool sliding = false;
  constexpr double slidingThreshold = 1e-3;
  std::size_t pairContactCount = 0u;
  for (const auto& curr_cont : results_vec_copy) {
    const auto current_pair
        = MakeNewPair(curr_cont.collisionObject1, curr_cont.collisionObject2);
    if (current_pair != pair)
      continue;

    ++pairContactCount;

    if (computeTangentialSpeed(curr_cont) > slidingThreshold) {
      sliding = true;
      break;
    }
  }

  if (sliding) {
    return;
  }

  const std::size_t pairTarget
      = std::min<std::size_t>(3u, option.maxNumContacts);
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
    for (const auto& curr_cont : results_vec_copy) {
      const auto res_pair
          = MakeNewPair(curr_cont.collisionObject1, curr_cont.collisionObject2);
      if (res_pair != pair) {
        continue;
      }
      auto dist_v = past_cont.point - curr_cont.point;
      const auto dist_m = (dist_v.transpose() * dist_v).coeff(0, 0);
      if (dist_m < 0.01) {
        continue;
      }
      if (result.getNumContacts() >= option.maxNumContacts) {
        return;
      }
      result.addContact(past_cont);
      if (--missing == 0u)
        break;
    }
    if (missing == 0u) {
      break;
    }
  }
  for (const auto& item : results_vec_copy) {
    const auto res_pair
        = MakeNewPair(item.collisionObject1, item.collisionObject2);
    if (res_pair == pair) {
      pastContacsVec.push_back(item);
    }
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

  contact.collisionObject1 = b1;
  contact.collisionObject2 = b2;

  if (option.enableContact) {
    contact.point = OdeTypes::convertVector3(odeContact.pos);
    contact.normal = OdeTypes::convertVector3(odeContact.normal);
    contact.penetrationDepth = odeContact.depth;
  }

  return contact;
}

double computeTangentialSpeed(const Contact& contact)
{
  const auto* frame1 = contact.collisionObject1
                           ? contact.collisionObject1->getShapeFrame()
                           : nullptr;
  const auto* frame2 = contact.collisionObject2
                           ? contact.collisionObject2->getShapeFrame()
                           : nullptr;

  const dynamics::BodyNode* bn1 = (frame1 && frame1->isShapeNode())
                                      ? frame1->asShapeNode()->getBodyNodePtr()
                                      : nullptr;
  const dynamics::BodyNode* bn2 = (frame2 && frame2->isShapeNode())
                                      ? frame2->asShapeNode()->getBodyNodePtr()
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
  // Persist contacts for any shape pair so resting contacts stay stable across
  // detector runs. The sliding/tangential-speed checks later will filter cases
  // where the cache should not be reused.
  return object1 != nullptr && object2 != nullptr;
}

} // anonymous namespace

//==============================================================================
void OdeCollisionDetector::pruneContactHistory(const CollisionResult& result)
{
  if (mContactHistory.empty())
    return;

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
