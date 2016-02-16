/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/collision/fcl_mesh/FCLMeshEngine.h"

#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/collision_data.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>

#include "dart/collision/CollisionObject.h"
#include "dart/collision/fcl/FCLTypes.h"
#include "dart/collision/fcl/FCLCollisionNode.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionNode.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionObjectEngineData.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionGroupEngineData.h"

namespace dart {
namespace collision {

namespace {

// Collision data stores the collision request and the result given by
// collision algorithm.
struct CollisionData
{
  // Collision request
  fcl::CollisionRequest request;

  // Collision result
  fcl::CollisionResult result;

  // FCL collision detector
//  FCLMeshEngine* collisionDetector;

  // Whether the collision iteration can stop
  bool done;

  bool checkAllCollisions;

  CollisionData();
};

bool collisionCallBack(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata);

void postProcess(const fcl::CollisionResult& fclResult, Result& result);

int evalContactPosition(
    const fcl::Contact& fclContact,
    const fcl::BVHModel<fcl::OBBRSS>* mesh1,
    const fcl::BVHModel<fcl::OBBRSS>* mesh2,
    const fcl::Transform3f& transform1,
    const fcl::Transform3f& transform2,
    Eigen::Vector3d* contactPosition1,
    Eigen::Vector3d* contactPosition2);

int FFtest(
    const fcl::Vec3f& r1, const fcl::Vec3f& r2, const fcl::Vec3f& r3,
    const fcl::Vec3f& R1, const fcl::Vec3f& R2, const fcl::Vec3f& R3,
    fcl::Vec3f* res1, fcl::Vec3f* res2);

double triArea(fcl::Vec3f& p1, fcl::Vec3f& p2, fcl::Vec3f& p3);

void convert(const Option& option, fcl::CollisionRequest& fclRequest);

void convert(const fcl::CollisionResult& fclResult, Result& result);

} // anonymous namespace



//==============================================================================
const std::string& FCLMeshEngine::getType() const
{
  return getTypeStatic();
}

//==============================================================================
FCLMeshEnginePtr FCLMeshEngine::create()
{
  return FCLMeshEnginePtr(new FCLMeshEngine());
}

//==============================================================================
const std::string& FCLMeshEngine::getTypeStatic()
{
  static const std::string& type("FCL");
  return type;
}

//==============================================================================
CollisionObjectEngineDataPtr FCLMeshEngine::createCollisionObjectData(
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
{
  return std::make_shared<FCLMeshCollisionObjectEngineData>(parent, shape);
}

//==============================================================================
CollisionGroupEngineDataPtr FCLMeshEngine::createCollisionGroupData(
    const CollisionObjects& collObjects)
{
  return std::make_shared<FCLMeshCollisionGroupEngineData>(collObjects);
}

//==============================================================================
bool FCLMeshEngine::detect(CollisionObject* object1,
                           CollisionObject* object2,
                           const Option& option,
                           Result& result)
{
  result.contacts.clear();

  assert(object1->getEngine()->getType() == FCLMeshEngine::getTypeStatic());
  assert(object2->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  object1->updateEngineData();
  object2->updateEngineData();

  auto data1 = static_cast<FCLMeshCollisionObjectEngineData*>(object1->getEngineData());
  auto data2 = static_cast<FCLMeshCollisionObjectEngineData*>(object2->getEngineData());

  auto fclCollObj1 = data1->getFCLCollisionObject();
  auto fclCollObj2 = data2->getFCLCollisionObject();

  CollisionData collData;
  convert(option, collData.request);

  collisionCallBack(fclCollObj1, fclCollObj2, &collData);

  convert(collData.result, result);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLMeshEngine::detect(CollisionObject* object, CollisionGroup* group,
                           const Option& option, Result& result)
{
  result.contacts.clear();

  assert(object);
  assert(group);
  assert(object->getEngine()->getType() == FCLMeshEngine::getTypeStatic());
  assert(group->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  object->updateEngineData();
  group->updateEngineData();

  auto objData = static_cast<FCLMeshCollisionObjectEngineData*>(object->getEngineData());
  auto groupData = static_cast<FCLMeshCollisionGroupEngineData*>(group->getEngineData());

  auto fclObject = objData->getFCLCollisionObject();
  auto broadPhaseAlg = groupData->getFCLCollisionManager();

  CollisionData collData;
  convert(option, collData.request);

  broadPhaseAlg->collide(fclObject, &collData, collisionCallBack);

  convert(collData.result, result);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLMeshEngine::detect(CollisionGroup* group,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(group);
  assert(group->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  group->updateEngineData();

  auto data = static_cast<FCLMeshCollisionGroupEngineData*>(group->getEngineData());

  auto broadPhaseAlg = data->getFCLCollisionManager();

  CollisionData collData;
  convert(option, collData.request);

  broadPhaseAlg->collide(&collData, collisionCallBack);

  convert(collData.result, result);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLMeshEngine::detect(CollisionGroup* group1, CollisionGroup* group2,
                       const Option& option, Result& result)
{
  result.contacts.clear();

  assert(group1);
  assert(group2);
  assert(group1->getEngine()->getType() == FCLMeshEngine::getTypeStatic());
  assert(group2->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  group1->updateEngineData();
  group2->updateEngineData();

  auto data1 = static_cast<FCLMeshCollisionGroupEngineData*>(group1->getEngineData());
  auto data2 = static_cast<FCLMeshCollisionGroupEngineData*>(group2->getEngineData());

  auto broadPhaseAlg1 = data1->getFCLCollisionManager();
  auto broadPhaseAlg2 = data2->getFCLCollisionManager();

  CollisionData collData;
  convert(option, collData.request);

  broadPhaseAlg1->collide(broadPhaseAlg2, &collData, collisionCallBack);

  convert(collData.result, result);

  return !result.contacts.empty();
}



namespace {

//==============================================================================
CollisionData::CollisionData()
{
  done = false;
}

//==============================================================================
bool collisionCallBack(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata)
{
  CollisionData* collData = static_cast<CollisionData*>(cdata);

  const fcl::CollisionRequest& request = collData->request;
        fcl::CollisionResult&  result  = collData->result;
//  FCLMeshEngine*        cd      = collData->collisionDetector;
  // TODO(JS): take filter object instead of collision detector

  if (collData->done)
    return true;

  // Filtering
//  if (!cd->isCollidable(cd->findCollisionNode(o1), cd->findCollisionNode(o2)))
//    return collData->done;
  // TODO(JS): disabled until other functionalities are implemented

  // Perform narrow-phase detection
  fcl::collide(o1, o2, request, result);

  if (!request.enable_cost
      && (result.isCollision())
      && ((result.numContacts() >= request.num_max_contacts)
          || !collData->checkAllCollisions))
  {
    collData->done = true;
  }

  return collData->done;
}

//==============================================================================
void postProcess(const fcl::CollisionResult& fclResult, Result& result)
{
  result.contacts.clear();

  int numCoplanarContacts = 0;
  int numNoContacts = 0;
  int numContacts = 0;

  std::vector<Contact> unfiltered;
  unfiltered.reserve(fclResult.numContacts());

  for (auto k = 0u; k < fclResult.numContacts(); ++k)
  {
    // for each pair of intersecting triangles, we create two contact points
    Contact pair1, pair2;
    const fcl::Contact& c = fclResult.getContact(k);

    FCLCollisionGeometryUserData* userData1
        = static_cast<FCLCollisionGeometryUserData*>(c.o1->getUserData());
    FCLCollisionGeometryUserData* userData2
        = static_cast<FCLCollisionGeometryUserData*>(c.o2->getUserData());
    assert(userData1);
    assert(userData2);

    auto collisionObject1 = userData1->mCollisionObject;
    auto collisionObject2 = userData2->mCollisionObject;

    int contactResult = evalContactPosition(
        c,
        static_cast<const fcl::BVHModel<fcl::OBBRSS>*>(c.o1),
        static_cast<const fcl::BVHModel<fcl::OBBRSS>*>(c.o2),
        FCLTypes::convertTransform(collisionObject1->getTransform()),
        FCLTypes::convertTransform(collisionObject2->getTransform()),
        &pair1.point, &pair2.point);

    if (contactResult == COPLANAR_CONTACT)
    {
      numCoplanarContacts++;

      if (numContacts > 2)
        continue;
    }
    else if (contactResult == NO_CONTACT)
    {
      numNoContacts++;

      continue;
    }
    else
    {
      numContacts++;
    }

    pair1.normal = FCLTypes::convertVector3(-fclResult.getContact(k).normal);
    pair2.normal = pair1.normal;
    pair1.penetrationDepth = c.penetration_depth;
    pair1.triID1 = c.b1;
    pair1.triID2 = c.b2;

    pair1.shape1 = userData1->mShape;
    pair1.shape2 = userData2->mShape;
    pair1.collisionObject1 = collisionObject1;
    pair1.collisionObject2 = collisionObject2;

    pair2 = pair1;
    unfiltered.push_back(pair1);
    unfiltered.push_back(pair2);
  }

  const double ZERO = 0.000001;
  const double ZERO2 = ZERO*ZERO;

  std::vector<bool> markForDeletion(unfiltered.size(), false);

  // mark all the repeated points
  for (unsigned int k = 0; k < unfiltered.size(); k++)
  {
    for (unsigned int l = k + 1; l < unfiltered.size(); l++)
    {
      const Eigen::Vector3d diff = unfiltered[k].point - unfiltered[l].point;

      if (diff.dot(diff) < 3 * ZERO2)
      {
        markForDeletion[k] = true;
        break;
      }
    }
  }

  // remove all the co-linear contact points
  for (auto k = 0u; k < unfiltered.size(); ++k)
  {
    if (markForDeletion[k])
      continue;

    for (auto l = 0u; l < unfiltered.size(); ++l)
    {
      if (l == k || markForDeletion[l])
        continue;

      if (markForDeletion[k])
        break;

      for (auto m = l + 1u; m < unfiltered.size(); ++m)
      {
        if (k == m)
          continue;

        const Eigen::Vector3d va = unfiltered[k].point - unfiltered[l].point;
        const Eigen::Vector3d vb = unfiltered[k].point - unfiltered[m].point;
        const Eigen::Vector3d v = va.cross(vb);

        if (v.dot(v) < ZERO2 && va.dot(vb) < 0)
        {
          markForDeletion[k] = true;
          break;
        }
      }
    }
  }

  for (size_t k = 0; k < unfiltered.size(); k++)
  {
    if (!markForDeletion[k])
      result.contacts.push_back(unfiltered[k]);
  }
}

//==============================================================================
int evalContactPosition(
    const fcl::Contact& fclContact,
    const fcl::BVHModel<fcl::OBBRSS>* mesh1,
    const fcl::BVHModel<fcl::OBBRSS>* mesh2,
    const fcl::Transform3f& transform1,
    const fcl::Transform3f& transform2,
    Eigen::Vector3d* contactPosition1,
    Eigen::Vector3d* contactPosition2)
{
  int id1 = fclContact.b1;
  int id2 = fclContact.b2;
  fcl::Triangle tri1 = mesh1->tri_indices[id1];
  fcl::Triangle tri2 = mesh2->tri_indices[id2];

  fcl::Vec3f v1, v2, v3, p1, p2, p3;
  v1 = mesh1->vertices[tri1[0]];
  v2 = mesh1->vertices[tri1[1]];
  v3 = mesh1->vertices[tri1[2]];

  p1 = mesh2->vertices[tri2[0]];
  p2 = mesh2->vertices[tri2[1]];
  p3 = mesh2->vertices[tri2[2]];

  fcl::Vec3f contact1, contact2;
  v1 = transform1.transform(v1);
  v2 = transform1.transform(v2);
  v3 = transform1.transform(v3);
  p1 = transform2.transform(p1);
  p2 = transform2.transform(p2);
  p3 = transform2.transform(p3);
  int testRes = FFtest(v1, v2, v3, p1, p2, p3, &contact1, &contact2);

  if (testRes == COPLANAR_CONTACT)
  {
    double area1 = triArea(v1, v2, v3);
    double area2 = triArea(p1, p2, p3);

    if (area1 < area2)
      contact1 = v1 + v2 + v3;
    else
      contact1 = p1 + p2 + p3;

    contact1[0] /= 3.0;
    contact1[1] /= 3.0;
    contact1[2] /= 3.0;
    contact2 = contact1;
  }

  *contactPosition1 = Eigen::Vector3d(contact1[0], contact1[1], contact1[2]);
  *contactPosition2 = Eigen::Vector3d(contact2[0], contact2[1], contact2[2]);

  return testRes;
}

//==============================================================================
int FFtest(
    const fcl::Vec3f& r1, const fcl::Vec3f& r2, const fcl::Vec3f& r3,
    const fcl::Vec3f& R1, const fcl::Vec3f& R2, const fcl::Vec3f& R3,
    fcl::Vec3f* res1, fcl::Vec3f* res2)
{
  float U0[3], U1[3], U2[3], V0[3], V1[3], V2[3], RES1[3], RES2[3];
  SET(U0, r1);
  SET(U1, r2);
  SET(U2, r3);
  SET(V0, R1);
  SET(V1, R2);
  SET(V2, R3);

  int contactResult = tri_tri_intersect(V0, V1, V2, U0, U1, U2, RES1, RES2);

  SET((*res1), RES1);
  SET((*res2), RES2);

  return contactResult;
}

//==============================================================================
double triArea(fcl::Vec3f& p1, fcl::Vec3f& p2, fcl::Vec3f& p3)
{
  fcl::Vec3f a = p2 - p1;
  fcl::Vec3f b = p3 - p1;
  double aMag = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
  double bMag = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
  double dp = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  double area =  0.5 * sqrt(aMag * bMag - dp * dp);

  return area;
}

//==============================================================================
void convert(const Option& fclResult, fcl::CollisionRequest& fclRequest)
{
  fclRequest.num_max_contacts = fclResult.maxNumContacts;
  fclRequest.enable_contact   = fclResult.enableContact;
#if FCL_VERSION_AT_LEAST(0,3,0)
  fclRequest.gjk_solver_type  = fcl::GST_LIBCCD;
#endif
}

//==============================================================================
void convert(const fcl::CollisionResult& fclResult, Result& result)
{
  postProcess(fclResult, result);
}

} // anonymous namespace

} // namespace collision
} // namespace dart
