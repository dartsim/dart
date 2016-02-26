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
#include "dart/collision/fcl/FCLCollisionObjectData.h"
#include "dart/collision/fcl_mesh/tri_tri_intersection_test.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionObjectData.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionGroupData.h"

namespace dart {
namespace collision {

namespace {

bool checkPair(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata);

void postProcess(const fcl::CollisionResult& fclResult,
                 fcl::CollisionObject* o1,
                 fcl::CollisionObject* o2,
                 Result& result);

Contact convertContact(const fcl::Contact& fclContact,
                       fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2);

void convertOption(const Option& option, fcl::CollisionRequest& fclRequest);

/// Collision data stores the collision request and the result given by
/// collision algorithm.
struct FCLCollisionData
{
  /// FCL collision request
  fcl::CollisionRequest mFclRequest;

  /// FCL collision result
  fcl::CollisionResult mFclResult;

  /// Collision option of DART
  const Option* mOption;

  /// Collision result of DART
  Result* mResult;

  /// Whether the collision iteration can stop
  bool done;

  /// Constructor
  FCLCollisionData(
      const Option* option = nullptr,
      Result* result = nullptr)
    : mOption(option),
      mResult(result),
      done(false)
  {
    if (mOption)
      convertOption(*mOption, mFclRequest);
  }
};

int evalContactPosition(
    const fcl::Contact& fclContact,
    const fcl::BVHModel<fcl::OBBRSS>* mesh1,
    const fcl::BVHModel<fcl::OBBRSS>* mesh2,
    const fcl::Transform3f& transform1,
    const fcl::Transform3f& transform2,
    Eigen::Vector3d* contactPosition1,
    Eigen::Vector3d* contactPosition2);

bool isColinear(const Eigen::Vector3d& pos1,
                const Eigen::Vector3d& pos2,
                const Eigen::Vector3d& pos3,
                double tol);

int FFtest(
    const fcl::Vec3f& r1, const fcl::Vec3f& r2, const fcl::Vec3f& r3,
    const fcl::Vec3f& R1, const fcl::Vec3f& R2, const fcl::Vec3f& R3,
    fcl::Vec3f* res1, fcl::Vec3f* res2);

double triArea(fcl::Vec3f& p1, fcl::Vec3f& p2, fcl::Vec3f& p3);

} // anonymous namespace



//==============================================================================
FCLMeshEnginePtr FCLMeshEngine::create()
{
  return FCLMeshEnginePtr(new FCLMeshEngine());
}

//==============================================================================
FCLMeshEngine::FCLMeshEngine()
{
  // Do nothing
}

//==============================================================================
FCLMeshEngine::~FCLMeshEngine()
{
  // Do nothing
}

//==============================================================================
const std::string& FCLMeshEngine::getType() const
{
  return getTypeStatic();
}

//==============================================================================
const std::string& FCLMeshEngine::getTypeStatic()
{
  static const std::string& type("FCLMesh");
  return type;
}

//==============================================================================
std::unique_ptr<CollisionObjectData>
FCLMeshEngine::createCollisionObjectData(
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
{
  return std::unique_ptr<CollisionObjectData>(
        new FCLMeshCollisionObjectData(this, parent, shape));
}

//==============================================================================
std::unique_ptr<CollisionGroupData>
FCLMeshEngine::createCollisionGroupData(
    CollisionGroup* parent,
    const CollisionObjectPtrs& collObjects)
{
  return std::unique_ptr<CollisionGroupData>(
        new FCLMeshCollisionGroupData(this, parent, collObjects));
}

//==============================================================================
bool FCLMeshEngine::detect(
    CollisionObjectData* objectData1,
    CollisionObjectData* objectData2,
    const Option& option, Result& result)
{
  result.contacts.clear();

  assert(objectData1->getEngine()->getType() == FCLMeshEngine::getTypeStatic());
  assert(objectData2->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  auto castedData1 = static_cast<const FCLMeshCollisionObjectData*>(objectData1);
  auto castedData2 = static_cast<const FCLMeshCollisionObjectData*>(objectData2);

  auto fclCollObj1 = castedData1->getFCLCollisionObject();
  auto fclCollObj2 = castedData2->getFCLCollisionObject();

  FCLCollisionData collData(&option, &result);
  checkPair(fclCollObj1, fclCollObj2, &collData);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLMeshEngine::detect(
    CollisionObjectData* objectData,
    CollisionGroupData* groupData,
    const Option& option, Result& result)
{
  result.contacts.clear();

  assert(objectData);
  assert(groupData);
  assert(objectData->getEngine()->getType() == FCLMeshEngine::getTypeStatic());
  assert(groupData->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  auto castedObjData = static_cast<FCLMeshCollisionObjectData*>(objectData);
  auto castedGrpData = static_cast<FCLMeshCollisionGroupData*>(groupData);

  auto fclObject = castedObjData->getFCLCollisionObject();
  auto broadPhaseAlg = castedGrpData->getFCLCollisionManager();

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg->collide(fclObject, &collData, checkPair);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLMeshEngine::detect(CollisionGroupData* groupData,
                           const Option& option, Result& result)
{
  result.contacts.clear();

  assert(groupData);
  assert(groupData->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  auto castedData = static_cast<FCLMeshCollisionGroupData*>(groupData);

  auto broadPhaseAlg = castedData->getFCLCollisionManager();

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg->collide(&collData, checkPair);

  return !result.contacts.empty();
}

//==============================================================================
bool FCLMeshEngine::detect(CollisionGroupData* groupData1,
                           CollisionGroupData* groupData2,
                           const Option& option, Result& result)
{
  result.contacts.clear();

  assert(groupData1);
  assert(groupData2);
  assert(groupData1->getEngine()->getType() == FCLMeshEngine::getTypeStatic());
  assert(groupData2->getEngine()->getType() == FCLMeshEngine::getTypeStatic());

  auto castedData1 = static_cast<FCLMeshCollisionGroupData*>(groupData1);
  auto castedData2 = static_cast<FCLMeshCollisionGroupData*>(groupData2);

  auto broadPhaseAlg1 = castedData1->getFCLCollisionManager();
  auto broadPhaseAlg2 = castedData2->getFCLCollisionManager();

  FCLCollisionData collData(&option, &result);
  broadPhaseAlg1->collide(broadPhaseAlg2, &collData, checkPair);

  return !result.contacts.empty();
}



namespace {

//==============================================================================
bool checkPair(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* cdata)
{
  FCLCollisionData* collData = static_cast<FCLCollisionData*>(cdata);

  const fcl::CollisionRequest& fclRequest = collData->mFclRequest;
        fcl::CollisionResult&  fclResult  = collData->mFclResult;
        Result&                result     = *(collData->mResult);
//  FCLEngine*        cd      = collData->collisionDetector;
  // TODO(JS): take filter object instead of collision detector

  if (collData->done)
    return true;

  // Filtering
//  if (!cd->isCollidable(cd->findCollisionNode(o1), cd->findCollisionNode(o2)))
//    return collData->done;
  // TODO(JS): disabled until other functionalities are implemented

  // Clear previous results
  fclResult.clear();

  // Perform narrow-phase detection
  fcl::collide(o1, o2, fclRequest, fclResult);

  if (!fclRequest.enable_cost
      && (fclResult.isCollision())
      && ((fclResult.numContacts() >= fclRequest.num_max_contacts)))
          //|| !collData->checkAllCollisions))
    // TODO(JS): checkAllCollisions should be in FCLCollisionData
  {
    collData->done = true;
  }

  postProcess(fclResult, o1, o2, result);

  return collData->done;
}

//==============================================================================
void postProcess(const fcl::CollisionResult& fclResult,
                 fcl::CollisionObject* o1,
                 fcl::CollisionObject* o2,
                 Result& result)
{
  auto initNumContacts = fclResult.numContacts();

  if (0 == initNumContacts)
    return;

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

    FCLCollisionObjectUserData* userData1
        = static_cast<FCLCollisionObjectUserData*>(o1->getUserData());
    FCLCollisionObjectUserData* userData2
        = static_cast<FCLCollisionObjectUserData*>(o2->getUserData());
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

    pair1.collisionObject1 = collisionObject1;
    pair1.collisionObject2 = collisionObject2;

    pair2 = pair1;
    unfiltered.push_back(pair1);
    unfiltered.push_back(pair2);
  }





  const double tol = 1e-12;

  auto unfilteredSize = unfiltered.size();

  std::vector<bool> markForDeletion(unfilteredSize, false);

  // mark all the repeated points
  for (auto i = 0u; i < unfilteredSize - 1u; ++i)
  {
    const auto& contact1 = unfiltered[i];

    for (auto j = i + 1u; j < unfilteredSize; ++j)
    {
      const auto& contact2 = unfiltered[j];

      const auto diff = contact1.point - contact2.point;

      if (diff.norm() < 3.0 * tol)
      {
        markForDeletion[i] = true;
        break;
      }
    }
  }

  // remove all the co-linear contact points
//  for (auto i = 0u; i < initNumContacts; ++i)
//  {
//    if (markForDeletion[i])
//      continue;

//    const auto& contact1 = unfiltered[i];

//    for (auto j = i + 1u; j < initNumContacts; ++j)
//    {
//      if (markForDeletion[j])
//        continue;

//      const auto& contact2 = unfiltered[j];

//      for (auto k = j + 1u; k < initNumContacts; ++k)
//      {
//        if (markForDeletion[k])
//          continue;

//        const auto& contact3 = unfiltered[k];

//        if (isColinear(contact1.point, contact2.point, contact3.point, tol))
//        {
//          markForDeletion[i] = true;
//          break;
//        }
//      }
//    }
//  }

  for (size_t i = 0; i < initNumContacts; ++i)
  {
//    if (!markForDeletion[i])
    {
      const auto& contact = unfiltered[i];
      result.contacts.push_back(contact);
    }
  }




//  for (auto i = 0u; i < numContacts; ++i)
//  {
//    const auto fclContact = fclResult.getContact(i);
//    result.contacts.push_back(convertContact(fclContact, o1, o2));
//  }



  std::cout << "=================" << std::endl;
  std::cout << "# of contacts:" << result.contacts.size() << std::endl;
  std::cout << "=================" << std::endl;
  for (auto i = 0u; i < result.contacts.size(); ++i)
  {
    auto contact = result.contacts[i];

    std::cout << "Contact (" << i << ")" << std::endl;
    std::cout << "Point : " << contact.point.transpose() << std::endl;
    std::cout << "Normal: " << contact.normal.transpose() << std::endl;
    std::cout << "Depth : " << contact.penetrationDepth << std::endl;
    std::cout << std::endl;
  }
  std::cout << "=================" << std::endl;

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
bool isColinear(const Eigen::Vector3d& pos1,
                const Eigen::Vector3d& pos2,
                const Eigen::Vector3d& pos3,
                double tol)
{
  const auto va = pos1 - pos2;
  const auto vb = pos1 - pos3;
  const auto v = va.cross(vb);

  return v.norm() < tol;
}


//==============================================================================
void convertOption(const Option& fclOption, fcl::CollisionRequest& request)
{
  request.num_max_contacts = fclOption.maxNumContacts;
  request.enable_contact   = fclOption.enableContact;
#if FCL_VERSION_AT_LEAST(0,3,0)
  request.gjk_solver_type  = fcl::GST_LIBCCD;
#endif
}

//==============================================================================
Contact convertContact(const fcl::Contact& fclContact,
                       fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2)
{
  Contact contact;

  Eigen::Vector3d point = FCLTypes::convertVector3(fclContact.pos);

  contact.point = point;
  contact.normal = -FCLTypes::convertVector3(fclContact.normal);
  contact.penetrationDepth = fclContact.penetration_depth;
  contact.triID1 = fclContact.b1;
  contact.triID2 = fclContact.b2;

  FCLCollisionObjectUserData* userData1
      = static_cast<FCLCollisionObjectUserData*>(o1->getUserData());
  FCLCollisionObjectUserData* userData2
      = static_cast<FCLCollisionObjectUserData*>(o2->getUserData());
  assert(userData1);
  assert(userData2);
  contact.collisionObject1 = userData1->mCollisionObject;
  contact.collisionObject2 = userData2->mCollisionObject;

  return contact;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
