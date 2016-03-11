/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/collision/bullet/BulletCollisionDetector.h"

#include <iostream>

#include "dart/common/Console.h"
#include "dart/collision/CollisionObject.h"
#include "dart/collision/bullet/BulletTypes.h"
#include "dart/collision/bullet/BulletCollisionObjectData.h"
#include "dart/collision/bullet/BulletCollisionGroupData.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"

namespace dart {
namespace collision {

namespace {

struct BulletOverlapFilterCallback : public btOverlapFilterCallback
{
  BulletOverlapFilterCallback(CollisionFilter* filter)
    : mFilter(filter)
  {
    // Do nothing
  }

  // return true when pairs need collision
  bool needBroadphaseCollision(btBroadphaseProxy* proxy0,
                               btBroadphaseProxy* proxy1) const override
  {
    assert((proxy0 != nullptr && proxy1 != nullptr) &&
           "Bullet broadphase overlapping pair proxies are nullptr");

    bool collide = (proxy0->m_collisionFilterGroup &
                    proxy1->m_collisionFilterMask) != 0;
    collide = collide && (proxy1->m_collisionFilterGroup &
                          proxy0->m_collisionFilterMask);

    if (collide && mFilter)
    {
      auto bulletCollObj0
          = static_cast<btCollisionObject*>(proxy0->m_clientObject);
      auto bulletCollObj1
          = static_cast<btCollisionObject*>(proxy1->m_clientObject);

      auto userData0 = static_cast<BulletCollisionObjectData::UserData*>(
            bulletCollObj0->getUserPointer());
      auto userData1 = static_cast<BulletCollisionObjectData::UserData*>(
            bulletCollObj1->getUserPointer());

      auto collisionDetector = userData0->collisionDetector;
      assert(collisionDetector == userData1->collisionDetector);

      auto castedCD = static_cast<BulletCollisionDetector*>(collisionDetector);

      auto collObj0 = castedCD->findCollisionObject(bulletCollObj0);
      auto collObj1 = castedCD->findCollisionObject(bulletCollObj1);

      collide = mFilter->needCollision(collObj0, collObj1);
    }

    return collide;
  }

  CollisionFilter* mFilter;
};

struct BulletContactResultCallback : btCollisionWorld::ContactResultCallback
{
  BulletContactResultCallback(Result& result);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;

  Result& mResult;
};

} // anonymous namespace



//==============================================================================
std::shared_ptr<BulletCollisionDetector> BulletCollisionDetector::create()
{
  return std::shared_ptr<BulletCollisionDetector>(
        new BulletCollisionDetector());
}

//==============================================================================
BulletCollisionDetector::~BulletCollisionDetector()
{
  assert(mShapeMap.empty());
}

//==============================================================================
const std::string& BulletCollisionDetector::getTypeStatic()
{
  static const std::string& type("Bullet");
  return type;
}

//==============================================================================
const std::string& BulletCollisionDetector::getType() const
{
  return getTypeStatic();
}

//==============================================================================
BulletCollisionObjectData* BulletCollisionDetector::findCollisionObjectData(
    btCollisionObject* bulletCollObj) const
{
  auto search = mCollisionObjectMap.find(bulletCollObj);
  if (mCollisionObjectMap.end() != search)
    return search->second;
  else
    return nullptr;
}

//==============================================================================
CollisionObject* BulletCollisionDetector::findCollisionObject(
    btCollisionObject* bulletCollObj) const
{
  auto data = findCollisionObjectData(bulletCollObj);

  if (data)
    return data->getCollisionObject();
  else
    return nullptr;
}

//==============================================================================
std::unique_ptr<CollisionObjectData>
BulletCollisionDetector::createCollisionObjectData(
    CollisionObject* parent, const dynamics::ShapePtr& shape)
{
  btCollisionShape* bulletCollGeom;

  auto findResult = mShapeMap.find(shape);
  if (mShapeMap.end() != findResult)
  {
    ShapeMapValue& pair = findResult->second;
    BulletCollsionPack& pack = pair.first;
    bulletCollGeom = pack.collisionShape.get();
    pair.second++;
  }
  else
  {
    BulletCollsionPack pack = createBulletCollisionShape(shape);
    bulletCollGeom = pack.collisionShape.get();
    mShapeMap[shape] = std::make_pair(std::move(pack), 1u);
  }

  return std::unique_ptr<CollisionObjectData>(
        new BulletCollisionObjectData(this, parent, bulletCollGeom));
}

//==============================================================================
void BulletCollisionDetector::reclaimCollisionObjectData(
    CollisionObjectData* collisionObjectData)
{
  // Retrieve associated shape
  auto shape = collisionObjectData->getCollisionObject()->getShape();
  assert(shape);

  auto findResult = mShapeMap.find(shape);
  assert(mShapeMap.end() != findResult);

  ShapeMapValue& pair = findResult->second;
  pair.second--;

  if (0u == pair.second)
  {
    mShapeMap.erase(findResult);
  }
}

//==============================================================================
std::unique_ptr<CollisionGroupData>
BulletCollisionDetector::createCollisionGroupData(
    CollisionGroup* parent,
    const CollisionObjectPtrs& collObjects)
{
  return std::unique_ptr<CollisionGroupData>(
        new BulletCollisionGroupData(this, parent, collObjects));
}

//==============================================================================
Contact convertContact(const btManifoldPoint& bulletManifoldPoint,
                       const BulletCollisionObjectUserData* userData1,
                       const BulletCollisionObjectUserData* userData2)
{
  assert(userData1);
  assert(userData2);

  Contact contact;

  contact.point = convertVector3(bulletManifoldPoint.getPositionWorldOnA());
  contact.normal = convertVector3(bulletManifoldPoint.m_normalWorldOnB);
  contact.penetrationDepth = -bulletManifoldPoint.m_distance1;
  contact.collisionObject1 = userData1->collisionObject;
  contact.collisionObject2 = userData2->collisionObject;

  return contact;
}

//==============================================================================
void convertContacts(btCollisionWorld* collWorld, Result& result)
{
  assert(collWorld);

  auto dispatcher = collWorld->getDispatcher();
  assert(dispatcher);

  auto numManifolds = dispatcher->getNumManifolds();

  for (auto i = 0; i < numManifolds; ++i)
  {
    auto contactManifold = dispatcher->getManifoldByIndexInternal(i);
    const auto bulletCollObj0 = contactManifold->getBody0();
    const auto bulletCollObj1 = contactManifold->getBody1();

    auto userPointer0 = bulletCollObj0->getUserPointer();
    auto userPointer1 = bulletCollObj1->getUserPointer();

    auto userDataA = static_cast<BulletCollisionObjectUserData*>(userPointer1);
    auto userDataB = static_cast<BulletCollisionObjectUserData*>(userPointer0);

    auto numContacts = contactManifold->getNumContacts();

    for (auto j = 0; j < numContacts; ++j)
    {
      auto& cp = contactManifold->getContactPoint(j);

      result.contacts.push_back(convertContact(cp, userDataA, userDataB));
    }
  }
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionObjectData* objectData1,
    CollisionObjectData* objectData2,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(objectData1->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());
  assert(objectData2->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());

  auto castedData1 = static_cast<const BulletCollisionObjectData*>(objectData1);
  auto castedData2 = static_cast<const BulletCollisionObjectData*>(objectData2);

  auto bulletCollObj1 = castedData1->getBulletCollisionObject();
  auto bulletCollObj2 = castedData2->getBulletCollisionObject();

  if (!mBulletCollisionGroupForSinglePair)
    mBulletCollisionGroupForSinglePair = createCollisionGroup();

  auto cb = BulletContactResultCallback(result);
  auto collisionGroupData = static_cast<BulletCollisionGroupData*>(
        mBulletCollisionGroupForSinglePair->getEngineData());
  auto bulletCollisionWorld = collisionGroupData->getBulletCollisionWorld();
  bulletCollisionWorld->contactPairTest(bulletCollObj1, bulletCollObj2, cb);

  return !result.contacts.empty();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionObjectData* objectData,
    CollisionGroupData* groupData,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(objectData);
  assert(groupData);
  assert(objectData->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());
  assert(groupData->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());

  auto castedObjData = static_cast<BulletCollisionObjectData*>(objectData);
  auto castedGrpData = static_cast<BulletCollisionGroupData*>(groupData);

  auto bulletCollObj = castedObjData->getBulletCollisionObject();
  auto bulletCollisionWorld = castedGrpData->getBulletCollisionWorld();

  auto cb = BulletContactResultCallback(result);
  bulletCollisionWorld->contactTest(bulletCollObj, cb);

  return !result.contacts.empty();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionGroupData* groupData,
    const Option& option, Result& result)
{
  result.contacts.clear();

  assert(groupData);
  assert(groupData->getCollisionDetector()->getType()
         == BulletCollisionDetector::getTypeStatic());

  auto castedData = static_cast<BulletCollisionGroupData*>(groupData);

  auto bulletCollisionWorld = castedData->getBulletCollisionWorld();
  auto bulletPairCache = bulletCollisionWorld->getPairCache();

  auto filterCallback
      = new BulletOverlapFilterCallback(option.collisionFilter.get());
  bulletPairCache->setOverlapFilterCallback(filterCallback);

  bulletCollisionWorld->performDiscreteCollisionDetection();

  convertContacts(bulletCollisionWorld, result);

  return !result.contacts.empty();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionGroupData* /*groupData1*/,
    CollisionGroupData* /*groupData2*/,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

//  assert(groupData1);
//  assert(groupData2);
//  assert(groupData1->getCollisionDetector()->getType() == BulletCollisionDetector::getTypeStatic());
//  assert(groupData2->getCollisionDetector()->getType() == BulletCollisionDetector::getTypeStatic());

//  auto castedData1 = static_cast<BulletCollisionGroupData*>(groupData1);
//  auto castedData2 = static_cast<BulletCollisionGroupData*>(groupData2);

//  auto bulletCollisionWorld1 = castedData1->getBulletCollisionWorld();
//  auto bulletCollisionWorld2 = castedData2->getBulletCollisionWorld();

//  BulletCollisionData collData(&option, &result);
//  bulletCollisionWorld1->collide(bulletCollisionWorld2, &collData, checkPair);

  return !result.contacts.empty();
}

//==============================================================================
BulletCollisionDetector::BulletCollsionPack
BulletCollisionDetector::createMesh(
    const Eigen::Vector3d& scale, const aiScene* mesh)
{
  BulletCollsionPack pack;
  pack.triMesh.reset(new btTriangleMesh());

  for (unsigned int i = 0; i < mesh->mNumMeshes; i++)
  {
    for (unsigned int j = 0; j < mesh->mMeshes[i]->mNumFaces; j++)
    {
      btVector3 vertices[3];
      for (unsigned int k = 0; k < 3; k++)
      {
        const aiVector3D& vertex = mesh->mMeshes[i]->mVertices[
                                   mesh->mMeshes[i]->mFaces[j].mIndices[k]];
        vertices[k] = btVector3(vertex.x * scale[0],
                                vertex.y * scale[1],
                                vertex.z * scale[2]);
      }
      pack.triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  auto gimpactMeshShape = new btGImpactMeshShape(pack.triMesh.get());
  gimpactMeshShape->updateBound();

  pack.collisionShape.reset(gimpactMeshShape);

  return pack;
}

//==============================================================================
BulletCollisionDetector::BulletCollsionPack
BulletCollisionDetector::createSoftMesh(const aiMesh* mesh)
{
  BulletCollsionPack pack;
  pack.triMesh.reset(new btTriangleMesh());

  for (auto i = 0u; i < mesh->mNumFaces; ++i)
  {
    btVector3 vertices[3];
    for (auto j = 0u; j < 3; ++j)
    {
      const aiVector3D& vertex = mesh->mVertices[mesh->mFaces[i].mIndices[j]];
      vertices[j] = btVector3(vertex.x, vertex.y, vertex.z);
    }
    pack.triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  auto gimpactMeshShape = new btGImpactMeshShape(pack.triMesh.get());
  gimpactMeshShape->updateBound();

  pack.collisionShape.reset(gimpactMeshShape);

  return pack;
}

//==============================================================================
BulletCollisionDetector::BulletCollsionPack
BulletCollisionDetector::createBulletCollisionShape(
    const dynamics::ShapePtr& shape)
{
  using dynamics::Shape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  BulletCollsionPack pack;

  auto& bulletCollisionShape = pack.collisionShape;

  switch (shape->getShapeType())
  {
    case Shape::BOX:
    {
      assert(dynamic_cast<BoxShape*>(shape.get()));

      const auto box = static_cast<const BoxShape*>(shape.get());
      const Eigen::Vector3d& size = box->getSize();

      bulletCollisionShape.reset(new btBoxShape(convertVector3(size*0.5)));

      break;
    }
    case Shape::ELLIPSOID:
    {
      assert(dynamic_cast<EllipsoidShape*>(shape.get()));

      const auto ellipsoid = static_cast<EllipsoidShape*>(shape.get());
      const Eigen::Vector3d& size = ellipsoid->getSize();

      if (ellipsoid->isSphere())
      {
        bulletCollisionShape.reset(new btSphereShape(size[0] * 0.5));
      }
      else
      {
        // TODO(JS): Add mesh for ellipsoid
      }

      break;
    }
    case Shape::CYLINDER:
    {
      assert(dynamic_cast<CylinderShape*>(shape.get()));

      const auto cylinder = static_cast<const CylinderShape*>(shape.get());
      const auto radius = cylinder->getRadius();
      const auto height = cylinder->getHeight();
      const auto size = btVector3(radius, radius, height * 0.5);

      bulletCollisionShape.reset(new btCylinderShapeZ(size));

      break;
    }
    case Shape::PLANE:
    {
      assert(dynamic_cast<PlaneShape*>(shape.get()));

      const auto plane = static_cast<PlaneShape*>(shape.get());
      const Eigen::Vector3d normal = plane->getNormal();
      const double offset = plane->getOffset();

      bulletCollisionShape.reset(new btStaticPlaneShape(
            convertVector3(normal), offset));

      break;
    }
    case Shape::MESH:
    {
      assert(dynamic_cast<MeshShape*>(shape.get()));

      const auto shapeMesh = static_cast<MeshShape*>(shape.get());
      const auto scale = shapeMesh->getScale();
      const auto mesh = shapeMesh->getMesh();

      pack = createMesh(scale, mesh);

      break;
    }
    case Shape::SOFT_MESH:
    {
      assert(dynamic_cast<SoftMeshShape*>(shape.get()));

      const auto softMeshShape = static_cast<SoftMeshShape*>(shape.get());
      const auto mesh = softMeshShape->getAssimpMesh();

      pack = createSoftMesh(mesh);

      break;
    }
    default:
    {
      dterr << "[BulletCollisionObjectData::init] "
            << "Attempting to create unsupported shape type '"
            << shape->getShapeType() << "'.\n";

      bulletCollisionShape.reset(new btSphereShape(0.1));

      break;
    }
  }

  return pack;
}



namespace {

//==============================================================================
BulletContactResultCallback::BulletContactResultCallback(Result& result)
  : ContactResultCallback(),
    mResult(result)
{
  // Do nothing
}

//==============================================================================
//bool BulletContactResultCallback::needsCollision(
//    btBroadphaseProxy* proxy0) const
//{
//  return true;
//}

//==============================================================================
btScalar BulletContactResultCallback::addSingleResult(
    btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap,
    int /*partId0*/,
    int /*index0*/,
    const btCollisionObjectWrapper* colObj1Wrap,
    int /*partId1*/,
    int /*index1*/)
{
  auto userPointer0 = colObj0Wrap->getCollisionObject()->getUserPointer();
  auto userPointer1 = colObj1Wrap->getCollisionObject()->getUserPointer();

  auto userDataA = static_cast<BulletCollisionObjectUserData*>(userPointer1);
  auto userDataB = static_cast<BulletCollisionObjectUserData*>(userPointer0);

  mResult.contacts.push_back(convertContact(cp, userDataA, userDataB));

  return 1.0f;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
