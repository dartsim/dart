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

#include "dart/collision/bullet/BulletCollisionDetector.hpp"

#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>

#include "dart/common/Console.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/bullet/BulletTypes.hpp"
#include "dart/collision/bullet/BulletCollisionObject.hpp"
#include "dart/collision/bullet/BulletCollisionGroup.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace collision {

namespace {

struct BulletOverlapFilterCallback : public btOverlapFilterCallback
{
  BulletOverlapFilterCallback(const Option& option, const Result& result)
    : mOption(option),
      mResult(result),
      mDone(false)
  {
    // Do nothing
  }

  // return true when pairs need collision
  bool needBroadphaseCollision(btBroadphaseProxy* proxy0,
                               btBroadphaseProxy* proxy1) const override
  {
    if (mDone)
      return false;

    if ((mOption.binaryCheck && mResult.isCollision())
        || (mResult.getNumContacts() >= mOption.maxNumContacts))
    {
      mDone = true;
      return false;
    }

    assert((proxy0 != nullptr && proxy1 != nullptr) &&
           "Bullet broadphase overlapping pair proxies are nullptr");

    bool collide = (proxy0->m_collisionFilterGroup &
                    proxy1->m_collisionFilterMask) != 0;
    collide = collide && (proxy1->m_collisionFilterGroup &
                          proxy0->m_collisionFilterMask);

    const auto& filter = mOption.collisionFilter;

    if (collide && filter)
    {
      auto bulletCollObj0
          = static_cast<btCollisionObject*>(proxy0->m_clientObject);
      auto bulletCollObj1
          = static_cast<btCollisionObject*>(proxy1->m_clientObject);

      auto userData0 = static_cast<BulletCollisionObject::UserData*>(
            bulletCollObj0->getUserPointer());
      auto userData1 = static_cast<BulletCollisionObject::UserData*>(
            bulletCollObj1->getUserPointer());

      collide = filter->needCollision(userData0->collisionObject,
                                      userData1->collisionObject);
    }

    return collide;
  }

  const Option& mOption;
  const Result& mResult;

  /// Whether the collision iteration can stop
  mutable bool mDone;
};

Contact convertContact(const btManifoldPoint& bulletManifoldPoint,
                       const BulletCollisionObject::UserData* userData1,
                       const BulletCollisionObject::UserData* userData2);

void convertContacts(
    btCollisionWorld* collWorld, const Option& option, Result& result);

BulletCollisionDetector::BulletCollsionShapePack
createBulletEllipsoidMesh(float sizeX, float sizeY, float sizeZ);

BulletCollisionDetector::BulletCollsionShapePack
createBulletCollisionShapePackFromAssimpScene(
    const Eigen::Vector3d& scale, const aiScene* scene);

BulletCollisionDetector::BulletCollsionShapePack
createBulletCollisionShapePackFromAssimpMesh(const aiMesh* mesh);

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
std::shared_ptr<CollisionGroup> BulletCollisionDetector::createCollisionGroup()
{
  return std::make_shared<BulletCollisionGroup>(shared_from_this());
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionGroup* group, const Option& option, Result& result)
{
  result.clear();

  if (!group)
    return false;

  if (group->getCollisionDetector()->getType()
      != BulletCollisionDetector::getTypeStatic())
  {
    return false;
  }

  group->update();

  auto castedData = static_cast<BulletCollisionGroup*>(group);
  auto bulletCollisionWorld = castedData->getBulletCollisionWorld();
  auto bulletPairCache = bulletCollisionWorld->getPairCache();
  auto filterCallback = new BulletOverlapFilterCallback(option, result);

  bulletPairCache->setOverlapFilterCallback(filterCallback);
  bulletCollisionWorld->performDiscreteCollisionDetection();

  convertContacts(bulletCollisionWorld, option, result);

  return result.isCollision();
}

//==============================================================================
bool BulletCollisionDetector::detect(
    CollisionGroup* group1, CollisionGroup* group2,
    const Option& option, Result& result)
{
  result.clear();

  if (!group1 || !group2)
    return false;

  if (group1->getCollisionDetector()->getType()
      != BulletCollisionDetector::getTypeStatic())
  {
    return false;
  }

  if (group2->getCollisionDetector()->getType()
      != BulletCollisionDetector::getTypeStatic())
  {
    return false;
  }

  auto group = common::make_unique<BulletCollisionGroup>(shared_from_this());
  group->registerShapeFrames(group1, group2);
  group->update();

  auto bulletCollisionWorld = group->getBulletCollisionWorld();
  auto bulletPairCache = bulletCollisionWorld->getPairCache();
  auto filterCallback = new BulletOverlapFilterCallback(option, result);

  bulletPairCache->setOverlapFilterCallback(filterCallback);
  bulletCollisionWorld->performDiscreteCollisionDetection();

  convertContacts(bulletCollisionWorld, option, result);

  return result.isCollision();
}

//==============================================================================
BulletCollisionDetector::BulletCollisionDetector()
{
  mCollisionObjectManager.reset(new NaiveCollisionObjectManager(this));
}

//==============================================================================
std::unique_ptr<CollisionObject>
BulletCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto pack = claimBulletCollisionGeometry(shapeFrame->getShape());
  auto collObj = new BulletCollisionObject(this, shapeFrame,
                                           pack.collisionShape.get());
  auto object = collObj->getBulletCollisionObject();

  mBulletCollisionObjectMap[object] = collObj;

  return std::unique_ptr<CollisionObject>(collObj);
}

//==============================================================================
void BulletCollisionDetector::notifyCollisionObjectDestorying(
    CollisionObject* collObj)
{
  if (!collObj)
    return;

  reclaimBulletCollisionGeometry(collObj->getShape());

  auto casted = static_cast<BulletCollisionObject*>(collObj);
  mBulletCollisionObjectMap.erase(casted->getBulletCollisionObject());
}

//==============================================================================
BulletCollisionDetector::BulletCollsionShapePack
BulletCollisionDetector::claimBulletCollisionGeometry(
    const dynamics::ConstShapePtr& shape)
{
  BulletCollsionShapePack pack;

  auto findResult = mShapeMap.find(shape);
  if (mShapeMap.end() != findResult)
  {
    auto& packAndCount = findResult->second;

    pack = packAndCount.first;

    auto& count = packAndCount.second;
    assert(0u != count);
    count++;
  }
  else
  {
    pack = createBulletCollisionShapePack(shape);
    mShapeMap[shape] = std::make_pair(pack, 1u);
  }

  return pack;
}

//==============================================================================
void BulletCollisionDetector::reclaimBulletCollisionGeometry(
    const dynamics::ConstShapePtr& shape)
{
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
BulletCollisionDetector::BulletCollsionShapePack
BulletCollisionDetector::createBulletCollisionShapePack(
    const dynamics::ConstShapePtr& shape)
{
  using dynamics::Shape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  BulletCollsionShapePack pack;

  auto& bulletCollisionShape = pack.collisionShape;

  switch (shape->getShapeType())
  {
    case Shape::BOX:
    {
      assert(dynamic_cast<const BoxShape*>(shape.get()));

      const auto box = static_cast<const BoxShape*>(shape.get());
      const Eigen::Vector3d& size = box->getSize();

      bulletCollisionShape.reset(new btBoxShape(convertVector3(size*0.5)));

      break;
    }
    case Shape::ELLIPSOID:
    {
      assert(dynamic_cast<const EllipsoidShape*>(shape.get()));

      const auto ellipsoid = static_cast<const EllipsoidShape*>(shape.get());
      const Eigen::Vector3d& size = ellipsoid->getSize();

      if (ellipsoid->isSphere())
        bulletCollisionShape.reset(new btSphereShape(size[0] * 0.5));
      else
        pack = createBulletEllipsoidMesh(size[0], size[1], size[2]);

      break;
    }
    case Shape::CYLINDER:
    {
      assert(dynamic_cast<const CylinderShape*>(shape.get()));

      const auto cylinder = static_cast<const CylinderShape*>(shape.get());
      const auto radius = cylinder->getRadius();
      const auto height = cylinder->getHeight();
      const auto size = btVector3(radius, radius, height * 0.5);

      bulletCollisionShape.reset(new btCylinderShapeZ(size));

      break;
    }
    case Shape::PLANE:
    {
      assert(dynamic_cast<const PlaneShape*>(shape.get()));

      const auto plane = static_cast<const PlaneShape*>(shape.get());
      const Eigen::Vector3d normal = plane->getNormal();
      const double offset = plane->getOffset();

      bulletCollisionShape.reset(new btStaticPlaneShape(
            convertVector3(normal), offset));

      break;
    }
    case Shape::MESH:
    {
      assert(dynamic_cast<const MeshShape*>(shape.get()));

      const auto shapeMesh = static_cast<const MeshShape*>(shape.get());
      const auto scale = shapeMesh->getScale();
      const auto mesh = shapeMesh->getMesh();

      pack = createBulletCollisionShapePackFromAssimpScene(scale, mesh);

      break;
    }
    case Shape::SOFT_MESH:
    {
      assert(dynamic_cast<const SoftMeshShape*>(shape.get()));

      const auto softMeshShape = static_cast<const SoftMeshShape*>(shape.get());
      const auto mesh = softMeshShape->getAssimpMesh();

      pack = createBulletCollisionShapePackFromAssimpMesh(mesh);

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
Contact convertContact(const btManifoldPoint& bulletManifoldPoint,
                       const BulletCollisionObject::UserData* userData1,
                       const BulletCollisionObject::UserData* userData2)
{
  assert(userData1);
  assert(userData2);

  Contact contact;

  contact.point = convertVector3(bulletManifoldPoint.getPositionWorldOnA());
  contact.normal = convertVector3(-bulletManifoldPoint.m_normalWorldOnB);
  contact.penetrationDepth = -bulletManifoldPoint.m_distance1;
  contact.collisionObject1 = userData1->collisionObject;
  contact.collisionObject2 = userData2->collisionObject;

  return contact;
}

//==============================================================================
void convertContacts(
    btCollisionWorld* collWorld, const Option& option, Result& result)
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

    auto userDataA
        = static_cast<BulletCollisionObject::UserData*>(userPointer1);
    auto userDataB
        = static_cast<BulletCollisionObject::UserData*>(userPointer0);

    auto numContacts = contactManifold->getNumContacts();

    for (auto j = 0; j < numContacts; ++j)
    {
      auto& cp = contactManifold->getContactPoint(j);

      result.addContact(convertContact(cp, userDataA, userDataB));

      if (option.binaryCheck)
        return;

      if (result.getNumContacts() >= option.maxNumContacts)
        break;
    }
  }
}

//==============================================================================
BulletCollisionDetector::BulletCollsionShapePack createBulletEllipsoidMesh(
    float sizeX, float sizeY, float sizeZ)
{
  float v[59][3] =
  {
    {0, 0, 0},
    {0.135299, -0.461940, -0.135299},
    {0.000000, -0.461940, -0.191342},
    {-0.135299, -0.461940, -0.135299},
    {-0.191342, -0.461940, 0.000000},
    {-0.135299, -0.461940, 0.135299},
    {0.000000, -0.461940, 0.191342},
    {0.135299, -0.461940, 0.135299},
    {0.191342, -0.461940, 0.000000},
    {0.250000, -0.353553, -0.250000},
    {0.000000, -0.353553, -0.353553},
    {-0.250000, -0.353553, -0.250000},
    {-0.353553, -0.353553, 0.000000},
    {-0.250000, -0.353553, 0.250000},
    {0.000000, -0.353553, 0.353553},
    {0.250000, -0.353553, 0.250000},
    {0.353553, -0.353553, 0.000000},
    {0.326641, -0.191342, -0.326641},
    {0.000000, -0.191342, -0.461940},
    {-0.326641, -0.191342, -0.326641},
    {-0.461940, -0.191342, 0.000000},
    {-0.326641, -0.191342, 0.326641},
    {0.000000, -0.191342, 0.461940},
    {0.326641, -0.191342, 0.326641},
    {0.461940, -0.191342, 0.000000},
    {0.353553, 0.000000, -0.353553},
    {0.000000, 0.000000, -0.500000},
    {-0.353553, 0.000000, -0.353553},
    {-0.500000, 0.000000, 0.000000},
    {-0.353553, 0.000000, 0.353553},
    {0.000000, 0.000000, 0.500000},
    {0.353553, 0.000000, 0.353553},
    {0.500000, 0.000000, 0.000000},
    {0.326641, 0.191342, -0.326641},
    {0.000000, 0.191342, -0.461940},
    {-0.326641, 0.191342, -0.326641},
    {-0.461940, 0.191342, 0.000000},
    {-0.326641, 0.191342, 0.326641},
    {0.000000, 0.191342, 0.461940},
    {0.326641, 0.191342, 0.326641},
    {0.461940, 0.191342, 0.000000},
    {0.250000, 0.353553, -0.250000},
    {0.000000, 0.353553, -0.353553},
    {-0.250000, 0.353553, -0.250000},
    {-0.353553, 0.353553, 0.000000},
    {-0.250000, 0.353553, 0.250000},
    {0.000000, 0.353553, 0.353553},
    {0.250000, 0.353553, 0.250000},
    {0.353553, 0.353553, 0.000000},
    {0.135299, 0.461940, -0.135299},
    {0.000000, 0.461940, -0.191342},
    {-0.135299, 0.461940, -0.135299},
    {-0.191342, 0.461940, 0.000000},
    {-0.135299, 0.461940, 0.135299},
    {0.000000, 0.461940, 0.191342},
    {0.135299, 0.461940, 0.135299},
    {0.191342, 0.461940, 0.000000},
    {0.000000, -0.500000, 0.000000},
    {0.000000, 0.500000, 0.000000}
  };

  int f[112][3] =
  {
    {1, 2, 9},
    {9, 2, 10},
    {2, 3, 10},
    {10, 3, 11},
    {3, 4, 11},
    {11, 4, 12},
    {4, 5, 12},
    {12, 5, 13},
    {5, 6, 13},
    {13, 6, 14},
    {6, 7, 14},
    {14, 7, 15},
    {7, 8, 15},
    {15, 8, 16},
    {8, 1, 16},
    {16, 1, 9},
    {9, 10, 17},
    {17, 10, 18},
    {10, 11, 18},
    {18, 11, 19},
    {11, 12, 19},
    {19, 12, 20},
    {12, 13, 20},
    {20, 13, 21},
    {13, 14, 21},
    {21, 14, 22},
    {14, 15, 22},
    {22, 15, 23},
    {15, 16, 23},
    {23, 16, 24},
    {16, 9, 24},
    {24, 9, 17},
    {17, 18, 25},
    {25, 18, 26},
    {18, 19, 26},
    {26, 19, 27},
    {19, 20, 27},
    {27, 20, 28},
    {20, 21, 28},
    {28, 21, 29},
    {21, 22, 29},
    {29, 22, 30},
    {22, 23, 30},
    {30, 23, 31},
    {23, 24, 31},
    {31, 24, 32},
    {24, 17, 32},
    {32, 17, 25},
    {25, 26, 33},
    {33, 26, 34},
    {26, 27, 34},
    {34, 27, 35},
    {27, 28, 35},
    {35, 28, 36},
    {28, 29, 36},
    {36, 29, 37},
    {29, 30, 37},
    {37, 30, 38},
    {30, 31, 38},
    {38, 31, 39},
    {31, 32, 39},
    {39, 32, 40},
    {32, 25, 40},
    {40, 25, 33},
    {33, 34, 41},
    {41, 34, 42},
    {34, 35, 42},
    {42, 35, 43},
    {35, 36, 43},
    {43, 36, 44},
    {36, 37, 44},
    {44, 37, 45},
    {37, 38, 45},
    {45, 38, 46},
    {38, 39, 46},
    {46, 39, 47},
    {39, 40, 47},
    {47, 40, 48},
    {40, 33, 48},
    {48, 33, 41},
    {41, 42, 49},
    {49, 42, 50},
    {42, 43, 50},
    {50, 43, 51},
    {43, 44, 51},
    {51, 44, 52},
    {44, 45, 52},
    {52, 45, 53},
    {45, 46, 53},
    {53, 46, 54},
    {46, 47, 54},
    {54, 47, 55},
    {47, 48, 55},
    {55, 48, 56},
    {48, 41, 56},
    {56, 41, 49},
    {2, 1, 57},
    {3, 2, 57},
    {4, 3, 57},
    {5, 4, 57},
    {6, 5, 57},
    {7, 6, 57},
    {8, 7, 57},
    {1, 8, 57},
    {49, 50, 58},
    {50, 51, 58},
    {51, 52, 58},
    {52, 53, 58},
    {53, 54, 58},
    {54, 55, 58},
    {55, 56, 58},
    {56, 49, 58}
  };

  BulletCollisionDetector::BulletCollsionShapePack pack;
  pack.triMesh.reset(new btTriangleMesh());

  for (auto i = 0u; i < 112; ++i)
  {
    btVector3 vertices[3];

    const auto& index0 = f[i][0];
    const auto& index1 = f[i][1];
    const auto& index2 = f[i][2];

    const auto& p0 = v[index0];
    const auto& p1 = v[index1];
    const auto& p2 = v[index2];

    vertices[0] = btVector3(p0[0] * sizeX, p0[1] * sizeY, p0[2] * sizeZ);
    vertices[1] = btVector3(p1[0] * sizeX, p1[1] * sizeY, p1[2] * sizeZ);
    vertices[2] = btVector3(p2[0] * sizeX, p2[1] * sizeY, p2[2] * sizeZ);

    pack.triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  auto gimpactMeshShape = new btGImpactMeshShape(pack.triMesh.get());
  gimpactMeshShape->updateBound();

  pack.collisionShape.reset(gimpactMeshShape);

  return pack;
}

//==============================================================================
BulletCollisionDetector::BulletCollsionShapePack
createBulletCollisionShapePackFromAssimpScene(
    const Eigen::Vector3d& scale, const aiScene* scene)
{
  BulletCollisionDetector::BulletCollsionShapePack pack;
  pack.triMesh.reset(new btTriangleMesh());

  for (auto i = 0u; i < scene->mNumMeshes; ++i)
  {
    for (auto j = 0u; j < scene->mMeshes[i]->mNumFaces; ++j)
    {
      btVector3 vertices[3];
      for (auto k = 0u; k < 3; ++k)
      {
        const aiVector3D& vertex = scene->mMeshes[i]->mVertices[
                                   scene->mMeshes[i]->mFaces[j].mIndices[k]];
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
BulletCollisionDetector::BulletCollsionShapePack
createBulletCollisionShapePackFromAssimpMesh(const aiMesh* mesh)
{
  BulletCollisionDetector::BulletCollsionShapePack pack;
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

} // anonymous namespace

} // namespace collision
} // namespace dart
