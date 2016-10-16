/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

// Must be included before any Bullet headers.
#include "dart/config.hpp"

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
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MultiSphereShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace collision {

namespace {

struct BulletOverlapFilterCallback : public btOverlapFilterCallback
{
  BulletOverlapFilterCallback(
      const CollisionOption& option,
      CollisionResult* result)
    : option(option),
      result(result),
      foundCollision(false),
      done(false)
  {
    // Do nothing
  }

  // return true when pairs need collision
  bool needBroadphaseCollision(
      btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
  {
    if (done)
      return false;

    assert((proxy0 != nullptr && proxy1 != nullptr) &&
           "Bullet broadphase overlapping pair proxies are nullptr");

    bool collide = (proxy0->m_collisionFilterGroup &
                    proxy1->m_collisionFilterMask) != 0;
    collide = collide && (proxy1->m_collisionFilterGroup &
                          proxy0->m_collisionFilterMask);

    const auto& filter = option.collisionFilter;

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

  const CollisionOption& option;
  const CollisionResult* result;

  /// True if at least one contact is found. This flag is used only when
  /// mResult is nullptr; otherwise the actual collision result is in mResult.
  bool foundCollision;

  /// Whether the collision iteration can stop
  mutable bool done;
};

Contact convertContact(const btManifoldPoint& bulletManifoldPoint,
                       const BulletCollisionObject::UserData* userData1,
                       const BulletCollisionObject::UserData* userData2);

void convertContacts(btCollisionWorld* collWorld,
                     BulletOverlapFilterCallback* overlapFilterCallback,
                     const CollisionOption& option,
                     CollisionResult& result);

btCollisionShape* createBulletEllipsoidMesh(
    float sizeX, float sizeY, float sizeZ);

btCollisionShape* createBulletCollisionShapeFromAssimpScene(
    const Eigen::Vector3d& scale, const aiScene* scene);

btCollisionShape* createBulletCollisionShapeFromAssimpMesh(const aiMesh* mesh);

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
std::shared_ptr<CollisionDetector>
BulletCollisionDetector::cloneWithoutCollisionObjects()
{
  return BulletCollisionDetector::create();
}

//==============================================================================
const std::string& BulletCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& BulletCollisionDetector::getStaticType()
{
  static const std::string type = "bullet";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup>
BulletCollisionDetector::createCollisionGroup()
{
  return common::make_unique<BulletCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(
    BulletCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get())
  {
    dterr << "[BulletCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
static bool isCollision(btCollisionWorld* world)
{
  assert(world);

  auto dispatcher = world->getDispatcher();
  assert(dispatcher);

  const auto numManifolds = dispatcher->getNumManifolds();

  for (auto i = 0; i < numManifolds; ++i)
  {
    const auto* contactManifold = dispatcher->getManifoldByIndexInternal(i);

    if (contactManifold->getNumContacts() > 0)
      return true;
  }

  return false;
}

//==============================================================================
bool BulletCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  // TODO(JS): It seems, when new BulletOverlapFilterCallback is set to
  // btCollisionWorld, bullet doesn't update the list of collision pairs that
  // was generated before. Also, there is no way to update the list manually.
  // Please report us if it's not true.
  //
  // In order to have filtered pairs in btCollisionWorld, we instead create a
  // new btCollisionWorld (by creating new BulletCollisionGroup) and add the
  // collision objects to the new btCollisionWorld so that the filter prevents
  // btCollisionWorld to generate unnecessary pairs, which is very inefficient
  // way.

  mGroupForFiltering.reset(new BulletCollisionGroup(shared_from_this()));
  auto bulletCollisionWorld = mGroupForFiltering->getBulletCollisionWorld();
  auto bulletPairCache = bulletCollisionWorld->getPairCache();
  auto filterCallback = new BulletOverlapFilterCallback(option, result);
  bulletPairCache->setOverlapFilterCallback(filterCallback);

  mGroupForFiltering->addShapeFramesOf(group);
  mGroupForFiltering->updateEngineData();

  bulletCollisionWorld->performDiscreteCollisionDetection();

  if (result)
  {
    convertContacts(bulletCollisionWorld, filterCallback, option, *result);

    return result->isCollision();
  }
  else
  {
    return isCollision(bulletCollisionWorld);
  }
}

//==============================================================================
bool BulletCollisionDetector::collide(
    CollisionGroup* group1, CollisionGroup* group2,
    const CollisionOption& option, CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  dtwarn << "[BulletCollisionDetector::collide] collide(group1, group2) "
         << "supposed to check collisions of the objects in group1 against the "
         << "objects in group2. However, the current implementation of this "
         << "function checks for all the objects against each other of both "
         << "group1 and group2, which is an incorrect behavior. This bug will "
         << "be fixed in the next patch release. (see #717 for the details)\n";

  mGroupForFiltering.reset(new BulletCollisionGroup(shared_from_this()));
  auto bulletCollisionWorld = mGroupForFiltering->getBulletCollisionWorld();
  auto bulletPairCache = bulletCollisionWorld->getPairCache();
  auto filterCallback = new BulletOverlapFilterCallback(option, result);
  bulletPairCache->setOverlapFilterCallback(filterCallback);

  mGroupForFiltering->addShapeFramesOf(group1, group2);
  mGroupForFiltering->updateEngineData();

  bulletCollisionWorld->performDiscreteCollisionDetection();

  if (result)
  {
    convertContacts(bulletCollisionWorld, filterCallback, option, *result);

    return result->isCollision();
  }
  else
  {
    return isCollision(bulletCollisionWorld);
  }
}

//==============================================================================
double BulletCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[BulletCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
double BulletCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[BulletCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning.\n";

  return 0.0;
}

//==============================================================================
BulletCollisionDetector::BulletCollisionDetector()
  : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForUnsharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject> BulletCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto bulletCollShape = claimBulletCollisionShape(shapeFrame->getShape());

  return std::unique_ptr<BulletCollisionObject>(
        new BulletCollisionObject(this, shapeFrame, bulletCollShape));
}

//==============================================================================
void BulletCollisionDetector::notifyCollisionObjectDestroying(
    CollisionObject* object)
{
  reclaimBulletCollisionShape(object->getShape());
}

//==============================================================================
btCollisionShape* BulletCollisionDetector::claimBulletCollisionShape(
    const dynamics::ConstShapePtr& shape)
{
  const auto search = mShapeMap.find(shape);

  if (mShapeMap.end() != search)
  {
    auto& bulletCollShapeAndCount = search->second;

    auto& bulletCollShape = bulletCollShapeAndCount.first;
    auto& count = bulletCollShapeAndCount.second;
    assert(0u != count);

    count++;

    return bulletCollShape;
  }

  auto newBulletCollisionShape = createBulletCollisionShape(shape);
  mShapeMap[shape] = std::make_pair(newBulletCollisionShape, 1u);

  return newBulletCollisionShape;
}

//==============================================================================
void BulletCollisionDetector::reclaimBulletCollisionShape(
    const dynamics::ConstShapePtr& shape)
{
  auto search = mShapeMap.find(shape);

  assert(mShapeMap.end() != search);

  auto& bulletCollShapeAndCount = search->second;

  auto& bulletCollShape = bulletCollShapeAndCount.first;
  auto& count = bulletCollShapeAndCount.second;

  count--;

  if (0u == count)
  {
    auto userPointer = bulletCollShape->getUserPointer();
    if (userPointer)
      delete static_cast<btTriangleMesh*>(userPointer);

    delete bulletCollShape;

    mShapeMap.erase(search);
  }
}

//==============================================================================
btCollisionShape* BulletCollisionDetector::createBulletCollisionShape(
    const dynamics::ConstShapePtr& shape)
{
  using dynamics::Shape;
  using dynamics::SphereShape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::CapsuleShape;
  using dynamics::ConeShape;
  using dynamics::PlaneShape;
  using dynamics::MultiSphereShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  btCollisionShape* bulletCollisionShape = nullptr;

  if (shape->is<SphereShape>())
  {
    assert(dynamic_cast<const SphereShape*>(shape.get()));

    const auto sphere = static_cast<const SphereShape*>(shape.get());
    const auto radius = sphere->getRadius();

    bulletCollisionShape = new btSphereShape(radius);
  }
  else if (shape->is<BoxShape>())
  {
    assert(dynamic_cast<const BoxShape*>(shape.get()));

    const auto box = static_cast<const BoxShape*>(shape.get());
    const Eigen::Vector3d& size = box->getSize();

    bulletCollisionShape = new btBoxShape(convertVector3(size*0.5));
  }
  else if (shape->is<EllipsoidShape>())
  {
    assert(dynamic_cast<const EllipsoidShape*>(shape.get()));

    const auto ellipsoid = static_cast<const EllipsoidShape*>(shape.get());
    const Eigen::Vector3d& size = ellipsoid->getSize();

    bulletCollisionShape = createBulletEllipsoidMesh(
          size[0], size[1], size[2]);
  }
  else if (shape->is<CylinderShape>())
  {
    assert(dynamic_cast<const CylinderShape*>(shape.get()));

    const auto cylinder = static_cast<const CylinderShape*>(shape.get());
    const auto radius = cylinder->getRadius();
    const auto height = cylinder->getHeight();
    const auto size = btVector3(radius, radius, height * 0.5);

    bulletCollisionShape = new btCylinderShapeZ(size);
  }
  else if (shape->is<CapsuleShape>())
  {
    assert(dynamic_cast<const CapsuleShape*>(shape.get()));

    const auto capsule = static_cast<const CapsuleShape*>(shape.get());
    const auto radius = capsule->getRadius();
    const auto height = capsule->getHeight();

    bulletCollisionShape = new btCapsuleShapeZ(radius, height);
  }
  else if (shape->is<ConeShape>())
  {
    assert(dynamic_cast<const ConeShape*>(shape.get()));

    const auto cone = static_cast<const ConeShape*>(shape.get());
    const auto radius = cone->getRadius();
    const auto height = cone->getHeight();

    bulletCollisionShape = new btConeShapeZ(radius, height);
    bulletCollisionShape->setMargin(0.0);
    // TODO(JS): Bullet seems to use constant margin 0.4, however this could be
    // dangerous when the cone is sufficiently small. We use zero margin here
    // until find better solution even using zero margin is not recommended:
    // https://www.sjbaker.org/wiki/index.php?title=Physics_-_Bullet_Collected_random_advice#Minimum_object_sizes_-_by_Erwin
  }
  else if (shape->is<PlaneShape>())
  {
    assert(dynamic_cast<const PlaneShape*>(shape.get()));

    const auto plane = static_cast<const PlaneShape*>(shape.get());
    const Eigen::Vector3d normal = plane->getNormal();
    const double offset = plane->getOffset();

    bulletCollisionShape = new btStaticPlaneShape(
          convertVector3(normal), offset);
  }
  else if (shape->is<MultiSphereShape>())
  {
    assert(dynamic_cast<const MultiSphereShape*>(shape.get()));

    const auto multiSphere = static_cast<const MultiSphereShape*>(shape.get());
    const auto numSpheres = multiSphere->getNumSpheres();
    const auto& spheres = multiSphere->getSpheres();

    std::vector<btVector3> bulletPositions(numSpheres);
    std::vector<btScalar> bulletRadii(numSpheres);

    for (auto i = 0u; i < numSpheres; ++i)
    {
      bulletRadii[i] = static_cast<btScalar>(spheres[i].first);
      bulletPositions[i] = convertVector3(spheres[i].second);
    }

    bulletCollisionShape = new btMultiSphereShape(
          bulletPositions.data(), bulletRadii.data(), numSpheres);
  }
  else if (shape->is<MeshShape>())
  {
    assert(dynamic_cast<const MeshShape*>(shape.get()));

    const auto shapeMesh = static_cast<const MeshShape*>(shape.get());
    const auto scale = shapeMesh->getScale();
    const auto mesh = shapeMesh->getMesh();

    bulletCollisionShape = createBulletCollisionShapeFromAssimpScene(
          scale, mesh);
  }
  else if (shape->is<SoftMeshShape>())
  {
    assert(dynamic_cast<const SoftMeshShape*>(shape.get()));

    const auto softMeshShape = static_cast<const SoftMeshShape*>(shape.get());
    const auto mesh = softMeshShape->getAssimpMesh();

    bulletCollisionShape = createBulletCollisionShapeFromAssimpMesh(mesh);
  }
  else
  {
    dterr << "[BulletCollisionDetector::createBulletCollisionShape] "
          << "Attempting to create an unsupported shape type ["
          << shape->getType() << "] Creating a sphere with 0.1 radius "
          << "instead.\n";

    bulletCollisionShape = new btSphereShape(0.1);
  }

  return bulletCollisionShape;
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
  contact.normal = convertVector3(bulletManifoldPoint.m_normalWorldOnB);
  contact.penetrationDepth = -bulletManifoldPoint.m_distance1;
  contact.collisionObject1 = userData1->collisionObject;
  contact.collisionObject2 = userData2->collisionObject;

  return contact;
}

//==============================================================================
void convertContacts(
    btCollisionWorld* collWorld,
    BulletOverlapFilterCallback* overlapFilterCallback,
    const CollisionOption& option,
    CollisionResult& result)
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
        = static_cast<BulletCollisionObject::UserData*>(userPointer0);
    auto userDataB
        = static_cast<BulletCollisionObject::UserData*>(userPointer1);

    auto numContacts = contactManifold->getNumContacts();

    for (auto j = 0; j < numContacts; ++j)
    {
      auto& cp = contactManifold->getContactPoint(j);

      result.addContact(convertContact(cp, userDataA, userDataB));

      if (result.getNumContacts() >= option.maxNumContacts)
      {
        overlapFilterCallback->done = true;
        return;
      }
    }
  }
}

//==============================================================================
btCollisionShape* createBulletEllipsoidMesh(
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

  auto triMesh = new btTriangleMesh();

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

    triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  auto gimpactMeshShape = new btGImpactMeshShape(triMesh);
  gimpactMeshShape->updateBound();

  return gimpactMeshShape;
}

//==============================================================================
btCollisionShape* createBulletCollisionShapeFromAssimpScene(
    const Eigen::Vector3d& scale, const aiScene* scene)
{
  auto triMesh = new btTriangleMesh();

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
      triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  auto gimpactMeshShape = new btGImpactMeshShape(triMesh);
  gimpactMeshShape->updateBound();
  gimpactMeshShape->setUserPointer(triMesh);

  return gimpactMeshShape;
}

//==============================================================================
btCollisionShape* createBulletCollisionShapeFromAssimpMesh(const aiMesh* mesh)
{
  auto triMesh = new btTriangleMesh();

  for (auto i = 0u; i < mesh->mNumFaces; ++i)
  {
    btVector3 vertices[3];
    for (auto j = 0u; j < 3; ++j)
    {
      const aiVector3D& vertex = mesh->mVertices[mesh->mFaces[i].mIndices[j]];
      vertices[j] = btVector3(vertex.x, vertex.y, vertex.z);
    }
    triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  auto gimpactMeshShape = new btGImpactMeshShape(triMesh);
  gimpactMeshShape->updateBound();

  return gimpactMeshShape;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
