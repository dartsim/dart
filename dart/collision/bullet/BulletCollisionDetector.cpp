/*
 * Copyright (c) 2011-2021, The DART development contributors
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

// Must be included before any Bullet headers.
#include "dart/collision/bullet/BulletCollisionDetector.hpp"

#include <algorithm>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/bullet/BulletCollisionGroup.hpp"
#include "dart/collision/bullet/BulletCollisionObject.hpp"
#include "dart/collision/bullet/BulletTypes.hpp"
#include "dart/collision/bullet/detail/BulletCollisionDispatcher.hpp"
#include "dart/collision/bullet/detail/BulletOverlapFilterCallback.hpp"
#include "dart/common/Console.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/HeightmapShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"

namespace dart {
namespace collision {

namespace {

Contact convertContact(
    const btManifoldPoint& bulletManifoldPoint,
    BulletCollisionObject* collObj1,
    BulletCollisionObject* collObj2);

void reportContacts(
    btCollisionWorld* collWorld,
    const CollisionOption& option,
    CollisionResult& result);

void reportRayHits(
    const btCollisionWorld::ClosestRayResultCallback callback,
    const RaycastOption& option,
    RaycastResult& result);

void reportRayHits(
    const btCollisionWorld::AllHitsRayResultCallback callback,
    const RaycastOption& option,
    RaycastResult& result);

std::unique_ptr<btCollisionShape> createBulletEllipsoidMesh(
    float sizeX, float sizeY, float sizeZ);

std::unique_ptr<btCollisionShape> createBulletCollisionShapeFromAssimpScene(
    const Eigen::Vector3d& scale, const aiScene* scene);

std::unique_ptr<btCollisionShape> createBulletCollisionShapeFromAssimpMesh(
    const aiMesh* mesh);

template <typename HeightmapShapeT>
std::unique_ptr<BulletCollisionShape> createBulletCollisionShapeFromHeightmap(
    const HeightmapShapeT* heightMap);

} // anonymous namespace

//==============================================================================
BulletCollisionDetector::Registrar<BulletCollisionDetector>
    BulletCollisionDetector::mRegistrar{
        BulletCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::BulletCollisionDetector> {
          return dart::collision::BulletCollisionDetector::create();
        }};

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
BulletCollisionDetector::cloneWithoutCollisionObjects() const
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
std::unique_ptr<CollisionGroup> BulletCollisionDetector::createCollisionGroup()
{
  return std::make_unique<BulletCollisionGroup>(shared_from_this());
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
void filterOutCollisions(btCollisionWorld* world)
{
  assert(world);

  auto dispatcher
      = static_cast<detail::BulletCollisionDispatcher*>(world->getDispatcher());
  assert(dispatcher);

  const auto filter = dispatcher->getFilter();
  if (!filter)
    return;

  const auto numManifolds = dispatcher->getNumManifolds();

  std::vector<btPersistentManifold*> manifoldsToRelease;

  for (auto i = 0; i < numManifolds; ++i)
  {
    const auto contactManifold = dispatcher->getManifoldByIndexInternal(i);

    const auto body0 = contactManifold->getBody0();
    const auto body1 = contactManifold->getBody1();

    const auto userPtr0 = body0->getUserPointer();
    const auto userPtr1 = body1->getUserPointer();

    const auto collObj0 = static_cast<BulletCollisionObject*>(userPtr0);
    const auto collObj1 = static_cast<BulletCollisionObject*>(userPtr1);

    if (filter->ignoresCollision(collObj0, collObj1))
      manifoldsToRelease.push_back(contactManifold);
  }

  for (const auto& manifold : manifoldsToRelease)
    dispatcher->clearManifold(manifold);
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

  // Check if 'this' is the collision engine of 'group'.
  if (!checkGroupValidity(this, group))
    return false;

  auto castedGroup = static_cast<BulletCollisionGroup*>(group);
  auto collisionWorld = castedGroup->getBulletCollisionWorld();

  auto dispatcher = static_cast<detail::BulletCollisionDispatcher*>(
      collisionWorld->getDispatcher());
  dispatcher->setFilter(option.collisionFilter);

  // Filter out persistent contact pairs already existing in the world
  filterOutCollisions(collisionWorld);

  castedGroup->updateEngineData();
  collisionWorld->performDiscreteCollisionDetection();

  if (result)
  {
    reportContacts(collisionWorld, option, *result);

    return result->isCollision();
  }
  else
  {
    return isCollision(collisionWorld);
  }
}

//==============================================================================
bool BulletCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  // Create a new collision group, merging the two groups into
  mGroupForFiltering.reset(new BulletCollisionGroup(shared_from_this()));
  auto bulletCollisionWorld = mGroupForFiltering->getBulletCollisionWorld();
  auto bulletPairCache = bulletCollisionWorld->getPairCache();
  auto filterCallback = new detail::BulletOverlapFilterCallback(
      option.collisionFilter, group1, group2);
  bulletPairCache->setOverlapFilterCallback(filterCallback);

  mGroupForFiltering->addShapeFramesOf(group1, group2);
  mGroupForFiltering->updateEngineData();

  bulletCollisionWorld->performDiscreteCollisionDetection();

  if (result)
  {
    reportContacts(bulletCollisionWorld, option, *result);

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
  static bool warned = false;
  if (!warned)
  {
    dtwarn
        << "[BulletCollisionDetector::distance] This collision detector does "
        << "not support (signed) distance queries. Returning 0.0.\n";
    warned = true;
  }

  return 0.0;
}

//==============================================================================
double BulletCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  static bool warned = false;
  if (!warned)
  {
    dtwarn
        << "[BulletCollisionDetector::distance] This collision detector does "
        << "not support (signed) distance queries. Returning.\n";
    warned = true;
  }

  return 0.0;
}

//==============================================================================
bool BulletCollisionDetector::raycast(
    CollisionGroup* group,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (result)
    result->clear();

  // Check if 'this' is the collision engine of 'group'.
  if (!checkGroupValidity(this, group))
    return false;

  auto castedGroup = static_cast<BulletCollisionGroup*>(group);
  auto collisionWorld = castedGroup->getBulletCollisionWorld();

  const auto btFrom = convertVector3(from);
  const auto btTo = convertVector3(to);

  if (option.mEnableAllHits)
  {
    auto callback = btCollisionWorld::AllHitsRayResultCallback(btFrom, btTo);
    castedGroup->updateEngineData();
    collisionWorld->rayTest(btFrom, btTo, callback);

    if (result == nullptr)
      return callback.hasHit();

    if (callback.hasHit())
    {
      reportRayHits(callback, option, *result);
      return result->hasHit();
    }
    else
    {
      return false;
    }
  }
  else
  {
    auto callback = btCollisionWorld::ClosestRayResultCallback(btFrom, btTo);
    castedGroup->updateEngineData();
    collisionWorld->rayTest(btFrom, btTo, callback);

    if (result == nullptr)
      return callback.hasHit();

    if (callback.hasHit())
    {
      reportRayHits(callback, option, *result);
      return result->hasHit();
    }
    else
    {
      return false;
    }
  }
}

//==============================================================================
BulletCollisionDetector::BulletCollisionDetector() : CollisionDetector()
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
void BulletCollisionDetector::refreshCollisionObject(CollisionObject* object)
{
  BulletCollisionObject* bullet = static_cast<BulletCollisionObject*>(object);

  bullet->mBulletCollisionShape = claimBulletCollisionShape(bullet->getShape());
  bullet->mBulletCollisionObject->setCollisionShape(
      bullet->mBulletCollisionShape->mCollisionShape.get());
}

//==============================================================================
void BulletCollisionDetector::notifyCollisionObjectDestroying(
    CollisionObject* object)
{
  reclaimBulletCollisionShape(object->getShape());
}

//==============================================================================
std::shared_ptr<BulletCollisionShape>
BulletCollisionDetector::claimBulletCollisionShape(
    const dynamics::ConstShapePtr& shape)
{
  const std::size_t currentVersion = shape->getVersion();

  const auto search = mShapeMap.insert(std::make_pair(shape, ShapeInfo()));
  const bool inserted = search.second;
  ShapeInfo& info = search.first->second;

  if (!inserted && currentVersion == info.mLastKnownVersion)
  {
    const auto& bulletCollShape = info.mShape.lock();
    assert(bulletCollShape);

    return bulletCollShape;
  }

  auto newBulletCollisionShape = std::shared_ptr<BulletCollisionShape>(
      createBulletCollisionShape(shape).release(),
      BulletCollisionShapeDeleter(this, shape));
  info.mShape = newBulletCollisionShape;
  info.mLastKnownVersion = currentVersion;

  return newBulletCollisionShape;
}

//==============================================================================
void BulletCollisionDetector::reclaimBulletCollisionShape(
    const dynamics::ConstShapePtr& shape)
{
  const auto& search = mShapeMap.find(shape);
  if (search == mShapeMap.end())
    return;

  const auto& bulletShape = search->second.mShape.lock();
  if (!bulletShape || bulletShape.use_count() <= 2)
    mShapeMap.erase(search);
}

//==============================================================================
std::unique_ptr<BulletCollisionShape>
BulletCollisionDetector::createBulletCollisionShape(
    const dynamics::ConstShapePtr& shape)
{
  using dynamics::BoxShape;
  using dynamics::CapsuleShape;
  using dynamics::ConeShape;
  using dynamics::CylinderShape;
  using dynamics::EllipsoidShape;
  using dynamics::HeightmapShaped;
  using dynamics::HeightmapShapef;
  using dynamics::MeshShape;
  using dynamics::MultiSphereConvexHullShape;
  using dynamics::PlaneShape;
  using dynamics::Shape;
  using dynamics::SoftMeshShape;
  using dynamics::SphereShape;

  if (shape->is<SphereShape>())
  {
    assert(dynamic_cast<const SphereShape*>(shape.get()));

    const auto sphere = static_cast<const SphereShape*>(shape.get());
    const auto radius = sphere->getRadius();

    auto bulletCollisionShape = std::make_unique<btSphereShape>(radius);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<BoxShape>())
  {
    assert(dynamic_cast<const BoxShape*>(shape.get()));

    const auto box = static_cast<const BoxShape*>(shape.get());
    const Eigen::Vector3d& size = box->getSize();

    auto bulletCollisionShape
        = std::make_unique<btBoxShape>(convertVector3(size * 0.5));

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<EllipsoidShape>())
  {
    assert(dynamic_cast<const EllipsoidShape*>(shape.get()));

    const auto ellipsoid = static_cast<const EllipsoidShape*>(shape.get());
    const Eigen::Vector3d& radii = ellipsoid->getRadii();

    auto bulletCollisionShape = createBulletEllipsoidMesh(
        radii[0] * 2.0, radii[1] * 2.0, radii[2] * 2.0);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<CylinderShape>())
  {
    assert(dynamic_cast<const CylinderShape*>(shape.get()));

    const auto cylinder = static_cast<const CylinderShape*>(shape.get());
    const auto radius = cylinder->getRadius();
    const auto height = cylinder->getHeight();
    const auto size = btVector3(radius, radius, height * 0.5);

    auto bulletCollisionShape = std::make_unique<btCylinderShapeZ>(size);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<CapsuleShape>())
  {
    assert(dynamic_cast<const CapsuleShape*>(shape.get()));

    const auto capsule = static_cast<const CapsuleShape*>(shape.get());
    const auto radius = capsule->getRadius();
    const auto height = capsule->getHeight();

    auto bulletCollisionShape
        = std::make_unique<btCapsuleShapeZ>(radius, height);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<ConeShape>())
  {
    assert(dynamic_cast<const ConeShape*>(shape.get()));

    const auto cone = static_cast<const ConeShape*>(shape.get());
    const auto radius = cone->getRadius();
    const auto height = cone->getHeight();

    auto bulletCollisionShape = std::make_unique<btConeShapeZ>(radius, height);
    bulletCollisionShape->setMargin(0.0);
    // TODO(JS): Bullet seems to use constant margin 0.4, however this could be
    // dangerous when the cone is sufficiently small. We use zero margin here
    // until find better solution even using zero margin is not recommended:
    // https://www.sjbaker.org/wiki/index.php?title=Physics_-_Bullet_Collected_random_advice#Minimum_object_sizes_-_by_Erwin

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<PlaneShape>())
  {
    assert(dynamic_cast<const PlaneShape*>(shape.get()));

    const auto plane = static_cast<const PlaneShape*>(shape.get());
    const Eigen::Vector3d normal = plane->getNormal();
    const double offset = plane->getOffset();

    auto bulletCollisionShape
        = std::make_unique<btStaticPlaneShape>(convertVector3(normal), offset);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<MultiSphereConvexHullShape>())
  {
    assert(dynamic_cast<const MultiSphereConvexHullShape*>(shape.get()));

    const auto multiSphere
        = static_cast<const MultiSphereConvexHullShape*>(shape.get());
    const auto numSpheres = multiSphere->getNumSpheres();
    const auto& spheres = multiSphere->getSpheres();

    std::vector<btVector3> bulletPositions(numSpheres);
    std::vector<btScalar> bulletRadii(numSpheres);

    for (auto i = 0u; i < numSpheres; ++i)
    {
      bulletRadii[i] = static_cast<btScalar>(spheres[i].first);
      bulletPositions[i] = convertVector3(spheres[i].second);
    }

    auto bulletCollisionShape = std::make_unique<btMultiSphereShape>(
        bulletPositions.data(), bulletRadii.data(), numSpheres);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<MeshShape>())
  {
    assert(dynamic_cast<const MeshShape*>(shape.get()));

    const auto shapeMesh = static_cast<const MeshShape*>(shape.get());
    const auto scale = shapeMesh->getScale();
    const auto mesh = shapeMesh->getMesh();

    auto bulletCollisionShape
        = createBulletCollisionShapeFromAssimpScene(scale, mesh);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<SoftMeshShape>())
  {
    assert(dynamic_cast<const SoftMeshShape*>(shape.get()));

    const auto softMeshShape = static_cast<const SoftMeshShape*>(shape.get());
    const auto mesh = softMeshShape->getAssimpMesh();

    auto bulletCollisionShape = createBulletCollisionShapeFromAssimpMesh(mesh);

    return std::make_unique<BulletCollisionShape>(
        std::move(bulletCollisionShape));
  }
  else if (shape->is<HeightmapShapef>())
  {
    assert(dynamic_cast<const HeightmapShapef*>(shape.get()));

    const auto heightMap = static_cast<const HeightmapShapef*>(shape.get());

    return createBulletCollisionShapeFromHeightmap(heightMap);
  }
  else if (shape->is<HeightmapShaped>())
  {
    assert(dynamic_cast<const HeightmapShaped*>(shape.get()));

    dterr << "[BulletCollisionDetector::createBulletCollisionShape] "
          << "Bullet does not support double height fields (shape type ["
          << shape->getType() << "]). Creating a sphere with 0.1 radius "
          << "instead.\n";

    return std::make_unique<BulletCollisionShape>(
        std::make_unique<btSphereShape>(0.1));

    // take this back in as soon as bullet supports double in heightmaps
    // const auto heightMap = static_cast<const HeightmapShaped*>(shape.get());
    // return createBulletCollisionShapeFromHeightmap(heightMap);
  }
  else
  {
    dterr << "[BulletCollisionDetector::createBulletCollisionShape] "
          << "Attempting to create an unsupported shape type ["
          << shape->getType() << "] Creating a sphere with 0.1 radius "
          << "instead.\n";

    return std::make_unique<BulletCollisionShape>(
        std::make_unique<btSphereShape>(0.1));
  }
}

//==============================================================================
BulletCollisionDetector::BulletCollisionShapeDeleter ::
    BulletCollisionShapeDeleter(
        BulletCollisionDetector* cd, const dynamics::ConstShapePtr& shape)
  : mBulletCollisionDetector(cd), mShape(shape)
{
  // Do nothing
}

//==============================================================================
void BulletCollisionDetector::BulletCollisionShapeDeleter ::operator()(
    BulletCollisionShape* shape) const
{
  mBulletCollisionDetector->reclaimBulletCollisionShape(mShape);

  delete shape;
}

namespace {

//==============================================================================
Contact convertContact(
    const btManifoldPoint& bulletManifoldPoint,
    BulletCollisionObject* collObj1,
    BulletCollisionObject* collObj2)
{
  assert(collObj1);
  assert(collObj2);

  Contact contact;

  contact.point = convertVector3(bulletManifoldPoint.getPositionWorldOnA());
  contact.normal = convertVector3(bulletManifoldPoint.m_normalWorldOnB);
  contact.penetrationDepth = -bulletManifoldPoint.m_distance1;
  contact.collisionObject1 = collObj1;
  contact.collisionObject2 = collObj2;

  return contact;
}

//==============================================================================
void reportContacts(
    btCollisionWorld* world,
    const CollisionOption& option,
    CollisionResult& result)
{
  assert(world);

  auto dispatcher
      = static_cast<detail::BulletCollisionDispatcher*>(world->getDispatcher());
  assert(dispatcher);

  const auto numManifolds = dispatcher->getNumManifolds();

  for (auto i = 0; i < numManifolds; ++i)
  {
    const auto contactManifold = dispatcher->getManifoldByIndexInternal(i);

    const auto body0 = contactManifold->getBody0();
    const auto body1 = contactManifold->getBody1();

    const auto userPointer0 = body0->getUserPointer();
    const auto userPointer1 = body1->getUserPointer();

    const auto collObj0 = static_cast<BulletCollisionObject*>(userPointer0);
    const auto collObj1 = static_cast<BulletCollisionObject*>(userPointer1);

    const auto numContacts = contactManifold->getNumContacts();

    for (auto j = 0; j < numContacts; ++j)
    {
      const auto& cp = contactManifold->getContactPoint(j);

      if (cp.m_normalWorldOnB.length2() < Contact::getNormalEpsilonSquared())
      {
        // Skip this contact. This is because we assume that a contact with
        // zero-length normal is invalid.
        continue;
      }

      result.addContact(convertContact(cp, collObj0, collObj1));

      // No need to check further collisions
      if (result.getNumContacts() >= option.maxNumContacts)
      {
        dispatcher->setDone(true);
        return;
      }
    }
  }
}

//==============================================================================
RayHit convertRayHit(
    const btCollisionObject* btCollObj,
    btVector3 hitPointWorld,
    btVector3 hitNormalWorld,
    btScalar closestHitFraction)
{
  RayHit rayHit;
  assert(btCollObj);
  const auto* userPointer = btCollObj->getUserPointer();
  assert(userPointer);
  const auto* collObj = static_cast<const BulletCollisionObject*>(userPointer);
  assert(collObj);
  rayHit.mCollisionObject = collObj;
  rayHit.mPoint = convertVector3(hitPointWorld);
  rayHit.mNormal = convertVector3(hitNormalWorld);
  rayHit.mFraction = static_cast<double>(closestHitFraction);

  return rayHit;
}

//==============================================================================
void reportRayHits(
    const btCollisionWorld::ClosestRayResultCallback callback,
    const RaycastOption& /*option*/,
    RaycastResult& result)
{
  // This function shouldn't be called if callback has not ray hit.
  assert(callback.hasHit());

  const auto rayHit = convertRayHit(
      callback.m_collisionObject,
      callback.m_hitPointWorld,
      callback.m_hitNormalWorld,
      callback.m_closestHitFraction);

  result.mRayHits.clear();
  result.mRayHits.reserve(1);
  result.mRayHits.emplace_back(rayHit);
}

//==============================================================================
struct FractionLess
{
  bool operator()(const RayHit& a, const RayHit& b)
  {
    return a.mFraction < b.mFraction;
  }
};

//==============================================================================
void reportRayHits(
    const btCollisionWorld::AllHitsRayResultCallback callback,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.mRayHits.clear();
  result.mRayHits.reserve(
      static_cast<std::size_t>(callback.m_hitPointWorld.size()));

  for (auto i = 0; i < callback.m_hitPointWorld.size(); ++i)
  {
    const auto rayHit = convertRayHit(
        callback.m_collisionObjects[i],
        callback.m_hitPointWorld[i],
        callback.m_hitNormalWorld[i],
        callback.m_hitFractions[i]);
    result.mRayHits.emplace_back(rayHit);
  }

  if (option.mSortByClosest)
    std::sort(result.mRayHits.begin(), result.mRayHits.end(), FractionLess());
}

//==============================================================================
std::unique_ptr<btCollisionShape> createBulletEllipsoidMesh(
    float sizeX, float sizeY, float sizeZ)
{
  float v[59][3] = {{0, 0, 0},
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
                    {0.000000, 0.500000, 0.000000}};

  int f[112][3]
      = {{1, 2, 9},    {9, 2, 10},   {2, 3, 10},   {10, 3, 11},  {3, 4, 11},
         {11, 4, 12},  {4, 5, 12},   {12, 5, 13},  {5, 6, 13},   {13, 6, 14},
         {6, 7, 14},   {14, 7, 15},  {7, 8, 15},   {15, 8, 16},  {8, 1, 16},
         {16, 1, 9},   {9, 10, 17},  {17, 10, 18}, {10, 11, 18}, {18, 11, 19},
         {11, 12, 19}, {19, 12, 20}, {12, 13, 20}, {20, 13, 21}, {13, 14, 21},
         {21, 14, 22}, {14, 15, 22}, {22, 15, 23}, {15, 16, 23}, {23, 16, 24},
         {16, 9, 24},  {24, 9, 17},  {17, 18, 25}, {25, 18, 26}, {18, 19, 26},
         {26, 19, 27}, {19, 20, 27}, {27, 20, 28}, {20, 21, 28}, {28, 21, 29},
         {21, 22, 29}, {29, 22, 30}, {22, 23, 30}, {30, 23, 31}, {23, 24, 31},
         {31, 24, 32}, {24, 17, 32}, {32, 17, 25}, {25, 26, 33}, {33, 26, 34},
         {26, 27, 34}, {34, 27, 35}, {27, 28, 35}, {35, 28, 36}, {28, 29, 36},
         {36, 29, 37}, {29, 30, 37}, {37, 30, 38}, {30, 31, 38}, {38, 31, 39},
         {31, 32, 39}, {39, 32, 40}, {32, 25, 40}, {40, 25, 33}, {33, 34, 41},
         {41, 34, 42}, {34, 35, 42}, {42, 35, 43}, {35, 36, 43}, {43, 36, 44},
         {36, 37, 44}, {44, 37, 45}, {37, 38, 45}, {45, 38, 46}, {38, 39, 46},
         {46, 39, 47}, {39, 40, 47}, {47, 40, 48}, {40, 33, 48}, {48, 33, 41},
         {41, 42, 49}, {49, 42, 50}, {42, 43, 50}, {50, 43, 51}, {43, 44, 51},
         {51, 44, 52}, {44, 45, 52}, {52, 45, 53}, {45, 46, 53}, {53, 46, 54},
         {46, 47, 54}, {54, 47, 55}, {47, 48, 55}, {55, 48, 56}, {48, 41, 56},
         {56, 41, 49}, {2, 1, 57},   {3, 2, 57},   {4, 3, 57},   {5, 4, 57},
         {6, 5, 57},   {7, 6, 57},   {8, 7, 57},   {1, 8, 57},   {49, 50, 58},
         {50, 51, 58}, {51, 52, 58}, {52, 53, 58}, {53, 54, 58}, {54, 55, 58},
         {55, 56, 58}, {56, 49, 58}};

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

  auto gimpactMeshShape = std::make_unique<btGImpactMeshShape>(triMesh);
  gimpactMeshShape->updateBound();

  return gimpactMeshShape;
}

//==============================================================================
std::unique_ptr<btCollisionShape> createBulletCollisionShapeFromAssimpScene(
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
        const aiVector3D& vertex
            = scene->mMeshes[i]
                  ->mVertices[scene->mMeshes[i]->mFaces[j].mIndices[k]];
        vertices[k] = btVector3(
            vertex.x * scale[0], vertex.y * scale[1], vertex.z * scale[2]);
      }
      triMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  auto gimpactMeshShape = std::make_unique<btGImpactMeshShape>(triMesh);
  gimpactMeshShape->updateBound();
  gimpactMeshShape->setUserPointer(triMesh);

  return gimpactMeshShape;
}

//==============================================================================
std::unique_ptr<btCollisionShape> createBulletCollisionShapeFromAssimpMesh(
    const aiMesh* mesh)
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

  auto gimpactMeshShape = std::make_unique<btGImpactMeshShape>(triMesh);
  gimpactMeshShape->updateBound();

  return gimpactMeshShape;
}

//==============================================================================
template <typename HeightmapShapeT>
std::unique_ptr<BulletCollisionShape> createBulletCollisionShapeFromHeightmap(
    const HeightmapShapeT* heightMap)
{
  // get the heightmap parameters
  const auto& scale = heightMap->getScale();
  const auto minHeight = heightMap->getMinHeight();
  const auto maxHeight = heightMap->getMaxHeight();

  // determine which data type (float or double) is to be used for the field
  PHY_ScalarType scalarType = PHY_FLOAT;
  if (std::is_same<typename HeightmapShapeT::S, double>::value)
  {
    dterr << "Bullet does not support DOUBLE as heightmap field yet.\n";
    return nullptr;
    // take this back in as soon as it is supported
    // scalarType = PHY_DOUBLE;
  }

  // the y-values in the height field need to be flipped
  heightMap->flipY();

  const auto& heights = heightMap->getHeightField();

  // create the height field
  const btVector3 localScaling(scale.x(), scale.y(), scale.z());
  const bool flipQuadEdges = false;
  auto heightFieldShape = std::make_unique<btHeightfieldTerrainShape>(
      heightMap->getWidth(), // Width of height field
      heightMap->getDepth(), // Depth of height field
      heights.data(),        // Height values
      1,                     // Height scaling
      minHeight,             // Min height
      maxHeight,             // Max height
      2,                     // Up axis
      scalarType,            // Float or double field
      flipQuadEdges);        // Flip quad edges
  heightFieldShape->setLocalScaling(localScaling);
  heightFieldShape->setUseZigzagSubdivision(true);

  // change the relative transform of the height field so that the minimum
  // height is at the same z coordinate. Bullet shifts the height map such
  // that its center is the AABB center.
  const btVector3 trans(
      0, 0, ((maxHeight - minHeight) * 0.5 + minHeight) * scale.z());
  btTransform relativeShapeTransform(btMatrix3x3::getIdentity(), trans);

  // bullet places the heightfield such that the origin is in the
  // middle of the AABB. We want however that the minimum height value
  // is on x/y plane.
  btVector3 min;
  btVector3 max;
  heightFieldShape->getAabb(btTransform::getIdentity(), min, max);
  dtdbg << "DART Bullet heightfield AABB: min = {" << min.x() << ", " << min.y()
        << ", " << min.z() << "}, max = {" << max.x() << ", " << max.y() << ", "
        << max.z() << "}"
        << " (will be translated by z=" << trans.z() << ")" << std::endl;

  return std::make_unique<BulletCollisionShape>(
      std::move(heightFieldShape), relativeShapeTransform);
}

} // anonymous namespace

} // namespace collision
} // namespace dart
