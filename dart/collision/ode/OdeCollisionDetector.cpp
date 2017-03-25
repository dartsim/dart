/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/collision/ode/OdeCollisionDetector.hpp"

#include <ode/ode.h>

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
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/ode/OdeTypes.hpp"
#include "dart/collision/ode/OdeCollisionGroup.hpp"
#include "dart/collision/ode/OdeCollisionObject.hpp"

namespace dart {
namespace collision {

namespace {

void CollisionCallback(void *data, dGeomID o1, dGeomID o2);

void reportContacts(int numContacts,
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

  int numContacts;

  OdeCollisionCallbackData(
      const CollisionOption& option,
      CollisionResult* result)
    : option(option),
      result(result),
      done(false),
      numContacts(0)
  {
    // Do nothing
  }
};

} // anonymous namespace

//==============================================================================
std::shared_ptr<OdeCollisionDetector> OdeCollisionDetector::create()
{
  return std::shared_ptr<OdeCollisionDetector>(
      new OdeCollisionDetector());
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
OdeCollisionDetector::cloneWithoutCollisionObjects()
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
  return common::make_unique<OdeCollisionGroup>(shared_from_this());
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
      &data, CollisionCallback);

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
  auto odeCollGeom = createOdeCollisionGeometry(shapeFrame->getShape());

  return std::unique_ptr<OdeCollisionObject>(
        new OdeCollisionObject(this, shapeFrame, odeCollGeom));
}

//==============================================================================
dWorldID OdeCollisionDetector::getOdeWorldId() const
{
  return mWorldId;
}

//==============================================================================
dGeomID OdeCollisionDetector::createOdeCollisionGeometry(
    const dynamics::ConstShapePtr& shape)
{
  using dynamics::Shape;
  using dynamics::SphereShape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CapsuleShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  dGeomID odeGeomId = nullptr;
  const auto& shapeType = shape->getType();

  if (shape->is<SphereShape>())
  {
    const auto sphere = static_cast<const SphereShape*>(shape.get());
    const auto radius = sphere->getRadius();

    odeGeomId = dCreateSphere(0, radius);
  }
  else if (shape->is<BoxShape>())
  {
    const auto box = static_cast<const BoxShape*>(shape.get());
    const Eigen::Vector3d& size = box->getSize();

    odeGeomId = dCreateBox(0, size.x(), size.y(), size.z());
  }
  //else if (shape->is<EllipsoidShape>())
  //{
  //  auto ellipsoid = static_cast<const EllipsoidShape*>(shape.get());
  //  const Eigen::Vector3d& radii = ellipsoid->getRadii();
  //
  //  odeGeomId = dCreateEllipsoid(0, size.x(), size.y(), size.z());
  //
  //}
  // TODO(JS): ODE doesn't support ellipsoid
  else if (shape->is<CapsuleShape>())
  {
    const auto capsule = static_cast<const CapsuleShape*>(shape.get());
    const auto radius = capsule->getRadius();
    const auto height = capsule->getHeight();

    odeGeomId = dCreateCapsule(0, radius, height);
  }
  else if (shape->is<CylinderShape>())
  {
    const auto cylinder = static_cast<const CylinderShape*>(shape.get());
    const auto radius = cylinder->getRadius();
    const auto height = cylinder->getHeight();

    odeGeomId = dCreateCylinder(0, radius, height);
  }
  else if (shape->is<PlaneShape>())
  {
    const auto plane = static_cast<const PlaneShape*>(shape.get());
    const Eigen::Vector3d normal = plane->getNormal();
    const double offset = plane->getOffset();

    // TODO(JS): transform the normal and offset according to the transform
    // of the parent body.
    odeGeomId = dCreatePlane(0, normal.x(), normal.y(), normal.z(), -0.375);
  }
  //else if (MeshShape::getStaticType() == shapeType)
  //{
  //  auto shapeMesh = static_cast<const MeshShape*>(shape.get());
  //  const Eigen::Vector3d& scale = shapeMesh->getScale();
  //  auto aiScene = shapeMesh->getMesh();
  //
  //  odeGeomId = dCreateTriMesh(0, ...);
  //}
  // TODO(SJ): not implemented
  //else if (SoftMeshShape::getStaticType() == shapeType)
  //{
  //  auto softMeshShape = static_cast<const SoftMeshShape*>(shape.get());
  //  auto aiMesh = softMeshShape->getAssimpMesh();
  //
  //}
  // TODO(SJ): not implemented
  else
  {
    dterr << "[FCLCollisionDetector::createFCLCollisionGeometry] "
          << "Attempting to create an unsupported shape type ["
          << shapeType << "]. Creating a sphere with 0.1 radius "
          << "instead.\n";

    odeGeomId = dCreateSphere(0, 0.001);
  }

  return odeGeomId;
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

        auto& odeResult   = cdData->contactGeoms;
        auto* result      = cdData->result;
  const auto& option      = cdData->option;
  const auto& filter      = option.collisionFilter;

  auto geomData1 = dGeomGetData(o1);
  auto geomData2 = dGeomGetData(o2);

  auto userData1 = static_cast<OdeCollisionObject::GeomUserData*>(geomData1);
  auto userData2 = static_cast<OdeCollisionObject::GeomUserData*>(geomData2);
  assert(userData1);
  assert(userData2);

  auto collObj1 = userData1->mCollisionObject;
  auto collObj2 = userData2->mCollisionObject;
  assert(collObj1);
  assert(collObj2);

  if (filter && !filter->needCollision(collObj2, collObj1))
      return;

  // Perform narrow-phase collision detection
  auto numc = dCollide(
      o1, o2, MAX_COLLIDE_RETURNS, odeResult, sizeof(odeResult[0]));

  cdData->numContacts = numc;

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
  if (1u == option.maxNumContacts)
  {
    result.addContact(convertContact(contactGeoms[0], b1, b2, option));

    return;
  }

  for (auto i = 0; i < numContacts; ++i)
  {
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

  if (option.enableContact)
  {
    contact.point = OdeTypes::convertVector3(odeContact.pos);
    contact.normal = OdeTypes::convertVector3(odeContact.normal);
    contact.penetrationDepth = odeContact.depth;
  }

  return contact;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
