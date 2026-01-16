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

#include "dart/collision/ode/OdeCollisionObject.hpp"

#include "dart/collision/ode/OdeTypes.hpp"
#include "dart/collision/ode/detail/OdeBox.hpp"
#include "dart/collision/ode/detail/OdeCapsule.hpp"
#include "dart/collision/ode/detail/OdeCylinder.hpp"
#include "dart/collision/ode/detail/OdeCylinderMesh.hpp"
#include "dart/collision/ode/detail/OdeHeightmap.hpp"
#include "dart/collision/ode/detail/OdeMesh.hpp"
#include "dart/collision/ode/detail/OdePlane.hpp"
#include "dart/collision/ode/detail/OdeSphere.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/HeightmapShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <ode/ode.h>

#include <cmath>

namespace dart {
namespace collision {

namespace {

bool gCylinderCollisionSupportKnown = false;
bool gCylinderCollisionSupported = true;

bool hasAlignedContactNormal(
    const dContactGeom* contacts,
    int numContacts,
    int axis,
    double minAxisAlignment,
    double maxOtherAlignment)
{
  for (auto i = 0; i < numContacts; ++i) {
    const double nx = contacts[i].normal[0];
    const double ny = contacts[i].normal[1];
    const double nz = contacts[i].normal[2];

    if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz))
      continue;

    const double absX = std::abs(nx);
    const double absY = std::abs(ny);
    const double absZ = std::abs(nz);

    double absAxis = 0.0;
    double maxOther = 0.0;
    if (axis == 0) {
      absAxis = absX;
      maxOther = absY > absZ ? absY : absZ;
    } else if (axis == 1) {
      absAxis = absY;
      maxOther = absX > absZ ? absX : absZ;
    } else {
      absAxis = absZ;
      maxOther = absX > absY ? absX : absY;
    }

    if (absAxis >= minAxisAlignment && maxOther <= maxOtherAlignment)
      return true;
  }

  return false;
}

// Some ODE builds report cylinder contacts with incorrect normals.
bool probeCylinderCollisionSupport()
{
  constexpr double kMinAxisAlignment = 0.9;
  constexpr double kMaxOtherAlignment = 0.2;

  bool cylinderCylinderOk = false;
  {
    dGeomID cylinder1 = dCreateCylinder(0, 1.0, 1.0);
    dGeomID cylinder2 = dCreateCylinder(0, 0.5, 1.0);

    dGeomSetPosition(cylinder1, 0.0, 0.0, 0.0);
    dGeomSetPosition(cylinder2, 0.75, 0.0, 0.0);

    dContactGeom contacts[4];
    const int numContacts
        = dCollide(cylinder1, cylinder2, 4, contacts, sizeof(contacts[0]));

    cylinderCylinderOk = hasAlignedContactNormal(
        contacts, numContacts, 0, kMinAxisAlignment, kMaxOtherAlignment);

    dGeomDestroy(cylinder1);
    dGeomDestroy(cylinder2);
  }

  bool cylinderPlaneOk = false;
  {
    dGeomID cylinder = dCreateCylinder(0, 1.0, 1.0);
    dGeomID plane = dCreatePlane(0, 0.0, 0.0, 1.0, 0.0);

    dGeomSetPosition(cylinder, 0.0, 0.0, 0.4);

    dContactGeom contacts[4];
    const int numContacts
        = dCollide(cylinder, plane, 4, contacts, sizeof(contacts[0]));

    cylinderPlaneOk = hasAlignedContactNormal(
        contacts, numContacts, 2, kMinAxisAlignment, kMaxOtherAlignment);

    dGeomDestroy(cylinder);
    dGeomDestroy(plane);
  }

  return cylinderCylinderOk && cylinderPlaneOk;
}

bool cylinderCollisionSupported()
{
  if (!gCylinderCollisionSupportKnown) {
    gCylinderCollisionSupported = probeCylinderCollisionSupport();
    gCylinderCollisionSupportKnown = true;
  }

  return gCylinderCollisionSupported;
}

} // namespace

//==============================================================================
static detail::OdeGeom* createOdeGeom(
    OdeCollisionObject* collObj, const dynamics::ShapeFrame* shapeFrame);

//==============================================================================
OdeCollisionObject::~OdeCollisionObject()
{
  if (mBodyId)
    dBodyDestroy(mBodyId);
}

//==============================================================================
OdeCollisionObject::OdeCollisionObject(
    OdeCollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame),
    mOdeGeom(nullptr),
    mBodyId(nullptr)
{
  // Create detail::OdeGeom according to the shape type.
  // The geometry may have a transform assigned to it which is to
  // be treated as relative transform to the main body.
  mOdeGeom.reset(createOdeGeom(this, shapeFrame));

  const auto geomId = mOdeGeom->getOdeGeomId();
  DART_ASSERT(geomId);

  if (mOdeGeom->isPlaceable()) {
    // if the geometry already has a pose, it is to be considered
    // a constant relative pose to the body.
    // Get the geometry pose to ensure this offset is set correctly.
    // Assigning a body to the geometry will overwrite the geometry
    // pose, so back it up first.
    dQuaternion geomRelRot;
    dGeomGetQuaternion(geomId, geomRelRot);
    const dReal* geomRelPos = dGeomGetPosition(geomId);
    DART_ASSERT(geomRelPos);

    // create the body
    mBodyId = dBodyCreate(collisionDetector->getOdeWorldId());
    // attach geometry to body. This will set the geometry pose to identity.
    dGeomSetBody(geomId, mBodyId);

    // set the offset
    dGeomSetOffsetPosition(geomId, geomRelPos[0], geomRelPos[1], geomRelPos[2]);
    dGeomSetOffsetQuaternion(geomId, geomRelRot);
  }
}

//==============================================================================
OdeCollisionObject& OdeCollisionObject::operator=(OdeCollisionObject&& other)
{
  // This should only be used for refreshing the collision objects, so the
  // detector and the shape frame should never need to change
  DART_ASSERT(mCollisionDetector == other.mCollisionDetector);
  DART_ASSERT(mShapeFrame == other.mShapeFrame);

  // We should never be assigning a collision object to itself
  DART_ASSERT(this != &other);

  // There should never be duplicate body IDs or geometries
  DART_ASSERT(!mBodyId || mBodyId != other.mBodyId);
  DART_ASSERT(mOdeGeom.get() != other.mOdeGeom.get());

  mOdeGeom = std::move(other.mOdeGeom);
  std::swap(mBodyId, other.mBodyId);

  return *this;
}

//==============================================================================
void OdeCollisionObject::updateEngineData()
{
  mOdeGeom->updateEngineData();

  // If body id is nullptr, this object is immobile. Immobile geom doesn't need
  // to update its pose.
  if (!mBodyId)
    return;

  const Eigen::Isometry3d& tf = getTransform();

  // Set position
  const Eigen::Vector3d pos = tf.translation();
  dBodySetPosition(mBodyId, pos.x(), pos.y(), pos.z());

  // Set orientation
  const Eigen::Quaterniond rot(tf.linear());
  dQuaternion odeQuat;
  odeQuat[0] = rot.w();
  odeQuat[1] = rot.x();
  odeQuat[2] = rot.y();
  odeQuat[3] = rot.z();
  dBodySetQuaternion(mBodyId, odeQuat);
}

//==============================================================================
dBodyID OdeCollisionObject::getOdeBodyId() const
{
  return mBodyId;
}

//==============================================================================
dGeomID OdeCollisionObject::getOdeGeomId() const
{
  return mOdeGeom->getOdeGeomId();
}

//==============================================================================
detail::OdeGeom* createOdeGeom(
    OdeCollisionObject* collObj, const dynamics::ShapeFrame* shapeFrame)
{
  using dynamics::BoxShape;
  using dynamics::CapsuleShape;
  using dynamics::CylinderShape;
  using dynamics::EllipsoidShape;
  using dynamics::HeightmapShaped;
  using dynamics::HeightmapShapef;
  using dynamics::MeshShape;
  using dynamics::PlaneShape;
  using dynamics::Shape;
  using dynamics::SoftMeshShape;
  using dynamics::SphereShape;

  detail::OdeGeom* geom = nullptr;
  const auto shape = shapeFrame->getShape().get();

  if (const auto sphere = shape->as<SphereShape>()) {
    const auto radius = sphere->getRadius();

    geom = new detail::OdeSphere(collObj, radius);
  } else if (const auto box = shape->as<BoxShape>()) {
    const Eigen::Vector3d& size = box->getSize();

    geom = new detail::OdeBox(collObj, size);
  } else if (const auto capsule = shape->as<CapsuleShape>()) {
    const auto radius = capsule->getRadius();
    const auto height = capsule->getHeight();

    geom = new detail::OdeCapsule(collObj, radius, height);
  } else if (const auto cylinder = shape->as<CylinderShape>()) {
    const auto radius = cylinder->getRadius();
    const auto height = cylinder->getHeight();

    if (cylinderCollisionSupported())
      geom = new detail::OdeCylinder(collObj, radius, height);
    else
      // TODO(JS): Replace the trimesh fallback once ODE cylinder contacts are
      // reliable across ports.
      geom = new detail::OdeCylinderMesh(collObj, radius, height);
  } else if (const auto plane = shape->as<PlaneShape>()) {
    const Eigen::Vector3d normal = plane->getNormal();
    const double offset = plane->getOffset();

    geom = new detail::OdePlane(collObj, normal, offset);
  } else if (const auto shapeMesh = shape->as<MeshShape>()) {
    const Eigen::Vector3d& scale = shapeMesh->getScale();
    auto aiScene = shapeMesh->getMesh();

    geom = new detail::OdeMesh(collObj, aiScene, scale);
  } else if (const auto heightMap = shape->as<HeightmapShapef>()) {
    geom = new detail::OdeHeightmapf(collObj, heightMap);
  } else if (const auto heightMap = shape->as<HeightmapShaped>()) {
    geom = new detail::OdeHeightmapd(collObj, heightMap);
  } else {
    dterr << "[OdeCollisionDetector] Attempting to create an unsupported shape "
          << "type '" << shape->getType() << "'. Creating a sphere with 0.01 "
          << "radius instead.\n";

    geom = new detail::OdeSphere(collObj, 0.01);
  }
  // TODO(JS): not implemented for EllipsoidShape, ConeShape, MultiSphereShape,
  // and SoftMeshShape.

  DART_ASSERT(geom);
  const auto geomId = geom->getOdeGeomId();
  dGeomSetData(geomId, collObj);

  return geom;
}

} // namespace collision
} // namespace dart
