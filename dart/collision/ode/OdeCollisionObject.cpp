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

#include "dart/collision/ode/OdeCollisionObject.hpp"

#include "dart/dynamics/PlaneShape.hpp"

namespace dart {
namespace collision {

//==============================================================================
OdeCollisionObject::UserData::UserData(OdeCollisionObject* collisionObject)
  : mCollisionObject(collisionObject)
{
  // Do nothing
}

//==============================================================================
OdeCollisionObject::~OdeCollisionObject()
{
  if (mBodyId)
    dBodyDestroy(mBodyId);
  mBodyId = nullptr;

  if (mGeomId)
    dGeomDestroy(mGeomId);
  mGeomId = nullptr;
}

//==============================================================================
OdeCollisionObject::OdeCollisionObject(
    OdeCollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame,
    dGeomID odeCollGeom)
  : CollisionObject(collisionDetector, shapeFrame),
    mOdeCollisionObjectUserData(new UserData(this)),
    mGeomId(odeCollGeom),
    mBodyId(nullptr)
{
  // Plane of ODE is non-placable geometry and is not allowed to bind to a body.
  if (!shapeFrame->getShape()->is<dynamics::PlaneShape>())
  {
    mBodyId = dBodyCreate(collisionDetector->getOdeWorldId());
    dGeomSetBody(mGeomId, mBodyId);
  }

  dGeomSetData(mGeomId, mOdeCollisionObjectUserData.get());
}

//==============================================================================
void OdeCollisionObject::updateEngineData()
{
  if (mShapeFrame->getShape()->is<dynamics::PlaneShape>())
    return;

  const Eigen::Isometry3d& tf = getTransform();
  const Eigen::Vector3d pos = tf.translation();
  const Eigen::Quaterniond rot(tf.linear());

  dQuaternion quat;
  quat[0] = rot.w();
  quat[1] = rot.x();
  quat[2] = rot.y();
  quat[3] = rot.z();

  dGeomSetOffsetPosition(mGeomId, pos.x(), pos.y(), pos.z());
  dGeomSetOffsetRotation(mGeomId, quat);
}

//==============================================================================
dBodyID OdeCollisionObject::getOdeBodyId() const
{
  return mBodyId;
}

//==============================================================================
dGeomID OdeCollisionObject::getOdeGeomId() const
{
  return mGeomId;
}

}  // namespace collision
}  // namespace dart
