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

#ifndef DART_COLLISION_ODE_ODECOLLISIONOBJECT_HPP_
#define DART_COLLISION_ODE_ODECOLLISIONOBJECT_HPP_

#include <ode/ode.h>

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/ode/OdeCollisionDetector.hpp"

namespace dart {
namespace collision {

class OdeCollisionObject : public CollisionObject
{
public:
  friend class OdeCollisionDetector;
  friend class OdeCollisionGroup;

  struct GeomUserData;

  /// Destructor
  virtual ~OdeCollisionObject();

protected:
  /// Constructor
  OdeCollisionObject(
      OdeCollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame,
      dGeomID odeCollGeom);

  // Documentation inherited
  void updateEngineData() override;

  /// Returns the ODE geom id associated to this object
  dGeomID getOdeGeomId() const;

  /// Returns the ODE body id associated to this object
  dBodyID getOdeBodyId() const;

protected:
  /// ODE collision geometry user data
  std::unique_ptr<GeomUserData> mOdeCollisionObjectUserData;

  /// ODE geom id associated with this object
  dGeomID mGeomId;

  /// ODE body id associated with this object
  ///
  /// If the ODE geom type is immobile, this is nullptr.
  dBodyID mBodyId;
};

/// OdeCollisionObject::GeomUserData to be embedded to ODE geom.
struct OdeCollisionObject::GeomUserData
{
  /// Collision object that will be stored in the ODE geom. This is necessary
  /// to know which collision object is associated with the ODE geom.
  OdeCollisionObject* mCollisionObject;

  /// Constructor
  GeomUserData(OdeCollisionObject* collisionObject);
};

} // namespace collision
} // namespace dart

#endif  // DART_COLLISION_ODE_ODECOLLISIONOBJECT_HPP_
