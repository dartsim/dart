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

#ifndef DART_COLLISION_ODE_DETAIL_ODEGEOM_HPP_
#define DART_COLLISION_ODE_DETAIL_ODEGEOM_HPP_

#include <ode/ode.h>

#include "dart/collision/ode/OdeCollisionObject.hpp"
#include "dart/collision/ode/OdeCollisionDetector.hpp"

namespace dart {
namespace collision {
namespace detail {

class OdeGeom
{
public:
  struct GeomUserData;

  /// Constructor.
  OdeGeom(const OdeCollisionObject* collObj);

  /// Destructor.
  virtual ~OdeGeom();

  /// Returns the parent collision object.
  const OdeCollisionObject* getParentCollisionObject() const;

  // Documentation inherited.
  virtual void updateEngineData();

  /// Returns the ODE geom ID associated to this object.
  dGeomID getOdeGeomId() const;

  /// Returns true if the ODE geom is placeable.
  virtual bool isPlaceable() const;

protected:
  /// Parent collision object
  const OdeCollisionObject* mParentCollisionObject;

  /// ODE geom ID associated with this object.
  ///
  /// This geom ID should be set by the concrete classes such as OdeBox and
  /// OdeSphere.
  dGeomID mGeomId;
};

} // namespace detail
} // namespace collision
} // namespace dart

#endif  // DART_COLLISION_ODE_DETAIL_ODEGEOM_HPP_
