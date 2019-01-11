/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COLLISION_ODE_ODECOLLISIONOBJECT_HPP_
#define DART_COLLISION_ODE_ODECOLLISIONOBJECT_HPP_

#include <ode/ode.h>

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/ode/OdeCollisionDetector.hpp"

namespace dart {
namespace collision {

namespace detail {
class OdeGeom;
} // namespace detail

class OdeCollisionObject : public CollisionObject
{
public:
  friend class OdeCollisionDetector;
  friend class OdeCollisionGroup;

  /// Destructor
  virtual ~OdeCollisionObject();

protected:
  /// Constructor
  OdeCollisionObject(
      OdeCollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame);

  /// Move assignment operator. This is used to refresh OdeCollisionObjects when
  /// their underlying shape information needs to be updated.
  OdeCollisionObject& operator=(OdeCollisionObject&& other);

  // Documentation inherited
  void updateEngineData() override;

  /// Returns the ODE body id associated to this object
  dBodyID getOdeBodyId() const;

  /// Returns the ODE body id associated to this object
  dGeomID getOdeGeomId() const;

protected:
  /// ODE geom
  std::unique_ptr<detail::OdeGeom> mOdeGeom;

  /// ODE body id associated with this object
  ///
  /// If the ODE geom type is immobile, this is nullptr.
  dBodyID mBodyId;
};

} // namespace collision
} // namespace dart

#endif  // DART_COLLISION_ODE_ODECOLLISIONOBJECT_HPP_
