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

#ifndef DART_COLLISION_ODE_ODECOLLISIONDETECTOR_HPP_
#define DART_COLLISION_ODE_ODECOLLISIONDETECTOR_HPP_

#include <ode/ode.h>

#include "dart/collision/CollisionDetector.hpp"

#define MAX_COLLIDE_RETURNS 250

namespace dart {
namespace collision {

/// OdeCollisionDetector wraps the ODE collision detector.
///
/// The supported collision shape types are sphere, box, capsule, cylinder,
/// plane, and trimesh.
///
/// ODE additionally supports ray and heightfiled, but DART doesn't support them
/// yet.
class OdeCollisionDetector : public CollisionDetector
{
public:
  friend class OdeCollisionObject;

  static std::shared_ptr<OdeCollisionDetector> create();

  /// Constructor
  virtual ~OdeCollisionDetector();

  // Documentation inherited
  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects() override;

  // Documentation inherited
  const std::string& getType() const override;

  /// Get collision detector type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  std::unique_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  /// \warning Not implemented yet.
  double distance(
      CollisionGroup* group,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  /// \warning Not implemented yet.
  double distance(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

protected:
  /// Constructor
  OdeCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  dWorldID getOdeWorldId() const;

protected:
  /// Top-level world for all bodies
  dWorldID mWorldId;

private:
  dGeomID createOdeCollisionGeometry(const dynamics::ConstShapePtr& shape);

private:
  dContactGeom contactCollisions[MAX_COLLIDE_RETURNS];
  static Registrar<OdeCollisionDetector> mRegistrar;
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_ODE_ODECOLLISIONDETECTOR_HPP_
