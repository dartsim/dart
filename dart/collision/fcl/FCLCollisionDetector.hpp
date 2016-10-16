/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_COLLISION_FCL_FCLCOLLISIONDETECTOR_HPP_
#define DART_COLLISION_FCL_FCLCOLLISIONDETECTOR_HPP_

#include <vector>
#include <fcl/collision_object.h>
#include <boost/weak_ptr.hpp> // This should be removed once we migrate to fcl 0.5
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/fcl/FCLTypes.hpp"

namespace dart {
namespace collision {

class FCLCollisionObject;

class FCLCollisionDetector : public CollisionDetector
{
public:

  static std::shared_ptr<FCLCollisionDetector> create();

  /// Whether to use analytic collision checking for primitive shapes.
  ///
  /// PRIMITIVE: Use FCL's analytic collision checking for primitive shapes.
  /// MESH: Don't use it. Instead, use approximate mesh shapes for the primitive
  /// shapes. The contact result is probably less accurate than the analytic
  /// result.
  ///
  /// Warning: FCL's primitive shape support is not complete. FCL 0.4.0 improved
  /// the support alot, but it still returns single contact point for a shape
  /// pair except for box-box collision. For this reason, we recommend using
  /// MESH until FCL fully supports primitive shapes.
  enum PrimitiveShape
  {
    PRIMITIVE = 0,
    MESH
  };

  /// Whether to use FCL's contact point computation.
  ///
  /// FCL: Use FCL's contact point computation.
  /// DART: Use DART's own contact point computation
  ///
  /// Warning: FCL's contact computation is not correct. See:
  /// https://github.com/flexible-collision-library/fcl/issues/106
  /// We recommend using DART until it's fixed in FCL.
  enum ContactPointComputationMethod
  {
    FCL = 0,
    DART
  };

  /// Constructor
  virtual ~FCLCollisionDetector();

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

  // Documentation inherited
  double distance(
      CollisionGroup* group,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  // Documentation inherited
  double distance(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  /// Set primitive shape type
  void setPrimitiveShapeType(PrimitiveShape type);

  /// Get primitive shape type
  PrimitiveShape getPrimitiveShapeType() const;

  /// Set contact point computation method
  void setContactPointComputationMethod(ContactPointComputationMethod method);

  /// Get contact point computation method
  ContactPointComputationMethod getContactPointComputationMethod() const;

protected:

  /// Constructor
  FCLCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  /// Return fcl::CollisionGeometry associated with give Shape. New
  /// fcl::CollisionGeome will be created if it hasn't created yet.
  fcl_shared_ptr<fcl::CollisionGeometry> claimFCLCollisionGeometry(
      const dynamics::ConstShapePtr& shape);

protected:

  PrimitiveShape mPrimitiveShapeType;

  ContactPointComputationMethod mContactPointComputationMethod;

private:

  /// This deleter is responsible for deleting fcl::CollisionGeometry and
  /// removing it from mShapeMap when it is not shared by any CollisionObjects.
  class FCLCollisionGeometryDeleter final
  {
  public:

    FCLCollisionGeometryDeleter(FCLCollisionDetector* cd,
                                const dynamics::ConstShapePtr& shape);

    void operator()(fcl::CollisionGeometry* geom) const;

  private:

    FCLCollisionDetector* mFCLCollisionDetector;

    dynamics::ConstShapePtr mShape;

  };

  /// Create fcl::CollisionGeometry with the custom deleter
  /// FCLCollisionGeometryDeleter
  fcl_shared_ptr<fcl::CollisionGeometry> createFCLCollisionGeometry(
      const dynamics::ConstShapePtr& shape,
      FCLCollisionDetector::PrimitiveShape type,
      const FCLCollisionGeometryDeleter& deleter);

private:

  using ShapeMap = std::map<dynamics::ConstShapePtr,
                            fcl_weak_ptr<fcl::CollisionGeometry>>;
  // TODO(JS): FCL replaced all the use of boost in version 0.5. Once we migrate
  // to 0.5 or greater, this also should be changed to
  // std::weak_ptr<fcl::CollisionGeometry>

  ShapeMap mShapeMap;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_FCLCOLLISIONDETECTOR_HPP_
