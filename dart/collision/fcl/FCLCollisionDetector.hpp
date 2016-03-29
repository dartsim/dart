/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang <ctang40@gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COLLISION_FCL_FCLCOLLISIONDETECTOR_H_
#define DART_COLLISION_FCL_FCLCOLLISIONDETECTOR_H_

#include <vector>
#include <fcl/collision_object.h>
#include "dart/collision/CollisionDetector.hpp"

namespace dart {
namespace collision {

class FCLCollisionObject;

class FCLCollisionDetector : public CollisionDetector
{
public:

  static std::shared_ptr<FCLCollisionDetector> create();

  /// Constructor
  virtual ~FCLCollisionDetector();

  /// Return engine type "FCL"
  static const std::string& getTypeStatic();

  // Documentation inherited
  const std::string& getType() const override;

  // Documentation inherited
  std::shared_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  bool detect(CollisionGroup* group,
              const Option& option, Result& result) override;

  // Documentation inherited
  bool detect(CollisionGroup* group1, CollisionGroup* group2,
              const Option& option, Result& result) override;

  /// FCL's primitive shape support is still not complete. FCL 0.4.0 fixed lots
  /// of the bugs, but it still returns single contact point per shape pair
  /// except for box-box collision. For this reason, we recommend using mesh for
  /// primitive shapes until FCL fully support primitive shapes.
  enum PrimitiveShape_t
  {
    MESH = 0,
    PRIMITIVE
  };

  /// Set primitive shape type
  void setPrimitiveShapeType(PrimitiveShape_t type);

  /// Get primitive shape type
  PrimitiveShape_t getPrimitiveShapeType() const;

  /// FCL's contact computation is still (at least until 0.4.0) not correct for
  /// mesh collision. We recommend using DART's contact point computation
  /// instead until it's fixed in FCL.
  enum ContactPointComputationMethod_t
  {
    DART = 0,
    FCL
  };

  /// Set contact point computation method
  void setContactPointComputationMethod(ContactPointComputationMethod_t method);

  /// Get contact point computation method
  ContactPointComputationMethod_t getContactPointComputationMethod() const;

protected:

  /// Constructor
  FCLCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  // Documentation inherited
  void notifyCollisionObjectDestorying(CollisionObject* collObj) override;

  ///
  boost::shared_ptr<fcl::CollisionGeometry> claimFCLCollisionGeometry(
      const dynamics::ConstShapePtr& shape);

  ///
  void reclaimFCLCollisionGeometry(const dynamics::ConstShapePtr& shape);

protected:

  using ShapeMapValue
      = std::pair<boost::shared_ptr<fcl::CollisionGeometry>, size_t>;

  std::map<dynamics::ConstShapePtr, ShapeMapValue> mShapeMap;

  std::map<fcl::CollisionObject*, FCLCollisionObject*> mFCLCollisionObjectMap;

  PrimitiveShape_t mPrimitiveShapeType;

  ContactPointComputationMethod_t mContactPointComputationMethod;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_FCLCOLLISIONDETECTOR_H_
