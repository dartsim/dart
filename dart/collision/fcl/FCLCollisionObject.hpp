/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_COLLISION_FCL_FCLCOLLISIONOBJECT_HPP_
#define DART_COLLISION_FCL_FCLCOLLISIONOBJECT_HPP_

#include <fcl/collision_object.h>
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/fcl/FCLTypes.hpp"

namespace dart {
namespace collision {

class CollisionObject;

class FCLCollisionObject : public CollisionObject
{
public:

  friend class FCLCollisionDetector;

  struct UserData
  {
    CollisionObject* mCollisionObject;

    UserData(CollisionObject* collisionObject);
  };

  /// Return FCL collision object
  fcl::CollisionObject* getFCLCollisionObject();

  /// Return FCL collision object
  const fcl::CollisionObject* getFCLCollisionObject() const;

protected:

  /// Constructor
  FCLCollisionObject(CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame,
      const fcl_shared_ptr<fcl::CollisionGeometry>& fclCollGeom);

  // Documentation inherited
  void updateEngineData() override;

protected:

  /// FCL collision geometry user data
  std::unique_ptr<UserData> mFCLCollisionObjectUserData;

  /// FCL collision object
  std::unique_ptr<fcl::CollisionObject> mFCLCollisionObject;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_FCLCOLLISIONOBJECT_HPP_
