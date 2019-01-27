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

#ifndef DART_COLLISION_COLLISIONOBJECT_HPP_
#define DART_COLLISION_COLLISIONOBJECT_HPP_

#include <Eigen/Dense>

#include "dart/collision/SmartPointer.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace collision {

class CollisionObject
{
public:

  friend class CollisionGroup;

  /// Destructor
  virtual ~CollisionObject() = default;

  /// Return collision detection engine associated with this CollisionObject
  CollisionDetector* getCollisionDetector();

  /// Return collision detection engine associated with this CollisionObject
  const CollisionDetector* getCollisionDetector() const;

  /// Return the associated ShapeFrame
  const dynamics::ShapeFrame* getShapeFrame() const;

  /// Return the associated Shape
  dynamics::ConstShapePtr getShape() const;

  /// Return the transformation of this CollisionObject in world coordinates
  const Eigen::Isometry3d& getTransform() const;

protected:

  /// Contructor
  CollisionObject(CollisionDetector* collisionDetector,
                  const dynamics::ShapeFrame* shapeFrame);

  /// Update the collision object of the collision detection engine. This
  /// function will be called ahead of every collision checking by
  /// CollisionGroup.
  virtual void updateEngineData() = 0;

protected:

  /// Collision detector
  CollisionDetector* mCollisionDetector;

  /// ShapeFrame
  const dynamics::ShapeFrame* mShapeFrame;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONOBJECT_HPP_
