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

#include "dart/collision/CollisionObject.hpp"

#include "dart/collision/CollisionDetector.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

namespace dart {
namespace collision {

//==============================================================================
CollisionDetector* CollisionObject::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
const CollisionDetector* CollisionObject::getCollisionDetector() const
{
  return mCollisionDetector;
}

//==============================================================================
const dynamics::ShapeFrame* CollisionObject::getShapeFrame() const
{
  return mShapeFrame;
}

//==============================================================================
dynamics::ConstShapePtr CollisionObject::getShape() const
{
  return mShapeFrame->getShape();
}

//==============================================================================
const Eigen::Isometry3d& CollisionObject::getTransform() const
{
  return mShapeFrame->getWorldTransform();
}

//==============================================================================
CollisionObject::CollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : mCollisionDetector(collisionDetector),
    mShapeFrame(shapeFrame)
{
  assert(mCollisionDetector);
  assert(mShapeFrame);
}

}  // namespace collision
}  // namespace dart
