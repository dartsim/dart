/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/collision/bullet/BulletCollisionObject.hpp"

#include "dart/collision/bullet/BulletTypes.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"

namespace dart {
namespace collision {

//==============================================================================
btCollisionObject* BulletCollisionObject::getBulletCollisionObject()
{
  return mBulletCollisionObject.get();
}

//==============================================================================
const btCollisionObject* BulletCollisionObject::getBulletCollisionObject() const
{
  return mBulletCollisionObject.get();
}

//==============================================================================
BulletCollisionObject::BulletCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame,
    const std::shared_ptr<BulletCollisionShape>& bulletCollisionShape)
  : CollisionObject(collisionDetector, shapeFrame),
    mBulletCollisionShape(bulletCollisionShape),
    mBulletCollisionObject(new btCollisionObject())
{
  DART_ASSERT(bulletCollisionShape);

  mBulletCollisionObject->setCollisionShape(
      mBulletCollisionShape->mCollisionShape.get());

  mBulletCollisionObject->setUserPointer(this);

  updateTransformFastPath();
}

//==============================================================================
void BulletCollisionObject::updateTransformFastPath()
{
  mLastTransformFastPathVersion = mShapeFrame->getVersion();
  mUseBodyNodeWorldTransform = false;

  if (mBulletCollisionShape->mRelativeTransform)
    return;

  if (mShapeNode == nullptr || mBodyNode == nullptr)
    return;

  mUseBodyNodeWorldTransform = mShapeNode->getRelativeTransform().matrix()
                               == Eigen::Isometry3d::Identity().matrix();
}

//==============================================================================
void BulletCollisionObject::updateEngineData()
{
  if (mLastTransformFastPathVersion != mShapeFrame->getVersion())
    updateTransformFastPath();

  const Eigen::Isometry3d& shapeFrameTf
      = mUseBodyNodeWorldTransform ? mBodyNode->getWorldTransform()
                                   : mShapeFrame->getWorldTransform();
  const Eigen::Vector3d& translation = shapeFrameTf.translation();
  const Eigen::Matrix3d& rotation = shapeFrameTf.linear();

  btTransform worldTransform(
      btMatrix3x3(
          rotation(0, 0),
          rotation(0, 1),
          rotation(0, 2),
          rotation(1, 0),
          rotation(1, 1),
          rotation(1, 2),
          rotation(2, 0),
          rotation(2, 1),
          rotation(2, 2)),
      btVector3(translation.x(), translation.y(), translation.z()));

  if (mBulletCollisionShape->mRelativeTransform)
    worldTransform *= (*mBulletCollisionShape->mRelativeTransform);

  mBulletCollisionObject->setWorldTransform(worldTransform);
}

} // namespace collision
} // namespace dart
