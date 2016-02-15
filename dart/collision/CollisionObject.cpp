/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/collision/CollisionObject.h"

#include "dart/collision/CollisionObjectData.h"

namespace dart {
namespace collision {

//==============================================================================
dynamics::ShapePtr CollisionObject::getShape() const
{
  return mShape;
}

//==============================================================================
//void CollisionObject::reportCollision()
//{
//  // Do nothing
//}

//==============================================================================
EngineType CollisionObject::getEngineType() const
{
  return mEngineType;
}

//==============================================================================
CollisionObjectData* CollisionObject::getEngineData() const
{
  return mEngineData.get();
}

//==============================================================================
void CollisionObject::updateEngineData()
{
  mEngineData->updateTransform(getTransform());
}

//==============================================================================
bool CollisionObject::detect(CollisionObject* other,
                             const Option& option,
                             Result& result)
{
  return Engine::detect(this, other, option, result);
}

//==============================================================================
CollisionObject::CollisionObject(const dynamics::ShapePtr& shape,
                                 EngineType type)
  : mShape(shape),
    mEngineType(type)
{
  mEngineData.reset(
        Engine::createCollisionObjectData(mEngineType, this, mShape));
}

//==============================================================================
FreeCollisionObject::FreeCollisionObject(const dynamics::ShapePtr& shape, const Eigen::Isometry3d& tf, EngineType type)
  : CollisionObject(shape, type),
    mW(Eigen::Isometry3d::Identity())
{
  // Do nothing
}

//==============================================================================
void FreeCollisionObject::setTransform(const Eigen::Isometry3d& tf)
{
  mW = tf;
}

//==============================================================================
void FreeCollisionObject::setRotation(const Eigen::Matrix3d& rotation)
{
  mW.linear() = rotation;
}

//==============================================================================
void FreeCollisionObject::setTranslation(const Eigen::Vector3d& translation)
{
  mW.translation() = translation;
}

//==============================================================================
const Eigen::Isometry3d FreeCollisionObject::getTransform() const
{
  return mW;
}

}  // namespace collision
}  // namespace dart
