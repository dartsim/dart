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

#include "dart/collision/dart/DARTCollisionObject.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/math/Geometry.hpp"

namespace dart {
namespace collision {

//==============================================================================
DARTCollisionObject::DARTCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame)
{
  refreshShapeCache();
}

//==============================================================================
const dynamics::Shape* DARTCollisionObject::getCachedShape() const
{
  return mCachedShape.get();
}

//==============================================================================
const std::string& DARTCollisionObject::getCachedShapeType() const
{
  static const std::string empty;
  return mCachedShapeType ? *mCachedShapeType : empty;
}

//==============================================================================
DARTCollisionObject::CachedShapeKind DARTCollisionObject::getCachedShapeKind()
    const
{
  return mCachedShapeKind;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsMin() const
{
  return mCachedLocalBoundsMin;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsMax() const
{
  return mCachedLocalBoundsMax;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsCenter() const
{
  return mCachedLocalBoundsCenter;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsHalfExtents()
    const
{
  return mCachedLocalBoundsHalfExtents;
}

//==============================================================================
bool DARTCollisionObject::hasFiniteCachedLocalBounds() const
{
  return mHasFiniteCachedLocalBounds;
}

//==============================================================================
bool DARTCollisionObject::isCachedPlaneShape() const
{
  return mIsCachedPlaneShape;
}

//==============================================================================
const Eigen::Isometry3d& DARTCollisionObject::getWorldTransformForCollision()
    const
{
  return mUseBodyNodeWorldTransform ? mBodyNode->getWorldTransform()
                                    : getTransform();
}

//==============================================================================
void DARTCollisionObject::refreshShapeCacheForCollision()
{
  refreshShapeCache();
}

//==============================================================================
void DARTCollisionObject::refreshShapeCache()
{
  const auto shapeFrameVersion = mShapeFrame ? mShapeFrame->getVersion() : 0u;
  if (mCachedShapeFrameVersion == shapeFrameVersion)
    return;

  mCachedShapeFrameVersion = shapeFrameVersion;
  mCachedShape = mShapeFrame ? mShapeFrame->getShape() : nullptr;
  mCachedShapeType = mCachedShape ? &mCachedShape->getType() : nullptr;
  mCachedShapeKind = CachedShapeKind::Unknown;
  mCachedLocalBoundsMin.setZero();
  mCachedLocalBoundsMax.setZero();
  mCachedLocalBoundsCenter.setZero();
  mCachedLocalBoundsHalfExtents.setZero();
  mHasFiniteCachedLocalBounds = false;
  mIsCachedPlaneShape = false;
  mUseBodyNodeWorldTransform = false;

  if (mShapeNode != nullptr && mBodyNode != nullptr) {
    mUseBodyNodeWorldTransform = mShapeNode->getRelativeTransform().matrix()
                                 == Eigen::Isometry3d::Identity().matrix();
  }

  if (!mCachedShape)
    return;

  const auto& shapeType = *mCachedShapeType;
  if (shapeType == dynamics::PlaneShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Plane;
    mIsCachedPlaneShape = true;
    return;
  }

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Sphere;
  } else if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto* ellipsoid
        = static_cast<const dynamics::EllipsoidShape*>(mCachedShape.get());
    if (ellipsoid->isSphere())
      mCachedShapeKind = CachedShapeKind::SphereEllipsoid;
  } else if (shapeType == dynamics::BoxShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Box;
  } else if (shapeType == dynamics::CylinderShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Cylinder;
  } else if (shapeType == dynamics::CapsuleShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Capsule;
  }

  const auto& localBox = mCachedShape->getBoundingBox();
  mCachedLocalBoundsMin = localBox.getMin();
  mCachedLocalBoundsMax = localBox.getMax();
  mCachedLocalBoundsCenter
      = 0.5 * (mCachedLocalBoundsMin + mCachedLocalBoundsMax);
  mCachedLocalBoundsHalfExtents
      = 0.5 * (mCachedLocalBoundsMax - mCachedLocalBoundsMin);
  mHasFiniteCachedLocalBounds
      = mCachedLocalBoundsMin.allFinite() && mCachedLocalBoundsMax.allFinite();
}

//==============================================================================
void DARTCollisionObject::updateEngineData()
{
  refreshShapeCacheForCollision();
}

} // namespace collision
} // namespace dart
