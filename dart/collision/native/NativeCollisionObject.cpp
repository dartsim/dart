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

#include "dart/collision/native/NativeCollisionObject.hpp"

#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/native/detail/NativeShapeConversion.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

#include <limits>

namespace dart {
namespace collision {
namespace {

constexpr std::size_t kNoShapeId = std::numeric_limits<std::size_t>::max();

//==============================================================================
class DartFallbackCollisionObject final : public DARTCollisionObject
{
public:
  DartFallbackCollisionObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame)
    : DARTCollisionObject(collisionDetector, shapeFrame)
  {
    // Do nothing
  }

  void refresh()
  {
    updateEngineData();
  }
};

//==============================================================================
void refreshDartFallbackObject(DARTCollisionObject* object)
{
  if (object == nullptr)
    return;

  static_cast<DartFallbackCollisionObject*>(object)->refresh();
}

//==============================================================================
bool shapeUsesDartFallback(const dynamics::Shape* shape)
{
  if (shape == nullptr)
    return false;

  const auto& shapeType = shape->getType();
  if (shapeType == dynamics::SoftMeshShape::getStaticType())
    return true;

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto* ellipsoid = static_cast<const dynamics::EllipsoidShape*>(shape);
    return !ellipsoid->isSphere();
  }

  return false;
}

//==============================================================================
bool shapeUsesSoftMeshFallback(const dynamics::Shape* shape)
{
  return shape != nullptr
         && shape->getType() == dynamics::SoftMeshShape::getStaticType();
}

//==============================================================================
bool shapeIsPlane(const dynamics::Shape* shape)
{
  return shape != nullptr
         && shape->getType() == dynamics::PlaneShape::getStaticType();
}

//==============================================================================
native::Aabb getShapeAabb(
    const dynamics::Shape& shape, const Eigen::Isometry3d& transform)
{
  const auto& bounds = shape.getBoundingBox();
  return native::Aabb::transformed(
      native::Aabb(bounds.getMin(), bounds.getMax()), transform);
}

//==============================================================================
native::Aabb getDartFallbackAabb(
    const DARTCollisionObject& object, const Eigen::Isometry3d& transform)
{
  if (!object.hasFiniteCachedLocalBounds())
    return native::Aabb();

  return native::Aabb::transformed(
      native::Aabb(
          object.getCachedLocalBoundsMin(), object.getCachedLocalBoundsMax()),
      transform);
}

} // namespace

//==============================================================================
NativeCollisionObject::NativeCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame)
{
  updateEngineData();
}

//==============================================================================
NativeCollisionObject::~NativeCollisionObject() = default;

//==============================================================================
const native::Shape* NativeCollisionObject::getNativeShape() const
{
  return mNativeShape.get();
}

//==============================================================================
const Eigen::Isometry3d& NativeCollisionObject::getNativeTransform() const
{
  return mNativeTransform;
}

//==============================================================================
const native::Aabb& NativeCollisionObject::getNativeAabb() const
{
  return mNativeAabb;
}

//==============================================================================
DARTCollisionObject* NativeCollisionObject::getDartFallbackObject()
{
  if (!mDartFallbackObject) {
    mDartFallbackObject = std::make_unique<DartFallbackCollisionObject>(
        getCollisionDetector(), getShapeFrame());
  }

  return mDartFallbackObject.get();
}

//==============================================================================
bool NativeCollisionObject::usesDartFallbackShape() const
{
  return mUsesDartFallbackShape;
}

//==============================================================================
bool NativeCollisionObject::usesSoftMeshFallbackShape() const
{
  return mUsesSoftMeshFallbackShape;
}

//==============================================================================
bool NativeCollisionObject::isPlaneShape() const
{
  return mIsPlaneShape;
}

//==============================================================================
void NativeCollisionObject::updateEngineData()
{
  const auto shape = getShape();
  const auto* shapePtr = shape.get();
  const std::size_t shapeId = shapePtr ? shapePtr->getID() : kNoShapeId;
  const std::size_t shapeVersion = shapePtr ? shapePtr->getVersion() : 0u;

  mNativeTransform = getTransform();
  mUsesDartFallbackShape = shapeUsesDartFallback(shapePtr);
  mUsesSoftMeshFallbackShape = shapeUsesSoftMeshFallback(shapePtr);
  mIsPlaneShape = shapeIsPlane(shapePtr);
  const bool shapeChanged
      = shapeId != mLastKnownShapeId || shapeVersion != mLastKnownShapeVersion;

  if (mUsesDartFallbackShape) {
    mLastKnownShapeId = shapeId;
    mLastKnownShapeVersion = shapeVersion;
    mNativeShape.reset();
    auto* fallbackObject = getDartFallbackObject();
    if (mUsesSoftMeshFallbackShape || shapeChanged)
      refreshDartFallbackObject(fallbackObject);
    mNativeAabb
        = fallbackObject != nullptr ? getDartFallbackAabb(
              *fallbackObject, fallbackObject->getWorldTransformForCollision())
                                    : getShapeAabb(*shapePtr, mNativeTransform);
    return;
  }

  if (shapeChanged)
    rebuildNativeShape();

  if (mNativeShape) {
    mNativeAabb = native::Aabb::transformed(
        mNativeShape->computeLocalAabb(), mNativeTransform);
  } else {
    mNativeAabb = native::Aabb();
  }

  if (shapeChanged)
    refreshDartFallbackObject(mDartFallbackObject.get());
}

//==============================================================================
void NativeCollisionObject::rebuildNativeShape()
{
  const auto shape = getShape();
  mLastKnownShapeId = shape ? shape->getID() : kNoShapeId;
  mLastKnownShapeVersion = shape ? shape->getVersion() : 0u;

  if (!shape) {
    mNativeShape.reset();
    mNativeAabb = native::Aabb();
    return;
  }

  mNativeShape = detail::NativeShapeConversion::create(*shape);
}

} // namespace collision
} // namespace dart
