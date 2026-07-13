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
#include <utility>

namespace dart {
namespace collision {
namespace {

constexpr std::size_t kNoShapeId = std::numeric_limits<std::size_t>::max();

// Keep this snapshot in sync with the released DART 6.20 object layout. The
// fallback bridge must reuse existing storage instead of growing the exported,
// subclassable NativeCollisionObject base.
class NativeCollisionObject620Layout : public CollisionObject
{
private:
  void updateEngineData() override
  {
    // Do nothing
  }

  std::unique_ptr<native::Shape> mNativeShape;
  Eigen::Isometry3d mNativeTransform;
  native::Aabb mNativeLocalAabb;
  native::Aabb mNativeAabb;
  [[maybe_unused]] std::size_t mLastKnownShapeId;
  [[maybe_unused]] std::size_t mLastKnownShapeVersion;
};

static_assert(
    sizeof(NativeCollisionObject) == sizeof(NativeCollisionObject620Layout),
    "NativeCollisionObject must preserve its released DART 6.20 ABI layout");
static_assert(
    alignof(NativeCollisionObject) == alignof(NativeCollisionObject620Layout),
    "NativeCollisionObject must preserve its released DART 6.20 alignment");

//==============================================================================
class DartFallbackCollisionObject final : public DARTCollisionObject
{
public:
  DartFallbackCollisionObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame)
    : DARTCollisionObject(collisionDetector, shapeFrame),
      mLastKnownShapeFrameVersion(shapeFrame->getVersion())
  {
    // Do nothing
  }

  void refresh(bool force)
  {
    const auto shapeFrameVersion = getShapeFrame()->getVersion();
    if (!force && shapeFrameVersion == mLastKnownShapeFrameVersion)
      return;

    updateEngineData();
    mLastKnownShapeFrameVersion = shapeFrameVersion;
  }

private:
  std::size_t mLastKnownShapeFrameVersion;
};

//==============================================================================
// Object state is stored behind the existing mNativeShape slot so old
// downstream subclasses keep the allocation size and member offsets they were
// compiled against. The wrapper adds one allocation when an object is created,
// then keeps the per-step path lock-free and preserves native/fallback object
// identity across shape refreshes.
class NativeCollisionObjectState final : public native::Shape
{
public:
  native::Shape* getNativeShape()
  {
    return mNativeShape.get();
  }

  const native::Shape* getNativeShape() const
  {
    return mNativeShape.get();
  }

  void setNativeShape(std::unique_ptr<native::Shape> nativeShape)
  {
    mNativeShape = std::move(nativeShape);
  }

  void setShapeClassification(
      bool usesDartFallbackShape,
      bool usesSoftMeshFallbackShape,
      bool isPlaneShape)
  {
    mUsesDartFallbackShape = usesDartFallbackShape;
    mUsesSoftMeshFallbackShape = usesSoftMeshFallbackShape;
    mIsPlaneShape = isPlaneShape;
  }

  bool usesDartFallbackShape() const
  {
    return mUsesDartFallbackShape;
  }

  bool usesSoftMeshFallbackShape() const
  {
    return mUsesSoftMeshFallbackShape;
  }

  bool isPlaneShape() const
  {
    return mIsPlaneShape;
  }

  DARTCollisionObject* getDartFallbackObject() const
  {
    return mDartFallbackObject.get();
  }

  DARTCollisionObject* getOrCreateDartFallbackObject(
      CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame)
  {
    if (!mDartFallbackObject) {
      mDartFallbackObject = std::make_unique<DartFallbackCollisionObject>(
          collisionDetector, shapeFrame);
    }

    return mDartFallbackObject.get();
  }

  native::ShapeType getType() const override
  {
    return mNativeShape ? mNativeShape->getType() : native::ShapeType::Compound;
  }

  native::Aabb computeLocalAabb() const override
  {
    return mNativeShape ? mNativeShape->computeLocalAabb() : native::Aabb();
  }

private:
  std::unique_ptr<native::Shape> mNativeShape;
  std::unique_ptr<DARTCollisionObject> mDartFallbackObject;
  bool mUsesDartFallbackShape{false};
  bool mUsesSoftMeshFallbackShape{false};
  bool mIsPlaneShape{false};
};

//==============================================================================
const NativeCollisionObjectState* findNativeCollisionObjectState(
    const std::unique_ptr<native::Shape>& state)
{
  return static_cast<const NativeCollisionObjectState*>(state.get());
}

//==============================================================================
NativeCollisionObjectState& getOrCreateNativeCollisionObjectState(
    std::unique_ptr<native::Shape>& state)
{
  if (!state)
    state = std::make_unique<NativeCollisionObjectState>();

  return *static_cast<NativeCollisionObjectState*>(state.get());
}

//==============================================================================
void refreshDartFallbackObject(DARTCollisionObject* object, bool force)
{
  if (object == nullptr)
    return;

  static_cast<DartFallbackCollisionObject*>(object)->refresh(force);
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
  : CollisionObject(collisionDetector, shapeFrame),
    mNativeShape(std::make_unique<NativeCollisionObjectState>())
{
  updateEngineData();
}

//==============================================================================
const native::Shape* NativeCollisionObject::getNativeShape() const
{
  const auto* state = findNativeCollisionObjectState(mNativeShape);
  return state ? state->getNativeShape() : nullptr;
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
  auto& state = getOrCreateNativeCollisionObjectState(mNativeShape);
  return state.getOrCreateDartFallbackObject(
      getCollisionDetector(), getShapeFrame());
}

//==============================================================================
bool NativeCollisionObject::usesDartFallbackShape() const
{
  const auto* state = findNativeCollisionObjectState(mNativeShape);
  return state && state->usesDartFallbackShape();
}

//==============================================================================
bool NativeCollisionObject::usesSoftMeshFallbackShape() const
{
  const auto* state = findNativeCollisionObjectState(mNativeShape);
  return state && state->usesSoftMeshFallbackShape();
}

//==============================================================================
bool NativeCollisionObject::isPlaneShape() const
{
  const auto* state = findNativeCollisionObjectState(mNativeShape);
  return state && state->isPlaneShape();
}

//==============================================================================
void NativeCollisionObject::updateEngineData()
{
  const bool stateMissing = !mNativeShape;
  const auto shape = getShape();
  const auto* shapePtr = shape.get();
  const std::size_t shapeId = shapePtr ? shapePtr->getID() : kNoShapeId;
  const std::size_t shapeVersion = shapePtr ? shapePtr->getVersion() : 0u;
  const bool shapeChanged = stateMissing || shapeId != mLastKnownShapeId
                            || shapeVersion != mLastKnownShapeVersion;
  auto& state = getOrCreateNativeCollisionObjectState(mNativeShape);
  if (shapeChanged) {
    state.setShapeClassification(
        shapeUsesDartFallback(shapePtr),
        shapeUsesSoftMeshFallback(shapePtr),
        shapeIsPlane(shapePtr));
  }

  if (state.usesDartFallbackShape()) {
    mLastKnownShapeId = shapeId;
    mLastKnownShapeVersion = shapeVersion;
    state.setNativeShape(nullptr);

    auto* fallbackObject = getDartFallbackObject();
    refreshDartFallbackObject(
        fallbackObject, state.usesSoftMeshFallbackShape() || shapeChanged);
    mNativeTransform = fallbackObject->getWorldTransformForCollision();
    mNativeAabb = getDartFallbackAabb(*fallbackObject, mNativeTransform);
    return;
  }

  mNativeTransform = getTransform();

  if (shapeChanged)
    rebuildNativeShape();

  const auto* nativeShape = state.getNativeShape();
  if (nativeShape != nullptr) {
    mNativeAabb = native::Aabb::transformed(mNativeLocalAabb, mNativeTransform);
  } else {
    mNativeAabb = native::Aabb();
  }

  refreshDartFallbackObject(state.getDartFallbackObject(), shapeChanged);
}

//==============================================================================
void NativeCollisionObject::rebuildNativeShape()
{
  const auto shape = getShape();
  mLastKnownShapeId = shape ? shape->getID() : kNoShapeId;
  mLastKnownShapeVersion = shape ? shape->getVersion() : 0u;
  auto& state = getOrCreateNativeCollisionObjectState(mNativeShape);

  if (!shape) {
    state.setNativeShape(nullptr);

    mNativeLocalAabb = native::Aabb();
    mNativeAabb = native::Aabb();
    return;
  }

  auto nativeShape = detail::NativeShapeConversion::create(*shape);
  state.setNativeShape(std::move(nativeShape));

  const auto* storedNativeShape = state.getNativeShape();
  mNativeLocalAabb = storedNativeShape ? storedNativeShape->computeLocalAabb()
                                       : native::Aabb();
}

} // namespace collision
} // namespace dart
