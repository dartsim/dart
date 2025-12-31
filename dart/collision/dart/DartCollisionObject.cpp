/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/dart/DartCollisionObject.hpp"

#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <limits>

namespace dart {
namespace collision {

//==============================================================================
DARTCollisionObject::DARTCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame)
{
  // Do nothing
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getWorldAabbMin() const
{
  return mCoreObject.worldAabbMin;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getWorldAabbMax() const
{
  return mCoreObject.worldAabbMax;
}

//==============================================================================
const CoreObject& DARTCollisionObject::getCoreObject() const
{
  return mCoreObject;
}

//==============================================================================
void DARTCollisionObject::updateEngineData()
{
  const Eigen::Isometry3d& tf = getTransform();
  const Eigen::Vector3d center = tf.translation();
  const auto shape = getShape();

  mCoreObject.worldTransform = tf;
  mCoreObject.shape = CoreShape();

  if (!shape) {
    mCoreObject.worldAabbMin = center;
    mCoreObject.worldAabbMax = center;
    return;
  }

  const auto& shapeType = shape->getType();

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    const auto* sphere = static_cast<const dynamics::SphereShape*>(shape.get());

    mCoreObject.shape.type = CoreShapeType::kSphere;
    mCoreObject.shape.radius = sphere->getRadius();
  } else if (shapeType == dynamics::BoxShape::getStaticType()) {
    const auto* box = static_cast<const dynamics::BoxShape*>(shape.get());

    mCoreObject.shape.type = CoreShapeType::kBox;
    mCoreObject.shape.size = box->getSize();
  } else if (shapeType == dynamics::CylinderShape::getStaticType()) {
    const auto* cylinder
        = static_cast<const dynamics::CylinderShape*>(shape.get());

    mCoreObject.shape.type = CoreShapeType::kCylinder;
    mCoreObject.shape.radius = cylinder->getRadius();
    mCoreObject.shape.height = cylinder->getHeight();
  } else if (shapeType == dynamics::PlaneShape::getStaticType()) {
    const auto* plane = static_cast<const dynamics::PlaneShape*>(shape.get());

    mCoreObject.shape.type = CoreShapeType::kPlane;
    mCoreObject.shape.planeNormal = plane->getNormal();
    mCoreObject.shape.planeOffset = plane->getOffset();
  } else if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto* ellipsoid
        = static_cast<const dynamics::EllipsoidShape*>(shape.get());

    if (ellipsoid->isSphere()) {
      mCoreObject.shape.type = CoreShapeType::kSphere;
      mCoreObject.shape.radius = ellipsoid->getRadii()[0];
    } else {
      mCoreObject.shape.type = CoreShapeType::kUnsupported;
    }
  } else {
    mCoreObject.shape.type = CoreShapeType::kUnsupported;
  }

  if (mCoreObject.shape.type == CoreShapeType::kPlane) {
    const double inf = std::numeric_limits<double>::infinity();
    mCoreObject.worldAabbMin = Eigen::Vector3d::Constant(-inf);
    mCoreObject.worldAabbMax = Eigen::Vector3d::Constant(inf);
    return;
  }

  const auto& bbox = shape->getBoundingBox();
  const Eigen::Vector3d localCenter = bbox.computeCenter();
  const Eigen::Vector3d localHalfExtents = bbox.computeHalfExtents();

  const Eigen::Vector3d worldCenter = tf * localCenter;
  const Eigen::Vector3d worldHalfExtents
      = tf.linear().cwiseAbs() * localHalfExtents;

  mCoreObject.worldAabbMin = worldCenter - worldHalfExtents;
  mCoreObject.worldAabbMax = worldCenter + worldHalfExtents;
}

} // namespace collision
} // namespace dart
