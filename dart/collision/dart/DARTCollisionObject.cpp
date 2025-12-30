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

#include "dart/collision/dart/DARTCollisionObject.hpp"

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace collision {

//==============================================================================
DARTCollisionObject::DARTCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame),
    mWorldAabbMin(Eigen::Vector3d::Zero()),
    mWorldAabbMax(Eigen::Vector3d::Zero())
{
  // Do nothing
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getWorldAabbMin() const
{
  return mWorldAabbMin;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getWorldAabbMax() const
{
  return mWorldAabbMax;
}

//==============================================================================
void DARTCollisionObject::updateEngineData()
{
  const Eigen::Isometry3d& tf = getTransform();
  const Eigen::Vector3d center = tf.translation();
  const auto shape = getShape();

  if (!shape) {
    mWorldAabbMin = center;
    mWorldAabbMax = center;
    return;
  }

  const auto& bbox = shape->getBoundingBox();
  const Eigen::Vector3d localCenter = bbox.computeCenter();
  const Eigen::Vector3d localHalfExtents = bbox.computeHalfExtents();

  const Eigen::Vector3d worldCenter = tf * localCenter;
  const Eigen::Vector3d worldHalfExtents
      = tf.linear().cwiseAbs() * localHalfExtents;

  mWorldAabbMin = worldCenter - worldHalfExtents;
  mWorldAabbMax = worldCenter + worldHalfExtents;
}

} // namespace collision
} // namespace dart
