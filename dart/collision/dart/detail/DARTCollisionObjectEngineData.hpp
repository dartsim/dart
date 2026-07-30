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

#pragma once

#include <dart/collision/dart/Aabb.hpp>
#include <dart/collision/dart/DARTCollisionObject.hpp>
#include <dart/collision/dart/shapes/Shape.hpp>

#include <limits>
#include <memory>

namespace dart {
namespace collision {
namespace detail {

struct DARTCollisionObjectAccessor
{
  static const native::Shape* getShape(const DARTCollisionObject* object)
  {
    return object->getEngineShape();
  }

  static const Eigen::Isometry3d& getTransform(
      const DARTCollisionObject* object)
  {
    return object->getEngineTransform();
  }

  static const native::Aabb& getAabb(const DARTCollisionObject* object)
  {
    return object->getEngineAabb();
  }
};

struct DARTCollisionObjectEngineData
{
  std::unique_ptr<native::Shape> nativeShape;
  Eigen::Isometry3d nativeTransform{Eigen::Isometry3d::Identity()};
  native::Aabb nativeLocalAabb;
  native::Aabb nativeAabb;
  std::size_t lastKnownShapeId{std::numeric_limits<std::size_t>::max()};
  std::size_t lastKnownShapeVersion{0u};
  bool hasNativeAabb{false};
};

} // namespace detail
} // namespace collision
} // namespace dart
