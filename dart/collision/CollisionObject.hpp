/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include <Eigen/Geometry>

#include "dart/collision/Types.hpp"
#include "dart/math/SmartPointer.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class CollisionObject
{
public:
  // Type aliases
  using S = S_;

  CollisionObject();

  /// Destructor
  virtual ~CollisionObject();

  /// Return collision detection engine associated with this CollisionObject
  CollisionDetector<S>* getCollisionDetector();

  /// Return collision detection engine associated with this CollisionObject
  const CollisionDetector<S>* getCollisionDetector() const;

  const void* getUserData() const;

  /// Return the associated Shape
  math::ConstGeometryPtr getShape() const;

  /// Return the transformation of this CollisionObject in world coordinates
  virtual const Eigen::Isometry3d& getTransform() const = 0;

protected:
  /// Contructor
  CollisionObject(CollisionGroup<S>* mCollisionGroup, math::GeometryPtr shape);

  /// Update the collision object of the collision detection engine. This
  /// function will be called ahead of every collision checking by
  /// CollisionGroup.
  virtual void updateEngineData() = 0;

protected:
  /// Collision group
  CollisionGroup<S>* mCollisionGroup;

  const math::GeometryPtr mShape;

  void* mUserData{nullptr};

private:
  friend class CollisionGroup<S>;
};

extern template class CollisionObject<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/detail/CollisionObject-impl.hpp"
