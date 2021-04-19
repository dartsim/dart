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

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/BackwardCompatibility.hpp"

namespace dart {
namespace collision2 {

class DARTCollisionObject : public CollisionObject
{
public:
  const Eigen::Isometry3d& getTransform() const override;

  /// Return FCL collision object
  dart::collision2::dart::CollisionObject* getFCLCollisionObject();

  /// Return FCL collision object
  const dart::collision2::dart::CollisionObject* getFCLCollisionObject() const;

protected:
  /// Constructor
  DARTCollisionObject(
      CollisionGroup* collisionGroup,
      math::GeometryPtr shape,
      const std::shared_ptr<dart::collision2::dart::CollisionGeometry>&
          fclCollGeom);

  // Documentation inherited
  void updateEngineData() override;

protected:
  DARTCollisionObject(CollisionGroup* collisionGroup, math::GeometryPtr shape);

  /// FCL collision object
  std::unique_ptr<dart::collision2::dart::CollisionObject> mFCLCollisionObject;

private:
  friend class FCLCollisionDetector;
};

} // namespace collision2
} // namespace dart
