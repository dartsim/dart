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

#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/native/persistent_manifold_cache.hpp>

namespace dart::collision {

class DARTCollisionDetector : public DartCollisionDetector
{
public:
  static std::shared_ptr<DARTCollisionDetector> create()
  {
    return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
  }

  bool raycast(
      CollisionGroup* group,
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option = RaycastOption(),
      RaycastResult* result = nullptr) override
  {
    // Keep the legacy display-name facade aligned with gz-physics, where
    // "dart" is still treated as an unsupported ray-intersection backend.
    return CollisionDetector::raycast(group, from, to, option, result);
  }

  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override
  {
    return DARTCollisionDetector::create();
  }

protected:
  DARTCollisionDetector() = default;
};

} // namespace dart::collision
