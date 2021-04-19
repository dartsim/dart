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

#include "dart/collision/CollisionGroup.hpp"

#include <cassert>

namespace dart {
namespace collision2 {

//==============================================================================
template <typename S>
CollisionGroup<S>::CollisionGroup(CollisionDetector<S>* collisionDetector)
  : mCollisionDetector(collisionDetector), mUpdateAutomatically(true)
{
  assert(mCollisionDetector);
}

//==============================================================================
template <typename S>
CollisionDetector<S>* CollisionGroup<S>::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
template <typename S>
const CollisionDetector<S>* CollisionGroup<S>::getCollisionDetector() const
{
  return mCollisionDetector;
}

//==============================================================================
template <typename S>
template <typename GeometryType, typename... Args>
CollisionObjectPtr<S> CollisionGroup<S>::createCollisionObject(Args&&... args)
{
  auto shape = std::make_shared<GeometryType>(std::forward<Args>(args)...);
  return createCollisionObject(std::move(shape));
}

} // namespace collision2
} // namespace dart
