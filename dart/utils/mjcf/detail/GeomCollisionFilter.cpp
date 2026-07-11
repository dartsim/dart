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

#include "dart/utils/mjcf/detail/GeomCollisionFilter.hpp"

#include "dart/collision/CollisionObject.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
void GeomCollisionFilter::setGeomBitmasks(
    const dynamics::ShapeFrame* shapeFrame, int contype, int conaffinity)
{
  if (!shapeFrame)
    return;

  mBitmasks[shapeFrame] = Bitmasks{contype, conaffinity};
  ++mBitmaskRevision;
}

//==============================================================================
bool GeomCollisionFilter::ignoresCollision(
    const collision::CollisionObject* object1,
    const collision::CollisionObject* object2) const
{
  // Preserve DART's default self-collision/adjacent-body/resting-pair
  // filtering behavior.
  if (BodyNodeCollisionFilter::ignoresCollision(object1, object2))
    return true;

  const auto it1 = mBitmasks.find(object1->getShapeFrame());
  const auto it2 = mBitmasks.find(object2->getShapeFrame());
  if (it1 == mBitmasks.end() || it2 == mBitmasks.end()) {
    // At least one side wasn't created from an MJCF <geom>; defer to the
    // default behavior for this pair.
    return false;
  }

  const Bitmasks& b1 = it1->second;
  const Bitmasks& b2 = it2->second;

  // MuJoCo pair rule: http://mujoco.org/book/computation.html#coCollision
  const bool mayCollide = ((b1.mContype & b2.mConaffinity) != 0)
                          || ((b2.mContype & b1.mConaffinity) != 0);

  return !mayCollide;
}

//==============================================================================
std::size_t GeomCollisionFilter::getCollisionFilterSnapshotRevision() const
{
  std::size_t seed = getBodyNodePairBlackListRevision();
  seed ^= mBitmaskRevision + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
