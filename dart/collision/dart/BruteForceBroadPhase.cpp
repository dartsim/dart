/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/collision/dart/BruteForceBroadPhase.hpp"

#include "dart/collision/CollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
const std::string& BruteForceBroadPhase::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& BruteForceBroadPhase::getStaticType()
{
  static std::string type{"BruteForceBroadPhase"};
  return type;
}

//==============================================================================
void BruteForceBroadPhase::addObject(CollisionObject* object)
{
  // TODO(JS): check duplicity
  mCollisionObjects.push_back(object);
}

//==============================================================================
void BruteForceBroadPhase::updateOverlappingPairs()
{
  // TODO(JS):
  // - Use filter
  // - Skip for object whoes body node is the same
  // - check duplicity of the pair

  for (auto itA = mCollisionObjects.cbegin(); itA != mCollisionObjects.cend();
       ++itA)
  {
    for (auto itB = itA + 1; itB != mCollisionObjects.cend(); ++itB)
    {
      if ((*itA)->getAabb().overlapsWith((*itB)->getAabb()))
      {
        mOverlappingPairs.push_back(std::make_pair(*itA, *itB));
      }
    }
  }
}

} // namespace collision
} // namespace dart
