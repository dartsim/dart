/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COLLISION_DART_DARTCOLLISIONGROUP_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONGROUP_HPP_

#include "dart/collision/CollisionGroup.hpp"

namespace dart {
namespace collision {

class DARTCollisionObject;

class DARTCollisionGroup : public CollisionGroup
{
public:

  friend class DARTCollisionDetector;

  /// Constructor
  DARTCollisionGroup(const CollisionDetectorPtr& collisionDetector);

  /// Destructor
  virtual ~DARTCollisionGroup() = default;

protected:

  // Documentation inherited
  void initializeEngineData() override;

  // Documentation inherited
  void addCollisionObjectToEngine(CollisionObject* object) override;

  // Documentation inherited
  void addCollisionObjectsToEngine(
      const std::vector<CollisionObject*>& collObjects) override;

  // Documentation inherited
  void removeCollisionObjectFromEngine(CollisionObject* object) override;

  // Documentation inherited
  void removeAllCollisionObjectsFromEngine() override;

  // Documentation inherited
  void updateCollisionGroupEngineData() override;

protected:

  /// CollisionObjects added to this DARTCollisionGroup
  std::vector<CollisionObject*> mCollisionObjects;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DARTCOLLISIONGROUP_HPP_
