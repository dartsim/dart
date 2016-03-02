/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COLLISION_FCL_FCLMESHCOLLISIONGROUPDATA_H_
#define DART_COLLISION_FCL_FCLMESHCOLLISIONGROUPDATA_H_

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_bruteforce.h>

#include "dart/collision/CollisionGroup.h"
#include "dart/collision/CollisionGroupData.h"

namespace dart {
namespace collision {

class CollisionObject;
class FCLCollisionObjectUserData;

class FCLMeshCollisionGroupData : public CollisionGroupData
{
public:

//  using FCLCollisionManager = fcl::DynamicAABBTreeCollisionManager;
  using FCLCollisionManager = fcl::NaiveCollisionManager;
  using CollisionObjects = CollisionGroup::CollisionObjectPtrs;

  /// Constructor
  FCLMeshCollisionGroupData(CollisionDetector* collisionDetector,
                            CollisionGroup* parent,
                            const CollisionObjects& collObjects);

  // Documentation inherited
  std::unique_ptr<CollisionGroupData> clone(
      CollisionGroup* newParent,
      const CollisionObjectPtrs& collObjects) const override;

  // Documentation inherited
  void init() override;

  // Documentation inherited
  void addCollisionObject(const CollisionObjectPtr& object,
                          const bool init) override;

  // Documentation inherited
  void addCollisionObjects(const CollisionObjectPtrs& collObjects,
                           const bool init) override;

  // Documentation inherited
  void removeCollisionObject(const CollisionObjectPtr& object,
                             const bool init) override;

  // Documentation inherited
  void removeAllCollisionObjects(bool init) override;

  // Documentation inherited
  void update() override;

  /// Return FCL collision manager that is also a broad-phase algorithm
  FCLCollisionManager* getFCLCollisionManager() const;

protected:

  /// FCL broad-phase algorithm
  std::unique_ptr<FCLCollisionManager> mBroadPhaseAlg;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_FCLMESHCOLLISIONGROUPDATA_H_
