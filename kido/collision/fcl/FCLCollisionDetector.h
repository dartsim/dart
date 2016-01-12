/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
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

#ifndef KIDO_COLLISION_FCL_FCLCOLLISIONDETECTOR_H_
#define KIDO_COLLISION_FCL_FCLCOLLISIONDETECTOR_H_

#include <fcl/collision_object.h>
#include <fcl/collision_data.h>
#include <fcl/broadphase/broadphase.h>

#include "kido/collision/CollisionDetector.h"

namespace kido {
namespace collision {

class FCLCollisionNode;

/// FCLCollisionDetector
class FCLCollisionDetector : public CollisionDetector
{
public:
  /// Constructor
  FCLCollisionDetector();

  /// Destructor
  virtual ~FCLCollisionDetector();

  // Documentation inherited
  virtual bool detectCollision(bool _checkAllCollisions,
                               bool _calculateContactPoints) override;

  // Documentation inherited
  virtual CollisionNode* createCollisionNode(dynamics::BodyNode* _bodyNode)
  override;

  /// Get collision node given FCL collision geometry
  CollisionNode* findCollisionNode(
      const fcl::CollisionGeometry* _fclCollGeom) const;

  /// Get collision node given FCL collision object
  FCLCollisionNode* findCollisionNode(
      const fcl::CollisionObject* _fclCollObj) const;

protected:
  // Documentation inherited
  virtual bool detectCollision(CollisionNode* _node1, CollisionNode* _node2,
                               bool _calculateContactPoints) override;

  /// Broad-phase collision checker of FCL
  fcl::DynamicAABBTreeCollisionManager* mBroadPhaseAlg;
};

}  // namespace collision
}  // namespace kido

#endif  // KIDO_COLLISION_FCL_FCLCOLLISIONDETECTOR_H_
