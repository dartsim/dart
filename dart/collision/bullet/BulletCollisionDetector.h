/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#ifndef DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_H_
#define DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_H_

// Must be included before any Bullet headers.
#include "dart/config.h"

#include <vector>
#include <assimp/scene.h>
#include <btBulletCollisionCommon.h>
#include "dart/collision/CollisionDetector.h"

namespace dart {
namespace collision {

class BulletCollisionObject;

class BulletCollisionDetector : public CollisionDetector
{
public:

  friend class CollisionDetector;

  static std::shared_ptr<BulletCollisionDetector> create();

  /// Constructor
  virtual ~BulletCollisionDetector();

  /// Return engine type "Bullet"
  static const std::string& getTypeStatic();

  // Documentation inherited
  const std::string& getType() const override;

  // Documentation inherited
  std::unique_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  bool collide(CollisionGroup* group,
               const CollisionOption& option, CollisionResult& result) override;

  // Documentation inherited
  bool collide(CollisionGroup* group1, CollisionGroup* group2,
               const CollisionOption& option, CollisionResult& result) override;

protected:

  /// Constructor
  BulletCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  // Documentation inherited
  void notifyCollisionObjectDestroying(CollisionObject* object) override;

private:

  btCollisionShape* claimBulletCollisionShape(
      const dynamics::ConstShapePtr& shape);

  void reclaimBulletCollisionShape(
      const dynamics::ConstShapePtr& shape);

  btCollisionShape* createBulletCollisionShape(
      const dynamics::ConstShapePtr& shape);

private:

  std::map<dynamics::ConstShapePtr,
           std::pair<btCollisionShape*, size_t>> mShapeMap;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_H_
