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
  std::shared_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  std::shared_ptr<CollisionGroup> createCollisionGroup(
      const dynamics::ShapeFrame* shapeFrame) override;

  // Documentation inherited
  std::shared_ptr<CollisionGroup> createCollisionGroup(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames) override;

  // Documentation inherited
  bool detect(CollisionGroup* group,
              const Option& option, Result& result) override;

  // Documentation inherited
  bool detect(CollisionGroup* group1, CollisionGroup* group2,
              const Option& option, Result& result) override;

protected:

  struct BulletCollsionPack
  {
    std::shared_ptr<btCollisionShape> collisionShape;
    std::shared_ptr<btTriangleMesh> triMesh;
    // TODO(JS): change to unique_ptr
  };

  /// Constructor
  BulletCollisionDetector() = default;

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

public:

  BulletCollisionObject* findCollisionObject(
      btCollisionObject* bulletCollObj) const;

protected:

  // Documentation inherited
  void notifyCollisionObjectDestorying(CollisionObject* collObj) override;

  ///
  BulletCollsionPack claimBulletCollisionGeometry(
      const dynamics::ConstShapePtr& shape);

  // Documentation inherited
  void reclaimBulletCollisionGeometry(const dynamics::ConstShapePtr& shape);

  BulletCollsionPack createMesh(
      const Eigen::Vector3d& scale, const aiScene* mesh);

  BulletCollsionPack createSoftMesh(const aiMesh* mesh);

  BulletCollsionPack createBulletCollisionShape(
      const dynamics::ConstShapePtr& shape);

protected:

  using ShapeMapValue = std::pair<BulletCollsionPack, size_t>;

  std::map<dynamics::ConstShapePtr, ShapeMapValue> mShapeMap;

  std::map<btCollisionObject*,
           BulletCollisionObject*> mBulletCollisionObjectMap;

  std::shared_ptr<CollisionGroup> mBulletCollisionGroupForSinglePair;
  // TODO(JS): should be unique_ptr

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_BULLET_BULLETCOLLISIONDETECTOR_H_
