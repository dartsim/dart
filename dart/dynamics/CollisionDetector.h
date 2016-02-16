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

#ifndef DART_DYNAMICS_COLLISIONDETECTOR_H_
#define DART_DYNAMICS_COLLISIONDETECTOR_H_

#include <memory>

#include "dart/collision/CollisionDetector.h"
#include "dart/dynamics/SmartPointer.h"
#include "dart/collision/Engine.h"
#include "dart/collision/fcl/FCLEngine.h"
#include "dart/collision/CollisionObject.h"
#include "dart/collision/CollisionGroup.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

class ShapeNodeCollisionObject : public collision::CollisionObject
{
public:

  ShapeNodeCollisionObject(const collision::EnginePtr& engine,
      const dynamics::ShapePtr& shape,
      const dynamics::BodyNodePtr& bodyNode);
  // TODO(JS): this should be replaced by ShapeNode

  // Documentation inherited
  const Eigen::Isometry3d getTransform() const override;

  /// Return BodyNode pointer associated with this ShapeNodeCollisionObject
  dynamics::BodyNodePtr getBodyNode() const;

protected:

  dynamics::BodyNodePtr mBodyNode;
  // TODO(JS): this should be changed to ShapeNode

};

class CollisionDetector
{
public:

  static std::shared_ptr<ShapeNodeCollisionObject> createCollisionObject(
      const collision::EnginePtr engine,
      const dynamics::ShapePtr& shape,
      const BodyNodePtr& bodyNode);

  static std::vector<collision::CollisionObjectPtr>
  createCollisionObjects(
      const collision::EnginePtr& engine,
      const dynamics::SkeletonPtr& skel);

  template <class ColDecEngine = collision::FCLEngine>
  static bool detect(const dynamics::ShapePtr& shape1,
                     const BodyNodePtr& body1,
                     const dynamics::ShapePtr& shape2,
                     const BodyNodePtr& body2,
                     const collision::Option& option,
                     collision::Result& result);

  template <class ColDecEngine = collision::FCLEngine>
  static bool detect(const dynamics::SkeletonPtr& skel1,
                     const dynamics::SkeletonPtr& skel2,
                     const collision::Option& option,
                     collision::Result& result);

};

//==============================================================================
template <class ColDecEngine>
bool CollisionDetector::detect(
    const ShapePtr& shape1, const BodyNodePtr& body1,
    const ShapePtr& shape2, const BodyNodePtr& body2,
    const collision::Option& option, collision::Result& result)
{
  auto engine = ColDecEngine::create();

  auto obj1 = ShapeNodeCollisionObject(engine, shape1, body1);
  auto obj2 = ShapeNodeCollisionObject(engine, shape2, body2);

  return obj1.detect(&obj2, option, result);
}

//==============================================================================
template <class ColDecEngine>
bool CollisionDetector::detect(
    const dynamics::SkeletonPtr& skel1,
    const dynamics::SkeletonPtr& skel2,
    const collision::Option& option, collision::Result& result)
{
  auto engine = ColDecEngine::create();

  auto objects1 = createCollisionObjects(engine, skel1);
  auto objects2 = createCollisionObjects(engine, skel2);

  auto group1 = collision::CollisionGroup(engine, objects1);
  auto group2 = collision::CollisionGroup(engine, objects2);

  return group1.detect(&group2, option, result);
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_COLLISIONDETECTOR_H_
