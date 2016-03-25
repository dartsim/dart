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

#ifndef DART_COLLISION_COLLISIONDETECTOR_H_
#define DART_COLLISION_COLLISIONDETECTOR_H_

#include <vector>
#include <map>

#include <Eigen/Dense>

#include "dart/collision/Contact.h"
#include "dart/collision/Option.h"
#include "dart/collision/Result.h"
#include "dart/collision/SmartPointer.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace collision {

class CollisionObject;

class CollisionDetector : public std::enable_shared_from_this<CollisionDetector>
{
public:

  friend class CollisionObject;
  friend class CollisionGroup;

  /// Destructor
  virtual ~CollisionDetector();

  /// Return collision detection engine type in std::string
  virtual const std::string& getType() const = 0;

  /// Create a collision group
  virtual std::shared_ptr<CollisionGroup> createCollisionGroup() = 0;

  /// Create a collision group from ShapeFrame
  virtual std::shared_ptr<CollisionGroup> createCollisionGroup(
      const dynamics::ShapeFrame* shapeFrame) = 0;

  /// Create a collision group from ShapeFrames
  virtual std::shared_ptr<CollisionGroup> createCollisionGroup(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames) = 0;

  /// Create a collision group from Skeleton
  std::shared_ptr<CollisionGroup> createCollisionGroup(
      dynamics::Skeleton* skeleton);

  /// Perform collision detection for group.
  virtual bool detect(CollisionGroup* group,
                      const Option& option, Result& result) = 0;

  /// Perform collision detection for group1-group2.
  virtual bool detect(CollisionGroup* group1, CollisionGroup* group2,
                      const Option& option, Result& result) = 0;

protected:

  /// Constructor
  CollisionDetector() = default;

  /// Return CollisionObject associated with shapeFrame. New CollisionObject
  /// will be created if it hasn't created yet for shapeFrame.
  CollisionObject* claimCollisionObject(const dynamics::ShapeFrame* shapeFrame);
  // TODO(JS): Maybe WeakShapeFramePtr

  /// Create CollisionObject
  virtual std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) = 0;

  ///
  virtual void notifyCollisionObjectDestorying(CollisionObject* collObj) = 0;

  /// Reclaim CollisionObject associated with shapeFrame. The CollisionObject
  /// will be destroyed if no CollisionGroup holds it.
  void reclaimCollisionObject(const CollisionObject* shapeFrame);

protected:

  using CollisionObjectMapValue
      = std::pair<std::unique_ptr<CollisionObject>, size_t>;
  using CollisionObjectMap
      = std::map<const dynamics::ShapeFrame*, CollisionObjectMapValue>;
  using CollisionObjectPair
      = std::pair<const dynamics::ShapeFrame*, CollisionObjectMapValue>;

  CollisionObjectMap mCollisionObjectMap;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONDETECTOR_H_
