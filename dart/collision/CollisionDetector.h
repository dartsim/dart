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

  template <class EngineType, class... Args>
  static CollisionDetectorPtr create(const Args&... args);

  /// Destructor
  virtual ~CollisionDetector();

  /// Return collision detection engine
  Engine* getEngine();

  /// Return (const) collision detection engine
  const Engine* getEngine() const;

  /// Create a collision object
  template <typename CollisionObjectType, typename... Args>
  std::shared_ptr<CollisionObjectType> createCollisionObject(
      const dynamics::ShapePtr& shape,
      const Args&... args);

  /// Create a collision group
  std::shared_ptr<CollisionGroup> createCollisionGroup(
      const std::vector<CollisionObjectPtr>& objects);

  /// Create a collision group
  std::shared_ptr<CollisionGroup> createCollisionGroup();

  /// Perform collision detection for object1-object2.
  bool detect(CollisionObject* object1, CollisionObject* object2,
              const Option& option, Result& result);

  /// Perform collision detection for object-group.
  bool detect(CollisionObject* object, CollisionGroup* group,
              const Option& option, Result& result);

  /// Identical with detect(object, group, option, result)
  bool detect(CollisionGroup* group, CollisionObject* object,
              const Option& option, Result& result);

  /// Perform collision detection for group.
  bool detect(CollisionGroup* group,
              const Option& option, Result& result);

  /// Perform collision detection for group1-group2.
  bool detect(CollisionGroup* group1, CollisionGroup* group2,
              const Option& option, Result& result);

protected:

  /// Constructor
  CollisionDetector(std::unique_ptr<Engine>&& engine);

protected:

  std::unique_ptr<Engine> mEngine;

};

//==============================================================================
template <class EngineType, class... Args>
CollisionDetectorPtr CollisionDetector::create(const Args&... args)
{
  auto engine = new EngineType(args...);

  return CollisionDetectorPtr(new CollisionDetector(
      std::move(std::unique_ptr<EngineType>(engine))));
}

//==============================================================================
template <typename CollisionObjectType, typename... Args>
std::shared_ptr<CollisionObjectType>
CollisionDetector::createCollisionObject(
    const dynamics::ShapePtr& shape,
    const Args&... args)
{
  return std::make_shared<CollisionObjectType>(
        shared_from_this(), shape, args...);
}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONDETECTOR_H_
