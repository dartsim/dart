/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_COLLISION_ODE_ODECOLLISIONDETECTOR_HPP_
#define DART_COLLISION_ODE_ODECOLLISIONDETECTOR_HPP_

#include <dart/collision/dart/DartCollisionDetector.hpp>

#include <dart/collision/ode/OdeStub.hpp>

#include <deque>
#include <utility>
#include <vector>

#define MAX_COLLIDE_RETURNS 250

namespace dart {
namespace collision {

class OdeCollisionObject;
class OdeCollisionGroup;

class DART_API OdeCollisionDetector : public DARTCollisionDetector
{
public:
  using CollisionDetector::createCollisionGroup;

  friend class OdeCollisionObject;
  friend class OdeCollisionGroup;
  using CollObjPair = std::pair<CollisionObject*, CollisionObject*>;

  struct ContactHistoryItem
  {
    CollObjPair pair;
    std::deque<Contact> history;
  };

  static std::shared_ptr<OdeCollisionDetector> create();

  ~OdeCollisionDetector() override;

  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override;

  const std::string& getType() const override;

  static const std::string& getStaticType();

  std::unique_ptr<CollisionGroup> createCollisionGroup() override;

  bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  double distance(
      CollisionGroup* group,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  double distance(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

protected:
  OdeCollisionDetector();

  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  dWorldID getOdeWorldId() const;

protected:
  dWorldID mWorldId;

private:
  dContactGeom contactCollisions[MAX_COLLIDE_RETURNS];
  std::vector<ContactHistoryItem> mContactHistory;
  static Registrar<OdeCollisionDetector> mRegistrar;
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_ODE_ODECOLLISIONDETECTOR_HPP_
