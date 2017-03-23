/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/collision/ode/OdeCollisionGroup.hpp"

#include "dart/collision/ode/OdeCollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
OdeCollisionGroup::OdeCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector)
{
  mSpaceId = dHashSpaceCreate(0);
  dHashSpaceSetLevels(mSpaceId, -2, 8);

  mContactGroupId = dJointGroupCreate(0);
}

//==============================================================================
void OdeCollisionGroup::initializeEngineData()
{
}

//==============================================================================
void OdeCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto casted = static_cast<OdeCollisionObject*>(object);
  auto geomId = casted->getGeomId();
  dSpaceAdd(mSpaceId, geomId);

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto collObj : collObjects)
  {
    auto casted = static_cast<OdeCollisionObject*>(collObj);

  }

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::removeCollisionObjectFromEngine(CollisionObject* object)
{
  auto casted = static_cast<OdeCollisionObject*>(object);


  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::removeAllCollisionObjectsFromEngine()
{

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::updateCollisionGroupEngineData()
{
}

//==============================================================================
dSpaceID OdeCollisionGroup::getOdeSpaceId() const
{
  return mSpaceId;
}

}  // namespace collision
}  // namespace dart
