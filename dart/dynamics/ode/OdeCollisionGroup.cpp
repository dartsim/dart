/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/dynamics/ode/OdeCollisionGroup.hpp"

#include "dart/dynamics/ode/OdeCollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
OdeCollisionGroup::OdeCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector)
{
  // This uses an internal data structure that records how each geom overlaps
  // cells in one of several three dimensional grids. Each grid has cubical
  // cells of side lengths 2i, where i is an integer that ranges from a minimum
  // to a maximum value. The time required to do intersection testing for n
  // objects is O(n) (as long as those objects are not clustered together too
  // closely), as each object can be quickly paired with the objects around it.
  //
  // Source:
  // https://www.ode-wiki.org/wiki/index.php?title=Manual:_Collision_Detection#Space_functions
  mSpaceId = dHashSpaceCreate(0);
  assert(mSpaceId);
  dHashSpaceSetLevels(mSpaceId, -2, 8);
}

//==============================================================================
OdeCollisionGroup::~OdeCollisionGroup()
{
  // This is important to call this function before detroy ODE space.
  removeAllShapeFrames();

  dSpaceDestroy(mSpaceId);
}

//==============================================================================
void OdeCollisionGroup::initializeEngineData()
{
  // ODE don't need to anything after a geom is added to the space.
}

//==============================================================================
void OdeCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto casted = static_cast<OdeCollisionObject*>(object);
  auto geomId = casted->getOdeGeomId();
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
    auto geomId = casted->getOdeGeomId();
    dSpaceAdd(mSpaceId, geomId);
  }

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::removeCollisionObjectFromEngine(CollisionObject* object)
{
  auto casted = static_cast<OdeCollisionObject*>(object);
  auto geomId = casted->getOdeGeomId();
  dSpaceRemove(mSpaceId, geomId);

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  dSpaceClean(mSpaceId);

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::updateCollisionGroupEngineData()
{
  // ODE requires nothing for this.
}

//==============================================================================
dSpaceID OdeCollisionGroup::getOdeSpaceId() const
{
  return mSpaceId;
}

} // namespace collision
} // namespace dart
