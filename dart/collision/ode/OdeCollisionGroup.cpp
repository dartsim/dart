/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/collision/ode/OdeCollisionGroup.hpp"

#include "dart/collision/ode/OdeCollisionDetector.hpp"
#include "dart/collision/ode/OdeCollisionObject.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"

#include <algorithm>
#include <memory>

namespace dart {
namespace collision {

namespace {

//==============================================================================
void eraseCollisionObject(
    std::vector<OdeCollisionObject*>& objects, OdeCollisionObject* object)
{
  objects.erase(
      std::remove(objects.begin(), objects.end(), object), objects.end());
}

//==============================================================================
bool isCylinderCollisionObject(const OdeCollisionObject* object)
{
  return object && object->getShape()->as<dynamics::CylinderShape>();
}

//==============================================================================
bool isPlaneCollisionObject(const OdeCollisionObject* object)
{
  return object && object->getShape()->as<dynamics::PlaneShape>();
}

} // namespace

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
  DART_ASSERT(mSpaceId);
  dHashSpaceSetLevels(mSpaceId, -2, 8);
}

//==============================================================================
OdeCollisionGroup::~OdeCollisionGroup()
{
  // This is important to call this function before destroy ODE space.
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

  if (isCylinderCollisionObject(casted))
    mCylinderCollisionObjects.push_back(casted);
  else if (isPlaneCollisionObject(casted))
    mPlaneCollisionObjects.push_back(casted);

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::addCollisionObjectsToEngine(
    const std::vector<CollisionObject*>& collObjects)
{
  for (auto collObj : collObjects) {
    auto casted = static_cast<OdeCollisionObject*>(collObj);
    auto geomId = casted->getOdeGeomId();
    dSpaceAdd(mSpaceId, geomId);

    if (isCylinderCollisionObject(casted))
      mCylinderCollisionObjects.push_back(casted);
    else if (isPlaneCollisionObject(casted))
      mPlaneCollisionObjects.push_back(casted);
  }

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::removeCollisionObjectFromEngine(CollisionObject* object)
{
  if (auto detector = std::static_pointer_cast<OdeCollisionDetector>(
          getCollisionDetector())) {
    detector->clearContactHistoryFor(object);
  }

  auto casted = static_cast<OdeCollisionObject*>(object);
  auto geomId = casted->getOdeGeomId();
  eraseCollisionObject(mCylinderCollisionObjects, casted);
  eraseCollisionObject(mPlaneCollisionObjects, casted);
  dSpaceRemove(mSpaceId, geomId);

  initializeEngineData();
}

//==============================================================================
void OdeCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  if (auto detector = std::static_pointer_cast<OdeCollisionDetector>(
          getCollisionDetector())) {
    detector->clearContactHistory();
  }

  dSpaceClean(mSpaceId);
  mCylinderCollisionObjects.clear();
  mPlaneCollisionObjects.clear();

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

//==============================================================================
const std::vector<OdeCollisionObject*>&
OdeCollisionGroup::getCylinderCollisionObjects() const
{
  return mCylinderCollisionObjects;
}

//==============================================================================
const std::vector<OdeCollisionObject*>&
OdeCollisionGroup::getPlaneCollisionObjects() const
{
  return mPlaneCollisionObjects;
}

} // namespace collision
} // namespace dart
