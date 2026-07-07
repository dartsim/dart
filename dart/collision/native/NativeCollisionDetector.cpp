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

#include "dart/collision/native/NativeCollisionDetector.hpp"

#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/native/NativeCollisionGroup.hpp"
#include "dart/collision/native/NativeCollisionObject.hpp"
#include "dart/common/Console.hpp"

namespace dart {
namespace collision {

namespace {

//==============================================================================
bool checkGroupValidity(
    const NativeCollisionDetector* detector, CollisionGroup* group)
{
  if (!group) {
    dterr << "[NativeCollisionDetector::collide] Attempting to check collision "
          << "with a nullptr collision group.\n";
    return false;
  }

  if (detector != group->getCollisionDetector().get()) {
    dterr << "[NativeCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

} // namespace

//==============================================================================
std::shared_ptr<NativeCollisionDetector> NativeCollisionDetector::create()
{
  return std::shared_ptr<NativeCollisionDetector>(
      new NativeCollisionDetector());
}

//==============================================================================
NativeCollisionDetector::~NativeCollisionDetector() = default;

//==============================================================================
std::shared_ptr<CollisionDetector>
NativeCollisionDetector::cloneWithoutCollisionObjects() const
{
  return NativeCollisionDetector::create();
}

//==============================================================================
const std::string& NativeCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& NativeCollisionDetector::getStaticType()
{
  static const std::string type = "native";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> NativeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<NativeCollisionGroup>(shared_from_this());
}

//==============================================================================
bool NativeCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& /*option*/,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return false;

  static_cast<NativeCollisionGroup*>(group)->updateEngineData();
  return false;
}

//==============================================================================
bool NativeCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& /*option*/,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  static_cast<NativeCollisionGroup*>(group1)->updateEngineData();
  static_cast<NativeCollisionGroup*>(group2)->updateEngineData();
  return false;
}

//==============================================================================
double NativeCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[NativeCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
double NativeCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[NativeCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
NativeCollisionDetector::NativeCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject> NativeCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<NativeCollisionObject>(
      new NativeCollisionObject(this, shapeFrame));
}

//==============================================================================
void NativeCollisionDetector::refreshCollisionObject(
    CollisionObject* /*object*/)
{
  // Do nothing. NativeCollisionObject refreshes lazily in updateEngineData().
}

} // namespace collision
} // namespace dart
