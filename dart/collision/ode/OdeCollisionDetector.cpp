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

#include "dart/collision/ode/OdeCollisionDetector.hpp"

#include "dart/collision/ode/OdeCollisionGroup.hpp"
#include "dart/collision/ode/OdeCollisionObject.hpp"

namespace dart {
namespace collision {

OdeCollisionDetector::Registrar<OdeCollisionDetector>
    OdeCollisionDetector::mRegistrar{
        OdeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::OdeCollisionDetector> {
          return dart::collision::OdeCollisionDetector::create();
        }};

std::shared_ptr<OdeCollisionDetector> OdeCollisionDetector::create()
{
  return std::shared_ptr<OdeCollisionDetector>(new OdeCollisionDetector());
}

OdeCollisionDetector::~OdeCollisionDetector() = default;

std::shared_ptr<CollisionDetector>
OdeCollisionDetector::cloneWithoutCollisionObjects() const
{
  return OdeCollisionDetector::create();
}

const std::string& OdeCollisionDetector::getType() const
{
  return getStaticType();
}

const std::string& OdeCollisionDetector::getStaticType()
{
  static const std::string type = "ode";
  return type;
}

std::unique_ptr<CollisionGroup> OdeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<OdeCollisionGroup>(shared_from_this());
}

bool OdeCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  return DARTCollisionDetector::collide(group, option, result);
}

bool OdeCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  return DARTCollisionDetector::collide(group1, group2, option, result);
}

double OdeCollisionDetector::distance(
    CollisionGroup* group,
    const DistanceOption& option,
    DistanceResult* result)
{
  return DARTCollisionDetector::distance(group, option, result);
}

double OdeCollisionDetector::distance(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const DistanceOption& option,
    DistanceResult* result)
{
  return DARTCollisionDetector::distance(group1, group2, option, result);
}

OdeCollisionDetector::OdeCollisionDetector()
  : DARTCollisionDetector(), mWorldId(nullptr)
{
  // Do nothing.
}

std::unique_ptr<CollisionObject> OdeCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<CollisionObject>(
      new OdeCollisionObject(this, shapeFrame));
}

dWorldID OdeCollisionDetector::getOdeWorldId() const
{
  return mWorldId;
}

} // namespace collision
} // namespace dart
