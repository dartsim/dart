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

#include "dart/collision/ode/OdeCollisionObject.hpp"

#include "dart/collision/ode/detail/OdeGeom.hpp"

#include <utility>

namespace dart {
namespace collision {

OdeCollisionObject::~OdeCollisionObject() = default;

OdeCollisionObject::OdeCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : DARTCollisionObject(collisionDetector, shapeFrame),
    mOdeGeom(nullptr),
    mBodyId(nullptr)
{
  // No backend-specific data is created.
}

OdeCollisionObject& OdeCollisionObject::operator=(OdeCollisionObject&& other)
{
  if (this == &other)
    return *this;

  mOdeGeom = std::move(other.mOdeGeom);
  mBodyId = other.mBodyId;
  other.mBodyId = nullptr;

  return *this;
}

void OdeCollisionObject::updateEngineData()
{
  DARTCollisionObject::updateEngineData();
}

dBodyID OdeCollisionObject::getOdeBodyId() const
{
  return mBodyId;
}

dGeomID OdeCollisionObject::getOdeGeomId() const
{
  return mOdeGeom ? mOdeGeom->getOdeGeomId() : nullptr;
}

} // namespace collision
} // namespace dart
