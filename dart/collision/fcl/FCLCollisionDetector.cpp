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

#include "dart/collision/fcl/FCLCollisionDetector.hpp"

#include "dart/collision/fcl/FCLCollisionGroup.hpp"
#include "dart/collision/fcl/FCLCollisionObject.hpp"

namespace dart {
namespace collision {

FCLCollisionDetector::Registrar<FCLCollisionDetector>
    FCLCollisionDetector::mRegistrar{
        FCLCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::FCLCollisionDetector> {
          return dart::collision::FCLCollisionDetector::create();
        }};

std::shared_ptr<FCLCollisionDetector> FCLCollisionDetector::create()
{
  return std::shared_ptr<FCLCollisionDetector>(new FCLCollisionDetector());
}

FCLCollisionDetector::~FCLCollisionDetector() = default;

std::shared_ptr<CollisionDetector>
FCLCollisionDetector::cloneWithoutCollisionObjects() const
{
  return FCLCollisionDetector::create();
}

const std::string& FCLCollisionDetector::getType() const
{
  return FCLCollisionDetector::getStaticType();
}

const std::string& FCLCollisionDetector::getStaticType()
{
  static const std::string type = "fcl";
  return type;
}

std::unique_ptr<CollisionGroup> FCLCollisionDetector::createCollisionGroup()
{
  return std::make_unique<FCLCollisionGroup>(shared_from_this());
}

void FCLCollisionDetector::setPrimitiveShapeType(PrimitiveShape type)
{
  mPrimitiveShapeType = type;
}

FCLCollisionDetector::PrimitiveShape
FCLCollisionDetector::getPrimitiveShapeType() const
{
  return mPrimitiveShapeType;
}

void FCLCollisionDetector::setContactPointComputationMethod(
    ContactPointComputationMethod method)
{
  mContactPointComputationMethod = method;
}

FCLCollisionDetector::ContactPointComputationMethod
FCLCollisionDetector::getContactPointComputationMethod() const
{
  return mContactPointComputationMethod;
}

FCLCollisionDetector::FCLCollisionDetector()
  : DARTCollisionDetector(),
    mPrimitiveShapeType(PRIMITIVE),
    mContactPointComputationMethod(DART)
{
  // Do nothing.
}

std::unique_ptr<CollisionObject> FCLCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<CollisionObject>(
      new FCLCollisionObject(this, shapeFrame, nullptr));
}

} // namespace collision
} // namespace dart
