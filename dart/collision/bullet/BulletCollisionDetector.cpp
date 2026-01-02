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

#include "dart/collision/bullet/BulletCollisionDetector.hpp"

#include "dart/collision/bullet/BulletCollisionGroup.hpp"
#include "dart/collision/bullet/BulletCollisionObject.hpp"

namespace dart {
namespace collision {

BulletCollisionDetector::Registrar<BulletCollisionDetector>
    BulletCollisionDetector::mRegistrar{
        BulletCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::BulletCollisionDetector> {
          return dart::collision::BulletCollisionDetector::create();
        }};

std::shared_ptr<BulletCollisionDetector> BulletCollisionDetector::create()
{
  return std::shared_ptr<BulletCollisionDetector>(
      new BulletCollisionDetector());
}

BulletCollisionDetector::~BulletCollisionDetector() = default;

std::shared_ptr<CollisionDetector>
BulletCollisionDetector::cloneWithoutCollisionObjects() const
{
  return BulletCollisionDetector::create();
}

const std::string& BulletCollisionDetector::getType() const
{
  return BulletCollisionDetector::getStaticType();
}

const std::string& BulletCollisionDetector::getStaticType()
{
  static const std::string type = "bullet";
  return type;
}

std::unique_ptr<CollisionGroup>
BulletCollisionDetector::createCollisionGroup()
{
  return std::make_unique<BulletCollisionGroup>(shared_from_this());
}

BulletCollisionDetector::BulletCollisionDetector() : DARTCollisionDetector()
{
  // Do nothing.
}

std::unique_ptr<CollisionObject>
BulletCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<CollisionObject>(
      new BulletCollisionObject(
          this,
          shapeFrame,
          shapeFrame ? shapeFrame->getShape() : dynamics::ConstShapePtr(),
          nullptr));
}

} // namespace collision
} // namespace dart
