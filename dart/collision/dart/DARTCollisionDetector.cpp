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

#include "dart/collision/dart/DARTCollisionDetector.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DartCollisionEngine.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/common/Logging.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

namespace dart {
namespace collision {

//==============================================================================
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mRegistrar{
        DARTCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
          return dart::collision::DARTCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
DARTCollisionDetector::cloneWithoutCollisionObjects() const
{
  return DARTCollisionDetector::create();
}

//==============================================================================
const std::string& DARTCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DARTCollisionDetector::getStaticType()
{
  static const std::string type = "dart";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(DARTCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get()) {
    DART_ERROR(
        "Attempting to check collision for a collision group that is created "
        "from a different collision detector instance.");

    return false;
  }

  return true;
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts) [[unlikely]] {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  if (!checkGroupValidity(this, group))
    return false;

  auto casted = static_cast<DARTCollisionGroup*>(group);
  const auto& objects = casted->mCollisionObjects;

  return mEngine->collide(objects, option, result);
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts) {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  auto casted1 = static_cast<DARTCollisionGroup*>(group1);
  auto casted2 = static_cast<DARTCollisionGroup*>(group2);

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  return mEngine->collide(objects1, objects2, option, result);
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_WARN(
      "This collision detector does not support (signed) distance queries. "
      "Returning 0.0.");

  return 0.0;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_WARN(
      "This collision detector does not support (signed) distance queries. "
      "Returning 0.0.");

  return 0.0;
}

//==============================================================================
DARTCollisionDetector::DARTCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
  mEngine = std::make_unique<DartCollisionEngine>();
}

//==============================================================================
void warnUnsupportedShapeType(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto& shape = shapeFrame->getShape();
  const auto& shapeType = shape->getType();

  if (shapeType == dynamics::SphereShape::getStaticType())
    return;

  if (shapeType == dynamics::BoxShape::getStaticType())
    return;

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);

    if (ellipsoid->isSphere())
      return;
  }

  DART_ERROR(
      "[DARTCollisionDetector] Attempting to create shape type [{}] that is "
      "not supported by DARTCollisionDetector. Currently, only BoxShape and "
      "EllipsoidShape (only when all the radii are equal) are supported. This "
      "shape will always get penetrated by other objects.",
      shapeType);
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  warnUnsupportedShapeType(shapeFrame);

  return std::unique_ptr<DARTCollisionObject>(
      new DARTCollisionObject(this, shapeFrame));
}

//==============================================================================
void DARTCollisionDetector::refreshCollisionObject(CollisionObject* /*object*/)
{
  // Do nothing
}
} // namespace collision
} // namespace dart
