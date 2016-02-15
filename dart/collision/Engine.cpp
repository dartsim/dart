/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/collision/Engine.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/collision/CollisionNode.h"
#include "dart/collision/CollisionObject.h"
#include "dart/collision/CollisionObjectData.h"
#include "dart/collision/fcl/FCLEngine.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionObjectData.h"
#endif

namespace dart {
namespace collision {

//==============================================================================
Option::Option(bool enableContact, size_t maxNumContacts)
  : enableContact(enableContact),
    maxNumContacts(maxNumContacts)
{
  // Do nothing
}

//==============================================================================
void Result::clear()
{
  contacts.clear();
}

//=============================================================================
bool Result::empty() const
{
  return contacts.empty();
}

//=============================================================================
CollisionObjectData* Engine::createCollisionObjectData(
    EngineType type,
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
{
  switch (type)
  {
  case FCL:
    return FCLEngine::createCollisionObjectData(parent, shape);
    break;
  default:
    dtwarn << "Unsupported collision detection engine '" << type << "'.\n";
    return nullptr;
    break;
  }
}

//==============================================================================
bool Engine::detect(CollisionObject* object1,
                    CollisionObject* object2,
                    const Option& option, Result& result)
{
  if (!object1 || !object2)
  {
    result.clear();
    return false;
  }

  auto engineType1 = object1->getEngineType();
  auto engineType2 = object2->getEngineType();

  if (engineType1 != engineType2)
  {
    result.clear();
    return false;
  }
  // TODO(JS): we can consider changing one engine of the two objects, or
  // cloning one object whose engine is the same with other's.

  switch (engineType1)
  {
  case FCL:
    return FCLEngine::detect(object1, object2, option, result);
    break;
  default:
    dtwarn << "Unsupported collision detection engine '" << engineType1 << "'.\n";
    return nullptr;
    break;
  }
}

}  // namespace collision
}  // namespace dart
