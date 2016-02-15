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

#ifndef DART_COLLISION_ENGINE_H_
#define DART_COLLISION_ENGINE_H_

#include <vector>
#include <map>

#include <Eigen/Dense>

#include "dart/config.h"
#include "dart/common/Console.h"
#include "dart/collision/Contact.h"
#include "dart/collision/CollisionGroup.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace collision {

class CollisionGroup;
class CollisionObject;
class CollisionObjectData;

enum EngineType
{
  FCL    = 0,
#ifdef HAVE_BULLET_COLLISION
  Bullet,
#endif
  DART,
};

struct Option
{
  bool enableContact;
  size_t maxNumContacts;

  Option(bool enableContact = true,
         size_t maxNumContacts = 100);
};

struct Result
{
  std::vector<Contact> contacts;

  void clear();

  bool empty() const;
};

namespace Engine
{

CollisionObjectData* createCollisionObjectData(EngineType type,
                                               CollisionObject* parent,
                                               const dynamics::ShapePtr& shape);

bool detect(CollisionObject* object1,
            CollisionObject* object2,
            const collision::Option& option,
            collision::Result& result);

}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_ENGINE_H_
