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

#include "dart/collision/dart/DARTEngine.h"

#include "dart/collision/CollisionObject.h"
#include "dart/collision/dart/DARTCollide.h"
#include "dart/collision/dart/DARTCollisionObjectData.h"
#include "dart/collision/dart/DARTCollisionGroupData.h"

namespace dart {
namespace collision {

namespace {

bool checkPair(CollisionObject* o1, CollisionObject* o2,
                       Result& result);

bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol);

void postProcess(CollisionObject* o1, CollisionObject* o2, Result& result);

} // anonymous namespace



//==============================================================================
DARTEnginePtr DARTEngine::create()
{
  return DARTEnginePtr(new DARTEngine());
}

//==============================================================================
DARTEngine::DARTEngine()
{
  // Do nothing
}

//==============================================================================
DARTEngine::~DARTEngine()
{
  // Do nothing
}

//==============================================================================
const std::string& DARTEngine::getTypeStatic()
{
  static const std::string& type("DART");
  return type;
}

//==============================================================================
const std::string& DARTEngine::getType() const
{
  return getTypeStatic();
}

//==============================================================================
std::unique_ptr<CollisionObjectData> DARTEngine::createCollisionObjectData(
    CollisionObject* parent,
    const dynamics::ShapePtr& /*shape*/)
{
  return std::unique_ptr<CollisionObjectData>(
        new DARTCollisionObjectData(this, parent));
}

//==============================================================================
std::unique_ptr<CollisionGroupData> DARTEngine::createCollisionGroupData(
    CollisionGroup* parent,
    const CollisionObjectPtrs& collObjects)
{
  return std::unique_ptr<CollisionGroupData>(
        new DARTCollisionGroupData(this, parent, collObjects));
}

//==============================================================================
bool DARTEngine::detect(
    CollisionObjectData* objectData1,
    CollisionObjectData* objectData2,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(objectData1->getEngine()->getType() == DARTEngine::getTypeStatic());
  assert(objectData2->getEngine()->getType() == DARTEngine::getTypeStatic());

  auto collObj1 = objectData1->getCollisionObject();
  auto collObj2 = objectData2->getCollisionObject();

  return checkPair(collObj1, collObj2, result);
}

//==============================================================================
bool DARTEngine::detect(
    CollisionObjectData* objectData,
    CollisionGroupData* groupData,
    const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(objectData);
  assert(groupData);
  assert(objectData->getEngine()->getType() == DARTEngine::getTypeStatic());
  assert(groupData->getEngine()->getType() == DARTEngine::getTypeStatic());

  auto collObj1 = objectData->getCollisionObject();
  auto collGrp = groupData->getCollisionGroup();
  auto collObjs2 = collGrp->getCollisionObjects();

  for (auto collObj2 : collObjs2)
    checkPair(collObj1, collObj2.get(), result);

  return !result.contacts.empty();
}

//==============================================================================
bool DARTEngine::detect(CollisionGroupData* groupData,
                       const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(groupData);
  assert(groupData->getEngine()->getType() == DARTEngine::getTypeStatic());

  auto collGrp = groupData->getCollisionGroup();
  auto collObjs = collGrp->getCollisionObjects();

  if (collObjs.empty())
    return false;

  for (auto i = 0u; i < collObjs.size() - 1; ++i)
  {
    auto collObj1 = collObjs[i];

    for (auto j = i + 1u; j < collObjs.size(); ++j)
    {
      auto collObj2 = collObjs[j];

      checkPair(collObj1.get(), collObj2.get(), result);
    }
  }

  return !result.contacts.empty();
}

//==============================================================================
bool DARTEngine::detect(CollisionGroupData* groupData1,
                       CollisionGroupData* groupData2,
                       const Option& /*option*/, Result& result)
{
  result.contacts.clear();

  assert(groupData1);
  assert(groupData2);
  assert(groupData1->getEngine()->getType() == DARTEngine::getTypeStatic());
  assert(groupData2->getEngine()->getType() == DARTEngine::getTypeStatic());

  auto collGrp1 = groupData1->getCollisionGroup();
  auto collGrp2 = groupData2->getCollisionGroup();

  auto collObjs1 = collGrp1->getCollisionObjects();
  auto collObjs2 = collGrp2->getCollisionObjects();

  if (collObjs1.empty() || collObjs2.empty())
    return false;

  for (auto i = 0u; i < collObjs1.size(); ++i)
  {
    auto collObj1 = collObjs1[i];

    for (auto j = 0u; j < collObjs2.size(); ++j)
    {
      auto collObj2 = collObjs2[j];

      checkPair(collObj1.get(), collObj2.get(), result);
    }
  }

  return !result.contacts.empty();
}



namespace {

//==============================================================================
bool checkPair(CollisionObject* o1, CollisionObject* o2, Result& result)
{
  // TODO(JS): filtering

  // Perform narrow-phase detection
  auto colliding = collide(o1->getShape(), o1->getTransform(),
                           o2->getShape(), o2->getTransform(),
                           &result.contacts);

  postProcess(o1, o2, result);

  return colliding != 0;
}

//==============================================================================
bool isClose(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
             double tol)
{
  return (pos1 - pos2).norm() < tol;
}

//==============================================================================
void postProcess(CollisionObject* o1, CollisionObject* o2,
                 Result& result)
{
  if (result.contacts.empty())
    return;

  // Remove all the repeated points
  const auto tol = 3.0e-12;
  auto i = result.contacts.begin();
  while (i != result.contacts.end() - 1)
  {
    for (auto j = i + 1; j != result.contacts.end(); ++j)
    {
      if (isClose(i->point, j->point, tol))
      {
        i = result.contacts.erase(i);
        break;
      }
    }

    ++i;
  }

  for (auto& contact : result.contacts)
  {
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
  }

  std::cout << "=================" << std::endl;
  std::cout << "# of contacts:" << result.contacts.size() << std::endl;
  std::cout << "=================" << std::endl;
  for (auto i = 0u; i < result.contacts.size(); ++i)
  {
    auto contact = result.contacts[i];

    std::cout << "Contact (" << i << ")" << std::endl;
    std::cout << "Point : " << contact.point.transpose() << std::endl;
    std::cout << "Normal: " << contact.normal.transpose() << std::endl;
    std::cout << "Depth : " << contact.penetrationDepth << std::endl;
    std::cout << std::endl;
  }
  std::cout << "=================" << std::endl;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
