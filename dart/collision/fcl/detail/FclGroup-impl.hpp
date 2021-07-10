/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#pragma once

#include "dart/collision/fcl/FclGroup.hpp"

#include "dart/collision/Object.hpp"
#include "dart/collision/fcl/FclEngine.hpp"
#include "dart/collision/fcl/FclObject.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/geometry/Sphere.hpp"

namespace dart {
namespace collision2 {

//==============================================================================
template <typename S>
std::shared_ptr<FclCollisionGeometry<S>> createFclCollisionGeometry(
    const math::ConstGeometryPtr& shape,
    typename FclEngine<S>::PrimitiveShape /*type*/)
{
  FclCollisionGeometry<S>* geom = nullptr;

  if (auto sphere = shape->as<math::Sphered>())
  {
    geom = new FclSphere<S>(sphere->getRadius());
  }
  else
  {
    dterr << "Unsupported geometry type: " << shape->getType() << "\n";
  }

  return std::shared_ptr<FclCollisionGeometry<S>>(geom);
}

//==============================================================================
template <typename S>
FclGroup<S>::FclGroup(Engine<S>* collisionDetector)
  : Group<S>(collisionDetector),
    mBroadPhaseAlg(new FclDynamicAABBTreeCollisionManager<S>())
{
  // Do nothing
}

//==============================================================================
template <typename S>
ObjectPtr<S> FclGroup<S>::createObject(math::GeometryPtr shape)
{
  if (!shape)
  {
    dtwarn
        << "Not allowed to create a collision object for a shape of nullptr.\n";
    return nullptr;
  }

  auto fclCollisionGeometry
      = createFclCollisionGeometry<S>(shape, FclEngine<S>::MESH);
  if (!fclCollisionGeometry)
  {
    dtwarn << "Failed to create FCL collision geometry.\n";
    return nullptr;
  }

  return std::shared_ptr<FclObject<S>>(
      new FclObject<S>(this, std::move(shape), fclCollisionGeometry));
}

//==============================================================================
// template <typename S>
// FclEngine<S>* FclGroup<S>::getFclEngine()
//{
//  return static_cast<FclEngine<S>*>(this->mEngine);
//}

//==============================================================================
template <typename S>
FclEngine<S>* FclGroup<S>::getFclEngine()
{
  return static_cast<FclEngine<S>*>(this->mEngine);
}

//==============================================================================
template <typename S>
const FclEngine<S>* FclGroup<S>::getFclEngine() const
{
  return static_cast<const FclEngine<S>*>(this->mEngine);
}

////==============================================================================
// void FCLGroup<S>::initializeEngineData()
//{
//  mBroadPhaseAlg->setup();
//}

////==============================================================================
// void FCLGroup<S>::addObjectToEngine(Object*
// object)
//{
//  auto casted = static_cast<FCLObject*>(object);
//  mBroadPhaseAlg->registerObject(casted->getFCLObject());

//  initializeEngineData();
//}

////==============================================================================
// void FCLGroup<S>::addObjectsToEngine(
//    const std::vector<Object*>& collObjects)
//{
//  for (auto collObj : collObjects)
//  {
//    auto casted = static_cast<FCLObject*>(collObj);

//    mBroadPhaseAlg->registerObject(casted->getFCLObject());
//  }

//  initializeEngineData();
//}

////==============================================================================
// void FCLGroup<S>::removeObjectFromEngine(Object*
// object)
//{
//  auto casted = static_cast<FCLObject*>(object);

//  mBroadPhaseAlg->unregisterObject(casted->getFCLObject());

//  initializeEngineData();
//}

////==============================================================================
// void FCLGroup<S>::removeAllObjectsFromEngine()
//{
//  mBroadPhaseAlg->clear();

//  initializeEngineData();
//}

////==============================================================================
// void FCLGroup<S>::updateGroupEngineData()
//{
//  mBroadPhaseAlg->update();
//}

//==============================================================================
template <typename S>
typename FclGroup<S>::FCLCollisionManager* FclGroup<S>::getFCLCollisionManager()
{
  return mBroadPhaseAlg.get();
}

//==============================================================================
template <typename S>
const typename FclGroup<S>::FCLCollisionManager*
FclGroup<S>::getFCLCollisionManager() const
{
  return mBroadPhaseAlg.get();
}

} // namespace collision2
} // namespace dart
