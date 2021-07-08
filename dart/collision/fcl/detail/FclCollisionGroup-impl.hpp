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

#include "dart/collision/fcl/FclCollisionGroup.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/fcl/FclCollisionDetector.hpp"
#include "dart/collision/fcl/FclCollisionObject.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/geometry/Sphere.hpp"

namespace dart {
namespace collision2 {

//==============================================================================
template <typename S>
std::shared_ptr<fcl::CollisionGeometry<S>> createFCLCollisionGeometry(
    const math::ConstGeometryPtr& shape,
    typename FclCollisionDetector<S>::PrimitiveShape /*type*/)
{
  fcl::CollisionGeometry<S>* geom = nullptr;

  if (shape->is<math::Sphered>())
  {
    return nullptr;
  }

  return std::shared_ptr<fcl::CollisionGeometry<S>>(geom);
}

//==============================================================================
template <typename S>
FclCollisionGroup<S>::FclCollisionGroup(CollisionDetector<S>* collisionDetector)
  : CollisionGroup<S>(collisionDetector),
    mBroadPhaseAlg(
        new dart::collision2::fcl::DynamicAABBTreeCollisionManager<S>())
{
  // Do nothing
}

//==============================================================================
template <typename S>
CollisionObjectPtr<S> FclCollisionGroup<S>::createCollisionObject(
    math::GeometryPtr shape)
{
  if (!shape)
  {
    dtwarn
        << "Not allowed to create a collision object for a shape of nullptr.\n";
    return nullptr;
  }

  auto fclCollisionGeometry
      = createFCLCollisionGeometry<S>(shape, FclCollisionDetector<S>::MESH);
  // TODO(JS): Check if fclCollisionGeometry is nullptr

  return std::shared_ptr<FclCollisionObject<S>>(
      new FclCollisionObject<S>(this, std::move(shape), fclCollisionGeometry));
}

//==============================================================================
// template <typename S>
// FclCollisionDetector<S>* FclCollisionGroup<S>::getFclCollisionDetector()
//{
//  return static_cast<FclCollisionDetector<S>*>(this->mCollisionDetector);
//}

//==============================================================================
template <typename S>
FclCollisionDetector<S>* FclCollisionGroup<S>::getFclCollisionDetector()
{
  return static_cast<FclCollisionDetector<S>*>(this->mCollisionDetector);
}

//==============================================================================
template <typename S>
const FclCollisionDetector<S>* FclCollisionGroup<S>::getFclCollisionDetector()
    const
{
  return static_cast<const FclCollisionDetector<S>*>(this->mCollisionDetector);
}

////==============================================================================
// void FCLCollisionGroup<S>::initializeEngineData()
//{
//  mBroadPhaseAlg->setup();
//}

////==============================================================================
// void FCLCollisionGroup<S>::addCollisionObjectToEngine(CollisionObject*
// object)
//{
//  auto casted = static_cast<FCLCollisionObject*>(object);
//  mBroadPhaseAlg->registerObject(casted->getFCLCollisionObject());

//  initializeEngineData();
//}

////==============================================================================
// void FCLCollisionGroup<S>::addCollisionObjectsToEngine(
//    const std::vector<CollisionObject*>& collObjects)
//{
//  for (auto collObj : collObjects)
//  {
//    auto casted = static_cast<FCLCollisionObject*>(collObj);

//    mBroadPhaseAlg->registerObject(casted->getFCLCollisionObject());
//  }

//  initializeEngineData();
//}

////==============================================================================
// void FCLCollisionGroup<S>::removeCollisionObjectFromEngine(CollisionObject*
// object)
//{
//  auto casted = static_cast<FCLCollisionObject*>(object);

//  mBroadPhaseAlg->unregisterObject(casted->getFCLCollisionObject());

//  initializeEngineData();
//}

////==============================================================================
// void FCLCollisionGroup<S>::removeAllCollisionObjectsFromEngine()
//{
//  mBroadPhaseAlg->clear();

//  initializeEngineData();
//}

////==============================================================================
// void FCLCollisionGroup<S>::updateCollisionGroupEngineData()
//{
//  mBroadPhaseAlg->update();
//}

//==============================================================================
template <typename S>
typename FclCollisionGroup<S>::FCLCollisionManager*
FclCollisionGroup<S>::getFCLCollisionManager()
{
  return mBroadPhaseAlg.get();
}

//==============================================================================
template <typename S>
const typename FclCollisionGroup<S>::FCLCollisionManager*
FclCollisionGroup<S>::getFCLCollisionManager() const
{
  return mBroadPhaseAlg.get();
}

} // namespace collision2
} // namespace dart
