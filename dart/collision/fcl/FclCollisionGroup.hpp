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

#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/Types.hpp"
#include "dart/collision/fcl/BackwardCompatibility.hpp"
#include "dart/collision/fcl/FclTypes.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class FclCollisionGroup : public CollisionGroup<S_>
{
public:
  using S = S_;
  using FCLCollisionManager
      = dart::collision2::fcl::DynamicAABBTreeCollisionManager<S>;

  friend class FclCollisionDetector<S>;

  /// Constructor
  FclCollisionGroup(CollisionDetector<S>* collisionDetector);

  /// Destructor
  virtual ~FclCollisionGroup() = default;

  CollisionObjectPtr<S> createCollisionObject(math::GeometryPtr shape) override;

protected:
  FclCollisionDetector<S>* getFclCollisionDetector();
  const FclCollisionDetector<S>* getFclCollisionDetector() const;

  //  using CollisionGroup::updateEngineData;

  //  // Documentation inherited
  //  void initializeEngineData() override;

  //  // Documentation inherited
  //  void addCollisionObjectToEngine(CollisionObject* object) override;

  //  // Documentation inherited
  //  void addCollisionObjectsToEngine(
  //      const std::vector<CollisionObject*>& collObjects) override;

  //  // Documentation inherited
  //  void removeCollisionObjectFromEngine(CollisionObject* object) override;

  //  // Documentation inherited
  //  void removeAllCollisionObjectsFromEngine() override;

  //  // Documentation inherited
  //  void updateCollisionGroupEngineData() override;

  /// Return FCL collision manager that is also a broad-phase algorithm
  FCLCollisionManager* getFCLCollisionManager();

  /// Return FCL collision manager that is also a broad-phase algorithm
  const FCLCollisionManager* getFCLCollisionManager() const;

protected:
  /// FCL broad-phase algorithm
  std::unique_ptr<FCLCollisionManager> mBroadPhaseAlg;
};

extern template class FclCollisionGroup<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/fcl/detail/FclCollisionGroup-impl.hpp"
