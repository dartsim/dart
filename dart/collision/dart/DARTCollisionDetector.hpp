/*
 * Copyright (c) 2011, The DART development contributors
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

#ifndef DART_COLLISION_DART_DARTCOLLISIONDETECTOR_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONDETECTOR_HPP_

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/dart/Fwd.hpp>

#include <memory>

namespace dart {
namespace collision {

class DARTCollisionObject;

class DARTCollisionDetector : public CollisionDetector
{
public:
  using CollisionDetector::createCollisionGroup;

  static std::shared_ptr<DARTCollisionDetector> create();

  ~DARTCollisionDetector() override;

  // Documentation inherited
  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override;

  // Documentation inherited
  const std::string& getType() const override;

  /// Get collision detector type for this class.
  static const std::string& getStaticType();

  /// Sets the number of worker participants requested for this detector.
  /// Preserved for source compatibility with the pre-consolidation
  /// DARTCollisionDetector API (and ConstraintSolver's thread-count
  /// propagation); the underlying engine does not yet parallelize collision
  /// queries across threads, so this is bookkeeping only.
  void setNumCollisionThreads(std::size_t numThreads);

  /// Returns the value most recently passed to setNumCollisionThreads().
  std::size_t getNumCollisionThreads() const;

  // Documentation inherited
  std::unique_ptr<CollisionGroup> createCollisionGroup() override;

  // Documentation inherited
  bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override;

  // Documentation inherited
  double distance(
      CollisionGroup* group,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  // Documentation inherited
  double distance(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) override;

  // Documentation inherited
  bool raycast(
      CollisionGroup* group,
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option = RaycastOption(),
      RaycastResult* result = nullptr) override;

  [[nodiscard]] native::CachedContact* getCachedContact(
      const DARTCollisionObject* object1,
      const DARTCollisionObject* object2,
      void* userData) const;

protected:
  DARTCollisionDetector();

  // Documentation inherited
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override;

  // Documentation inherited
  void refreshCollisionObject(CollisionObject* object) override;

  // Documentation inherited
  void notifyCollisionObjectDestroying(CollisionObject* object) override;

private:
  static Registrar<DARTCollisionDetector> mRegistrar;
  // Transition alias: "native" was the interim key for this engine before
  // it was folded into DARTCollisionDetector as "dart". 6.20.0 is
  // unreleased, so no in-tree caller should rely on it going forward.
  // Remove in 6.21.
  static Registrar<DARTCollisionDetector> mNativeAliasRegistrar;

  std::size_t mNumCollisionThreads{1u};
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONDETECTOR_HPP_
