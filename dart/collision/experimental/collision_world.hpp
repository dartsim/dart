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

#pragma once

#include <dart/collision/experimental/batch.hpp>
#include <dart/collision/experimental/broad_phase/broad_phase.hpp>
#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/fwd.hpp>
#include <dart/collision/experimental/types.hpp>

#include <entt/entt.hpp>

#include <memory>
#include <vector>

namespace dart::collision::experimental {

class DART_COLLISION_EXPERIMENTAL_API CollisionWorld
{
public:
  explicit CollisionWorld(
      BroadPhaseType broadPhaseType = BroadPhaseType::AabbTree);
  ~CollisionWorld();

  CollisionWorld(const CollisionWorld&) = delete;
  CollisionWorld& operator=(const CollisionWorld&) = delete;
  CollisionWorld(CollisionWorld&&) = default;
  CollisionWorld& operator=(CollisionWorld&&) = default;

  CollisionObject createObject(
      std::unique_ptr<Shape> shape,
      const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity());

  void destroyObject(CollisionObject object);

  void updateObject(CollisionObject object);

  [[nodiscard]] std::size_t numObjects() const;
  [[nodiscard]] CollisionObject getObject(std::size_t index);

  [[nodiscard]] std::size_t updateAll();

  [[nodiscard]] std::size_t updateAll(
      const BatchSettings& settings, BatchStats* stats = nullptr);

  [[nodiscard]] BroadPhaseSnapshot buildBroadPhaseSnapshot() const;

  [[nodiscard]] BroadPhaseSnapshot buildBroadPhaseSnapshot(
      const BatchSettings& settings) const;

  void buildBroadPhaseSnapshot(BroadPhaseSnapshot& out) const;

  void buildBroadPhaseSnapshot(
      BroadPhaseSnapshot& out, const BatchSettings& settings) const;

  bool collideAll(
      const BroadPhaseSnapshot& snapshot,
      const CollisionOption& option,
      CollisionResult& result,
      BatchStats* stats = nullptr);

  bool collideAll(
      const BroadPhaseSnapshot& snapshot,
      const CollisionOption& option,
      CollisionResult& result,
      const BatchSettings& settings,
      BatchStats* stats = nullptr);

  bool collideAll(
      const BroadPhaseSnapshot& snapshot,
      const CollisionOption& option,
      BatchOutput& out,
      const BatchSettings& settings = BatchSettings(),
      BatchStats* stats = nullptr);

  bool collide(const CollisionOption& option, CollisionResult& result);

  bool collide(
      CollisionObject obj1,
      CollisionObject obj2,
      const CollisionOption& option,
      CollisionResult& result);

  bool raycast(
      const Ray& ray, const RaycastOption& option, RaycastResult& result);

  bool raycastAll(
      const Ray& ray,
      const RaycastOption& option,
      std::vector<RaycastResult>& results);

  bool sphereCast(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      double radius,
      const CcdOption& option,
      CcdResult& result);

  bool sphereCastAll(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      double radius,
      const CcdOption& option,
      std::vector<CcdResult>& results);

  bool capsuleCast(
      const Eigen::Isometry3d& capsuleStart,
      const Eigen::Isometry3d& capsuleEnd,
      const CapsuleShape& capsule,
      const CcdOption& option,
      CcdResult& result);

  bool capsuleCastAll(
      const Eigen::Isometry3d& capsuleStart,
      const Eigen::Isometry3d& capsuleEnd,
      const CapsuleShape& capsule,
      const CcdOption& option,
      std::vector<CcdResult>& results);

  void clear();

  [[nodiscard]] entt::registry& getRegistry()
  {
    return m_registry;
  }

  [[nodiscard]] const entt::registry& getRegistry() const
  {
    return m_registry;
  }

  [[nodiscard]] BroadPhaseType getBroadPhaseType() const
  {
    return m_broadPhaseType;
  }

  [[nodiscard]] BroadPhase& getBroadPhase()
  {
    return *m_broadPhase;
  }

  [[nodiscard]] const BroadPhase& getBroadPhase() const
  {
    return *m_broadPhase;
  }

private:
  static std::unique_ptr<BroadPhase> createBroadPhase(BroadPhaseType type);

  entt::registry m_registry;
  BroadPhaseType m_broadPhaseType;
  std::unique_ptr<BroadPhase> m_broadPhase;
};

} // namespace dart::collision::experimental
