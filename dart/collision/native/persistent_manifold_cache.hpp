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

#include <dart/collision/native/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>

#include <cstddef>

namespace dart::collision::native {

struct DART_COLLISION_NATIVE_API CachedContact
{
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double penetrationDepth = 0.0;
  double cachedNormalImpulse = 0.0;
  double cachedFrictionImpulse1 = 0.0;
  double cachedFrictionImpulse2 = 0.0;
  int lifetime = 0;
};

struct DART_COLLISION_NATIVE_API PersistentManifold
{
  static constexpr int kMaxContacts = 4;

  std::array<CachedContact, kMaxContacts> contacts;
  int numContacts = 0;

  [[nodiscard]] int findMatch(
      const Eigen::Vector3d& localPointA, double threshold = 0.02) const;

  void addOrReplace(const CachedContact& contact);

  void reduce();

  void refresh(
      const Eigen::Isometry3d& tfA,
      const Eigen::Isometry3d& tfB,
      double breakingThreshold = 0.04);
};

struct DART_COLLISION_NATIVE_API PairKey
{
  std::size_t idA = 0;
  std::size_t idB = 0;

  [[nodiscard]] bool operator==(const PairKey& other) const;
};

struct DART_COLLISION_NATIVE_API PairKeyHash
{
  [[nodiscard]] std::size_t operator()(const PairKey& key) const;
};

class DART_COLLISION_NATIVE_API PersistentManifoldCache
{
public:
  using TransformProvider = std::function<
      std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>>(
          std::size_t idA, std::size_t idB)>;

  PersistentManifold& getOrCreate(std::size_t idA, std::size_t idB);

  void remove(std::size_t idA, std::size_t idB);

  void refreshAll(
      const TransformProvider& transformProvider,
      double breakingThreshold = 0.04);

  void clear();

  [[nodiscard]] std::size_t size() const;

private:
  std::unordered_map<PairKey, PersistentManifold, PairKeyHash> manifolds_;
};

} // namespace dart::collision::native
