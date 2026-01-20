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

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/export.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <span>

namespace dart::collision::experimental {

struct DART_COLLISION_EXPERIMENTAL_API SdfQueryOptions
{
  bool interpolate = true;
  bool requireObserved = true;
  double maxDistance = std::numeric_limits<double>::infinity();
};

struct DART_COLLISION_EXPERIMENTAL_API SdfQueryResult
{
  double distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  bool observed = false;
};

class DART_COLLISION_EXPERIMENTAL_API SignedDistanceField
{
public:
  virtual ~SignedDistanceField() = default;

  virtual bool distance(
      const Eigen::Vector3d& point_F,
      double* distance,
      const SdfQueryOptions& options) const = 0;

  virtual bool distanceAndGradient(
      const Eigen::Vector3d& point_F,
      double* distance,
      Eigen::Vector3d* gradient,
      const SdfQueryOptions& options) const = 0;

  virtual void batchDistanceAndGradient(
      std::span<const Eigen::Vector3d> points_F,
      std::span<double> distances,
      std::span<Eigen::Vector3d> gradients,
      std::span<std::uint8_t> observed,
      const SdfQueryOptions& options) const = 0;

  [[nodiscard]] virtual Aabb localAabb() const = 0;
  [[nodiscard]] virtual double voxelSize() const = 0;
  [[nodiscard]] virtual double maxDistance() const = 0;
};

}  // namespace dart::collision::experimental
