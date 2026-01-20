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

#include <dart/collision/experimental/sdf/dense_tsdf_field.hpp>
#include <dart/collision/experimental/sdf/signed_distance_field.hpp>

#include <Eigen/Core>

#include <limits>
#include <vector>

#include <cstdint>

namespace dart::collision::experimental {

struct DART_COLLISION_EXPERIMENTAL_API EsdfBuildOptions
{
  double surfaceDistance = 0.0;
  double maxDistance = std::numeric_limits<double>::infinity();
  double minWeight = 1e-6;
  bool useDiagonalNeighbors = true;
};

class DART_COLLISION_EXPERIMENTAL_API DenseEsdfField final
  : public SignedDistanceField
{
public:
  DenseEsdfField(
      const Eigen::Vector3d& origin,
      const Eigen::Vector3i& dims,
      double voxel_size,
      double max_distance = std::numeric_limits<double>::infinity());

  void setDistance(const Eigen::Vector3i& index, double distance);
  void setObserved(const Eigen::Vector3i& index, bool observed);

  [[nodiscard]] const Eigen::Vector3d& origin() const;
  [[nodiscard]] const Eigen::Vector3i& dims() const;

  bool buildFromTsdf(
      const DenseTsdfField& tsdf,
      const EsdfBuildOptions& options = EsdfBuildOptions());

  bool distance(
      const Eigen::Vector3d& point_F,
      double* distance,
      const SdfQueryOptions& options) const override;

  bool distanceAndGradient(
      const Eigen::Vector3d& point_F,
      double* distance,
      Eigen::Vector3d* gradient,
      const SdfQueryOptions& options) const override;

  void batchDistanceAndGradient(
      std::span<const Eigen::Vector3d> points_F,
      std::span<double> distances,
      std::span<Eigen::Vector3d> gradients,
      std::span<std::uint8_t> observed,
      const SdfQueryOptions& options) const override;

  [[nodiscard]] Aabb localAabb() const override;
  [[nodiscard]] double voxelSize() const override;
  [[nodiscard]] double maxDistance() const override;

private:
  bool isValidIndex(const Eigen::Vector3i& index) const;
  std::size_t toLinear(const Eigen::Vector3i& index) const;

  bool sampleDistance(
      const Eigen::Vector3d& point_F,
      bool interpolate,
      bool require_observed,
      double max_distance,
      double* distance) const;

  Eigen::Vector3d origin_;
  Eigen::Vector3i dims_;
  double voxel_size_ = 0.0;
  double max_distance_ = std::numeric_limits<double>::infinity();

  std::vector<double> distances_;
  std::vector<std::uint8_t> observed_;
};

} // namespace dart::collision::experimental
