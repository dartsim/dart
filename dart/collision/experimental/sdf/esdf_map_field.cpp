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

#include <dart/collision/experimental/sdf/esdf_map_field.hpp>

#if DART_EXPERIMENTAL_HAVE_ESDF_MAP
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace dart::collision::experimental {

EsdfMapField::EsdfMapField(
    std::shared_ptr<const voxblox::EsdfMap> map,
    double max_distance)
  : map_(std::move(map)), max_distance_(max_distance)
{
}

const voxblox::EsdfMap* EsdfMapField::getMap() const
{
  return map_.get();
}

bool EsdfMapField::distance(
    const Eigen::Vector3d& point_F,
    double* distance,
    const SdfQueryOptions& options) const
{
  if (!map_) {
    return false;
  }

  double dist = 0.0;
  bool ok = map_->getDistanceAtPosition(point_F, options.interpolate, &dist);
  if (!ok && options.requireObserved) {
    return false;
  }

  if (distance) {
    *distance = dist;
  }

  if (dist > options.maxDistance) {
    return false;
  }
  return ok || !options.requireObserved;
}

bool EsdfMapField::distanceAndGradient(
    const Eigen::Vector3d& point_F,
    double* distance,
    Eigen::Vector3d* gradient,
    const SdfQueryOptions& options) const
{
  if (!map_) {
    return false;
  }

  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  bool ok = map_->getDistanceAndGradientAtPosition(
      point_F, options.interpolate, &dist, &grad);
  if (!ok && options.requireObserved) {
    return false;
  }

  if (distance) {
    *distance = dist;
  }
  if (gradient) {
    *gradient = grad;
  }

  if (dist > options.maxDistance) {
    return false;
  }
  return ok || !options.requireObserved;
}

void EsdfMapField::batchDistanceAndGradient(
    std::span<const Eigen::Vector3d> points_F,
    std::span<double> distances,
    std::span<Eigen::Vector3d> gradients,
    std::span<std::uint8_t> observed,
    const SdfQueryOptions& options) const
{
  const std::size_t count = std::min(
      {points_F.size(), distances.size(), gradients.size(), observed.size()});
  if (!map_) {
    for (std::size_t i = 0; i < count; ++i) {
      distances[i] = std::numeric_limits<double>::infinity();
      gradients[i] = Eigen::Vector3d::Zero();
      observed[i] = 0;
    }
    return;
  }

  if (options.interpolate) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> positions(3, count);
    Eigen::VectorXd dist_vec(count);
    Eigen::Matrix<double, 3, Eigen::Dynamic> grad_mat(3, count);
    Eigen::VectorXi obs_vec(count);

    for (std::size_t i = 0; i < count; ++i) {
      positions.col(static_cast<int>(i)) = points_F[i];
    }

    map_->batchGetDistanceAndGradientAtPosition(
        positions, dist_vec, grad_mat, obs_vec);

    for (std::size_t i = 0; i < count; ++i) {
      const double dist = dist_vec(static_cast<int>(i));
      distances[i] = dist;
      gradients[i] = grad_mat.col(static_cast<int>(i));
      observed[i] = static_cast<std::uint8_t>(obs_vec(static_cast<int>(i)) != 0);

      if (dist > options.maxDistance) {
        observed[i] = 0;
      }
      if (options.requireObserved && observed[i] == 0) {
        distances[i] = std::numeric_limits<double>::infinity();
        gradients[i] = Eigen::Vector3d::Zero();
      }
    }
    return;
  }

  for (std::size_t i = 0; i < count; ++i) {
    double dist = 0.0;
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    bool ok = map_->getDistanceAndGradientAtPosition(
        points_F[i], false, &dist, &grad);
    if (dist > options.maxDistance) {
      ok = false;
    }
    if (options.requireObserved && !ok) {
      distances[i] = std::numeric_limits<double>::infinity();
      gradients[i] = Eigen::Vector3d::Zero();
      observed[i] = 0;
      continue;
    }
    distances[i] = dist;
    gradients[i] = grad;
    observed[i] = static_cast<std::uint8_t>(ok);
  }
}

Aabb EsdfMapField::localAabb() const
{
  if (!map_) {
    return Aabb();
  }

  const voxblox::Layer<voxblox::EsdfVoxel>* layer =
      map_->getEsdfLayerConstPtr();
  if (!layer) {
    return Aabb();
  }

  voxblox::BlockIndexList blocks;
  layer->getAllAllocatedBlocks(&blocks);
  if (blocks.empty()) {
    return Aabb();
  }

  bool first = true;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  const double block_size = layer->block_size();

  for (const auto& index : blocks) {
    auto block = layer->getBlockPtrByIndex(index);
    if (!block) {
      continue;
    }
    const auto origin = block->origin();
    Eigen::Vector3d block_min(origin.x(), origin.y(), origin.z());
    Eigen::Vector3d block_max = block_min
        + Eigen::Vector3d::Constant(block_size);

    if (first) {
      min = block_min;
      max = block_max;
      first = false;
    } else {
      min = min.cwiseMin(block_min);
      max = max.cwiseMax(block_max);
    }
  }

  return Aabb(min, max);
}

double EsdfMapField::voxelSize() const
{
  if (!map_) {
    return 0.0;
  }
  return map_->voxel_size();
}

double EsdfMapField::maxDistance() const
{
  return max_distance_;
}

}  // namespace dart::collision::experimental
#endif
