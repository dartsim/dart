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

#include <dart/collision/experimental/sdf/dense_esdf_field.hpp>

#include <algorithm>
#include <limits>
#include <queue>
#include <vector>

#include <cmath>

namespace dart::collision::experimental {

namespace {

struct NeighborOffset
{
  Eigen::Vector3i offset;
  double cost;
};

std::vector<NeighborOffset> buildNeighborOffsets(bool use_diagonal)
{
  std::vector<NeighborOffset> offsets;
  offsets.reserve(use_diagonal ? 26 : 6);
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
        if (!use_diagonal && manhattan != 1) {
          continue;
        }
        const double cost
            = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
        offsets.push_back({Eigen::Vector3i(dx, dy, dz), cost});
      }
    }
  }
  return offsets;
}

std::vector<NeighborOffset> buildSurfaceOffsets()
{
  std::vector<NeighborOffset> offsets;
  offsets.reserve(6);
  offsets.push_back({Eigen::Vector3i(1, 0, 0), 1.0});
  offsets.push_back({Eigen::Vector3i(-1, 0, 0), 1.0});
  offsets.push_back({Eigen::Vector3i(0, 1, 0), 1.0});
  offsets.push_back({Eigen::Vector3i(0, -1, 0), 1.0});
  offsets.push_back({Eigen::Vector3i(0, 0, 1), 1.0});
  offsets.push_back({Eigen::Vector3i(0, 0, -1), 1.0});
  return offsets;
}

const std::vector<NeighborOffset>& surfaceOffsets()
{
  static const std::vector<NeighborOffset> kSurfaceOffsets
      = buildSurfaceOffsets();
  return kSurfaceOffsets;
}

const std::vector<NeighborOffset>& neighborOffsets(bool use_diagonal)
{
  static const std::vector<NeighborOffset> kSixOffsets
      = buildNeighborOffsets(false);
  static const std::vector<NeighborOffset> kTwentySixOffsets
      = buildNeighborOffsets(true);
  return use_diagonal ? kTwentySixOffsets : kSixOffsets;
}

} // namespace

DenseEsdfField::DenseEsdfField(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3i& dims,
    double voxel_size,
    double max_distance)
  : origin_(origin),
    dims_(dims),
    voxel_size_(voxel_size),
    max_distance_(max_distance)
{
  const std::size_t size = static_cast<std::size_t>(dims_.x())
                           * static_cast<std::size_t>(dims_.y())
                           * static_cast<std::size_t>(dims_.z());
  distances_.assign(size, max_distance_);
  observed_.assign(size, 0);
}

void DenseEsdfField::ensureScratch(std::size_t size)
{
  if (signs_.size() != size) {
    signs_.assign(size, 0);
  } else {
    std::fill(signs_.begin(), signs_.end(), 0);
  }

  if (surface_.size() != size) {
    surface_.assign(size, 0);
  } else {
    std::fill(surface_.begin(), surface_.end(), 0);
  }
}

void DenseEsdfField::setDistance(const Eigen::Vector3i& index, double distance)
{
  if (!isValidIndex(index)) {
    return;
  }
  distances_[toLinear(index)] = distance;
}

void DenseEsdfField::setObserved(const Eigen::Vector3i& index, bool observed)
{
  if (!isValidIndex(index)) {
    return;
  }
  observed_[toLinear(index)] = observed ? 1 : 0;
}

const Eigen::Vector3d& DenseEsdfField::origin() const
{
  return origin_;
}

const Eigen::Vector3i& DenseEsdfField::dims() const
{
  return dims_;
}

bool DenseEsdfField::buildFromTsdf(
    const DenseTsdfField& tsdf, const EsdfBuildOptions& options)
{
  if (dims_ != tsdf.dims()) {
    return false;
  }
  if (std::abs(voxel_size_ - tsdf.voxelSize()) > 1e-10) {
    return false;
  }
  if (origin_ != tsdf.origin()) {
    return false;
  }

  if (std::isfinite(options.maxDistance)) {
    max_distance_ = options.maxDistance;
  }

  const std::size_t size = static_cast<std::size_t>(dims_.x())
                           * static_cast<std::size_t>(dims_.y())
                           * static_cast<std::size_t>(dims_.z());
  if (distances_.size() != size) {
    distances_.assign(size, max_distance_);
  } else {
    std::fill(distances_.begin(), distances_.end(), max_distance_);
  }
  if (observed_.size() != size) {
    observed_.assign(size, 0);
  } else {
    std::fill(observed_.begin(), observed_.end(), 0);
  }
  ensureScratch(size);

  const double surface_band = (options.surfaceDistance > 0.0)
                                  ? options.surfaceDistance
                                  : 0.5 * voxel_size_;

  for (int z = 0; z < dims_.z(); ++z) {
    for (int y = 0; y < dims_.y(); ++y) {
      for (int x = 0; x < dims_.x(); ++x) {
        const Eigen::Vector3i index(x, y, z);
        const std::size_t linear = toLinear(index);
        const double weight = tsdf.getWeight(index);
        if (weight < options.minWeight || !tsdf.isObserved(index)) {
          continue;
        }
        observed_[linear] = 1;
        const double dist = tsdf.getDistance(index);
        signs_[linear] = (dist >= 0.0) ? 1 : -1;
        if (std::abs(dist) <= surface_band) {
          surface_[linear] = 1;
        }
      }
    }
  }

  const auto& surface_offsets = surfaceOffsets();
  for (int z = 0; z < dims_.z(); ++z) {
    for (int y = 0; y < dims_.y(); ++y) {
      for (int x = 0; x < dims_.x(); ++x) {
        const Eigen::Vector3i index(x, y, z);
        const std::size_t linear = toLinear(index);
        if (!observed_[linear] || surface_[linear]) {
          continue;
        }
        const int8_t sign = signs_[linear];
        for (const auto& neighbor : surface_offsets) {
          const Eigen::Vector3i neighbor_index = index + neighbor.offset;
          if (!isValidIndex(neighbor_index)) {
            continue;
          }
          const std::size_t neighbor_linear = toLinear(neighbor_index);
          if (!observed_[neighbor_linear]) {
            continue;
          }
          if (signs_[neighbor_linear] != sign) {
            surface_[linear] = 1;
            break;
          }
        }
      }
    }
  }

  struct QueueNode
  {
    double distance;
    std::size_t index;
  };
  auto cmp = [](const QueueNode& a, const QueueNode& b) {
    return a.distance > b.distance;
  };
  std::priority_queue<QueueNode, std::vector<QueueNode>, decltype(cmp)> queue(
      cmp);

  for (std::size_t i = 0; i < size; ++i) {
    if (observed_[i] && surface_[i]) {
      distances_[i] = 0.0;
      queue.push({0.0, i});
    }
  }

  const auto& neighbor_offsets = neighborOffsets(options.useDiagonalNeighbors);
  const int stride_x = dims_.x();
  const int stride_y = dims_.x() * dims_.y();

  auto fromLinear = [&](std::size_t linear) {
    const int z = static_cast<int>(linear / stride_y);
    const int rem
        = static_cast<int>(linear - static_cast<std::size_t>(z) * stride_y);
    const int y = rem / stride_x;
    const int x = rem - y * stride_x;
    return Eigen::Vector3i(x, y, z);
  };

  while (!queue.empty()) {
    const QueueNode node = queue.top();
    queue.pop();
    if (node.distance > distances_[node.index]) {
      continue;
    }
    if (node.distance > max_distance_) {
      continue;
    }

    const Eigen::Vector3i index = fromLinear(node.index);
    for (const auto& neighbor : neighbor_offsets) {
      const Eigen::Vector3i neighbor_index = index + neighbor.offset;
      if (!isValidIndex(neighbor_index)) {
        continue;
      }
      const std::size_t neighbor_linear = toLinear(neighbor_index);
      if (!observed_[neighbor_linear]) {
        continue;
      }
      const double new_distance = node.distance + neighbor.cost * voxel_size_;
      if (new_distance < distances_[neighbor_linear]
          && new_distance <= max_distance_) {
        distances_[neighbor_linear] = new_distance;
        queue.push({new_distance, neighbor_linear});
      }
    }
  }

  for (std::size_t i = 0; i < size; ++i) {
    if (!observed_[i]) {
      distances_[i] = max_distance_;
      continue;
    }
    distances_[i] = distances_[i] * static_cast<double>(signs_[i]);
  }

  return true;
}

bool DenseEsdfField::distance(
    const Eigen::Vector3d& point_F,
    double* distance,
    const SdfQueryOptions& options) const
{
  return sampleDistance(
      point_F,
      options.interpolate,
      options.requireObserved,
      options.maxDistance,
      distance);
}

bool DenseEsdfField::distanceAndGradient(
    const Eigen::Vector3d& point_F,
    double* distance,
    Eigen::Vector3d* gradient,
    const SdfQueryOptions& options) const
{
  if (!gradient) {
    return false;
  }

  double dist = 0.0;
  if (!sampleDistance(
          point_F,
          options.interpolate,
          options.requireObserved,
          options.maxDistance,
          &dist)) {
    return false;
  }

  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  const double step = voxel_size_;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    offset[axis] = step;

    double plus = 0.0;
    double minus = 0.0;
    if (!sampleDistance(
            point_F + offset,
            options.interpolate,
            options.requireObserved,
            options.maxDistance,
            &plus)) {
      return false;
    }
    if (!sampleDistance(
            point_F - offset,
            options.interpolate,
            options.requireObserved,
            options.maxDistance,
            &minus)) {
      return false;
    }
    grad[axis] = (plus - minus) / (2.0 * step);
  }

  if (distance) {
    *distance = dist;
  }
  *gradient = grad;
  return true;
}

void DenseEsdfField::batchDistanceAndGradient(
    std::span<const Eigen::Vector3d> points_F,
    std::span<double> distances,
    std::span<Eigen::Vector3d> gradients,
    std::span<std::uint8_t> observed,
    const SdfQueryOptions& options) const
{
  const std::size_t count = std::min(
      {points_F.size(), distances.size(), gradients.size(), observed.size()});
  for (std::size_t i = 0; i < count; ++i) {
    double dist = 0.0;
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    if (distanceAndGradient(points_F[i], &dist, &grad, options)) {
      distances[i] = dist;
      gradients[i] = grad;
      observed[i] = 1;
    } else {
      distances[i] = std::numeric_limits<double>::infinity();
      gradients[i] = Eigen::Vector3d::Zero();
      observed[i] = 0;
    }
  }
}

Aabb DenseEsdfField::localAabb() const
{
  const Eigen::Vector3d extents = dims_.cast<double>() * voxel_size_;
  return Aabb(origin_, origin_ + extents);
}

double DenseEsdfField::voxelSize() const
{
  return voxel_size_;
}

double DenseEsdfField::maxDistance() const
{
  return max_distance_;
}

bool DenseEsdfField::isValidIndex(const Eigen::Vector3i& index) const
{
  return (index.array() >= 0).all() && (index.array() < dims_.array()).all();
}

std::size_t DenseEsdfField::toLinear(const Eigen::Vector3i& index) const
{
  return static_cast<std::size_t>(index.x())
         + static_cast<std::size_t>(dims_.x())
               * (static_cast<std::size_t>(index.y())
                  + static_cast<std::size_t>(dims_.y())
                        * static_cast<std::size_t>(index.z()));
}

bool DenseEsdfField::sampleDistance(
    const Eigen::Vector3d& point_F,
    bool interpolate,
    bool require_observed,
    double max_distance,
    double* distance) const
{
  if (dims_.minCoeff() <= 0) {
    return false;
  }

  const Eigen::Vector3d local = (point_F - origin_) / voxel_size_;

  if (!interpolate) {
    Eigen::Vector3i index = (local.array() + 0.5).floor().cast<int>();
    if (!isValidIndex(index)) {
      return false;
    }
    if (require_observed && !observed_[toLinear(index)]) {
      return false;
    }
    const double dist = distances_[toLinear(index)];
    if (distance) {
      *distance = dist;
    }
    return dist <= max_distance;
  }

  const Eigen::Vector3d base = local.array().floor();
  const Eigen::Vector3i base_index = base.cast<int>();
  const Eigen::Vector3d frac = local - base;

  if ((base_index.array() < 0).any()) {
    return false;
  }
  if ((base_index.array() >= (dims_.array() - 1)).any()) {
    return false;
  }

  double samples[2][2][2];
  for (int dz = 0; dz <= 1; ++dz) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dx = 0; dx <= 1; ++dx) {
        const Eigen::Vector3i idx = base_index + Eigen::Vector3i(dx, dy, dz);
        if (require_observed && !observed_[toLinear(idx)]) {
          return false;
        }
        samples[dx][dy][dz] = distances_[toLinear(idx)];
      }
    }
  }

  const double wx = frac.x();
  const double wy = frac.y();
  const double wz = frac.z();

  const double c00 = samples[0][0][0] * (1.0 - wx) + samples[1][0][0] * wx;
  const double c10 = samples[0][1][0] * (1.0 - wx) + samples[1][1][0] * wx;
  const double c01 = samples[0][0][1] * (1.0 - wx) + samples[1][0][1] * wx;
  const double c11 = samples[0][1][1] * (1.0 - wx) + samples[1][1][1] * wx;
  const double c0 = c00 * (1.0 - wy) + c10 * wy;
  const double c1 = c01 * (1.0 - wy) + c11 * wy;
  const double dist = c0 * (1.0 - wz) + c1 * wz;

  if (distance) {
    *distance = dist;
  }
  return dist <= max_distance;
}

} // namespace dart::collision::experimental
