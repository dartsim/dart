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

#include <dart/collision/experimental/sdf/dense_sdf_field.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dart::collision::experimental {

DenseSdfField::DenseSdfField(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3i& dims,
    double voxel_size,
    double max_distance)
  : origin_(origin),
    dims_(dims),
    voxel_size_(voxel_size),
    max_distance_(max_distance)
{
  const std::size_t size =
      static_cast<std::size_t>(dims_.x())
      * static_cast<std::size_t>(dims_.y())
      * static_cast<std::size_t>(dims_.z());
  distances_.assign(size, max_distance_);
  observed_.assign(size, 0);
}

void DenseSdfField::setDistance(const Eigen::Vector3i& index, double distance)
{
  if (!isValidIndex(index)) {
    return;
  }
  distances_[toLinear(index)] = distance;
}

void DenseSdfField::setObserved(const Eigen::Vector3i& index, bool observed)
{
  if (!isValidIndex(index)) {
    return;
  }
  observed_[toLinear(index)] = observed ? 1 : 0;
}

const Eigen::Vector3d& DenseSdfField::origin() const
{
  return origin_;
}

const Eigen::Vector3i& DenseSdfField::dims() const
{
  return dims_;
}

bool DenseSdfField::distance(
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

bool DenseSdfField::distanceAndGradient(
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

void DenseSdfField::batchDistanceAndGradient(
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

Aabb DenseSdfField::localAabb() const
{
  const Eigen::Vector3d extents =
      dims_.cast<double>() * voxel_size_;
  return Aabb(origin_, origin_ + extents);
}

double DenseSdfField::voxelSize() const
{
  return voxel_size_;
}

double DenseSdfField::maxDistance() const
{
  return max_distance_;
}

bool DenseSdfField::isValidIndex(const Eigen::Vector3i& index) const
{
  return (index.array() >= 0).all()
      && (index.array() < dims_.array()).all();
}

std::size_t DenseSdfField::toLinear(const Eigen::Vector3i& index) const
{
  return static_cast<std::size_t>(index.x())
      + static_cast<std::size_t>(dims_.x())
          * (static_cast<std::size_t>(index.y())
             + static_cast<std::size_t>(dims_.y())
                 * static_cast<std::size_t>(index.z()));
}

bool DenseSdfField::sampleDistance(
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
    Eigen::Vector3i index =
        (local.array() + 0.5).floor().cast<int>();
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
        const Eigen::Vector3i idx =
            base_index + Eigen::Vector3i(dx, dy, dz);
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

  const double c00 =
      samples[0][0][0] * (1.0 - wx) + samples[1][0][0] * wx;
  const double c10 =
      samples[0][1][0] * (1.0 - wx) + samples[1][1][0] * wx;
  const double c01 =
      samples[0][0][1] * (1.0 - wx) + samples[1][0][1] * wx;
  const double c11 =
      samples[0][1][1] * (1.0 - wx) + samples[1][1][1] * wx;
  const double c0 = c00 * (1.0 - wy) + c10 * wy;
  const double c1 = c01 * (1.0 - wy) + c11 * wy;
  const double dist = c0 * (1.0 - wz) + c1 * wz;

  if (distance) {
    *distance = dist;
  }
  return dist <= max_distance;
}

}  // namespace dart::collision::experimental
