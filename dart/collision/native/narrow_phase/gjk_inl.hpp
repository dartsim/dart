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

// Templated, allocation-free GJK closest-point query for hot paths that hold
// their support functions as concrete callables (e.g. the continuous-collision
// casts). It is the same algorithm as the std::function-based Gjk::query but
// removes the per-support-evaluation indirect call, which dominates the inner
// loop of conservative advancement. The simplex-processing helpers are pure and
// remain defined once in gjk.cpp; only their declarations are needed here.

#include <dart/collision/native/narrow_phase/gjk.hpp>

#include <array>
#include <limits>

#include <cmath>

namespace dart::collision::native::detail {

// Defined in gjk.cpp (shared with the std::function query path).
bool isUsableDirection(const Eigen::Vector3d& direction);
bool reduceSimplex(
    GjkSimplex& simplex,
    Eigen::Vector3d& closest,
    std::array<double, 4>& weights);
void fillSeparationResult(
    const GjkSimplex& simplex,
    const std::array<double, 4>& weights,
    const Eigen::Vector3d& closest,
    GjkResult& result);

template <typename SupportA, typename SupportB>
SupportPoint computeSupportT(
    const SupportA& supportA,
    const SupportB& supportB,
    const Eigen::Vector3d& direction)
{
  constexpr double kEpsilon = 1e-12;
  Eigen::Vector3d dir = direction;
  if (!dir.allFinite() || dir.squaredNorm() < kEpsilon) {
    dir = Eigen::Vector3d::UnitX();
  } else {
    dir.normalize();
  }

  SupportPoint point;
  point.v1 = supportA(dir);
  point.v2 = supportB(-dir);
  point.v = point.v1 - point.v2;
  point.direction = dir;
  return point;
}

template <typename SupportA, typename SupportB>
GjkResult queryFromSimplexT(
    const SupportA& supportA,
    const SupportB& supportB,
    GjkSimplex simplex,
    const Eigen::Vector3d& fallbackInitialDirection)
{
  GjkResult result;
  std::array<double, 4> weights{};

  Eigen::Vector3d direction = fallbackInitialDirection;
  if (!isUsableDirection(direction)) {
    direction = Eigen::Vector3d::UnitX();
  }

  if (simplex.size == 0) {
    simplex.push(computeSupportT(supportA, supportB, direction));
  }

  Eigen::Vector3d closest = Eigen::Vector3d::Zero();
  if (reduceSimplex(simplex, closest, weights)) {
    result.intersecting = true;
    result.simplex = simplex;
    return result;
  }

  double prevDist2 = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < Gjk::kMaxIterations; ++iter) {
    const double dist2 = closest.squaredNorm();
    if (dist2 <= Gjk::kTolerance * Gjk::kTolerance) {
      result.intersecting = true;
      result.simplex = simplex;
      return result;
    }

    direction = -closest;
    if (!isUsableDirection(direction)) {
      direction = Eigen::Vector3d::UnitX();
    }
    SupportPoint newPoint = computeSupportT(supportA, supportB, direction);

    const double delta = newPoint.v.dot(direction) - closest.dot(direction);
    if (delta <= Gjk::kTolerance) {
      fillSeparationResult(simplex, weights, closest, result);
      result.simplex = simplex;
      return result;
    }

    simplex.push(newPoint);

    if (reduceSimplex(simplex, closest, weights)) {
      result.intersecting = true;
      result.simplex = simplex;
      return result;
    }

    const double newDist2 = closest.squaredNorm();
    if (std::abs(prevDist2 - newDist2) <= Gjk::kTolerance * Gjk::kTolerance) {
      fillSeparationResult(simplex, weights, closest, result);
      result.simplex = simplex;
      return result;
    }
    prevDist2 = newDist2;
  }

  fillSeparationResult(simplex, weights, closest, result);
  result.simplex = simplex;
  return result;
}

template <typename SupportA, typename SupportB>
GjkSimplex buildWarmStartSimplexT(
    const SupportA& supportA,
    const SupportB& supportB,
    const GjkSimplex& initialSimplex)
{
  GjkSimplex simplex;
  for (int i = 0; i < initialSimplex.size; ++i) {
    Eigen::Vector3d direction = initialSimplex.points[i].direction;
    if (!isUsableDirection(direction)) {
      direction = initialSimplex.points[i].v;
    }
    if (isUsableDirection(direction)) {
      simplex.push(computeSupportT(supportA, supportB, direction));
    }
  }
  return simplex;
}

/// Templated equivalent of Gjk::query (direction seed).
template <typename SupportA, typename SupportB>
GjkResult queryT(
    const SupportA& supportA,
    const SupportB& supportB,
    const Eigen::Vector3d& initialDirection)
{
  GjkSimplex simplex;
  Eigen::Vector3d direction = initialDirection;
  if (!isUsableDirection(direction)) {
    direction = Eigen::Vector3d::UnitX();
  }
  simplex.push(computeSupportT(supportA, supportB, direction));
  return queryFromSimplexT(supportA, supportB, simplex, direction);
}

/// Templated equivalent of Gjk::query (warm-start from a previous simplex).
template <typename SupportA, typename SupportB>
GjkResult queryT(
    const SupportA& supportA,
    const SupportB& supportB,
    const GjkSimplex& initialSimplex,
    const Eigen::Vector3d& fallbackInitialDirection)
{
  GjkSimplex simplex
      = buildWarmStartSimplexT(supportA, supportB, initialSimplex);
  return queryFromSimplexT(
      supportA, supportB, simplex, fallbackInitialDirection);
}

} // namespace dart::collision::native::detail
