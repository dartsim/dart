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

#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <functional>

namespace dart::collision::experimental {

using SupportFunction =
    std::function<Eigen::Vector3d(const Eigen::Vector3d& direction)>;

struct DART_COLLISION_EXPERIMENTAL_API SupportPoint
{
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
};

struct DART_COLLISION_EXPERIMENTAL_API GjkSimplex
{
  std::array<SupportPoint, 4> points;
  int size = 0;

  void clear()
  {
    size = 0;
  }

  void push(const SupportPoint& point)
  {
    if (size < static_cast<int>(points.size())) {
      points[size] = point;
      ++size;
    }
  }

  void set(int index, const SupportPoint& point)
  {
    points[index] = point;
  }
};

struct DART_COLLISION_EXPERIMENTAL_API GjkResult
{
  bool intersecting = false;
  double distance = 0.0;
  Eigen::Vector3d closestPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d closestPointB = Eigen::Vector3d::Zero();
  Eigen::Vector3d separationAxis = Eigen::Vector3d::Zero();
  GjkSimplex simplex;
};

struct DART_COLLISION_EXPERIMENTAL_API EpaResult
{
  bool success = false;
  double depth = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointOnA = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointOnB = Eigen::Vector3d::Zero();
};

class DART_COLLISION_EXPERIMENTAL_API Gjk
{
public:
  static constexpr int kMaxIterations = 64;
  static constexpr double kTolerance = 1e-8;

  static GjkResult query(
      const SupportFunction& supportA,
      const SupportFunction& supportB,
      const Eigen::Vector3d& initialDirection = Eigen::Vector3d::UnitX());

  static bool intersect(
      const SupportFunction& supportA,
      const SupportFunction& supportB,
      const Eigen::Vector3d& initialDirection = Eigen::Vector3d::UnitX());
};

class DART_COLLISION_EXPERIMENTAL_API Epa
{
public:
  static constexpr int kMaxIterations = 64;
  static constexpr double kTolerance = 1e-6;

  static EpaResult penetration(
      const SupportFunction& supportA,
      const SupportFunction& supportB,
      const GjkSimplex& simplex);
};

}
