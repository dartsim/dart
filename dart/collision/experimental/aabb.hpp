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

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dart::collision::experimental {

class DART_COLLISION_EXPERIMENTAL_API Aabb
{
public:
  Aabb();
  Aabb(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  [[nodiscard]] bool overlaps(const Aabb& other) const;
  [[nodiscard]] bool contains(const Eigen::Vector3d& point) const;
  [[nodiscard]] bool contains(const Aabb& other) const;

  [[nodiscard]] Eigen::Vector3d center() const;
  [[nodiscard]] Eigen::Vector3d halfExtents() const;
  [[nodiscard]] Eigen::Vector3d extents() const;
  [[nodiscard]] double volume() const;

  void merge(const Aabb& other);
  void expand(double margin);

  [[nodiscard]] static Aabb forSphere(double radius);
  [[nodiscard]] static Aabb forBox(const Eigen::Vector3d& halfExtents);
  [[nodiscard]] static Aabb forCapsule(double radius, double height);
  [[nodiscard]] static Aabb forCylinder(double radius, double height);
  [[nodiscard]] static Aabb transformed(
      const Aabb& local, const Eigen::Isometry3d& transform);

  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

} // namespace dart::collision::experimental
