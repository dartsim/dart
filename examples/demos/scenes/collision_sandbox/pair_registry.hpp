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

#include <dart/collision/native/shapes/shape.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <span>
#include <string>
#include <string_view>

#include <cstddef>

namespace dart::examples::collision_sandbox {

namespace collision = dart::collision::native;

enum class PairStatus
{
  Contact,
  AdaptedFallback,
  DistanceOnly,
  Unsupported
};

struct ShapeFixture
{
  collision::ShapeType type;
  std::string_view id;
  std::string_view label;
};

struct PairCase
{
  std::size_t index = 0;
  collision::ShapeType shapeA = collision::ShapeType::Sphere;
  collision::ShapeType shapeB = collision::ShapeType::Sphere;
  PairStatus status = PairStatus::Unsupported;
  std::string id;
  std::string label;
  std::string note;

  [[nodiscard]] bool supportsContact() const
  {
    return status == PairStatus::Contact
           || status == PairStatus::AdaptedFallback;
  }

  [[nodiscard]] bool supportsDistance() const
  {
    return supportsContact() || status == PairStatus::DistanceOnly;
  }
};

struct PairPose
{
  Eigen::Isometry3d transformA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d transformB = Eigen::Isometry3d::Identity();
};

struct ShapeParameters
{
  double radius = 0.45;
  double height = 0.9;
  Eigen::Vector3d halfExtents = Eigen::Vector3d(0.45, 0.35, 0.3);
};

[[nodiscard]] std::span<const ShapeFixture> shapeFixtures();
[[nodiscard]] std::span<const PairCase> pairCases();
[[nodiscard]] const PairCase& pairCaseAt(std::size_t index);
[[nodiscard]] const PairCase* findPairCase(std::string_view id);

[[nodiscard]] std::string_view shapeTypeId(collision::ShapeType type);
[[nodiscard]] std::string_view shapeTypeLabel(collision::ShapeType type);
[[nodiscard]] std::string_view pairStatusLabel(PairStatus status);
[[nodiscard]] std::string_view pairStatusDescription(PairStatus status);
[[nodiscard]] bool isAdaptedFallbackPair(
    collision::ShapeType shapeA, collision::ShapeType shapeB);

[[nodiscard]] ShapeParameters defaultShapeParameters(collision::ShapeType type);
[[nodiscard]] std::unique_ptr<collision::Shape> makeShape(
    collision::ShapeType type, double scale = 1.0);
[[nodiscard]] std::unique_ptr<collision::Shape> makeShape(
    collision::ShapeType type, const ShapeParameters& params);
[[nodiscard]] PairPose defaultPairPose(const PairCase& pair);

} // namespace dart::examples::collision_sandbox
