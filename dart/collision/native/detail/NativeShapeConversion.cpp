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

#include "dart/collision/native/detail/NativeShapeConversion.hpp"

#include "dart/collision/native/shapes/shape.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/ConvexMeshShape.hpp"
#include "dart/dynamics/PyramidShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <set>
#include <string>
#include <vector>

#include <cmath>

namespace dart {
namespace collision {
namespace detail {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::unique_ptr<native::Shape> createConvexOrNull(
    std::vector<Eigen::Vector3d> vertices, const std::string& shapeType)
{
  if (vertices.empty()) {
    static std::set<std::string> warnedInvalidShapeTypes;
    if (warnedInvalidShapeTypes.insert(shapeType).second) {
      dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
             << "] did not provide convex vertices. This shape will be "
             << "skipped by the native adapter.\n";
    }
    return nullptr;
  }

  return std::make_unique<native::ConvexShape>(std::move(vertices));
}

std::vector<Eigen::Vector3d> makeConeVertices(double radius, double height)
{
  std::vector<Eigen::Vector3d> vertices;
  constexpr std::size_t kNumBaseVertices = 16;
  vertices.reserve(kNumBaseVertices + 1);

  const double baseZ = -0.5 * height;
  for (std::size_t i = 0; i < kNumBaseVertices; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i)
                         / static_cast<double>(kNumBaseVertices);
    vertices.emplace_back(
        radius * std::cos(theta), radius * std::sin(theta), baseZ);
  }

  vertices.emplace_back(0.0, 0.0, 0.5 * height);
  return vertices;
}

std::vector<Eigen::Vector3d> makePyramidVertices(
    double baseWidth, double baseDepth, double height)
{
  const double halfWidth = 0.5 * baseWidth;
  const double halfDepth = 0.5 * baseDepth;
  const double baseZ = -0.5 * height;
  return {
      {-halfWidth, -halfDepth, baseZ},
      {halfWidth, -halfDepth, baseZ},
      {halfWidth, halfDepth, baseZ},
      {-halfWidth, halfDepth, baseZ},
      {0.0, 0.0, 0.5 * height}};
}

} // namespace

//==============================================================================
std::unique_ptr<native::Shape> NativeShapeConversion::create(
    const dynamics::Shape& shape)
{
  const auto& shapeType = shape.getType();

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    const auto& sphere = static_cast<const dynamics::SphereShape&>(shape);
    return std::make_unique<native::SphereShape>(sphere.getRadius());
  }

  if (shapeType == dynamics::BoxShape::getStaticType()) {
    const auto& box = static_cast<const dynamics::BoxShape&>(shape);
    return std::make_unique<native::BoxShape>(0.5 * box.getSize());
  }

  if (shapeType == dynamics::CapsuleShape::getStaticType()) {
    const auto& capsule = static_cast<const dynamics::CapsuleShape&>(shape);
    return std::make_unique<native::CapsuleShape>(
        capsule.getRadius(), capsule.getHeight());
  }

  if (shapeType == dynamics::ConvexMeshShape::getStaticType()) {
    const auto& convex = static_cast<const dynamics::ConvexMeshShape&>(shape);
    const auto& mesh = convex.getMesh();
    if (!mesh) {
      return createConvexOrNull({}, shapeType);
    }
    return createConvexOrNull(mesh->getVertices(), shapeType);
  }

  if (shapeType == dynamics::ConeShape::getStaticType()) {
    const auto& cone = static_cast<const dynamics::ConeShape&>(shape);
    return createConvexOrNull(
        makeConeVertices(cone.getRadius(), cone.getHeight()), shapeType);
  }

  if (shapeType == dynamics::PyramidShape::getStaticType()) {
    const auto& pyramid = static_cast<const dynamics::PyramidShape&>(shape);
    return createConvexOrNull(
        makePyramidVertices(
            pyramid.getBaseWidth(),
            pyramid.getBaseDepth(),
            pyramid.getHeight()),
        shapeType);
  }

  static std::set<std::string> warnedShapeTypes;
  if (warnedShapeTypes.insert(shapeType).second) {
    dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
           << "] is not supported by NativeCollisionDetector yet. This "
           << "shape will be skipped by the native adapter.\n";
  }

  return nullptr;
}

} // namespace detail
} // namespace collision
} // namespace dart
