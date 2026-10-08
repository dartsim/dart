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

#include <dart/collision/dart/narrow_phase/ConvexConvex-impl.hpp>

#include <limits>
#include <stdexcept>
#include <vector>

#include <cmath>

namespace dart::collision::native {

SupportFunction makeConvexSupportFunction(
    const ConvexShape& shape, const Eigen::Isometry3d& transform)
{
  return detail::makeConvexSupportFunctionT(shape, transform);
}

SupportFunction makeMeshSupportFunction(
    const MeshShape& shape, const Eigen::Isometry3d& transform)
{
  return detail::makeMeshSupportFunctionT(shape, transform);
}

SupportFunction makeSphereSupportFunction(
    const SphereShape& shape, const Eigen::Isometry3d& transform)
{
  return detail::makeSphereSupportFunctionT(shape, transform);
}

SupportFunction makeBoxSupportFunction(
    const BoxShape& shape, const Eigen::Isometry3d& transform)
{
  return detail::makeBoxSupportFunctionT(shape, transform);
}

SupportFunction makeCapsuleSupportFunction(
    const CapsuleShape& shape, const Eigen::Isometry3d& transform)
{
  return detail::makeCapsuleSupportFunctionT(shape, transform);
}

SupportFunction makeCylinderSupportFunction(
    const CylinderShape& shape, const Eigen::Isometry3d& transform)
{
  return detail::makeCylinderSupportFunctionT(shape, transform);
}

namespace {

Eigen::Vector3d averageVertexPosition(
    const std::vector<Eigen::Vector3d>& vertices)
{
  if (vertices.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& v : vertices) {
    sum += v;
  }
  return sum / static_cast<double>(vertices.size());
}

Eigen::Vector3d computeShapeCenter(
    const Shape& shape, const Eigen::Isometry3d& transform)
{
  switch (shape.getType()) {
    case ShapeType::Sphere:
    case ShapeType::Box:
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
      return transform.translation();
    case ShapeType::Convex: {
      const auto& convex = static_cast<const ConvexShape&>(shape);
      return transform * averageVertexPosition(convex.getVertices());
    }
    case ShapeType::Mesh: {
      const auto& mesh = static_cast<const MeshShape&>(shape);
      return transform * averageVertexPosition(mesh.getVertices());
    }
    default:
      return transform.translation();
  }
}

void alignPenetrationWitnesses(
    double depth,
    const Eigen::Vector3d& penetrationNormal,
    Eigen::Vector3d& pointA,
    Eigen::Vector3d& pointB)
{
  if (!(depth > 0.0) || !std::isfinite(depth)) {
    return;
  }

  const double witnessDistance = (pointB - pointA).norm();
  if (std::abs(witnessDistance - depth) <= 1e-6) {
    return;
  }

  Eigen::Vector3d normal = penetrationNormal;
  if (!normal.allFinite() || normal.squaredNorm() < 1e-12) {
    normal = pointA - pointB;
  }
  if (!normal.allFinite() || normal.squaredNorm() < 1e-12) {
    return;
  }

  normal.normalize();
  const Eigen::Vector3d midpoint = 0.5 * (pointA + pointB);
  pointA = midpoint + 0.5 * depth * normal;
  pointB = midpoint - 0.5 * depth * normal;
}

} // namespace

bool collideSupportFunctions(
    const SupportFunction& supportA,
    const Eigen::Vector3d& centerA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& centerB,
    CollisionResult& result,
    const CollisionOption& option)
{
  return detail::collideSupportFunctionsT(
      supportA, centerA, supportB, centerB, result, option);
}

bool collideConvexConvex(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    CollisionResult& result,
    const CollisionOption& option)
{
  const detail::ShapeSupport supportA{shape1, tf1};
  const detail::ShapeSupport supportB{shape2, tf2};
  const Eigen::Vector3d centerA = computeShapeCenter(shape1, tf1);
  const Eigen::Vector3d centerB = computeShapeCenter(shape2, tf2);
  return detail::collideSupportFunctionsT(
      supportA, centerA, supportB, centerB, result, option);
}

void collideConvexConvexBatch(
    span<const ConvexPair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "collideConvexConvexBatch requires one result for each pair");
  }

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument(
          "collideConvexConvexBatch received a null shape");
    }

    [[maybe_unused]] const bool collided = collideConvexConvex(
        *pair.shapeA, pair.tfA, *pair.shapeB, pair.tfB, results[i], option);
  }
}

double distanceConvexConvex(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    DistanceResult& result,
    const DistanceOption& option)
{
  const detail::ShapeSupport supportA{shape1, tf1};
  const detail::ShapeSupport supportB{shape2, tf2};

  Eigen::Vector3d initialDir = tf2.translation() - tf1.translation();
  if (initialDir.squaredNorm() < 1e-10) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  GjkResult gjkResult = detail::queryT(supportA, supportB, initialDir);

  if (!gjkResult.intersecting) {
    if (option.upperBound < gjkResult.distance) {
      result.distance = std::numeric_limits<double>::max();
      return std::numeric_limits<double>::max();
    }

    result.distance = gjkResult.distance;
    if (option.enableNearestPoints) {
      result.pointOnObject1 = gjkResult.closestPointA;
      result.pointOnObject2 = gjkResult.closestPointB;
      if (gjkResult.distance > 1e-12) {
        result.normal
            = (gjkResult.closestPointB - gjkResult.closestPointA).normalized();
      } else {
        result.normal = Eigen::Vector3d::UnitX();
      }
    }
    return gjkResult.distance;
  }

  EpaResult epaResult
      = detail::penetrationT(supportA, supportB, gjkResult.simplex);
  double depth = 0.0;
  Eigen::Vector3d pointA = tf1.translation();
  Eigen::Vector3d pointB = tf2.translation();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
  Eigen::Vector3d penetrationNormal = Eigen::Vector3d::UnitX();

  if (epaResult.success) {
    depth = epaResult.depth;
    pointA = epaResult.pointOnA;
    pointB = epaResult.pointOnB;
    penetrationNormal = epaResult.normal;
  } else {
    const Eigen::Vector3d centerA = computeShapeCenter(shape1, tf1);
    const Eigen::Vector3d centerB = computeShapeCenter(shape2, tf2);
    MprResult mprResult
        = detail::mpr::penetrationT(supportA, supportB, centerA, centerB);
    if (mprResult.success) {
      depth = mprResult.depth;
      pointA = mprResult.pointOnA;
      pointB = mprResult.pointOnB;
      penetrationNormal = mprResult.normal;
    }
  }

  alignPenetrationWitnesses(depth, penetrationNormal, pointA, pointB);
  if ((pointB - pointA).squaredNorm() > 1e-12) {
    normal = (pointB - pointA).normalized();
  } else if (penetrationNormal.squaredNorm() > 1e-12) {
    normal = -penetrationNormal.normalized();
  }

  result.distance = -depth;
  if (option.enableNearestPoints) {
    result.pointOnObject1 = pointA;
    result.pointOnObject2 = pointB;
    result.normal = normal;
  }

  return -depth;
}

} // namespace dart::collision::native
