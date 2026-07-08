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

#include <dart/collision/native/detail/span.hpp>
#include <dart/collision/native/narrow_phase/box_box/sat.hpp>
#include <dart/collision/native/narrow_phase/distance.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <algorithm>
#include <array>
#include <limits>

#include <cmath>

namespace dart::collision::native {

namespace {

struct SegmentClosestResult
{
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
  double distSq;
};

struct Segment
{
  Eigen::Vector3d start;
  Eigen::Vector3d end;
};

struct SdfSdfCandidate
{
  double distance = std::numeric_limits<double>::max();
  Eigen::Vector3d pointOnSource = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointOnTarget = Eigen::Vector3d::Zero();
  Eigen::Vector3d normalSourceToTarget = Eigen::Vector3d::UnitX();
};

SegmentClosestResult closestPointsBetweenSegments(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;

  const double a = d1.squaredNorm();
  const double e = d2.squaredNorm();
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  constexpr double eps = 1e-10;

  if (a <= eps && e <= eps) {
    s = t = 0.0;
  } else if (a <= eps) {
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= eps) {
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;

      if (denom != 0.0) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0.0;
      }

      t = (b * s + f) / e;

      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  SegmentClosestResult res;
  res.point1 = p1 + d1 * s;
  res.point2 = p2 + d2 * t;
  res.distSq = (res.point1 - res.point2).squaredNorm();
  return res;
}

Eigen::Vector3d closestPointOnBox(
    const Eigen::Vector3d& point, const Eigen::Vector3d& halfExtents)
{
  return Eigen::Vector3d(
      std::clamp(point.x(), -halfExtents.x(), halfExtents.x()),
      std::clamp(point.y(), -halfExtents.y(), halfExtents.y()),
      std::clamp(point.z(), -halfExtents.z(), halfExtents.z()));
}

Eigen::Vector3d closestPointOnSegmentInBoxSpace(
    const Eigen::Vector3d& segmentStart,
    const Eigen::Vector3d& segmentEnd,
    const Eigen::Vector3d& halfExtents,
    Eigen::Vector3d& closestOnSegment)
{
  const Eigen::Vector3d segment = segmentEnd - segmentStart;
  const double segmentLengthSq = segment.squaredNorm();

  if (segmentLengthSq < 1e-10) {
    closestOnSegment = segmentStart;
    return closestPointOnBox(segmentStart, halfExtents);
  }

  double bestDistSq = std::numeric_limits<double>::max();
  double bestInteriorMargin = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d bestOnBox = Eigen::Vector3d::Zero();
  Eigen::Vector3d bestOnSegment = segmentStart;

  auto computeInteriorMargin = [&](const Eigen::Vector3d& point) {
    double margin = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
      const double axisMargin = halfExtents[axis] - std::abs(point[axis]);
      if (axisMargin < 0.0) {
        return -std::numeric_limits<double>::infinity();
      }
      margin = std::min(margin, axisMargin);
    }
    return margin;
  };

  auto testCandidate = [&](double t) {
    const Eigen::Vector3d pointOnSegment = segmentStart + segment * t;
    const Eigen::Vector3d pointOnBox
        = closestPointOnBox(pointOnSegment, halfExtents);
    const double distSq = (pointOnSegment - pointOnBox).squaredNorm();
    const double interiorMargin = computeInteriorMargin(pointOnSegment);

    if (distSq < bestDistSq - 1e-18
        || (std::abs(distSq - bestDistSq) <= 1e-18
            && interiorMargin > bestInteriorMargin)) {
      bestDistSq = distSq;
      bestInteriorMargin = interiorMargin;
      bestOnBox = pointOnBox;
      bestOnSegment = pointOnSegment;
    }
  };

  std::array<double, 8> breakpoints{};
  std::size_t numBreakpoints = 0;
  breakpoints[numBreakpoints++] = 0.0;
  breakpoints[numBreakpoints++] = 1.0;

  constexpr double kAxisEps = 1e-12;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(segment[axis]) <= kAxisEps) {
      continue;
    }

    for (const double face : {-halfExtents[axis], halfExtents[axis]}) {
      const double t = (face - segmentStart[axis]) / segment[axis];
      if (t > 0.0 && t < 1.0) {
        breakpoints[numBreakpoints++] = t;
      }
    }
  }

  std::sort(breakpoints.begin(), breakpoints.begin() + numBreakpoints);

  std::array<double, 8> uniqueBreakpoints{};
  std::size_t numUniqueBreakpoints = 0;
  for (std::size_t i = 0; i < numBreakpoints; ++i) {
    if (numUniqueBreakpoints == 0
        || std::abs(
               breakpoints[i] - uniqueBreakpoints[numUniqueBreakpoints - 1])
               > kAxisEps) {
      uniqueBreakpoints[numUniqueBreakpoints++] = breakpoints[i];
      testCandidate(breakpoints[i]);
    }
  }

  for (std::size_t i = 0; i + 1 < numUniqueBreakpoints; ++i) {
    const double lo = uniqueBreakpoints[i];
    const double hi = uniqueBreakpoints[i + 1];
    if (hi - lo <= kAxisEps) {
      continue;
    }

    const double mid = 0.5 * (lo + hi);
    double numerator = 0.0;
    double denominator = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double coord = segmentStart[axis] + segment[axis] * mid;
      double face = 0.0;
      if (coord < -halfExtents[axis]) {
        face = -halfExtents[axis];
      } else if (coord > halfExtents[axis]) {
        face = halfExtents[axis];
      } else {
        continue;
      }

      numerator += segment[axis] * (face - segmentStart[axis]);
      denominator += segment[axis] * segment[axis];
    }

    if (denominator > kAxisEps) {
      testCandidate(std::clamp(numerator / denominator, lo, hi));
    } else {
      testCandidate(mid);
    }
  }

  closestOnSegment = bestOnSegment;
  return bestOnBox;
}

std::array<Segment, 12> boxEdges(const Eigen::Vector3d& halfExtents)
{
  std::array<Segment, 12> edges;
  int edgeIndex = 0;

  for (int axis = 0; axis < 3; ++axis) {
    const int axis1 = (axis + 1) % 3;
    const int axis2 = (axis + 2) % 3;

    for (const double sign1 : {-1.0, 1.0}) {
      for (const double sign2 : {-1.0, 1.0}) {
        Eigen::Vector3d start = Eigen::Vector3d::Zero();
        Eigen::Vector3d end = Eigen::Vector3d::Zero();
        start[axis] = -halfExtents[axis];
        end[axis] = halfExtents[axis];
        start[axis1] = end[axis1] = sign1 * halfExtents[axis1];
        start[axis2] = end[axis2] = sign2 * halfExtents[axis2];
        edges[edgeIndex++] = {start, end};
      }
    }
  }

  return edges;
}

[[nodiscard]] double computeBoxBoundaryTolerance(
    const Eigen::Vector3d& halfExtents, const Eigen::Vector3d& point)
{
  const double boundaryScale = std::max(
      {1.0, halfExtents.cwiseAbs().maxCoeff(), point.cwiseAbs().maxCoeff()});
  return 64.0 * std::numeric_limits<double>::epsilon() * boundaryScale;
}

void snapNearBoxBoundary(
    Eigen::Vector3d& point,
    const Eigen::Vector3d& halfExtents,
    double tolerance)
{
  for (int axis = 0; axis < 3; ++axis) {
    const double extent = halfExtents[axis];
    const double coordinate = point[axis];
    if (std::abs(std::abs(coordinate) - extent) <= tolerance) {
      point[axis] = std::copysign(extent, coordinate);
    }
  }
}

bool distanceSameOrientationBoxes(
    const Eigen::Vector3d& half1,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& half2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option)
{
  const Eigen::Matrix3d relRot
      = transform1.linear().transpose() * transform2.linear();
  if (!relRot.isApprox(Eigen::Matrix3d::Identity(), 1e-12)) {
    return false;
  }

  const Eigen::Vector3d center2
      = transform1.linear().transpose()
        * (transform2.translation() - transform1.translation());
  const Eigen::Vector3d combinedHalf = half1 + half2;
  const Eigen::Vector3d axisSeparations = center2.cwiseAbs() - combinedHalf;
  const bool separated = (axisSeparations.array() > 0.0).any();
  const double dist = separated ? axisSeparations.cwiseMax(0.0).norm()
                                : axisSeparations.maxCoeff();

  result.distance = dist;
  if (dist > option.upperBound) {
    return true;
  }

  if (!option.enableNearestPoints) {
    return true;
  }

  Eigen::Vector3d point1Local = Eigen::Vector3d::Zero();
  Eigen::Vector3d point2Local = Eigen::Vector3d::Zero();

  Eigen::Index penetrationAxis = 0;
  if (!separated) {
    axisSeparations.maxCoeff(&penetrationAxis);
  }

  for (int axis = 0; axis < 3; ++axis) {
    if (!separated && axis == penetrationAxis) {
      const double direction = (center2[axis] >= 0.0) ? 1.0 : -1.0;
      point1Local[axis] = direction * half1[axis];
      point2Local[axis] = -direction * half2[axis];
    } else if (center2[axis] > combinedHalf[axis]) {
      point1Local[axis] = half1[axis];
      point2Local[axis] = -half2[axis];
    } else if (center2[axis] < -combinedHalf[axis]) {
      point1Local[axis] = -half1[axis];
      point2Local[axis] = half2[axis];
    } else {
      const double overlapMin
          = std::max(-half1[axis], center2[axis] - half2[axis]);
      const double overlapMax
          = std::min(half1[axis], center2[axis] + half2[axis]);
      point1Local[axis] = 0.5 * (overlapMin + overlapMax);
      point2Local[axis] = point1Local[axis] - center2[axis];
    }
  }

  result.pointOnObject1 = transform1 * point1Local;
  result.pointOnObject2 = transform2 * point2Local;

  if (!separated) {
    const double direction = (center2[penetrationAxis] >= 0.0) ? 1.0 : -1.0;
    result.normal = direction * transform1.linear().col(penetrationAxis);
  } else {
    const Eigen::Vector3d diff = result.pointOnObject2 - result.pointOnObject1;
    if (diff.squaredNorm() > 1e-10) {
      result.normal = diff.normalized();
    } else {
      result.normal = Eigen::Vector3d::UnitX();
    }
  }

  return true;
}

constexpr double kSupportEps = 1e-12;
constexpr double kPi = 3.141592653589793238462643383279502884;

bool supportPointOnShape(
    const Shape& shape,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& directionWorld,
    Eigen::Vector3d& supportWorld)
{
  Eigen::Vector3d dir = directionWorld;
  if (dir.squaredNorm() < kSupportEps) {
    dir = Eigen::Vector3d::UnitX();
  }

  switch (shape.getType()) {
    case ShapeType::Sphere: {
      const auto& sphere = static_cast<const SphereShape&>(shape);
      supportWorld
          = transform.translation() + sphere.getRadius() * dir.normalized();
      return true;
    }
    case ShapeType::Box: {
      const auto& box = static_cast<const BoxShape&>(shape);
      const Eigen::Vector3d localDir = transform.linear().transpose() * dir;
      Eigen::Vector3d localSupport;
      const Eigen::Vector3d& halfExtents = box.getHalfExtents();
      localSupport.x()
          = (localDir.x() >= 0.0) ? halfExtents.x() : -halfExtents.x();
      localSupport.y()
          = (localDir.y() >= 0.0) ? halfExtents.y() : -halfExtents.y();
      localSupport.z()
          = (localDir.z() >= 0.0) ? halfExtents.z() : -halfExtents.z();
      supportWorld = transform * localSupport;
      return true;
    }
    case ShapeType::Capsule: {
      const auto& capsule = static_cast<const CapsuleShape&>(shape);
      const double radius = capsule.getRadius();
      const double halfHeight = capsule.getHeight() * 0.5;
      Eigen::Vector3d localDir = transform.linear().transpose() * dir;
      if (localDir.squaredNorm() < kSupportEps) {
        localDir = Eigen::Vector3d::UnitX();
      }
      const Eigen::Vector3d dirNorm = localDir.normalized();
      const Eigen::Vector3d axisPoint
          = (dirNorm.z() >= 0.0) ? Eigen::Vector3d(0, 0, halfHeight)
                                 : Eigen::Vector3d(0, 0, -halfHeight);
      supportWorld = transform * (axisPoint + radius * dirNorm);
      return true;
    }
    case ShapeType::Cylinder: {
      const auto& cylinder = static_cast<const CylinderShape&>(shape);
      const double radius = cylinder.getRadius();
      const double halfHeight = cylinder.getHeight() * 0.5;
      const Eigen::Vector3d localDir = transform.linear().transpose() * dir;
      const double xyLen = std::sqrt(
          localDir.x() * localDir.x() + localDir.y() * localDir.y());
      Eigen::Vector3d localSupport;
      if (xyLen < kSupportEps) {
        localSupport.x() = radius;
        localSupport.y() = 0.0;
      } else {
        localSupport.x() = radius * localDir.x() / xyLen;
        localSupport.y() = radius * localDir.y() / xyLen;
      }
      localSupport.z() = (localDir.z() >= 0.0) ? halfHeight : -halfHeight;
      supportWorld = transform * localSupport;
      return true;
    }
    case ShapeType::Convex: {
      const auto& convex = static_cast<const ConvexShape&>(shape);
      Eigen::Vector3d localDir = transform.linear().transpose() * dir;
      if (localDir.squaredNorm() < kSupportEps) {
        localDir = Eigen::Vector3d::UnitX();
      }
      supportWorld = transform * convex.support(localDir);
      return true;
    }
    case ShapeType::Mesh: {
      const auto& mesh = static_cast<const MeshShape&>(shape);
      Eigen::Vector3d localDir = transform.linear().transpose() * dir;
      if (localDir.squaredNorm() < kSupportEps) {
        localDir = Eigen::Vector3d::UnitX();
      }
      supportWorld = transform * mesh.support(localDir);
      return true;
    }
    default:
      return false;
  }
}

bool querySdfDistanceAndGradient(
    const SignedDistanceField* field,
    const Eigen::Isometry3d& sdfInverse,
    const Eigen::Matrix3d& sdfRotation,
    const Eigen::Vector3d& pointWorld,
    const SdfQueryOptions& options,
    double* distance,
    Eigen::Vector3d* gradientWorld)
{
  if (!field) {
    return false;
  }

  Eigen::Vector3d gradientField = Eigen::Vector3d::Zero();
  const Eigen::Vector3d pointField = sdfInverse * pointWorld;
  if (!field->distanceAndGradient(
          pointField, distance, &gradientField, options)) {
    return false;
  }

  *gradientWorld = sdfRotation * gradientField;
  return true;
}

double boundedSdfQueryDistance(double upperBound, double margin)
{
  if (!std::isfinite(upperBound)) {
    return std::numeric_limits<double>::max();
  }
  if (upperBound > std::numeric_limits<double>::max() - margin) {
    return std::numeric_limits<double>::max();
  }
  return upperBound + margin;
}

int sdfAxisSampleCount(double extent, double voxelSize)
{
  if (!std::isfinite(extent) || extent <= 0.0) {
    return 2;
  }

  return std::clamp(static_cast<int>(std::ceil(extent / voxelSize)) + 1, 2, 64);
}

bool isValidAabb(const Aabb& aabb)
{
  return aabb.min.allFinite() && aabb.max.allFinite()
         && (aabb.max.array() >= aabb.min.array()).all();
}

bool sampleSdfSurfaceAgainstSdf(
    const SignedDistanceField* sourceField,
    const Eigen::Isometry3d& sourceTransform,
    const SignedDistanceField* targetField,
    const Eigen::Isometry3d& targetTransform,
    double shellThickness,
    const DistanceOption& option,
    SdfSdfCandidate& best)
{
  if (!sourceField || !targetField) {
    return false;
  }

  const Aabb sourceAabb = sourceField->localAabb();
  if (!isValidAabb(sourceAabb)) {
    return false;
  }

  const double sourceVoxelSize = std::max(sourceField->voxelSize(), 1e-6);
  const Eigen::Vector3d extents = sourceAabb.extents();
  const Eigen::Vector3i samples(
      sdfAxisSampleCount(extents.x(), sourceVoxelSize),
      sdfAxisSampleCount(extents.y(), sourceVoxelSize),
      sdfAxisSampleCount(extents.z(), sourceVoxelSize));

  SdfQueryOptions sourceOptions;
  sourceOptions.maxDistance = shellThickness;

  SdfQueryOptions targetOptions;
  targetOptions.maxDistance
      = boundedSdfQueryDistance(option.upperBound, shellThickness);

  const Eigen::Isometry3d targetInverse = targetTransform.inverse();
  const Eigen::Matrix3d targetRotation = targetTransform.rotation();

  bool found = false;

  for (int z = 0; z < samples.z(); ++z) {
    const double zAlpha
        = (samples.z() == 1)
              ? 0.5
              : static_cast<double>(z) / static_cast<double>(samples.z() - 1);
    for (int y = 0; y < samples.y(); ++y) {
      const double yAlpha
          = (samples.y() == 1)
                ? 0.5
                : static_cast<double>(y) / static_cast<double>(samples.y() - 1);
      for (int x = 0; x < samples.x(); ++x) {
        const double xAlpha = (samples.x() == 1)
                                  ? 0.5
                                  : static_cast<double>(x)
                                        / static_cast<double>(samples.x() - 1);

        const Eigen::Vector3d sourcePointLocal(
            sourceAabb.min.x() + extents.x() * xAlpha,
            sourceAabb.min.y() + extents.y() * yAlpha,
            sourceAabb.min.z() + extents.z() * zAlpha);

        double sourceDistance = 0.0;
        Eigen::Vector3d sourceGradient = Eigen::Vector3d::Zero();
        if (!sourceField->distanceAndGradient(
                sourcePointLocal,
                &sourceDistance,
                &sourceGradient,
                sourceOptions)) {
          continue;
        }
        if (std::abs(sourceDistance) > shellThickness) {
          continue;
        }

        Eigen::Vector3d sourceSurfaceLocal = sourcePointLocal;
        const double sourceGradientNorm = sourceGradient.norm();
        if (sourceGradientNorm > 1e-12) {
          sourceSurfaceLocal
              -= (sourceGradient / sourceGradientNorm) * sourceDistance;
        }

        const Eigen::Vector3d sourceSurfaceWorld
            = sourceTransform * sourceSurfaceLocal;

        double targetDistance = 0.0;
        Eigen::Vector3d targetGradientWorld = Eigen::Vector3d::Zero();
        if (!querySdfDistanceAndGradient(
                targetField,
                targetInverse,
                targetRotation,
                sourceSurfaceWorld,
                targetOptions,
                &targetDistance,
                &targetGradientWorld)) {
          continue;
        }

        if (targetDistance >= best.distance) {
          continue;
        }

        Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
        Eigen::Vector3d targetSurfaceWorld = sourceSurfaceWorld;
        const double targetGradientNorm = targetGradientWorld.norm();
        if (targetGradientNorm > 1e-12) {
          const Eigen::Vector3d targetGradientUnit
              = targetGradientWorld / targetGradientNorm;
          const double sign = (targetDistance >= 0.0) ? 1.0 : -1.0;
          normal = targetGradientUnit * sign;
          targetSurfaceWorld
              = sourceSurfaceWorld - targetGradientUnit * targetDistance;
        }

        best.distance = targetDistance;
        best.pointOnSource = sourceSurfaceWorld;
        best.pointOnTarget = targetSurfaceWorld;
        best.normalSourceToTarget = normal;
        found = true;
      }
    }
  }

  return found;
}

double distancePointSetSdf(
    span<const Eigen::Vector3d> pointsLocal,
    const Eigen::Isometry3d& pointSetTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  if (pointsLocal.empty()) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const SignedDistanceField* field = sdf.getField();
  if (!field) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const Eigen::Isometry3d sdfInverse = sdfTransform.inverse();
  const Eigen::Matrix3d sdfRotation = sdfTransform.rotation();

  SdfQueryOptions queryOptions;
  queryOptions.maxDistance = boundedSdfQueryDistance(option.upperBound, 0.0);

  bool found = false;
  double bestDistance = std::numeric_limits<double>::max();
  Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d bestGradient = Eigen::Vector3d::Zero();

  for (const auto& pointLocal : pointsLocal) {
    const Eigen::Vector3d pointWorld = pointSetTransform * pointLocal;
    double fieldDistance = 0.0;
    Eigen::Vector3d gradientWorld = Eigen::Vector3d::Zero();
    if (!querySdfDistanceAndGradient(
            field,
            sdfInverse,
            sdfRotation,
            pointWorld,
            queryOptions,
            &fieldDistance,
            &gradientWorld)) {
      continue;
    }

    if (fieldDistance < bestDistance) {
      bestDistance = fieldDistance;
      bestPoint = pointWorld;
      bestGradient = gradientWorld;
      found = true;
    }
  }

  if (!found) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  result.distance = bestDistance;
  if (bestDistance > option.upperBound) {
    return bestDistance;
  }

  if (option.enableNearestPoints) {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
    const double gradNorm = bestGradient.norm();
    if (gradNorm > 1e-12) {
      const Eigen::Vector3d gradUnit = bestGradient / gradNorm;
      const double sign = (bestDistance >= 0.0) ? 1.0 : -1.0;
      normal = gradUnit * sign;
      result.pointOnObject2 = bestPoint - gradUnit * bestDistance;
    } else {
      result.pointOnObject2 = bestPoint;
    }
    result.pointOnObject1 = bestPoint;
    result.normal = normal;
  }

  return bestDistance;
}

} // namespace

double distanceSphereSphere(
    const SphereShape& sphere1,
    const Eigen::Isometry3d& transform1,
    const SphereShape& sphere2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option)
{
  const Eigen::Vector3d center1 = transform1.translation();
  const Eigen::Vector3d center2 = transform2.translation();
  const double r1 = sphere1.getRadius();
  const double r2 = sphere2.getRadius();

  const Eigen::Vector3d diff = center2 - center1;
  const double centerDist = diff.norm();
  const double dist = centerDist - r1 - r2;

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    if (centerDist > 1e-10) {
      const Eigen::Vector3d dir = diff / centerDist;
      result.pointOnObject1 = center1 + dir * r1;
      result.pointOnObject2 = center2 - dir * r2;
      result.normal = dir;
    } else {
      result.pointOnObject1 = center1 + Eigen::Vector3d(r1, 0, 0);
      result.pointOnObject2 = center2 - Eigen::Vector3d(r2, 0, 0);
      result.normal = Eigen::Vector3d::UnitX();
    }
  }

  return dist;
}

double distanceSphereBox(
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const double sphereRadius = sphere.getRadius();
  const Eigen::Vector3d& boxHalf = box.getHalfExtents();

  const Eigen::Isometry3d boxInv = boxTransform.inverse();
  const Eigen::Vector3d sphereCenterLocal
      = boxInv * sphereTransform.translation();

  const Eigen::Vector3d closestOnBoxLocal
      = closestPointOnBox(sphereCenterLocal, boxHalf);
  const Eigen::Vector3d diffLocal = sphereCenterLocal - closestOnBoxLocal;
  const double distToSurface = diffLocal.norm();
  const double boundaryTolerance
      = computeBoxBoundaryTolerance(boxHalf, sphereCenterLocal);

  Eigen::Vector3d normalLocal;
  Eigen::Vector3d pointOnBoxLocal = closestOnBoxLocal;
  double dist = distToSurface - sphereRadius;

  if (distToSurface > boundaryTolerance) {
    normalLocal = diffLocal / distToSurface;
  } else {
    Eigen::Vector3d insideLocalSphereCenter = sphereCenterLocal;
    snapNearBoxBoundary(insideLocalSphereCenter, boxHalf, boundaryTolerance);

    double minDist = boxHalf.x() - std::abs(insideLocalSphereCenter.x());
    int minAxis = 0;

    const double distY = boxHalf.y() - std::abs(insideLocalSphereCenter.y());
    if (distY < minDist) {
      minDist = distY;
      minAxis = 1;
    }

    const double distZ = boxHalf.z() - std::abs(insideLocalSphereCenter.z());
    if (distZ < minDist) {
      minDist = distZ;
      minAxis = 2;
    }

    normalLocal = Eigen::Vector3d::Zero();
    normalLocal[minAxis]
        = (insideLocalSphereCenter[minAxis] >= 0.0) ? 1.0 : -1.0;
    pointOnBoxLocal = insideLocalSphereCenter;
    pointOnBoxLocal[minAxis]
        = normalLocal[minAxis] > 0.0 ? boxHalf[minAxis] : -boxHalf[minAxis];
    dist = -(sphereRadius + minDist);
  }

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    result.pointOnObject2 = boxTransform * pointOnBoxLocal;
    result.pointOnObject1
        = sphereTransform.translation()
          - (boxTransform.rotation() * normalLocal) * sphereRadius;
    result.normal = boxTransform.rotation() * normalLocal;
  }

  return dist;
}

double distanceBoxBox(
    const BoxShape& box1,
    const Eigen::Isometry3d& transform1,
    const BoxShape& box2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option)
{
  const Eigen::Vector3d& half1 = box1.getHalfExtents();
  const Eigen::Vector3d& half2 = box2.getHalfExtents();

  if (distanceSameOrientationBoxes(
          half1, transform1, half2, transform2, result, option)) {
    return result.distance;
  }

  box_box::SatResult sat;
  const box_box::BoxData boxData1{
      transform1.translation(), half1, transform1.linear()};
  const box_box::BoxData boxData2{
      transform2.translation(), half2, transform2.linear()};
  if (box_box::computeBoxBoxSat(boxData1, boxData2, sat)) {
    const double dist = -sat.penetration;
    result.distance = dist;
    if (dist > option.upperBound) {
      return dist;
    }

    if (option.enableNearestPoints) {
      Eigen::Vector3d normal = -sat.normal;
      if (normal.squaredNorm() < 1e-10) {
        const Eigen::Vector3d centerDiff
            = transform2.translation() - transform1.translation();
        normal = centerDiff.squaredNorm() > 1e-10 ? centerDiff.normalized()
                                                  : Eigen::Vector3d::UnitX();
      }

      result.normal = normal;
      result.pointOnObject1 = transform1.translation()
                              + normal * box_box::projectBox(boxData1, normal);
      result.pointOnObject2 = transform2.translation()
                              - normal * box_box::projectBox(boxData2, normal);
    }

    return dist;
  }

  const Eigen::Isometry3d inv1 = transform1.inverse();
  const Eigen::Isometry3d box2In1 = inv1 * transform2;

  double minDistSq = std::numeric_limits<double>::max();
  Eigen::Vector3d bestPoint1World = Eigen::Vector3d::Zero();
  Eigen::Vector3d bestPoint2World = Eigen::Vector3d::Zero();

  auto updateCandidate
      = [&](const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
          const double distSq = (point2 - point1).squaredNorm();
          if (distSq < minDistSq) {
            minDistSq = distSq;
            bestPoint1World = point1;
            bestPoint2World = point2;
          }
        };

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d corner2Local(
        (i & 1) ? half2.x() : -half2.x(),
        (i & 2) ? half2.y() : -half2.y(),
        (i & 4) ? half2.z() : -half2.z());

    const Eigen::Vector3d corner2In1 = box2In1 * corner2Local;
    const Eigen::Vector3d closest1 = closestPointOnBox(corner2In1, half1);
    updateCandidate(transform1 * closest1, transform2 * corner2Local);
  }

  const Eigen::Isometry3d inv2 = transform2.inverse();
  const Eigen::Isometry3d box1In2 = inv2 * transform1;

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d corner1Local(
        (i & 1) ? half1.x() : -half1.x(),
        (i & 2) ? half1.y() : -half1.y(),
        (i & 4) ? half1.z() : -half1.z());

    const Eigen::Vector3d corner1In2 = box1In2 * corner1Local;
    const Eigen::Vector3d closest2 = closestPointOnBox(corner1In2, half2);
    updateCandidate(transform1 * corner1Local, transform2 * closest2);
  }

  const auto edges1 = boxEdges(half1);
  const auto edges2 = boxEdges(half2);
  for (const auto& edge1 : edges1) {
    const Eigen::Vector3d edge1Start = transform1 * edge1.start;
    const Eigen::Vector3d edge1End = transform1 * edge1.end;
    for (const auto& edge2 : edges2) {
      const auto closest = closestPointsBetweenSegments(
          edge1Start,
          edge1End,
          transform2 * edge2.start,
          transform2 * edge2.end);
      updateCandidate(closest.point1, closest.point2);
    }
  }

  const double minDist = std::sqrt(minDistSq);
  result.distance = minDist;

  if (minDist > option.upperBound) {
    return minDist;
  }

  if (option.enableNearestPoints) {
    result.pointOnObject1 = bestPoint1World;
    result.pointOnObject2 = bestPoint2World;

    const Eigen::Vector3d diff = result.pointOnObject2 - result.pointOnObject1;
    if (diff.squaredNorm() > 1e-10) {
      result.normal = diff.normalized();
    } else {
      result.normal = Eigen::Vector3d::UnitX();
    }
  }

  return minDist;
}

double distanceCapsuleCapsule(
    const CapsuleShape& capsule1,
    const Eigen::Isometry3d& transform1,
    const CapsuleShape& capsule2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option)
{
  const double r1 = capsule1.getRadius();
  const double h1 = capsule1.getHeight() * 0.5;
  const double r2 = capsule2.getRadius();
  const double h2 = capsule2.getHeight() * 0.5;

  const Eigen::Vector3d axis1 = transform1.rotation().col(2);
  const Eigen::Vector3d axis2 = transform2.rotation().col(2);
  const Eigen::Vector3d center1 = transform1.translation();
  const Eigen::Vector3d center2 = transform2.translation();

  const Eigen::Vector3d top1 = center1 + axis1 * h1;
  const Eigen::Vector3d bot1 = center1 - axis1 * h1;
  const Eigen::Vector3d top2 = center2 + axis2 * h2;
  const Eigen::Vector3d bot2 = center2 - axis2 * h2;

  auto closest = closestPointsBetweenSegments(bot1, top1, bot2, top2);

  const double axisDist = std::sqrt(closest.distSq);
  const double dist = axisDist - r1 - r2;

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    Eigen::Vector3d dir;
    if (axisDist > 1e-10) {
      dir = (closest.point2 - closest.point1) / axisDist;
    } else {
      dir = axis1.cross(axis2);
      if (dir.squaredNorm() < 1e-10) {
        dir = Eigen::Vector3d::UnitX();
      }
      dir.normalize();
    }

    result.pointOnObject1 = closest.point1 + dir * r1;
    result.pointOnObject2 = closest.point2 - dir * r2;
    result.normal = dir;
  }

  return dist;
}

double distanceCapsuleSphere(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const double capRadius = capsule.getRadius();
  const double capHalfHeight = capsule.getHeight() * 0.5;
  const double sphereRadius = sphere.getRadius();

  const Eigen::Vector3d capAxis = capsuleTransform.rotation().col(2);
  const Eigen::Vector3d capCenter = capsuleTransform.translation();
  const Eigen::Vector3d sphereCenter = sphereTransform.translation();

  const Eigen::Vector3d toSphere = sphereCenter - capCenter;
  const double proj = toSphere.dot(capAxis);
  const double clampedProj = std::clamp(proj, -capHalfHeight, capHalfHeight);
  const Eigen::Vector3d closestOnAxis = capCenter + capAxis * clampedProj;

  const Eigen::Vector3d diff = sphereCenter - closestOnAxis;
  const double axisDist = diff.norm();
  const double dist = axisDist - capRadius - sphereRadius;

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    Eigen::Vector3d dir;
    if (axisDist > 1e-10) {
      dir = diff / axisDist;
    } else {
      dir = Eigen::Vector3d::UnitX();
    }

    result.pointOnObject1 = closestOnAxis + dir * capRadius;
    result.pointOnObject2 = sphereCenter - dir * sphereRadius;
    result.normal = dir;
  }

  return dist;
}

double distanceCapsuleBox(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const double capRadius = capsule.getRadius();
  const double capHalfHeight = capsule.getHeight() * 0.5;
  const Eigen::Vector3d& boxHalf = box.getHalfExtents();

  const Eigen::Vector3d capAxis = capsuleTransform.rotation().col(2);
  const Eigen::Vector3d capCenter = capsuleTransform.translation();
  const Eigen::Vector3d capTop = capCenter + capAxis * capHalfHeight;
  const Eigen::Vector3d capBot = capCenter - capAxis * capHalfHeight;

  const Eigen::Isometry3d boxInv = boxTransform.inverse();
  const Eigen::Vector3d topLocal = boxInv * capTop;
  const Eigen::Vector3d botLocal = boxInv * capBot;

  Eigen::Vector3d bestCapsulePoint = Eigen::Vector3d::Zero();
  const Eigen::Vector3d bestBoxPoint = closestPointOnSegmentInBoxSpace(
      botLocal, topLocal, boxHalf, bestCapsulePoint);
  const double minDist = (bestCapsulePoint - bestBoxPoint).norm();
  const double boundaryTolerance
      = computeBoxBoundaryTolerance(boxHalf, bestCapsulePoint);

  Eigen::Vector3d dirLocal = bestCapsulePoint - bestBoxPoint;
  Eigen::Vector3d pointOnBoxLocal = bestBoxPoint;
  double dist = minDist - capRadius;

  if (minDist > boundaryTolerance) {
    dirLocal /= minDist;
  } else {
    Eigen::Vector3d insideLocalCapsulePoint = bestCapsulePoint;
    snapNearBoxBoundary(insideLocalCapsulePoint, boxHalf, boundaryTolerance);

    double minFaceDist = boxHalf.x() - std::abs(insideLocalCapsulePoint.x());
    int minAxis = 0;

    const double distY = boxHalf.y() - std::abs(insideLocalCapsulePoint.y());
    if (distY < minFaceDist) {
      minFaceDist = distY;
      minAxis = 1;
    }

    const double distZ = boxHalf.z() - std::abs(insideLocalCapsulePoint.z());
    if (distZ < minFaceDist) {
      minFaceDist = distZ;
      minAxis = 2;
    }

    minFaceDist = std::max(0.0, minFaceDist);
    dirLocal = Eigen::Vector3d::Zero();
    dirLocal[minAxis] = (insideLocalCapsulePoint[minAxis] >= 0.0) ? 1.0 : -1.0;
    pointOnBoxLocal = insideLocalCapsulePoint;
    pointOnBoxLocal[minAxis]
        = dirLocal[minAxis] > 0.0 ? boxHalf[minAxis] : -boxHalf[minAxis];
    dist = -(capRadius + minFaceDist);
  }

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    result.pointOnObject2 = boxTransform * pointOnBoxLocal;
    result.pointOnObject1
        = boxTransform * (bestCapsulePoint - dirLocal * capRadius);
    result.normal = boxTransform.rotation() * dirLocal;
  }

  return dist;
}

double distancePlaneShape(
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    const Shape& shape,
    const Eigen::Isometry3d& shapeTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  Eigen::Vector3d supportPoint = Eigen::Vector3d::Zero();
  if (!supportPointOnShape(shape, shapeTransform, -worldNormal, supportPoint)) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const double signedDist = worldNormal.dot(supportPoint - planePoint);
  if (signedDist > option.upperBound) {
    result.distance = signedDist;
    return signedDist;
  }

  result.distance = signedDist;

  if (option.enableNearestPoints) {
    result.pointOnObject2 = supportPoint;
    result.pointOnObject1 = supportPoint - worldNormal * signedDist;
    const Eigen::Vector3d delta = result.pointOnObject2 - result.pointOnObject1;
    if (delta.squaredNorm() > kSupportEps) {
      result.normal = delta.normalized();
    } else {
      result.normal = worldNormal;
    }
  }

  return signedDist;
}

double distanceSphereSdf(
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const Eigen::Vector3d center = sphereTransform.translation();
  const double radius = sphere.getRadius();

  const SignedDistanceField* field = sdf.getField();
  if (!field) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const Eigen::Isometry3d sdf_inverse = sdfTransform.inverse();
  const Eigen::Matrix3d sdf_rotation = sdfTransform.rotation();

  SdfQueryOptions query_options;
  query_options.maxDistance = option.upperBound + radius;

  double field_distance = 0.0;
  Eigen::Vector3d gradientWorld = Eigen::Vector3d::Zero();
  if (!querySdfDistanceAndGradient(
          field,
          sdf_inverse,
          sdf_rotation,
          center,
          query_options,
          &field_distance,
          &gradientWorld)) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const double dist = field_distance - radius;
  result.distance = dist;

  if (dist > option.upperBound) {
    return dist;
  }

  if (option.enableNearestPoints) {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
    const double grad_norm = gradientWorld.norm();
    if (grad_norm > 1e-12) {
      const Eigen::Vector3d grad_unit = gradientWorld / grad_norm;
      const double sign = (field_distance >= 0.0) ? 1.0 : -1.0;
      normal = grad_unit * sign;

      result.pointOnObject2 = center - grad_unit * field_distance;
      result.pointOnObject1 = center + normal * radius;
    } else {
      result.pointOnObject2 = center;
      result.pointOnObject1 = center + normal * radius;
    }
    result.normal = normal;
  }

  return dist;
}

double distanceCapsuleSdf(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const double radius = capsule.getRadius();
  const double halfHeight = capsule.getHeight() * 0.5;

  const Eigen::Vector3d axis = capsuleTransform.rotation().col(2);
  const Eigen::Vector3d center = capsuleTransform.translation();
  const Eigen::Vector3d top = center + axis * halfHeight;
  const Eigen::Vector3d bottom = center - axis * halfHeight;

  const SignedDistanceField* field = sdf.getField();
  if (!field) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const Eigen::Isometry3d sdf_inverse = sdfTransform.inverse();
  const Eigen::Matrix3d sdf_rotation = sdfTransform.rotation();

  SdfQueryOptions query_options;
  query_options.maxDistance = option.upperBound + radius;

  const double voxel_size = std::max(field->voxelSize(), 1e-6);
  const int raw_samples
      = static_cast<int>(std::ceil(capsule.getHeight() / voxel_size)) + 1;
  const int num_samples = std::clamp(raw_samples, 2, 32);

  bool found = false;
  double best_dist = std::numeric_limits<double>::max();
  double best_field_distance = 0.0;
  Eigen::Vector3d best_axis_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d best_gradient = Eigen::Vector3d::Zero();

  for (int i = 0; i < num_samples; ++i) {
    const double t
        = (num_samples == 1) ? 0.5 : static_cast<double>(i) / (num_samples - 1);
    const Eigen::Vector3d axis_point = bottom + (top - bottom) * t;

    double field_distance = 0.0;
    Eigen::Vector3d gradientWorld = Eigen::Vector3d::Zero();
    if (!querySdfDistanceAndGradient(
            field,
            sdf_inverse,
            sdf_rotation,
            axis_point,
            query_options,
            &field_distance,
            &gradientWorld)) {
      continue;
    }

    const double dist = field_distance - radius;
    if (dist < best_dist) {
      best_dist = dist;
      best_field_distance = field_distance;
      best_axis_point = axis_point;
      best_gradient = gradientWorld;
      found = true;
    }
  }

  if (!found) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  result.distance = best_dist;
  if (best_dist > option.upperBound) {
    return best_dist;
  }

  if (option.enableNearestPoints) {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
    const double grad_norm = best_gradient.norm();
    if (grad_norm > 1e-12) {
      const Eigen::Vector3d grad_unit = best_gradient / grad_norm;
      const double sign = (best_field_distance >= 0.0) ? 1.0 : -1.0;
      normal = grad_unit * sign;

      result.pointOnObject2 = best_axis_point - grad_unit * best_field_distance;
      result.pointOnObject1 = best_axis_point + normal * radius;
    } else {
      result.pointOnObject2 = best_axis_point;
      result.pointOnObject1 = best_axis_point + normal * radius;
    }
    result.normal = normal;
  }

  return best_dist;
}

double distanceCylinderSdf(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const SignedDistanceField* field = sdf.getField();
  if (!field) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const double radius = cylinder.getRadius();
  const double halfHeight = cylinder.getHeight() * 0.5;
  const double voxelSize = std::max(field->voxelSize(), 1e-6);
  const int axialSamples = std::clamp(
      static_cast<int>(std::ceil(cylinder.getHeight() / voxelSize)) + 1, 2, 32);
  const int radialSamples
      = std::clamp(static_cast<int>(std::ceil(radius / voxelSize)) + 1, 2, 16);
  const int angularSamples = std::clamp(
      static_cast<int>(std::ceil(2.0 * kPi * radius / voxelSize)), 16, 64);

  const Eigen::Isometry3d sdfInverse = sdfTransform.inverse();
  const Eigen::Matrix3d sdfRotation = sdfTransform.rotation();

  SdfQueryOptions queryOptions;
  const double boundingRadius
      = std::sqrt(radius * radius + halfHeight * halfHeight);
  queryOptions.maxDistance
      = boundedSdfQueryDistance(option.upperBound, boundingRadius);

  bool found = false;
  double bestDistance = std::numeric_limits<double>::max();
  Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d bestGradient = Eigen::Vector3d::Zero();

  const auto querySample = [&](const Eigen::Vector3d& pointLocal) {
    const Eigen::Vector3d pointWorld = cylinderTransform * pointLocal;
    double fieldDistance = 0.0;
    Eigen::Vector3d gradientWorld = Eigen::Vector3d::Zero();
    if (!querySdfDistanceAndGradient(
            field,
            sdfInverse,
            sdfRotation,
            pointWorld,
            queryOptions,
            &fieldDistance,
            &gradientWorld)) {
      return;
    }

    if (fieldDistance < bestDistance) {
      bestDistance = fieldDistance;
      bestPoint = pointWorld;
      bestGradient = gradientWorld;
      found = true;
    }
  };

  for (int zIndex = 0; zIndex < axialSamples; ++zIndex) {
    const double zAlpha = (axialSamples == 1)
                              ? 0.5
                              : static_cast<double>(zIndex)
                                    / static_cast<double>(axialSamples - 1);
    const double z = -halfHeight + 2.0 * halfHeight * zAlpha;

    querySample(Eigen::Vector3d(0.0, 0.0, z));

    for (int rIndex = 1; rIndex < radialSamples; ++rIndex) {
      const double r = radius * static_cast<double>(rIndex)
                       / static_cast<double>(radialSamples - 1);
      for (int angleIndex = 0; angleIndex < angularSamples; ++angleIndex) {
        const double theta = 2.0 * kPi * static_cast<double>(angleIndex)
                             / static_cast<double>(angularSamples);
        querySample(
            Eigen::Vector3d(r * std::cos(theta), r * std::sin(theta), z));
      }
    }
  }

  if (!found) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  result.distance = bestDistance;
  if (bestDistance > option.upperBound) {
    return bestDistance;
  }

  if (option.enableNearestPoints) {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
    const double gradNorm = bestGradient.norm();
    if (gradNorm > 1e-12) {
      const Eigen::Vector3d gradUnit = bestGradient / gradNorm;
      const double sign = (bestDistance >= 0.0) ? 1.0 : -1.0;
      normal = gradUnit * sign;
      result.pointOnObject2 = bestPoint - gradUnit * bestDistance;
    } else {
      result.pointOnObject2 = bestPoint;
    }
    result.pointOnObject1 = bestPoint;
    result.normal = normal;
  }

  return bestDistance;
}

double distanceConvexSdf(
    const ConvexShape& convex,
    const Eigen::Isometry3d& convexTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  return distancePointSetSdf(
      convex.getVertices(), convexTransform, sdf, sdfTransform, result, option);
}

double distanceMeshSdf(
    const MeshShape& mesh,
    const Eigen::Isometry3d& meshTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option)
{
  return distancePointSetSdf(
      mesh.getVertices(), meshTransform, sdf, sdfTransform, result, option);
}

double distanceSdfSdf(
    const SdfShape& sdf1,
    const Eigen::Isometry3d& sdf1Transform,
    const SdfShape& sdf2,
    const Eigen::Isometry3d& sdf2Transform,
    DistanceResult& result,
    const DistanceOption& option)
{
  const SignedDistanceField* field1 = sdf1.getField();
  const SignedDistanceField* field2 = sdf2.getField();
  if (!field1 || !field2) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const double shellThickness
      = 1.5 * std::max({field1->voxelSize(), field2->voxelSize(), 1e-6});

  SdfSdfCandidate best;
  bool found = sampleSdfSurfaceAgainstSdf(
      field1,
      sdf1Transform,
      field2,
      sdf2Transform,
      shellThickness,
      option,
      best);

  SdfSdfCandidate reverseBest;
  if (sampleSdfSurfaceAgainstSdf(
          field2,
          sdf2Transform,
          field1,
          sdf1Transform,
          shellThickness,
          option,
          reverseBest)
      && reverseBest.distance < best.distance) {
    best.distance = reverseBest.distance;
    best.pointOnSource = reverseBest.pointOnTarget;
    best.pointOnTarget = reverseBest.pointOnSource;
    best.normalSourceToTarget = -reverseBest.normalSourceToTarget;
    found = true;
  }

  if (!found) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  result.distance = best.distance;
  if (best.distance > option.upperBound) {
    return best.distance;
  }

  if (option.enableNearestPoints) {
    result.pointOnObject1 = best.pointOnSource;
    result.pointOnObject2 = best.pointOnTarget;
    result.normal = best.normalSourceToTarget;
  }

  return best.distance;
}

} // namespace dart::collision::native
