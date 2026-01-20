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

#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

namespace {

struct SegmentClosestResult
{
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
  double distSq;
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

constexpr double kSupportEps = 1e-12;

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
  const double dist = distToSurface - sphereRadius;

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    Eigen::Vector3d normalLocal;
    if (distToSurface > 1e-10) {
      normalLocal = diffLocal / distToSurface;
    } else {
      normalLocal = Eigen::Vector3d::UnitX();
    }

    result.pointOnObject2 = boxTransform * closestOnBoxLocal;
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

  const Eigen::Isometry3d inv1 = transform1.inverse();
  const Eigen::Isometry3d box2In1 = inv1 * transform2;

  double minDist = std::numeric_limits<double>::max();
  Eigen::Vector3d bestPoint1Local, bestPoint2Local;

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d corner2Local(
        (i & 1) ? half2.x() : -half2.x(),
        (i & 2) ? half2.y() : -half2.y(),
        (i & 4) ? half2.z() : -half2.z());

    const Eigen::Vector3d corner2In1 = box2In1 * corner2Local;
    const Eigen::Vector3d closest1 = closestPointOnBox(corner2In1, half1);
    const double d = (corner2In1 - closest1).norm();

    if (d < minDist) {
      minDist = d;
      bestPoint1Local = closest1;
      bestPoint2Local = corner2Local;
    }
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
    const double d = (corner1In2 - closest2).norm();

    if (d < minDist) {
      minDist = d;
      bestPoint1Local = corner1Local;
      bestPoint2Local = closest2;
    }
  }

  result.distance = minDist;

  if (minDist > option.upperBound) {
    return minDist;
  }

  if (option.enableNearestPoints) {
    result.pointOnObject1 = transform1 * bestPoint1Local;
    result.pointOnObject2 = transform2 * bestPoint2Local;

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

  double minDist = std::numeric_limits<double>::max();
  Eigen::Vector3d bestCapsulePoint, bestBoxPoint;

  constexpr int numSamples = 5;
  for (int i = 0; i < numSamples; ++i) {
    const double t = static_cast<double>(i) / (numSamples - 1);
    const Eigen::Vector3d axisPointLocal = botLocal + (topLocal - botLocal) * t;
    const Eigen::Vector3d closestOnBox
        = closestPointOnBox(axisPointLocal, boxHalf);
    const double d = (axisPointLocal - closestOnBox).norm();

    if (d < minDist) {
      minDist = d;
      bestCapsulePoint = axisPointLocal;
      bestBoxPoint = closestOnBox;
    }
  }

  const double dist = minDist - capRadius;

  if (dist > option.upperBound) {
    result.distance = dist;
    return dist;
  }

  result.distance = dist;

  if (option.enableNearestPoints) {
    Eigen::Vector3d dirLocal = bestCapsulePoint - bestBoxPoint;
    if (dirLocal.squaredNorm() > 1e-10) {
      dirLocal.normalize();
    } else {
      dirLocal = Eigen::Vector3d::UnitX();
    }

    result.pointOnObject2 = boxTransform * bestBoxPoint;
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

} // namespace dart::collision::experimental
