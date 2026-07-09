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

#include <dart/collision/native/narrow_phase/Ccd.hpp>
#include <dart/collision/native/narrow_phase/Gjk-impl.hpp>
#include <dart/collision/native/narrow_phase/Gjk.hpp>

#include <algorithm>
#include <array>

#include <cmath>

namespace dart::collision::native {

namespace {

constexpr double kEpsilon = 1e-10;

double solveQuadraticSmallestPositive(double a, double b, double c)
{
  if (std::abs(a) < kEpsilon) {
    if (std::abs(b) < kEpsilon) {
      return -1.0;
    }
    double t = -c / b;
    return (t >= 0.0) ? t : -1.0;
  }

  double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return -1.0;
  }

  double sqrtDisc = std::sqrt(discriminant);
  double t0 = (-b - sqrtDisc) / (2.0 * a);
  double t1 = (-b + sqrtDisc) / (2.0 * a);

  if (t0 > t1) {
    std::swap(t0, t1);
  }

  if (t0 >= 0.0) {
    return t0;
  }
  if (t1 >= 0.0) {
    return t1;
  }
  return -1.0;
}

ConvexShape makeTriangleConvex(
    const MeshShape& mesh, const MeshShape::Triangle& triangle)
{
  const auto& vertices = mesh.getVertices();
  return ConvexShape(
      {vertices[static_cast<std::size_t>(triangle[0])],
       vertices[static_cast<std::size_t>(triangle[1])],
       vertices[static_cast<std::size_t>(triangle[2])]});
}

void updateBestCcdResult(
    const CcdResult& candidate, double& bestT, CcdResult& bestResult)
{
  if (candidate.timeOfImpact < bestT) {
    bestT = candidate.timeOfImpact;
    bestResult = candidate;
  }
}

Eigen::Vector3d normalizedOr(
    const Eigen::Vector3d& value, const Eigen::Vector3d& fallback)
{
  if (value.squaredNorm() < kEpsilon) {
    return fallback;
  }
  return value.normalized();
}

Eigen::Vector3d getCapsuleEndpoint(
    const Eigen::Isometry3d& transform, double halfHeight, bool top)
{
  Eigen::Vector3d localPoint(0, 0, top ? halfHeight : -halfHeight);
  return transform * localPoint;
}

// Samples a rigid motion start -> end at parameter t in [0, 1]. Endpoint
// quaternions are built once at construction so the conservative-advancement
// loop pays only a slerp (or nothing, for translation) per iteration instead of
// reconstructing quaternions from rotation matrices every step.
class MotionSample
{
public:
  MotionSample(const Eigen::Isometry3d& start, const Eigen::Isometry3d& end)
    : startTranslation_(start.translation()),
      endTranslation_(end.translation()),
      startRotation_(start.rotation()),
      rotates_(!startRotation_.isApprox(end.rotation())),
      qStart_(startRotation_),
      qEnd_(end.rotation())
  {
  }

  [[nodiscard]] Eigen::Isometry3d at(double t) const
  {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = (1.0 - t) * startTranslation_ + t * endTranslation_;
    // Translation-only motion skips the slerp; rotational motion reuses the
    // cached endpoint quaternions.
    result.linear() = rotates_ ? qStart_.slerp(t, qEnd_).toRotationMatrix()
                               : startRotation_;
    return result;
  }

private:
  Eigen::Vector3d startTranslation_;
  Eigen::Vector3d endTranslation_;
  Eigen::Matrix3d startRotation_;
  bool rotates_;
  Eigen::Quaterniond qStart_;
  Eigen::Quaterniond qEnd_;
};

double computeCapsuleMotionBound(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& transformStart,
    const Eigen::Isometry3d& transformEnd)
{
  const double linearVelocity
      = (transformEnd.translation() - transformStart.translation()).norm();

  const Eigen::Quaterniond qStart(transformStart.rotation());
  const Eigen::Quaterniond qEnd(transformEnd.rotation());
  const double angle = qStart.angularDistance(qEnd);

  const double maxRadius = capsule.getHeight() / 2.0 + capsule.getRadius();
  return linearVelocity + angle * maxRadius;
}

// World-space support functions for the convex primitive targets (the shape
// classes do not expose support(), so they are provided analytically here).
Eigen::Vector3d sphereSupport(
    const Eigen::Vector3d& center,
    double radius,
    const Eigen::Vector3d& direction)
{
  return center + radius * normalizedOr(direction, Eigen::Vector3d::UnitX());
}

Eigen::Vector3d boxSupport(
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector3d& direction)
{
  const Eigen::Vector3d localDir = transform.rotation().transpose() * direction;
  const Eigen::Vector3d localPoint(
      std::copysign(halfExtents.x(), localDir.x()),
      std::copysign(halfExtents.y(), localDir.y()),
      std::copysign(halfExtents.z(), localDir.z()));
  return transform * localPoint;
}

Eigen::Vector3d closestPointOnBox(
    const Eigen::Vector3d& localPoint, const Eigen::Vector3d& halfExtents)
{
  Eigen::Vector3d closest;
  for (int i = 0; i < 3; ++i) {
    closest[i] = std::clamp(localPoint[i], -halfExtents[i], halfExtents[i]);
  }
  return closest;
}

Eigen::Vector3d getCapsuleSupport(
    const Eigen::Isometry3d& transform,
    const CapsuleShape& capsule,
    const Eigen::Vector3d& direction)
{
  const Eigen::Vector3d worldDir
      = normalizedOr(direction, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d localDir = transform.rotation().transpose() * worldDir;
  const double halfHeight = capsule.getHeight() / 2.0;
  const Eigen::Vector3d localEndpoint(
      0.0, 0.0, localDir.z() >= 0.0 ? halfHeight : -halfHeight);
  return transform * localEndpoint + capsule.getRadius() * worldDir;
}

Eigen::Vector3d cylinderSupport(
    const Eigen::Isometry3d& transform,
    double radius,
    double halfHeight,
    const Eigen::Vector3d& direction)
{
  const Eigen::Vector3d localDir = transform.rotation().transpose() * direction;
  Eigen::Vector3d localPoint(0.0, 0.0, std::copysign(halfHeight, localDir.z()));
  const double radial
      = std::sqrt(localDir.x() * localDir.x() + localDir.y() * localDir.y());
  if (radial > kEpsilon) {
    localPoint.x() = radius * localDir.x() / radial;
    localPoint.y() = radius * localDir.y() / radial;
  }
  return transform * localPoint;
}

// Conservative advancement of a moving sphere against any static convex target
// supplied as a world-space support function. This mirrors the ConvexShape
// sphere cast but also lets primitive targets with analytic support (cylinders,
// capsules) cover rounded edge cases that slab/quadratic decompositions miss.
template <typename TargetSupport>
bool sphereCastConvexTarget(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const TargetSupport& targetSupport,
    const Eigen::Vector3d& targetSeed,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  double motionBound = (sphereEnd - sphereStart).norm();
  if (motionBound < option.tolerance) {
    motionBound = option.tolerance;
  }

  double t = 0.0;

  Eigen::Vector3d initialDir = targetSeed - sphereStart;
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  for (int iter = 0; iter < std::max(1, option.maxIterations); ++iter) {
    const Eigen::Vector3d currentCenter
        = sphereStart + t * (sphereEnd - sphereStart);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return sphereSupport(currentCenter, sphereRadius, dir);
    };

    GjkResult gjkResult = detail::queryT(supportA, targetSupport, initialDir);

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = targetSupport(Eigen::Vector3d::UnitX());
      result.normal = normalizedOr(
          currentCenter - result.point, Eigen::Vector3d::UnitZ());
      return true;
    }

    const Eigen::Vector3d pointA = gjkResult.closestPointA;
    const Eigen::Vector3d pointB = gjkResult.closestPointB;
    const double distance = gjkResult.distance;

    Eigen::Vector3d sepAxis = gjkResult.separationAxis;
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = pointB - pointA;
    }
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = Eigen::Vector3d::UnitX();
    }
    sepAxis.normalize();

    if (distance < option.tolerance) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = pointB;
      result.normal = -sepAxis;
      return true;
    }

    t += distance / motionBound;
    if (t > 1.0) {
      return false;
    }

    initialDir = sepAxis;
  }

  return false;
}

} // namespace

bool sphereCastSphere(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const SphereShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  (void)option;
  result.clear();

  const Eigen::Vector3d targetCenter = targetTransform.translation();
  const double targetRadius = target.getRadius();
  const double combinedRadius = sphereRadius + targetRadius;

  const Eigen::Vector3d d = sphereEnd - sphereStart;
  const Eigen::Vector3d f = sphereStart - targetCenter;

  const double a = d.squaredNorm();
  const double b = 2.0 * f.dot(d);
  const double c = f.squaredNorm() - combinedRadius * combinedRadius;

  if (c < 0.0) {
    result.hit = true;
    result.timeOfImpact = 0.0;
    result.point = sphereStart;
    const Eigen::Vector3d normal = sphereStart - targetCenter;
    if (normal.squaredNorm() < kEpsilon) {
      result.normal = Eigen::Vector3d::UnitZ();
    } else {
      result.normal = normal.normalized();
    }
    return true;
  }

  double t = solveQuadraticSmallestPositive(a, b, c);

  if (t < 0.0 || t > 1.0) {
    return false;
  }

  result.hit = true;
  result.timeOfImpact = t;

  Eigen::Vector3d hitSphereCenter = sphereStart + t * d;
  result.normal = (hitSphereCenter - targetCenter).normalized();
  result.point = targetCenter + targetRadius * result.normal;

  return true;
}

bool sphereCastBox(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const BoxShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  const Eigen::Isometry3d invTransform = targetTransform.inverse();
  const Eigen::Vector3d localStart = invTransform * sphereStart;
  const Eigen::Vector3d localEnd = invTransform * sphereEnd;
  const Eigen::Vector3d localDir = localEnd - localStart;

  const Eigen::Vector3d& halfExtents = target.getHalfExtents();
  const Eigen::Vector3d expandedHalf
      = halfExtents + Eigen::Vector3d::Constant(sphereRadius);
  const double expandedRadius = sphereRadius + option.tolerance;
  const double expandedRadiusSq = expandedRadius * expandedRadius;

  auto runSupportFallback = [&]() {
    return sphereCastConvexTarget(
        sphereStart,
        sphereEnd,
        sphereRadius,
        [&](const Eigen::Vector3d& dir) {
          return boxSupport(targetTransform, halfExtents, dir);
        },
        targetTransform.translation(),
        option,
        result);
  };

  double tMin = 0.0;
  double tMax = 1.0;
  int hitAxis = -1;
  int hitSign = 1;

  for (int i = 0; i < 3; ++i) {
    if (std::abs(localDir[i]) < kEpsilon) {
      if (localStart[i] < -expandedHalf[i] || localStart[i] > expandedHalf[i]) {
        return false;
      }
    } else {
      double invD = 1.0 / localDir[i];
      double t1 = (-expandedHalf[i] - localStart[i]) * invD;
      double t2 = (expandedHalf[i] - localStart[i]) * invD;

      int sign = -1;
      if (t1 > t2) {
        std::swap(t1, t2);
        sign = 1;
      }

      if (t1 > tMin) {
        tMin = t1;
        hitAxis = i;
        hitSign = sign;
      }
      tMax = std::min(tMax, t2);

      if (tMin > tMax) {
        return false;
      }
    }
  }

  if (hitAxis < 0 || tMin <= 0.0) {
    const Eigen::Vector3d closestOnBox
        = closestPointOnBox(localStart, halfExtents);
    Eigen::Vector3d localNormal = localStart - closestOnBox;
    if (localNormal.squaredNorm() > expandedRadiusSq) {
      return runSupportFallback();
    }

    result.hit = true;
    result.timeOfImpact = 0.0;
    result.point = targetTransform * closestOnBox;

    if (localNormal.squaredNorm() < kEpsilon) {
      localNormal = Eigen::Vector3d::UnitZ();
    } else {
      localNormal.normalize();
    }
    result.normal = targetTransform.rotation() * localNormal;
    return true;
  }

  result.hit = true;
  result.timeOfImpact = tMin;

  Eigen::Vector3d hitCenter = localStart + tMin * localDir;
  const Eigen::Vector3d closestOnBox
      = closestPointOnBox(hitCenter, halfExtents);
  Eigen::Vector3d localNormal = hitCenter - closestOnBox;
  if (localNormal.squaredNorm() > expandedRadiusSq) {
    return runSupportFallback();
  }

  if (localNormal.squaredNorm() < kEpsilon) {
    localNormal = Eigen::Vector3d::Zero();
    localNormal[hitAxis] = static_cast<double>(hitSign);
  } else {
    localNormal.normalize();
  }

  result.point = targetTransform * closestOnBox;
  result.normal = targetTransform.rotation() * localNormal;

  return true;
}

bool sphereCastCapsule(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CapsuleShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  (void)option;
  result.clear();

  const double capsuleRadius = target.getRadius();
  const double halfHeight = target.getHeight() / 2.0;
  const double combinedRadius = sphereRadius + capsuleRadius;

  const Eigen::Isometry3d invTransform = targetTransform.inverse();
  const Eigen::Vector3d localStart = invTransform * sphereStart;
  const Eigen::Vector3d localEnd = invTransform * sphereEnd;
  const Eigen::Vector3d localDir = localEnd - localStart;

  const double closestZ = std::clamp(localStart.z(), -halfHeight, halfHeight);
  const Eigen::Vector3d closestAxisPoint(0.0, 0.0, closestZ);
  const Eigen::Vector3d initialDelta = localStart - closestAxisPoint;
  if (initialDelta.squaredNorm() <= combinedRadius * combinedRadius) {
    Eigen::Vector3d localNormal
        = normalizedOr(initialDelta, Eigen::Vector3d::UnitX());
    if (initialDelta.squaredNorm() < kEpsilon) {
      if (localStart.z() >= halfHeight) {
        localNormal = Eigen::Vector3d::UnitZ();
      } else if (localStart.z() <= -halfHeight) {
        localNormal = -Eigen::Vector3d::UnitZ();
      }
    }

    result.hit = true;
    result.timeOfImpact = 0.0;
    result.normal = targetTransform.rotation() * localNormal;
    result.point
        = targetTransform * (closestAxisPoint + capsuleRadius * localNormal);
    return true;
  }

  double bestT = 2.0;
  Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();

  auto testSphereCap = [&](const Eigen::Vector3d& capCenter) {
    Eigen::Vector3d f = localStart - capCenter;
    double a = localDir.squaredNorm();
    double b = 2.0 * f.dot(localDir);
    double c = f.squaredNorm() - combinedRadius * combinedRadius;

    double t = solveQuadraticSmallestPositive(a, b, c);
    if (t >= 0.0 && t <= 1.0 && t < bestT) {
      bestT = t;
      Eigen::Vector3d hitCenter = localStart + t * localDir;
      bestNormal
          = normalizedOr(hitCenter - capCenter, Eigen::Vector3d::UnitZ());
      bestPoint = capCenter + capsuleRadius * bestNormal;
    }
  };

  testSphereCap(Eigen::Vector3d(0, 0, halfHeight));
  testSphereCap(Eigen::Vector3d(0, 0, -halfHeight));

  double dx = localDir.x();
  double dy = localDir.y();
  double ox = localStart.x();
  double oy = localStart.y();

  double a = dx * dx + dy * dy;
  double b = 2.0 * (ox * dx + oy * dy);
  double c = ox * ox + oy * oy - combinedRadius * combinedRadius;

  if (a > kEpsilon) {
    double t = solveQuadraticSmallestPositive(a, b, c);
    if (t >= 0.0 && t <= 1.0 && t < bestT) {
      Eigen::Vector3d hitCenter = localStart + t * localDir;
      double z = hitCenter.z();
      if (z >= -halfHeight && z <= halfHeight) {
        bestT = t;
        bestNormal
            = Eigen::Vector3d(hitCenter.x(), hitCenter.y(), 0.0).normalized();
        bestPoint = Eigen::Vector3d(
            capsuleRadius * bestNormal.x(), capsuleRadius * bestNormal.y(), z);
      }
    }
  }

  if (bestT > 1.0) {
    return false;
  }

  result.hit = true;
  result.timeOfImpact = bestT;
  result.normal = targetTransform.rotation() * bestNormal;
  result.point = targetTransform * bestPoint;

  return true;
}

bool sphereCastPlane(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const PlaneShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  (void)option;
  result.clear();

  const Eigen::Vector3d worldNormal
      = targetTransform.rotation() * target.getNormal();
  const Eigen::Vector3d worldPoint
      = targetTransform.translation() + target.getOffset() * worldNormal;

  double startDist = (sphereStart - worldPoint).dot(worldNormal);
  double endDist = (sphereEnd - worldPoint).dot(worldNormal);

  double effectiveStartDist = startDist - sphereRadius;
  double effectiveEndDist = endDist - sphereRadius;

  if (effectiveStartDist <= 0.0) {
    result.hit = true;
    result.timeOfImpact = 0.0;
    result.point = sphereStart - sphereRadius * worldNormal;
    result.normal = worldNormal;
    return true;
  }

  if (effectiveEndDist >= 0.0) {
    return false;
  }

  double t = effectiveStartDist / (effectiveStartDist - effectiveEndDist);

  result.hit = true;
  result.timeOfImpact = t;
  Eigen::Vector3d hitCenter = sphereStart + t * (sphereEnd - sphereStart);
  result.point = hitCenter - sphereRadius * worldNormal;
  result.normal = worldNormal;

  return true;
}

bool sphereCastCylinder(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CylinderShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  const double cylinderRadius = target.getRadius();
  const double halfHeight = target.getHeight() / 2.0;
  const double combinedRadius = sphereRadius + cylinderRadius;

  const Eigen::Isometry3d invTransform = targetTransform.inverse();
  const Eigen::Vector3d localStart = invTransform * sphereStart;
  const Eigen::Vector3d localEnd = invTransform * sphereEnd;
  const Eigen::Vector3d localDir = localEnd - localStart;

  const double startRadial = std::sqrt(
      localStart.x() * localStart.x() + localStart.y() * localStart.y());
  const double closestRadius = std::min(startRadial, cylinderRadius);
  const double closestZ = std::clamp(localStart.z(), -halfHeight, halfHeight);
  Eigen::Vector3d closestOnCylinder(0.0, 0.0, closestZ);
  if (startRadial > kEpsilon) {
    closestOnCylinder.x() = closestRadius * localStart.x() / startRadial;
    closestOnCylinder.y() = closestRadius * localStart.y() / startRadial;
  }

  const Eigen::Vector3d initialDelta = localStart - closestOnCylinder;
  if (initialDelta.squaredNorm() <= sphereRadius * sphereRadius) {
    Eigen::Vector3d localNormal
        = normalizedOr(initialDelta, Eigen::Vector3d::UnitX());
    Eigen::Vector3d localPoint = closestOnCylinder;

    if (initialDelta.squaredNorm() < kEpsilon) {
      Eigen::Vector3d radialNormal = Eigen::Vector3d::UnitX();
      if (startRadial > kEpsilon) {
        radialNormal = Eigen::Vector3d(
            localStart.x() / startRadial, localStart.y() / startRadial, 0.0);
      }

      double nearestDistance = cylinderRadius - startRadial;
      localNormal = radialNormal;
      localPoint = Eigen::Vector3d(
          cylinderRadius * radialNormal.x(),
          cylinderRadius * radialNormal.y(),
          localStart.z());

      const double topDistance = halfHeight - localStart.z();
      if (topDistance < nearestDistance) {
        nearestDistance = topDistance;
        localNormal = Eigen::Vector3d::UnitZ();
        localPoint
            = Eigen::Vector3d(localStart.x(), localStart.y(), halfHeight);
      }

      const double bottomDistance = localStart.z() + halfHeight;
      if (bottomDistance < nearestDistance) {
        localNormal = -Eigen::Vector3d::UnitZ();
        localPoint
            = Eigen::Vector3d(localStart.x(), localStart.y(), -halfHeight);
      }
    }

    result.hit = true;
    result.timeOfImpact = 0.0;
    result.normal = targetTransform.rotation() * localNormal;
    result.point = targetTransform * localPoint;
    return true;
  }

  double bestT = 2.0;
  Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();

  double dx = localDir.x();
  double dy = localDir.y();
  double ox = localStart.x();
  double oy = localStart.y();

  double a = dx * dx + dy * dy;
  double b = 2.0 * (ox * dx + oy * dy);
  double c = ox * ox + oy * oy - combinedRadius * combinedRadius;

  if (a > kEpsilon) {
    double t = solveQuadraticSmallestPositive(a, b, c);
    if (t >= 0.0 && t <= 1.0 && t < bestT) {
      Eigen::Vector3d hitCenter = localStart + t * localDir;
      double z = hitCenter.z();
      if (z >= -halfHeight && z <= halfHeight) {
        bestT = t;
        bestNormal
            = Eigen::Vector3d(hitCenter.x(), hitCenter.y(), 0.0).normalized();
        bestPoint = Eigen::Vector3d(
            cylinderRadius * bestNormal.x(),
            cylinderRadius * bestNormal.y(),
            z);
      }
    }
  }

  double expandedHalfHeight = halfHeight + sphereRadius;
  if (std::abs(localDir.z()) > kEpsilon) {
    for (double capZ : {-expandedHalfHeight, expandedHalfHeight}) {
      double t = (capZ - localStart.z()) / localDir.z();
      if (t >= 0.0 && t <= 1.0 && t < bestT) {
        Eigen::Vector3d hitCenter = localStart + t * localDir;
        double distSq
            = hitCenter.x() * hitCenter.x() + hitCenter.y() * hitCenter.y();
        if (distSq <= cylinderRadius * cylinderRadius) {
          bestT = t;
          bestNormal = (capZ > 0.0) ? Eigen::Vector3d(0, 0, 1)
                                    : Eigen::Vector3d(0, 0, -1);
          double actualCapZ = (capZ > 0.0) ? halfHeight : -halfHeight;
          bestPoint = Eigen::Vector3d(hitCenter.x(), hitCenter.y(), actualCapZ);
        }
      }
    }
  }

  CcdResult supportResult;
  if (sphereCastConvexTarget(
          sphereStart,
          sphereEnd,
          sphereRadius,
          [&](const Eigen::Vector3d& dir) {
            return cylinderSupport(
                targetTransform, cylinderRadius, halfHeight, dir);
          },
          targetTransform.translation(),
          option,
          supportResult)
      && (bestT > 1.0
          || supportResult.timeOfImpact + option.tolerance < bestT)) {
    result = supportResult;
    return true;
  }

  if (bestT > 1.0) {
    return false;
  }

  result.hit = true;
  result.timeOfImpact = bestT;
  result.normal = targetTransform.rotation() * bestNormal;
  result.point = targetTransform * bestPoint;

  return true;
}

bool sphereCastConvex(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const ConvexShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  return sphereCastConvexTarget(
      sphereStart,
      sphereEnd,
      sphereRadius,
      [&](const Eigen::Vector3d& dir) {
        return targetTransform
               * target.support(targetTransform.rotation().transpose() * dir);
      },
      targetTransform.translation(),
      option,
      result);
}

bool sphereCastMesh(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const MeshShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  double bestT = 2.0;
  CcdResult bestResult;

  for (const auto& triangle : target.getTriangles()) {
    const ConvexShape triangleTarget = makeTriangleConvex(target, triangle);

    CcdResult localResult;
    if (sphereCastConvex(
            sphereStart,
            sphereEnd,
            sphereRadius,
            triangleTarget,
            targetTransform,
            option,
            localResult)) {
      updateBestCcdResult(localResult, bestT, bestResult);
    }
  }

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
}

namespace {

// Conservative advancement of a moving capsule against any static convex
// target, given the target's world-space support function. Uses the full
// capsule support (caps AND cylindrical body), so it catches side-of-capsule
// contacts that a caps-only endpoint sweep misses. `targetSeed` is a point
// on/near the target used to seed the first search direction.
template <typename TargetSupport>
bool capsuleCastConvexTarget(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const TargetSupport& targetSupport,
    const Eigen::Vector3d& targetSeed,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  double motionBound
      = computeCapsuleMotionBound(capsule, capsuleStart, capsuleEnd);
  if (motionBound < option.tolerance) {
    motionBound = option.tolerance;
  }

  double t = 0.0;

  Eigen::Vector3d initialDir = targetSeed - capsuleStart.translation();
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  const MotionSample capsuleMotion(capsuleStart, capsuleEnd);

  // Clamp to >= 1 so a zero/negative budget still tests the t = 0 overlap.
  for (int iter = 0; iter < std::max(1, option.maxIterations); ++iter) {
    const Eigen::Isometry3d currentCapsuleTransform = capsuleMotion.at(t);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return getCapsuleSupport(currentCapsuleTransform, capsule, dir);
    };

    GjkResult gjkResult = detail::queryT(supportA, targetSupport, initialDir);

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = targetSupport(Eigen::Vector3d::UnitX());
      Eigen::Vector3d normal
          = supportA(Eigen::Vector3d::UnitX()) - result.point;
      if (normal.squaredNorm() < kEpsilon) {
        normal = Eigen::Vector3d::UnitZ();
      } else {
        normal.normalize();
      }
      result.normal = normal;
      return true;
    }

    const Eigen::Vector3d pointA = gjkResult.closestPointA;
    const Eigen::Vector3d pointB = gjkResult.closestPointB;
    const double distance = gjkResult.distance;

    Eigen::Vector3d sepAxis = gjkResult.separationAxis;
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = pointB - pointA;
    }
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = Eigen::Vector3d::UnitX();
    }
    sepAxis.normalize();

    if (distance < option.tolerance) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = pointB;
      result.normal = -sepAxis;
      return true;
    }

    t += distance / motionBound;
    if (t > 1.0) {
      return false;
    }

    initialDir = sepAxis;
  }

  return false;
}

} // namespace

bool capsuleCastSphere(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const SphereShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  const Eigen::Vector3d center = targetTransform.translation();
  const double radius = target.getRadius();
  return capsuleCastConvexTarget(
      capsuleStart,
      capsuleEnd,
      capsule,
      [&](const Eigen::Vector3d& dir) {
        return sphereSupport(center, radius, dir);
      },
      center,
      option,
      result);
}

bool capsuleCastBox(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const BoxShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  const Eigen::Vector3d halfExtents = target.getHalfExtents();
  return capsuleCastConvexTarget(
      capsuleStart,
      capsuleEnd,
      capsule,
      [&](const Eigen::Vector3d& dir) {
        return boxSupport(targetTransform, halfExtents, dir);
      },
      targetTransform.translation(),
      option,
      result);
}

bool capsuleCastCapsule(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CapsuleShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  return capsuleCastConvexTarget(
      capsuleStart,
      capsuleEnd,
      capsule,
      [&](const Eigen::Vector3d& dir) {
        return getCapsuleSupport(targetTransform, target, dir);
      },
      targetTransform.translation(),
      option,
      result);
}

bool capsuleCastPlane(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const PlaneShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  const double capsuleRadius = capsule.getRadius();
  const double halfHeight = capsule.getHeight() / 2.0;
  const Eigen::Vector3d worldNormal
      = targetTransform.rotation() * target.getNormal();
  const Eigen::Vector3d worldPoint
      = targetTransform.translation() + target.getOffset() * worldNormal;

  double motionBound
      = computeCapsuleMotionBound(capsule, capsuleStart, capsuleEnd);
  if (motionBound < option.tolerance) {
    motionBound = option.tolerance;
  }

  const MotionSample capsuleMotion(capsuleStart, capsuleEnd);
  double t = 0.0;

  for (int iter = 0; iter < std::max(1, option.maxIterations); ++iter) {
    const Eigen::Isometry3d currentCapsuleTransform = capsuleMotion.at(t);
    const Eigen::Vector3d topEndpoint
        = getCapsuleEndpoint(currentCapsuleTransform, halfHeight, true);
    const Eigen::Vector3d bottomEndpoint
        = getCapsuleEndpoint(currentCapsuleTransform, halfHeight, false);

    const double topDistance = (topEndpoint - worldPoint).dot(worldNormal);
    const double bottomDistance
        = (bottomEndpoint - worldPoint).dot(worldNormal);
    const bool useTop = topDistance < bottomDistance;
    const Eigen::Vector3d closestEndpoint
        = useTop ? topEndpoint : bottomEndpoint;
    const double clearance
        = (useTop ? topDistance : bottomDistance) - capsuleRadius;

    if (clearance <= option.tolerance) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = closestEndpoint - capsuleRadius * worldNormal;
      result.normal = worldNormal;
      return true;
    }

    t += clearance / motionBound;
    if (t > 1.0) {
      return false;
    }
  }

  return false;
}

bool capsuleCastCylinder(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CylinderShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  const double radius = target.getRadius();
  const double halfHeight = target.getHeight() / 2.0;
  return capsuleCastConvexTarget(
      capsuleStart,
      capsuleEnd,
      capsule,
      [&](const Eigen::Vector3d& dir) {
        return cylinderSupport(targetTransform, radius, halfHeight, dir);
      },
      targetTransform.translation(),
      option,
      result);
}

bool capsuleCastConvex(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const ConvexShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  return capsuleCastConvexTarget(
      capsuleStart,
      capsuleEnd,
      capsule,
      [&](const Eigen::Vector3d& dir) {
        return targetTransform
               * target.support(targetTransform.rotation().transpose() * dir);
      },
      targetTransform.translation(),
      option,
      result);
}

bool capsuleCastMesh(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const MeshShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  const double capsuleRadius = capsule.getRadius();
  const double halfHeight = capsule.getHeight() / 2.0;

  double bestT = 2.0;
  CcdResult bestResult;

  auto testEndpoint = [&](bool top) {
    Eigen::Vector3d startPos
        = getCapsuleEndpoint(capsuleStart, halfHeight, top);
    Eigen::Vector3d endPos = getCapsuleEndpoint(capsuleEnd, halfHeight, top);

    CcdResult localResult;
    if (sphereCastMesh(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      updateBestCcdResult(localResult, bestT, bestResult);
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  for (const auto& triangle : target.getTriangles()) {
    const ConvexShape triangleTarget = makeTriangleConvex(target, triangle);

    CcdResult localResult;
    if (capsuleCastConvex(
            capsuleStart,
            capsuleEnd,
            capsule,
            triangleTarget,
            targetTransform,
            option,
            localResult)) {
      updateBestCcdResult(localResult, bestT, bestResult);
    }
  }

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
}

namespace {

// Precomputed rotational data for a shape's motion, used to evaluate a directed
// (closest-direction-aware) conservative-advancement bound each step. The
// rotation contributes to the closest-distance rate only through its component
// perpendicular to the closest direction n: (omega x r).n <= |omega| * r_perp *
// |n x axis|. So the angular term carries a |n x axis| factor that vanishes
// when the rotation axis is parallel to n (e.g. a coaxial screw, where the spin
// does not close the gap) and is maximal when the axis is perpendicular to n
// (an ordinary rotational sweep). This is a C2A-style directed bound (Tang/Kim/
// Manocha, ICRA 2009) restricted to the angular term; the linear term stays
// undirected (|v|) so translation remains strictly conservative.
struct RotationBoundData
{
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  double angle = 0.0;
  double maxPerpRadius = 0.0;

  [[nodiscard]] double angularBound(const Eigen::Vector3d& direction) const
  {
    if (angle < kEpsilon) {
      return 0.0;
    }
    return angle * maxPerpRadius * direction.cross(axis).norm();
  }
};

RotationBoundData makeRotationBoundData(
    const ConvexShape& shape,
    const Eigen::Isometry3d& transformStart,
    const Eigen::Isometry3d& transformEnd)
{
  RotationBoundData data;

  const Eigen::Quaterniond qStart(transformStart.rotation());
  const Eigen::Quaterniond qEnd(transformEnd.rotation());
  data.angle = qStart.angularDistance(qEnd);
  if (data.angle < kEpsilon) {
    return data;
  }

  data.axis = Eigen::AngleAxisd(qEnd * qStart.conjugate()).axis();
  const Eigen::Matrix3d rotStart = transformStart.rotation();

  // Perpendicular distance from the (world) rotation axis is invariant along
  // the sweep, so it is evaluated once at the start configuration.
  for (const auto& vertex : shape.getVertices()) {
    const Eigen::Vector3d p = rotStart * vertex;
    const double perp = (p - p.dot(data.axis) * data.axis).norm();
    data.maxPerpRadius = std::max(data.maxPerpRadius, perp);
  }
  return data;
}

// Samples a cubic-Bezier (spline) motion and supplies the ingredients for a
// closed-form conservative advancement step over a remaining interval [t, 1].
// Translation follows the cubic Bezier of the four translation control points;
// orientation is the exponential of the cubic Bezier of the four
// rotation-vector control points -- the same curve the reference spline-motion
// model traces.
//
// The step uses an acceleration-bounded speed model: |T'(t+s)| <= |T'(t)| +
// s*A, where A bounds |T''| over the step. This is far tighter than a flat
// max-speed bound when the curve is slow now but fast later (a curve whose tail
// accelerates away), turning the conservative-advancement step from
// distance/max_speed into the larger root of a quadratic -- fewer GJK
// evaluations to reach the time of impact, with no loss of conservativeness.
class SplineSample
{
public:
  SplineSample(
      const std::array<Eigen::Vector3d, 4>& translation,
      const std::array<Eigen::Vector3d, 4>& rotation)
    : translation_(translation),
      rotation_(rotation),
      rotates_(
          rotation[0].squaredNorm() + rotation[1].squaredNorm()
              + rotation[2].squaredNorm() + rotation[3].squaredNorm()
          > kEpsilon)
  {
    // Hodograph (derivative) control points: a cubic Bezier's derivative is a
    // quadratic Bezier with control points 3*(P_{i+1} - P_i).
    for (int i = 0; i < 3; ++i) {
      translationVel_[i] = 3.0 * (translation_[i + 1] - translation_[i]);
      rotationVel_[i] = 3.0 * (rotation_[i + 1] - rotation_[i]);
    }
    // T''(s) is linear in s: T''(s) = accelBase_ + s * accelSlope_.
    accelBase_ = 2.0 * (translationVel_[1] - translationVel_[0]);
    accelSlope_ = 2.0
                  * (translationVel_[0] - 2.0 * translationVel_[1]
                     + translationVel_[2]);
  }

  [[nodiscard]] Eigen::Isometry3d at(double t) const
  {
    const double u = 1.0 - t;
    const double b0 = u * u * u;
    const double b1 = 3.0 * t * u * u;
    const double b2 = 3.0 * t * t * u;
    const double b3 = t * t * t;

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = b0 * translation_[0] + b1 * translation_[1]
                           + b2 * translation_[2] + b3 * translation_[3];
    if (rotates_) {
      const Eigen::Vector3d w = b0 * rotation_[0] + b1 * rotation_[1]
                                + b2 * rotation_[2] + b3 * rotation_[3];
      const double angle = w.norm();
      if (angle > kEpsilon) {
        result.linear()
            = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
      }
    }
    return result;
  }

  [[nodiscard]] bool rotates() const
  {
    return rotates_;
  }

  // Translation alone at parameter t (skips the orientation work in at()).
  [[nodiscard]] Eigen::Vector3d translationAt(double t) const
  {
    const double u = 1.0 - t;
    return u * u * u * translation_[0] + 3.0 * t * u * u * translation_[1]
           + 3.0 * t * t * u * translation_[2] + t * t * t * translation_[3];
  }

  // Instantaneous translational speed |T'(t)| (hodograph evaluated at t).
  [[nodiscard]] double linearSpeedAt(double t) const
  {
    const double u = 1.0 - t;
    const Eigen::Vector3d v = u * u * translationVel_[0]
                              + 2.0 * u * t * translationVel_[1]
                              + t * t * translationVel_[2];
    return v.norm();
  }

  // Upper bound on |T''(s)| for s in [t, 1]. T'' is linear in s, and a linear
  // vector function's norm is convex, so its maximum over an interval is at an
  // endpoint.
  [[nodiscard]] double maxLinearAccel(double t) const
  {
    const double accelAtT = (accelBase_ + t * accelSlope_).norm();
    const double accelAtEnd = (accelBase_ + accelSlope_).norm();
    return std::max(accelAtT, accelAtEnd);
  }

  // Upper bound on angular speed |omega(s)| for s in [t, 1]. The body-frame
  // angular velocity is omega = J_r(W) W', and the SO(3) right Jacobian J_r is
  // a contraction (its singular values are at most 1), so |omega| <= |W'|.
  // Hence the hodograph bound on |W'| also bounds |omega|.
  [[nodiscard]] double maxAngularSpeed(double t) const
  {
    return rotates_ ? maxQuadraticBezierNorm(rotationVel_, t) : 0.0;
  }

  // Maximum forward translational displacement along direction n over [t, 1],
  // i.e. max_{s in [t,1]} (T(s) - T(t)) . n, evaluated in closed form at the
  // interval ends and the interior extrema of the projected curve. This is the
  // largest distance the body can travel toward the obstacle over the rest of
  // the motion. Stepping distance/displacement is the (non-conservative) fast
  // advancement: it can stride past a curved first contact and converge on a
  // later one, so it is only used when CcdAdvancement::Fast is requested.
  [[nodiscard]] double maxForwardDisplacement(
      double t, const Eigen::Vector3d& n) const
  {
    const double v0 = translationVel_[0].dot(n);
    const double v1 = translationVel_[1].dot(n);
    const double v2 = translationVel_[2].dot(n);

    const Eigen::Vector3d base = translationAt(t);
    double best = 0.0;
    const auto consider = [&](double s) {
      if (s > t && s < 1.0) {
        best = std::max(best, (translationAt(s) - base).dot(n));
      }
    };
    best = std::max(best, (translationAt(1.0) - base).dot(n));

    // Interior extrema solve T'(s).n = 0, a quadratic in monomial form.
    const double a = v0 - 2.0 * v1 + v2;
    const double b = 2.0 * (v1 - v0);
    const double c = v0;
    if (std::abs(a) < kEpsilon) {
      if (std::abs(b) > kEpsilon) {
        consider(-c / b);
      }
    } else {
      const double disc = b * b - 4.0 * a * c;
      if (disc >= 0.0) {
        const double sd = std::sqrt(disc);
        consider((-b + sd) / (2.0 * a));
        consider((-b - sd) / (2.0 * a));
      }
    }
    return best;
  }

private:
  // Upper bound on the Euclidean norm of a quadratic Bezier (control points v)
  // over [t, 1]. de Casteljau subdivision at t yields the right sub-curve's
  // control points; the curve lies in their convex hull, so the largest control
  // point norm bounds the curve norm on the remaining interval.
  static double maxQuadraticBezierNorm(
      const std::array<Eigen::Vector3d, 3>& v, double t)
  {
    const Eigen::Vector3d a = v[0] + t * (v[1] - v[0]);
    const Eigen::Vector3d b = v[1] + t * (v[2] - v[1]);
    const Eigen::Vector3d r0 = a + t * (b - a); // curve value at t
    return std::max({r0.norm(), b.norm(), v[2].norm()});
  }

  std::array<Eigen::Vector3d, 4> translation_;
  std::array<Eigen::Vector3d, 4> rotation_;
  std::array<Eigen::Vector3d, 3> translationVel_;
  std::array<Eigen::Vector3d, 3> rotationVel_;
  Eigen::Vector3d accelBase_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelSlope_ = Eigen::Vector3d::Zero();
  bool rotates_;
};

} // namespace

bool conservativeAdvancement(
    const ConvexShape& shapeA,
    const Eigen::Isometry3d& transformAStart,
    const Eigen::Isometry3d& transformAEnd,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformB,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  // Directed conservative-advancement bound (recomputed each step from the
  // closest direction): the linear term stays undirected; only the rotational
  // term is projected via |n x axis|.
  const double linearBound
      = (transformAEnd.translation() - transformAStart.translation()).norm();
  const RotationBoundData rotationA
      = makeRotationBoundData(shapeA, transformAStart, transformAEnd);

  double t = 0.0;

  Eigen::Vector3d initialDir
      = transformB.translation() - transformAStart.translation();
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  const MotionSample motionA(transformAStart, transformAEnd);

  GjkSimplex warmSimplex;
  bool haveWarm = false;

  // Clamp to >= 1 so a zero/negative budget still tests the t = 0 overlap.
  for (int iter = 0; iter < std::max(1, option.maxIterations); ++iter) {
    Eigen::Isometry3d currentTransformA = motionA.at(t);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return currentTransformA
             * shapeA.support(currentTransformA.rotation().transpose() * dir);
    };
    auto supportB = [&](const Eigen::Vector3d& dir) {
      return transformB
             * shapeB.support(transformB.rotation().transpose() * dir);
    };

    // Warm-start GJK from the previous step's simplex (see convexCast).
    GjkResult gjkResult
        = haveWarm ? detail::queryT(supportA, supportB, warmSimplex, initialDir)
                   : detail::queryT(supportA, supportB, initialDir);
    warmSimplex = gjkResult.simplex;
    haveWarm = true;

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      Eigen::Vector3d pointA = supportA(Eigen::Vector3d::UnitX());
      Eigen::Vector3d pointB = supportB(-Eigen::Vector3d::UnitX());
      result.point = pointB;
      // Check the magnitude before normalizing: coincident support points
      // (degenerate/overlapping configs) would otherwise yield a NaN normal
      // that slips past a post-normalization guard (NaN < kEpsilon is false).
      const Eigen::Vector3d normal = pointA - pointB;
      if (normal.squaredNorm() < kEpsilon) {
        result.normal = Eigen::Vector3d::UnitZ();
      } else {
        result.normal = normal.normalized();
      }
      return true;
    }

    Eigen::Vector3d pointA = gjkResult.closestPointA;
    Eigen::Vector3d pointB = gjkResult.closestPointB;
    double distance = gjkResult.distance;

    Eigen::Vector3d sepAxis = gjkResult.separationAxis;
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = pointB - pointA;
    }
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = Eigen::Vector3d::UnitX();
    }
    sepAxis.normalize();

    if (distance < option.tolerance) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = pointB;
      result.normal = -sepAxis;
      return true;
    }

    double motionBound = linearBound + rotationA.angularBound(sepAxis);
    if (motionBound < option.tolerance) {
      motionBound = option.tolerance;
    }
    double dt = distance / motionBound;
    t += dt;

    if (t > 1.0) {
      return false;
    }

    initialDir = sepAxis;
  }

  return false;
}

bool convexCast(
    const ConvexShape& shapeA,
    const Eigen::Isometry3d& transformAStart,
    const Eigen::Isometry3d& transformAEnd,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformBStart,
    const Eigen::Isometry3d& transformBEnd,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  // Directed conservative-advancement bound, recomputed each step from the
  // closest direction. Linear term uses the relative translation (undirected);
  // each body's rotational term is projected via |n x axis|.
  const Eigen::Vector3d relativeLinearVel
      = (transformAEnd.translation() - transformAStart.translation())
        - (transformBEnd.translation() - transformBStart.translation());
  const double linearBound = relativeLinearVel.norm();
  const RotationBoundData rotationA
      = makeRotationBoundData(shapeA, transformAStart, transformAEnd);
  const RotationBoundData rotationB
      = makeRotationBoundData(shapeB, transformBStart, transformBEnd);

  double t = 0.0;

  Eigen::Vector3d initialDir
      = transformBStart.translation() - transformAStart.translation();
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  const MotionSample motionA(transformAStart, transformAEnd);
  const MotionSample motionB(transformBStart, transformBEnd);

  GjkSimplex warmSimplex;
  bool haveWarm = false;

  // Clamp to >= 1 so a zero/negative budget still tests the t = 0 overlap.
  for (int iter = 0; iter < std::max(1, option.maxIterations); ++iter) {
    const Eigen::Isometry3d currentTransformA = motionA.at(t);
    const Eigen::Isometry3d currentTransformB = motionB.at(t);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return currentTransformA
             * shapeA.support(currentTransformA.rotation().transpose() * dir);
    };
    auto supportB = [&](const Eigen::Vector3d& dir) {
      return currentTransformB
             * shapeB.support(currentTransformB.rotation().transpose() * dir);
    };

    // Warm-start GJK from the previous step's simplex: consecutive
    // conservative- advancement configurations are nearly identical, so the
    // prior search directions seed the closest-point query close to its answer.
    // The warm-start recomputes supports at the current configuration, so the
    // result is identical -- only the iteration count drops.
    GjkResult gjkResult
        = haveWarm ? detail::queryT(supportA, supportB, warmSimplex, initialDir)
                   : detail::queryT(supportA, supportB, initialDir);
    warmSimplex = gjkResult.simplex;
    haveWarm = true;

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      Eigen::Vector3d pointA = supportA(Eigen::Vector3d::UnitX());
      Eigen::Vector3d pointB = supportB(-Eigen::Vector3d::UnitX());
      result.point = pointB;
      // Check the magnitude before normalizing: coincident support points
      // (degenerate/overlapping configs) would otherwise yield a NaN normal
      // that slips past a post-normalization guard (NaN < kEpsilon is false).
      const Eigen::Vector3d normal = pointA - pointB;
      if (normal.squaredNorm() < kEpsilon) {
        result.normal = Eigen::Vector3d::UnitZ();
      } else {
        result.normal = normal.normalized();
      }
      return true;
    }

    Eigen::Vector3d pointA = gjkResult.closestPointA;
    Eigen::Vector3d pointB = gjkResult.closestPointB;
    double distance = gjkResult.distance;

    Eigen::Vector3d sepAxis = gjkResult.separationAxis;
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = pointB - pointA;
    }
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = Eigen::Vector3d::UnitX();
    }
    sepAxis.normalize();

    if (distance < option.tolerance) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = pointB;
      result.normal = -sepAxis;
      return true;
    }

    double motionBound = linearBound + rotationA.angularBound(sepAxis)
                         + rotationB.angularBound(sepAxis);
    if (motionBound < option.tolerance) {
      motionBound = option.tolerance;
    }
    t += distance / motionBound;
    if (t > 1.0) {
      return false;
    }

    initialDir = sepAxis;
  }

  return false;
}

bool splineCast(
    const ConvexShape& shapeA,
    const std::array<Eigen::Vector3d, 4>& translationControlPoints,
    const std::array<Eigen::Vector3d, 4>& rotationControlPoints,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformB,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  const SplineSample motionA(translationControlPoints, rotationControlPoints);

  // Rotation pivots about shape A's local origin, so the swept radius that
  // converts angular speed to point speed is the farthest vertex from it.
  double maxRadius = 0.0;
  if (motionA.rotates()) {
    for (const auto& vertex : shapeA.getVertices()) {
      maxRadius = std::max(maxRadius, vertex.norm());
    }
  }

  double t = 0.0;

  Eigen::Vector3d initialDir
      = transformB.translation() - translationControlPoints[0];
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  // Shape A spins only when its rotation control points are non-trivial; shape
  // B is static. When a body has no rotation, its support reduces to a vertex
  // scan plus a translation, skipping two matrix-vector products per support
  // call -- a large saving since the conservative-advancement loop evaluates
  // supports many times.
  const bool aRotates = motionA.rotates();
  const Eigen::Vector3d bTranslation = transformB.translation();
  const Eigen::Matrix3d bRotation = transformB.rotation();
  const bool bRotates = !bRotation.isIdentity(kEpsilon);
  const Eigen::Matrix3d bRotationT = bRotation.transpose();

  auto supportB = [&](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    if (bRotates) {
      return bTranslation + bRotation * shapeB.support(bRotationT * dir);
    }
    return bTranslation + shapeB.support(dir);
  };

  GjkSimplex warmSimplex;
  bool haveWarm = false;

  // Clamp to >= 1 so a zero/negative budget still tests the t = 0 overlap.
  for (int iter = 0; iter < std::max(1, option.maxIterations); ++iter) {
    const Eigen::Vector3d currentTranslationA = motionA.translationAt(t);
    Eigen::Matrix3d currentRotationA = Eigen::Matrix3d::Identity();
    if (aRotates) {
      currentRotationA = motionA.at(t).rotation();
    }

    auto supportA = [&](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
      if (aRotates) {
        return currentTranslationA
               + currentRotationA
                     * shapeA.support(currentRotationA.transpose() * dir);
      }
      return currentTranslationA + shapeA.support(dir);
    };

    GjkResult gjkResult
        = haveWarm ? detail::queryT(supportA, supportB, warmSimplex, initialDir)
                   : detail::queryT(supportA, supportB, initialDir);
    warmSimplex = gjkResult.simplex;
    haveWarm = true;

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      Eigen::Vector3d pointA = supportA(Eigen::Vector3d::UnitX());
      Eigen::Vector3d pointB = supportB(-Eigen::Vector3d::UnitX());
      result.point = pointB;
      // Check the magnitude before normalizing: coincident support points
      // (degenerate/overlapping configs) would otherwise yield a NaN normal
      // that slips past a post-normalization guard (NaN < kEpsilon is false).
      const Eigen::Vector3d normal = pointA - pointB;
      if (normal.squaredNorm() < kEpsilon) {
        result.normal = Eigen::Vector3d::UnitZ();
      } else {
        result.normal = normal.normalized();
      }
      return true;
    }

    const double distance = gjkResult.distance;

    Eigen::Vector3d sepAxis = gjkResult.separationAxis;
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = gjkResult.closestPointB - gjkResult.closestPointA;
    }
    if (sepAxis.squaredNorm() < kEpsilon) {
      sepAxis = Eigen::Vector3d::UnitX();
    }
    sepAxis.normalize();

    if (distance < option.tolerance) {
      result.hit = true;
      result.timeOfImpact = t;
      result.point = gjkResult.closestPointB;
      result.normal = -sepAxis;
      return true;
    }

    double dt;
    if (option.advancement == CcdAdvancement::Fast) {
      // Fast (non-conservative) step: divide the gap by the maximum forward
      // displacement over the rest of the motion. This strides much closer to
      // the obstacle per iteration but can overshoot a curved first contact and
      // converge on a later one -- the speed-for-tunnelling trade the caller
      // opts into.
      Eigen::Vector3d toward
          = gjkResult.closestPointB - gjkResult.closestPointA;
      if (toward.squaredNorm() < kEpsilon) {
        toward = sepAxis;
      }
      toward.normalize();
      const double disp = motionA.maxForwardDisplacement(t, toward)
                          + motionA.maxAngularSpeed(t) * (1.0 - t) * maxRadius;
      dt = distance / std::max(disp, option.tolerance);
    } else {
      // Acceleration-bounded conservative step. Over [t, t+dt] the point speed
      // is at most v0 + s*accel (translation, linearized by its
      // second-derivative bound) plus a constant angular contribution, so the
      // closing of the gap is at most (v0 + angular)*dt + accel*dt^2/2. Solving
      // that quadratic for the dt that closes exactly `distance` gives the
      // largest provably safe step -- tighter than distance/max_speed because
      // it does not assume the curve is already moving at its peak speed.
      const double v0 = motionA.linearSpeedAt(t);
      const double angular = motionA.maxAngularSpeed(t) * maxRadius;
      const double accel = motionA.maxLinearAccel(t);
      const double linearTerm = v0 + angular;

      if (accel < option.tolerance) {
        dt = distance / std::max(linearTerm, option.tolerance);
      } else {
        // Larger root of (accel/2) dt^2 + linearTerm dt - distance = 0.
        dt = (-linearTerm
              + std::sqrt(linearTerm * linearTerm + 2.0 * accel * distance))
             / accel;
      }
    }
    if (dt < kEpsilon) {
      dt = kEpsilon;
    }
    t += dt;
    if (t > 1.0) {
      return false;
    }

    initialDir = sepAxis;
  }

  return false;
}

} // namespace dart::collision::native
