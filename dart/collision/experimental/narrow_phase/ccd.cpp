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

#include <dart/collision/experimental/narrow_phase/ccd.hpp>
#include <dart/collision/experimental/narrow_phase/gjk.hpp>

#include <dart/common/Macros.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

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
  DART_UNUSED(option);
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
    result.normal = (sphereStart - targetCenter).normalized();
    if (result.normal.squaredNorm() < kEpsilon) {
      result.normal = Eigen::Vector3d::UnitZ();
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
  DART_UNUSED(option);
  result.clear();

  const Eigen::Isometry3d invTransform = targetTransform.inverse();
  const Eigen::Vector3d localStart = invTransform * sphereStart;
  const Eigen::Vector3d localEnd = invTransform * sphereEnd;
  const Eigen::Vector3d localDir = localEnd - localStart;

  const Eigen::Vector3d& halfExtents = target.getHalfExtents();
  const Eigen::Vector3d expandedHalf
      = halfExtents + Eigen::Vector3d::Constant(sphereRadius);

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

  if (tMin < 0.0) {
    result.hit = true;
    result.timeOfImpact = 0.0;
    result.point = sphereStart;

    Eigen::Vector3d closestOnBox;
    for (int i = 0; i < 3; ++i) {
      closestOnBox[i]
          = std::clamp(localStart[i], -halfExtents[i], halfExtents[i]);
    }
    Eigen::Vector3d localNormal = localStart - closestOnBox;
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
  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  localNormal[hitAxis] = static_cast<double>(hitSign);

  Eigen::Vector3d closestOnBox;
  for (int i = 0; i < 3; ++i) {
    closestOnBox[i] = std::clamp(hitCenter[i], -halfExtents[i], halfExtents[i]);
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
  DART_UNUSED(option);
  result.clear();

  const double capsuleRadius = target.getRadius();
  const double halfHeight = target.getHeight() / 2.0;
  const double combinedRadius = sphereRadius + capsuleRadius;

  const Eigen::Isometry3d invTransform = targetTransform.inverse();
  const Eigen::Vector3d localStart = invTransform * sphereStart;
  const Eigen::Vector3d localEnd = invTransform * sphereEnd;
  const Eigen::Vector3d localDir = localEnd - localStart;

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
      bestNormal = (hitCenter - capCenter).normalized();
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
  DART_UNUSED(option);
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
  DART_UNUSED(option);
  result.clear();

  const double cylinderRadius = target.getRadius();
  const double halfHeight = target.getHeight() / 2.0;
  const double combinedRadius = sphereRadius + cylinderRadius;

  const Eigen::Isometry3d invTransform = targetTransform.inverse();
  const Eigen::Vector3d localStart = invTransform * sphereStart;
  const Eigen::Vector3d localEnd = invTransform * sphereEnd;
  const Eigen::Vector3d localDir = localEnd - localStart;

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
  result.clear();

  SphereShape sphere(sphereRadius);
  ConvexShape expandedTarget = target;

  Eigen::Isometry3d sphereTransformStart = Eigen::Isometry3d::Identity();
  sphereTransformStart.translation() = sphereStart;
  Eigen::Isometry3d sphereTransformEnd = Eigen::Isometry3d::Identity();
  sphereTransformEnd.translation() = sphereEnd;

  double motionBound = (sphereEnd - sphereStart).norm();
  if (motionBound < option.tolerance) {
    motionBound = option.tolerance;
  }

  double t = 0.0;

  Eigen::Vector3d initialDir = targetTransform.translation() - sphereStart;
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  for (int iter = 0; iter < option.maxIterations; ++iter) {
    Eigen::Vector3d currentCenter = sphereStart + t * (sphereEnd - sphereStart);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return currentCenter + sphereRadius * dir.normalized();
    };
    auto supportB = [&](const Eigen::Vector3d& dir) {
      return targetTransform
             * target.support(targetTransform.rotation().transpose() * dir);
    };

    GjkResult gjkResult = Gjk::query(supportA, supportB, initialDir);

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      Eigen::Vector3d pointB = supportB(Eigen::Vector3d::UnitX());
      result.point = pointB;
      result.normal = (currentCenter - pointB).normalized();
      if (result.normal.squaredNorm() < kEpsilon) {
        result.normal = Eigen::Vector3d::UnitZ();
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
      result.normal = sepAxis;
      return true;
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

  double motionBound = (sphereEnd - sphereStart).norm();
  if (motionBound < option.tolerance) {
    motionBound = option.tolerance;
  }

  double t = 0.0;

  Eigen::Vector3d initialDir = targetTransform.translation() - sphereStart;
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  for (int iter = 0; iter < option.maxIterations; ++iter) {
    Eigen::Vector3d currentCenter = sphereStart + t * (sphereEnd - sphereStart);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return currentCenter + sphereRadius * dir.normalized();
    };
    auto supportB = [&](const Eigen::Vector3d& dir) {
      return targetTransform
             * target.support(targetTransform.rotation().transpose() * dir);
    };

    GjkResult gjkResult = Gjk::query(supportA, supportB, initialDir);

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      Eigen::Vector3d pointB = supportB(Eigen::Vector3d::UnitX());
      result.point = pointB;
      result.normal = (currentCenter - pointB).normalized();
      if (result.normal.squaredNorm() < kEpsilon) {
        result.normal = Eigen::Vector3d::UnitZ();
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
      result.normal = sepAxis;
      return true;
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

namespace {

Eigen::Vector3d getCapsuleEndpoint(
    const Eigen::Isometry3d& transform, double halfHeight, bool top)
{
  Eigen::Vector3d localPoint(0, 0, top ? halfHeight : -halfHeight);
  return transform * localPoint;
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
    if (sphereCastSphere(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
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
    if (sphereCastBox(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
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
    if (sphereCastCapsule(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
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

  double bestT = 2.0;
  CcdResult bestResult;

  auto testEndpoint = [&](bool top) {
    Eigen::Vector3d startPos
        = getCapsuleEndpoint(capsuleStart, halfHeight, top);
    Eigen::Vector3d endPos = getCapsuleEndpoint(capsuleEnd, halfHeight, top);

    CcdResult localResult;
    if (sphereCastPlane(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
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
    if (sphereCastCylinder(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
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
    if (sphereCastConvex(
            startPos,
            endPos,
            capsuleRadius,
            target,
            targetTransform,
            option,
            localResult)) {
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
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
      if (localResult.timeOfImpact < bestT) {
        bestT = localResult.timeOfImpact;
        bestResult = localResult;
      }
    }
  };

  testEndpoint(true);
  testEndpoint(false);

  if (bestT > 1.0) {
    return false;
  }

  result = bestResult;
  return true;
}

namespace {

Eigen::Isometry3d interpolateTransform(
    const Eigen::Isometry3d& start, const Eigen::Isometry3d& end, double t)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation()
      = (1.0 - t) * start.translation() + t * end.translation();

  Eigen::Quaterniond qStart(start.rotation());
  Eigen::Quaterniond qEnd(end.rotation());
  result.linear() = qStart.slerp(t, qEnd).toRotationMatrix();

  return result;
}

double computeMotionBound(
    const ConvexShape& shape,
    const Eigen::Isometry3d& transformStart,
    const Eigen::Isometry3d& transformEnd)
{
  double linearVelocity
      = (transformEnd.translation() - transformStart.translation()).norm();

  Eigen::Quaterniond qStart(transformStart.rotation());
  Eigen::Quaterniond qEnd(transformEnd.rotation());
  double angle = qStart.angularDistance(qEnd);

  double maxRadius = 0.0;
  for (const auto& vertex : shape.getVertices()) {
    maxRadius = std::max(maxRadius, vertex.norm());
  }

  double angularContribution = angle * maxRadius;

  return linearVelocity + angularContribution;
}

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

  double motionBound
      = computeMotionBound(shapeA, transformAStart, transformAEnd);
  if (motionBound < option.tolerance) {
    motionBound = option.tolerance;
  }

  double t = 0.0;

  Eigen::Vector3d initialDir
      = transformB.translation() - transformAStart.translation();
  if (initialDir.squaredNorm() < kEpsilon) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  for (int iter = 0; iter < option.maxIterations; ++iter) {
    Eigen::Isometry3d currentTransformA
        = interpolateTransform(transformAStart, transformAEnd, t);

    auto supportA = [&](const Eigen::Vector3d& dir) {
      return currentTransformA
             * shapeA.support(currentTransformA.rotation().transpose() * dir);
    };
    auto supportB = [&](const Eigen::Vector3d& dir) {
      return transformB
             * shapeB.support(transformB.rotation().transpose() * dir);
    };

    GjkResult gjkResult = Gjk::query(supportA, supportB, initialDir);

    if (gjkResult.intersecting) {
      result.hit = true;
      result.timeOfImpact = t;
      Eigen::Vector3d pointA = supportA(Eigen::Vector3d::UnitX());
      Eigen::Vector3d pointB = supportB(-Eigen::Vector3d::UnitX());
      result.point = pointB;
      result.normal = (pointA - pointB).normalized();
      if (result.normal.squaredNorm() < kEpsilon) {
        result.normal = Eigen::Vector3d::UnitZ();
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
      result.normal = sepAxis;
      return true;
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

} // namespace dart::collision::experimental
