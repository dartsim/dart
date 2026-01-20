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

#include <dart/collision/experimental/narrow_phase/gjk.hpp>
#include <dart/collision/experimental/narrow_phase/raycast.hpp>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::collision::experimental {

namespace {

constexpr double kEpsilon = 1e-10;

double solveQuadratic(double a, double b, double c, double& t0, double& t1)
{
  double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return -1.0;
  }

  double sqrtDisc = std::sqrt(discriminant);
  double q = (b > 0.0) ? -0.5 * (b + sqrtDisc) : -0.5 * (b - sqrtDisc);

  t0 = q / a;
  t1 = c / q;

  if (t0 > t1) {
    std::swap(t0, t1);
  }

  return discriminant;
}

} // namespace

bool raycastSphere(
    const Ray& ray,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const Eigen::Vector3d center = sphereTransform.translation();
  const double radius = sphere.getRadius();

  const Eigen::Vector3d oc = ray.origin - center;

  const double a = ray.direction.squaredNorm();
  const double b = 2.0 * oc.dot(ray.direction);
  const double c = oc.squaredNorm() - radius * radius;

  double t0, t1;
  if (solveQuadratic(a, b, c, t0, t1) < 0.0) {
    return false;
  }

  double t = t0;
  if (t < 0.0) {
    t = t1;
    if (t < 0.0) {
      return false;
    }
  }

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  if (t > maxDist) {
    return false;
  }

  result.hit = true;
  result.distance = t;
  result.point = ray.pointAt(t);
  result.normal = (result.point - center).normalized();

  return true;
}

bool raycastBox(
    const Ray& ray,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const Eigen::Isometry3d invTransform = boxTransform.inverse();
  const Eigen::Vector3d localOrigin = invTransform * ray.origin;
  const Eigen::Vector3d localDir = invTransform.rotation() * ray.direction;

  const Eigen::Vector3d& halfExtents = box.getHalfExtents();

  double tMin = 0.0;
  double tMax = std::numeric_limits<double>::max();
  int hitAxis = -1;
  int hitSign = 1;

  for (int i = 0; i < 3; ++i) {
    if (std::abs(localDir[i]) < kEpsilon) {
      if (localOrigin[i] < -halfExtents[i] || localOrigin[i] > halfExtents[i]) {
        return false;
      }
    } else {
      double invD = 1.0 / localDir[i];
      double t1 = (-halfExtents[i] - localOrigin[i]) * invD;
      double t2 = (halfExtents[i] - localOrigin[i]) * invD;

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
    return false;
  }

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  if (tMin > maxDist) {
    return false;
  }

  result.hit = true;
  result.distance = tMin;
  result.point = ray.pointAt(tMin);

  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  localNormal[hitAxis] = static_cast<double>(hitSign);
  result.normal = boxTransform.rotation() * localNormal;

  return true;
}

bool raycastCapsule(
    const Ray& ray,
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const double radius = capsule.getRadius();
  const double halfHeight = capsule.getHeight() / 2.0;

  const Eigen::Isometry3d invTransform = capsuleTransform.inverse();
  const Eigen::Vector3d localOrigin = invTransform * ray.origin;
  const Eigen::Vector3d localDir = invTransform.rotation() * ray.direction;

  double bestT = std::numeric_limits<double>::max();
  Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();

  const double dx = localDir.x();
  const double dy = localDir.y();
  const double ox = localOrigin.x();
  const double oy = localOrigin.y();

  double a = dx * dx + dy * dy;
  double b = 2.0 * (ox * dx + oy * dy);
  double c = ox * ox + oy * oy - radius * radius;

  if (a > kEpsilon) {
    double t0, t1;
    if (solveQuadratic(a, b, c, t0, t1) >= 0.0) {
      for (double t : {t0, t1}) {
        if (t >= 0.0 && t < bestT) {
          double z = localOrigin.z() + t * localDir.z();
          if (z >= -halfHeight && z <= halfHeight) {
            Eigen::Vector3d hitPoint = localOrigin + t * localDir;
            Eigen::Vector3d normal(hitPoint.x(), hitPoint.y(), 0.0);
            if (normal.squaredNorm() > kEpsilon) {
              bestT = t;
              bestNormal = normal.normalized();
            }
          }
        }
      }
    }
  }

  auto testSphereCap = [&](const Eigen::Vector3d& center) {
    Eigen::Vector3d oc = localOrigin - center;
    double a2 = localDir.squaredNorm();
    double b2 = 2.0 * oc.dot(localDir);
    double c2 = oc.squaredNorm() - radius * radius;

    double t0, t1;
    if (solveQuadratic(a2, b2, c2, t0, t1) >= 0.0) {
      for (double t : {t0, t1}) {
        if (t >= 0.0 && t < bestT) {
          bestT = t;
          Eigen::Vector3d hitPoint = localOrigin + t * localDir;
          bestNormal = (hitPoint - center).normalized();
        }
      }
    }
  };

  testSphereCap(Eigen::Vector3d(0, 0, halfHeight));
  testSphereCap(Eigen::Vector3d(0, 0, -halfHeight));

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  if (bestT > maxDist || bestT == std::numeric_limits<double>::max()) {
    return false;
  }

  result.hit = true;
  result.distance = bestT;
  result.point = ray.pointAt(bestT);
  result.normal = capsuleTransform.rotation() * bestNormal;

  return true;
}

bool raycastCylinder(
    const Ray& ray,
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const double radius = cylinder.getRadius();
  const double halfHeight = cylinder.getHeight() / 2.0;

  const Eigen::Isometry3d invTransform = cylinderTransform.inverse();
  const Eigen::Vector3d localOrigin = invTransform * ray.origin;
  const Eigen::Vector3d localDir = invTransform.rotation() * ray.direction;

  double bestT = std::numeric_limits<double>::max();
  Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();

  const double dx = localDir.x();
  const double dy = localDir.y();
  const double ox = localOrigin.x();
  const double oy = localOrigin.y();

  double a = dx * dx + dy * dy;
  double b = 2.0 * (ox * dx + oy * dy);
  double c = ox * ox + oy * oy - radius * radius;

  if (a > kEpsilon) {
    double t0, t1;
    if (solveQuadratic(a, b, c, t0, t1) >= 0.0) {
      for (double t : {t0, t1}) {
        if (t >= 0.0 && t < bestT) {
          double z = localOrigin.z() + t * localDir.z();
          if (z >= -halfHeight && z <= halfHeight) {
            Eigen::Vector3d hitPoint = localOrigin + t * localDir;
            Eigen::Vector3d normal(hitPoint.x(), hitPoint.y(), 0.0);
            if (normal.squaredNorm() > kEpsilon) {
              bestT = t;
              bestNormal = normal.normalized();
            }
          }
        }
      }
    }
  }

  if (std::abs(localDir.z()) > kEpsilon) {
    for (double capZ : {-halfHeight, halfHeight}) {
      double t = (capZ - localOrigin.z()) / localDir.z();
      if (t >= 0.0 && t < bestT) {
        Eigen::Vector3d hitPoint = localOrigin + t * localDir;
        double distSq
            = hitPoint.x() * hitPoint.x() + hitPoint.y() * hitPoint.y();
        if (distSq <= radius * radius) {
          bestT = t;
          bestNormal = (capZ > 0.0) ? Eigen::Vector3d(0, 0, 1)
                                    : Eigen::Vector3d(0, 0, -1);
        }
      }
    }
  }

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  if (bestT > maxDist || bestT == std::numeric_limits<double>::max()) {
    return false;
  }

  result.hit = true;
  result.distance = bestT;
  result.point = ray.pointAt(bestT);
  result.normal = cylinderTransform.rotation() * bestNormal;

  return true;
}

bool raycastPlane(
    const Ray& ray,
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d worldPoint
      = planeTransform.translation() + plane.getOffset() * worldNormal;

  double denom = worldNormal.dot(ray.direction);

  if (option.backfaceCulling && denom > -kEpsilon) {
    return false;
  }

  if (std::abs(denom) < kEpsilon) {
    return false;
  }

  double t = (worldPoint - ray.origin).dot(worldNormal) / denom;

  if (t < 0.0) {
    return false;
  }

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  if (t > maxDist) {
    return false;
  }

  result.hit = true;
  result.distance = t;
  result.point = ray.pointAt(t);
  result.normal = (denom < 0.0) ? worldNormal : -worldNormal;

  return true;
}

namespace {

constexpr double kMeshEpsilon = 1e-10;

bool rayTriangleMollerTrumbore(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& dir,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    double& t,
    double& u,
    double& v,
    bool backfaceCulling)
{
  const Eigen::Vector3d edge1 = v1 - v0;
  const Eigen::Vector3d edge2 = v2 - v0;

  const Eigen::Vector3d h = dir.cross(edge2);
  const double a = edge1.dot(h);

  if (backfaceCulling) {
    if (a < kMeshEpsilon) {
      return false;
    }
  } else {
    if (std::abs(a) < kMeshEpsilon) {
      return false;
    }
  }

  const double f = 1.0 / a;
  const Eigen::Vector3d s = origin - v0;
  u = f * s.dot(h);

  if (u < 0.0 || u > 1.0) {
    return false;
  }

  const Eigen::Vector3d q = s.cross(edge1);
  v = f * dir.dot(q);

  if (v < 0.0 || u + v > 1.0) {
    return false;
  }

  t = f * edge2.dot(q);

  return t > kMeshEpsilon;
}

} // namespace

bool raycastMesh(
    const Ray& ray,
    const MeshShape& mesh,
    const Eigen::Isometry3d& meshTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const Eigen::Isometry3d invTransform = meshTransform.inverse();
  const Eigen::Vector3d localOrigin = invTransform * ray.origin;
  const Eigen::Vector3d localDir = invTransform.rotation() * ray.direction;

  const auto& vertices = mesh.getVertices();
  const auto& triangles = mesh.getTriangles();

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  double bestT = std::numeric_limits<double>::max();
  Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();

  for (const auto& tri : triangles) {
    const Eigen::Vector3d& v0 = vertices[static_cast<std::size_t>(tri[0])];
    const Eigen::Vector3d& v1 = vertices[static_cast<std::size_t>(tri[1])];
    const Eigen::Vector3d& v2 = vertices[static_cast<std::size_t>(tri[2])];

    double t, u, v;
    if (rayTriangleMollerTrumbore(
            localOrigin,
            localDir,
            v0,
            v1,
            v2,
            t,
            u,
            v,
            option.backfaceCulling)) {
      if (t > 0.0 && t < bestT && t <= maxDist) {
        bestT = t;
        const Eigen::Vector3d edge1 = v1 - v0;
        const Eigen::Vector3d edge2 = v2 - v0;
        bestNormal = edge1.cross(edge2).normalized();
      }
    }
  }

  if (bestT >= std::numeric_limits<double>::max() || bestT > maxDist) {
    return false;
  }

  result.hit = true;
  result.distance = bestT;
  result.point = ray.pointAt(bestT);
  result.normal = meshTransform.rotation() * bestNormal;

  if (result.normal.dot(ray.direction) > 0.0) {
    result.normal = -result.normal;
  }

  return true;
}

namespace {

bool isPointInConvex(
    const Eigen::Vector3d& point,
    const ConvexShape& convex,
    const Eigen::Isometry3d& transform)
{
  auto pointSupport = [&point](const Eigen::Vector3d&) {
    return point;
  };

  auto convexSupport = [&convex, transform](const Eigen::Vector3d& dir) {
    Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    return transform * convex.support(localDir);
  };

  Eigen::Vector3d initialDir = point - transform.translation();
  if (initialDir.squaredNorm() < 1e-10) {
    initialDir = Eigen::Vector3d::UnitX();
  }

  return Gjk::intersect(pointSupport, convexSupport, initialDir);
}

double computeBoundingRadius(const ConvexShape& convex)
{
  double maxRadiusSq = 0.0;
  for (const auto& v : convex.getVertices()) {
    maxRadiusSq = std::max(maxRadiusSq, v.squaredNorm());
  }
  return std::sqrt(maxRadiusSq);
}

} // namespace

bool raycastConvex(
    const Ray& ray,
    const ConvexShape& convex,
    const Eigen::Isometry3d& convexTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const double maxDist = std::min(ray.maxDistance, option.maxDistance);

  if (isPointInConvex(ray.origin, convex, convexTransform)) {
    result.hit = true;
    result.distance = 0.0;
    result.point = ray.origin;
    result.normal = -ray.direction.normalized();
    return true;
  }

  const Eigen::Vector3d center = convexTransform.translation();
  const double boundingRadius = computeBoundingRadius(convex);

  const Eigen::Vector3d oc = ray.origin - center;
  const double a = ray.direction.squaredNorm();
  const double b = 2.0 * oc.dot(ray.direction);
  const double c = oc.squaredNorm() - boundingRadius * boundingRadius;

  double t0, t1;
  if (solveQuadratic(a, b, c, t0, t1) < 0.0) {
    return false;
  }

  if (t1 < 0.0 || t0 > maxDist) {
    return false;
  }

  double lo = std::max(0.0, t0);
  double hi = std::min(maxDist, t1);

  if (!isPointInConvex(ray.pointAt(hi), convex, convexTransform)) {
    bool foundEntry = false;
    constexpr int kCoarseSteps = 16;
    double step = (hi - lo) / kCoarseSteps;
    for (int i = 1; i <= kCoarseSteps; ++i) {
      double t = lo + i * step;
      if (isPointInConvex(ray.pointAt(t), convex, convexTransform)) {
        hi = t;
        lo = t - step;
        foundEntry = true;
        break;
      }
    }
    if (!foundEntry) {
      return false;
    }
  }

  constexpr int kBinarySearchIterations = 32;
  for (int i = 0; i < kBinarySearchIterations; ++i) {
    double mid = (lo + hi) * 0.5;
    if (isPointInConvex(ray.pointAt(mid), convex, convexTransform)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }

  double hitT = hi;
  if (hitT > maxDist) {
    return false;
  }

  result.hit = true;
  result.distance = hitT;
  result.point = ray.pointAt(hitT);

  const Eigen::Isometry3d invTransform = convexTransform.inverse();
  const Eigen::Vector3d localPoint = invTransform * result.point;

  Eigen::Vector3d bestNormal = -ray.direction.normalized();
  double bestDot = -std::numeric_limits<double>::max();

  constexpr int kNormalSamples = 6;
  const Eigen::Vector3d directions[kNormalSamples]
      = {Eigen::Vector3d::UnitX(),
         -Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         -Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ(),
         -Eigen::Vector3d::UnitZ()};

  for (const auto& dir : directions) {
    Eigen::Vector3d support = convex.support(dir);
    Eigen::Vector3d toPoint = localPoint - support;
    double dot = toPoint.dot(dir);
    if (dot > bestDot) {
      bestDot = dot;
      bestNormal = convexTransform.rotation() * dir;
    }
  }

  if (bestNormal.dot(ray.direction) > 0.0) {
    bestNormal = -bestNormal;
  }

  result.normal = bestNormal;

  return true;
}

} // namespace dart::collision::experimental
