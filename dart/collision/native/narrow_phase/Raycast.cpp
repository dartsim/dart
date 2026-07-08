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

#include <dart/collision/native/narrow_phase/Raycast.hpp>

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>

namespace dart::collision::native {

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

  if (std::abs(q) < kEpsilon) {
    const double invTwoA = 0.5 / a;
    t0 = (-b - sqrtDisc) * invTwoA;
    t1 = (-b + sqrtDisc) * invTwoA;
  } else {
    t0 = q / a;
    t1 = c / q;
  }

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
  if (c < 0.0 && t <= kEpsilon) {
    t = t1;
  }
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
  const double maxDist = std::min(ray.maxDistance, option.maxDistance);

  int boundaryAxis = -1;
  int boundarySign = 1;
  double boundaryDirectionDot = std::numeric_limits<double>::max();
  bool originInsideOrOn = true;
  for (int i = 0; i < 3; ++i) {
    if (localOrigin[i] < -halfExtents[i] - kEpsilon
        || localOrigin[i] > halfExtents[i] + kEpsilon) {
      originInsideOrOn = false;
      break;
    }

    const double distanceToMin = std::abs(localOrigin[i] + halfExtents[i]);
    const double distanceToMax = std::abs(localOrigin[i] - halfExtents[i]);
    if (distanceToMin <= kEpsilon) {
      const double directionDot = -localDir[i];
      if (directionDot < boundaryDirectionDot) {
        boundaryAxis = i;
        boundarySign = -1;
        boundaryDirectionDot = directionDot;
      }
    }
    if (distanceToMax <= kEpsilon) {
      const double directionDot = localDir[i];
      if (directionDot < boundaryDirectionDot) {
        boundaryAxis = i;
        boundarySign = 1;
        boundaryDirectionDot = directionDot;
      }
    }
  }

  if (originInsideOrOn && boundaryAxis >= 0 && maxDist >= 0.0) {
    result.hit = true;
    result.distance = 0.0;
    result.point = ray.origin;

    Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
    localNormal[boundaryAxis] = static_cast<double>(boundarySign);
    result.normal = boxTransform.rotation() * localNormal;

    return true;
  }

  double tMin = -std::numeric_limits<double>::max();
  double tMax = std::numeric_limits<double>::max();
  int hitAxisMin = -1;
  int hitSignMin = 1;
  int hitAxisMax = -1;
  int hitSignMax = 1;

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
        hitAxisMin = i;
        hitSignMin = sign;
      }
      if (t2 < tMax) {
        tMax = t2;
        hitAxisMax = i;
        hitSignMax = -sign;
      }

      if (tMin > tMax) {
        return false;
      }
    }
  }

  double tHit = tMin;
  int hitAxis = hitAxisMin;
  int hitSign = hitSignMin;
  if (hitAxis < 0 || tHit < 0.0) {
    if (tMax < 0.0) {
      return false;
    }
    tHit = tMax;
    hitAxis = hitAxisMax;
    hitSign = hitSignMax;
  }

  if (tHit > maxDist) {
    return false;
  }

  result.hit = true;
  result.distance = tHit;
  result.point = ray.pointAt(tHit);

  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  if (hitAxis < 0) {
    return false;
  }
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
  const double radiusSquared = radius * radius;
  const double radialSquared = ox * ox + oy * oy;
  const bool originStrictlyInsideCylinder
      = radialSquared < radiusSquared - kEpsilon
        && localOrigin.z() >= -halfHeight - kEpsilon
        && localOrigin.z() <= halfHeight + kEpsilon;
  const bool originStrictlyInsideTopCap
      = (localOrigin - Eigen::Vector3d(0, 0, halfHeight)).squaredNorm()
        < radiusSquared - kEpsilon;
  const bool originStrictlyInsideBottomCap
      = (localOrigin - Eigen::Vector3d(0, 0, -halfHeight)).squaredNorm()
        < radiusSquared - kEpsilon;
  const bool originStrictlyInsideCapsule = originStrictlyInsideCylinder
                                           || originStrictlyInsideTopCap
                                           || originStrictlyInsideBottomCap;

  auto acceptsHitDistance = [originStrictlyInsideCapsule](double& t) {
    if (std::abs(t) <= kEpsilon) {
      t = 0.0;
    }
    return originStrictlyInsideCapsule ? t > kEpsilon : t >= 0.0;
  };

  double a = dx * dx + dy * dy;
  double b = 2.0 * (ox * dx + oy * dy);
  double c = radialSquared - radiusSquared;

  if (a > kEpsilon) {
    double t0, t1;
    if (solveQuadratic(a, b, c, t0, t1) >= 0.0) {
      for (double t : {t0, t1}) {
        if (acceptsHitDistance(t) && t < bestT) {
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

  auto testSphereCap = [&](const Eigen::Vector3d& center, double capSign) {
    Eigen::Vector3d oc = localOrigin - center;
    double a2 = localDir.squaredNorm();
    double b2 = 2.0 * oc.dot(localDir);
    double c2 = oc.squaredNorm() - radiusSquared;

    double t0, t1;
    if (solveQuadratic(a2, b2, c2, t0, t1) >= 0.0) {
      for (double t : {t0, t1}) {
        if (acceptsHitDistance(t) && t < bestT) {
          Eigen::Vector3d hitPoint = localOrigin + t * localDir;
          if (capSign * hitPoint.z() < halfHeight - kEpsilon) {
            continue;
          }

          bestT = t;
          bestNormal = (hitPoint - center).normalized();
        }
      }
    }
  };

  testSphereCap(Eigen::Vector3d(0, 0, halfHeight), 1.0);
  testSphereCap(Eigen::Vector3d(0, 0, -halfHeight), -1.0);

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
  if (std::abs(t) <= kMeshEpsilon) {
    t = 0.0;
  }

  return t >= 0.0;
}

bool rayAabbIntersect(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Aabb& box,
    double maxDistance,
    double& tEnter,
    double& tExit)
{
  tEnter = 0.0;
  tExit = maxDistance;

  for (int i = 0; i < 3; ++i) {
    const double o = origin[i];
    const double d = direction[i];
    const double bMin = box.min[i];
    const double bMax = box.max[i];

    if (std::abs(d) < kMeshEpsilon) {
      if (o < bMin || o > bMax) {
        return false;
      }
      continue;
    }

    const double invD = 1.0 / d;
    double t0 = (bMin - o) * invD;
    double t1 = (bMax - o) * invD;
    if (t0 > t1) {
      std::swap(t0, t1);
    }

    tEnter = std::max(tEnter, t0);
    tExit = std::min(tExit, t1);
    if (tEnter > tExit) {
      return false;
    }
  }

  return tExit >= 0.0;
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
  const auto& nodes = mesh.bvhNodes();
  const auto& triIndices = mesh.bvhTriIndices();

  if (nodes.empty()) {
    return false;
  }

  double maxDist = std::min(ray.maxDistance, option.maxDistance);
  double bestT = std::numeric_limits<double>::max();
  Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();

  std::vector<int> stack;
  stack.push_back(0);
  while (!stack.empty()) {
    const int nodeIndex = stack.back();
    stack.pop_back();

    const auto& node = nodes[static_cast<std::size_t>(nodeIndex)];
    double tEnter = 0.0;
    double tExit = 0.0;
    if (!rayAabbIntersect(
            localOrigin, localDir, node.box, maxDist, tEnter, tExit)
        || tEnter > bestT) {
      continue;
    }

    if (node.count > 0) {
      for (int i = 0; i < node.count; ++i) {
        const int triIndex
            = triIndices[static_cast<std::size_t>(node.first + i)];
        const auto& tri = triangles[static_cast<std::size_t>(triIndex)];
        const Eigen::Vector3d& v0 = vertices[static_cast<std::size_t>(tri[0])];
        const Eigen::Vector3d& v1 = vertices[static_cast<std::size_t>(tri[1])];
        const Eigen::Vector3d& v2 = vertices[static_cast<std::size_t>(tri[2])];

        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
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
          if (t >= 0.0 && t < bestT && t <= maxDist) {
            bestT = t;
            const Eigen::Vector3d edge1 = v1 - v0;
            const Eigen::Vector3d edge2 = v2 - v0;
            bestNormal = edge1.cross(edge2).normalized();
          }
        }
      }
      continue;
    }

    if (node.left >= 0) {
      stack.push_back(node.left);
    }
    if (node.right >= 0) {
      stack.push_back(node.right);
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

bool raycastConvex(
    const Ray& ray,
    const ConvexShape& convex,
    const Eigen::Isometry3d& convexTransform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  const double maxDist = std::min(ray.maxDistance, option.maxDistance);
  const auto& faces = convex.getFaces();
  if (faces.empty()) {
    return false;
  }

  double entryDistance = 0.0;
  double exitDistance = maxDist;
  Eigen::Vector3d entryNormal = -ray.direction;
  Eigen::Vector3d exitNormal = ray.direction;
  Eigen::Vector3d boundaryNormal = -ray.direction;
  bool startsInside = true;
  bool originOnBoundary = false;
  bool hasExitWithinMaxDistance = false;

  for (const auto& face : faces) {
    const Eigen::Vector3d worldPoint = convexTransform * face.point;
    const Eigen::Vector3d worldNormal
        = convexTransform.rotation() * face.normal;

    const double signedDistance = worldNormal.dot(ray.origin - worldPoint);
    const double directionDot = worldNormal.dot(ray.direction);

    if (signedDistance > kEpsilon) {
      startsInside = false;
      if (directionDot >= -kEpsilon) {
        return false;
      }

      const double candidateEntry = -signedDistance / directionDot;
      if (candidateEntry > entryDistance) {
        entryDistance = candidateEntry;
        entryNormal = worldNormal;
      }
    } else if (std::abs(signedDistance) <= kEpsilon) {
      originOnBoundary = true;
      boundaryNormal = worldNormal;
      if (directionDot > kEpsilon) {
        exitDistance = 0.0;
        exitNormal = worldNormal;
        hasExitWithinMaxDistance = true;
      }
    } else if (directionDot > kEpsilon) {
      const double candidateExit = -signedDistance / directionDot;
      if (candidateExit >= 0.0 && candidateExit <= exitDistance) {
        exitDistance = candidateExit;
        exitNormal = worldNormal;
        hasExitWithinMaxDistance = true;
      }
    }

    if (entryDistance - exitDistance > kEpsilon) {
      return false;
    }
  }

  if (startsInside && !originOnBoundary) {
    if (!hasExitWithinMaxDistance) {
      return false;
    }

    result.hit = true;
    result.distance = exitDistance;
    result.point = ray.pointAt(result.distance);
    result.normal = exitNormal;
    return true;
  }

  if (entryDistance < 0.0 || entryDistance > maxDist) {
    return false;
  }

  if (startsInside) {
    result.hit = true;
    result.distance = 0.0;
    result.point = ray.origin;
    result.normal = boundaryNormal;
    return true;
  }

  result.hit = true;
  result.distance = entryDistance;
  result.point = ray.pointAt(result.distance);
  result.normal = entryNormal;

  if (result.normal.dot(ray.direction) > 0.0) {
    result.normal = -result.normal;
  }

  return true;
}

} // namespace dart::collision::native
