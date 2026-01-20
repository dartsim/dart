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

#include <dart/collision/experimental/narrow_phase/mpr.hpp>

#include <algorithm>
#include <array>

#include <cmath>

namespace dart::collision::experimental {

namespace {

constexpr double kEpsilon = 1e-12;

struct Portal
{
  std::array<SupportPoint, 4> points;
  int size = 0;
};

SupportPoint computeSupport(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& direction)
{
  Eigen::Vector3d dir = direction;
  if (dir.squaredNorm() < kEpsilon) {
    dir = Eigen::Vector3d::UnitX();
  }

  SupportPoint point;
  point.v1 = supportA(dir);
  point.v2 = supportB(-dir);
  point.v = point.v1 - point.v2;
  return point;
}

SupportPoint makeCenterPoint(
    const Eigen::Vector3d& centerA, const Eigen::Vector3d& centerB)
{
  SupportPoint point;
  point.v1 = centerA;
  point.v2 = centerB;
  point.v = centerA - centerB;
  return point;
}

bool isZero(double value)
{
  return std::abs(value) < kEpsilon;
}

bool normalizeSafe(Eigen::Vector3d& v)
{
  const double norm = v.norm();
  if (norm < kEpsilon) {
    return false;
  }
  v /= norm;
  return true;
}

void portalDir(const Portal& portal, Eigen::Vector3d& dir)
{
  const Eigen::Vector3d v2v1 = portal.points[2].v - portal.points[1].v;
  const Eigen::Vector3d v3v1 = portal.points[3].v - portal.points[1].v;
  dir = v2v1.cross(v3v1);
  if (!normalizeSafe(dir)) {
    dir = Eigen::Vector3d::UnitX();
  }
}

bool portalEncapsulatesOrigin(const Portal& portal, const Eigen::Vector3d& dir)
{
  const double dot = dir.dot(portal.points[1].v);
  return isZero(dot) || dot > 0.0;
}

bool portalReachTolerance(
    const Portal& portal, const SupportPoint& v4, const Eigen::Vector3d& dir)
{
  const double dv1 = portal.points[1].v.dot(dir);
  const double dv2 = portal.points[2].v.dot(dir);
  const double dv3 = portal.points[3].v.dot(dir);
  const double dv4 = v4.v.dot(dir);

  double delta = std::min({dv4 - dv1, dv4 - dv2, dv4 - dv3});
  return delta <= Mpr::kTolerance || isZero(delta - Mpr::kTolerance);
}

bool portalCanEncapsulateOrigin(
    const SupportPoint& v4, const Eigen::Vector3d& dir)
{
  const double dot = v4.v.dot(dir);
  return isZero(dot) || dot > 0.0;
}

void expandPortal(Portal& portal, const SupportPoint& v4)
{
  const Eigen::Vector3d v4v0 = v4.v.cross(portal.points[0].v);
  double dot = portal.points[1].v.dot(v4v0);
  if (dot > 0.0) {
    dot = portal.points[2].v.dot(v4v0);
    if (dot > 0.0) {
      portal.points[1] = v4;
    } else {
      portal.points[3] = v4;
    }
  } else {
    dot = portal.points[3].v.dot(v4v0);
    if (dot > 0.0) {
      portal.points[2] = v4;
    } else {
      portal.points[1] = v4;
    }
  }
}

int discoverPortal(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& centerA,
    const Eigen::Vector3d& centerB,
    Portal& portal)
{
  portal.points[0] = makeCenterPoint(centerA, centerB);
  portal.size = 1;

  if (portal.points[0].v.squaredNorm() < kEpsilon) {
    portal.points[0].v += Eigen::Vector3d(Mpr::kTolerance * 10.0, 0.0, 0.0);
  }

  Eigen::Vector3d dir = -portal.points[0].v;
  if (!normalizeSafe(dir)) {
    dir = Eigen::Vector3d::UnitX();
  }

  portal.points[1] = computeSupport(supportA, supportB, dir);
  portal.size = 2;

  double dot = portal.points[1].v.dot(dir);
  if (isZero(dot) || dot < 0.0) {
    return -1;
  }

  dir = portal.points[0].v.cross(portal.points[1].v);
  if (dir.squaredNorm() < kEpsilon) {
    if (portal.points[1].v.squaredNorm() < kEpsilon) {
      return 1;
    }
    return 2;
  }

  dir.normalize();
  portal.points[2] = computeSupport(supportA, supportB, dir);
  dot = portal.points[2].v.dot(dir);
  if (isZero(dot) || dot < 0.0) {
    return -1;
  }

  portal.size = 3;

  Eigen::Vector3d va = portal.points[1].v - portal.points[0].v;
  Eigen::Vector3d vb = portal.points[2].v - portal.points[0].v;
  dir = va.cross(vb);
  if (!normalizeSafe(dir)) {
    return -1;
  }

  dot = dir.dot(portal.points[0].v);
  if (dot > 0.0) {
    std::swap(portal.points[1], portal.points[2]);
    dir = -dir;
  }

  while (portal.size < 4) {
    portal.points[3] = computeSupport(supportA, supportB, dir);
    dot = portal.points[3].v.dot(dir);
    if (isZero(dot) || dot < 0.0) {
      return -1;
    }

    int cont = 0;
    Eigen::Vector3d cross = portal.points[1].v.cross(portal.points[3].v);
    dot = cross.dot(portal.points[0].v);
    if (dot < 0.0 && !isZero(dot)) {
      portal.points[2] = portal.points[3];
      cont = 1;
    }

    if (!cont) {
      cross = portal.points[3].v.cross(portal.points[2].v);
      dot = cross.dot(portal.points[0].v);
      if (dot < 0.0 && !isZero(dot)) {
        portal.points[1] = portal.points[3];
        cont = 1;
      }
    }

    if (cont) {
      va = portal.points[1].v - portal.points[0].v;
      vb = portal.points[2].v - portal.points[0].v;
      dir = va.cross(vb);
      if (!normalizeSafe(dir)) {
        return -1;
      }
    } else {
      portal.size = 4;
    }
  }

  return 0;
}

int refinePortal(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    Portal& portal)
{
  while (true) {
    Eigen::Vector3d dir;
    portalDir(portal, dir);

    if (portalEncapsulatesOrigin(portal, dir)) {
      return 0;
    }

    SupportPoint v4 = computeSupport(supportA, supportB, dir);

    if (!portalCanEncapsulateOrigin(v4, dir)
        || portalReachTolerance(portal, v4, dir)) {
      return -1;
    }

    expandPortal(portal, v4);
  }

  return -1;
}

struct SegmentClosestResult
{
  Eigen::Vector3d closest = Eigen::Vector3d::Zero();
  double t = 0.0;
};

SegmentClosestResult closestPointOnSegmentToOrigin(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  SegmentClosestResult result;
  const Eigen::Vector3d ab = b - a;
  const double abLen2 = ab.squaredNorm();
  if (abLen2 < kEpsilon) {
    result.closest = a;
    result.t = 0.0;
    return result;
  }

  double t = -a.dot(ab) / abLen2;
  t = std::clamp(t, 0.0, 1.0);
  result.t = t;
  result.closest = a + t * ab;
  return result;
}

Eigen::Vector3d closestPointOnTriangleToOrigin(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = -a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }

  const Eigen::Vector3d bp = -b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }

  const Eigen::Vector3d cp = -c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const double denom = va + vb + vc;
  if (isZero(denom)) {
    auto closest = closestPointOnSegmentToOrigin(a, b).closest;
    double bestDist2 = closest.squaredNorm();

    const Eigen::Vector3d acClosest
        = closestPointOnSegmentToOrigin(a, c).closest;
    double dist2 = acClosest.squaredNorm();
    if (dist2 < bestDist2) {
      closest = acClosest;
      bestDist2 = dist2;
    }

    const Eigen::Vector3d bcClosest
        = closestPointOnSegmentToOrigin(b, c).closest;
    dist2 = bcClosest.squaredNorm();
    if (dist2 < bestDist2) {
      closest = bcClosest;
    }

    return closest;
  }

  const double inv = 1.0 / denom;
  const double v = vb * inv;
  const double w = vc * inv;
  const double u = 1.0 - v - w;
  return u * a + v * b + w * c;
}

void findPos(
    const Portal& portal, Eigen::Vector3d& pointA, Eigen::Vector3d& pointB)
{
  Eigen::Vector3d dir;
  portalDir(portal, dir);

  std::array<double, 4> b{};
  Eigen::Vector3d vec;

  vec = portal.points[1].v.cross(portal.points[2].v);
  b[0] = vec.dot(portal.points[3].v);

  vec = portal.points[3].v.cross(portal.points[2].v);
  b[1] = vec.dot(portal.points[0].v);

  vec = portal.points[0].v.cross(portal.points[1].v);
  b[2] = vec.dot(portal.points[3].v);

  vec = portal.points[2].v.cross(portal.points[1].v);
  b[3] = vec.dot(portal.points[0].v);

  double sum = b[0] + b[1] + b[2] + b[3];
  if (isZero(sum) || sum < 0.0) {
    b[0] = 0.0;
    vec = portal.points[2].v.cross(portal.points[3].v);
    b[1] = vec.dot(dir);
    vec = portal.points[3].v.cross(portal.points[1].v);
    b[2] = vec.dot(dir);
    vec = portal.points[1].v.cross(portal.points[2].v);
    b[3] = vec.dot(dir);
    sum = b[1] + b[2] + b[3];
  }

  const double inv = 1.0 / sum;

  pointA.setZero();
  pointB.setZero();
  for (int i = 0; i < 4; ++i) {
    pointA += b[i] * portal.points[i].v1;
    pointB += b[i] * portal.points[i].v2;
  }
  pointA *= inv;
  pointB *= inv;
}

void findPenetration(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    Portal& portal,
    MprResult& result)
{
  for (int iter = 0; iter < Mpr::kMaxIterations; ++iter) {
    Eigen::Vector3d dir;
    portalDir(portal, dir);
    SupportPoint v4 = computeSupport(supportA, supportB, dir);

    if (portalReachTolerance(portal, v4, dir)
        || iter + 1 >= Mpr::kMaxIterations) {
      const Eigen::Vector3d a = portal.points[1].v;
      const Eigen::Vector3d b = portal.points[2].v;
      const Eigen::Vector3d c = portal.points[3].v;
      const Eigen::Vector3d closest = closestPointOnTriangleToOrigin(a, b, c);
      const double depth = closest.norm();

      result.depth = depth;
      if (depth < kEpsilon) {
        result.normal = Eigen::Vector3d::Zero();
      } else {
        result.normal = closest / depth;
      }

      findPos(portal, result.pointOnA, result.pointOnB);
      result.position = 0.5 * (result.pointOnA + result.pointOnB);
      result.success = true;
      return;
    }

    expandPortal(portal, v4);
  }
}

void findPenetrationTouch(Portal& portal, MprResult& result)
{
  result.depth = 0.0;
  result.normal = Eigen::Vector3d::Zero();
  result.pointOnA = portal.points[1].v1;
  result.pointOnB = portal.points[1].v2;
  result.position = 0.5 * (result.pointOnA + result.pointOnB);
  result.success = true;
}

void findPenetrationSegment(Portal& portal, MprResult& result)
{
  result.pointOnA = portal.points[1].v1;
  result.pointOnB = portal.points[1].v2;
  result.position = 0.5 * (result.pointOnA + result.pointOnB);

  result.normal = portal.points[1].v;
  result.depth = result.normal.norm();
  if (result.depth > kEpsilon) {
    result.normal /= result.depth;
  } else {
    result.normal = Eigen::Vector3d::Zero();
  }
  result.success = true;
}

} // namespace

bool Mpr::intersect(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& centerA,
    const Eigen::Vector3d& centerB)
{
  Portal portal;
  int res = discoverPortal(supportA, supportB, centerA, centerB, portal);
  if (res < 0) {
    return false;
  }
  if (res > 0) {
    return true;
  }

  res = refinePortal(supportA, supportB, portal);
  return res == 0;
}

MprResult Mpr::penetration(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& centerA,
    const Eigen::Vector3d& centerB)
{
  MprResult result;
  Portal portal;

  const int res = discoverPortal(supportA, supportB, centerA, centerB, portal);
  if (res < 0) {
    return result;
  }

  if (res == 1) {
    findPenetrationTouch(portal, result);
    return result;
  }

  if (res == 2) {
    findPenetrationSegment(portal, result);
    return result;
  }

  if (refinePortal(supportA, supportB, portal) < 0) {
    return result;
  }

  findPenetration(supportA, supportB, portal, result);
  return result;
}

} // namespace dart::collision::experimental
