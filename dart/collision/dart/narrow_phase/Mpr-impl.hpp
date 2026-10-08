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

#include <dart/collision/dart/narrow_phase/Mpr.hpp>

namespace dart::collision::native::detail::mpr {

constexpr double kEpsilon = 1e-12;

struct Portal
{
  std::array<SupportPoint, 4> points;
  int size = 0;
};

SupportPoint makeCenterPoint(
    const Eigen::Vector3d& centerA, const Eigen::Vector3d& centerB);
bool isZero(double value);
bool normalizeSafe(Eigen::Vector3d& v);
void portalDir(const Portal& portal, Eigen::Vector3d& dir);
bool portalEncapsulatesOrigin(const Portal& portal, const Eigen::Vector3d& dir);
bool portalReachTolerance(
    const Portal& portal, const SupportPoint& v4, const Eigen::Vector3d& dir);
bool portalCanEncapsulateOrigin(
    const SupportPoint& v4, const Eigen::Vector3d& dir);
void expandPortal(Portal& portal, const SupportPoint& v4);
Eigen::Vector3d closestPointOnTriangleToOrigin(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c);
void findPos(
    const Portal& portal, Eigen::Vector3d& pointA, Eigen::Vector3d& pointB);
void findPenetrationTouch(Portal& portal, MprResult& result);
void findPenetrationSegment(Portal& portal, MprResult& result);

template <typename SupportA, typename SupportB>
SupportPoint computeSupport(
    const SupportA& supportA,
    const SupportB& supportB,
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

template <typename SupportA, typename SupportB>
int discoverPortal(
    const SupportA& supportA,
    const SupportB& supportB,
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

template <typename SupportA, typename SupportB>
int refinePortal(
    const SupportA& supportA, const SupportB& supportB, Portal& portal)
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

template <typename SupportA, typename SupportB>
void findPenetration(
    const SupportA& supportA,
    const SupportB& supportB,
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

template <typename SupportA, typename SupportB>
MprResult penetrationT(
    const SupportA& supportA,
    const SupportB& supportB,
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

} // namespace dart::collision::native::detail::mpr
