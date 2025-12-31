/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/dart/DartCollisionEngine.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/DistanceFilter.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/DartCollisionBroadphase.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace dart {
namespace collision {

namespace {

constexpr double kDistanceEps = 1.0e-12;

bool overlaps(const CoreObject& a, const CoreObject& b)
{
  return (a.worldAabbMin.array() <= b.worldAabbMax.array()).all()
         && (a.worldAabbMax.array() >= b.worldAabbMin.array()).all();
}

bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol)
{
  return (pos1 - pos2).norm() < tol;
}

void mergeContacts(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    const CollisionResult& pairResult)
{
  if (!pairResult.isCollision()) [[likely]]
    return;

  const auto tol = 3.0e-12;

  for (auto pairContact : pairResult.getContacts()) {
    if (!option.allowNegativePenetrationDepthContacts
        && pairContact.penetrationDepth < 0.0) {
      continue;
    }

    bool foundClose = false;

    for (const auto& totalContact : totalResult.getContacts()) {
      if (isClose(pairContact.point, totalContact.point, tol)) {
        foundClose = true;
        break;
      }
    }

    if (foundClose)
      continue;

    auto contact = pairContact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    totalResult.addContact(contact);

    if (totalResult.getNumContacts() >= option.maxNumContacts)
      break;
  }
}

struct DistanceInfo
{
  double distance{0.0};
  Eigen::Vector3d point1{Eigen::Vector3d::Zero()};
  Eigen::Vector3d point2{Eigen::Vector3d::Zero()};
};

struct SatDistanceResult
{
  bool separated{false};
  double separation{std::numeric_limits<double>::infinity()};
  double overlap{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};
};

double aabbDistanceSquared(const CoreObject& a, const CoreObject& b)
{
  double dist2 = 0.0;
  for (int i = 0; i < 3; ++i) {
    double d = 0.0;
    if (a.worldAabbMax[i] < b.worldAabbMin[i])
      d = b.worldAabbMin[i] - a.worldAabbMax[i];
    else if (b.worldAabbMax[i] < a.worldAabbMin[i])
      d = a.worldAabbMin[i] - b.worldAabbMax[i];
    dist2 += d * d;
  }
  return dist2;
}

Eigen::Vector3d clampPoint(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& minPoint,
    const Eigen::Vector3d& maxPoint)
{
  return point.cwiseMin(maxPoint).cwiseMax(minPoint);
}

double projectBoxExtent(
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& halfSize,
    const Eigen::Isometry3d& T)
{
  const double axisNorm = axis.norm();
  if (axisNorm < kDistanceEps)
    return 0.0;

  const Eigen::Vector3d n = axis / axisNorm;
  const Eigen::Vector3d axis0 = T.linear().col(0);
  const Eigen::Vector3d axis1 = T.linear().col(1);
  const Eigen::Vector3d axis2 = T.linear().col(2);

  return std::abs(n.dot(axis0)) * halfSize[0]
         + std::abs(n.dot(axis1)) * halfSize[1]
         + std::abs(n.dot(axis2)) * halfSize[2];
}

double projectCylinderExtent(
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& cylinderAxis,
    double halfHeight,
    double radius)
{
  const double axisNorm = axis.norm();
  if (axisNorm < kDistanceEps)
    return 0.0;

  const Eigen::Vector3d n = axis / axisNorm;
  const double axisDot = n.dot(cylinderAxis);
  const double axial = std::abs(axisDot) * halfHeight;
  const double radial
      = radius * std::sqrt(std::max(0.0, 1.0 - axisDot * axisDot));
  return axial + radial;
}

Eigen::Vector3d supportBox(
    const Eigen::Isometry3d& T,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& dir)
{
  const Eigen::Vector3d halfSize = 0.5 * size;
  Eigen::Vector3d result = T.translation();

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d axis = T.linear().col(i);
    result += axis * ((dir.dot(axis) >= 0.0) ? halfSize[i] : -halfSize[i]);
  }

  return result;
}

Eigen::Vector3d supportCylinder(
    const Eigen::Isometry3d& T,
    double radius,
    double halfHeight,
    const Eigen::Vector3d& dir)
{
  Eigen::Vector3d dirNorm = dir;
  const double dirNormValue = dirNorm.norm();
  if (dirNormValue > kDistanceEps)
    dirNorm /= dirNormValue;
  else
    dirNorm = Eigen::Vector3d::UnitX();

  const Eigen::Vector3d axis = T.linear().col(2);
  const double axisDot = dirNorm.dot(axis);
  const double sign = (axisDot >= 0.0) ? 1.0 : -1.0;
  const Eigen::Vector3d radial = dirNorm - axisDot * axis;
  Eigen::Vector3d radialDir = Eigen::Vector3d::Zero();
  if (radial.squaredNorm() > kDistanceEps)
    radialDir = radial.normalized();

  return T.translation() + sign * halfHeight * axis + radius * radialDir;
}

void updateSatAxis(
    const Eigen::Vector3d& axis,
    double extent1,
    double extent2,
    const Eigen::Vector3d& centerDelta,
    SatDistanceResult& result)
{
  if (axis.squaredNorm() < kDistanceEps)
    return;

  const Eigen::Vector3d n = axis.normalized();
  const double centerDiff = centerDelta.dot(n);
  const double gap = std::abs(centerDiff) - (extent1 + extent2);

  if (gap > 0.0) {
    if (!result.separated || gap < result.separation) {
      result.separated = true;
      result.separation = gap;
      result.axis = (centerDiff >= 0.0) ? n : -n;
    }
  } else if (!result.separated) {
    const double overlap = -gap;
    if (overlap < result.overlap) {
      result.overlap = overlap;
      result.axis = (centerDiff >= 0.0) ? n : -n;
    }
  }
}

double signedDistancePointCylinderLocal(
    const Eigen::Vector3d& point,
    double radius,
    double halfHeight,
    Eigen::Vector3d* closestPoint)
{
  const double radial = std::sqrt(point.x() * point.x() + point.y() * point.y());
  const double dr = radial - radius;
  const double dz = std::abs(point.z()) - halfHeight;

  Eigen::Vector3d result = point;
  Eigen::Vector2d radialDir(1.0, 0.0);
  if (radial > kDistanceEps)
    radialDir = point.head<2>() / radial;

  const bool outsideRadial = dr > 0.0;
  const bool outsideAxial = dz > 0.0;

  if (!outsideRadial && !outsideAxial) {
    if (dr > dz) {
      result.x() = radialDir.x() * radius;
      result.y() = radialDir.y() * radius;
    } else {
      result.z() = (point.z() >= 0.0) ? halfHeight : -halfHeight;
    }
  } else if (outsideRadial && !outsideAxial) {
    result.x() = radialDir.x() * radius;
    result.y() = radialDir.y() * radius;
  } else if (!outsideRadial && outsideAxial) {
    result.z() = (point.z() >= 0.0) ? halfHeight : -halfHeight;
  } else {
    result.x() = radialDir.x() * radius;
    result.y() = radialDir.y() * radius;
    result.z() = (point.z() >= 0.0) ? halfHeight : -halfHeight;
  }

  if (closestPoint)
    *closestPoint = result;

  const double outsideRadialDist = std::max(dr, 0.0);
  const double outsideAxialDist = std::max(dz, 0.0);
  const double outsideDist
      = std::sqrt(outsideRadialDist * outsideRadialDist
                  + outsideAxialDist * outsideAxialDist);
  const double insideDist = std::min(std::max(dr, dz), 0.0);

  return outsideDist + insideDist;
}

double signedDistanceToPlane(
    const CoreObject& plane,
    const Eigen::Vector3d& point,
    Eigen::Vector3d* normalWorld)
{
  const Eigen::Vector3d& normalLocal = plane.shape.planeNormal;
  const Eigen::Vector3d normal = plane.worldTransform.linear() * normalLocal;
  if (normalWorld)
    *normalWorld = normal.normalized();

  const Eigen::Vector3d pointLocal = plane.worldTransform.inverse() * point;
  return normalLocal.dot(pointLocal) - plane.shape.planeOffset;
}

bool distanceSphereSphere(
    const CoreObject& sphere1,
    const CoreObject& sphere2,
    DistanceInfo* out)
{
  if (!out)
    return false;

  const Eigen::Vector3d c1 = sphere1.worldTransform.translation();
  const Eigen::Vector3d c2 = sphere2.worldTransform.translation();
  const Eigen::Vector3d delta = c1 - c2;
  const double dist = delta.norm();
  const double radiusSum = sphere1.shape.radius + sphere2.shape.radius;
  const Eigen::Vector3d normal
      = (dist > kDistanceEps) ? delta / dist : Eigen::Vector3d::UnitX();

  out->distance = dist - radiusSum;
  out->point1 = c1 - normal * sphere1.shape.radius;
  out->point2 = c2 + normal * sphere2.shape.radius;
  return true;
}

bool distanceSphereBox(
    const CoreObject& sphere,
    const CoreObject& box,
    DistanceInfo* out)
{
  if (!out)
    return false;

  const Eigen::Vector3d center = sphere.worldTransform.translation();
  const Eigen::Vector3d halfSize = 0.5 * box.shape.size;
  const Eigen::Isometry3d invBox = box.worldTransform.inverse();
  Eigen::Vector3d centerLocal = invBox * center;

  Eigen::Vector3d closestLocal = clampPoint(
      centerLocal, -halfSize, halfSize);
  Eigen::Vector3d closestWorld = box.worldTransform * closestLocal;

  Eigen::Vector3d delta = center - closestWorld;
  double dist = delta.norm();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
  double signedPointDistance = dist;

  if (dist > kDistanceEps) {
    normal = delta / dist;
  } else {
    Eigen::Vector3d absLocal = centerLocal.cwiseAbs();
    Eigen::Vector3d faceDistance = halfSize - absLocal;
    int axis = 0;
    double minFace = faceDistance[0];
    for (int i = 1; i < 3; ++i) {
      if (faceDistance[i] < minFace) {
        minFace = faceDistance[i];
        axis = i;
      }
    }
    Eigen::Vector3d normalLocal = Eigen::Vector3d::Zero();
    normalLocal[axis] = (centerLocal[axis] >= 0.0) ? 1.0 : -1.0;
    normal = box.worldTransform.linear() * normalLocal;
    signedPointDistance = -minFace;

    closestLocal = centerLocal;
    closestLocal[axis] = normalLocal[axis] * halfSize[axis];
    closestWorld = box.worldTransform * closestLocal;
  }

  out->distance = signedPointDistance - sphere.shape.radius;
  out->point2 = closestWorld;
  out->point1 = center - normal * sphere.shape.radius;
  return true;
}

bool distanceBoxSphere(
    const CoreObject& box,
    const CoreObject& sphere,
    DistanceInfo* out)
{
  if (!out)
    return false;

  DistanceInfo info;
  if (!distanceSphereBox(sphere, box, &info))
    return false;

  out->distance = info.distance;
  out->point1 = info.point2;
  out->point2 = info.point1;
  return true;
}

bool distanceSpherePlane(
    const CoreObject& sphere,
    const CoreObject& plane,
    DistanceInfo* out)
{
  if (!out)
    return false;

  Eigen::Vector3d normalWorld = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d center = sphere.worldTransform.translation();
  const double signedDistance = signedDistanceToPlane(plane, center, &normalWorld);
  const double absDistance = std::abs(signedDistance);
  const double distance = absDistance - sphere.shape.radius;
  const Eigen::Vector3d normal
      = (signedDistance >= 0.0) ? normalWorld : -normalWorld;

  out->distance = distance;
  out->point2 = center - normal * absDistance;
  out->point1 = center - normal * sphere.shape.radius;
  return true;
}

bool distancePlaneSphere(
    const CoreObject& plane,
    const CoreObject& sphere,
    DistanceInfo* out)
{
  if (!out)
    return false;

  DistanceInfo info;
  if (!distanceSpherePlane(sphere, plane, &info))
    return false;

  out->distance = info.distance;
  out->point1 = info.point2;
  out->point2 = info.point1;
  return true;
}

bool distanceBoxBox(
    const CoreObject& box1,
    const CoreObject& box2,
    DistanceInfo* out)
{
  if (!out)
    return false;

  SatDistanceResult sat;
  const Eigen::Vector3d centerDelta
      = box1.worldTransform.translation() - box2.worldTransform.translation();
  const Eigen::Vector3d halfSize1 = 0.5 * box1.shape.size;
  const Eigen::Vector3d halfSize2 = 0.5 * box2.shape.size;

  const Eigen::Vector3d axes1[3]
      = {box1.worldTransform.linear().col(0),
         box1.worldTransform.linear().col(1),
         box1.worldTransform.linear().col(2)};
  const Eigen::Vector3d axes2[3]
      = {box2.worldTransform.linear().col(0),
         box2.worldTransform.linear().col(1),
         box2.worldTransform.linear().col(2)};

  for (int i = 0; i < 3; ++i) {
    updateSatAxis(
        axes1[i],
        projectBoxExtent(axes1[i], halfSize1, box1.worldTransform),
        projectBoxExtent(axes1[i], halfSize2, box2.worldTransform),
        centerDelta,
        sat);
    updateSatAxis(
        axes2[i],
        projectBoxExtent(axes2[i], halfSize1, box1.worldTransform),
        projectBoxExtent(axes2[i], halfSize2, box2.worldTransform),
        centerDelta,
        sat);
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const Eigen::Vector3d axis = axes1[i].cross(axes2[j]);
      updateSatAxis(
          axis,
          projectBoxExtent(axis, halfSize1, box1.worldTransform),
          projectBoxExtent(axis, halfSize2, box2.worldTransform),
          centerDelta,
          sat);
    }
  }

  const Eigen::Vector3d normal = sat.axis;
  out->distance = sat.separated ? sat.separation : -sat.overlap;
  out->point1 = supportBox(box1.worldTransform, box1.shape.size, -normal);
  out->point2 = supportBox(box2.worldTransform, box2.shape.size, normal);
  return true;
}

bool distanceCylinderSphere(
    const CoreObject& cylinder,
    const CoreObject& sphere,
    DistanceInfo* out)
{
  if (!out)
    return false;

  const Eigen::Vector3d center = sphere.worldTransform.translation();
  const Eigen::Isometry3d invCyl = cylinder.worldTransform.inverse();
  const Eigen::Vector3d centerLocal = invCyl * center;
  Eigen::Vector3d closestLocal = Eigen::Vector3d::Zero();
  const double signedPointDistance = signedDistancePointCylinderLocal(
      centerLocal, cylinder.shape.radius, 0.5 * cylinder.shape.height, &closestLocal);

  const Eigen::Vector3d closestWorld = cylinder.worldTransform * closestLocal;
  Eigen::Vector3d normal = center - closestWorld;
  const double normalNorm = normal.norm();
  if (normalNorm > kDistanceEps)
    normal /= normalNorm;
  else
    normal = cylinder.worldTransform.linear().col(2);

  out->distance = signedPointDistance - sphere.shape.radius;
  out->point1 = closestWorld;
  out->point2 = center - normal * sphere.shape.radius;
  return true;
}

bool distanceSphereCylinder(
    const CoreObject& sphere,
    const CoreObject& cylinder,
    DistanceInfo* out)
{
  if (!out)
    return false;

  DistanceInfo info;
  if (!distanceCylinderSphere(cylinder, sphere, &info))
    return false;

  out->distance = info.distance;
  out->point1 = info.point2;
  out->point2 = info.point1;
  return true;
}

bool distanceCylinderPlane(
    const CoreObject& cylinder,
    const CoreObject& plane,
    DistanceInfo* out)
{
  if (!out)
    return false;

  Eigen::Vector3d normalWorld = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d center = cylinder.worldTransform.translation();
  const double signedDistance = signedDistanceToPlane(plane, center, &normalWorld);
  const Eigen::Vector3d axis = cylinder.worldTransform.linear().col(2);
  const double extent = projectCylinderExtent(
      normalWorld.normalized(),
      axis,
      0.5 * cylinder.shape.height,
      cylinder.shape.radius);

  const double distance = std::abs(signedDistance) - extent;
  const Eigen::Vector3d normal
      = (signedDistance >= 0.0) ? normalWorld.normalized()
                                : -normalWorld.normalized();

  const Eigen::Vector3d pointCylinder
      = supportCylinder(cylinder.worldTransform,
                        cylinder.shape.radius,
                        0.5 * cylinder.shape.height,
                        -normal);
  out->distance = distance;
  out->point1 = pointCylinder;
  out->point2 = pointCylinder - normal * distance;
  return true;
}

bool distancePlaneCylinder(
    const CoreObject& plane,
    const CoreObject& cylinder,
    DistanceInfo* out)
{
  if (!out)
    return false;

  DistanceInfo info;
  if (!distanceCylinderPlane(cylinder, plane, &info))
    return false;

  out->distance = info.distance;
  out->point1 = info.point2;
  out->point2 = info.point1;
  return true;
}

bool distanceBoxPlane(
    const CoreObject& box,
    const CoreObject& plane,
    DistanceInfo* out)
{
  if (!out)
    return false;

  Eigen::Vector3d normalWorld = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d center = box.worldTransform.translation();
  const double signedDistance = signedDistanceToPlane(plane, center, &normalWorld);
  const double extent = projectBoxExtent(
      normalWorld.normalized(),
      0.5 * box.shape.size,
      box.worldTransform);

  const double distance = std::abs(signedDistance) - extent;
  const Eigen::Vector3d normal
      = (signedDistance >= 0.0) ? normalWorld.normalized()
                                : -normalWorld.normalized();

  const Eigen::Vector3d pointBox
      = supportBox(box.worldTransform, box.shape.size, -normal);
  out->distance = distance;
  out->point1 = pointBox;
  out->point2 = pointBox - normal * distance;
  return true;
}

bool distancePlaneBox(
    const CoreObject& plane,
    const CoreObject& box,
    DistanceInfo* out)
{
  if (!out)
    return false;

  DistanceInfo info;
  if (!distanceBoxPlane(box, plane, &info))
    return false;

  out->distance = info.distance;
  out->point1 = info.point2;
  out->point2 = info.point1;
  return true;
}

bool distanceCylinderBox(
    const CoreObject& cylinder,
    const CoreObject& box,
    DistanceInfo* out)
{
  if (!out)
    return false;

  SatDistanceResult sat;
  const Eigen::Vector3d centerDelta
      = cylinder.worldTransform.translation() - box.worldTransform.translation();
  const Eigen::Vector3d cylAxis = cylinder.worldTransform.linear().col(2);
  const Eigen::Vector3d halfSize = 0.5 * box.shape.size;

  const Eigen::Vector3d boxAxes[3]
      = {box.worldTransform.linear().col(0),
         box.worldTransform.linear().col(1),
         box.worldTransform.linear().col(2)};

  updateSatAxis(
      boxAxes[0],
      projectCylinderExtent(
          boxAxes[0], cylAxis, 0.5 * cylinder.shape.height, cylinder.shape.radius),
      projectBoxExtent(boxAxes[0], halfSize, box.worldTransform),
      centerDelta,
      sat);
  updateSatAxis(
      boxAxes[1],
      projectCylinderExtent(
          boxAxes[1], cylAxis, 0.5 * cylinder.shape.height, cylinder.shape.radius),
      projectBoxExtent(boxAxes[1], halfSize, box.worldTransform),
      centerDelta,
      sat);
  updateSatAxis(
      boxAxes[2],
      projectCylinderExtent(
          boxAxes[2], cylAxis, 0.5 * cylinder.shape.height, cylinder.shape.radius),
      projectBoxExtent(boxAxes[2], halfSize, box.worldTransform),
      centerDelta,
      sat);
  updateSatAxis(
      cylAxis,
      projectCylinderExtent(
          cylAxis, cylAxis, 0.5 * cylinder.shape.height, cylinder.shape.radius),
      projectBoxExtent(cylAxis, halfSize, box.worldTransform),
      centerDelta,
      sat);

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d axis = cylAxis.cross(boxAxes[i]);
    updateSatAxis(
        axis,
        projectCylinderExtent(
            axis, cylAxis, 0.5 * cylinder.shape.height, cylinder.shape.radius),
        projectBoxExtent(axis, halfSize, box.worldTransform),
        centerDelta,
        sat);
  }

  const Eigen::Vector3d normal = sat.axis;
  out->distance = sat.separated ? sat.separation : -sat.overlap;
  out->point1 = supportCylinder(
      cylinder.worldTransform,
      cylinder.shape.radius,
      0.5 * cylinder.shape.height,
      -normal);
  out->point2 = supportBox(box.worldTransform, box.shape.size, normal);
  return true;
}

bool distanceBoxCylinder(
    const CoreObject& box,
    const CoreObject& cylinder,
    DistanceInfo* out)
{
  if (!out)
    return false;

  DistanceInfo info;
  if (!distanceCylinderBox(cylinder, box, &info))
    return false;

  out->distance = info.distance;
  out->point1 = info.point2;
  out->point2 = info.point1;
  return true;
}

bool distanceCylinderCylinder(
    const CoreObject& cylinder1,
    const CoreObject& cylinder2,
    DistanceInfo* out)
{
  if (!out)
    return false;

  SatDistanceResult sat;
  const Eigen::Vector3d centerDelta
      = cylinder1.worldTransform.translation() - cylinder2.worldTransform.translation();
  const Eigen::Vector3d axis1 = cylinder1.worldTransform.linear().col(2);
  const Eigen::Vector3d axis2 = cylinder2.worldTransform.linear().col(2);

  updateSatAxis(
      axis1,
      projectCylinderExtent(
          axis1, axis1, 0.5 * cylinder1.shape.height, cylinder1.shape.radius),
      projectCylinderExtent(
          axis1, axis2, 0.5 * cylinder2.shape.height, cylinder2.shape.radius),
      centerDelta,
      sat);
  updateSatAxis(
      axis2,
      projectCylinderExtent(
          axis2, axis1, 0.5 * cylinder1.shape.height, cylinder1.shape.radius),
      projectCylinderExtent(
          axis2, axis2, 0.5 * cylinder2.shape.height, cylinder2.shape.radius),
      centerDelta,
      sat);

  const Eigen::Vector3d crossAxis = axis1.cross(axis2);
  updateSatAxis(
      crossAxis,
      projectCylinderExtent(
          crossAxis, axis1, 0.5 * cylinder1.shape.height, cylinder1.shape.radius),
      projectCylinderExtent(
          crossAxis, axis2, 0.5 * cylinder2.shape.height, cylinder2.shape.radius),
      centerDelta,
      sat);

  const Eigen::Vector3d deltaPerp = centerDelta - centerDelta.dot(axis1) * axis1;
  updateSatAxis(
      deltaPerp,
      projectCylinderExtent(
          deltaPerp, axis1, 0.5 * cylinder1.shape.height, cylinder1.shape.radius),
      projectCylinderExtent(
          deltaPerp, axis2, 0.5 * cylinder2.shape.height, cylinder2.shape.radius),
      centerDelta,
      sat);

  const Eigen::Vector3d normal = sat.axis;
  out->distance = sat.separated ? sat.separation : -sat.overlap;
  out->point1 = supportCylinder(
      cylinder1.worldTransform,
      cylinder1.shape.radius,
      0.5 * cylinder1.shape.height,
      -normal);
  out->point2 = supportCylinder(
      cylinder2.worldTransform,
      cylinder2.shape.radius,
      0.5 * cylinder2.shape.height,
      normal);
  return true;
}

bool distanceCore(const CoreObject& obj1, const CoreObject& obj2, DistanceInfo* out)
{
  if (!out)
    return false;

  switch (obj1.shape.type) {
    case CoreShapeType::kSphere:
      switch (obj2.shape.type) {
        case CoreShapeType::kSphere:
          return distanceSphereSphere(obj1, obj2, out);
        case CoreShapeType::kBox:
          return distanceSphereBox(obj1, obj2, out);
        case CoreShapeType::kCylinder:
          return distanceSphereCylinder(obj1, obj2, out);
        case CoreShapeType::kPlane:
          return distanceSpherePlane(obj1, obj2, out);
        default:
          break;
      }
      break;
    case CoreShapeType::kBox:
      switch (obj2.shape.type) {
        case CoreShapeType::kSphere:
          return distanceBoxSphere(obj1, obj2, out);
        case CoreShapeType::kBox:
          return distanceBoxBox(obj1, obj2, out);
        case CoreShapeType::kCylinder:
          return distanceBoxCylinder(obj1, obj2, out);
        case CoreShapeType::kPlane:
          return distanceBoxPlane(obj1, obj2, out);
        default:
          break;
      }
      break;
    case CoreShapeType::kCylinder:
      switch (obj2.shape.type) {
        case CoreShapeType::kSphere:
          return distanceCylinderSphere(obj1, obj2, out);
        case CoreShapeType::kBox:
          return distanceCylinderBox(obj1, obj2, out);
        case CoreShapeType::kCylinder:
          return distanceCylinderCylinder(obj1, obj2, out);
        case CoreShapeType::kPlane:
          return distanceCylinderPlane(obj1, obj2, out);
        default:
          break;
      }
      break;
    case CoreShapeType::kPlane:
      switch (obj2.shape.type) {
        case CoreShapeType::kSphere:
          return distancePlaneSphere(obj1, obj2, out);
        case CoreShapeType::kBox:
          return distancePlaneBox(obj1, obj2, out);
        case CoreShapeType::kCylinder:
          return distancePlaneCylinder(obj1, obj2, out);
        default:
          break;
      }
      break;
    default:
      break;
  }

  return false;
}

} // namespace

//==============================================================================
bool DartCollisionEngine::collide(
    const ObjectList& objects,
    const CollisionOption& option,
    CollisionResult* result) const
{
  if (objects.size() < 2u) [[unlikely]]
    return false;

  constexpr std::uint8_t kGroup = 1u;

  std::vector<CollisionObject*> collObjects;
  collObjects.reserve(objects.size());

  std::vector<CoreBroadphaseEntry> entries;
  entries.reserve(objects.size());
  for (auto* object : objects) {
    const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
    collObjects.push_back(object);
    entries.push_back({&dartObject->getCoreObject(), kGroup});
  }

  std::vector<CoreBroadphasePair> pairs;
  pairs.reserve(objects.size());
  computeSweepPairs(entries, kGroup, kGroup, &pairs);

  bool collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (const auto& pair : pairs) {
    auto* collObj1 = collObjects[pair.first];
    auto* collObj2 = collObjects[pair.second];

    if (filter && filter->ignoresCollision(collObj1, collObj2)) [[unlikely]]
      continue;

    const auto& aabb1 = *entries[pair.first].object;
    const auto& aabb2 = *entries[pair.second].object;
    if (aabb1.shape.type == CoreShapeType::kNone
        || aabb2.shape.type == CoreShapeType::kNone
        || aabb1.shape.type == CoreShapeType::kUnsupported
        || aabb2.shape.type == CoreShapeType::kUnsupported) {
      continue;
    }
    if (!overlaps(aabb1, aabb2))
      continue;

    collisionFound = checkPair(collObj1, collObj2, option, result);

    if (result) {
      if (result->getNumContacts() >= option.maxNumContacts) [[unlikely]]
        return true;
    } else if (collisionFound) [[unlikely]] {
      return true;
    }
  }

  return collisionFound;
}

//==============================================================================
bool DartCollisionEngine::collide(
    const ObjectList& objects1,
    const ObjectList& objects2,
    const CollisionOption& option,
    CollisionResult* result) const
{
  if (objects1.empty() || objects2.empty()) [[unlikely]]
    return false;

  constexpr std::uint8_t kGroup1 = 1u;
  constexpr std::uint8_t kGroup2 = 1u << 1u;

  std::vector<CollisionObject*> collObjects;
  collObjects.reserve(objects1.size() + objects2.size());

  std::vector<CoreBroadphaseEntry> entries;
  entries.reserve(objects1.size() + objects2.size());

  std::unordered_map<CollisionObject*, std::size_t> entryMap;
  entryMap.reserve(objects1.size() + objects2.size());

  auto addEntry = [&](CollisionObject* object, std::uint8_t mask) {
    auto [it, inserted] = entryMap.emplace(object, entries.size());
    if (inserted) {
      const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
      entries.push_back({&dartObject->getCoreObject(), mask});
      collObjects.push_back(object);
    } else {
      entries[it->second].mask |= mask;
    }
  };

  for (auto* object : objects1)
    addEntry(object, kGroup1);
  for (auto* object : objects2)
    addEntry(object, kGroup2);

  std::vector<CoreBroadphasePair> pairs;
  pairs.reserve(entries.size());
  computeSweepPairs(entries, kGroup1, kGroup2, &pairs);

  bool collisionFound = false;
  const auto& filter = option.collisionFilter;

  for (const auto& pair : pairs) {
    auto* collObj1 = collObjects[pair.first];
    auto* collObj2 = collObjects[pair.second];

    if (filter && filter->ignoresCollision(collObj1, collObj2))
      continue;

    const auto& aabb1 = *entries[pair.first].object;
    const auto& aabb2 = *entries[pair.second].object;
    if (aabb1.shape.type == CoreShapeType::kNone
        || aabb2.shape.type == CoreShapeType::kNone
        || aabb1.shape.type == CoreShapeType::kUnsupported
        || aabb2.shape.type == CoreShapeType::kUnsupported) {
      continue;
    }
    if (!overlaps(aabb1, aabb2))
      continue;

    collisionFound = checkPair(collObj1, collObj2, option, result);

    if (result) {
      if (result->getNumContacts() >= option.maxNumContacts)
        return true;
    } else if (collisionFound) {
      return true;
    }
  }

  return collisionFound;
}

//==============================================================================
double DartCollisionEngine::distance(
    const ObjectList& objects,
    const DistanceOption& option,
    DistanceResult* result) const
{
  if (result)
    result->clear();

  if (objects.size() < 2u)
    return 0.0;

  const auto& filter = option.distanceFilter;
  bool hasResult = false;
  DistanceInfo bestInfo;
  const CollisionObject* bestObj1 = nullptr;
  const CollisionObject* bestObj2 = nullptr;
  double bestDistance = std::numeric_limits<double>::infinity();
  bool done = false;

  for (std::size_t i = 0; i + 1u < objects.size() && !done; ++i) {
    auto* obj1 = objects[i];
    const auto& core1
        = static_cast<const DARTCollisionObject*>(obj1)->getCoreObject();
    if (core1.shape.type == CoreShapeType::kNone
        || core1.shape.type == CoreShapeType::kUnsupported) {
      continue;
    }

    for (std::size_t j = i + 1u; j < objects.size(); ++j) {
      auto* obj2 = objects[j];
      if (filter && !filter->needDistance(obj2, obj1))
        continue;

      const auto& core2
          = static_cast<const DARTCollisionObject*>(obj2)->getCoreObject();
      if (core2.shape.type == CoreShapeType::kNone
          || core2.shape.type == CoreShapeType::kUnsupported) {
        continue;
      }

      if (hasResult && bestDistance >= 0.0) {
        const double lowerBound2 = aabbDistanceSquared(core1, core2);
        if (lowerBound2 > bestDistance * bestDistance)
          continue;
      }

      DistanceInfo info;
      if (!distanceCore(core1, core2, &info))
        continue;

      if (!hasResult || info.distance < bestDistance) {
        bestDistance = info.distance;
        bestInfo = info;
        bestObj1 = obj1;
        bestObj2 = obj2;
        hasResult = true;
      }

      if (hasResult && bestDistance <= option.distanceLowerBound) {
        done = true;
        break;
      }
    }
  }

  if (!hasResult)
    return 0.0;

  if (result) {
    result->unclampedMinDistance = bestDistance;
    result->minDistance
        = std::max(bestDistance, option.distanceLowerBound);
    result->shapeFrame1 = bestObj1 ? bestObj1->getShapeFrame() : nullptr;
    result->shapeFrame2 = bestObj2 ? bestObj2->getShapeFrame() : nullptr;
    if (option.enableNearestPoints) {
      result->nearestPoint1 = bestInfo.point1;
      result->nearestPoint2 = bestInfo.point2;
    }
  }

  return std::max(bestDistance, option.distanceLowerBound);
}

//==============================================================================
double DartCollisionEngine::distance(
    const ObjectList& objects1,
    const ObjectList& objects2,
    const DistanceOption& option,
    DistanceResult* result) const
{
  if (result)
    result->clear();

  if (objects1.empty() || objects2.empty())
    return 0.0;

  const auto& filter = option.distanceFilter;
  bool hasResult = false;
  DistanceInfo bestInfo;
  const CollisionObject* bestObj1 = nullptr;
  const CollisionObject* bestObj2 = nullptr;
  double bestDistance = std::numeric_limits<double>::infinity();
  bool done = false;

  for (auto* obj1 : objects1) {
    if (done)
      break;

    const auto& core1
        = static_cast<const DARTCollisionObject*>(obj1)->getCoreObject();
    if (core1.shape.type == CoreShapeType::kNone
        || core1.shape.type == CoreShapeType::kUnsupported) {
      continue;
    }

    for (auto* obj2 : objects2) {
      if (obj1 == obj2)
        continue;

      if (filter && !filter->needDistance(obj2, obj1))
        continue;

      const auto& core2
          = static_cast<const DARTCollisionObject*>(obj2)->getCoreObject();
      if (core2.shape.type == CoreShapeType::kNone
          || core2.shape.type == CoreShapeType::kUnsupported) {
        continue;
      }

      if (hasResult && bestDistance >= 0.0) {
        const double lowerBound2 = aabbDistanceSquared(core1, core2);
        if (lowerBound2 > bestDistance * bestDistance)
          continue;
      }

      DistanceInfo info;
      if (!distanceCore(core1, core2, &info))
        continue;

      if (!hasResult || info.distance < bestDistance) {
        bestDistance = info.distance;
        bestInfo = info;
        bestObj1 = obj1;
        bestObj2 = obj2;
        hasResult = true;
      }

      if (hasResult && bestDistance <= option.distanceLowerBound) {
        done = true;
        break;
      }
    }
  }

  if (!hasResult)
    return 0.0;

  if (result) {
    result->unclampedMinDistance = bestDistance;
    result->minDistance
        = std::max(bestDistance, option.distanceLowerBound);
    result->shapeFrame1 = bestObj1 ? bestObj1->getShapeFrame() : nullptr;
    result->shapeFrame2 = bestObj2 ? bestObj2->getShapeFrame() : nullptr;
    if (option.enableNearestPoints) {
      result->nearestPoint1 = bestInfo.point1;
      result->nearestPoint2 = bestInfo.point2;
    }
  }

  return std::max(bestDistance, option.distanceLowerBound);
}

//==============================================================================
bool DartCollisionEngine::checkPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result) const
{
  CollisionResult pairResult;

  const auto* dartObj1 = static_cast<const DARTCollisionObject*>(o1);
  const auto* dartObj2 = static_cast<const DARTCollisionObject*>(o2);

  dart::collision::collideCore(
      o1, o2, dartObj1->getCoreObject(), dartObj2->getCoreObject(), pairResult);

  if (!result) {
    if (option.allowNegativePenetrationDepthContacts)
      return pairResult.isCollision();

    for (const auto& contact : pairResult.getContacts()) {
      if (contact.penetrationDepth >= 0.0)
        return true;
    }

    return false;
  }

  mergeContacts(o1, o2, option, *result, pairResult);

  return pairResult.isCollision();
}

} // namespace collision
} // namespace dart
