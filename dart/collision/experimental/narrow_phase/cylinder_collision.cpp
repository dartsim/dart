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

#include <dart/collision/experimental/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>
#include <limits>

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

  SegmentClosestResult result;
  result.point1 = p1 + d1 * s;
  result.point2 = p2 + d2 * t;
  result.distSq = (result.point1 - result.point2).squaredNorm();
  return result;
}

} // namespace

bool collideCylinders(
    const CylinderShape& cyl1,
    const Eigen::Isometry3d& transform1,
    const CylinderShape& cyl2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double r1 = cyl1.getRadius();
  const double h1 = cyl1.getHeight() * 0.5;
  const double r2 = cyl2.getRadius();
  const double h2 = cyl2.getHeight() * 0.5;

  const Eigen::Vector3d axis1 = transform1.rotation().col(2);
  const Eigen::Vector3d axis2 = transform2.rotation().col(2);
  const Eigen::Vector3d center1 = transform1.translation();
  const Eigen::Vector3d center2 = transform2.translation();

  const Eigen::Vector3d top1 = center1 + axis1 * h1;
  const Eigen::Vector3d bot1 = center1 - axis1 * h1;
  const Eigen::Vector3d top2 = center2 + axis2 * h2;
  const Eigen::Vector3d bot2 = center2 - axis2 * h2;

  auto closest = closestPointsBetweenSegments(bot1, top1, bot2, top2);

  const double sumRadii = r1 + r2;
  if (closest.distSq > sumRadii * sumRadii) {
    return false;
  }

  const double dist = std::sqrt(closest.distSq);
  const double penetration = sumRadii - dist;

  Eigen::Vector3d normal;
  if (dist < 1e-10) {
    normal = axis1.cross(axis2);
    if (normal.squaredNorm() < 1e-10) {
      normal = Eigen::Vector3d::UnitX();
    }
    normal.normalize();
  } else {
    normal = (closest.point1 - closest.point2) / dist;
  }

  const Eigen::Vector3d contactPoint
      = closest.point2 + normal * (r2 - penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = normal;
  contact.depth = penetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderSphere(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;
  const double sphereRadius = sphere.getRadius();

  const Eigen::Isometry3d cylInv = cylinderTransform.inverse();
  const Eigen::Vector3d sphereCenterLocal
      = cylInv * sphereTransform.translation();

  const double clampedZ
      = std::clamp(sphereCenterLocal.z(), -cylHalfHeight, cylHalfHeight);

  Eigen::Vector3d closestLocal;
  const double lateralDistSq = sphereCenterLocal.x() * sphereCenterLocal.x()
                               + sphereCenterLocal.y() * sphereCenterLocal.y();

  if (lateralDistSq <= cylRadius * cylRadius) {
    closestLocal = Eigen::Vector3d(
        sphereCenterLocal.x(), sphereCenterLocal.y(), clampedZ);
  } else {
    const double lateralDist = std::sqrt(lateralDistSq);
    const double scale = cylRadius / lateralDist;
    closestLocal = Eigen::Vector3d(
        sphereCenterLocal.x() * scale, sphereCenterLocal.y() * scale, clampedZ);
  }

  const Eigen::Vector3d diff = sphereCenterLocal - closestLocal;
  const double distSq = diff.squaredNorm();

  if (distSq > sphereRadius * sphereRadius) {
    return false;
  }

  const double dist = std::sqrt(distSq);
  const double penetration = sphereRadius - dist;

  Eigen::Vector3d normalLocal;
  if (dist < 1e-10) {
    if (std::abs(sphereCenterLocal.z()) > cylHalfHeight - 1e-6) {
      normalLocal
          = Eigen::Vector3d(0, 0, sphereCenterLocal.z() > 0 ? 1.0 : -1.0);
    } else {
      normalLocal = Eigen::Vector3d(1, 0, 0);
    }
  } else {
    normalLocal = diff / dist;
  }

  const Eigen::Vector3d normalWorld
      = cylinderTransform.rotation() * normalLocal;
  const Eigen::Vector3d closestWorld = cylinderTransform * closestLocal;
  const Eigen::Vector3d contactPoint
      = closestWorld + normalWorld * (penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = -normalWorld;
  contact.depth = penetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderBox(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;
  const Eigen::Vector3d& boxHalf = box.getHalfExtents();

  const Eigen::Isometry3d cylInv = cylinderTransform.inverse();
  const Eigen::Isometry3d boxInCyl = cylInv * boxTransform;

  double maxPenetration = -std::numeric_limits<double>::max();
  Eigen::Vector3d bestContactPoint;
  Eigen::Vector3d bestNormal;
  bool foundCollision = false;

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1) ? boxHalf.x() : -boxHalf.x(),
        (i & 2) ? boxHalf.y() : -boxHalf.y(),
        (i & 4) ? boxHalf.z() : -boxHalf.z());

    const Eigen::Vector3d cornerInCyl = boxInCyl * localCorner;

    const double lateralDistSq
        = cornerInCyl.x() * cornerInCyl.x() + cornerInCyl.y() * cornerInCyl.y();
    const double lateralDist = std::sqrt(lateralDistSq);

    const double lateralPen = cylRadius - lateralDist;
    const double axialPen = cylHalfHeight - std::abs(cornerInCyl.z());

    if (lateralPen > 0.0 && axialPen > 0.0) {
      foundCollision = true;

      double penetration;
      Eigen::Vector3d normalLocal;

      if (lateralPen < axialPen) {
        penetration = lateralPen;
        if (lateralDist > 1e-10) {
          normalLocal = Eigen::Vector3d(
              -cornerInCyl.x() / lateralDist,
              -cornerInCyl.y() / lateralDist,
              0);
        } else {
          normalLocal = Eigen::Vector3d(-1, 0, 0);
        }
      } else {
        penetration = axialPen;
        normalLocal = Eigen::Vector3d(0, 0, cornerInCyl.z() > 0 ? -1.0 : 1.0);
      }

      if (penetration > maxPenetration) {
        maxPenetration = penetration;
        bestContactPoint = cornerInCyl;
        bestNormal = normalLocal;
      }
    }
  }

  if (!foundCollision) {
    return false;
  }

  const Eigen::Vector3d normalWorld = cylinderTransform.rotation() * bestNormal;
  const Eigen::Vector3d contactWorld = cylinderTransform * bestContactPoint
                                       + normalWorld * (maxPenetration * 0.5);

  ContactPoint contact;
  contact.position = contactWorld;
  contact.normal = normalWorld;
  contact.depth = maxPenetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderCapsule(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;
  const double capRadius = capsule.getRadius();
  const double capHalfHeight = capsule.getHeight() * 0.5;

  const Eigen::Vector3d capAxis = capsuleTransform.rotation().col(2);
  const Eigen::Vector3d capCenter = capsuleTransform.translation();
  const Eigen::Vector3d capTop = capCenter + capAxis * capHalfHeight;
  const Eigen::Vector3d capBot = capCenter - capAxis * capHalfHeight;

  const Eigen::Isometry3d cylInv = cylinderTransform.inverse();
  const Eigen::Vector3d capTopLocal = cylInv * capTop;
  const Eigen::Vector3d capBotLocal = cylInv * capBot;

  double bestPenetration = -std::numeric_limits<double>::max();
  Eigen::Vector3d bestCylPoint;
  Eigen::Vector3d bestCapPoint;
  bool foundCollision = false;

  auto checkCapsuleEndpoint = [&](const Eigen::Vector3d& capEndLocal) {
    const double lateralDistSq
        = capEndLocal.x() * capEndLocal.x() + capEndLocal.y() * capEndLocal.y();
    const double lateralDist = std::sqrt(lateralDistSq);

    Eigen::Vector3d closestOnCyl;
    double dist;

    if (std::abs(capEndLocal.z()) <= cylHalfHeight) {
      if (lateralDist <= cylRadius) {
        double lateralPen = cylRadius - lateralDist;
        double axialPen = cylHalfHeight - std::abs(capEndLocal.z());
        if (lateralPen < axialPen && lateralDist > 1e-10) {
          closestOnCyl = Eigen::Vector3d(
              capEndLocal.x() * cylRadius / lateralDist,
              capEndLocal.y() * cylRadius / lateralDist,
              capEndLocal.z());
        } else {
          double cappedZ = capEndLocal.z() > 0 ? cylHalfHeight : -cylHalfHeight;
          closestOnCyl
              = Eigen::Vector3d(capEndLocal.x(), capEndLocal.y(), cappedZ);
        }
      } else {
        closestOnCyl = Eigen::Vector3d(
            capEndLocal.x() * cylRadius / lateralDist,
            capEndLocal.y() * cylRadius / lateralDist,
            capEndLocal.z());
      }
    } else {
      double cappedZ
          = std::clamp(capEndLocal.z(), -cylHalfHeight, cylHalfHeight);
      if (lateralDist <= cylRadius) {
        closestOnCyl
            = Eigen::Vector3d(capEndLocal.x(), capEndLocal.y(), cappedZ);
      } else {
        closestOnCyl = Eigen::Vector3d(
            capEndLocal.x() * cylRadius / lateralDist,
            capEndLocal.y() * cylRadius / lateralDist,
            cappedZ);
      }
    }

    dist = (capEndLocal - closestOnCyl).norm();
    double penetration = capRadius - dist;

    if (penetration > 0 && penetration > bestPenetration) {
      bestPenetration = penetration;
      bestCylPoint = closestOnCyl;
      bestCapPoint = capEndLocal;
      foundCollision = true;
    }
  };

  checkCapsuleEndpoint(capTopLocal);
  checkCapsuleEndpoint(capBotLocal);

  if (!foundCollision) {
    return false;
  }

  Eigen::Vector3d normalLocal = bestCapPoint - bestCylPoint;
  if (normalLocal.squaredNorm() < 1e-10) {
    normalLocal = Eigen::Vector3d::UnitZ();
  } else {
    normalLocal.normalize();
  }

  const Eigen::Vector3d normalWorld
      = cylinderTransform.rotation() * normalLocal;
  const Eigen::Vector3d contactWorld = cylinderTransform * bestCylPoint
                                       + normalWorld * (bestPenetration * 0.5);

  ContactPoint contact;
  contact.position = contactWorld;
  contact.normal = -normalWorld;
  contact.depth = bestPenetration;

  result.addContact(contact);
  return true;
}

bool collideCylinderPlane(
    const CylinderShape& cylinder,
    const Eigen::Isometry3d& cylinderTransform,
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double cylRadius = cylinder.getRadius();
  const double cylHalfHeight = cylinder.getHeight() * 0.5;

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  const Eigen::Vector3d cylAxis = cylinderTransform.rotation().col(2);
  const Eigen::Vector3d cylCenter = cylinderTransform.translation();

  const Eigen::Vector3d cylTop = cylCenter + cylAxis * cylHalfHeight;
  const Eigen::Vector3d cylBot = cylCenter - cylAxis * cylHalfHeight;

  const double dotAxis = cylAxis.dot(worldNormal);
  const Eigen::Vector3d radialDir
      = (worldNormal - cylAxis * dotAxis).normalized();

  Eigen::Vector3d deepestPoint;
  if (radialDir.squaredNorm() < 1e-10) {
    const double distTop = worldNormal.dot(cylTop - planePoint);
    const double distBot = worldNormal.dot(cylBot - planePoint);
    deepestPoint = (distTop < distBot) ? cylTop : cylBot;
  } else {
    const Eigen::Vector3d rimOffset = -radialDir * cylRadius;
    const Eigen::Vector3d topRim = cylTop + rimOffset;
    const Eigen::Vector3d botRim = cylBot + rimOffset;

    const double distTopRim = worldNormal.dot(topRim - planePoint);
    const double distBotRim = worldNormal.dot(botRim - planePoint);

    deepestPoint = (distTopRim < distBotRim) ? topRim : botRim;
  }

  const double signedDist = worldNormal.dot(deepestPoint - planePoint);

  if (signedDist > 0.0) {
    return false;
  }

  const double penetration = -signedDist;
  const Eigen::Vector3d contactPoint
      = deepestPoint + worldNormal * (penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = worldNormal;
  contact.depth = penetration;

  result.addContact(contact);
  return true;
}

} // namespace dart::collision::experimental
