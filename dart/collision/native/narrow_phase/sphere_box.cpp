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

#include <dart/collision/native/narrow_phase/sphere_box.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::collision::native {

namespace {

[[nodiscard]] bool hasIdentityRotation(const Eigen::Isometry3d& transform)
{
  const auto& rotation = transform.linear();
  return rotation(0, 0) == 1.0 && rotation(0, 1) == 0.0 && rotation(0, 2) == 0.0
         && rotation(1, 0) == 0.0 && rotation(1, 1) == 1.0
         && rotation(1, 2) == 0.0 && rotation(2, 0) == 0.0
         && rotation(2, 1) == 0.0 && rotation(2, 2) == 1.0;
}

[[nodiscard]] double computeBoundaryTolerance(
    const Eigen::Vector3d& boxHalfExtents,
    const Eigen::Vector3d& localSphereCenter)
{
  const double boundaryScale = std::max(
      {1.0,
       boxHalfExtents.cwiseAbs().maxCoeff(),
       localSphereCenter.cwiseAbs().maxCoeff()});
  return 64.0 * std::numeric_limits<double>::epsilon() * boundaryScale;
}

void snapNearBoxBoundary(
    Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& boxHalfExtents,
    double tolerance)
{
  for (int axis = 0; axis < 3; ++axis) {
    const double extent = boxHalfExtents[axis];
    const double coordinate = localPoint[axis];
    if (std::abs(std::abs(coordinate) - extent) <= tolerance) {
      localPoint[axis] = std::copysign(extent, coordinate);
    }
  }
}

bool sphereOverlapsBox(
    const Eigen::Vector3d& sphereCenter,
    double sphereRadius,
    const Eigen::Vector3d& boxHalfExtents,
    const Eigen::Isometry3d& boxTransform)
{
  const Eigen::Vector3d localSphereCenter
      = boxTransform.linear().transpose()
        * (sphereCenter - boxTransform.translation());

  const double closestX = std::clamp(
      localSphereCenter.x(), -boxHalfExtents.x(), boxHalfExtents.x());
  const double closestY = std::clamp(
      localSphereCenter.y(), -boxHalfExtents.y(), boxHalfExtents.y());
  const double closestZ = std::clamp(
      localSphereCenter.z(), -boxHalfExtents.z(), boxHalfExtents.z());

  const double dx = localSphereCenter.x() - closestX;
  const double dy = localSphereCenter.y() - closestY;
  const double dz = localSphereCenter.z() - closestZ;
  return dx * dx + dy * dy + dz * dz <= sphereRadius * sphereRadius;
}

bool collideSphereTranslatedBox(
    const Eigen::Vector3d& sphereCenter,
    double sphereRadius,
    const Eigen::Vector3d& boxHalfExtents,
    const Eigen::Vector3d& boxTranslation,
    CollisionResult& result)
{
  const Eigen::Vector3d localSphereCenter = sphereCenter - boxTranslation;

  if (localSphereCenter.x() == 0.0 && localSphereCenter.y() == 0.0
      && localSphereCenter.z() == 0.0) {
    double minDist = boxHalfExtents.x();
    int minAxis = 0;
    if (boxHalfExtents.y() < minDist) {
      minDist = boxHalfExtents.y();
      minAxis = 1;
    }
    if (boxHalfExtents.z() < minDist) {
      minDist = boxHalfExtents.z();
      minAxis = 2;
    }

    switch (minAxis) {
      case 0:
        result.addContact(
            boxTranslation.x() + boxHalfExtents.x(),
            boxTranslation.y(),
            boxTranslation.z(),
            -1.0,
            0.0,
            0.0,
            sphereRadius + minDist);
        break;
      case 1:
        result.addContact(
            boxTranslation.x(),
            boxTranslation.y() + boxHalfExtents.y(),
            boxTranslation.z(),
            0.0,
            -1.0,
            0.0,
            sphereRadius + minDist);
        break;
      default:
        result.addContact(
            boxTranslation.x(),
            boxTranslation.y(),
            boxTranslation.z() + boxHalfExtents.z(),
            0.0,
            0.0,
            -1.0,
            sphereRadius + minDist);
        break;
    }
    return true;
  }

  const double closestX = std::clamp(
      localSphereCenter.x(), -boxHalfExtents.x(), boxHalfExtents.x());
  const double closestY = std::clamp(
      localSphereCenter.y(), -boxHalfExtents.y(), boxHalfExtents.y());
  const double closestZ = std::clamp(
      localSphereCenter.z(), -boxHalfExtents.z(), boxHalfExtents.z());

  const double dx = localSphereCenter.x() - closestX;
  const double dy = localSphereCenter.y() - closestY;
  const double dz = localSphereCenter.z() - closestZ;
  const double distSquared = dx * dx + dy * dy + dz * dz;

  if (distSquared > sphereRadius * sphereRadius) {
    return false;
  }

  const double boundaryTolerance
      = computeBoundaryTolerance(boxHalfExtents, localSphereCenter);
  const bool sphereCenterInside
      = distSquared <= boundaryTolerance * boundaryTolerance;

  Eigen::Vector3d insideLocalSphereCenter = localSphereCenter;
  if (sphereCenterInside) {
    snapNearBoxBoundary(
        insideLocalSphereCenter, boxHalfExtents, boundaryTolerance);
  }

  if (sphereCenterInside
      && insideLocalSphereCenter.squaredNorm()
             <= boundaryTolerance * boundaryTolerance) {
    double minDist = boxHalfExtents.x();
    int minAxis = 0;
    if (boxHalfExtents.y() < minDist) {
      minDist = boxHalfExtents.y();
      minAxis = 1;
    }
    if (boxHalfExtents.z() < minDist) {
      minDist = boxHalfExtents.z();
      minAxis = 2;
    }

    switch (minAxis) {
      case 0:
        result.addContact(
            boxTranslation.x() + boxHalfExtents.x(),
            boxTranslation.y(),
            boxTranslation.z(),
            -1.0,
            0.0,
            0.0,
            sphereRadius + minDist);
        break;
      case 1:
        result.addContact(
            boxTranslation.x(),
            boxTranslation.y() + boxHalfExtents.y(),
            boxTranslation.z(),
            0.0,
            -1.0,
            0.0,
            sphereRadius + minDist);
        break;
      default:
        result.addContact(
            boxTranslation.x(),
            boxTranslation.y(),
            boxTranslation.z() + boxHalfExtents.z(),
            0.0,
            0.0,
            -1.0,
            sphereRadius + minDist);
        break;
    }
    return true;
  }

  double penetration = 0.0;

  if (sphereCenterInside) {
    double minDist = boxHalfExtents.x() - std::abs(insideLocalSphereCenter.x());
    int minAxis = 0;

    const double distY
        = boxHalfExtents.y() - std::abs(insideLocalSphereCenter.y());
    if (distY < minDist) {
      minDist = distY;
      minAxis = 1;
    }

    const double distZ
        = boxHalfExtents.z() - std::abs(insideLocalSphereCenter.z());
    if (distZ < minDist) {
      minDist = distZ;
      minAxis = 2;
    }

    const double normalSign
        = (insideLocalSphereCenter[minAxis] >= 0.0) ? 1.0 : -1.0;
    penetration = sphereRadius + minDist;

    switch (minAxis) {
      case 0:
        result.addContact(
            boxTranslation.x() + normalSign * boxHalfExtents.x(),
            boxTranslation.y() + insideLocalSphereCenter.y(),
            boxTranslation.z() + insideLocalSphereCenter.z(),
            -normalSign,
            0.0,
            0.0,
            penetration);
        break;
      case 1:
        result.addContact(
            boxTranslation.x() + insideLocalSphereCenter.x(),
            boxTranslation.y() + normalSign * boxHalfExtents.y(),
            boxTranslation.z() + insideLocalSphereCenter.z(),
            0.0,
            -normalSign,
            0.0,
            penetration);
        break;
      default:
        result.addContact(
            boxTranslation.x() + insideLocalSphereCenter.x(),
            boxTranslation.y() + insideLocalSphereCenter.y(),
            boxTranslation.z() + normalSign * boxHalfExtents.z(),
            0.0,
            0.0,
            -normalSign,
            penetration);
        break;
    }
  } else {
    const double dist = std::sqrt(distSquared);
    penetration = sphereRadius - dist;
    result.addContact(
        boxTranslation.x() + closestX,
        boxTranslation.y() + closestY,
        boxTranslation.z() + closestZ,
        -dx / dist,
        -dy / dist,
        -dz / dist,
        penetration);
  }
  return true;
}

} // namespace

bool collideSphereBox(
    const Eigen::Vector3d& sphereCenter,
    double sphereRadius,
    const Eigen::Vector3d& boxHalfExtents,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (option.maxNumContacts == 0) {
    return false;
  }

  if (!option.enableContact) {
    return sphereOverlapsBox(
        sphereCenter, sphereRadius, boxHalfExtents, boxTransform);
  }

  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  if (hasIdentityRotation(boxTransform)) {
    return collideSphereTranslatedBox(
        sphereCenter,
        sphereRadius,
        boxHalfExtents,
        boxTransform.translation(),
        result);
  }

  const Eigen::Matrix3d& boxRotation = boxTransform.linear();
  const Eigen::Vector3d& boxTranslation = boxTransform.translation();
  Eigen::Vector3d localSphereCenter
      = boxRotation.transpose() * (sphereCenter - boxTranslation);

  Eigen::Vector3d closestPointLocal;
  closestPointLocal.x() = std::clamp(
      localSphereCenter.x(), -boxHalfExtents.x(), boxHalfExtents.x());
  closestPointLocal.y() = std::clamp(
      localSphereCenter.y(), -boxHalfExtents.y(), boxHalfExtents.y());
  closestPointLocal.z() = std::clamp(
      localSphereCenter.z(), -boxHalfExtents.z(), boxHalfExtents.z());

  Eigen::Vector3d diff = localSphereCenter - closestPointLocal;
  double distSquared = diff.squaredNorm();

  if (distSquared > sphereRadius * sphereRadius) {
    return false;
  }

  const double boundaryTolerance
      = computeBoundaryTolerance(boxHalfExtents, localSphereCenter);
  bool sphereCenterInside
      = distSquared <= boundaryTolerance * boundaryTolerance;

  Eigen::Vector3d normal;
  double penetration;
  Eigen::Vector3d contactPoint;

  if (sphereCenterInside) {
    Eigen::Vector3d insideLocalSphereCenter = localSphereCenter;
    snapNearBoxBoundary(
        insideLocalSphereCenter, boxHalfExtents, boundaryTolerance);

    double minDist = boxHalfExtents.x() - std::abs(insideLocalSphereCenter.x());
    int minAxis = 0;

    double distY = boxHalfExtents.y() - std::abs(insideLocalSphereCenter.y());
    if (distY < minDist) {
      minDist = distY;
      minAxis = 1;
    }

    double distZ = boxHalfExtents.z() - std::abs(insideLocalSphereCenter.z());
    if (distZ < minDist) {
      minDist = distZ;
      minAxis = 2;
    }

    Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
    localNormal[minAxis] = (insideLocalSphereCenter[minAxis] >= 0) ? 1.0 : -1.0;

    normal = boxRotation * localNormal;
    penetration = sphereRadius + minDist;

    Eigen::Vector3d localContactPoint = insideLocalSphereCenter;
    localContactPoint[minAxis] = (insideLocalSphereCenter[minAxis] >= 0)
                                     ? boxHalfExtents[minAxis]
                                     : -boxHalfExtents[minAxis];
    contactPoint = boxTranslation + boxRotation * localContactPoint;
  } else {
    double dist = std::sqrt(distSquared);

    Eigen::Vector3d localNormal = diff / dist;
    normal = boxRotation * localNormal;

    penetration = sphereRadius - dist;
    contactPoint = boxTranslation + boxRotation * closestPointLocal;
  }

  normal = -normal;

  result.addContact(contactPoint, normal, penetration);
  return true;
}

bool collideSphereBox(
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  return collideSphereBox(
      sphereTransform.translation(),
      detail::getRadius(sphere),
      detail::getHalfExtents(box),
      boxTransform,
      result,
      option);
}

} // namespace dart::collision::native
