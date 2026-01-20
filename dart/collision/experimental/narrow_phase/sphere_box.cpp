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

#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

bool collideSphereBox(
    const Eigen::Vector3d& sphereCenter,
    double sphereRadius,
    const Eigen::Vector3d& boxHalfExtents,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  Eigen::Vector3d localSphereCenter = boxTransform.inverse() * sphereCenter;

  Eigen::Vector3d closestPointLocal;
  closestPointLocal.x() = std::clamp(
      localSphereCenter.x(), -boxHalfExtents.x(), boxHalfExtents.x());
  closestPointLocal.y() = std::clamp(
      localSphereCenter.y(), -boxHalfExtents.y(), boxHalfExtents.y());
  closestPointLocal.z() = std::clamp(
      localSphereCenter.z(), -boxHalfExtents.z(), boxHalfExtents.z());

  Eigen::Vector3d diff = localSphereCenter - closestPointLocal;
  double distSquared = diff.squaredNorm();

  bool sphereCenterInside = (closestPointLocal == localSphereCenter);

  if (!sphereCenterInside && distSquared > sphereRadius * sphereRadius) {
    return false;
  }

  Eigen::Vector3d normal;
  double penetration;
  Eigen::Vector3d contactPoint;

  if (sphereCenterInside) {
    double minDist = boxHalfExtents.x() - std::abs(localSphereCenter.x());
    int minAxis = 0;

    double distY = boxHalfExtents.y() - std::abs(localSphereCenter.y());
    if (distY < minDist) {
      minDist = distY;
      minAxis = 1;
    }

    double distZ = boxHalfExtents.z() - std::abs(localSphereCenter.z());
    if (distZ < minDist) {
      minDist = distZ;
      minAxis = 2;
    }

    Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
    localNormal[minAxis] = (localSphereCenter[minAxis] >= 0) ? 1.0 : -1.0;

    normal = boxTransform.rotation() * localNormal;
    penetration = sphereRadius + minDist;

    Eigen::Vector3d localContactPoint = localSphereCenter;
    localContactPoint[minAxis] = (localSphereCenter[minAxis] >= 0)
                                     ? boxHalfExtents[minAxis]
                                     : -boxHalfExtents[minAxis];
    contactPoint = boxTransform * localContactPoint;
  } else {
    double dist = std::sqrt(distSquared);

    Eigen::Vector3d localNormal = diff / dist;
    normal = boxTransform.rotation() * localNormal;

    penetration = sphereRadius - dist;
    contactPoint = boxTransform * closestPointLocal;
  }

  normal = -normal;

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = normal;
  contact.depth = penetration;

  result.addContact(contact);

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
      sphere.getRadius(),
      box.getHalfExtents(),
      boxTransform,
      result,
      option);
}

} // namespace dart::collision::experimental
