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

#include <dart/collision/experimental/narrow_phase/plane_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

bool collidePlaneSphere(
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  const Eigen::Vector3d sphereCenter = sphereTransform.translation();
  const double radius = sphere.getRadius();

  const double signedDist = worldNormal.dot(sphereCenter - planePoint);

  if (signedDist > radius) {
    return false;
  }

  const double penetration = radius - signedDist;
  const Eigen::Vector3d contactPoint = sphereCenter - worldNormal * signedDist;

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = worldNormal;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

bool collidePlaneBox(
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  const Eigen::Vector3d& halfExtents = box.getHalfExtents();

  double minDist = std::numeric_limits<double>::max();
  Eigen::Vector3d deepestCorner;

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1) ? halfExtents.x() : -halfExtents.x(),
        (i & 2) ? halfExtents.y() : -halfExtents.y(),
        (i & 4) ? halfExtents.z() : -halfExtents.z());

    const Eigen::Vector3d worldCorner = boxTransform * localCorner;
    const double signedDist = worldNormal.dot(worldCorner - planePoint);

    if (signedDist < minDist) {
      minDist = signedDist;
      deepestCorner = worldCorner;
    }
  }

  if (minDist > 0.0) {
    return false;
  }

  const double penetration = -minDist;
  const Eigen::Vector3d contactPoint
      = deepestCorner + worldNormal * (penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = worldNormal;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

bool collidePlaneCapsule(
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  const double radius = capsule.getRadius();
  const double halfHeight = capsule.getHeight() * 0.5;

  const Eigen::Vector3d localTop(0, 0, halfHeight);
  const Eigen::Vector3d localBottom(0, 0, -halfHeight);

  const Eigen::Vector3d worldTop = capsuleTransform * localTop;
  const Eigen::Vector3d worldBottom = capsuleTransform * localBottom;

  const double distTop = worldNormal.dot(worldTop - planePoint);
  const double distBottom = worldNormal.dot(worldBottom - planePoint);

  const Eigen::Vector3d* closestEndpoint;
  double minDist;
  if (distTop < distBottom) {
    closestEndpoint = &worldTop;
    minDist = distTop;
  } else {
    closestEndpoint = &worldBottom;
    minDist = distBottom;
  }

  if (minDist > radius) {
    return false;
  }

  const double penetration = radius - minDist;
  const Eigen::Vector3d contactPoint
      = *closestEndpoint - worldNormal * (minDist - penetration * 0.5);

  ContactPoint contact;
  contact.position = contactPoint;
  contact.normal = worldNormal;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

} // namespace dart::collision::experimental
