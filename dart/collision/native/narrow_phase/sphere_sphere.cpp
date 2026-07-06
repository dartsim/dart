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

#include <dart/collision/native/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <stdexcept>

#include <cmath>

namespace dart::collision::native {

namespace {

bool collideSpheresOnAxis(
    double center1X,
    double center1Y,
    double center1Z,
    double radius1,
    double radius2,
    double delta,
    int axis,
    CollisionResult& result)
{
  const double sumRadii = radius1 + radius2;
  const double dist = std::abs(delta);
  if (dist > sumRadii) {
    return false;
  }

  const double penetration = sumRadii - dist;

  if (dist < 1e-10) {
    result.addContact(center1X, center1Y, center1Z, 0.0, 0.0, 1.0, penetration);
  } else {
    const double normalSign = (delta > 0.0) ? -1.0 : 1.0;
    const double pointOffset = normalSign * (-radius1 + penetration * 0.5);
    switch (axis) {
      case 0:
        result.addContact(
            center1X + pointOffset,
            center1Y,
            center1Z,
            normalSign,
            0.0,
            0.0,
            penetration);
        break;
      case 1:
        result.addContact(
            center1X,
            center1Y + pointOffset,
            center1Z,
            0.0,
            normalSign,
            0.0,
            penetration);
        break;
      default:
        result.addContact(
            center1X,
            center1Y,
            center1Z + pointOffset,
            0.0,
            0.0,
            normalSign,
            penetration);
        break;
    }
  }
  return true;
}

bool spheresOverlap(
    double dx, double dy, double dz, double radius1, double radius2)
{
  const double sumRadii = radius1 + radius2;
  return dx * dx + dy * dy + dz * dz <= sumRadii * sumRadii;
}

bool collideSphereCenters(
    const Eigen::Vector3d& center1,
    double radius1,
    const Eigen::Vector3d& center2,
    double radius2,
    CollisionResult& result,
    const CollisionOption& option)
{
  const double dx = center2.x() - center1.x();
  const double dy = center2.y() - center1.y();
  const double dz = center2.z() - center1.z();

  // A zero contact limit short-circuits detection and returns false (see the
  // CollisionOption::maxNumContacts contract); this must precede the
  // enableContact binary-check path.
  if (option.maxNumContacts == 0) {
    return false;
  }

  if (!option.enableContact) {
    return spheresOverlap(dx, dy, dz, radius1, radius2);
  }

  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  if (dy == 0.0 && dz == 0.0) {
    return collideSpheresOnAxis(
        center1.x(), center1.y(), center1.z(), radius1, radius2, dx, 0, result);
  }
  if (dx == 0.0 && dz == 0.0) {
    return collideSpheresOnAxis(
        center1.x(), center1.y(), center1.z(), radius1, radius2, dy, 1, result);
  }
  if (dx == 0.0 && dy == 0.0) {
    return collideSpheresOnAxis(
        center1.x(), center1.y(), center1.z(), radius1, radius2, dz, 2, result);
  }

  const Eigen::Vector3d diff(dx, dy, dz);
  const double distSquared = dx * dx + dy * dy + dz * dz;
  const double sumRadii = radius1 + radius2;

  if (distSquared > sumRadii * sumRadii) {
    return false;
  }

  const double dist = std::sqrt(distSquared);
  const double penetration = sumRadii - dist;

  Eigen::Vector3d normal;
  Eigen::Vector3d point;

  if (dist < 1e-10) {
    normal = Eigen::Vector3d::UnitZ();
    point = center1;
  } else {
    normal = -diff / dist;
    point = center1 + normal * (-radius1 + penetration * 0.5);
  }

  result.addContact(point, normal, penetration);

  return true;
}

} // namespace

bool collideSpheres(
    const Eigen::Vector3d& center1,
    double radius1,
    const Eigen::Vector3d& center2,
    double radius2,
    CollisionResult& result,
    const CollisionOption& option)
{
  return collideSphereCenters(
      center1, radius1, center2, radius2, result, option);
}

bool collideSpheres(
    const SphereShape& sphere1,
    const Eigen::Isometry3d& transform1,
    const SphereShape& sphere2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  const auto& translation1 = transform1.translation();
  const auto& translation2 = transform2.translation();
  const double center1X = translation1.x();
  const double center1Y = translation1.y();
  const double center1Z = translation1.z();
  const double dx = translation2.x() - center1X;
  const double dy = translation2.y() - center1Y;
  const double dz = translation2.z() - center1Z;
  const double radius1 = detail::getRadius(sphere1);
  const double radius2 = detail::getRadius(sphere2);

  // A zero contact limit short-circuits detection and returns false (see the
  // CollisionOption::maxNumContacts contract); this must precede the
  // enableContact binary-check path.
  if (option.maxNumContacts == 0) {
    return false;
  }

  if (!option.enableContact) {
    return spheresOverlap(dx, dy, dz, radius1, radius2);
  }

  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  if (dy == 0.0 && dz == 0.0) {
    const double sumRadii = radius1 + radius2;
    const double dist = std::abs(dx);
    if (dist > sumRadii) {
      return false;
    }

    const double penetration = sumRadii - dist;
    if (dist < 1e-10) {
      result.addContact(
          center1X, center1Y, center1Z, 0.0, 0.0, 1.0, penetration);
    } else {
      const double normalX = (dx > 0.0) ? -1.0 : 1.0;
      result.addContact(
          center1X + normalX * (-radius1 + penetration * 0.5),
          center1Y,
          center1Z,
          normalX,
          0.0,
          0.0,
          penetration);
    }
    return true;
  }
  if (dx == 0.0 && dz == 0.0) {
    return collideSpheresOnAxis(
        translation1.x(),
        translation1.y(),
        translation1.z(),
        radius1,
        radius2,
        dy,
        1,
        result);
  }
  if (dx == 0.0 && dy == 0.0) {
    return collideSpheresOnAxis(
        translation1.x(),
        translation1.y(),
        translation1.z(),
        radius1,
        radius2,
        dz,
        2,
        result);
  }

  return collideSphereCenters(
      translation1, radius1, translation2, radius2, result, option);
}

void collideSpheresBatch(
    span<const SpherePair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "collideSpheresBatch requires one result for each pair");
  }

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument("collideSpheresBatch received a null sphere");
    }

    [[maybe_unused]] const bool collided = collideSpheres(
        *pair.shapeA, pair.tfA, *pair.shapeB, pair.tfB, results[i], option);
  }
}

} // namespace dart::collision::native
