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

#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <cmath>

namespace dart::collision::experimental {

bool collideSpheres(
    const Eigen::Vector3d& center1,
    double radius1,
    const Eigen::Vector3d& center2,
    double radius2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d diff = center2 - center1;
  const double distSquared = diff.squaredNorm();
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

  ContactPoint contact;
  contact.position = point;
  contact.normal = normal;
  contact.depth = penetration;

  result.addContact(contact);

  return true;
}

bool collideSpheres(
    const SphereShape& sphere1,
    const Eigen::Isometry3d& transform1,
    const SphereShape& sphere2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result,
    const CollisionOption& option)
{
  return collideSpheres(
      transform1.translation(),
      sphere1.getRadius(),
      transform2.translation(),
      sphere2.getRadius(),
      result,
      option);
}

} // namespace dart::collision::experimental
