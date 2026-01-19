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

#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>

#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

namespace dart::collision::experimental {

bool NarrowPhase::collide(
    const CollisionObject& obj1,
    const CollisionObject& obj2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const Shape* shape1 = obj1.getShape();
  const Shape* shape2 = obj2.getShape();

  if (!shape1 || !shape2) {
    return false;
  }

  ShapeType type1 = shape1->getType();
  ShapeType type2 = shape2->getType();

  const Eigen::Isometry3d& tf1 = obj1.getTransform();
  const Eigen::Isometry3d& tf2 = obj2.getTransform();

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    const auto* s1 = static_cast<const SphereShape*>(shape1);
    const auto* s2 = static_cast<const SphereShape*>(shape2);
    return collideSpheres(*s1, tf1, *s2, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    const auto* b1 = static_cast<const BoxShape*>(shape1);
    const auto* b2 = static_cast<const BoxShape*>(shape2);
    return collideBoxes(*b1, tf1, *b2, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Box) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideSphereBox(*s, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Sphere) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideSphereBox(*s, tf2, *b, tf1, result, option);
  }

  return false;
}

bool NarrowPhase::isSupported(ShapeType type1, ShapeType type2)
{
  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    return true;
  }
  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Sphere)) {
    return true;
  }
  return false;
}

}
