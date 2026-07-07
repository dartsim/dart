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

#include <dart/collision/native/narrow_phase/BoxBox.hpp>
#include <dart/collision/native/narrow_phase/NarrowPhase.hpp>
#include <dart/collision/native/narrow_phase/SphereBox.hpp>
#include <dart/collision/native/narrow_phase/SphereSphere.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <stdexcept>

namespace dart::collision::native {

namespace {

template <typename CollideFn>
bool collideWithFlippedNormals(
    CollisionResult& result,
    const CollisionOption& option,
    CollideFn&& collideFn)
{
  const auto existingContacts = result.numContacts();
  if (option.enableContact && existingContacts >= option.maxNumContacts) {
    return false;
  }

  CollisionOption localOption = option;
  if (option.enableContact) {
    localOption.maxNumContacts = option.maxNumContacts - existingContacts;
  }

  CollisionResult localResult;
  const bool hit = collideFn(localResult, localOption);
  if (!hit) {
    return false;
  }

  const auto numContacts = localResult.numContacts();
  for (std::size_t i = 0; i < numContacts; ++i) {
    ContactPoint contact = localResult.getContact(i);
    contact.normal = -contact.normal;
    result.addContact(contact);
  }

  return true;
}

bool collideShapes(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const CollisionOption& option,
    CollisionResult& result)
{
  if (!shape1 || !shape2) {
    return false;
  }

  if (option.maxNumContacts == 0) {
    return false;
  }

  ShapeType type1 = shape1->getType();
  ShapeType type2 = shape2->getType();

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
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideSphereBox(*s, tf1, *b, tf2, local, opt);
        });
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Sphere) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideSphereBox(*s, tf2, *b, tf1, result, option);
  }

  return false;
}

bool collideBatchShapes(
    span<const NarrowPhasePair> pairs,
    span<CollisionResult> results,
    span<bool> hits,
    bool recordHits,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "NarrowPhase::collideBatch requires one result for each pair");
  }
  if (recordHits && hits.size() < pairs.size()) {
    throw std::invalid_argument(
        "NarrowPhase::collideBatch requires one hit flag for each pair");
  }

  bool anyHit = false;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument(
          "NarrowPhase::collideBatch received a null shape");
    }

    const bool hit = collideShapes(
        pair.shapeA, pair.tfA, pair.shapeB, pair.tfB, option, results[i]);
    if (recordHits) {
      hits[i] = hit;
    }
    anyHit |= hit;
  }

  return anyHit;
}

} // namespace

bool NarrowPhase::collide(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const CollisionOption& option,
    CollisionResult& result)
{
  return collideShapes(shape1, tf1, shape2, tf2, option, result);
}

bool NarrowPhase::collideBatch(
    span<const NarrowPhasePair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  return collideBatchShapes(pairs, results, span<bool>(), false, option);
}

bool NarrowPhase::collideBatch(
    span<const NarrowPhasePair> pairs,
    span<CollisionResult> results,
    span<bool> hits,
    const CollisionOption& option)
{
  return collideBatchShapes(pairs, results, hits, true, option);
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

} // namespace dart::collision::native
