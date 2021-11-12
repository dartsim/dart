/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <limits>

#include "dart/collision/object.hpp"
#include "dart/collision/scene.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart::collision {

//==============================================================================
template <typename Scalar>
Scene<Scalar>::Scene(Engine<Scalar>* engine) : m_engine(engine)
{
  assert(m_engine);
}

//==============================================================================
template <typename Scalar>
Engine<Scalar>* Scene<Scalar>::get_mutable_engine()
{
  return m_engine;
}

//==============================================================================
template <typename Scalar>
const Engine<Scalar>* Scene<Scalar>::get_engine() const
{
  return m_engine;
}

//==============================================================================
template <typename Scalar>
ObjectPtr<Scalar> Scene<Scalar>::create_object(math::GeometryPtr shape)
{
  // Check object ID
  if (m_next_collision_object_id == std::numeric_limits<ObjectId>::max()) {
    // TODO(JS): Improve
    DART_FATAL("Collision object ID reached to the max value.");
    return nullptr;
  }

  // Create new object
  auto new_object = create_object_impl(std::move(shape));
  if (!new_object) {
    DART_WARN(
        "Failed to create a new collision object by the underlying collision "
        "detection engine.");
    return nullptr;
  }

  // Set ID
  new_object->m_id = m_next_collision_object_id++;

  // TODO(JS): Store the created object (maybe introduce a tailored container?)

  return new_object;
}

//==============================================================================
template <typename Scalar>
void Scene<Scalar>::remove_object(ObjectPtr<Scalar> object)
{
  (void)object;
  DART_NOT_IMPLEMENTED;
}

//==============================================================================
template <typename Scalar>
template <typename... Args>
ObjectPtr<Scalar> Scene<Scalar>::create_sphere_object(Args&&... args)
{
  auto geometry
      = std::make_shared<math::Sphere<Scalar>>(std::forward<Args>(args)...);
  if (!geometry) {
    return nullptr;
  }

  return create_object(std::move(geometry));
}

} // namespace dart::collision
