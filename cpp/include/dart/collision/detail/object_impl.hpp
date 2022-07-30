/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/collision/engine.hpp"
#include "dart/collision/object.hpp"
#include "dart/collision/scene.hpp"

namespace dart::collision {

//==============================================================================
template <typename Scalar>
Object<Scalar>::~Object()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
ObjectId Object<Scalar>::get_id() const
{
  return m_id;
}

//==============================================================================
template <typename Scalar>
Engine<Scalar>* Object<Scalar>::get_mutable_engine()
{
  return m_scene->get_mutable_engine();
}

//==============================================================================
template <typename Scalar>
const Engine<Scalar>* Object<Scalar>::get_engine() const
{
  return m_scene->get_engine();
}

//==============================================================================
template <typename Scalar>
Scene<Scalar>* Object<Scalar>::get_mutable_scene()
{
  return m_scene;
}

//==============================================================================
template <typename Scalar>
const Scene<Scalar>* Object<Scalar>::get_scene() const
{
  return m_scene;
}

//==============================================================================
template <typename Scalar>
const void* Object<Scalar>::get_user_data() const
{
  return m_user_data;
}

//==============================================================================
template <typename Scalar>
void* Object<Scalar>::get_mutable_user_data()
{
  return m_user_data;
}

//==============================================================================
template <typename Scalar>
math::ConstGeometryPtr Object<Scalar>::get_geometry() const
{
  return m_geometry;
}

//==============================================================================
template <typename Scalar>
void Object<Scalar>::set_position(Scalar x, Scalar y, Scalar z)
{
  set_position(math::R3<Scalar>(x, y, z));
}

//==============================================================================
template <typename Scalar>
bool Object<Scalar>::collide(
    Object<Scalar>* other,
    const CollisionOption<Scalar>& option,
    CollisionResult<Scalar>* result)
{
  auto engine = get_mutable_engine();
  auto other_engine = other->get_engine();
  DART_ASSERT(engine);
  DART_ASSERT(other_engine);
  if (engine != other_engine) {
    DART_WARN("Cannot check collision for objects from different engines.");
    return false;
  }

  return engine->collide(this, other, option, result);
}

//==============================================================================
template <typename Scalar>
Object<Scalar>::Object(
    Scene<Scalar>* scene, ObjectId id, math::Geometry3Ptr<Scalar> geometry)
  : m_scene(scene), m_id(id), m_geometry(std::move(geometry))
{
  // Do nothing
}

} // namespace dart::collision
