/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/collision/engine.hpp"
#include "dart/collision/scene.hpp"

namespace dart::collision {

//==============================================================================
template <typename Scalar>
Engine<Scalar>::Engine(common::MemoryManager& allocator)
  : m_memory_manager(allocator)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Engine<Scalar>::~Engine()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
template <typename GeometryType, typename... Args>
Object<Scalar>* Engine<Scalar>::create_object(Args&&... args)
{
  return get_default_scene()->template create_object_impl<GeometryType>(
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename Scalar>
template <typename... Args>
Object<Scalar>* Engine<Scalar>::create_sphere_object(Args&&... args)
{
  return get_default_scene()->create_sphere_object(std::forward<Args>(args)...);
}

//==============================================================================
template <typename Scalar>
const common::MemoryManager& Engine<Scalar>::get_memory_manager() const
{
  return m_memory_manager;
}

//==============================================================================
template <typename Scalar>
common::MemoryManager& Engine<Scalar>::get_mutable_memory_manager()
{
  return m_memory_manager;
}

//==============================================================================
template <typename Scalar>
void Engine<Scalar>::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[*::print is not implemented]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "*::print is not implemented:\n";
}

//==============================================================================
template <typename Scalar>
Scene<Scalar>* Engine<Scalar>::get_default_scene()
{
  if (!m_default_scene) {
    m_default_scene = create_scene();
  }

  return m_default_scene;
}

//==============================================================================
template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const Engine<Scalar>& engine)
{
  engine.print(os);
  return os;
}

} // namespace dart::collision
