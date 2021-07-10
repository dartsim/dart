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

#include "dart/collision/engine.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
std::unordered_map<std::string, EnginePtr<S>> Engine<S>::m_engines;

//==============================================================================
template <typename S>
EnginePtr<S> Engine<S>::create(const std::string& engine_name) {
  const auto& result = m_engines.find(engine_name);
  if (result != m_engines.end()) {
    return result->second;
  }

  auto factory = SingletonFactory::getSingletonPtr();
  auto newEngine = factory->create(engine_name);

  if (!newEngine) {
    dtwarn << "Failed to create a collision detector with the given engine "
           << "name '" << engine_name << "'.\n";
    return nullptr;
  }

  m_engines[engine_name] = newEngine;

  return newEngine;
}

//==============================================================================
template <typename S>
Engine<S>::~Engine() {
  // Do nothing
}

//==============================================================================
template <typename S>
template <typename GeometryType, typename... Args>
ObjectPtr<S> Engine<S>::create_object(Args&&... args) {
  return get_default_group()->template create_object<GeometryType>(
      std::forward<Args>(args)...);
}

//==============================================================================
template <typename S>
template <typename... Args>
ObjectPtr<S> Engine<S>::create_sphere_object(Args&&... args) {
  return get_default_group()->create_sphere_object(std::forward<Args>(args)...);
}

//==============================================================================
template <typename S>
Group<S>* Engine<S>::get_default_group() {
  if (!m_default_group) {
    m_default_group = create_group();
  }

  return m_default_group.get();
}

} // namespace collision
} // namespace dart
