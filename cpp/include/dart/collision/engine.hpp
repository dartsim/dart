/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#include <memory>
#include <string>
#include <unordered_map>

#include "dart/collision/export.hpp"
#include "dart/collision/type.hpp"
#include "dart/common/macro.hpp"

namespace dart::collision {

template <typename Scalar_>
class Engine
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Destructor
  virtual ~Engine();

  /// Returns collision detection engine type as a std::string.
  virtual const std::string& get_type() const = 0;

  /// Creates a collision scene.
  virtual ScenePtr<Scalar> create_scene() = 0;

  /// Create an collision object for a geometry type
  template <typename GeometryType, typename... Args>
  ObjectPtr<Scalar> create_object(Args&&... args);

  /// Create an collision object for a sphere shape
  template <typename... Args>
  ObjectPtr<Scalar> create_sphere_object(Args&&... args);

  /// Performs narrow phase collision detection
  virtual bool collide(
      ObjectPtr<Scalar> object1,
      ObjectPtr<Scalar> object2,
      const CollisionOption<Scalar>& option = {},
      CollisionResult<Scalar>* result = nullptr)
      = 0;

protected:
  /// Constructor
  Engine() = default;

private:
  Scene<Scalar>* get_default_scene();

  ScenePtr<Scalar> m_default_scene;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, Engine)

} // namespace dart::collision

#include "dart/collision/detail/engine_impl.hpp"
