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

#include "dart/collision/export.hpp"
#include "dart/collision/type.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/geometry/type.hpp"

namespace dart::collision {

template <typename Scalar_>
class Scene
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Destructor
  virtual ~Scene() = default;

  /// Return collision detection engine associated with this Scene
  Engine<Scalar>* get_mutable_engine();

  /// Return (const) collision detection engine associated with this
  /// Scene
  const Engine<Scalar>* get_engine() const;

  /// Creates a collision object.
  virtual ObjectPtr<Scalar> create_object(math::GeometryPtr shape) = 0;

  /// Creates a collision object with sphere
  template <typename... Args>
  ObjectPtr<Scalar> create_sphere_object(Args&&... args);

protected:
  /// Constructor
  ///
  /// \param[in] collisionDetector: Collision detector that created this group.
  Scene(Engine<Scalar>* engine);

  /// The parent collision engine that created this scene
  Engine<Scalar>* m_engine;

private:
  /// Set this to true to have this Scene check for updates
  /// automatically. Default is true.
  bool m_update_automatically;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, Scene)

} // namespace dart::collision

#include "dart/collision/detail/scene_impl.hpp"
