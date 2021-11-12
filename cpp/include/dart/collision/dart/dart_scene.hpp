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

#include "dart/collision/dart/dart_type.hpp"
#include "dart/collision/scene.hpp"
#include "dart/collision/type.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class DartScene : public Scene<Scalar_>
{
public:
  using Scalar = Scalar_;

  friend class DartEngine<Scalar>;

  /// Constructor
  DartScene(Engine<Scalar>* engine);

  /// Destructor
  ~DartScene() override;

  ObjectPtr<Scalar> create_object(math::GeometryPtr shape) override;

  void update(Scalar time_step = 1e-3) override;

protected:
  DartEngine<Scalar>* get_mutable_dart_engine();

  const DartEngine<Scalar>* get_dart_engine() const;

private:
  friend class DartObject<Scalar>;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, DartScene)

} // namespace collision
} // namespace dart

#include "dart/collision/dart/detail/dart_scene_impl.hpp"
