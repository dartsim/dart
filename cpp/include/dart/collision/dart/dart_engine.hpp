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

#include <map>

#include "dart/collision/dart/dart_type.hpp"
#include "dart/collision/dart/narrow_phase/type.hpp"
#include "dart/collision/engine.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class DartEngine : public Engine<Scalar_>
{
public:
  // Type aliases
  using Scalar = Scalar_;

  [[nodiscard]] static std::shared_ptr<DartEngine> Create();

  /// Destructor
  ~DartEngine() override;

  // Documentation inherited
  const std::string& get_type() const override;

  /// Get collision detector type for this class
  static const std::string& GetType();

  // Documentation inherited
  bool collide(
      Object<Scalar>* object1,
      Object<Scalar>* object2,
      const CollisionOption<Scalar>& option = {},
      CollisionResult<Scalar>* result = nullptr) override;

  void print(std::ostream& os = std::cout, int indent = 0) const override;

protected:
  /// Constructor
  DartEngine(
      common::MemoryManager& memory_manager
      = common::MemoryManager::GetDefault());

  // Documentation inherited
  Scene<Scalar>* create_scene_impl() override;

  // Documentation inherited
  void destroy_scene_impl(Scene<Scalar>* scene) override;

  // Documentation inherited
  const common::ArrayForBasePtr<Scene<Scalar>>& get_scenes() const override;

  // Documentation inherited
  common::ArrayForBasePtr<Scene<Scalar>>& get_mutable_scenes() override;

private:
  friend class DartScene<Scalar>;

  common::ArrayForDerivedPtr<Scene<Scalar>, DartScene<Scalar>> m_scenes;

  // DART_REGISTER_ENGINE_IN_HEADER(DartEngine<Scalar>);
};

// DART_REGISTER_ENGINE_OUT_HEADER(DartEngine<Scalar>);
DART_TEMPLATE_CLASS_HEADER(COLLISION, DartEngine)

} // namespace collision
} // namespace dart

#include "dart/collision/dart/detail/dart_engine_impl.hpp"
