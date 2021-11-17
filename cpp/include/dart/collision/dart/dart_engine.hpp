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

namespace dart::collision {

template <typename Scalar_>
class DartEngine : public Engine<Scalar_>
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Creates a new DartEngine
  [[nodiscard]] static std::shared_ptr<DartEngine> Create(
      common::MemoryManager& memory_manager
      = common::MemoryManager::GetDefault());

  /// Constructor
  ///
  /// @param[in] memory_manager: Memory manager to use for all memory
  /// allocations of the engine.
  DartEngine(
      common::MemoryManager& memory_manager
      = common::MemoryManager::GetDefault());

  /// Destructor
  ~DartEngine() override;

  // Documentation inherited
  const std::string& get_type() const override;

  /// Returns collision detector type for this class
  static const std::string& GetType();

  // Documentation inherited
  Scene<Scalar>* create_scene() override;

  // Documentation inherited
  void destroy_scene(Scene<Scalar>* scene) override;

  // Documentation inherited
  bool collide(
      Object<Scalar>* object1,
      Object<Scalar>* object2,
      const CollisionOption<Scalar>& option = {},
      CollisionResult<Scalar>* result = nullptr) override;

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

protected:
  // Documentation inherited
  const DartSceneArray<Scalar>& get_scenes() const override;

  // Documentation inherited
  DartSceneArray<Scalar>& get_mutable_scenes() override;

private:
  friend class DartScene<Scalar>;

  /// Collision scenes created by this engine.
  DartSceneArray<Scalar> m_scenes;

  // DART_REGISTER_ENGINE_IN_HEADER(DartEngine<Scalar>);
};

// DART_REGISTER_ENGINE_OUT_HEADER(DartEngine<Scalar>);
DART_TEMPLATE_CLASS_HEADER(COLLISION, DartEngine)

} // namespace dart::collision

#include "dart/collision/dart/detail/dart_engine_impl.hpp"
