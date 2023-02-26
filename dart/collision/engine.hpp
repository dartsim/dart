/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/collision/fwd.hpp>

#include <dart/common/allocator/AlignedAllocator.hpp>
#include <dart/common/allocator/Allocator.hpp>
#include <dart/common/allocator/StdAllocator.hpp>

namespace dart::collision {

template <typename S = double>
class Engine
{
public:
  using Scalar = S;

  explicit Engine(
      common::Allocator& base_allocator = common::Allocator::GetDefault());

  virtual ~Engine();

  Scene<S>* createScene();

  common::Allocator& getAllocator();

private:
  common::Allocator& m_allocator;
  std::vector<Scene<S>*, common::StdAllocator<Scene<S>*>> m_scenes;
};

} // namespace dart::collision

//==============================================================================
// Implementation
//==============================================================================

namespace dart::collision {

//==============================================================================
template <typename S>
Engine<S>::Engine(
    common::Allocator& base_allocator)
  : m_allocator(base_allocator)
{
  // Empty
}

//==============================================================================
template <typename S>
Engine<S>::~Engine()
{
  for (auto scene : m_scenes) {
    m_allocator.destroy(scene);
  }
  m_scenes.clear();
}

//==============================================================================
template <typename S>
Scene<S>* Engine<S>::createScene()
{
  auto scene = m_allocator.template construct<Scene<Scalar>>(this);
  if (!scene) {
    return nullptr;
  }

  m_scenes.push_back(scene);
  return scene;
}

//==============================================================================
template <typename S>
common::Allocator& Engine<S>::getAllocator()
{
  return m_allocator;
}

} // namespace dart::collision
