/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "core/SceneRegistry.hpp"

#include "scenes/basic_dynamics/BasicDynamicsScene.hpp"
#include "scenes/controlled_pendulum/ControlledPendulumScene.hpp"

#include <algorithm>

namespace dart::demo {

void SceneRegistry::add(ScenePtr scene)
{
  if (scene)
    mScenes.push_back(std::move(scene));
}

const SceneList& SceneRegistry::scenes() const
{
  return mScenes;
}

SceneList SceneRegistry::takeScenes()
{
  return std::move(mScenes);
}

ScenePtr SceneRegistry::findByKey(const std::string& key) const
{
  const auto it = std::find_if(
      mScenes.begin(), mScenes.end(), [&key](const ScenePtr& scene) {
        return scene && scene->metadata().key == key;
      });

  if (it != mScenes.end())
    return *it;

  return nullptr;
}

SceneRegistry makeDefaultSceneRegistry()
{
  SceneRegistry registry;
  registry.add(createBasicDynamicsScene());
  registry.add(createControlledPendulumScene());
  return registry;
}

} // namespace dart::demo
