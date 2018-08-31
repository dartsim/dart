/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_GUI_FILAMENT_WORLDSCENE_HPP_
#define DART_GUI_FILAMENT_WORLDSCENE_HPP_

#include <unordered_map>

#include <filament/Scene.h>

#include "dart/gui/filament/ShapeFrameEntity.hpp"
#include "dart/simulation/World.hpp"

namespace dart {
namespace gui {
namespace flmt {

DART_COMMON_DECLARE_SMART_POINTERS(WorldScene)

class WorldScene
{
public:
  WorldScene(simulation::WorldPtr world);

  virtual ~WorldScene();

  void setScene(filament::Engine* engine, filament::Scene* scene);

  void refresh();

  filament::Engine* getEngine()
  {
    return mEngine;
  }

  filament::Scene* getScene()
  {
    return mScene;
  }

protected:
  void refreshSkeletons();
  void refreshBaseFrameNode(dynamics::Frame* frame);
  void refreshShapeFrameNode(dynamics::Frame* frame);
  void clearUnusedNodes();

  simulation::WorldPtr mWorld;

  filament::Engine* mEngine;
  filament::Scene* mScene;

  using NodeMap
      = std::unordered_map<dynamics::Frame*, std::unique_ptr<ShapeFrameEntity>>;

  NodeMap mFrameToNode;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_WORLDSCENE_HPP_
