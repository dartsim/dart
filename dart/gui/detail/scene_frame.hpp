/*
 * Copyright (c) 2011, The DART development contributors
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

#ifndef DART_GUI_DETAIL_SCENE_FRAME_HPP_
#define DART_GUI_DETAIL_SCENE_FRAME_HPP_

#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/profile.hpp>
#include <dart/gui/renderable.hpp>

#include <optional>

struct GLFWwindow;

namespace filament {
class Engine;
class Scene;
} // namespace filament

namespace dart::gui {

struct OrbitCameraController;
struct RunOptions;
struct ViewerLifecycleState;

} // namespace dart::gui

namespace dart::gui::detail {

struct DartScene;
struct FrameViewport;
struct InitialSceneState;
struct SceneLights;
class SelectionController;
class SimulationStepper;

class SceneFrameUpdater
{
public:
  SceneFrameUpdater(
      GLFWwindow* window,
      ::filament::Engine& engine,
      ::filament::Scene& scene,
      const MaterialSet& materials,
      MaterialResources& materialResources,
      const dart::gui::RunOptions& options,
      DartScene& dartScene,
      const dart::gui::OrbitCameraController& cameraController,
      SelectionController& selectionController,
      InitialSceneState& sceneState,
      std::optional<Renderable>& selectionDebugOverlay,
      dart::gui::ViewerLifecycleState& lifecycle,
      SimulationStepper& simulationStepper,
      SceneLights& lights,
      dart::gui::ProfileAccumulator::Clock::time_point orbitStartClock,
      dart::gui::ProfileAccumulator& profile);

  void update(
      const FrameViewport& viewport,
      bool showUi,
      bool uiCapturesMouse,
      bool orbitLight,
      double orbitLightPeriodSeconds);

private:
  GLFWwindow* mWindow = nullptr;
  ::filament::Engine& mEngine;
  ::filament::Scene& mScene;
  const MaterialSet& mMaterials;
  MaterialResources& mMaterialResources;
  const dart::gui::RunOptions& mOptions;
  DartScene& mDartScene;
  const dart::gui::OrbitCameraController& mCameraController;
  SelectionController& mSelectionController;
  InitialSceneState& mSceneState;
  std::optional<Renderable>& mSelectionDebugOverlay;
  dart::gui::ViewerLifecycleState& mLifecycle;
  SimulationStepper& mSimulationStepper;
  SceneLights& mLights;
  dart::gui::ProfileAccumulator::Clock::time_point mOrbitStartClock;
  dart::gui::ProfileAccumulator& mProfile;
  dart::gui::RenderableExtractor mExtractor;
};

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_SCENE_FRAME_HPP_
