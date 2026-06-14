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

#ifndef DART_GUI_DETAIL_RENDER_ENVIRONMENT_HPP_
#define DART_GUI_DETAIL_RENDER_ENVIRONMENT_HPP_

#include <utils/Entity.h>

namespace filament {

class Camera;
class ColorGrading;
class Engine;
class IndirectLight;
class Scene;
class Skybox;
class View;

} // namespace filament

namespace dart::gui {

struct OrbitCamera;
struct RenderSettings;

} // namespace dart::gui

namespace dart::gui::detail {

struct SceneLights
{
  utils::Entity key;
  utils::Entity fill;
  utils::Entity rim;
};

::filament::ColorGrading* createDebugColorGrading(::filament::Engine& engine);

::filament::IndirectLight* createNeutralIndirectLight(
    ::filament::Engine& engine);

::filament::Skybox* createNeutralSkybox(::filament::Engine& engine);

/// True when the user opted into full-fidelity rendering on the headless path
/// (the `DART_GUI_HIGH_FIDELITY` environment variable). The heavy screen-space
/// passes are normally disabled when headless because CI renders on the
/// llvmpipe software rasterizer; on a real GPU (synthetic-data / sensor
/// capture) this opt-in restores the windowed-quality passes without changing
/// the CI default.
bool highFidelityHeadlessRequested();

void configureViewQuality(::filament::View& view, bool headless);

void configureMainView(
    ::filament::View& view,
    ::filament::ColorGrading* colorGrading,
    bool headless);

void applyRenderSettings(
    ::filament::View& view, const dart::gui::RenderSettings& settings);

void clearMainViewColorGrading(::filament::View& view);

void configureViewportCamera(
    ::filament::View& view,
    ::filament::Camera& camera,
    const dart::gui::OrbitCamera& orbitCamera,
    int width,
    int height);

void configureViewportCamera(
    ::filament::View& view,
    ::filament::Camera& camera,
    const dart::gui::OrbitCamera& orbitCamera,
    int x,
    int y,
    int width,
    int height);

SceneLights createSceneLights(
    ::filament::Engine& engine,
    bool headless,
    bool orbitLight,
    double orbitPeriodSeconds);

void attachSceneEnvironment(
    ::filament::Scene& scene,
    ::filament::IndirectLight* indirectLight,
    ::filament::Skybox* skybox,
    const SceneLights& lights);

void detachSceneEnvironment(
    ::filament::Scene& scene, const SceneLights& lights);

void destroySceneLights(::filament::Engine& engine, const SceneLights& lights);

void setSceneLightsEnabled(
    ::filament::Engine& engine, const SceneLights& lights, bool enabled);

void destroyRenderEnvironmentResources(
    ::filament::Engine& engine,
    ::filament::IndirectLight* indirectLight,
    ::filament::Skybox* skybox,
    ::filament::ColorGrading* colorGrading);

void updateOrbitingKeyLight(
    ::filament::Engine& engine,
    const SceneLights& sceneLights,
    double elapsedSeconds,
    double orbitPeriodSeconds);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_RENDER_ENVIRONMENT_HPP_
