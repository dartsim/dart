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

#pragma once

#include <dart/gui/detail/backend_fwd.hpp>
#include <dart/gui/detail/viewer_types.hpp>

#include <math/vec3.h>
#include <utils/Entity.h>

namespace dart::gui::detail {

struct SceneLights
{
  utils::Entity key;
  utils::Entity fill;
  utils::Entity rim;
  utils::Entity headlight;
};

::filament::ColorGrading* createDebugColorGrading(::filament::Engine& engine);

::filament::IndirectLight* createNeutralIndirectLight(
    ::filament::Engine& engine);

void configureMainView(
    ::filament::View& view, ::filament::ColorGrading* colorGrading);

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

DART_GUI_API SceneLights createSceneLights(
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

DART_GUI_API void destroySceneLights(
    ::filament::Engine& engine, const SceneLights& lights);

DART_GUI_API void setCameraHeadlightEnabled(
    ::filament::Engine& engine, const SceneLights& lights, bool enabled);

DART_GUI_API void updateCameraHeadlight(
    ::filament::Engine& engine,
    const SceneLights& lights,
    const ::filament::math::float3& direction);

DART_GUI_API void rotateSceneLights(
    ::filament::Engine& engine,
    const SceneLights& lights,
    const Eigen::Vector3d& up);

void destroyRenderEnvironmentResources(
    ::filament::Engine& engine,
    ::filament::IndirectLight* indirectLight,
    ::filament::Skybox* skybox,
    ::filament::ColorGrading* colorGrading);

DART_GUI_API void updateOrbitingKeyLight(
    ::filament::Engine& engine,
    const SceneLights& sceneLights,
    double elapsedSeconds,
    double orbitPeriodSeconds);

} // namespace dart::gui::detail
