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

#include "render_environment.hpp"

#include <dart/gui/viewer.hpp>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Options.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/ToneMapper.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/vec3.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <array>
#include <string_view>

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace dart::gui::detail {
namespace {

using ::filament::math::float3;

constexpr float kKeyLightIntensity = 82000.0f;
constexpr float kFillLightIntensity = 26000.0f;
constexpr float kRimLightIntensity = 18000.0f;

float3 normalizeOr(const float3& vector, const float3& fallback)
{
  const float lengthSquared
      = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  if (lengthSquared <= 1e-12f) {
    return fallback;
  }

  const float inverseLength = 1.0f / std::sqrt(lengthSquared);
  return {
      vector.x * inverseLength,
      vector.y * inverseLength,
      vector.z * inverseLength};
}

} // namespace

bool highFidelityHeadlessRequested()
{
  // Keep the accepted token set in sync with isTruthyEnvironmentVariable in
  // detail/application.cpp so all DART_GUI_* switches parse identically.
  const char* value = std::getenv("DART_GUI_HIGH_FIDELITY");
  if (value == nullptr) {
    return false;
  }
  const std::string_view text(value);
  return text == "1" || text == "true" || text == "TRUE" || text == "on"
         || text == "ON" || text == "yes" || text == "YES";
}

::filament::ColorGrading* createDebugColorGrading(::filament::Engine& engine)
{
  ::filament::PBRNeutralToneMapper toneMapper;
  return ::filament::ColorGrading::Builder()
      .quality(::filament::ColorGrading::QualityLevel::HIGH)
      .toneMapper(&toneMapper)
      .luminanceScaling(true)
      .gamutMapping(true)
      .build(engine);
}

::filament::IndirectLight* createNeutralIndirectLight(
    ::filament::Engine& engine)
{
  static constexpr std::array<float3, 9> kDiffuseIrradiance = {{
      {0.27f, 0.29f, 0.33f},
      {-0.01f, -0.01f, -0.01f},
      {0.042f, 0.046f, 0.054f},
      {0.020f, 0.022f, 0.026f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
  }};

  return ::filament::IndirectLight::Builder()
      .irradiance(3, kDiffuseIrradiance.data())
      .intensity(38000.0f)
      .build(engine);
}

::filament::Skybox* createNeutralSkybox(::filament::Engine& engine)
{
  return ::filament::Skybox::Builder()
      .color({0.30f, 0.33f, 0.38f, 1.0f})
      .showSun(true)
      .build(engine);
}

void configureNeutralViewportAtmosphere(::filament::View& view)
{
  ::filament::FogOptions fogOptions;
  fogOptions.enabled = true;
  fogOptions.distance = 24.0f;
  fogOptions.maximumOpacity = 0.16f;
  fogOptions.height = -0.6f;
  fogOptions.heightFalloff = 0.04f;
  fogOptions.color = {0.32f, 0.35f, 0.39f};
  fogOptions.density = 0.008f;
  fogOptions.inScatteringSize = 16.0f;
  view.setFogOptions(fogOptions);
}

void configureViewQuality(::filament::View& view, bool headless)
{
  ::filament::RenderQuality renderQuality;
  renderQuality.hdrColorBuffer = ::filament::QualityLevel::HIGH;
  view.setRenderQuality(renderQuality);

  // The heavy screen-space passes -- ground-truth ambient occlusion (GTAO),
  // bloom, and 4x MSAA -- are kept to the WINDOWED path, where a real GPU
  // backend is guaranteed. The headless path renders on the CI software
  // rasterizer (llvmpipe), where these multi-sample/multi-pass effects can be
  // prohibitively slow or unsupported, so headless stays light. Headless still
  // gets the cheap, shader-space improvements below (FXAA + temporal
  // dithering), which need no GPU multisample target.
  if (!headless) {
    // GTAO darkens the creases where bodies meet the ground and each other,
    // which is exactly what grounds the contact-rich rigid IPC scenes visually.
    ::filament::AmbientOcclusionOptions ambientOcclusionOptions;
    ambientOcclusionOptions.enabled = true;
    ambientOcclusionOptions.aoType
        = ::filament::AmbientOcclusionOptions::AmbientOcclusionType::GTAO;
    ambientOcclusionOptions.radius = 0.32f;
    ambientOcclusionOptions.intensity = 0.62f;
    ambientOcclusionOptions.power = 1.15f;
    ambientOcclusionOptions.quality = ::filament::QualityLevel::HIGH;
    ambientOcclusionOptions.lowPassFilter = ::filament::QualityLevel::HIGH;
    ambientOcclusionOptions.upsampling = ::filament::QualityLevel::HIGH;
    ambientOcclusionOptions.bentNormals = true;
    view.setAmbientOcclusionOptions(ambientOcclusionOptions);

    // Subtle physically-based bloom on bright specular highlights gives
    // metallic and glossy bodies energy without washing the scene out.
    ::filament::BloomOptions bloomOptions;
    bloomOptions.enabled = true;
    bloomOptions.strength = 0.08f;
    view.setBloomOptions(bloomOptions);

    ::filament::MultiSampleAntiAliasingOptions multiSampleAntiAliasingOptions;
    multiSampleAntiAliasingOptions.enabled = true;
    multiSampleAntiAliasingOptions.sampleCount = 4;
    view.setMultiSampleAntiAliasingOptions(multiSampleAntiAliasingOptions);
  }

  view.setAntiAliasing(::filament::AntiAliasing::FXAA);
  // Temporal dithering breaks up 8-bit banding in the smooth background and
  // soft shadow gradients (was NONE, which left visible banding).
  view.setDithering(::filament::Dithering::TEMPORAL);
}

void configureMainView(
    ::filament::View& view,
    ::filament::ColorGrading* colorGrading,
    bool headless)
{
  view.setColorGrading(colorGrading);
  applyRenderSettings(view, dart::gui::RenderSettings{});
  view.setShadowType(::filament::ShadowType::PCF);
  // configureViewQuality applies HDR + FXAA + temporal dithering in both modes,
  // and the GPU-heavy passes (GTAO, bloom, MSAA) only when windowed. The
  // volumetric fog is windowed-only too (it can flatten distant contrast on the
  // headless contrast smoke).
  configureViewQuality(view, headless);
  if (!headless) {
    configureNeutralViewportAtmosphere(view);
  }
}

void applyRenderSettings(
    ::filament::View& view, const dart::gui::RenderSettings& settings)
{
  view.setShadowingEnabled(settings.shadowsEnabled);
}

void clearMainViewColorGrading(::filament::View& view)
{
  view.setColorGrading(nullptr);
}

void configureViewportCamera(
    ::filament::View& view,
    ::filament::Camera& camera,
    const dart::gui::OrbitCamera& orbitCamera,
    int width,
    int height)
{
  configureViewportCamera(view, camera, orbitCamera, 0, 0, width, height);
}

void configureViewportCamera(
    ::filament::View& view,
    ::filament::Camera& camera,
    const dart::gui::OrbitCamera& orbitCamera,
    int x,
    int y,
    int width,
    int height)
{
  view.setViewport(
      {static_cast<std::int32_t>(x),
       static_cast<std::int32_t>(y),
       static_cast<std::uint32_t>(width),
       static_cast<std::uint32_t>(height)});
  const auto projection
      = dart::gui::makePerspectiveProjection(orbitCamera, width, height);
  camera.setProjection(
      projection.verticalFovDegrees,
      projection.aspectRatio,
      projection.nearPlane,
      projection.farPlane,
      ::filament::Camera::Fov::VERTICAL);
  const auto basis = dart::gui::makeOrbitCameraBasis(orbitCamera);
  const Eigen::Vector3d& eye = basis.eye;
  camera.lookAt(
      {eye.x(), eye.y(), eye.z()},
      {orbitCamera.target.x(), orbitCamera.target.y(), orbitCamera.target.z()},
      {basis.up.x(), basis.up.y(), basis.up.z()});
}

float3 orbitingKeyLightDirection(
    double elapsedSeconds, double orbitPeriodSeconds)
{
  constexpr double pi = 3.14159265358979323846;
  const double angle
      = elapsedSeconds * 2.0 * pi / std::max(orbitPeriodSeconds, 1.0);
  return normalizeOr(
      {static_cast<float>(0.68 * std::cos(angle)),
       static_cast<float>(0.68 * std::sin(angle)),
       -0.74f},
      {-0.30f, -0.42f, -1.0f});
}

SceneLights createSceneLights(
    ::filament::Engine& engine,
    bool headless,
    bool orbitLight,
    double orbitPeriodSeconds)
{
  SceneLights lights{
      utils::EntityManager::get().create(),
      utils::EntityManager::get().create(),
      utils::EntityManager::get().create()};

  ::filament::LightManager::ShadowOptions shadowOptions;
  // Keep the headless shadow atlas at the original light cost (2048 / 3
  // cascades): headless renders on the CI software rasterizer, where a larger
  // atlas and an extra cascade are pure overhead. Windowed (real GPU) gets the
  // sharper 4096 / 4-cascade shadows.
  shadowOptions.mapSize = headless ? 2048u : 4096u;
  shadowOptions.shadowCascades = headless ? 3 : 4;
  shadowOptions.cascadeSplitPositions[0] = 0.10f;
  shadowOptions.cascadeSplitPositions[1] = 0.30f;
  shadowOptions.cascadeSplitPositions[2] = 0.62f;
  shadowOptions.shadowFar = headless ? 32.0f : 48.0f;
  shadowOptions.shadowFarHint = headless ? 16.0f : 24.0f;
  // Screen-space contact shadows tighten the contact between resting/stacked
  // bodies and the surfaces they sit on -- the soft cascade shadow alone leaves
  // a small gap at the contact, which reads as "floating". This adds the short,
  // sharp contact occlusion that visually seats a body on the ground. It is an
  // extra screen-space ray-march pass, so keep it to the windowed (GPU) path
  // and off the headless software rasterizer.
  shadowOptions.screenSpaceContactShadows = !headless;
  shadowOptions.stepCount = 12;
  shadowOptions.maxShadowDistance = 0.5f;

  const float3 keyLightDirection
      = orbitLight ? orbitingKeyLightDirection(0.0, orbitPeriodSeconds)
                   : float3{-0.30f, -0.42f, -1.0f};
  ::filament::LightManager::Builder(::filament::LightManager::Type::SUN)
      .color({1.0f, 0.96f, 0.88f})
      .intensity(kKeyLightIntensity)
      .direction(keyLightDirection)
      .castShadows(true)
      .shadowOptions(shadowOptions)
      .build(engine, lights.key);

  ::filament::LightManager::Builder(::filament::LightManager::Type::DIRECTIONAL)
      .color({0.80f, 0.88f, 1.0f})
      .intensity(kFillLightIntensity)
      .direction({0.42f, 0.18f, -0.7f})
      .castShadows(false)
      .build(engine, lights.fill);

  ::filament::LightManager::Builder(::filament::LightManager::Type::DIRECTIONAL)
      .color({0.88f, 0.93f, 1.0f})
      .intensity(kRimLightIntensity)
      .direction({-0.65f, 0.40f, -0.45f})
      .castShadows(false)
      .build(engine, lights.rim);

  return lights;
}

void attachSceneEnvironment(
    ::filament::Scene& scene,
    ::filament::IndirectLight* indirectLight,
    ::filament::Skybox* skybox,
    const SceneLights& lights)
{
  scene.setIndirectLight(indirectLight);
  scene.setSkybox(skybox);
  scene.addEntity(lights.key);
  scene.addEntity(lights.fill);
  scene.addEntity(lights.rim);
}

void detachSceneEnvironment(::filament::Scene& scene, const SceneLights& lights)
{
  scene.remove(lights.key);
  scene.remove(lights.fill);
  scene.remove(lights.rim);
  scene.setIndirectLight(nullptr);
  scene.setSkybox(nullptr);
}

void destroySceneLights(::filament::Engine& engine, const SceneLights& lights)
{
  engine.destroy(lights.key);
  engine.destroy(lights.fill);
  engine.destroy(lights.rim);
  utils::EntityManager::get().destroy(lights.key);
  utils::EntityManager::get().destroy(lights.fill);
  utils::EntityManager::get().destroy(lights.rim);
}

void setSceneLightsEnabled(
    ::filament::Engine& engine, const SceneLights& sceneLights, bool enabled)
{
  auto& lights = engine.getLightManager();
  const float enabledScale = enabled ? 1.0f : 0.0f;
  lights.setIntensity(
      lights.getInstance(sceneLights.key), enabledScale * kKeyLightIntensity);
  lights.setIntensity(
      lights.getInstance(sceneLights.fill), enabledScale * kFillLightIntensity);
  lights.setIntensity(
      lights.getInstance(sceneLights.rim), enabledScale * kRimLightIntensity);
}

void destroyRenderEnvironmentResources(
    ::filament::Engine& engine,
    ::filament::IndirectLight* indirectLight,
    ::filament::Skybox* skybox,
    ::filament::ColorGrading* colorGrading)
{
  engine.destroy(indirectLight);
  engine.destroy(skybox);
  engine.destroy(colorGrading);
}

void updateOrbitingKeyLight(
    ::filament::Engine& engine,
    const SceneLights& sceneLights,
    double elapsedSeconds,
    double orbitPeriodSeconds)
{
  auto& lights = engine.getLightManager();
  lights.setDirection(
      lights.getInstance(sceneLights.key),
      orbitingKeyLightDirection(elapsedSeconds, orbitPeriodSeconds));
}

} // namespace dart::gui::detail
