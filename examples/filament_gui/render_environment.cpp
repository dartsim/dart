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

#include <filament/ColorGrading.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Options.h>
#include <filament/Skybox.h>
#include <filament/ToneMapper.h>
#include <filament/View.h>
#include <math/vec3.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <array>

#include <cmath>

namespace dart::examples::filament_gui {
namespace {

using filament::math::float3;

float3 normalizeOr(const float3& vector, const float3& fallback)
{
  const float lengthSquared = vector.x * vector.x + vector.y * vector.y
                              + vector.z * vector.z;
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

filament::ColorGrading* createDebugColorGrading(filament::Engine& engine)
{
  filament::PBRNeutralToneMapper toneMapper;
  return filament::ColorGrading::Builder()
      .quality(filament::ColorGrading::QualityLevel::HIGH)
      .toneMapper(&toneMapper)
      .luminanceScaling(true)
      .gamutMapping(true)
      .build(engine);
}

filament::IndirectLight* createNeutralIndirectLight(filament::Engine& engine)
{
  static constexpr std::array<float3, 9> kDiffuseIrradiance = {{
      {0.22f, 0.24f, 0.28f},
      {-0.01f, -0.01f, -0.01f},
      {0.04f, 0.045f, 0.052f},
      {0.018f, 0.020f, 0.024f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
  }};

  return filament::IndirectLight::Builder()
      .irradiance(3, kDiffuseIrradiance.data())
      .intensity(52000.0f)
      .build(engine);
}

filament::Skybox* createNeutralSkybox(filament::Engine& engine)
{
  return filament::Skybox::Builder()
      .color({0.46f, 0.50f, 0.56f, 1.0f})
      .showSun(true)
      .build(engine);
}

void configureWindowedViewQuality(filament::View& view)
{
  filament::RenderQuality renderQuality;
  renderQuality.hdrColorBuffer = filament::QualityLevel::HIGH;
  view.setRenderQuality(renderQuality);

  filament::AmbientOcclusionOptions ambientOcclusionOptions;
  ambientOcclusionOptions.enabled = true;
  ambientOcclusionOptions.aoType
      = filament::AmbientOcclusionOptions::AmbientOcclusionType::GTAO;
  ambientOcclusionOptions.radius = 0.35f;
  ambientOcclusionOptions.intensity = 0.38f;
  ambientOcclusionOptions.quality = filament::QualityLevel::MEDIUM;
  ambientOcclusionOptions.lowPassFilter = filament::QualityLevel::HIGH;
  view.setAmbientOcclusionOptions(ambientOcclusionOptions);

  filament::MultiSampleAntiAliasingOptions multiSampleAntiAliasingOptions;
  multiSampleAntiAliasingOptions.enabled = true;
  multiSampleAntiAliasingOptions.sampleCount = 4;
  view.setMultiSampleAntiAliasingOptions(multiSampleAntiAliasingOptions);
  view.setAntiAliasing(filament::AntiAliasing::FXAA);
  view.setDithering(filament::Dithering::NONE);
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
    filament::Engine& engine,
    bool headless,
    bool orbitLight,
    double orbitPeriodSeconds)
{
  SceneLights lights{
      utils::EntityManager::get().create(),
      utils::EntityManager::get().create(),
      utils::EntityManager::get().create()};

  filament::LightManager::ShadowOptions shadowOptions;
  shadowOptions.mapSize = headless ? 2048 : 4096;
  shadowOptions.shadowCascades = headless ? 3 : 4;
  shadowOptions.cascadeSplitPositions[0] = 0.10f;
  shadowOptions.cascadeSplitPositions[1] = 0.30f;
  shadowOptions.cascadeSplitPositions[2] = 0.62f;
  shadowOptions.shadowFar = headless ? 10.0f : 14.0f;
  shadowOptions.shadowFarHint = headless ? 4.5f : 5.5f;
  shadowOptions.screenSpaceContactShadows = false;

  const float3 keyLightDirection
      = orbitLight ? orbitingKeyLightDirection(0.0, orbitPeriodSeconds)
                   : float3{-0.30f, -0.42f, -1.0f};
  filament::LightManager::Builder(filament::LightManager::Type::SUN)
      .color({1.0f, 0.96f, 0.88f})
      .intensity(82000.0f)
      .direction(keyLightDirection)
      .castShadows(true)
      .shadowOptions(shadowOptions)
      .build(engine, lights.key);

  filament::LightManager::Builder(filament::LightManager::Type::DIRECTIONAL)
      .color({0.80f, 0.88f, 1.0f})
      .intensity(62000.0f)
      .direction({0.42f, 0.18f, -0.7f})
      .castShadows(false)
      .build(engine, lights.fill);

  filament::LightManager::Builder(filament::LightManager::Type::DIRECTIONAL)
      .color({0.88f, 0.93f, 1.0f})
      .intensity(42000.0f)
      .direction({-0.65f, 0.40f, -0.45f})
      .castShadows(false)
      .build(engine, lights.rim);

  return lights;
}

void updateOrbitingKeyLight(
    filament::LightManager& lights,
    const SceneLights& sceneLights,
    double elapsedSeconds,
    double orbitPeriodSeconds)
{
  lights.setDirection(
      lights.getInstance(sceneLights.key),
      orbitingKeyLightDirection(elapsedSeconds, orbitPeriodSeconds));
}

} // namespace dart::examples::filament_gui
