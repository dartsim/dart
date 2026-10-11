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

#include <dart/gui/detail/viewer_types.hpp>

#include <Eigen/Geometry>
#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Options.h>
#include <filament/Scene.h>
#include <filament/ToneMapper.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/vec3.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <array>

#include <cmath>
#include <cstdint>

namespace dart::gui::detail {
namespace {

using ::filament::math::float3;

constexpr float kKeyLightIntensity = 82000.0f;
constexpr float kFillLightIntensity = 26000.0f;
constexpr float kRimLightIntensity = 18000.0f;
// Calibrated against OSG's sole camera light at gray .25/.5/.75, without
// changing the scene exposure, tone mapper, or indirect illumination.
constexpr float kHeadlightIntensity = 70000.0f;

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

float3 shadowDirection(::filament::LightManager& manager, utils::Entity entity)
{
  const auto instance = manager.getInstance(entity);
  return manager.getShadowOptions(instance).transform
         * manager.getDirection(instance);
}

void setShadowDirection(
    ::filament::LightManager& manager,
    utils::Entity entity,
    const float3& direction)
{
  const auto instance = manager.getInstance(entity);
  auto options = manager.getShadowOptions(instance);
  options.transform = ::filament::math::quatf::fromDirectedRotation(
      normalizeOr(manager.getDirection(instance), {0, 0, -1}),
      normalizeOr(direction, {0, 0, -1}));
  manager.setShadowOptions(instance, options);
}

void updateHeadlightShadowDirection(
    ::filament::LightManager& manager, const SceneLights& lights)
{
  // OSG's default shadow technique uses fixed light1 even when the camera
  // headlight supplies direct illumination. The SDK transform affects only
  // shadows, so changing the camera does not rotate the ground shadow.
  setShadowDirection(
      manager, lights.headlight, shadowDirection(manager, lights.key));
}

} // namespace

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

void configureMainView(
    ::filament::View& view, ::filament::ColorGrading* colorGrading)
{
  view.setColorGrading(colorGrading);
  applyRenderSettings(view, dart::gui::RenderSettings{});
  view.setShadowType(::filament::ShadowType::PCF);
  ::filament::RenderQuality renderQuality;
  renderQuality.hdrColorBuffer = ::filament::QualityLevel::HIGH;
  view.setRenderQuality(renderQuality);
  // FXAA changes authored one-pixel colors by blending with the background.
  view.setAntiAliasing(::filament::AntiAliasing::NONE);
  view.setDithering(::filament::Dithering::TEMPORAL);
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
  // Cascaded PCF supplies object and ground shadows. Use the SDK's default
  // contact-shadow setting; the optional screen-space pass can self-occlude
  // flat faces, and is independent of the cascaded shadow feature.
  shadowOptions.screenSpaceContactShadows = false;
  shadowOptions.stepCount = 12;
  shadowOptions.maxShadowDistance = 0.5f;

  const float3 keyLightDirection
      = orbitLight ? orbitingKeyLightDirection(0.0, orbitPeriodSeconds)
                   : float3{-0.30f, -0.42f, -1.0f};
  const float3 fixedShadowDirection
      = orbitLight ? keyLightDirection : float3{0.0f, -1.0f, -1.0f};
  shadowOptions.transform = ::filament::math::quatf::fromDirectedRotation(
      normalizeOr(keyLightDirection, {0, 0, -1}),
      normalizeOr(fixedShadowDirection, {0, 0, -1}));
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

  shadowOptions.transform = ::filament::math::quatf::fromDirectedRotation(
      float3{0, 0, -1}, normalizeOr(fixedShadowDirection, {0, 0, -1}));
  ::filament::LightManager::Builder(::filament::LightManager::Type::DIRECTIONAL)
      .color({1.0f, 1.0f, 1.0f})
      .intensity(kHeadlightIntensity)
      .direction({0.0f, 0.0f, -1.0f})
      .castShadows(true)
      .shadowOptions(shadowOptions)
      .build(engine, lights.headlight);

  setCameraHeadlightEnabled(engine, lights, true);
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
  scene.addEntity(lights.headlight);
}

void detachSceneEnvironment(::filament::Scene& scene, const SceneLights& lights)
{
  scene.remove(lights.key);
  scene.remove(lights.fill);
  scene.remove(lights.rim);
  scene.remove(lights.headlight);
  scene.setIndirectLight(nullptr);
  scene.setSkybox(nullptr);
}

void destroySceneLights(::filament::Engine& engine, const SceneLights& lights)
{
  engine.destroy(lights.key);
  engine.destroy(lights.fill);
  engine.destroy(lights.rim);
  engine.destroy(lights.headlight);
  utils::EntityManager::get().destroy(lights.key);
  utils::EntityManager::get().destroy(lights.fill);
  utils::EntityManager::get().destroy(lights.rim);
  utils::EntityManager::get().destroy(lights.headlight);
}

void setCameraHeadlightEnabled(
    ::filament::Engine& engine, const SceneLights& sceneLights, bool enabled)
{
  auto& lights = engine.getLightManager();
  // OSG switches between the camera light and the fixed scene lights.
  // Keep indirect illumination in both modes, as its ambient component does.
  lights.setIntensity(
      lights.getInstance(sceneLights.key), enabled ? 0.0f : kKeyLightIntensity);
  lights.setIntensity(
      lights.getInstance(sceneLights.fill),
      enabled ? 0.0f : kFillLightIntensity);
  lights.setIntensity(
      lights.getInstance(sceneLights.rim), enabled ? 0.0f : kRimLightIntensity);
  lights.setIntensity(
      lights.getInstance(sceneLights.headlight),
      enabled ? kHeadlightIntensity : 0.0f);
}

void updateCameraHeadlight(
    ::filament::Engine& engine,
    const SceneLights& sceneLights,
    const float3& direction)
{
  auto& lights = engine.getLightManager();
  lights.setDirection(lights.getInstance(sceneLights.headlight), direction);
  updateHeadlightShadowDirection(lights, sceneLights);
}

void rotateSceneLights(
    ::filament::Engine& engine,
    const SceneLights& sceneLights,
    const Eigen::Vector3d& up)
{
  const Eigen::Vector3d normalized = up.allFinite() && up.squaredNorm() > 1e-12
                                         ? up.normalized().eval()
                                         : Eigen::Vector3d::UnitZ().eval();
  const auto rotation = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), normalized);
  auto& manager = engine.getLightManager();
  const auto apply
      = [&](utils::Entity entity, const Eigen::Vector3d& direction) {
          const Eigen::Vector3d rotated = rotation * direction;
          manager.setDirection(
              manager.getInstance(entity),
              {static_cast<float>(rotated.x()),
               static_cast<float>(rotated.y()),
               static_cast<float>(rotated.z())});
        };
  apply(sceneLights.key, {-0.30, -0.42, -1.0});
  apply(sceneLights.fill, {0.42, 0.18, -0.7});
  apply(sceneLights.rim, {-0.65, 0.40, -0.45});
  Eigen::Vector3d over = normalized.cross(Eigen::Vector3d::UnitX());
  if (over.squaredNorm() < 1e-24)
    over = Eigen::Vector3d::UnitZ().cross(normalized);
  over.normalize();
  const Eigen::Vector3d shadow = -normalized - over;
  setShadowDirection(
      manager,
      sceneLights.key,
      {static_cast<float>(shadow.x()),
       static_cast<float>(shadow.y()),
       static_cast<float>(shadow.z())});
  updateHeadlightShadowDirection(manager, sceneLights);
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
  setShadowDirection(
      lights,
      sceneLights.key,
      lights.getDirection(lights.getInstance(sceneLights.key)));
  updateHeadlightShadowDirection(lights, sceneLights);
}

} // namespace dart::gui::detail
