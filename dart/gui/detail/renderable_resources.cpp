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

#include "renderable_resources.hpp"

#include "debug_color_material.hpp"
#include "default_lit_material.hpp"
#include "textured_lit_material.hpp"
#include "transparent_lit_material.hpp"
#include "transparent_textured_lit_material.hpp"

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <array>
#include <iostream>

#include <cstdint>
#include <cstdlib>

namespace dart::gui::detail {
namespace {

using ::filament::math::float3;
using ::filament::math::float4;

// Roughness floor for the lit material: below this Filament's specular lobe
// degenerates to a near-perfect mirror highlight. The default configuration
// and per-shape descriptor overrides must clamp to the same floor so the two
// paths cannot diverge.
constexpr float kMinLitRoughness = 0.04f;

float3 rgb(const float4& color)
{
  return {color.x, color.y, color.z};
}

float4 withRgb(const float4& color, const float3& newRgb)
{
  return {newRgb.x, newRgb.y, newRgb.z, color.w};
}

bool isTransparent(const float4& color)
{
  return color.w < 0.999f;
}

float luminance(const float3& color)
{
  return 0.2126f * color.x + 0.7152f * color.y + 0.0722f * color.z;
}

float4 ensureReadableDisplayColor(const float4& color)
{
  constexpr float kMinLuminance = 0.16f;
  const float currentLuminance = luminance(rgb(color));
  if (currentLuminance >= kMinLuminance || color.w <= 0.0f) {
    return color;
  }

  constexpr float3 kNeutralMeshColor{0.28f, 0.30f, 0.34f};
  const float blend = std::clamp(
      (kMinLuminance - currentLuminance) / kMinLuminance, 0.0f, 1.0f);
  return {
      std::clamp(
          color.x * (1.0f - blend) + kNeutralMeshColor.x * blend, 0.0f, 1.0f),
      std::clamp(
          color.y * (1.0f - blend) + kNeutralMeshColor.y * blend, 0.0f, 1.0f),
      std::clamp(
          color.z * (1.0f - blend) + kNeutralMeshColor.z * blend, 0.0f, 1.0f),
      color.w};
}

float3 selectionTint(const float3& color)
{
  return {
      std::min(1.0f, color.x + 0.35f),
      std::min(1.0f, color.y + 0.25f),
      std::max(0.10f, color.z * 0.45f)};
}

} // namespace

MaterialSet MaterialResources::materialSet()
{
  return {
      *defaultLit,
      *texturedLit,
      *transparentLit,
      *transparentTexturedLit,
      *debugColor,
      checkerTexture,
      fallbackTexture};
}

MaterialResources createMaterialResources(::filament::Engine& engine)
{
  MaterialResources resources;
  resources.defaultLit
      = ::filament::Material::Builder()
            .package(kDefaultLitMaterial, kDefaultLitMaterialSize)
            .build(engine);
  resources.texturedLit
      = ::filament::Material::Builder()
            .package(kTexturedLitMaterial, kTexturedLitMaterialSize)
            .build(engine);
  resources.transparentLit
      = ::filament::Material::Builder()
            .package(kTransparentLitMaterial, kTransparentLitMaterialSize)
            .build(engine);
  resources.transparentTexturedLit
      = ::filament::Material::Builder()
            .package(
                kTransparentTexturedLitMaterial,
                kTransparentTexturedLitMaterialSize)
            .build(engine);
  resources.debugColor
      = ::filament::Material::Builder()
            .package(kDebugColorMaterial, kDebugColorMaterialSize)
            .build(engine);
  resources.checkerTexture.texture = createCheckerTexture(engine);
  resources.fallbackTexture.texture = createSolidTexture(
      engine,
      std::array<std::uint8_t, 4>{255, 255, 255, 255},
      TextureColorSpace::Linear);
  return resources;
}

void destroyMaterialResources(
    ::filament::Engine& engine, MaterialResources& resources)
{
  for (auto* texture : resources.textureCache.ownedTextures) {
    engine.destroy(texture);
  }
  resources.textureCache.ownedTextures.clear();
  resources.textureCache.bindings.clear();

  engine.destroy(resources.fallbackTexture.texture);
  resources.fallbackTexture.texture = nullptr;
  engine.destroy(resources.checkerTexture.texture);
  resources.checkerTexture.texture = nullptr;

  engine.destroy(resources.debugColor);
  resources.debugColor = nullptr;
  engine.destroy(resources.transparentTexturedLit);
  resources.transparentTexturedLit = nullptr;
  engine.destroy(resources.transparentLit);
  resources.transparentLit = nullptr;
  engine.destroy(resources.texturedLit);
  resources.texturedLit = nullptr;
  engine.destroy(resources.defaultLit);
  resources.defaultLit = nullptr;
}

::filament::MaterialInstance* addRenderableMaterial(
    Renderable& renderable,
    ::filament::Material& material,
    const std::optional<float4>& baseColor,
    bool followsDescriptorColor)
{
  auto* instance = material.createInstance();
  renderable.materials.push_back(
      {instance,
       baseColor.value_or(float4{0.0f, 0.0f, 0.0f, 1.0f}),
       baseColor.has_value(),
       followsDescriptorColor});
  return instance;
}

void updateRenderableSelection(
    Renderable& renderable,
    const float4& descriptorColor,
    bool selected,
    const std::optional<float4>& overrideColor)
{
  for (Renderable::MaterialInstance& material : renderable.materials) {
    if (material.instance == nullptr || !material.hasBaseColor) {
      continue;
    }
    if (material.followsDescriptorColor) {
      material.baseColor = ensureReadableDisplayColor(descriptorColor);
    }
    const float4 displayColor = overrideColor.value_or(material.baseColor);
    material.instance->setParameter(
        "baseColor",
        selected ? withRgb(displayColor, selectionTint(rgb(displayColor)))
                 : displayColor);
  }
}

void configureLitMaterialInstance(
    ::filament::MaterialInstance& material,
    const float4& color,
    float metallic,
    float roughness,
    const float3& emissiveColor,
    const PbrTextureBindings& textures,
    const TextureBinding* fallbackTexture)
{
  material.setParameter("baseColor", color);
  material.setParameter("metallic", std::clamp(metallic, 0.0f, 1.0f));
  material.setParameter(
      "roughness", std::clamp(roughness, kMinLitRoughness, 1.0f));
  material.setParameter("reflectance", 0.5f);
  material.setParameter("emissiveColor", emissiveColor);
  if (hasTextureBindings(textures)) {
    if (fallbackTexture == nullptr || fallbackTexture->texture == nullptr) {
      std::cerr << "Textured Filament material is missing fallback texture\n";
      std::exit(1);
    }
    setPbrTextureParameters(material, *fallbackTexture, textures);
  }
}

void applyDescriptorMaterialOverride(
    Renderable& renderable, const dart::gui::MaterialDescriptor& material)
{
  if (!material.metallic && !material.roughness && !material.reflectance) {
    return;
  }
  for (auto& slot : renderable.materials) {
    if (slot.instance == nullptr) {
      continue;
    }
    if (material.metallic) {
      slot.instance->setParameter(
          "metallic",
          std::clamp(static_cast<float>(*material.metallic), 0.0f, 1.0f));
    }
    if (material.roughness) {
      slot.instance->setParameter(
          "roughness",
          std::clamp(
              static_cast<float>(*material.roughness), kMinLitRoughness, 1.0f));
    }
    if (material.reflectance) {
      slot.instance->setParameter(
          "reflectance",
          std::clamp(static_cast<float>(*material.reflectance), 0.0f, 1.0f));
    }
  }
}

void applyRenderableShadowSettings(
    ::filament::Engine& engine,
    const Renderable& renderable,
    const dart::gui::MaterialDescriptor& material)
{
  auto& renderables = engine.getRenderableManager();
  const auto instance = renderables.getInstance(renderable.entity);
  renderables.setCastShadows(instance, material.castsShadows);
  renderables.setReceiveShadows(instance, material.receivesShadows);
  renderables.setScreenSpaceContactShadows(
      instance, material.castsShadows || material.receivesShadows);
}

::filament::Material& selectLitMaterial(
    const MaterialSet& materials, bool usesTextures, const float4& color)
{
  if (isTransparent(color)) {
    return usesTextures ? materials.transparentTexturedLit
                        : materials.transparentLit;
  }
  return usesTextures ? materials.texturedLit : materials.defaultLit;
}

void destroyRenderable(::filament::Engine& engine, Renderable& renderable)
{
  engine.destroy(renderable.entity);
  for (Renderable::MaterialInstance& material : renderable.materials) {
    if (material.instance != nullptr) {
      engine.destroy(material.instance);
      material.instance = nullptr;
    }
  }
  renderable.materials.clear();
  engine.destroy(renderable.vertexBuffer);
  engine.destroy(renderable.indexBuffer);
  utils::EntityManager::get().destroy(renderable.entity);
}

} // namespace dart::gui::detail
