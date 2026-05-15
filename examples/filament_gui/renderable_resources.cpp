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

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <iostream>

#include <cstdlib>

namespace dart::examples::filament_gui {
namespace {

using filament::math::float3;
using filament::math::float4;

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

float3 selectionTint(const float3& color)
{
  return {
      std::min(1.0f, color.x + 0.35f),
      std::min(1.0f, color.y + 0.25f),
      std::max(0.10f, color.z * 0.45f)};
}

} // namespace

filament::MaterialInstance* addRenderableMaterial(
    Renderable& renderable,
    filament::Material& material,
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
    Renderable& renderable, const float4& descriptorColor, bool selected)
{
  for (Renderable::MaterialInstance& material : renderable.materials) {
    if (material.instance == nullptr || !material.hasBaseColor) {
      continue;
    }
    if (material.followsDescriptorColor) {
      material.baseColor = descriptorColor;
    }
    material.instance->setParameter(
        "baseColor",
        selected ? withRgb(material.baseColor, selectionTint(rgb(material.baseColor)))
                 : material.baseColor);
  }
}

void configureLitMaterialInstance(
    filament::MaterialInstance& material,
    const float4& color,
    float metallic,
    float roughness,
    const float3& emissiveColor,
    const PbrTextureBindings& textures,
    const TextureBinding* fallbackTexture)
{
  material.setParameter("baseColor", color);
  material.setParameter("metallic", std::clamp(metallic, 0.0f, 1.0f));
  material.setParameter("roughness", std::clamp(roughness, 0.04f, 1.0f));
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

void applyRenderableShadowSettings(
    filament::Engine& engine,
    const Renderable& renderable,
    const dart::gui::experimental::MaterialDescriptor& material)
{
  auto& renderables = engine.getRenderableManager();
  const auto instance = renderables.getInstance(renderable.entity);
  renderables.setCastShadows(instance, material.castsShadows);
  renderables.setReceiveShadows(instance, material.receivesShadows);
  renderables.setScreenSpaceContactShadows(
      instance, material.castsShadows || material.receivesShadows);
}

filament::Material& selectLitMaterial(
    const MaterialSet& materials, bool usesTextures, const float4& color)
{
  if (isTransparent(color)) {
    return usesTextures ? materials.transparentTexturedLit
                        : materials.transparentLit;
  }
  return usesTextures ? materials.texturedLit : materials.defaultLit;
}

void destroyRenderable(filament::Engine& engine, Renderable& renderable)
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

} // namespace dart::examples::filament_gui
