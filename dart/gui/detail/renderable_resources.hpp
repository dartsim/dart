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
#include <dart/gui/detail/renderable.hpp>
#include <dart/gui/detail/textures.hpp>

#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/Entity.h>

#include <optional>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::gui::detail {

struct MaterialSet
{
  ::filament::Material& defaultLit;
  ::filament::Material& texturedLit;
  ::filament::Material& transparentLit;
  ::filament::Material& transparentTexturedLit;
  ::filament::Material& debugColor;
  ::filament::Material& pointColor;
  ::filament::Material& opaquePointColor;
  ::filament::Material& legacyLit;
  ::filament::Material& legacyTexturedLit;
  ::filament::Material& legacyTransparentLit;
  ::filament::Material& legacyTransparentTexturedLit;
  TextureBinding checkerTexture;
  TextureBinding fallbackTexture;
};

struct MaterialResources
{
  ::filament::Material* defaultLit = nullptr;
  ::filament::Material* texturedLit = nullptr;
  ::filament::Material* transparentLit = nullptr;
  ::filament::Material* transparentTexturedLit = nullptr;
  ::filament::Material* debugColor = nullptr;
  ::filament::Material* pointColor = nullptr;
  ::filament::Material* opaquePointColor = nullptr;
  ::filament::Material* legacyLit = nullptr;
  ::filament::Material* legacyTexturedLit = nullptr;
  ::filament::Material* legacyTransparentLit = nullptr;
  ::filament::Material* legacyTransparentTexturedLit = nullptr;
  TextureBinding checkerTexture;
  TextureBinding fallbackTexture;
  TextureCache textureCache;

  MaterialSet materialSet();
};

struct Renderable
{
  utils::Entity entity;
  ::filament::VertexBuffer* vertexBuffer = nullptr;
  ::filament::IndexBuffer* indexBuffer = nullptr;
  struct MaterialInstance
  {
    ::filament::MaterialInstance* instance = nullptr;
    ::filament::math::float4 baseColor{0.0f, 0.0f, 0.0f, 1.0f};
    bool hasBaseColor = false;
    bool followsDescriptorColor = false;
  };
  std::vector<MaterialInstance> materials;
  std::vector<::filament::Texture*> sampledTextures;
  std::vector<::filament::MaterialInstance*> originalPrimitiveMaterials;
  struct GeometryRange
  {
    std::size_t offset = 0, count = 0;
    bool triangles = true;
    bool wireframeEligible = true;
  };
  std::vector<std::uint32_t> cpuIndices;
  std::vector<GeometryRange> geometryRanges;
  ::filament::IndexBuffer* wireframeIndexBuffer = nullptr;
  std::vector<GeometryRange> wireframeRanges;
  struct ScreenGeometryState
  {
    Eigen::Matrix4d worldTransform;
    Eigen::Vector3d eye, center, up;
    Eigen::Vector4d rgba;
    double fovy = 0;
    int viewportHeight = 0;
  };
  std::optional<ScreenGeometryState> screenGeometryState;
  std::size_t meshScreenVertexOffset = 0;
  std::vector<MeshPartDescriptor> meshScreenParts;
  bool depthApplied = false;
  bool wireframeApplied = false;
  bool wireframeEligible = true;
};

struct SceneRenderable
{
  dart::gui::RenderableId id = 0;
  std::size_t shapeVersion = 0;
  std::size_t renderResourceVersion = 0;
  Renderable renderable;
};

DART_GUI_API MaterialResources
createMaterialResources(::filament::Engine& engine);

DART_GUI_API void destroyMaterialResources(
    ::filament::Engine& engine, MaterialResources& resources);

::filament::MaterialInstance* addRenderableMaterial(
    Renderable& renderable,
    ::filament::Material& material,
    const std::optional<::filament::math::float4>& baseColor = std::nullopt,
    bool followsDescriptorColor = false);

void updateRenderableSelection(
    Renderable& renderable,
    const ::filament::math::float4& descriptorColor,
    bool selected,
    const std::optional<::filament::math::float4>& overrideColor
    = std::nullopt);

void configureLitMaterialInstance(
    ::filament::MaterialInstance& material,
    const ::filament::math::float4& color,
    float metallic,
    float roughness,
    const ::filament::math::float3& emissiveColor,
    const PbrTextureBindings& textures = {},
    const TextureBinding* fallbackTexture = nullptr);

void applyRenderableShadowSettings(
    ::filament::Engine& engine,
    const Renderable& renderable,
    const dart::gui::MaterialDescriptor& material);

/// Applies a descriptor's optional PBR overrides (metallic/roughness/
/// reflectance) to every lit material instance of a renderable. Unset values
/// are left at the creator's per-shape default. Call only for shapes that use a
/// lit material (not unlit line/point/voxel renderables or asset meshes).
void applyDescriptorMaterialOverride(
    Renderable& renderable, const dart::gui::MaterialDescriptor& material);

::filament::Material& selectLitMaterial(
    const MaterialSet& materials, bool usesTextures, bool transparent);

DART_GUI_API void destroyRenderable(
    ::filament::Engine& engine, Renderable& renderable);

} // namespace dart::gui::detail
