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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#include "imgui_overlay.hpp"

#include "imgui_draw_data.hpp"
#include "imgui_material.hpp"
#include "textures.hpp"

#include <backend/BufferDescriptor.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Fence.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <filesystem>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <cstring>

// The modern viewer theme below assigns Dear ImGui style slots and fields from
// the 1.89-1.91 series (e.g. ImGuiCol_NavCursor, renamed in 1.91.4). DART
// targets the bundled ImGui 1.92.x (see dart/gui/imgui/CMakeLists.txt);
// building dart::gui against an older ImGui is unsupported and fails here with
// a clear message instead of a cascade of unknown-identifier errors.
#if IMGUI_VERSION_NUM < 19280
  #error                                                                       \
      "dart::gui requires Dear ImGui >= 1.92.8 (use the bundled ImGui via DART_USE_SYSTEM_IMGUI=OFF, or upgrade the system ImGui)."
#endif

namespace dart::gui::detail {
namespace {

struct ImGuiVertex
{
  ::filament::math::float3 position;
  ::filament::math::float2 uv;
  std::uint32_t color = 0;
};

template <typename T>
::filament::backend::BufferDescriptor makeBufferDescriptor(
    std::vector<T>&& data)
{
  auto* owned = new std::vector<T>(std::move(data));
  return ::filament::backend::BufferDescriptor(
      owned->data(),
      owned->size() * sizeof(T),
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<T>*>(user);
      },
      owned);
}

::filament::backend::PixelBufferDescriptor makePixelBufferDescriptor(
    std::vector<std::uint8_t>&& data,
    ::filament::backend::PixelDataFormat format,
    ::filament::backend::PixelDataType type)
{
  auto* owned = new std::vector<std::uint8_t>(std::move(data));
  return ::filament::backend::PixelBufferDescriptor(
      owned->data(),
      owned->size(),
      format,
      type,
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<std::uint8_t>*>(user);
      },
      owned);
}

void updateImGuiTextures(
    ::filament::Engine& engine,
    ImGuiOverlay& overlay,
    ImVector<ImTextureData*>* textures)
{
  if (textures == nullptr)
    return;
  for (ImTextureData* data : *textures) {
    if (data->Status == ImTextureStatus_WantDestroy) {
      auto it = overlay.textures.find(data);
      if (it != overlay.textures.end()) {
        // Material instances cache a single-level texture's raw handle, and
        // any later descriptor commit (e.g. shared UBO growth) resends it.
        // Unbind the base and every retained clip instance, including
        // inactive ones, before the handle is freed.
        const ::filament::TextureSampler sampler(
            ::filament::TextureSampler::MinFilter::LINEAR,
            ::filament::TextureSampler::MagFilter::LINEAR);
        overlay.materialInstance->setParameter(
            "fontTexture", overlay.fontTexture, sampler);
        for (auto* instance : overlay.mesh.clipMaterialInstances)
          instance->setParameter("fontTexture", overlay.fontTexture, sampler);
        engine.destroy(it->second);
        overlay.textures.erase(it);
      }
      data->BackendUserData = nullptr;
      data->SetTexID(ImTextureID_Invalid);
      data->SetStatus(ImTextureStatus_Destroyed);
      continue;
    }
    if (data->Status != ImTextureStatus_WantCreate
        && data->Status != ImTextureStatus_WantUpdates)
      continue;
    auto* texture = static_cast<::filament::Texture*>(data->BackendUserData);
    if (data->Status == ImTextureStatus_WantCreate) {
      texture = ::filament::Texture::Builder()
                    .width(data->Width)
                    .height(data->Height)
                    .levels(1)
                    .sampler(::filament::Texture::Sampler::SAMPLER_2D)
                    .format(::filament::Texture::InternalFormat::RGBA8)
                    .build(engine);
      overlay.textures[data] = texture;
      data->BackendUserData = texture;
    }
    std::vector<std::uint8_t> pixels(
        static_cast<std::size_t>(data->Width) * data->Height * 4u);
    if (data->Format == ImTextureFormat_RGBA32) {
      std::memcpy(pixels.data(), data->Pixels, pixels.size());
    } else {
      for (std::size_t i = 0; i < pixels.size() / 4u; ++i) {
        pixels[i * 4] = 255;
        pixels[i * 4 + 1] = 255;
        pixels[i * 4 + 2] = 255;
        pixels[i * 4 + 3] = data->Pixels[i];
      }
    }
    texture->setImage(
        engine,
        0,
        makePixelBufferDescriptor(
            std::move(pixels),
            ::filament::backend::PixelDataFormat::RGBA,
            ::filament::backend::PixelDataType::UBYTE));
    data->SetTexID(
        static_cast<ImTextureID>(reinterpret_cast<std::uintptr_t>(texture)));
    data->SetStatus(ImTextureStatus_OK);
  }
}

void removeOverlayRenderable(
    ::filament::Engine& engine, const OverlayMesh& mesh)
{
  if (mesh.entity) {
    auto& renderableManager = engine.getRenderableManager();
    if (renderableManager.hasComponent(mesh.entity)) {
      renderableManager.destroy(mesh.entity);
    }
  }
}

void destroyOverlayMesh(
    ::filament::Engine& engine, ::filament::Scene* scene, OverlayMesh& mesh)
{
  removeOverlayRenderable(engine, mesh);
  if (mesh.entity) {
    if (scene != nullptr) {
      scene->remove(mesh.entity);
    }
    engine.destroy(mesh.entity);
    utils::EntityManager::get().destroy(mesh.entity);
    mesh.entity.clear();
  }
  if (mesh.vertexBuffer != nullptr) {
    engine.destroy(mesh.vertexBuffer);
    mesh.vertexBuffer = nullptr;
  }
  if (mesh.indexBuffer != nullptr) {
    engine.destroy(mesh.indexBuffer);
    mesh.indexBuffer = nullptr;
  }
  for (auto* materialInstance : mesh.clipMaterialInstances) {
    engine.destroy(materialInstance);
  }
  mesh.clipMaterialInstances.clear();
  mesh.vertexCapacity = 0;
  mesh.indexCapacity = 0;
}

} // namespace

ImGuiOverlay createImGuiOverlay(::filament::Engine& engine)
{
  ImGuiOverlay overlay;
  overlay.view = engine.createView();
  overlay.scene = engine.createScene();
  overlay.cameraEntity = utils::EntityManager::get().create();
  overlay.camera = engine.createCamera(overlay.cameraEntity);
  overlay.view->setScene(overlay.scene);
  overlay.view->setCamera(overlay.camera);
  overlay.view->setBlendMode(::filament::BlendMode::TRANSLUCENT);
  overlay.view->setPostProcessingEnabled(false);

  overlay.material = ::filament::Material::Builder()
                         .package(kImGuiMaterial, kImGuiMaterialSize)
                         .build(engine);
  overlay.materialInstance = overlay.material->createInstance();

  configureImGuiOverlayRenderer(ImGui::GetIO());
  overlay.fontTexture = createSolidTexture(
      engine, {255, 255, 255, 255}, TextureColorSpace::Linear);
  updateImGuiTextures(engine, overlay, &ImGui::GetPlatformIO().Textures);
  return overlay;
}

void updateImGuiOverlay(
    ::filament::Engine& engine,
    ImGuiOverlay& overlay,
    const ImDrawData* drawData,
    std::uint32_t width,
    std::uint32_t height)
{
  updateImGuiTextures(
      engine,
      overlay,
      drawData != nullptr ? drawData->Textures
                          : &ImGui::GetPlatformIO().Textures);
  overlay.view->setViewport({0, 0, width, height});
  overlay.camera->setProjection(
      ::filament::Camera::Projection::ORTHO,
      0.0,
      static_cast<double>(width),
      static_cast<double>(height),
      0.0,
      0.01,
      10.0);
  overlay.camera->lookAt({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});

  if (drawData == nullptr || drawData->TotalVtxCount <= 0
      || drawData->TotalIdxCount <= 0) {
    removeOverlayRenderable(engine, overlay.mesh);
    return;
  }

  const ImVec2 displayPos = drawData->DisplayPos;
  std::vector<ImGuiVertex> vertices;
  vertices.reserve(static_cast<std::size_t>(drawData->TotalVtxCount));

  // CmdLists.Size, not the obsolete CmdListsCount mirror: ImGui 1.92.9
  // regressed that field to always report 0 (fixed upstream in 1.92.9b), which
  // leaves this vertex buffer empty and blanks the overlay.
  for (int listIndex = 0; listIndex < drawData->CmdLists.Size; ++listIndex) {
    const ImDrawList* commandList = drawData->CmdLists[listIndex];

    for (const ImDrawVert& vertex : commandList->VtxBuffer) {
      vertices.push_back(ImGuiVertex{
          {(vertex.pos.x - displayPos.x) * drawData->FramebufferScale.x,
           (vertex.pos.y - displayPos.y) * drawData->FramebufferScale.y,
           0.0f},
          {vertex.uv.x, 1.0f - vertex.uv.y},
          vertex.col});
    }
  }
  ImGuiOverlayDrawPlan drawPlan
      = buildImGuiOverlayDrawPlan(*drawData, width, height);

  const std::size_t vertexCount = vertices.size();
  const std::size_t indexCount = drawPlan.indices.size();
  if (drawPlan.commands.empty() || indexCount == 0u) {
    removeOverlayRenderable(engine, overlay.mesh);
    return;
  }

  if (!overlay.mesh.entity) {
    overlay.mesh.entity = utils::EntityManager::get().create();
    overlay.scene->addEntity(overlay.mesh.entity);
  }
  removeOverlayRenderable(engine, overlay.mesh);

  const bool replaceVertexBuffer = overlay.mesh.vertexBuffer == nullptr
                                   || vertexCount > overlay.mesh.vertexCapacity;
  const bool replaceIndexBuffer = overlay.mesh.indexBuffer == nullptr
                                  || indexCount > overlay.mesh.indexCapacity;
  if ((replaceVertexBuffer && overlay.mesh.vertexBuffer != nullptr)
      || (replaceIndexBuffer && overlay.mesh.indexBuffer != nullptr)) {
    ::filament::Fence::waitAndDestroy(engine.createFence());
  }

  if (replaceVertexBuffer) {
    if (overlay.mesh.vertexBuffer != nullptr) {
      engine.destroy(overlay.mesh.vertexBuffer);
    }
    overlay.mesh.vertexBuffer
        = ::filament::VertexBuffer::Builder()
              .vertexCount(vertexCount)
              .bufferCount(1)
              .attribute(
                  ::filament::VertexAttribute::POSITION,
                  0,
                  ::filament::VertexBuffer::AttributeType::FLOAT3,
                  offsetof(ImGuiVertex, position),
                  sizeof(ImGuiVertex))
              .attribute(
                  ::filament::VertexAttribute::UV0,
                  0,
                  ::filament::VertexBuffer::AttributeType::FLOAT2,
                  offsetof(ImGuiVertex, uv),
                  sizeof(ImGuiVertex))
              .attribute(
                  ::filament::VertexAttribute::COLOR,
                  0,
                  ::filament::VertexBuffer::AttributeType::UBYTE4,
                  offsetof(ImGuiVertex, color),
                  sizeof(ImGuiVertex))
              .normalized(::filament::VertexAttribute::COLOR)
              .build(engine);
    overlay.mesh.vertexCapacity = vertexCount;
  }

  if (replaceIndexBuffer) {
    if (overlay.mesh.indexBuffer != nullptr) {
      engine.destroy(overlay.mesh.indexBuffer);
    }
    overlay.mesh.indexBuffer
        = ::filament::IndexBuffer::Builder()
              .indexCount(indexCount)
              .bufferType(::filament::IndexBuffer::IndexType::UINT)
              .build(engine);
    overlay.mesh.indexCapacity = indexCount;
  }

  while (overlay.mesh.clipMaterialInstances.size() + 1u
         < drawPlan.commands.size()) {
    overlay.mesh.clipMaterialInstances.push_back(
        ::filament::MaterialInstance::duplicate(overlay.materialInstance));
  }

  overlay.mesh.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));
  overlay.mesh.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(drawPlan.indices)));

  ::filament::RenderableManager::Builder builder(drawPlan.commands.size());
  builder
      .boundingBox(
          {{0.0f, 0.0f, -1.0f},
           {static_cast<float>(width), static_cast<float>(height), 1.0f}})
      .culling(false)
      .castShadows(false)
      .receiveShadows(false);
  for (std::size_t commandIndex = 0u; commandIndex < drawPlan.commands.size();
       ++commandIndex) {
    const ImGuiOverlayDrawCommand& command = drawPlan.commands[commandIndex];
    auto* materialInstance
        = commandIndex == 0u
              ? overlay.materialInstance
              : overlay.mesh.clipMaterialInstances[commandIndex - 1u];
    const ::filament::TextureSampler sampler(
        ::filament::TextureSampler::MinFilter::LINEAR,
        ::filament::TextureSampler::MagFilter::LINEAR);
    auto* texture = reinterpret_cast<::filament::Texture*>(
        static_cast<std::uintptr_t>(command.textureId));
    materialInstance->setParameter(
        "fontTexture",
        texture != nullptr ? texture : overlay.fontTexture,
        sampler);
    materialInstance->setScissor(
        command.scissorLeft,
        command.scissorBottom,
        command.scissorWidth,
        command.scissorHeight);
    builder.material(commandIndex, materialInstance)
        .geometry(
            commandIndex,
            ::filament::RenderableManager::PrimitiveType::TRIANGLES,
            overlay.mesh.vertexBuffer,
            overlay.mesh.indexBuffer,
            command.indexOffset,
            command.indexCount)
        .blendOrder(
            commandIndex,
            static_cast<std::uint16_t>(
                std::min<std::size_t>(commandIndex, 0x7FFFu)));
  }
  builder.build(engine, overlay.mesh.entity);
}

void destroyImGuiOverlay(::filament::Engine& engine, ImGuiOverlay& overlay)
{
  destroyOverlayMesh(engine, overlay.scene, overlay.mesh);
  for (auto& entry : overlay.textures) {
    engine.destroy(entry.second);
    entry.first->BackendUserData = nullptr;
    entry.first->SetTexID(ImTextureID_Invalid);
    entry.first->SetStatus(ImTextureStatus_Destroyed);
  }
  overlay.textures.clear();
  if (overlay.materialInstance != nullptr) {
    engine.destroy(overlay.materialInstance);
    overlay.materialInstance = nullptr;
  }
  if (overlay.fontTexture != nullptr) {
    engine.destroy(overlay.fontTexture);
    overlay.fontTexture = nullptr;
  }
  if (overlay.material != nullptr) {
    engine.destroy(overlay.material);
    overlay.material = nullptr;
  }
  if (overlay.view != nullptr) {
    engine.destroy(overlay.view);
    overlay.view = nullptr;
  }
  if (overlay.camera != nullptr) {
    engine.destroyCameraComponent(overlay.cameraEntity);
    overlay.camera = nullptr;
  }
  if (overlay.cameraEntity) {
    utils::EntityManager::get().destroy(overlay.cameraEntity);
    overlay.cameraEntity.clear();
  }
  if (overlay.scene != nullptr) {
    engine.destroy(overlay.scene);
    overlay.scene = nullptr;
  }
}

} // namespace dart::gui::detail
