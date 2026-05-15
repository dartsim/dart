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

#include "imgui_material.hpp"

#include <backend/BufferDescriptor.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Engine.h>
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
#include <utils/EntityManager.h>

#include <filesystem>
#include <vector>

#include <cstdlib>
#include <cstring>

namespace dart::examples::filament_gui {
namespace {

struct ImGuiVertex
{
  filament::math::float3 position;
  filament::math::float2 uv;
  std::uint32_t color = 0;
};

template <typename T>
filament::backend::BufferDescriptor makeBufferDescriptor(std::vector<T>&& data)
{
  auto* owned = new std::vector<T>(std::move(data));
  return filament::backend::BufferDescriptor(
      owned->data(),
      owned->size() * sizeof(T),
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<T>*>(user);
      },
      owned);
}

filament::backend::PixelBufferDescriptor makePixelBufferDescriptor(
    std::vector<std::uint8_t>&& data,
    filament::backend::PixelDataFormat format,
    filament::backend::PixelDataType type)
{
  auto* owned = new std::vector<std::uint8_t>(std::move(data));
  return filament::backend::PixelBufferDescriptor(
      owned->data(),
      owned->size(),
      format,
      type,
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<std::uint8_t>*>(user);
      },
      owned);
}

void destroyOverlayMesh(
    filament::Engine& engine, filament::Scene* scene, OverlayMesh& mesh)
{
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
  mesh.vertexCount = 0;
  mesh.indexCount = 0;
}

} // namespace

ImGuiOverlay createImGuiOverlay(filament::Engine& engine)
{
  ImGuiOverlay overlay;
  overlay.view = engine.createView();
  overlay.scene = engine.createScene();
  overlay.cameraEntity = utils::EntityManager::get().create();
  overlay.camera = engine.createCamera(overlay.cameraEntity);
  overlay.view->setScene(overlay.scene);
  overlay.view->setCamera(overlay.camera);
  overlay.view->setBlendMode(filament::BlendMode::TRANSLUCENT);
  overlay.view->setPostProcessingEnabled(false);

  overlay.material = filament::Material::Builder()
                         .package(kImGuiMaterial, kImGuiMaterialSize)
                         .build(engine);
  overlay.materialInstance = overlay.material->createInstance();

  unsigned char* pixels = nullptr;
  int width = 0;
  int height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  std::vector<std::uint8_t> fontPixels(
      pixels, pixels + static_cast<std::size_t>(width) * height * 4);

  overlay.fontTexture = filament::Texture::Builder()
                            .width(static_cast<std::uint32_t>(width))
                            .height(static_cast<std::uint32_t>(height))
                            .levels(1)
                            .sampler(filament::Texture::Sampler::SAMPLER_2D)
                            .format(filament::Texture::InternalFormat::RGBA8)
                            .build(engine);
  overlay.fontTexture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(fontPixels),
          filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE));

  const filament::TextureSampler sampler(
      filament::TextureSampler::MinFilter::LINEAR,
      filament::TextureSampler::MagFilter::LINEAR);
  overlay.materialInstance->setParameter(
      "fontTexture", overlay.fontTexture, sampler);
  return overlay;
}

void loadImGuiFont(ImGuiIO& io, float guiScale)
{
  std::vector<std::filesystem::path> candidates;
  if (const char* fontPath = std::getenv("DART_FILAMENT_GUI_FONT");
      fontPath != nullptr && std::strlen(fontPath) > 0) {
    candidates.emplace_back(fontPath);
  }

  candidates.emplace_back("/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf");
  candidates.emplace_back("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
  candidates.emplace_back(
      "/usr/share/fonts/liberation-sans-fonts/LiberationSans-Regular.ttf");
  candidates.emplace_back("/System/Library/Fonts/Supplemental/Arial.ttf");
  candidates.emplace_back("/Library/Fonts/Arial.ttf");
  candidates.emplace_back("C:/Windows/Fonts/segoeui.ttf");

  ImFontConfig config;
  config.OversampleH = 3;
  config.OversampleV = 2;
  config.PixelSnapH = false;
  const float fontSize = 15.0f * guiScale;
  for (const auto& path : candidates) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
      continue;
    }
    if (io.Fonts->AddFontFromFileTTF(path.string().c_str(), fontSize, &config)
        != nullptr) {
      return;
    }
  }

  config.SizePixels = 13.0f * guiScale;
  io.Fonts->AddFontDefault(&config);
}

void updateImGuiOverlay(
    filament::Engine& engine,
    ImGuiOverlay& overlay,
    const ImDrawData* drawData,
    std::uint32_t width,
    std::uint32_t height)
{
  overlay.view->setViewport({0, 0, width, height});
  overlay.camera->setProjection(
      filament::Camera::Projection::ORTHO,
      0.0,
      static_cast<double>(width),
      static_cast<double>(height),
      0.0,
      0.01,
      10.0);
  overlay.camera->lookAt({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});

  if (drawData == nullptr || drawData->TotalVtxCount <= 0
      || drawData->TotalIdxCount <= 0) {
    destroyOverlayMesh(engine, overlay.scene, overlay.mesh);
    return;
  }

  const ImVec2 displayPos = drawData->DisplayPos;
  std::vector<ImGuiVertex> vertices;
  std::vector<std::uint32_t> indices;
  vertices.reserve(static_cast<std::size_t>(drawData->TotalVtxCount));
  indices.reserve(static_cast<std::size_t>(drawData->TotalIdxCount));

  for (int listIndex = 0; listIndex < drawData->CmdListsCount; ++listIndex) {
    const ImDrawList* commandList = drawData->CmdLists[listIndex];
    const std::uint32_t vertexBase
        = static_cast<std::uint32_t>(vertices.size());

    for (const ImDrawVert& vertex : commandList->VtxBuffer) {
      vertices.push_back(
          ImGuiVertex{
              {vertex.pos.x - displayPos.x, vertex.pos.y - displayPos.y, 0.0f},
              {vertex.uv.x, 1.0f - vertex.uv.y},
              vertex.col});
    }

    for (const ImDrawIdx index : commandList->IdxBuffer) {
      indices.push_back(vertexBase + static_cast<std::uint32_t>(index));
    }
  }

  const std::size_t vertexCount = vertices.size();
  const std::size_t indexCount = indices.size();
  if (!overlay.mesh.entity || overlay.mesh.vertexCount != vertexCount
      || overlay.mesh.indexCount != indexCount) {
    destroyOverlayMesh(engine, overlay.scene, overlay.mesh);

    overlay.mesh.vertexBuffer
        = filament::VertexBuffer::Builder()
              .vertexCount(vertexCount)
              .bufferCount(1)
              .attribute(
                  filament::VertexAttribute::POSITION,
                  0,
                  filament::VertexBuffer::AttributeType::FLOAT3,
                  offsetof(ImGuiVertex, position),
                  sizeof(ImGuiVertex))
              .attribute(
                  filament::VertexAttribute::UV0,
                  0,
                  filament::VertexBuffer::AttributeType::FLOAT2,
                  offsetof(ImGuiVertex, uv),
                  sizeof(ImGuiVertex))
              .attribute(
                  filament::VertexAttribute::COLOR,
                  0,
                  filament::VertexBuffer::AttributeType::UBYTE4,
                  offsetof(ImGuiVertex, color),
                  sizeof(ImGuiVertex))
              .normalized(filament::VertexAttribute::COLOR)
              .build(engine);

    overlay.mesh.indexBuffer
        = filament::IndexBuffer::Builder()
              .indexCount(indexCount)
              .bufferType(filament::IndexBuffer::IndexType::UINT)
              .build(engine);

    overlay.mesh.entity = utils::EntityManager::get().create();
    filament::RenderableManager::Builder(1)
        .boundingBox(
            {{0.0f, 0.0f, -1.0f},
             {static_cast<float>(width), static_cast<float>(height), 1.0f}})
        .material(0, overlay.materialInstance)
        .geometry(
            0,
            filament::RenderableManager::PrimitiveType::TRIANGLES,
            overlay.mesh.vertexBuffer,
            overlay.mesh.indexBuffer,
            0,
            indexCount)
        .culling(false)
        .castShadows(false)
        .receiveShadows(false)
        .build(engine, overlay.mesh.entity);
    overlay.scene->addEntity(overlay.mesh.entity);
    overlay.mesh.vertexCount = vertexCount;
    overlay.mesh.indexCount = indexCount;
  }

  overlay.mesh.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));
  overlay.mesh.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));
}

void destroyImGuiOverlay(filament::Engine& engine, ImGuiOverlay& overlay)
{
  destroyOverlayMesh(engine, overlay.scene, overlay.mesh);
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
  if (overlay.camera != nullptr) {
    engine.destroyCameraComponent(overlay.cameraEntity);
    overlay.camera = nullptr;
  }
  if (overlay.cameraEntity) {
    utils::EntityManager::get().destroy(overlay.cameraEntity);
    overlay.cameraEntity.clear();
  }
  if (overlay.view != nullptr) {
    engine.destroy(overlay.view);
    overlay.view = nullptr;
  }
  if (overlay.scene != nullptr) {
    engine.destroy(overlay.scene);
    overlay.scene = nullptr;
  }
}

} // namespace dart::examples::filament_gui
