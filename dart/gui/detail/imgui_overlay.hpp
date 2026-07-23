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

#ifndef DART_GUI_DETAIL_IMGUI_OVERLAY_HPP_
#define DART_GUI_DETAIL_IMGUI_OVERLAY_HPP_

#include <utils/Entity.h>

#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  #include <vector>
#endif

#include <cstddef>
#include <cstdint>

namespace filament {

class Camera;
class Engine;
class IndexBuffer;
class Material;
class MaterialInstance;
class Scene;
class Texture;
class VertexBuffer;
class View;

} // namespace filament

struct ImDrawData;
struct ImGuiIO;

namespace dart::gui::detail {

struct OverlayMesh
{
  utils::Entity entity;
  ::filament::VertexBuffer* vertexBuffer = nullptr;
  ::filament::IndexBuffer* indexBuffer = nullptr;
#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  std::vector<::filament::MaterialInstance*> clipMaterialInstances;
  std::size_t vertexCapacity = 0;
  std::size_t indexCapacity = 0;
#else
  std::size_t vertexCount = 0;
  std::size_t indexCount = 0;
#endif
};

struct ImGuiOverlay
{
  ::filament::View* view = nullptr;
  ::filament::Scene* scene = nullptr;
  ::filament::Camera* camera = nullptr;
  utils::Entity cameraEntity;
  ::filament::Material* material = nullptr;
  ::filament::MaterialInstance* materialInstance = nullptr;
  ::filament::Texture* fontTexture = nullptr;
  OverlayMesh mesh;
  float uiScale = 1.0f;
};

ImGuiOverlay createConfiguredImGuiOverlay(
    ::filament::Engine& engine, float uiScale);

void updateConfiguredImGuiOverlayScale(
    ::filament::Engine& engine, ImGuiOverlay& overlay, float uiScale);

ImGuiIO& getCurrentImGuiIo();

void updateImGuiOverlay(
    ::filament::Engine& engine,
    ImGuiOverlay& overlay,
    const ImDrawData* drawData,
    std::uint32_t width,
    std::uint32_t height);

void destroyConfiguredImGuiOverlay(
    ::filament::Engine& engine, ImGuiOverlay& overlay);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_IMGUI_OVERLAY_HPP_
