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

#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  #include "imgui_draw_data.hpp"
#endif
#include "imgui_material.hpp"

#include <dart/gui/application.hpp>

#include <backend/BufferDescriptor.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Engine.h>
#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  #include <filament/Fence.h>
#endif
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

#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  #include <algorithm>
#endif
#include <filesystem>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <cstring>

// The modern viewer theme below assigns Dear ImGui style slots and fields from
// the 1.89-1.91 series (e.g. ImGuiCol_NavCursor, renamed in 1.91.4). DART
// targets the bundled ImGui 1.92.x (see cmake/dart_find_dependencies.cmake);
// building dart::gui against an older ImGui is unsupported and fails here with
// a clear message instead of a cascade of unknown-identifier errors.
#if IMGUI_VERSION_NUM < 19200
  #error                                                                       \
      "dart::gui requires Dear ImGui >= 1.92 (use the bundled ImGui via DART_USE_SYSTEM_IMGUI=OFF, or upgrade the system ImGui)."
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

void scaleImGuiWindow(ImGuiWindow& window, float scale)
{
  const ImVec2 origin
      = window.Viewport != nullptr ? window.Viewport->Pos : ImVec2(0.0f, 0.0f);
  window.Pos = ImFloor(ImVec2(
      (window.Pos.x - origin.x) * scale + origin.x,
      (window.Pos.y - origin.y) * scale + origin.y));
  window.Size = ImTrunc(ImVec2(window.Size.x * scale, window.Size.y * scale));
  window.SizeFull
      = ImTrunc(ImVec2(window.SizeFull.x * scale, window.SizeFull.y * scale));
  window.ContentSize = ImTrunc(
      ImVec2(window.ContentSize.x * scale, window.ContentSize.y * scale));
}

void scaleExistingImGuiWindows(float scale)
{
  if (scale <= 0.0f || !std::isfinite(scale) || scale == 1.0f) {
    return;
  }

  auto* context = ImGui::GetCurrentContext();
  if (context == nullptr) {
    return;
  }

  for (ImGuiWindow* window : context->Windows) {
    if (window != nullptr) {
      scaleImGuiWindow(*window, scale);
    }
  }
}

::filament::Texture* createFontTexture(::filament::Engine& engine)
{
  unsigned char* pixels = nullptr;
  int width = 0;
  int height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  std::vector<std::uint8_t> fontPixels(
      pixels, pixels + static_cast<std::size_t>(width) * height * 4);

  auto* fontTexture = ::filament::Texture::Builder()
                          .width(static_cast<std::uint32_t>(width))
                          .height(static_cast<std::uint32_t>(height))
                          .levels(1)
                          .sampler(::filament::Texture::Sampler::SAMPLER_2D)
                          .format(::filament::Texture::InternalFormat::RGBA8)
                          .build(engine);
  fontTexture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(fontPixels),
          ::filament::backend::PixelDataFormat::RGBA,
          ::filament::backend::PixelDataType::UBYTE));
  return fontTexture;
}

void updateFontTexture(::filament::Engine& engine, ImGuiOverlay& overlay)
{
  if (overlay.fontTexture != nullptr) {
    engine.destroy(overlay.fontTexture);
    overlay.fontTexture = nullptr;
  }

  overlay.fontTexture = createFontTexture(engine);

  const ::filament::TextureSampler sampler(
      ::filament::TextureSampler::MinFilter::LINEAR,
      ::filament::TextureSampler::MagFilter::LINEAR);
  overlay.materialInstance->setParameter(
      "fontTexture", overlay.fontTexture, sampler);
#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  for (auto* materialInstance : overlay.mesh.clipMaterialInstances) {
    materialInstance->setParameter("fontTexture", overlay.fontTexture, sampler);
  }
#endif
}

#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
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
#endif

void destroyOverlayMesh(
    ::filament::Engine& engine, ::filament::Scene* scene, OverlayMesh& mesh)
{
#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  removeOverlayRenderable(engine, mesh);
#endif
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
#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  for (auto* materialInstance : mesh.clipMaterialInstances) {
    engine.destroy(materialInstance);
  }
  mesh.clipMaterialInstances.clear();
  mesh.vertexCapacity = 0;
  mesh.indexCapacity = 0;
#else
  mesh.vertexCount = 0;
  mesh.indexCount = 0;
#endif
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

  updateFontTexture(engine, overlay);
  return overlay;
}

void loadImGuiFont(ImGuiIO& io, float uiScale)
{
  io.Fonts->Clear();

  std::vector<std::filesystem::path> candidates;
  if (const char* fontPath = std::getenv("DART_GUI_FILAMENT_FONT");
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
  const float fontSize = 15.0f * uiScale;
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

  config.SizePixels = 13.0f * uiScale;
  io.Fonts->AddFontDefault(&config);
}

namespace {

// ── Modern editor theme ──────────────────────────────────────────────────────
// A cohesive dark palette in the spirit of VS Code / Blender / Unity: cool
// neutral surfaces, a single restrained blue accent, low-contrast borders, and
// soft rounding. Centralizing it here keeps every panel, the perf HUD, and the
// docked regions visually consistent instead of ImGui's stock high-contrast
// dark theme (saturated-blue selection, square widgets, cramped spacing).

ImVec4 srgb(int r, int g, int b, float a = 1.0f)
{
  return ImVec4(
      static_cast<float>(r) / 255.0f,
      static_cast<float>(g) / 255.0f,
      static_cast<float>(b) / 255.0f,
      a);
}

void applyModernDarkColors(ImGuiStyle& style)
{
  // Surfaces, deepest to lightest.
  const ImVec4 bgWindow = srgb(0x1b, 0x1d, 0x23); // floating/docked window body
  const ImVec4 bgPanel = srgb(0x21, 0x24, 0x2b);  // docked tab content, hover
  const ImVec4 bgHeader
      = srgb(0x16, 0x18, 0x1d); // title/menu/tab bars, headers
  // Control fills (text inputs, sliders, combos, buttons).
  const ImVec4 frame = srgb(0x2a, 0x2e, 0x37);
  const ImVec4 frameHover = srgb(0x33, 0x39, 0x44);
  const ImVec4 frameActive = srgb(0x3c, 0x43, 0x51);
  const ImVec4 button = srgb(0x2f, 0x35, 0x40);
  const ImVec4 buttonHover = srgb(0x3a, 0x42, 0x50);
  // Accent: one clean blue, used sparingly for focus/selection/active state.
  const ImVec4 accent = srgb(0x4c, 0x8c, 0xf0);
  const ImVec4 accentHover = srgb(0x66, 0x9a, 0xf3);
  const ImVec4 accentActive = srgb(0x3a, 0x78, 0xdc);
  const ImVec4 accentSoft = srgb(0x4c, 0x8c, 0xf0, 0.22f); // selection fills
  const ImVec4 accentSofter
      = srgb(0x4c, 0x8c, 0xf0, 0.38f); // hover/active fills
  // Text and lines.
  const ImVec4 text = srgb(0xcd, 0xd3, 0xde);
  const ImVec4 textDim = srgb(0x7e, 0x87, 0x94);
  const ImVec4 border = srgb(0x32, 0x38, 0x43, 0.60f);

  ImVec4* c = style.Colors;
  c[ImGuiCol_Text] = text;
  c[ImGuiCol_TextDisabled] = textDim;
  c[ImGuiCol_TextLink] = accentHover;
  c[ImGuiCol_WindowBg] = bgWindow;
  c[ImGuiCol_ChildBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_PopupBg] = srgb(0x1d, 0x20, 0x27, 0.98f);
  c[ImGuiCol_Border] = border;
  c[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_FrameBg] = frame;
  c[ImGuiCol_FrameBgHovered] = frameHover;
  c[ImGuiCol_FrameBgActive] = frameActive;
  c[ImGuiCol_TitleBg] = bgHeader;
  c[ImGuiCol_TitleBgActive] = bgHeader;
  c[ImGuiCol_TitleBgCollapsed] = bgHeader;
  c[ImGuiCol_MenuBarBg] = bgHeader;
  c[ImGuiCol_ScrollbarBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_ScrollbarGrab] = srgb(0x3a, 0x41, 0x4e);
  c[ImGuiCol_ScrollbarGrabHovered] = srgb(0x47, 0x4f, 0x5e);
  c[ImGuiCol_ScrollbarGrabActive] = accent;
  c[ImGuiCol_CheckMark] = accent;
  c[ImGuiCol_SliderGrab] = accent;
  c[ImGuiCol_SliderGrabActive] = accentHover;
  c[ImGuiCol_Button] = button;
  c[ImGuiCol_ButtonHovered] = buttonHover;
  c[ImGuiCol_ButtonActive] = accentActive;
  c[ImGuiCol_Header] = accentSoft;
  c[ImGuiCol_HeaderHovered] = accentSofter;
  c[ImGuiCol_HeaderActive] = accentSofter;
  c[ImGuiCol_Separator] = border;
  c[ImGuiCol_SeparatorHovered] = accent;
  c[ImGuiCol_SeparatorActive] = accentHover;
  c[ImGuiCol_ResizeGrip] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f); // edge-drag instead
  c[ImGuiCol_ResizeGripHovered] = accentSofter;
  c[ImGuiCol_ResizeGripActive] = accent;
  c[ImGuiCol_Tab] = bgHeader;
  c[ImGuiCol_TabHovered] = frameHover;
  c[ImGuiCol_TabSelected] = bgPanel;
  c[ImGuiCol_TabSelectedOverline] = accent; // VS Code-style active-tab accent
  c[ImGuiCol_TabDimmed] = bgHeader;
  c[ImGuiCol_TabDimmedSelected] = bgPanel;
  // Unfocused (dimmed) tab bars get no accent overline.
  c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
#ifdef IMGUI_HAS_DOCK
  // Docking color slots only exist on the ImGui docking branch (the look used
  // by `pixi run py-demos`/`dartsim`). Non-docking builds (system ImGui) omit
  // them.
  c[ImGuiCol_DockingPreview] = accentSofter;
  c[ImGuiCol_DockingEmptyBg] = srgb(0x12, 0x14, 0x18);
#endif
  c[ImGuiCol_PlotLines] = textDim;
  c[ImGuiCol_PlotLinesHovered] = accentHover;
  c[ImGuiCol_PlotHistogram] = accent;
  c[ImGuiCol_PlotHistogramHovered] = accentHover;
  c[ImGuiCol_TableHeaderBg] = bgHeader;
  c[ImGuiCol_TableBorderStrong] = srgb(0x32, 0x38, 0x43);
  c[ImGuiCol_TableBorderLight] = srgb(0x28, 0x2d, 0x36);
  c[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_TableRowBgAlt] = srgb(0xff, 0xff, 0xff, 0.025f);
  c[ImGuiCol_TextSelectedBg] = accentSofter;
  c[ImGuiCol_DragDropTarget] = accentHover;
  c[ImGuiCol_NavCursor] = accent;
  c[ImGuiCol_NavWindowingHighlight] = srgb(0xff, 0xff, 0xff, 0.70f);
  c[ImGuiCol_NavWindowingDimBg] = srgb(0x08, 0x09, 0x0c, 0.55f);
  c[ImGuiCol_ModalWindowDimBg] = srgb(0x08, 0x09, 0x0c, 0.55f);
}

void applyModernDarkMetrics(ImGuiStyle& style)
{
  // Generous, even spacing reads as "designed" rather than dense-default ImGui.
  style.WindowPadding = ImVec2(12.0f, 10.0f);
  style.FramePadding = ImVec2(9.0f, 5.0f);
  style.CellPadding = ImVec2(8.0f, 4.0f);
  style.ItemSpacing = ImVec2(9.0f, 7.0f);
  style.ItemInnerSpacing = ImVec2(7.0f, 5.0f);
  style.IndentSpacing = 20.0f;
  style.ScrollbarSize = 13.0f;
  style.GrabMinSize = 11.0f;

  // Thin hairline borders for definition; flat (border-less) widgets.
  style.WindowBorderSize = 1.0f;
  style.ChildBorderSize = 1.0f;
  style.PopupBorderSize = 1.0f;
  style.FrameBorderSize = 0.0f;
  style.TabBarBorderSize = 1.0f;
  style.TabBarOverlineSize = 2.0f;
#ifdef IMGUI_HAS_DOCK
  // DockingSeparatorSize only exists on the ImGui docking branch.
  style.DockingSeparatorSize = 1.0f;
#endif
  style.SeparatorTextBorderSize = 2.0f;

  // Soft, consistent rounding; pill-shaped scrollbars.
  style.WindowRounding = 7.0f;
  style.ChildRounding = 6.0f;
  style.PopupRounding = 6.0f;
  style.FrameRounding = 5.0f;
  style.GrabRounding = 5.0f;
  style.TabRounding = 6.0f;
  style.ScrollbarRounding = 9.0f;

  // Left-aligned titles, no collapse caret: a cleaner, editor-like title bar.
  style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
  style.WindowMenuButtonPosition = ImGuiDir_None;
  style.SeparatorTextAlign = ImVec2(0.0f, 0.5f);
  style.SeparatorTextPadding = ImVec2(20.0f, 6.0f);

  style.Alpha = 1.0f;
  style.DisabledAlpha = 0.5f;
}

} // namespace

void configureImGuiStyle(float uiScale)
{
  auto& style = ImGui::GetStyle();
  style = ImGuiStyle();
  // Start from the stock dark theme so any field we do not override keeps a
  // sensible value, then layer the DART editor look on top.
  ImGui::StyleColorsDark();
  applyModernDarkMetrics(style);
  applyModernDarkColors(style);
  // Sizes are authored at 1.0; scale them last for HiDPI/user scale. Colors and
  // alpha are scale-independent and stay untouched.
  style.ScaleAllSizes(uiScale);
}

void configureImGuiFonts(float uiScale)
{
  auto& io = ImGui::GetIO();
  loadImGuiFont(io, uiScale);
  io.Fonts->Build();
}

ImGuiOverlay createConfiguredImGuiOverlay(
    ::filament::Engine& engine, float uiScale)
{
  ImGui::CreateContext();
#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  configureImGuiOverlayRenderer(ImGui::GetIO());
#endif
#ifdef IMGUI_HAS_DOCK
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
  configureImGuiStyle(uiScale);
  configureImGuiFonts(uiScale);
  ImGuiOverlay overlay = createImGuiOverlay(engine);
  overlay.uiScale = uiScale;
  return overlay;
}

void updateConfiguredImGuiOverlayScale(
    ::filament::Engine& engine, ImGuiOverlay& overlay, float uiScale)
{
  if (!std::isfinite(uiScale) || uiScale <= 0.0f) {
    uiScale = 1.0f;
  }

  if (std::abs(overlay.uiScale - uiScale) <= 1e-4f) {
    return;
  }

  const float previousScale = overlay.uiScale > 0.0f ? overlay.uiScale : 1.0f;
  configureImGuiStyle(uiScale);
  configureImGuiFonts(uiScale);
  updateFontTexture(engine, overlay);
  scaleExistingImGuiWindows(uiScale / previousScale);
  overlay.uiScale = uiScale;
}

ImGuiIO& getCurrentImGuiIo()
{
  return ImGui::GetIO();
}

void updateImGuiOverlay(
    ::filament::Engine& engine,
    ImGuiOverlay& overlay,
    const ImDrawData* drawData,
    std::uint32_t width,
    std::uint32_t height)
{
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

#if defined(DART_GUI_IMGUI_LARGE_DRAW_DATA)
  if (drawData == nullptr || drawData->TotalVtxCount <= 0
      || drawData->TotalIdxCount <= 0) {
    removeOverlayRenderable(engine, overlay.mesh);
    return;
  }

  const ImVec2 displayPos = drawData->DisplayPos;
  std::vector<ImGuiVertex> vertices;
  vertices.reserve(static_cast<std::size_t>(drawData->TotalVtxCount));

  for (int listIndex = 0; listIndex < drawData->CmdListsCount; ++listIndex) {
    const ImDrawList* commandList = drawData->CmdLists[listIndex];

    for (const ImDrawVert& vertex : commandList->VtxBuffer) {
      vertices.push_back(
          ImGuiVertex{
              {vertex.pos.x - displayPos.x, vertex.pos.y - displayPos.y, 0.0f},
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
#else
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

    overlay.mesh.indexBuffer
        = ::filament::IndexBuffer::Builder()
              .indexCount(indexCount)
              .bufferType(::filament::IndexBuffer::IndexType::UINT)
              .build(engine);

    overlay.mesh.entity = utils::EntityManager::get().create();
    ::filament::RenderableManager::Builder(1)
        .boundingBox(
            {{0.0f, 0.0f, -1.0f},
             {static_cast<float>(width), static_cast<float>(height), 1.0f}})
        .material(0, overlay.materialInstance)
        .geometry(
            0,
            ::filament::RenderableManager::PrimitiveType::TRIANGLES,
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
#endif
}

void destroyImGuiOverlay(::filament::Engine& engine, ImGuiOverlay& overlay)
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

void destroyConfiguredImGuiOverlay(
    ::filament::Engine& engine, ImGuiOverlay& overlay)
{
  destroyImGuiOverlay(engine, overlay);
  ImGui::DestroyContext();
}

} // namespace dart::gui::detail

namespace dart::gui {

bool isDockingAvailable()
{
#ifdef IMGUI_HAS_DOCK
  return true;
#else
  return false;
#endif
}

} // namespace dart::gui
