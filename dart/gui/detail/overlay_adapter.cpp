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

#include <dart/gui/DebugOverlay.hpp>
#include <dart/gui/GridVisual.hpp>
#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/detail/overlay_adapter.hpp>

#include <Eigen/Geometry>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>

#include <cfloat>
#include <cmath>

namespace dart::gui::detail {
namespace {
struct OverlayProjection
{
  Eigen::Vector3d eye, forward, right, up;
  double pixelScale;
  int width, height;
  OverlayProjection(
      const Eigen::Vector3d& e,
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& u,
      double fovy,
      int w,
      int h)
    : eye(e),
      forward((c - e).normalized()),
      right(forward.cross(u).normalized()),
      up(right.cross(forward).normalized()),
      pixelScale(
          std::max(1, h) / (2 * std::tan(fovy * 3.14159265358979323846 / 360))),
      width(std::max(1, w)),
      height(std::max(1, h))
  {
  }
  Eigen::Vector3d project(const Eigen::Vector3d& p) const
  {
    const Eigen::Vector3d v = p - eye;
    const double depth = v.dot(forward);
    return {
        width * 0.5 + v.dot(right) * pixelScale / depth,
        height * 0.5 - v.dot(up) * pixelScale / depth,
        depth};
  }
  bool clip(Eigen::Vector3d& a, Eigen::Vector3d& b) const
  {
    constexpr double nearDepth = 1e-4;
    const double da = (a - eye).dot(forward), db = (b - eye).dot(forward);
    if (!a.allFinite() || !b.allFinite() || (da < nearDepth && db < nearDepth))
      return false;
    if (da < nearDepth)
      a += (b - a) * ((nearDepth - da) / (db - da));
    else if (db < nearDepth)
      b += (a - b) * ((nearDepth - db) / (da - db));
    return true;
  }
};
ImU32 overlayColor(const Eigen::Vector4d& rgba)
{
  return ImGui::ColorConvertFloat4ToU32(ImVec4(
      static_cast<float>(std::clamp(rgba.x(), 0.0, 1.0)),
      static_cast<float>(std::clamp(rgba.y(), 0.0, 1.0)),
      static_cast<float>(std::clamp(rgba.z(), 0.0, 1.0)),
      static_cast<float>(std::clamp(rgba.w(), 0.0, 1.0))));
}
} // namespace

void collectDebugScene(
    const std::vector<std::shared_ptr<ViewerAttachment>>& attachments,
    DebugScene& scene)
{
  scene.lines.clear();
  scene.triangles.clear();
  scene.labels.clear();
  for (const auto& attachment : attachments) {
    if (const auto* overlay
        = dynamic_cast<const DebugOverlay*>(attachment.get()))
      OverlayAccess::append(*overlay, scene);
    else if (
        const auto* grid = dynamic_cast<const GridVisual*>(attachment.get()))
      OverlayAccess::append(*grid, scene);
    else if (
        const auto* support
        = dynamic_cast<const SupportPolygonVisual*>(attachment.get()))
      OverlayAccess::append(*support, scene);
  }
}
void prepareDebugScene(
    DebugScene& scene,
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up,
    double fovy,
    int width,
    int height)
{
  const OverlayProjection view(eye, center, up, fovy, width, height);
  auto keep = scene.lines.begin();
  for (auto it = scene.lines.begin(); it != scene.lines.end(); ++it) {
    if (it->alwaysOnTop || it->pixelWidth <= 0) {
      if (keep != it)
        *keep = *it;
      ++keep;
      continue;
    }
    Eigen::Vector3d a = it->from, b = it->to;
    if (!view.clip(a, b))
      continue;
    const Eigen::Vector3d pa = view.project(a), pb = view.project(b);
    Eigen::Vector2d tangent = pb.head<2>() - pa.head<2>();
    if (!tangent.allFinite() || tangent.norm() < 1e-8)
      continue;
    tangent.normalize();
    const Eigen::Vector3d offset
        = (view.right * (-tangent.y()) - view.up * tangent.x()) * it->pixelWidth
          * 0.5 / view.pixelScale;
    const Eigen::Vector3d oa = offset * pa.z(), ob = offset * pb.z();
    DebugTriangleDescriptor first, second;
    first.a = a + oa;
    first.b = a - oa;
    first.c = b - ob;
    first.rgba = it->rgba;
    first.label = it->label;
    first.alwaysOnTop = false;
    second = first;
    second.b = b - ob;
    second.c = b + ob;
    scene.triangles.push_back(std::move(first));
    scene.triangles.push_back(std::move(second));
  }
  scene.lines.erase(keep, scene.lines.end());
}
struct OverlayRenderer::Impl
{
  std::unordered_map<std::string, ImFont*> fonts;
};
OverlayRenderer::OverlayRenderer() : mImpl(std::make_unique<Impl>()) {}
OverlayRenderer::~OverlayRenderer() = default;
void OverlayRenderer::prepareFonts(const DebugScene& scene)
{
  auto& fonts = mImpl->fonts;
  for (const auto& label : scene.labels) {
    if (label.fontPath.empty() || fonts.count(label.fontPath))
      continue;
    if (!std::filesystem::is_regular_file(label.fontPath)
        || std::filesystem::file_size(label.fontPath) == 0)
      throw std::runtime_error("Cannot load overlay font: " + label.fontPath);
    auto* font = ImGui::GetIO().Fonts->AddFontFromFileTTF(
        label.fontPath.c_str(), 16.0f);
    if (!font)
      throw std::runtime_error("Cannot load overlay font: " + label.fontPath);
    fonts.emplace(label.fontPath, font);
  }
}
void OverlayRenderer::render(
    const DebugScene& scene,
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up,
    double fovy,
    int width,
    int height)
{
  const OverlayProjection view(eye, center, up, fovy, width, height);
  const auto& io = ImGui::GetIO();
  const double sx = std::max(1e-6f, io.DisplayFramebufferScale.x);
  const double sy = std::max(1e-6f, io.DisplayFramebufferScale.y);
  auto logical = [&](const Eigen::Vector2d& p) {
    return ImVec2(
        static_cast<float>(p.x() / sx), static_cast<float>(p.y() / sy));
  };
  auto* draw = ImGui::GetBackgroundDrawList();
  draw->PushClipRect(
      ImVec2(0, 0),
      ImVec2(static_cast<float>(width / sx), static_cast<float>(height / sy)),
      true);
  // Thin diagnostic quads must preserve their requested opaque pixel width.
  // The fill fringe otherwise consumes the interior of two-pixel arrows.
  const auto previousFlags = draw->Flags;
  draw->Flags &= ~ImDrawListFlags_AntiAliasedFill;
  for (const auto& line : scene.lines) {
    if (!line.alwaysOnTop || line.pixelWidth <= 0)
      continue;
    Eigen::Vector3d a = line.from, b = line.to;
    if (!view.clip(a, b))
      continue;
    const Eigen::Vector2d pa = view.project(a).head<2>(),
                          pb = view.project(b).head<2>();
    Eigen::Vector2d tangent = pb - pa;
    if (!tangent.allFinite() || tangent.norm() < 1e-8)
      continue;
    tangent.normalize();
    const Eigen::Vector2d offset(
        -tangent.y() * line.pixelWidth * 0.5,
        tangent.x() * line.pixelWidth * 0.5);
    const std::array<ImVec2, 4> points{
        {logical(pa + offset),
         logical(pa - offset),
         logical(pb - offset),
         logical(pb + offset)}};
    draw->AddConvexPolyFilled(
        points.data(),
        static_cast<int>(points.size()),
        overlayColor(line.rgba));
  }
  draw->Flags = previousFlags;
  for (const auto& label : scene.labels) {
    if (label.text.empty() || !label.position.allFinite()
        || label.characterSize <= 0)
      continue;
    Eigen::Vector3d p
        = label.screenSpace ? label.position : view.project(label.position);
    if (!p.allFinite() || (!label.screenSpace && p.z() <= 1e-4))
      continue;
    const double pixelSize
        = (label.screenSpace || label.pixelSize)
              ? label.characterSize
              : label.characterSize * view.pixelScale / p.z();
    if (!std::isfinite(pixelSize) || pixelSize <= 0)
      continue;
    auto* font = label.fontPath.empty() ? ImGui::GetFont()
                                        : mImpl->fonts.at(label.fontPath);
    // Glyph size is expressed in logical units, independent of the widget DPI
    // scale.
    const float size = static_cast<float>(pixelSize / sy);
    ImVec2 position = logical(p.head<2>());
    auto* labelDraw = label.screenSpace ? ImGui::GetForegroundDrawList() : draw;
    if (!label.screenSpace) {
      const ImVec2 extent
          = font->CalcTextSizeA(size, FLT_MAX, 0, label.text.c_str());
      position.x -= extent.x * 0.5f;
      position.y -= extent.y * 0.5f;
    }
    if (label.outlineWidth > 0) {
      for (const auto& delta : std::array<Eigen::Vector2d, 8>{
               {{-1, -1},
                {0, -1},
                {1, -1},
                {-1, 0},
                {1, 0},
                {-1, 1},
                {0, 1},
                {1, 1}}}) {
        const ImVec2 offset = logical(delta * label.outlineWidth);
        labelDraw->AddText(
            font,
            size,
            ImVec2(position.x + offset.x, position.y + offset.y),
            overlayColor(label.outlineColor),
            label.text.c_str());
      }
    }
    labelDraw->AddText(
        font, size, position, overlayColor(label.rgba), label.text.c_str());
  }
  draw->PopClipRect();
}
} // namespace dart::gui::detail
