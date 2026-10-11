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
#include <dart/gui/detail/overlay_adapter.hpp>

#include <stdexcept>

#include <cmath>

namespace dart::gui {
struct DebugOverlay::Impl
{
  DebugScene scene;
  std::string fontPath;
  float lineWidth = 2.0f;
  double characterSize = 0.08;
  double outlineWidth = 0.0;
  Eigen::Vector4d outlineColor{0, 0, 0, 1};
};

DebugOverlay::DebugOverlay() : mImpl(std::make_unique<Impl>()) {}
DebugOverlay::~DebugOverlay() = default;
void DebugOverlay::setFont(const std::string& path)
{
  mImpl->fontPath = path;
}
const std::string& DebugOverlay::getFont() const
{
  return mImpl->fontPath;
}
void DebugOverlay::setLineWidth(float width)
{
  if (!std::isfinite(width) || width <= 0)
    throw std::invalid_argument(
        "Overlay line width must be finite and positive");
  mImpl->lineWidth = width;
}
float DebugOverlay::getLineWidth() const
{
  return mImpl->lineWidth;
}
void DebugOverlay::setCharacterSize(double size)
{
  if (!std::isfinite(size) || size <= 0)
    throw std::invalid_argument(
        "Overlay character size must be finite and positive");
  mImpl->characterSize = size;
}
double DebugOverlay::getCharacterSize() const
{
  return mImpl->characterSize;
}
std::size_t DebugOverlay::addLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector4d& color)
{
  if (!start.allFinite() || !end.allFinite() || !color.allFinite())
    throw std::invalid_argument(
        "Overlay lines require finite coordinates and color");
  DebugLineDescriptor line;
  line.from = start;
  line.to = end;
  line.rgba = color;
  mImpl->scene.lines.push_back(std::move(line));
  return mImpl->scene.lines.size() - 1;
}
std::size_t DebugOverlay::addLabel(
    const Eigen::Vector3d& position,
    const std::string& text,
    const Eigen::Vector4d& color)
{
  return addLabel(position, text, color, mImpl->characterSize);
}
std::size_t DebugOverlay::addLabel(
    const Eigen::Vector3d& position,
    const std::string& text,
    const Eigen::Vector4d& color,
    double size)
{
  if (!position.allFinite() || !color.allFinite() || !std::isfinite(size)
      || size <= 0)
    throw std::invalid_argument(
        "Overlay labels require finite coordinates, color, and positive size");
  DebugLabelDescriptor label;
  label.position = position;
  label.text = text;
  label.rgba = color;
  label.characterSize = size;
  mImpl->scene.labels.push_back(std::move(label));
  return mImpl->scene.labels.size() - 1;
}
std::size_t DebugOverlay::addLabelPixels(
    const Eigen::Vector3d& position,
    const std::string& text,
    const Eigen::Vector4d& color,
    double size)
{
  const auto index = addLabel(position, text, color, size);
  mImpl->scene.labels[index].pixelSize = true;
  return index;
}
std::size_t DebugOverlay::addScreenLabel(
    const Eigen::Vector2d& position,
    const std::string& text,
    const Eigen::Vector4d& color,
    double size)
{
  const auto index = addLabel(
      Eigen::Vector3d(position.x(), position.y(), 0), text, color, size);
  mImpl->scene.labels[index].screenSpace = true;
  return index;
}
void DebugOverlay::setTextOutlineWidth(double width)
{
  if (!std::isfinite(width) || width < 0)
    throw std::invalid_argument(
        "Overlay outline width must be finite and nonnegative");
  mImpl->outlineWidth = width;
}
double DebugOverlay::getTextOutlineWidth() const
{
  return mImpl->outlineWidth;
}
void DebugOverlay::setTextOutlineColor(const Eigen::Vector4d& color)
{
  if (!color.allFinite())
    throw std::invalid_argument("Overlay color must be finite");
  mImpl->outlineColor = color;
}
Eigen::Vector4d DebugOverlay::getTextOutlineColor() const
{
  return mImpl->outlineColor;
}
void DebugOverlay::clear()
{
  mImpl->scene = {};
}
std::size_t DebugOverlay::getNumLines() const
{
  return mImpl->scene.lines.size();
}
std::size_t DebugOverlay::getNumLabels() const
{
  return mImpl->scene.labels.size();
}
void DebugOverlay::refresh() {}
void detail::OverlayAccess::append(
    const DebugOverlay& overlay, DebugScene& scene)
{
  const auto& data = *overlay.mImpl;
  for (auto line : data.scene.lines) {
    line.pixelWidth = data.lineWidth;
    scene.lines.push_back(std::move(line));
  }
  for (auto label : data.scene.labels) {
    label.fontPath = data.fontPath;
    label.outlineWidth = data.outlineWidth;
    label.outlineColor = data.outlineColor;
    scene.labels.push_back(std::move(label));
  }
}
} // namespace dart::gui
