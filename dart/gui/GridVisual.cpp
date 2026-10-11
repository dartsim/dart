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

#include <dart/gui/GridVisual.hpp>
#include <dart/gui/detail/overlay_adapter.hpp>

#include <array>
#include <stdexcept>

#include <cmath>

namespace dart::gui {
struct GridVisual::Impl
{
  std::size_t cells = 20, minorPerMajor = 5;
  double step = 0.1;
  PlaneType plane = PlaneType::XY;
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  Eigen::Vector4d major{0.4, 0.4, 0.4, 1}, minor{0.5, 0.5, 0.5, 1};
  float axisWidth = 2, majorWidth = 2, minorWidth = 1;
  bool displayed = true, dirty = true;
  DebugScene scene;
};
GridVisual::GridVisual() : mImpl(std::make_unique<Impl>()) {}
GridVisual::~GridVisual() = default;
void GridVisual::setNumCells(std::size_t value)
{
  mImpl->cells = value;
  mImpl->dirty = true;
}
std::size_t GridVisual::getNumCells() const
{
  return mImpl->cells;
}
void GridVisual::setMinorLineStepSize(double value)
{
  if (!std::isfinite(value))
    throw std::invalid_argument("Grid step must be finite");
  mImpl->step = std::max(0.0, value);
  mImpl->dirty = true;
}
double GridVisual::getMinorLineStepSize() const
{
  return mImpl->step;
}
void GridVisual::setNumMinorLinesPerMajorLine(std::size_t value)
{
  mImpl->minorPerMajor = value;
  mImpl->dirty = true;
}
std::size_t GridVisual::getNumMinorLinesPerMajorLine() const
{
  return mImpl->minorPerMajor;
}
void GridVisual::setPlaneType(PlaneType value)
{
  if (value != PlaneType::XY && value != PlaneType::YZ
      && value != PlaneType::ZX)
    throw std::invalid_argument("Unknown grid plane");
  mImpl->plane = value;
  mImpl->dirty = true;
}
GridVisual::PlaneType GridVisual::getPlaneType() const
{
  return mImpl->plane;
}
void GridVisual::setOffset(const Eigen::Vector3d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("Grid offset must be finite");
  mImpl->offset = value;
  mImpl->dirty = true;
}
const Eigen::Vector3d& GridVisual::getOffset() const
{
  return mImpl->offset;
}
void GridVisual::display(bool value)
{
  mImpl->displayed = value;
}
bool GridVisual::isDisplayed() const
{
  return mImpl->displayed;
}
void GridVisual::setMajorLineColor(const Eigen::Vector4d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("Grid color must be finite");
  mImpl->major = value;
  mImpl->dirty = true;
}
Eigen::Vector4d GridVisual::getMajorLineColor() const
{
  return mImpl->major;
}
void GridVisual::setMinorLineColor(const Eigen::Vector4d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("Grid color must be finite");
  mImpl->minor = value;
  mImpl->dirty = true;
}
Eigen::Vector4d GridVisual::getMinorLineColor() const
{
  return mImpl->minor;
}
void GridVisual::setAxisLineWidth(float value)
{
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument("Grid width must be finite and positive");
  mImpl->axisWidth = value;
  mImpl->dirty = true;
}
float GridVisual::getAxisLineWidth() const
{
  return mImpl->axisWidth;
}
void GridVisual::setMajorLineWidth(float value)
{
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument("Grid width must be finite and positive");
  mImpl->majorWidth = value;
  mImpl->dirty = true;
}
float GridVisual::getMajorLineWidth() const
{
  return mImpl->majorWidth;
}
void GridVisual::setMinorLineWidth(float value)
{
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument("Grid width must be finite and positive");
  mImpl->minorWidth = value;
  mImpl->dirty = true;
}
float GridVisual::getMinorLineWidth() const
{
  return mImpl->minorWidth;
}
void GridVisual::refresh()
{
  auto& d = *mImpl;
  if (!d.dirty || !d.displayed)
    return;
  d.scene = {};
  if (d.cells == 0 || d.step == 0) {
    d.dirty = false;
    return;
  }
  const double extent = d.cells * d.step;
  if (!std::isfinite(extent) || d.cells > (d.scene.lines.max_size() - 4) / 4)
    throw std::length_error("Grid dimensions exceed representable geometry");
  d.scene.lines.reserve(4 + 4 * d.cells);
  const int first = d.plane == PlaneType::XY   ? 0
                    : d.plane == PlaneType::YZ ? 1
                                               : 2;
  const int second = (first + 1) % 3;
  const std::array<Eigen::Vector4d, 3> colors{
      {{0.9, 0.1, 0.1, 1}, {0.1, 0.9, 0.1, 1}, {0.1, 0.1, 0.9, 1}}};
  auto add = [&](Eigen::Vector3d a,
                 Eigen::Vector3d b,
                 const Eigen::Vector4d& color,
                 float width) {
    DebugLineDescriptor line;
    line.from = a + d.offset;
    line.to = b + d.offset;
    line.rgba = color;
    line.pixelWidth = width;
    line.alwaysOnTop = false;
    d.scene.lines.push_back(std::move(line));
  };
  for (int axis : {first, second}) {
    Eigen::Vector3d end = Eigen::Vector3d::Zero();
    end[axis] = extent;
    add(Eigen::Vector3d::Zero(), end, colors[axis], d.axisWidth);
    add(Eigen::Vector3d::Zero(), -end, d.major, d.axisWidth);
  }
  for (std::size_t i = 1; i <= d.cells; ++i) {
    const bool major = d.minorPerMajor > 0 && i % d.minorPerMajor == 0;
    for (auto axes : {std::pair<int, int>{first, second}, {second, first}}) {
      for (double sign : {-1.0, 1.0}) {
        Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = a;
        a[axes.first] = -extent;
        b[axes.first] = extent;
        a[axes.second] = b[axes.second] = sign * i * d.step;
        add(a,
            b,
            major ? d.major : d.minor,
            major ? d.majorWidth : d.minorWidth);
      }
    }
  }
  d.dirty = false;
}
void detail::OverlayAccess::append(const GridVisual& grid, DebugScene& scene)
{
  if (grid.mImpl->displayed)
    scene.lines.insert(
        scene.lines.end(),
        grid.mImpl->scene.lines.begin(),
        grid.mImpl->scene.lines.end());
}
} // namespace dart::gui
