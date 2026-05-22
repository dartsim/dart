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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_EXAMPLES_GUI_SOURCE_GRID_HPP_
#define DART_EXAMPLES_GUI_SOURCE_GRID_HPP_

#include <dart/gui/panel.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <string>

#include <cmath>

namespace dart::examples {

enum class SourceOwnedGridPlane
{
  XY,
  YZ,
  ZX,
};

struct SourceOwnedGridState
{
  std::shared_ptr<dart::dynamics::SimpleFrame> minorFrame;
  std::shared_ptr<dart::dynamics::SimpleFrame> majorFrame;
  std::shared_ptr<dart::dynamics::SimpleFrame> axisFrame;
  SourceOwnedGridPlane plane = SourceOwnedGridPlane::XY;
  bool visible = true;
  double xOffset = 0.0;
  double yOffset = 0.0;
  double zOffset = -0.01;
  double lineCount = 20.0;
  double lineStepSize = 0.1;
  double minorLinesPerMajorLine = 5.0;
  double minorLineWidth = 1.0;
  double majorLineWidth = 2.0;
  double axisLineWidth = 3.0;
  Eigen::Vector4d minorLineColor = Eigen::Vector4d(0.45, 0.45, 0.45, 0.42);
  Eigen::Vector4d majorLineColor = Eigen::Vector4d(0.56, 0.56, 0.56, 0.58);
  Eigen::Vector4d axisLineColor = Eigen::Vector4d(0.20, 0.26, 0.34, 0.82);
};

namespace detail {

enum class SourceOwnedGridLayer
{
  Minor,
  Major,
  Axis,
};

inline int roundedClampedInt(double& value, int minimum, int maximum)
{
  int rounded = static_cast<int>(std::round(value));
  rounded = std::clamp(rounded, minimum, maximum);
  value = static_cast<double>(rounded);
  return rounded;
}

inline int roundedEvenCellCount(double& value)
{
  int cellCount = roundedClampedInt(value, 2, 80);
  if (cellCount % 2 != 0) {
    ++cellCount;
  }
  cellCount = std::clamp(cellCount, 2, 80);
  value = static_cast<double>(cellCount);
  return cellCount;
}

inline Eigen::Vector3d sourceOwnedGridPoint(
    SourceOwnedGridPlane plane, double u, double v)
{
  switch (plane) {
    case SourceOwnedGridPlane::XY:
      return Eigen::Vector3d(u, v, 0.0);
    case SourceOwnedGridPlane::YZ:
      return Eigen::Vector3d(0.0, u, v);
    case SourceOwnedGridPlane::ZX:
      return Eigen::Vector3d(u, 0.0, v);
  }
  return Eigen::Vector3d(u, v, 0.0);
}

inline SourceOwnedGridLayer sourceOwnedGridLayerForLine(
    int index, int cellCount, int majorStride)
{
  if (index == cellCount / 2) {
    return SourceOwnedGridLayer::Axis;
  }
  if (majorStride > 0 && index % majorStride == 0) {
    return SourceOwnedGridLayer::Major;
  }
  return SourceOwnedGridLayer::Minor;
}

inline void addSourceOwnedGridLine(
    dart::dynamics::LineSegmentShape& shape,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  const auto startIndex = shape.addVertex(start);
  shape.addVertex(end, startIndex);
}

inline std::shared_ptr<dart::dynamics::LineSegmentShape>
makeSourceOwnedGridShape(
    const SourceOwnedGridState& state,
    SourceOwnedGridLayer layer,
    int cellCount,
    int majorStride,
    double lineStepSize,
    double thickness)
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(
      static_cast<float>(thickness));
  shape->setDataVariance(dart::dynamics::Shape::DYNAMIC);

  const double halfExtent = 0.5 * static_cast<double>(cellCount) * lineStepSize;
  for (int i = 0; i <= cellCount; ++i) {
    if (sourceOwnedGridLayerForLine(i, cellCount, majorStride) != layer) {
      continue;
    }

    const double coordinate
        = -halfExtent + static_cast<double>(i) * lineStepSize;
    addSourceOwnedGridLine(
        *shape,
        sourceOwnedGridPoint(state.plane, -halfExtent, coordinate),
        sourceOwnedGridPoint(state.plane, halfExtent, coordinate));
    addSourceOwnedGridLine(
        *shape,
        sourceOwnedGridPoint(state.plane, coordinate, -halfExtent),
        sourceOwnedGridPoint(state.plane, coordinate, halfExtent));
  }

  return shape;
}

inline Eigen::Vector3d sourceOwnedGridOffset(const SourceOwnedGridState& state)
{
  return Eigen::Vector3d(state.xOffset, state.yOffset, state.zOffset);
}

inline std::shared_ptr<dart::dynamics::SimpleFrame> makeSourceOwnedGridFrame(
    const std::string& name, const Eigen::Vector4d& color)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  frame->setShape(std::make_shared<dart::dynamics::LineSegmentShape>());
  frame->getVisualAspect(true)->setRGBA(color);
  return frame;
}

inline void setSourceOwnedGridFrameVisible(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& frame, bool visible)
{
  if (frame == nullptr) {
    return;
  }

  auto* visual = frame->getVisualAspect(true);
  if (visible) {
    visual->show();
  } else {
    visual->hide();
  }
}

} // namespace detail

inline void updateSourceOwnedGrid(SourceOwnedGridState& state)
{
  int cellCount = detail::roundedEvenCellCount(state.lineCount);
  int majorStride = detail::roundedClampedInt(
      state.minorLinesPerMajorLine, 1, std::max(1, cellCount));
  state.lineStepSize = std::clamp(state.lineStepSize, 0.01, 1.0);
  state.minorLineWidth = std::clamp(state.minorLineWidth, 1.0, 8.0);
  state.majorLineWidth = std::clamp(state.majorLineWidth, 1.0, 8.0);
  state.axisLineWidth = std::clamp(state.axisLineWidth, 1.0, 8.0);

  const Eigen::Vector3d offset = detail::sourceOwnedGridOffset(state);
  const auto assignLayer
      = [&](const std::shared_ptr<dart::dynamics::SimpleFrame>& frame,
            detail::SourceOwnedGridLayer layer,
            double thickness,
            const Eigen::Vector4d& color) {
          if (frame == nullptr) {
            return;
          }

          auto shape = detail::makeSourceOwnedGridShape(
              state,
              layer,
              cellCount,
              majorStride,
              state.lineStepSize,
              thickness);
          frame->setShape(shape);
          frame->setTranslation(offset);
          frame->getVisualAspect(true)->setRGBA(color);
          detail::setSourceOwnedGridFrameVisible(
              frame, state.visible && !shape->getConnections().empty());
        };

  assignLayer(
      state.minorFrame,
      detail::SourceOwnedGridLayer::Minor,
      state.minorLineWidth,
      state.minorLineColor);
  assignLayer(
      state.majorFrame,
      detail::SourceOwnedGridLayer::Major,
      state.majorLineWidth,
      state.majorLineColor);
  assignLayer(
      state.axisFrame,
      detail::SourceOwnedGridLayer::Axis,
      state.axisLineWidth,
      state.axisLineColor);
}

inline void setSourceOwnedGridVisible(SourceOwnedGridState& state, bool visible)
{
  state.visible = visible;
  updateSourceOwnedGrid(state);
}

inline void attachSourceOwnedGridFrames(
    const dart::simulation::WorldPtr& world,
    SourceOwnedGridState& state,
    const std::string& baseName)
{
  if (world == nullptr) {
    return;
  }

  state.minorFrame = detail::makeSourceOwnedGridFrame(
      baseName + "_minor_lines", state.minorLineColor);
  state.majorFrame = detail::makeSourceOwnedGridFrame(
      baseName + "_major_lines", state.majorLineColor);
  state.axisFrame = detail::makeSourceOwnedGridFrame(
      baseName + "_axis_lines", state.axisLineColor);
  world->addSimpleFrame(state.minorFrame);
  world->addSimpleFrame(state.majorFrame);
  world->addSimpleFrame(state.axisFrame);
  updateSourceOwnedGrid(state);
}

inline const char* sourceOwnedGridPlaneLabel(SourceOwnedGridPlane plane)
{
  switch (plane) {
    case SourceOwnedGridPlane::XY:
      return "XY-Plane";
    case SourceOwnedGridPlane::YZ:
      return "YZ-Plane";
    case SourceOwnedGridPlane::ZX:
      return "ZX-Plane";
  }
  return "XY-Plane";
}

inline void addSourceOwnedGridPanelControls(
    dart::gui::PanelBuilder& builder, SourceOwnedGridState& state)
{
  bool changed = false;
  if (builder.checkbox("Show Grid", state.visible)) {
    changed = true;
  }

  builder.text(
      std::string("Grid Plane: ") + sourceOwnedGridPlaneLabel(state.plane));
  if (builder.button("XY-Plane")) {
    state.plane = SourceOwnedGridPlane::XY;
    changed = true;
  }
  builder.sameLine();
  if (builder.button("YZ-Plane")) {
    state.plane = SourceOwnedGridPlane::YZ;
    changed = true;
  }
  builder.sameLine();
  if (builder.button("ZX-Plane")) {
    state.plane = SourceOwnedGridPlane::ZX;
    changed = true;
  }

  changed |= builder.slider("Grid X Offset", state.xOffset, -4.0, 4.0);
  changed |= builder.slider("Grid Y Offset", state.yOffset, -4.0, 4.0);
  changed |= builder.slider("Grid Z Offset", state.zOffset, -4.0, 4.0);
  changed |= builder.slider("Line Count", state.lineCount, 2.0, 80.0);
  changed |= builder.slider("Line Step Size", state.lineStepSize, 0.01, 1.0);
  changed |= builder.slider(
      "Minor Lines per Major Line", state.minorLinesPerMajorLine, 1.0, 20.0);
  changed |= builder.slider("Axis Line Width", state.axisLineWidth, 1.0, 8.0);
  changed |= builder.slider("Major Line Width", state.majorLineWidth, 1.0, 8.0);
  changed |= builder.slider("Minor Line Width", state.minorLineWidth, 1.0, 8.0);
  changed |= builder.colorEdit("Major Line Color", state.majorLineColor);
  changed |= builder.colorEdit("Minor Line Color", state.minorLineColor);
  changed |= builder.colorEdit("Axis Line Color", state.axisLineColor);

  if (changed) {
    updateSourceOwnedGrid(state);
  }
}

} // namespace dart::examples

#endif // DART_EXAMPLES_GUI_SOURCE_GRID_HPP_
