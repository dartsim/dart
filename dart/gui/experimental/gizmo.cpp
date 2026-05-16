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

#include <dart/gui/gizmo.hpp>

#include <dart/dynamics/frame.hpp>

#include <algorithm>
#include <utility>

#include <cmath>

namespace dart::gui {
namespace {

void appendLine(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    std::string label)
{
  DebugLineDescriptor line;
  line.from = from;
  line.to = to;
  line.rgba = color;
  line.label = std::move(label);
  lines.push_back(std::move(line));
}

void appendArrowLines(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& axis,
    double length,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  if (length <= 0.0 || !std::isfinite(length) || !axis.allFinite()) {
    return;
  }

  const Eigen::Vector3d direction = axis.normalized();
  if (!direction.allFinite()) {
    return;
  }

  const Eigen::Vector3d tip = origin + direction * length;
  appendLine(lines, origin, tip, color, label);

  const Eigen::Vector3d seed = std::abs(direction.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d side = direction.cross(seed).normalized();
  if (!side.allFinite()) {
    return;
  }

  const double headLength = length * 0.25;
  const double headWidth = headLength * 0.45;
  const Eigen::Vector3d base = tip - direction * headLength;
  appendLine(lines, tip, base + side * headWidth, color, label + ".head_a");
  appendLine(lines, tip, base - side * headWidth, color, label + ".head_b");
}

void appendFreeMoveHandle(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& origin,
    double radius,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  if (radius <= 0.0 || !std::isfinite(radius) || !origin.allFinite()) {
    return;
  }

  appendLine(
      lines,
      origin - Eigen::Vector3d::UnitX() * radius,
      origin + Eigen::Vector3d::UnitX() * radius,
      color,
      label + ".free_x");
  appendLine(
      lines,
      origin - Eigen::Vector3d::UnitY() * radius,
      origin + Eigen::Vector3d::UnitY() * radius,
      color,
      label + ".free_y");
  appendLine(
      lines,
      origin - Eigen::Vector3d::UnitZ() * radius,
      origin + Eigen::Vector3d::UnitZ() * radius,
      color,
      label + ".free_z");
}

std::string makeGizmoLabel(const Gizmo& gizmo)
{
  return gizmo.label.empty() ? "gizmo" : gizmo.label;
}

} // namespace

std::vector<DebugLineDescriptor> makeGizmoDebugLines(
    const Gizmo& gizmo, double scale)
{
  std::vector<DebugLineDescriptor> lines;
  if (gizmo.target == nullptr || gizmo.size <= 0.0 || !std::isfinite(gizmo.size)
      || scale <= 0.0 || !std::isfinite(scale)) {
    return lines;
  }

  if (!hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)
      && !hasGizmoFlag(gizmo.flags, GizmoFlags::TranslateXY)) {
    return lines;
  }

  const Eigen::Isometry3d transform = gizmo.target->getWorldTransform();
  if (!transform.matrix().allFinite()) {
    return lines;
  }

  const Eigen::Vector3d origin = transform.translation();
  const Eigen::Matrix3d rotation = transform.linear();
  const double length = gizmo.size * scale;
  const std::string label = makeGizmoLabel(gizmo);

  lines.reserve(12);
  appendArrowLines(
      lines, origin, rotation.col(0), length, gizmo.colors.x, label + ".x");
  appendArrowLines(
      lines, origin, rotation.col(1), length, gizmo.colors.y, label + ".y");
  if (hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)) {
    appendArrowLines(
        lines, origin, rotation.col(2), length, gizmo.colors.z, label + ".z");
  }
  appendFreeMoveHandle(
      lines, origin, length * 0.18, gizmo.colors.highlight, label);

  return lines;
}

std::vector<DebugLineDescriptor> makeGizmoDebugLines(
    const std::vector<Gizmo>& gizmos, double scale)
{
  std::vector<DebugLineDescriptor> lines;
  for (const Gizmo& gizmo : gizmos) {
    auto gizmoLines = makeGizmoDebugLines(gizmo, scale);
    lines.insert(
        lines.end(),
        std::make_move_iterator(gizmoLines.begin()),
        std::make_move_iterator(gizmoLines.end()));
  }
  return lines;
}

} // namespace dart::gui
