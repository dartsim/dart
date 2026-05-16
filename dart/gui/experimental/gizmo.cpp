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
#include <dart/dynamics/simple_frame.hpp>

#include <algorithm>
#include <limits>
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

struct RaySegmentHit
{
  double rayDistance = 0.0;
  double separation = 0.0;
  Eigen::Vector3d segmentPoint = Eigen::Vector3d::Zero();
};

std::string makeGizmoLabel(const Gizmo& gizmo)
{
  return gizmo.label.empty() ? "gizmo" : gizmo.label;
}

GizmoHandleKind handleKindForAxisIndex(int axisIndex)
{
  switch (axisIndex) {
    case 0:
      return GizmoHandleKind::TranslateX;
    case 1:
      return GizmoHandleKind::TranslateY;
    case 2:
      return GizmoHandleKind::TranslateZ;
    default:
      return GizmoHandleKind::None;
  }
}

std::optional<RaySegmentHit> intersectRaySegment(
    const PickRay& ray,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    double radius)
{
  if (!ray.origin.allFinite() || !ray.direction.allFinite() || !from.allFinite()
      || !to.allFinite() || radius <= 0.0 || !std::isfinite(radius)) {
    return std::nullopt;
  }

  const double rayNorm = ray.direction.norm();
  const Eigen::Vector3d segment = to - from;
  const double segmentLengthSquared = segment.squaredNorm();
  if (!std::isfinite(rayNorm) || rayNorm < 1e-12
      || !std::isfinite(segmentLengthSquared) || segmentLengthSquared < 1e-18) {
    return std::nullopt;
  }

  const Eigen::Vector3d direction = ray.direction / rayNorm;
  const Eigen::Vector3d offset = ray.origin - from;
  const double raySegmentDot = direction.dot(segment);
  const double rayOffsetDot = direction.dot(offset);
  const double segmentOffsetDot = segment.dot(offset);
  const double denom = segmentLengthSquared - raySegmentDot * raySegmentDot;

  double rayDistance = 0.0;
  double segmentParameter = 0.0;
  if (std::abs(denom) > 1e-12 && std::isfinite(denom)) {
    rayDistance = (raySegmentDot * segmentOffsetDot
                   - segmentLengthSquared * rayOffsetDot)
                  / denom;
    segmentParameter
        = (segmentOffsetDot - raySegmentDot * rayOffsetDot) / denom;
  } else {
    segmentParameter
        = std::clamp(-segmentOffsetDot / segmentLengthSquared, 0.0, 1.0);
    rayDistance = direction.dot(from + segment * segmentParameter - ray.origin);
  }

  if (!std::isfinite(rayDistance) || !std::isfinite(segmentParameter)) {
    return std::nullopt;
  }

  segmentParameter = std::clamp(segmentParameter, 0.0, 1.0);
  rayDistance = direction.dot(from + segment * segmentParameter - ray.origin);
  if (rayDistance < 0.0) {
    rayDistance = 0.0;
    segmentParameter = std::clamp(
        segment.dot(ray.origin - from) / segmentLengthSquared, 0.0, 1.0);
  }

  const Eigen::Vector3d rayPoint = ray.origin + direction * rayDistance;
  const Eigen::Vector3d segmentPoint = from + segment * segmentParameter;
  const double separation = (rayPoint - segmentPoint).norm();
  if (!std::isfinite(separation) || separation > radius) {
    return std::nullopt;
  }

  RaySegmentHit hit;
  hit.rayDistance = rayDistance;
  hit.separation = separation;
  hit.segmentPoint = segmentPoint;
  return hit;
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

std::optional<GizmoHandleHit> pickNearestGizmoHandle(
    const std::vector<Gizmo>& gizmos,
    const PickRay& ray,
    double scale,
    double handleRadius)
{
  if (scale <= 0.0 || !std::isfinite(scale) || handleRadius <= 0.0
      || !std::isfinite(handleRadius)) {
    return std::nullopt;
  }

  std::optional<GizmoHandleHit> nearest;
  double nearestSeparation = std::numeric_limits<double>::infinity();
  for (std::size_t gizmoIndex = 0; gizmoIndex < gizmos.size(); ++gizmoIndex) {
    const Gizmo& gizmo = gizmos[gizmoIndex];
    if (gizmo.target == nullptr || gizmo.size <= 0.0
        || !std::isfinite(gizmo.size)
        || (!hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)
            && !hasGizmoFlag(gizmo.flags, GizmoFlags::TranslateXY))) {
      continue;
    }

    const Eigen::Isometry3d transform = gizmo.target->getWorldTransform();
    if (!transform.matrix().allFinite()) {
      continue;
    }

    const Eigen::Vector3d origin = transform.translation();
    const Eigen::Matrix3d rotation = transform.linear();
    const double length = gizmo.size * scale;
    const double radius = handleRadius * scale;
    const int axisCount
        = hasGizmoFlag(gizmo.flags, GizmoFlags::Translate) ? 3 : 2;
    for (int axisIndex = 0; axisIndex < axisCount; ++axisIndex) {
      Eigen::Vector3d axis = rotation.col(axisIndex);
      const double axisNorm = axis.norm();
      if (!axis.allFinite() || axisNorm < 1e-12 || !std::isfinite(axisNorm)) {
        continue;
      }
      axis /= axisNorm;

      const auto hit
          = intersectRaySegment(ray, origin, origin + axis * length, radius);
      if (!hit) {
        continue;
      }

      if (!nearest || hit->rayDistance < nearest->distance
          || (std::abs(hit->rayDistance - nearest->distance) < 1e-12
              && hit->separation < nearestSeparation)) {
        GizmoHandleHit gizmoHit;
        gizmoHit.gizmoIndex = gizmoIndex;
        gizmoHit.handle = handleKindForAxisIndex(axisIndex);
        gizmoHit.distance = hit->rayDistance;
        gizmoHit.point = hit->segmentPoint;
        gizmoHit.axis = axis;
        nearest = gizmoHit;
        nearestSeparation = hit->separation;
      }
    }
  }

  return nearest;
}

bool translateGizmoTarget(Gizmo& gizmo, const Eigen::Vector3d& worldTranslation)
{
  if (gizmo.target == nullptr || !worldTranslation.allFinite()) {
    return false;
  }

  const auto simpleFrame
      = std::dynamic_pointer_cast<dart::dynamics::SimpleFrame>(gizmo.target);
  if (!simpleFrame) {
    return false;
  }

  Eigen::Isometry3d transform = simpleFrame->getWorldTransform();
  transform.translation() += worldTranslation;
  simpleFrame->setTransform(transform, dart::dynamics::Frame::World());
  if (gizmo.onChanged) {
    gizmo.onChanged(transform);
  }
  return true;
}

} // namespace dart::gui
