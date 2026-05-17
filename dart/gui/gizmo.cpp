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
#include <array>
#include <limits>
#include <utility>

#include <cmath>

namespace dart::gui {
namespace {

constexpr int kRingSegments = 48;
constexpr double kRingRadiusScale = 0.62;
constexpr double kPlaneHandleOffsetScale = 0.18;
constexpr double kPlaneHandleSizeScale = 0.15;
constexpr double kHandleThicknessScale = 0.022;
constexpr double kTwoPi = 6.2831853071795864769;

void appendLine(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    double thickness,
    std::string label)
{
  DebugLineDescriptor line;
  line.from = from;
  line.to = to;
  line.rgba = color;
  line.thickness = thickness;
  line.label = std::move(label);
  lines.push_back(std::move(line));
}

void appendArrowLines(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& axis,
    double length,
    const Eigen::Vector4d& color,
    double thickness,
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
  appendLine(lines, origin, tip, color, thickness, label);

  const Eigen::Vector3d seed = std::abs(direction.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d side = direction.cross(seed).normalized();
  if (!side.allFinite()) {
    return;
  }

  const double headLength = length * 0.18;
  const double headWidth = headLength * 0.65;
  const Eigen::Vector3d base = tip - direction * headLength;
  appendLine(
      lines, tip, base + side * headWidth, color, thickness, label + ".head_a");
  appendLine(
      lines, tip, base - side * headWidth, color, thickness, label + ".head_b");
}

void appendFreeMoveHandle(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& origin,
    double radius,
    const Eigen::Vector4d& color,
    double thickness,
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
      thickness,
      label + ".free_x");
  appendLine(
      lines,
      origin - Eigen::Vector3d::UnitY() * radius,
      origin + Eigen::Vector3d::UnitY() * radius,
      color,
      thickness,
      label + ".free_y");
  appendLine(
      lines,
      origin - Eigen::Vector3d::UnitZ() * radius,
      origin + Eigen::Vector3d::UnitZ() * radius,
      color,
      thickness,
      label + ".free_z");
}

void appendRingLines(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& firstAxis,
    const Eigen::Vector3d& secondAxis,
    double radius,
    const Eigen::Vector4d& color,
    double thickness,
    const std::string& label)
{
  if (!origin.allFinite() || !firstAxis.allFinite() || !secondAxis.allFinite()
      || radius <= 0.0 || !std::isfinite(radius)) {
    return;
  }

  const double firstNorm = firstAxis.norm();
  const double secondNorm = secondAxis.norm();
  if (!std::isfinite(firstNorm) || firstNorm < 1e-12
      || !std::isfinite(secondNorm) || secondNorm < 1e-12) {
    return;
  }

  const Eigen::Vector3d first = firstAxis / firstNorm;
  const Eigen::Vector3d second = secondAxis / secondNorm;
  for (int segment = 0; segment < kRingSegments; ++segment) {
    const double angle0 = kTwoPi * static_cast<double>(segment)
                          / static_cast<double>(kRingSegments);
    const double angle1 = kTwoPi * static_cast<double>(segment + 1)
                          / static_cast<double>(kRingSegments);
    const Eigen::Vector3d from
        = origin
          + (first * std::cos(angle0) + second * std::sin(angle0)) * radius;
    const Eigen::Vector3d to
        = origin
          + (first * std::cos(angle1) + second * std::sin(angle1)) * radius;
    appendLine(lines, from, to, color, thickness, label);
  }
}

void appendPlaneHandleLines(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& firstAxis,
    const Eigen::Vector3d& secondAxis,
    double offset,
    double size,
    const Eigen::Vector4d& color,
    double thickness,
    const std::string& label)
{
  if (!origin.allFinite() || !firstAxis.allFinite() || !secondAxis.allFinite()
      || offset <= 0.0 || !std::isfinite(offset) || size <= 0.0
      || !std::isfinite(size)) {
    return;
  }

  const double firstNorm = firstAxis.norm();
  const double secondNorm = secondAxis.norm();
  if (!std::isfinite(firstNorm) || firstNorm < 1e-12
      || !std::isfinite(secondNorm) || secondNorm < 1e-12) {
    return;
  }

  const Eigen::Vector3d first = firstAxis / firstNorm;
  const Eigen::Vector3d second = secondAxis / secondNorm;
  const Eigen::Vector3d corner = origin + (first + second) * offset;
  const Eigen::Vector3d a = corner;
  const Eigen::Vector3d b = corner + first * size;
  const Eigen::Vector3d c = corner + first * size + second * size;
  const Eigen::Vector3d d = corner + second * size;
  appendLine(lines, a, b, color, thickness, label);
  appendLine(lines, b, c, color, thickness, label);
  appendLine(lines, c, d, color, thickness, label);
  appendLine(lines, d, a, color, thickness, label);
  appendLine(lines, a, c, color, thickness * 0.65, label + ".diagonal");
}

struct RaySegmentHit
{
  double rayDistance = 0.0;
  double separation = 0.0;
  Eigen::Vector3d segmentPoint = Eigen::Vector3d::Zero();
};

struct PlaneHandleHit
{
  double rayDistance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

std::string makeGizmoLabel(const Gizmo& gizmo)
{
  return gizmo.label.empty() ? "gizmo" : gizmo.label;
}

bool isGizmoVisible(const Gizmo& gizmo)
{
  return !gizmo.isVisible || gizmo.isVisible();
}

bool isHighlightedHandle(
    GizmoHandleKind highlightedHandle, GizmoHandleKind candidate)
{
  return highlightedHandle != GizmoHandleKind::None
         && highlightedHandle == candidate;
}

Eigen::Vector4d colorForHandle(
    const Gizmo& gizmo,
    GizmoHandleKind highlightedHandle,
    GizmoHandleKind candidate,
    const Eigen::Vector4d& defaultColor)
{
  return isHighlightedHandle(highlightedHandle, candidate)
             ? gizmo.colors.highlight
             : defaultColor;
}

Eigen::Vector4d makeFreeMoveColor(const GizmoAxisColors& colors)
{
  Eigen::Vector4d color = (colors.x + colors.y + colors.z) / 3.0;
  color.w() = std::max({colors.x.w(), colors.y.w(), colors.z.w()});
  return color;
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

GizmoHandleKind planeHandleKindForNormalAxisIndex(int axisIndex)
{
  switch (axisIndex) {
    case 0:
      return GizmoHandleKind::TranslateYZ;
    case 1:
      return GizmoHandleKind::TranslateXZ;
    case 2:
      return GizmoHandleKind::TranslateXY;
    default:
      return GizmoHandleKind::None;
  }
}

GizmoHandleKind rotationHandleKindForAxisIndex(int axisIndex)
{
  switch (axisIndex) {
    case 0:
      return GizmoHandleKind::RotateX;
    case 1:
      return GizmoHandleKind::RotateY;
    case 2:
      return GizmoHandleKind::RotateZ;
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

std::optional<PlaneHandleHit> intersectRayPlaneHandle(
    const PickRay& ray,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& firstAxis,
    const Eigen::Vector3d& secondAxis,
    double offset,
    double size)
{
  if (!ray.origin.allFinite() || !ray.direction.allFinite()
      || !origin.allFinite() || !firstAxis.allFinite()
      || !secondAxis.allFinite() || offset <= 0.0 || !std::isfinite(offset)
      || size <= 0.0 || !std::isfinite(size)) {
    return std::nullopt;
  }

  const double rayNorm = ray.direction.norm();
  const double firstNorm = firstAxis.norm();
  const double secondNorm = secondAxis.norm();
  if (!std::isfinite(rayNorm) || rayNorm < 1e-12 || !std::isfinite(firstNorm)
      || firstNorm < 1e-12 || !std::isfinite(secondNorm)
      || secondNorm < 1e-12) {
    return std::nullopt;
  }

  const Eigen::Vector3d direction = ray.direction / rayNorm;
  const Eigen::Vector3d first = firstAxis / firstNorm;
  const Eigen::Vector3d second = secondAxis / secondNorm;
  Eigen::Vector3d normal = first.cross(second);
  const double normalNorm = normal.norm();
  if (!std::isfinite(normalNorm) || normalNorm < 1e-12) {
    return std::nullopt;
  }
  normal /= normalNorm;

  const Eigen::Vector3d corner = origin + (first + second) * offset;
  const double denom = direction.dot(normal);
  if (!std::isfinite(denom) || std::abs(denom) < 1e-12) {
    return std::nullopt;
  }

  const double rayDistance = (corner - ray.origin).dot(normal) / denom;
  if (!std::isfinite(rayDistance) || rayDistance < 0.0) {
    return std::nullopt;
  }

  const Eigen::Vector3d point = ray.origin + direction * rayDistance;
  const Eigen::Vector3d local = point - corner;
  const double firstParameter = local.dot(first);
  const double secondParameter = local.dot(second);
  if (!std::isfinite(firstParameter) || !std::isfinite(secondParameter)
      || firstParameter < 0.0 || firstParameter > size || secondParameter < 0.0
      || secondParameter > size) {
    return std::nullopt;
  }

  PlaneHandleHit hit;
  hit.rayDistance = rayDistance;
  hit.point = point;
  hit.normal = normal;
  return hit;
}

} // namespace

std::vector<DebugLineDescriptor> makeGizmoDebugLines(
    const Gizmo& gizmo, double scale, GizmoHandleKind highlightedHandle)
{
  std::vector<DebugLineDescriptor> lines;
  if (!isGizmoVisible(gizmo) || gizmo.target == nullptr || gizmo.size <= 0.0
      || !std::isfinite(gizmo.size) || scale <= 0.0 || !std::isfinite(scale)) {
    return lines;
  }

  if (!hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)
      && !hasGizmoFlag(gizmo.flags, GizmoFlags::TranslateXY)
      && !hasGizmoFlag(gizmo.flags, GizmoFlags::Rotate)) {
    return lines;
  }

  const Eigen::Isometry3d transform = gizmo.target->getWorldTransform();
  if (!transform.matrix().allFinite()) {
    return lines;
  }

  const Eigen::Vector3d origin = transform.translation();
  const Eigen::Matrix3d rotation = transform.linear();
  const double length = gizmo.size * scale;
  const double thickness = length * kHandleThicknessScale;
  const std::string label = makeGizmoLabel(gizmo);

  lines.reserve(12 + kRingSegments * 3);
  if (hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)
      || hasGizmoFlag(gizmo.flags, GizmoFlags::TranslateXY)) {
    appendArrowLines(
        lines,
        origin,
        rotation.col(0),
        length,
        colorForHandle(
            gizmo,
            highlightedHandle,
            GizmoHandleKind::TranslateX,
            gizmo.colors.x),
        thickness,
        label + ".x");
    appendArrowLines(
        lines,
        origin,
        rotation.col(1),
        length,
        colorForHandle(
            gizmo,
            highlightedHandle,
            GizmoHandleKind::TranslateY,
            gizmo.colors.y),
        thickness,
        label + ".y");
    if (hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)) {
      appendArrowLines(
          lines,
          origin,
          rotation.col(2),
          length,
          colorForHandle(
              gizmo,
              highlightedHandle,
              GizmoHandleKind::TranslateZ,
              gizmo.colors.z),
          thickness,
          label + ".z");
    }
    appendFreeMoveHandle(
        lines,
        origin,
        length * 0.14,
        makeFreeMoveColor(gizmo.colors),
        thickness,
        label);

    const double planeOffset = length * kPlaneHandleOffsetScale;
    const double planeSize = length * kPlaneHandleSizeScale;
    appendPlaneHandleLines(
        lines,
        origin,
        rotation.col(0),
        rotation.col(1),
        planeOffset,
        planeSize,
        colorForHandle(
            gizmo,
            highlightedHandle,
            GizmoHandleKind::TranslateXY,
            gizmo.colors.z),
        thickness,
        label + ".translate_xy");
    if (hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)) {
      appendPlaneHandleLines(
          lines,
          origin,
          rotation.col(1),
          rotation.col(2),
          planeOffset,
          planeSize,
          colorForHandle(
              gizmo,
              highlightedHandle,
              GizmoHandleKind::TranslateYZ,
              gizmo.colors.x),
          thickness,
          label + ".translate_yz");
      appendPlaneHandleLines(
          lines,
          origin,
          rotation.col(0),
          rotation.col(2),
          planeOffset,
          planeSize,
          colorForHandle(
              gizmo,
              highlightedHandle,
              GizmoHandleKind::TranslateXZ,
              gizmo.colors.y),
          thickness,
          label + ".translate_xz");
    }
  }

  if (hasGizmoFlag(gizmo.flags, GizmoFlags::Rotate)) {
    const double ringRadius = length * kRingRadiusScale;
    appendRingLines(
        lines,
        origin,
        rotation.col(1),
        rotation.col(2),
        ringRadius,
        colorForHandle(
            gizmo, highlightedHandle, GizmoHandleKind::RotateX, gizmo.colors.x),
        thickness,
        label + ".rotate_x");
    appendRingLines(
        lines,
        origin,
        rotation.col(2),
        rotation.col(0),
        ringRadius,
        colorForHandle(
            gizmo, highlightedHandle, GizmoHandleKind::RotateY, gizmo.colors.y),
        thickness,
        label + ".rotate_y");
    appendRingLines(
        lines,
        origin,
        rotation.col(0),
        rotation.col(1),
        ringRadius,
        colorForHandle(
            gizmo, highlightedHandle, GizmoHandleKind::RotateZ, gizmo.colors.z),
        thickness,
        label + ".rotate_z");
  }

  return lines;
}

std::vector<DebugLineDescriptor> makeGizmoDebugLines(
    const std::vector<Gizmo>& gizmos,
    double scale,
    std::optional<GizmoHandleHit> highlightedHandle)
{
  std::vector<DebugLineDescriptor> lines;
  for (std::size_t gizmoIndex = 0; gizmoIndex < gizmos.size(); ++gizmoIndex) {
    const Gizmo& gizmo = gizmos[gizmoIndex];
    const GizmoHandleKind handleToHighlight
        = (highlightedHandle && highlightedHandle->gizmoIndex == gizmoIndex)
              ? highlightedHandle->handle
              : GizmoHandleKind::None;
    auto gizmoLines = makeGizmoDebugLines(gizmo, scale, handleToHighlight);
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
    if (!isGizmoVisible(gizmo) || gizmo.target == nullptr || gizmo.size <= 0.0
        || !std::isfinite(gizmo.size)
        || (!hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)
            && !hasGizmoFlag(gizmo.flags, GizmoFlags::TranslateXY)
            && !hasGizmoFlag(gizmo.flags, GizmoFlags::Rotate))) {
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
    const auto considerHit = [&](const RaySegmentHit& hit,
                                 GizmoHandleKind handle,
                                 const Eigen::Vector3d& axis) {
      if (!nearest || hit.rayDistance < nearest->distance
          || (std::abs(hit.rayDistance - nearest->distance) < 1e-12
              && hit.separation < nearestSeparation)) {
        GizmoHandleHit gizmoHit;
        gizmoHit.gizmoIndex = gizmoIndex;
        gizmoHit.handle = handle;
        gizmoHit.distance = hit.rayDistance;
        gizmoHit.point = hit.segmentPoint;
        gizmoHit.axis = axis;
        nearest = gizmoHit;
        nearestSeparation = hit.separation;
      }
    };
    const auto considerPlaneHit = [&](const PlaneHandleHit& hit,
                                      GizmoHandleKind handle,
                                      const Eigen::Vector3d& normal) {
      if (!nearest || hit.rayDistance < nearest->distance) {
        GizmoHandleHit gizmoHit;
        gizmoHit.gizmoIndex = gizmoIndex;
        gizmoHit.handle = handle;
        gizmoHit.distance = hit.rayDistance;
        gizmoHit.point = hit.point;
        gizmoHit.axis = normal;
        nearest = gizmoHit;
        nearestSeparation = 0.0;
      }
    };

    const int axisCount
        = hasGizmoFlag(gizmo.flags, GizmoFlags::Translate) ? 3 : 2;
    if (hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)
        || hasGizmoFlag(gizmo.flags, GizmoFlags::TranslateXY)) {
      for (int axisIndex = 0; axisIndex < axisCount; ++axisIndex) {
        Eigen::Vector3d axis = rotation.col(axisIndex);
        const double axisNorm = axis.norm();
        if (!axis.allFinite() || axisNorm < 1e-12 || !std::isfinite(axisNorm)) {
          continue;
        }
        axis /= axisNorm;

        const auto hit
            = intersectRaySegment(ray, origin, origin + axis * length, radius);
        if (hit) {
          considerHit(*hit, handleKindForAxisIndex(axisIndex), axis);
        }
      }

      const double planeOffset = length * kPlaneHandleOffsetScale;
      const double planeSize = length * kPlaneHandleSizeScale;
      const auto considerPlane = [&](int normalAxisIndex,
                                     int firstAxisIndex,
                                     int secondAxisIndex) {
        Eigen::Vector3d normal = rotation.col(normalAxisIndex);
        const double normalNorm = normal.norm();
        if (!normal.allFinite() || !std::isfinite(normalNorm)
            || normalNorm < 1e-12) {
          return;
        }
        normal /= normalNorm;

        const auto hit = intersectRayPlaneHandle(
            ray,
            origin,
            rotation.col(firstAxisIndex),
            rotation.col(secondAxisIndex),
            planeOffset,
            planeSize);
        if (hit) {
          considerPlaneHit(
              *hit, planeHandleKindForNormalAxisIndex(normalAxisIndex), normal);
        }
      };

      considerPlane(2, 0, 1);
      if (hasGizmoFlag(gizmo.flags, GizmoFlags::Translate)) {
        considerPlane(0, 1, 2);
        considerPlane(1, 0, 2);
      }
    }

    if (hasGizmoFlag(gizmo.flags, GizmoFlags::Rotate)) {
      const double ringRadius = length * kRingRadiusScale;
      const std::array<std::pair<int, int>, 3> ringPlaneAxes{
          {{1, 2}, {2, 0}, {0, 1}}};
      for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
        Eigen::Vector3d axis = rotation.col(axisIndex);
        const double axisNorm = axis.norm();
        if (!axis.allFinite() || axisNorm < 1e-12 || !std::isfinite(axisNorm)) {
          continue;
        }
        axis /= axisNorm;

        Eigen::Vector3d first = rotation.col(ringPlaneAxes[axisIndex].first);
        Eigen::Vector3d second = rotation.col(ringPlaneAxes[axisIndex].second);
        const double firstNorm = first.norm();
        const double secondNorm = second.norm();
        if (!first.allFinite() || !second.allFinite()
            || !std::isfinite(firstNorm) || firstNorm < 1e-12
            || !std::isfinite(secondNorm) || secondNorm < 1e-12) {
          continue;
        }
        first /= firstNorm;
        second /= secondNorm;

        for (int segment = 0; segment < kRingSegments; ++segment) {
          const double angle0 = kTwoPi * static_cast<double>(segment)
                                / static_cast<double>(kRingSegments);
          const double angle1 = kTwoPi * static_cast<double>(segment + 1)
                                / static_cast<double>(kRingSegments);
          const Eigen::Vector3d from
              = origin
                + (first * std::cos(angle0) + second * std::sin(angle0))
                      * ringRadius;
          const Eigen::Vector3d to
              = origin
                + (first * std::cos(angle1) + second * std::sin(angle1))
                      * ringRadius;
          const auto hit = intersectRaySegment(ray, from, to, radius);
          if (hit) {
            considerHit(*hit, rotationHandleKindForAxisIndex(axisIndex), axis);
          }
        }
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

bool rotateGizmoTarget(
    Gizmo& gizmo, const Eigen::Vector3d& worldAxis, double angle)
{
  if (gizmo.target == nullptr || !worldAxis.allFinite()
      || !std::isfinite(angle)) {
    return false;
  }

  const double axisNorm = worldAxis.norm();
  if (!std::isfinite(axisNorm) || axisNorm <= 1e-12) {
    return false;
  }

  const auto simpleFrame
      = std::dynamic_pointer_cast<dart::dynamics::SimpleFrame>(gizmo.target);
  if (!simpleFrame) {
    return false;
  }

  Eigen::Isometry3d transform = simpleFrame->getWorldTransform();
  transform.linear()
      = Eigen::AngleAxisd(angle, worldAxis / axisNorm).toRotationMatrix()
        * transform.linear();
  simpleFrame->setTransform(transform, dart::dynamics::Frame::World());
  if (gizmo.onChanged) {
    gizmo.onChanged(transform);
  }
  return true;
}

} // namespace dart::gui
