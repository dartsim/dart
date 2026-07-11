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

#include <dart/gui/debug.hpp>

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>
#include <dart/collision/contact.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/geometry.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::gui {
namespace {

constexpr double kMinDebugLineLengthSquared = 1e-18;

Eigen::Vector4d rgba(double red, double green, double blue, double alpha = 1.0)
{
  return {red, green, blue, alpha};
}

void appendLine(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    std::string label = {})
{
  const Eigen::Vector3d segment = to - from;
  const double lengthSquared = segment.squaredNorm();
  if (!from.allFinite() || !to.allFinite() || !std::isfinite(lengthSquared)
      || lengthSquared <= kMinDebugLineLengthSquared) {
    return;
  }

  DebugLineDescriptor line;
  line.from = from;
  line.to = to;
  line.rgba = color;
  line.label = std::move(label);
  lines.push_back(std::move(line));
}

void appendArrowLines(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  appendLine(lines, from, to, color, label);

  const Eigen::Vector3d vector = to - from;
  const double length = vector.norm();
  if (!std::isfinite(length) || length <= 1e-12) {
    return;
  }

  const Eigen::Vector3d direction = vector / length;
  const Eigen::Vector3d seed = std::abs(direction.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d side = direction.cross(seed).normalized();
  const double headLength = length * 0.25;
  const double headWidth = headLength * 0.45;
  const Eigen::Vector3d base = to - direction * headLength;
  appendLine(lines, to, base + side * headWidth, color, label);
  appendLine(lines, to, base - side * headWidth, color, label);
}

void appendFrameAxes(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix)
{
  if (axisLength <= 0.0 || !std::isfinite(axisLength)) {
    return;
  }

  const Eigen::Vector3d origin = transform.translation();
  const Eigen::Matrix3d rotation = transform.linear();
  appendLine(
      lines,
      origin,
      origin + rotation.col(0).normalized() * axisLength,
      rgba(0.9, 0.28, 0.28),
      labelPrefix.empty() ? "x" : labelPrefix + ".x");
  appendLine(
      lines,
      origin,
      origin + rotation.col(1).normalized() * axisLength,
      rgba(0.31, 0.75, 0.43),
      labelPrefix.empty() ? "y" : labelPrefix + ".y");
  appendLine(
      lines,
      origin,
      origin + rotation.col(2).normalized() * axisLength,
      rgba(0.28, 0.47, 0.92),
      labelPrefix.empty() ? "z" : labelPrefix + ".z");
}

void appendAxisMarker(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const Eigen::Vector4d& color,
    const std::string& labelPrefix,
    const std::string& markerName)
{
  if (radius <= 0.0 || !std::isfinite(radius) || !center.allFinite()) {
    return;
  }

  const std::string markerPrefix
      = labelPrefix.empty() ? markerName : labelPrefix + "." + markerName;
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitX() * radius,
      center + Eigen::Vector3d::UnitX() * radius,
      color,
      markerPrefix + ".x");
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitY() * radius,
      center + Eigen::Vector3d::UnitY() * radius,
      color,
      markerPrefix + ".y");
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitZ() * radius,
      center + Eigen::Vector3d::UnitZ() * radius,
      color,
      markerPrefix + ".z");
}

void appendCenterOfMassMarker(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const std::string& labelPrefix)
{
  appendAxisMarker(
      lines, center, radius, rgba(0.22, 0.82, 0.86), labelPrefix, "com");
}

void appendBoxEdges(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& axes,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  if (!center.allFinite() || !axes.allFinite() || !halfExtents.allFinite()
      || (halfExtents.array() <= 0.0).any()) {
    return;
  }

  std::array<Eigen::Vector3d, 8> corners;
  std::size_t index = 0u;
  for (double x : {-1.0, 1.0}) {
    for (double y : {-1.0, 1.0}) {
      for (double z : {-1.0, 1.0}) {
        corners[index++] = center + axes.col(0) * halfExtents.x() * x
                           + axes.col(1) * halfExtents.y() * y
                           + axes.col(2) * halfExtents.z() * z;
      }
    }
  }

  const std::array<std::pair<std::size_t, std::size_t>, 12> edges
      = {std::pair<std::size_t, std::size_t>{0, 4},
         {1, 5},
         {2, 6},
         {3, 7},
         {0, 2},
         {1, 3},
         {4, 6},
         {5, 7},
         {0, 1},
         {2, 3},
         {4, 5},
         {6, 7}};
  for (const auto& [from, to] : edges) {
    appendLine(lines, corners[from], corners[to], color, label);
  }
}

void appendTransformedBoundsEdges(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& boundsMin,
    const Eigen::Vector3d& boundsMax,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector4d& color,
    const std::string& label)
{
  if (!boundsMin.allFinite() || !boundsMax.allFinite()
      || !transform.matrix().allFinite()) {
    return;
  }

  const Eigen::Vector3d min = boundsMin.cwiseMin(boundsMax);
  const Eigen::Vector3d max = boundsMin.cwiseMax(boundsMax);
  if ((max - min).cwiseAbs().maxCoeff() < 1e-12) {
    return;
  }

  std::array<Eigen::Vector3d, 8> corners
      = {Eigen::Vector3d(min.x(), min.y(), min.z()),
         Eigen::Vector3d(max.x(), min.y(), min.z()),
         Eigen::Vector3d(max.x(), max.y(), min.z()),
         Eigen::Vector3d(min.x(), max.y(), min.z()),
         Eigen::Vector3d(min.x(), min.y(), max.z()),
         Eigen::Vector3d(max.x(), min.y(), max.z()),
         Eigen::Vector3d(max.x(), max.y(), max.z()),
         Eigen::Vector3d(min.x(), max.y(), max.z())};
  for (Eigen::Vector3d& corner : corners) {
    corner = transform * corner;
  }

  const std::array<std::pair<std::size_t, std::size_t>, 12> edges
      = {std::pair<std::size_t, std::size_t>{0, 1},
         {1, 2},
         {2, 3},
         {3, 0},
         {4, 5},
         {5, 6},
         {6, 7},
         {7, 4},
         {0, 4},
         {1, 5},
         {2, 6},
         {3, 7}};

  for (const auto& [from, to] : edges) {
    appendLine(lines, corners[from], corners[to], color, label);
  }
}

} // namespace

void applyDebugVisualStyle(
    dynamics::ShapeFrame& shapeFrame, const Eigen::Vector4d& rgba)
{
  auto* visualAspect = shapeFrame.getVisualAspect(true);
  visualAspect->setRGBA(rgba);
  visualAspect->setShadowed(false);
}

std::vector<DebugLineDescriptor> makeGridDebugLines(
    const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawGrid || options.gridHalfExtent <= 0.0
      || options.gridSpacing <= 0.0 || !std::isfinite(options.gridHalfExtent)
      || !std::isfinite(options.gridSpacing)) {
    return lines;
  }

  const auto steps = static_cast<int>(
      std::ceil(options.gridHalfExtent / options.gridSpacing));
  lines.reserve(static_cast<std::size_t>(steps * 4 + 4));
  const Eigen::Vector4d gridColor = rgba(0.46, 0.49, 0.5, 0.59);
  for (int i = -steps; i <= steps; ++i) {
    const double coordinate = static_cast<double>(i) * options.gridSpacing;
    if (std::abs(coordinate) > options.gridHalfExtent + 1e-12) {
      continue;
    }

    appendLine(
        lines,
        {-options.gridHalfExtent, coordinate, options.gridZ},
        {options.gridHalfExtent, coordinate, options.gridZ},
        gridColor,
        "grid");
    appendLine(
        lines,
        {coordinate, -options.gridHalfExtent, options.gridZ},
        {coordinate, options.gridHalfExtent, options.gridZ},
        gridColor,
        "grid");
  }

  return lines;
}

std::vector<DebugLineDescriptor> makeFrameDebugLines(
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  lines.reserve(3);
  appendFrameAxes(lines, transform, axisLength, labelPrefix);
  return lines;
}

std::vector<DebugLineDescriptor> makeSelectionDebugLines(
    const RenderableDescriptor& renderable,
    const Eigen::Vector4d& rgba,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!renderable.material.visible || !renderable.geometry.hasLocalBounds) {
    return lines;
  }

  const std::string label
      = labelPrefix.empty() ? "selection.bounds" : labelPrefix + ".bounds";
  lines.reserve(12u);
  appendTransformedBoundsEdges(
      lines,
      renderable.geometry.localBoundsMin,
      renderable.geometry.localBoundsMax,
      renderable.worldTransform,
      rgba,
      label);

  return lines;
}

std::vector<DebugLineDescriptor> makeInertiaDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawInertiaBoxes || options.inertiaBoxScale <= 0.0
      || !std::isfinite(options.inertiaBoxScale)) {
    return lines;
  }

  const double mass = bodyNode.getMass();
  if (mass <= 0.0 || !std::isfinite(mass)) {
    return lines;
  }

  const Eigen::Matrix3d moment = bodyNode.getInertia().getMoment();
  if (!moment.allFinite()) {
    return lines;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(moment);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()
      || !solver.eigenvectors().allFinite()) {
    return lines;
  }

  const Eigen::Vector3d principalMoments = solver.eigenvalues();
  const Eigen::Vector3d dimensionsSquared
      = (Eigen::Vector3d(
             principalMoments.y() + principalMoments.z() - principalMoments.x(),
             principalMoments.x() + principalMoments.z() - principalMoments.y(),
             principalMoments.x() + principalMoments.y() - principalMoments.z())
         * (6.0 / mass));
  if (!dimensionsSquared.allFinite()
      || (dimensionsSquared.array() <= 1e-18).any()) {
    return lines;
  }

  const Eigen::Vector3d halfExtents
      = dimensionsSquared.cwiseSqrt() * 0.5 * options.inertiaBoxScale;
  const Eigen::Matrix3d worldAxes
      = bodyNode.getWorldTransform().linear() * solver.eigenvectors();
  const std::string label
      = labelPrefix.empty() ? "inertia" : labelPrefix + ".inertia";
  lines.reserve(12u);
  appendBoxEdges(
      lines,
      bodyNode.getCOM(),
      worldAxes,
      halfExtents,
      rgba(0.58, 0.44, 0.95, 0.82),
      label);
  return lines;
}

std::vector<DebugLineDescriptor> makeCollisionShapeDebugLines(
    const dynamics::ShapeNode& shapeNode,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawCollisionShapeBounds
      || !std::isfinite(options.collisionBoundsPadding)
      || options.collisionBoundsPadding < 0.0) {
    return lines;
  }

  if (shapeNode.getCollisionAspect() == nullptr) {
    return lines;
  }

  const auto shape = shapeNode.getShape();
  if (!shape) {
    return lines;
  }

  auto geometry = describeShape(*shape);
  if (!geometry || !geometry->hasLocalBounds) {
    return lines;
  }

  const Eigen::Vector3d padding
      = Eigen::Vector3d::Constant(options.collisionBoundsPadding);
  const std::string label = labelPrefix.empty()
                                ? "collision.bounds"
                                : labelPrefix + ".collision_bounds";
  lines.reserve(12u);
  appendTransformedBoundsEdges(
      lines,
      geometry->localBoundsMin - padding,
      geometry->localBoundsMax + padding,
      shapeNode.getWorldTransform(),
      rgba(0.2, 0.86, 0.43, 0.72),
      label);
  return lines;
}

std::vector<DebugLineDescriptor> makeSupportPolygonDebugLines(
    const dynamics::Skeleton& skeleton,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawSupportPolygons
      || !std::isfinite(options.supportPolygonElevation)) {
    return lines;
  }

  const math::SupportPolygon& polygon = skeleton.getSupportPolygon();
  if (polygon.empty()) {
    return lines;
  }

  const auto& axes = skeleton.getSupportAxes();
  if (!axes.first.allFinite() || !axes.second.allFinite()) {
    return lines;
  }

  const Eigen::Vector3d up = axes.first.cross(axes.second);
  if (!up.allFinite() || up.squaredNorm() <= 1e-18) {
    return lines;
  }

  const Eigen::Vector3d elevation
      = up.normalized() * options.supportPolygonElevation;
  const auto toWorldPoint = [&](const Eigen::Vector2d& point) {
    return axes.first * point.x() + axes.second * point.y() + elevation;
  };

  const std::size_t edgeCount
      = polygon.size() == 2u ? 1u : (polygon.size() > 2u ? polygon.size() : 0u);
  lines.reserve(edgeCount + (options.drawSupportCentroids ? 3u : 0u));

  const Eigen::Vector4d polygonColor = rgba(0.97, 0.78, 0.24, 0.86);
  const std::string polygonLabel = labelPrefix.empty()
                                       ? "support_polygon"
                                       : labelPrefix + ".support_polygon";
  if (polygon.size() >= 2u) {
    for (std::size_t i = 0; i + 1u < polygon.size(); ++i) {
      appendLine(
          lines,
          toWorldPoint(polygon[i]),
          toWorldPoint(polygon[i + 1u]),
          polygonColor,
          polygonLabel);
    }

    if (polygon.size() > 2u) {
      appendLine(
          lines,
          toWorldPoint(polygon.back()),
          toWorldPoint(polygon.front()),
          polygonColor,
          polygonLabel);
    }
  }

  if (options.drawSupportCentroids) {
    const Eigen::Vector2d& centroid = skeleton.getSupportCentroid();
    if (centroid.allFinite()) {
      appendAxisMarker(
          lines,
          toWorldPoint(centroid),
          options.supportCentroidMarkerRadius,
          rgba(0.16, 0.78, 0.58, 0.9),
          labelPrefix,
          "support_centroid");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> makeJointAxisDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawJointAxes || options.jointAxisLength <= 0.0
      || !std::isfinite(options.jointAxisLength)) {
    return lines;
  }

  const dynamics::Joint* joint = bodyNode.getParentJoint();
  if (joint == nullptr) {
    return lines;
  }

  // The single-DOF axis is expressed in the joint frame; map it to world via
  // the joint frame's world transform (child body world * child->joint).
  const Eigen::Isometry3d jointWorld
      = bodyNode.getWorldTransform() * joint->getTransformFromChildBodyNode();
  if (!jointWorld.matrix().allFinite()) {
    return lines;
  }
  const Eigen::Vector3d anchor = jointWorld.translation();

  const std::string label
      = labelPrefix.empty() ? "joint.axis" : labelPrefix + ".joint_axis";
  const Eigen::Vector4d axisColor = rgba(0.96, 0.62, 0.12, 0.95);

  if (joint->getType() == dynamics::RevoluteJoint::getStaticType()) {
    const auto* revolute = static_cast<const dynamics::RevoluteJoint*>(joint);
    const Eigen::Vector3d worldAxis
        = (jointWorld.linear() * revolute->getAxis()).normalized();
    if (!worldAxis.allFinite()) {
      return lines;
    }
    appendLine(
        lines,
        anchor - worldAxis * options.jointAxisLength,
        anchor + worldAxis * options.jointAxisLength,
        axisColor,
        label);
  } else if (joint->getType() == dynamics::PrismaticJoint::getStaticType()) {
    const auto* prismatic = static_cast<const dynamics::PrismaticJoint*>(joint);
    const Eigen::Vector3d worldAxis
        = (jointWorld.linear() * prismatic->getAxis()).normalized();
    if (!worldAxis.allFinite()) {
      return lines;
    }
    appendArrowLines(
        lines,
        anchor,
        anchor + worldAxis * options.jointAxisLength,
        axisColor,
        label);
  }

  return lines;
}

std::vector<DebugLineDescriptor> makeVelocityDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;

  const auto scaledLength = [&](double magnitude, double scale) {
    return std::clamp(
        magnitude * scale,
        options.velocityMinLength,
        options.velocityMaxLength);
  };

  if (options.drawLinearVelocities && options.linearVelocityScale > 0.0) {
    const Eigen::Vector3d velocity = bodyNode.getCOMLinearVelocity();
    const double magnitude = velocity.norm();
    if (velocity.allFinite() && magnitude > 1e-9) {
      const Eigen::Vector3d origin = bodyNode.getCOM();
      appendArrowLines(
          lines,
          origin,
          origin
              + velocity.normalized()
                    * scaledLength(magnitude, options.linearVelocityScale),
          rgba(0.32, 0.74, 0.98, 0.95),
          labelPrefix.empty() ? "vel.linear" : labelPrefix + ".vel_linear");
    }
  }

  if (options.drawAngularVelocities && options.angularVelocityScale > 0.0) {
    const Eigen::Vector3d velocity = bodyNode.getAngularVelocity();
    const double magnitude = velocity.norm();
    if (velocity.allFinite() && magnitude > 1e-9) {
      const Eigen::Vector3d origin = bodyNode.getWorldTransform().translation();
      appendArrowLines(
          lines,
          origin,
          origin
              + velocity.normalized()
                    * scaledLength(magnitude, options.angularVelocityScale),
          rgba(0.74, 0.52, 0.98, 0.95),
          labelPrefix.empty() ? "vel.angular" : labelPrefix + ".vel_angular");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractContactDebugLines(
    const collision::CollisionResult& result, const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawContacts) {
    return lines;
  }

  const auto contacts = result.getContacts();
  lines.reserve(contacts.size() * 8u);
  const Eigen::Vector4d pointColor = rgba(1.0, 0.92, 0.38);
  const Eigen::Vector4d normalColor = rgba(1.0, 0.75, 0.25);
  const Eigen::Vector4d forceColor = rgba(0.93, 0.31, 0.67);
  for (const auto& contact : contacts) {
    const Eigen::Vector3d point = contact.point;
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");

    if (options.drawContactNormals
        && contact.normal.squaredNorm()
               > collision::Contact::getNormalEpsilonSquared()) {
      appendArrowLines(
          lines,
          point,
          point + contact.normal.normalized() * options.contactNormalLength,
          normalColor,
          "contact.normal");
    }

    const double forceNorm = contact.force.norm();
    if (options.drawContactForces && forceNorm > 1e-9) {
      const double forceLength = std::clamp(
          forceNorm * options.contactForceScale,
          options.contactForceMinLength,
          options.contactForceMaxLength);
      appendArrowLines(
          lines,
          point,
          point + contact.force.normalized() * forceLength,
          forceColor,
          "contact.force");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractDebugLines(
    const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines = makeGridDebugLines(options);

  if (options.drawWorldFrame) {
    auto worldFrameLines = makeFrameDebugLines(
        Eigen::Isometry3d::Identity(), options.worldFrameAxisLength, "world");
    lines.insert(lines.end(), worldFrameLines.begin(), worldFrameLines.end());
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractContactDebugLines(
    const std::vector<simulation::Contact>& contacts,
    const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawContacts) {
    return lines;
  }

  lines.reserve(contacts.size() * 8u);
  const Eigen::Vector4d pointColor = rgba(1.0, 0.92, 0.38);
  const Eigen::Vector4d normalColor = rgba(1.0, 0.75, 0.25);
  for (const auto& contact : contacts) {
    const Eigen::Vector3d point = contact.point;
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");

    if (options.drawContactNormals && contact.normal.squaredNorm() > 1e-12) {
      appendArrowLines(
          lines,
          point,
          point + contact.normal.normalized() * options.contactNormalLength,
          normalColor,
          "contact.normal");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> makePolylineDebugLines(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector4d& rgba,
    const std::string& label)
{
  std::vector<DebugLineDescriptor> lines;
  if (points.size() < 2u) {
    return lines;
  }
  lines.reserve(points.size() - 1u);
  for (std::size_t i = 0; i + 1u < points.size(); ++i) {
    appendLine(lines, points[i], points[i + 1u], rgba, label);
  }
  return lines;
}

namespace {

std::optional<Eigen::Vector3d> collisionShapeLocalHalfExtents(
    const simulation::CollisionShape& shape)
{
  using Type = simulation::CollisionShapeType;
  switch (shape.type) {
    case Type::Sphere:
      return Eigen::Vector3d::Constant(shape.radius);
    case Type::Box:
      return shape.halfExtents;
    case Type::Capsule:
      return Eigen::Vector3d(
          shape.radius, shape.radius, shape.halfExtents.z() + shape.radius);
    case Type::Cylinder:
      return Eigen::Vector3d(shape.radius, shape.radius, shape.halfExtents.z());
    case Type::Mesh: {
      if (shape.vertices.empty()) {
        return std::nullopt;
      }
      Eigen::Vector3d boundsMin = shape.vertices.front();
      Eigen::Vector3d boundsMax = shape.vertices.front();
      for (const auto& vertex : shape.vertices) {
        boundsMin = boundsMin.cwiseMin(vertex);
        boundsMax = boundsMax.cwiseMax(vertex);
      }
      // Mesh bounds may be off-center; return the enclosing centered box so
      // the shared box-edge helper applies (conservative for offset meshes).
      return 0.5 * (boundsMax - boundsMin)
             + (0.5 * (boundsMax + boundsMin)).cwiseAbs();
    }
    case Type::Plane:
      return std::nullopt; // unbounded
  }
  return std::nullopt;
}

void appendRigidBodyDebugLines(
    std::vector<DebugLineDescriptor>& lines,
    simulation::RigidBody& body,
    const std::string& name,
    const DebugDrawOptions& options)
{
  const Eigen::Isometry3d transform = body.getTransform();

  if (options.drawBodyFrames) {
    appendFrameAxes(lines, transform, options.bodyFrameAxisLength, name);
  }

  if (options.drawCentersOfMass) {
    // RigidBody's center of mass is at its frame origin by convention.
    appendCenterOfMassMarker(
        lines, transform.translation(), options.centerOfMassMarkerRadius, name);
  }

  if (options.drawInertiaBoxes && body.getMass() > 0.0) {
    const Eigen::Matrix3d inertia = body.getInertia();
    if (inertia.allFinite()) {
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
          0.5 * (inertia + inertia.transpose()));
      const Eigen::Vector3d moments = solver.eigenvalues();
      if ((moments.array() > 0.0).all()) {
        // Solid-box equivalence: I_x = m/12 (b^2 + c^2), etc.
        const Eigen::Vector3d sums
            = 6.0 / body.getMass()
              * (Eigen::Vector3d::Constant(moments.sum()) - 2.0 * moments);
        if ((sums.array() > 0.0).all()) {
          appendBoxEdges(
              lines,
              transform.translation(),
              transform.linear() * solver.eigenvectors(),
              sums.cwiseSqrt() * 0.5 * options.inertiaBoxScale,
              rgba(0.58, 0.44, 0.95, 0.82),
              name + ".inertia");
        }
      }
    }
  }

  if (options.drawCollisionShapeBounds) {
    for (const auto& shape : body.getCollisionShapes()) {
      const auto halfExtents = collisionShapeLocalHalfExtents(shape);
      if (!halfExtents.has_value()) {
        continue;
      }
      const Eigen::Isometry3d shapeTransform = transform * shape.localTransform;
      // Clamp so flat shapes (a zero-thickness box axis) still draw as a
      // degenerate rectangle instead of being rejected by the box helper.
      const Eigen::Vector3d paddedHalfExtents
          = (*halfExtents
             + Eigen::Vector3d::Constant(options.collisionBoundsPadding))
                .cwiseMax(1e-9);
      appendBoxEdges(
          lines,
          shapeTransform.translation(),
          shapeTransform.linear(),
          paddedHalfExtents,
          rgba(0.2, 0.86, 0.43, 0.72),
          name + ".bounds");
    }
  }

  const auto scaledLength = [&](double magnitude, double scale) {
    return std::clamp(
        magnitude * scale,
        options.velocityMinLength,
        options.velocityMaxLength);
  };

  if (options.drawLinearVelocities && options.linearVelocityScale > 0.0) {
    const Eigen::Vector3d velocity = body.getLinearVelocity();
    const double magnitude = velocity.norm();
    if (velocity.allFinite() && magnitude > 1e-9) {
      appendArrowLines(
          lines,
          transform.translation(),
          transform.translation()
              + velocity.normalized()
                    * scaledLength(magnitude, options.linearVelocityScale),
          rgba(0.32, 0.74, 0.98, 0.95),
          name + ".vel_linear");
    }
  }

  if (options.drawAngularVelocities && options.angularVelocityScale > 0.0) {
    const Eigen::Vector3d velocity = body.getAngularVelocity();
    const double magnitude = velocity.norm();
    if (velocity.allFinite() && magnitude > 1e-9) {
      appendArrowLines(
          lines,
          transform.translation(),
          transform.translation()
              + velocity.normalized()
                    * scaledLength(magnitude, options.angularVelocityScale),
          rgba(0.74, 0.52, 0.98, 0.95),
          name + ".vel_angular");
    }
  }
}

} // namespace

std::vector<DebugLineDescriptor> extractDebugLines(
    simulation::World& world, const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines = extractDebugLines(options);
  for (const std::string& name : world.getRigidBodyNames()) {
    auto body = world.getRigidBody(name);
    if (body.has_value()) {
      appendRigidBodyDebugLines(lines, *body, name, options);
    }
  }
  return lines;
}

} // namespace dart::gui
