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

#ifndef DART_GUI_DEBUG_HPP_
#define DART_GUI_DEBUG_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/renderable.hpp>

#include <dart/collision/fwd.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::gui {

struct DebugLineDescriptor
{
  Eigen::Vector3d from = Eigen::Vector3d::Zero();
  Eigen::Vector3d to = Eigen::Vector3d::Zero();
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  double thickness = 0.0;
  std::string label;
};

struct DebugTriangleDescriptor
{
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  std::string label;
};

struct DebugLabelDescriptor
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  std::string text;
};

struct DebugDrawOptions
{
  bool drawGrid = true;
  bool drawWorldFrame = true;
  bool drawBodyFrames = false;
  bool drawCentersOfMass = false;
  bool drawInertiaBoxes = false;
  bool drawCollisionShapeBounds = false;
  bool drawSupportPolygons = false;
  bool drawSupportCentroids = true;
  bool drawContacts = true;
  bool drawContactNormals = true;
  bool drawContactForces = true;
  double gridHalfExtent = 4.0;
  double gridSpacing = 0.5;
  double gridZ = 0.08;
  double worldFrameAxisLength = 0.9;
  double bodyFrameAxisLength = 0.22;
  double centerOfMassMarkerRadius = 0.08;
  double inertiaBoxScale = 1.0;
  double collisionBoundsPadding = 0.0;
  double supportPolygonElevation = 0.02;
  double supportCentroidMarkerRadius = 0.06;
  double contactMarkerHalfExtent = 0.035;
  double contactNormalLength = 0.22;
  double contactForceScale = 0.002;
  double contactForceMinLength = 0.04;
  double contactForceMaxLength = 0.45;
};

/// Applies the standard debug visual styling to a world-backed shape frame.
///
/// Persistent debug visuals should not cast or receive shadows because they
/// describe scene state rather than physical geometry.
DART_GUI_API void applyDebugVisualStyle(
    dynamics::ShapeFrame& shapeFrame,
    const Eigen::Vector4d& rgba = Eigen::Vector4d::Ones());

DART_GUI_API std::vector<DebugLineDescriptor> makeGridDebugLines(
    const DebugDrawOptions& options = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeFrameDebugLines(
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeSelectionDebugLines(
    const RenderableDescriptor& renderable,
    const Eigen::Vector4d& rgba = Eigen::Vector4d(1.0, 0.84, 0.18, 1.0),
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeInertiaDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeCollisionShapeDebugLines(
    const dynamics::ShapeNode& shapeNode,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> makeSupportPolygonDebugLines(
    const dynamics::Skeleton& skeleton,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> extractContactDebugLines(
    const collision::CollisionResult& result,
    const DebugDrawOptions& options = {});

DART_GUI_API std::vector<DebugLineDescriptor> extractDebugLines(
    const DebugDrawOptions& options = {});

DART_GUI_API std::vector<DebugLineDescriptor> extractDebugLines(
    const simulation::World& world, const DebugDrawOptions& options = {});

} // namespace dart::gui

#endif // DART_GUI_DEBUG_HPP_
