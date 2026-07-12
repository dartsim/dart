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
struct Contact;
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

/// A renderer-neutral bundle of application-supplied debug geometry.
///
/// Returned by an application's per-frame debug provider so any host can feed
/// custom debug lines, triangles, and labels into the built-in debug overlay
/// with the correct unlit, no-shadow, always-on-top treatment, instead of each
/// example re-implementing its own overlay wiring.
struct DebugScene
{
  std::vector<DebugLineDescriptor> lines;
  std::vector<DebugTriangleDescriptor> triangles;
  std::vector<DebugLabelDescriptor> labels;
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
  bool drawJointAxes = false;
  bool drawLinearVelocities = false;
  bool drawAngularVelocities = false;
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
  double jointAxisLength = 0.3;
  double linearVelocityScale = 0.15;
  double angularVelocityScale = 0.15;
  double velocityMinLength = 0.03;
  double velocityMaxLength = 1.2;
  /// World-space half-thickness applied to the per-body, velocity, and contact
  /// debug lines produced by the world-aware extraction so headless overlays
  /// render as legible ribbons/tubes instead of single-pixel hairlines. The
  /// grid is intentionally left thin. Set to 0 to keep every line a hairline.
  double lineThickness = 0.006;
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

/// Draws the motion axis of a body's parent joint (a double-ended line for
/// revolute joints, an arrow for prismatic joints) anchored at the joint
/// origin.
DART_GUI_API std::vector<DebugLineDescriptor> makeJointAxisDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

/// Draws linear (at the center of mass) and/or angular (about the body origin)
/// velocity vectors as arrows, scaled and clamped by the options.
DART_GUI_API std::vector<DebugLineDescriptor> makeVelocityDebugLines(
    const dynamics::BodyNode& bodyNode,
    const DebugDrawOptions& options = {},
    const std::string& labelPrefix = {});

DART_GUI_API std::vector<DebugLineDescriptor> extractContactDebugLines(
    const collision::CollisionResult& result,
    const DebugDrawOptions& options = {});

/// Contact markers/normals for the promoted `simulation::World` contact type
/// (as returned by `World::collide()`). Contact forces are not part of the
/// simulation contact payload, so only points and normals are drawn.
DART_GUI_API std::vector<DebugLineDescriptor> extractContactDebugLines(
    const std::vector<simulation::Contact>& contacts,
    const DebugDrawOptions& options = {});

/// A polyline (e.g. a recorded body trajectory) as consecutive debug-line
/// segments; zero-length segments are dropped.
DART_GUI_API std::vector<DebugLineDescriptor> makePolylineDebugLines(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector4d& rgba,
    const std::string& label = {},
    double thickness = 0.0);

DART_GUI_API std::vector<DebugLineDescriptor> extractDebugLines(
    const DebugDrawOptions& options = {});

/// World-aware debug extraction for the promoted `simulation::World`: the
/// static layers (grid, world frame) plus per-rigid-body layers selected by
/// the options — body frames, center-of-mass markers, inertia boxes
/// (solid-box equivalents of the body inertia), collision-shape bounds, and
/// linear/angular velocity arrows. Contacts come from the explicit
/// `extractContactDebugLines(std::vector<simulation::Contact>, ...)` overload
/// so captures stay deterministic relative to a chosen step. Takes the world
/// non-const because rigid-body handles resolve through it.
///
/// Line-label conventions: frame axes are `<body>.x/.y/.z`, center-of-mass
/// markers `<body>.com.x/.y/.z` (axis-marker convention), inertia boxes
/// `<body>.inertia`, collision bounds `<body>.bounds` (one label for all of
/// a body's shapes), velocities `<body>.vel_linear` / `<body>.vel_angular`.
DART_GUI_API std::vector<DebugLineDescriptor> extractDebugLines(
    simulation::World& world, const DebugDrawOptions& options = {});

} // namespace dart::gui

#endif // DART_GUI_DEBUG_HPP_
