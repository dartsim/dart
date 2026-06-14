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

#ifndef DART_GUI_GIZMO_HPP_
#define DART_GUI_GIZMO_HPP_

#include <dart/gui/debug.hpp>
#include <dart/gui/export.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::gui {

enum class GizmoFlags : unsigned int
{
  None = 0u,
  Translate = 1u << 0u,
  Rotate = 1u << 1u,
  Scale = 1u << 2u,
  TranslateXY = 1u << 3u,
  All = (1u << 0u) | (1u << 1u),
};

constexpr GizmoFlags operator|(GizmoFlags lhs, GizmoFlags rhs)
{
  return static_cast<GizmoFlags>(
      std::to_underlying(lhs) | std::to_underlying(rhs));
}

constexpr GizmoFlags operator&(GizmoFlags lhs, GizmoFlags rhs)
{
  return static_cast<GizmoFlags>(
      std::to_underlying(lhs) & std::to_underlying(rhs));
}

constexpr bool hasGizmoFlag(GizmoFlags flags, GizmoFlags flag)
{
  return (flags & flag) != GizmoFlags::None;
}

struct GizmoAxisColors
{
  Eigen::Vector4d x = Eigen::Vector4d(0.85, 0.15, 0.15, 1.0);
  Eigen::Vector4d y = Eigen::Vector4d(0.15, 0.85, 0.15, 1.0);
  Eigen::Vector4d z = Eigen::Vector4d(0.15, 0.30, 0.85, 1.0);
  Eigen::Vector4d highlight = Eigen::Vector4d(1.00, 0.80, 0.10, 1.0);
};

struct DART_GUI_API Gizmo
{
  std::shared_ptr<dart::dynamics::Frame> target;
  std::string label;
  GizmoFlags flags = GizmoFlags::All;
  double size = 0.15;
  GizmoAxisColors colors;
  std::function<bool()> isVisible;
  std::function<void(const Eigen::Isometry3d& newWorldTransform)> onChanged;
};

enum class GizmoHandleKind
{
  None,
  TranslateX,
  TranslateY,
  TranslateZ,
  TranslateXY,
  TranslateYZ,
  TranslateXZ,
  RotateX,
  RotateY,
  RotateZ,
};

struct GizmoHandleHit
{
  std::size_t gizmoIndex = 0u;
  GizmoHandleKind handle = GizmoHandleKind::None;
  double distance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
};

DART_GUI_API std::vector<DebugLineDescriptor> makeGizmoDebugLines(
    const Gizmo& gizmo,
    double scale = 1.0,
    GizmoHandleKind highlightedHandle = GizmoHandleKind::None);

DART_GUI_API std::vector<DebugLineDescriptor> makeGizmoDebugLines(
    const std::vector<Gizmo>& gizmos,
    double scale = 1.0,
    std::optional<GizmoHandleHit> highlightedHandle = std::nullopt);

DART_GUI_API std::vector<DebugTriangleDescriptor> makeGizmoDebugTriangles(
    const Gizmo& gizmo,
    double scale = 1.0,
    GizmoHandleKind highlightedHandle = GizmoHandleKind::None);

DART_GUI_API std::vector<DebugTriangleDescriptor> makeGizmoDebugTriangles(
    const std::vector<Gizmo>& gizmos,
    double scale = 1.0,
    std::optional<GizmoHandleHit> highlightedHandle = std::nullopt);

DART_GUI_API std::optional<GizmoHandleHit> pickNearestGizmoHandle(
    const std::vector<Gizmo>& gizmos,
    const PickRay& ray,
    double scale = 1.0,
    double handleRadius = 0.025);

DART_GUI_API bool translateGizmoTarget(
    Gizmo& gizmo, const Eigen::Vector3d& worldTranslation);

DART_GUI_API bool rotateGizmoTarget(
    Gizmo& gizmo, const Eigen::Vector3d& worldAxis, double angle);

} // namespace dart::gui

#endif // DART_GUI_GIZMO_HPP_
