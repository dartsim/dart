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

#ifndef DART_GUI_INTERACTION_HPP_
#define DART_GUI_INTERACTION_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>

#include <limits>
#include <optional>
#include <vector>

namespace dart::gui {

struct PickHit
{
  RenderableId id = 0;
  std::size_t renderableIndex = 0;
  double distance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

DART_GUI_API std::optional<double> intersectRenderable(
    const RenderableDescriptor& renderable, const PickRay& ray);

DART_GUI_API std::optional<PickHit> pickNearestRenderable(
    const std::vector<RenderableDescriptor>& renderables,
    const PickRay& ray,
    double maxDistance = std::numeric_limits<double>::infinity());

DART_GUI_API std::optional<Eigen::Vector3d> intersectPlane(
    const PickRay& ray,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal);

DART_GUI_API std::optional<Eigen::Vector3d> computePlaneDragTranslation(
    const PickRay& previousRay,
    const PickRay& currentRay,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal);

DART_GUI_API std::optional<Eigen::Vector3d> computeAxisDragTranslation(
    const PickRay& previousRay,
    const PickRay& currentRay,
    const Eigen::Vector3d& axisPoint,
    const Eigen::Vector3d& axisDirection);

DART_GUI_API bool translateFreeJointRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation);

DART_GUI_API bool translateSimpleFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation);

DART_GUI_API bool translateFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation);

DART_GUI_API bool rotateSimpleFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldAxis,
    double angle);

DART_GUI_API bool rotateFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldAxis,
    double angle);

} // namespace dart::gui

#endif // DART_GUI_INTERACTION_HPP_
