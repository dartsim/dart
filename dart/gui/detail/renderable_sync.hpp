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

#pragma once

#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/renderable.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <optional>
#include <vector>

namespace filament {
class Engine;
class Scene;
} // namespace filament

namespace dart::gui {
struct OrbitCamera;
struct RenderSettings;
} // namespace dart::gui

namespace dart::gui::detail {

using RenderableFactory = std::function<std::optional<Renderable>(
    const dart::gui::RenderableDescriptor&)>;

void setRenderableTransform(
    ::filament::Engine& engine,
    const Renderable& renderable,
    const Eigen::Isometry3d& transform);

void addRenderableToScene(
    ::filament::Scene& scene, const Renderable& renderable);

void removeRenderableFromScene(
    ::filament::Scene& scene, const Renderable& renderable);

void updateSceneRenderableFromDescriptor(
    ::filament::Engine& engine,
    SceneRenderable& sceneRenderable,
    const dart::gui::RenderableDescriptor& descriptor,
    const dart::gui::RenderSettings& renderSettings,
    const dart::gui::OrbitCamera& camera,
    int viewportWidth,
    int viewportHeight,
    bool selected);

bool updateSceneRenderablesFromDescriptors(
    ::filament::Engine& engine,
    const std::vector<dart::gui::RenderableDescriptor>& descriptors,
    std::vector<SceneRenderable>& sceneRenderables,
    dart::gui::RenderableId selectedRenderableId,
    const dart::gui::RenderSettings& renderSettings,
    const dart::gui::OrbitCamera& camera,
    int viewportWidth,
    int viewportHeight);

void logUnsupportedRenderableDescriptorOnce(
    std::vector<dart::gui::RenderableId>& loggedUnsupportedRenderableIds,
    const dart::gui::RenderableDescriptor& descriptor);

void synchronizeSceneRenderables(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    const std::vector<dart::gui::RenderableDescriptor>& descriptors,
    std::vector<SceneRenderable>& sceneRenderables,
    std::vector<dart::gui::RenderableId>& loggedUnsupportedRenderableIds,
    const RenderableFactory& createRenderable);

} // namespace dart::gui::detail
