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

#ifndef DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_DEBUG_OVERLAY_HPP_
#define DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_DEBUG_OVERLAY_HPP_

#include <dart/gui/experimental/debug.hpp>
#include <dart/gui/experimental/detail/filament/renderable_resources.hpp>

#include <optional>
#include <vector>

namespace filament {
class Engine;
class Material;
class Scene;
} // namespace filament

namespace dart::gui::experimental::filament {

struct DebugOverlayController
{
  dart::gui::experimental::DebugDrawOptions staticOptions;
  dart::gui::experimental::DebugDrawOptions contactOptions;
  std::optional<Renderable> staticOverlay;
  std::optional<Renderable> contactOverlay;
};

DebugOverlayController makeDebugOverlayController(bool drawSupportPolygons);

void clearDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    std::optional<Renderable>& overlay);

void clearDebugOverlays(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    DebugOverlayController& controller);

void refreshDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::experimental::DebugLineDescriptor>& lines,
    std::optional<Renderable>& overlay);

void refreshStaticDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const dart::simulation::World& world,
    DebugOverlayController& controller);

void refreshContactDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const dart::collision::CollisionResult& result,
    DebugOverlayController& controller);

void refreshSelectionDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::experimental::RenderableDescriptor>&
        descriptors,
    dart::gui::experimental::RenderableId selectedRenderableId,
    std::optional<Renderable>& overlay);

} // namespace dart::gui::experimental::filament

#endif // DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_DEBUG_OVERLAY_HPP_
