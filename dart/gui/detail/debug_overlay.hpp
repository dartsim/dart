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

#ifndef DART_GUI_DETAIL_DEBUG_OVERLAY_HPP_
#define DART_GUI_DETAIL_DEBUG_OVERLAY_HPP_

#include <dart/gui/debug.hpp>
#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/gizmo.hpp>

#include <optional>
#include <vector>

namespace filament {
class Engine;
class Material;
class Scene;
} // namespace filament

namespace dart::gui::detail {

struct DebugOverlayController
{
  dart::gui::DebugDrawOptions staticOptions;
  dart::gui::DebugDrawOptions contactOptions;
  std::optional<Renderable> staticOverlay;
  std::optional<Renderable> contactOverlay;
  std::optional<Renderable> gizmoOverlay;
  /// Overlay and labels supplied by the application's per-frame debug provider.
  std::optional<Renderable> providerOverlay;
  std::vector<dart::gui::DebugLabelDescriptor> providerLabels;
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

/// Clears the application-provided debug overlay geometry and its labels as a
/// unit. The inverse of refreshProviderDebugOverlay: a labels-only DebugScene
/// populates providerLabels without creating an overlay entity, so the overlay
/// and labels must always be cleared together.
void clearProviderDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    DebugOverlayController& controller);

void refreshDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    std::optional<Renderable>& overlay);

void refreshStaticDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    DebugOverlayController& controller);

void refreshContactDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const dart::collision::CollisionResult& result,
    DebugOverlayController& controller);

/// Refreshes the application-provided debug overlay from a DebugScene's lines
/// and triangles. Labels are stored on the controller for the UI pass.
void refreshProviderDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    const std::vector<dart::gui::DebugTriangleDescriptor>& triangles,
    DebugOverlayController& controller);

void refreshGizmoDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::Gizmo>& gizmos,
    double guiScale,
    std::optional<dart::gui::GizmoHandleHit> highlightedGizmoHandle,
    DebugOverlayController& controller);

void refreshSelectionDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::RenderableDescriptor>& descriptors,
    dart::gui::RenderableId selectedRenderableId,
    const std::vector<dart::gui::DebugLineDescriptor>& forceDragLines,
    std::optional<Renderable>& overlay);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_DEBUG_OVERLAY_HPP_
