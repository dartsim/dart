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

#include <dart/gui/detail/debug_overlay.hpp>
#include <dart/gui/detail/renderable_factory.hpp>
#include <dart/gui/detail/renderable_sync.hpp>
#include <dart/gui/gizmo.hpp>

#include <algorithm>

namespace dart::gui::detail {

namespace {

constexpr double kGizmoWorldScale = 1.0;

} // namespace

using dart::gui::DebugDrawOptions;
using dart::gui::extractContactDebugLines;
using dart::gui::extractDebugLines;
using dart::gui::makeSelectionDebugLines;
using dart::gui::RenderableDescriptor;
using dart::gui::RenderableId;

DebugOverlayController makeDebugOverlayController(bool drawSupportPolygons)
{
  DebugOverlayController controller;
  controller.staticOptions.drawBodyFrames = true;
  controller.staticOptions.drawCentersOfMass = true;
  controller.staticOptions.drawInertiaBoxes = false;
  controller.staticOptions.drawCollisionShapeBounds = false;
  controller.staticOptions.drawSupportPolygons = drawSupportPolygons;
  controller.staticOptions.drawContacts = false;

  controller.contactOptions.drawGrid = false;
  controller.contactOptions.drawWorldFrame = false;
  controller.contactOptions.drawBodyFrames = false;
  controller.contactOptions.drawCentersOfMass = false;
  controller.contactOptions.drawContacts = false;
  controller.contactOptions.drawContactNormals = false;
  controller.contactOptions.drawContactForces = false;
  return controller;
}

void clearDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    std::optional<Renderable>& overlay)
{
  if (!overlay) {
    return;
  }

  removeRenderableFromScene(scene, *overlay);
  destroyRenderable(engine, *overlay);
  overlay.reset();
}

void clearProviderDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    DebugOverlayController& controller)
{
  clearDebugLineOverlay(engine, scene, controller.providerOverlay);
  controller.providerLabels.clear();
}

void clearDebugOverlays(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    DebugOverlayController& controller)
{
  clearDebugLineOverlay(engine, scene, controller.staticOverlay);
  clearDebugLineOverlay(engine, scene, controller.contactOverlay);
  clearDebugLineOverlay(engine, scene, controller.gizmoOverlay);
  clearProviderDebugOverlay(engine, scene, controller);
}

void refreshDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    const std::vector<dart::gui::DebugTriangleDescriptor>& triangles,
    std::optional<Renderable>& overlay)
{
  clearDebugLineOverlay(engine, scene, overlay);

  overlay = createDebugLineRenderable(engine, material, lines, triangles);
  if (overlay) {
    addRenderableToScene(scene, *overlay);
  }
}

void refreshDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    std::optional<Renderable>& overlay)
{
  refreshDebugLineOverlay(engine, scene, material, lines, {}, overlay);
}

void refreshStaticDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    DebugOverlayController& controller)
{
  refreshDebugLineOverlay(
      engine,
      scene,
      material,
      extractDebugLines(controller.staticOptions),
      controller.staticOverlay);
}

void refreshContactDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const dart::collision::CollisionResult& result,
    DebugOverlayController& controller)
{
  refreshDebugLineOverlay(
      engine,
      scene,
      material,
      extractContactDebugLines(result, controller.contactOptions),
      controller.contactOverlay);
}

void refreshProviderDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    const std::vector<dart::gui::DebugTriangleDescriptor>& triangles,
    DebugOverlayController& controller)
{
  refreshDebugLineOverlay(
      engine, scene, material, lines, triangles, controller.providerOverlay);
}

void refreshGizmoDebugOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<dart::gui::Gizmo>& gizmos,
    double,
    std::optional<dart::gui::GizmoHandleHit> highlightedGizmoHandle,
    DebugOverlayController& controller)
{
  refreshDebugLineOverlay(
      engine,
      scene,
      material,
      dart::gui::makeGizmoDebugLines(
          gizmos, kGizmoWorldScale, highlightedGizmoHandle),
      dart::gui::makeGizmoDebugTriangles(
          gizmos, kGizmoWorldScale, highlightedGizmoHandle),
      controller.gizmoOverlay);
}

void refreshSelectionDebugLineOverlay(
    ::filament::Engine& engine,
    ::filament::Scene& scene,
    ::filament::Material& material,
    const std::vector<RenderableDescriptor>& descriptors,
    RenderableId selectedRenderableId,
    const std::vector<dart::gui::DebugLineDescriptor>& forceDragLines,
    std::optional<Renderable>& overlay)
{
  std::vector<dart::gui::DebugLineDescriptor> lines = forceDragLines;
  if (selectedRenderableId == 0) {
    if (!lines.empty()) {
      refreshDebugLineOverlay(engine, scene, material, lines, overlay);
      return;
    }
    clearDebugLineOverlay(engine, scene, overlay);
    return;
  }

  const auto selectedDescriptor = std::find_if(
      descriptors.begin(),
      descriptors.end(),
      [&](const RenderableDescriptor& candidate) {
        return candidate.id == selectedRenderableId;
      });
  if (selectedDescriptor == descriptors.end()
      || !selectedDescriptor->material.visible) {
    if (!lines.empty()) {
      refreshDebugLineOverlay(engine, scene, material, lines, overlay);
      return;
    }
    clearDebugLineOverlay(engine, scene, overlay);
    return;
  }

  const auto selectionLines = makeSelectionDebugLines(*selectedDescriptor);
  lines.insert(lines.end(), selectionLines.begin(), selectionLines.end());
  refreshDebugLineOverlay(engine, scene, material, lines, overlay);
}

} // namespace dart::gui::detail
