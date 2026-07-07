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

// Ported from examples/heightmap: a procedurally generated HeightmapShape<
// float> on a SimpleFrame, with a panel to regenerate the terrain (resolution
// / size / height range / color) and configure a GridVisual overlay.
//
// Deviations from the original: the panel no longer creates its own ImGui
// window (renderPanel already runs inside the host's own Inspector panel), so
// the stale title ("Point Cloud & Voxel Grid Demo"), the "TODO." description,
// the Menu bar, and the Help/Simulation sections (redundant with the host's
// own chrome and Simulation toolbar) are all dropped. The grid editor's
// function-local `static` variables (offset, cell count/step, line widths,
// colors) are promoted to scene-state members -- the original portRisk notes
// they "persist across widget instances/scene switches ... and would leak
// state between demos"; as scene-owned state they now reset cleanly every
// time this scene is (re)built. The GridVisual is registered as a host
// attachment via onActivate, so it is added/removed with this scene rather
// than for the app's lifetime.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>

#include <cmath>

namespace dart_demos {

namespace {

//==============================================================================
dart::dynamics::ShapePtr createHeightmapShape(
    std::size_t xResolution,
    std::size_t yResolution,
    float xSize,
    float ySize,
    float zMin,
    float zMax)
{
  dart::dynamics::HeightmapShapef::HeightField data(yResolution, xResolution);
  for (std::size_t i = 0; i < yResolution; ++i) {
    for (std::size_t j = 0; j < xResolution; ++j)
      data(i, j) = dart::math::Random::uniform(zMin, zMax);
  }

  const Eigen::Vector3f scale(
      xSize / static_cast<float>(xResolution),
      ySize / static_cast<float>(yResolution),
      1.f);

  auto terrainShape = std::make_shared<dart::dynamics::HeightmapShapef>();
  terrainShape->setScale(scale);
  terrainShape->setHeightField(data);
  return terrainShape;
}

//==============================================================================
/// Per-instance state captured by this scene's panel/onActivate lambdas.
struct HeightmapState
{
  dart::dynamics::SimpleFramePtr terrain;
  ::osg::ref_ptr<dart::gui::osg::GridVisual> grid;

  int xResolution = 100;
  int yResolution = 100;
  float xSize = 2.f;
  float ySize = 2.f;
  float zMin = 0.f;
  float zMax = 0.1f;

  // Grid editor fields: promoted from the original's function-local statistics
  // (see the file comment) so they are scene-owned instead of leaking
  // between scenes/widget instances.
  Eigen::Vector3f gridOffset = Eigen::Vector3f::Zero();
  int gridLineCount = 0;
  float gridLineStepSize = 0.f;
  int gridMinorLinesPerMajor = 0;
  float gridAxisLineWidth = 0.f;
  float gridMajorLineWidth = 0.f;
  float gridMinorLineWidth = 0.f;
  // 4 components (rgb + alpha) even though only rgb is editable via
  // ColorEdit3: GridVisual's color getters/setters are Vector4d, so alpha
  // must round-trip unchanged.
  Eigen::Vector4f gridMajorColor = Eigen::Vector4f::Zero();
  Eigen::Vector4f gridMinorColor = Eigen::Vector4f::Zero();
  bool gridFieldsInitialized = false;
};

//==============================================================================
void updateHeightmapShape(HeightmapState& state)
{
  const std::size_t xRes
      = static_cast<std::size_t>(std::max(1, state.xResolution));
  const std::size_t yRes
      = static_cast<std::size_t>(std::max(1, state.yResolution));

  state.terrain->setShape(createHeightmapShape(
      xRes, yRes, state.xSize, state.ySize, state.zMin, state.zMax));

  auto tf = state.terrain->getRelativeTransform();
  tf.translation()[0] = -static_cast<double>(state.xSize) / 2.0;
  tf.translation()[1] = +static_cast<double>(state.ySize) / 2.0;
  state.terrain->setRelativeTransform(tf);
}

//==============================================================================
/// Pulls the GridVisual's current values into state the first time the panel
/// runs, so the editor reflects the grid's actual defaults instead of
/// zero-initialized members.
void ensureGridFieldsInitialized(HeightmapState& state)
{
  if (state.gridFieldsInitialized || !state.grid)
    return;

  state.gridOffset = state.grid->getOffset().cast<float>();
  state.gridLineCount = static_cast<int>(state.grid->getNumCells());
  state.gridLineStepSize
      = static_cast<float>(state.grid->getMinorLineStepSize());
  state.gridMinorLinesPerMajor
      = static_cast<int>(state.grid->getNumMinorLinesPerMajorLine());
  state.gridAxisLineWidth = state.grid->getAxisLineWidth();
  state.gridMajorLineWidth = state.grid->getMajorLineWidth();
  state.gridMinorLineWidth = state.grid->getMinorLineWidth();
  state.gridMajorColor = state.grid->getMajorLineColor().cast<float>();
  state.gridMinorColor = state.grid->getMinorLineColor().cast<float>();
  state.gridFieldsInitialized = true;
}

} // namespace

//==============================================================================
DemoScene makeHeightmapScene()
{
  DemoScene scene;
  scene.id = "heightmap";
  scene.title = "Heightmap";
  scene.category = "Visualization";
  scene.summary
      = "An interactive procedural heightmap with a configurable grid.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());

    auto state = std::make_shared<HeightmapState>();

    state->terrain = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World());
    state->terrain->createVisualAspect();
    world->addSimpleFrame(state->terrain);
    updateHeightmapShape(*state);

    state->grid = new dart::gui::osg::GridVisual();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.57, 3.14, 1.64),
        ::osg::Vec3d(0.00, 0.00, 0.30),
        ::osg::Vec3d(-0.24, -0.25, 0.94)};

    setup.onActivate = [state](DemoHostContext& ctx) {
      ctx.addAttachment(state->grid.get());
    };

    setup.renderPanel = [state] {
      ensureGridFieldsInitialized(*state);

      if (ImGui::CollapsingHeader(
              "Heightmap", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto* aspect = state->terrain->getVisualAspect();
        bool display = !aspect->isHidden();
        if (ImGui::Checkbox("Show##Terrain", &display)) {
          if (display)
            aspect->show();
          else
            aspect->hide();
        }

        bool changed = false;

        // Every field edits a local copy (seeded from state each frame) and
        // only commits a clamped, finite value that actually differs from the
        // current one. This keeps a typed "nan"/"inf" out of persistent state
        // (it would otherwise reach Random::uniform / the frame transform on
        // the next regeneration) and makes a clamped no-op edit -- e.g. "-" at
        // the minimum resolution -- not re-randomize the terrain, matching the
        // original's old-vs-new comparison.
        int xRes = state->xResolution;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::InputInt("X Resolution", &xRes, 5, 10)) {
          xRes = std::max(5, xRes);
          if (xRes != state->xResolution) {
            state->xResolution = xRes;
            changed = true;
          }
        }

        int yRes = state->yResolution;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::InputInt("Y Resolution", &yRes, 5, 10)) {
          yRes = std::max(5, yRes);
          if (yRes != state->yResolution) {
            state->yResolution = yRes;
            changed = true;
          }
        }

        ImGui::Separator();

        float xSize = state->xSize;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::InputFloat("X Size", &xSize, 0.1f, 0.2f)
            && std::isfinite(xSize)) {
          xSize = std::max(0.1f, xSize);
          if (xSize != state->xSize) {
            state->xSize = xSize;
            changed = true;
          }
        }

        float ySize = state->ySize;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::InputFloat("Y Size", &ySize, 0.1f, 0.2f)
            && std::isfinite(ySize)) {
          ySize = std::max(0.1f, ySize);
          if (ySize != state->ySize) {
            state->ySize = ySize;
            changed = true;
          }
        }

        ImGui::Separator();

        float zMin = state->zMin;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::InputFloat("Z Min", &zMin, 0.05f, 0.1f)
            && std::isfinite(zMin) && zMin != state->zMin) {
          state->zMin = zMin;
          changed = true;
        }

        float zMax = state->zMax;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
        if (ImGui::InputFloat("Z Max", &zMax, 0.05f, 0.1f)
            && std::isfinite(zMax) && zMax != state->zMax) {
          state->zMax = zMax;
          changed = true;
        }

        // Unlike the original, keep zMin <= zMax: a fully inverted range fed
        // straight into Random::uniform(zMin, zMax) is undefined behavior,
        // not just a cosmetic glitch. (Both are finite by construction here.)
        if (state->zMin > state->zMax)
          std::swap(state->zMin, state->zMax);

        if (changed)
          updateHeightmapShape(*state);

        ImGui::Separator();

        auto* visualAspect = state->terrain->getVisualAspect();
        Eigen::Vector4d visualColor = visualAspect->getRGBA();
        float color[4]
            = {static_cast<float>(visualColor[0]),
               static_cast<float>(visualColor[1]),
               static_cast<float>(visualColor[2]),
               static_cast<float>(visualColor[3])};
        if (ImGui::ColorEdit4("Color##Heightmap", color)) {
          visualAspect->setColor(
              Eigen::Vector4d(color[0], color[1], color[2], color[3]));
        }
      }

      if (ImGui::CollapsingHeader("Grid")) {
        bool display = state->grid->isDisplayed();
        if (ImGui::Checkbox("Show##Grid", &display))
          state->grid->display(display);

        if (display) {
          int plane = static_cast<int>(state->grid->getPlaneType());
          if (ImGui::RadioButton("XY-Plane", &plane, 0))
            state->grid->setPlaneType(
                dart::gui::osg::GridVisual::PlaneType::XY);
          ImGui::SameLine();
          if (ImGui::RadioButton("YZ-Plane", &plane, 1))
            state->grid->setPlaneType(
                dart::gui::osg::GridVisual::PlaneType::YZ);
          ImGui::SameLine();
          if (ImGui::RadioButton("ZX-Plane", &plane, 2))
            state->grid->setPlaneType(
                dart::gui::osg::GridVisual::PlaneType::ZX);

          // Same local-copy discipline as the terrain fields above: route each
          // grid tunable through a temporary and commit only a finite (and, for
          // widths, non-negative) value, so a typed "nan"/"inf" or a negative
          // line width never reaches GridVisual (its own guards clamp only the
          // step size).
          const char* const kOffsetLabels[3] = {"X", "Y", "Z"};
          ImGui::Columns(3);
          for (int c = 0; c < 3; ++c) {
            float off = state->gridOffset[c];
            if (ImGui::InputFloat(kOffsetLabels[c], &off, 0.1f, 0.5f, "%.1f")
                && std::isfinite(off) && off != state->gridOffset[c]) {
              state->gridOffset[c] = off;
              state->grid->setOffset(state->gridOffset.cast<double>());
            }
            if (c < 2)
              ImGui::NextColumn();
          }
          ImGui::Columns(1);

          int lineCount = state->gridLineCount;
          if (ImGui::InputInt("Line Count", &lineCount, 1, 5)) {
            lineCount = std::max(0, lineCount);
            if (lineCount != state->gridLineCount) {
              state->gridLineCount = lineCount;
              state->grid->setNumCells(
                  static_cast<std::size_t>(state->gridLineCount));
            }
          }

          float lineStep = state->gridLineStepSize;
          if (ImGui::InputFloat("Line Step Size", &lineStep, 0.001f, 0.1f)
              && std::isfinite(lineStep)
              && lineStep != state->gridLineStepSize) {
            state->gridLineStepSize = lineStep;
            state->grid->setMinorLineStepSize(
                static_cast<double>(state->gridLineStepSize));
          }

          int minorPerMajor = state->gridMinorLinesPerMajor;
          if (ImGui::InputInt(
                  "Minor Lines per Major Line", &minorPerMajor, 1, 5)) {
            minorPerMajor = std::max(0, minorPerMajor);
            if (minorPerMajor != state->gridMinorLinesPerMajor) {
              state->gridMinorLinesPerMajor = minorPerMajor;
              state->grid->setNumMinorLinesPerMajorLine(
                  static_cast<std::size_t>(state->gridMinorLinesPerMajor));
            }
          }

          float axisWidth = state->gridAxisLineWidth;
          if (ImGui::InputFloat("Axis Line Width", &axisWidth, 1.f, 2.f, "%.0f")
              && std::isfinite(axisWidth)) {
            axisWidth = std::max(0.f, axisWidth);
            if (axisWidth != state->gridAxisLineWidth) {
              state->gridAxisLineWidth = axisWidth;
              state->grid->setAxisLineWidth(state->gridAxisLineWidth);
            }
          }

          float majorWidth = state->gridMajorLineWidth;
          if (ImGui::InputFloat(
                  "Major Line Width", &majorWidth, 1.f, 2.f, "%.0f")
              && std::isfinite(majorWidth)) {
            majorWidth = std::max(0.f, majorWidth);
            if (majorWidth != state->gridMajorLineWidth) {
              state->gridMajorLineWidth = majorWidth;
              state->grid->setMajorLineWidth(state->gridMajorLineWidth);
            }
          }

          if (ImGui::ColorEdit3(
                  "Major Line Color", state->gridMajorColor.data()))
            state->grid->setMajorLineColor(
                state->gridMajorColor.cast<double>());

          float minorWidth = state->gridMinorLineWidth;
          if (ImGui::InputFloat(
                  "Minor Line Width", &minorWidth, 1.f, 2.f, "%.0f")
              && std::isfinite(minorWidth)) {
            minorWidth = std::max(0.f, minorWidth);
            if (minorWidth != state->gridMinorLineWidth) {
              state->gridMinorLineWidth = minorWidth;
              state->grid->setMinorLineWidth(state->gridMinorLineWidth);
            }
          }

          if (ImGui::ColorEdit3(
                  "Minor Line Color", state->gridMinorColor.data()))
            state->grid->setMinorLineColor(
                state->gridMinorColor.cast<double>());
        }
      }
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
