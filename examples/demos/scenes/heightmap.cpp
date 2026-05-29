/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../../gui_source_grid.hpp"
#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart::examples::demos {

namespace {

constexpr const char* kInteractiveTerrainName = "visual_heightmap";
constexpr const char* kInteractiveGridName = "visual_heightmap_grid";
constexpr const char* kReferenceBoxName = "visual_heightmap_reference_box";
constexpr const char* kSampleBallPrefix = "visual_heightmap_sample_ball_";

struct HeightmapPanelState
{
  std::shared_ptr<dart::dynamics::SimpleFrame> terrain;
  dart::examples::SourceOwnedGridState grid;
  double xResolution = 100.0;
  double yResolution = 100.0;
  double xSize = 2.0;
  double ySize = 2.0;
  double zMin = -0.1;
  double zMax = 0.4;
  Eigen::Vector4d terrainColor = Eigen::Vector4d(0.24, 0.58, 0.88, 1.0);
  bool terrainVisible = true;
  int generation = 0;
};

struct HeightmapScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<HeightmapPanelState> panelState;
};

std::size_t clampResolution(double value)
{
  const double rounded = std::round(value);
  return static_cast<std::size_t>(std::clamp(rounded, 5.0, 100.0));
}

std::vector<double> generateHeightField(
    std::size_t xResolution,
    std::size_t yResolution,
    double zMin,
    double zMax,
    int generation)
{
  if (zMax < zMin) {
    std::swap(zMin, zMax);
  }

  std::vector<double> heights;
  heights.reserve(xResolution * yResolution);
  const double phase = static_cast<double>(generation) * 0.17;
  for (std::size_t y = 0; y < yResolution; ++y) {
    for (std::size_t x = 0; x < xResolution; ++x) {
      const double xPhase = static_cast<double>(x) * 0.31 + phase;
      const double yPhase = static_cast<double>(y) * 0.27 - phase * 0.5;
      const double ridge
          = (x == xResolution / 2 || y == yResolution / 2) ? 0.18 : 0.0;
      const double normalized = std::clamp(
          0.5 + ridge + 0.24 * std::sin(xPhase) * std::cos(yPhase), 0.0, 1.0);
      heights.push_back(zMin + (zMax - zMin) * normalized);
    }
  }
  return heights;
}

std::shared_ptr<dart::dynamics::HeightmapShaped> createHeightmapShape(
    std::size_t xResolution,
    std::size_t yResolution,
    double xSize,
    double ySize,
    double zMin,
    double zMax,
    int generation)
{
  auto shape = std::make_shared<dart::dynamics::HeightmapShaped>();
  shape->setDataVariance(dart::dynamics::Shape::DYNAMIC);
  shape->setHeightField(
      xResolution,
      yResolution,
      generateHeightField(xResolution, yResolution, zMin, zMax, generation));
  const auto xStride = xResolution > 1 ? xResolution - 1 : 1u;
  const auto yStride = yResolution > 1 ? yResolution - 1 : 1u;
  shape->setScale(
      Eigen::Vector3d(
          xSize / static_cast<double>(xStride),
          ySize / static_cast<double>(yStride),
          1.0));
  return shape;
}

void applyVisibility(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& frame, bool visible)
{
  if (frame == nullptr) {
    return;
  }

  auto* visual = frame->getVisualAspect(true);
  if (visible) {
    visual->show();
  } else {
    visual->hide();
  }
}

void applyTerrainColor(const std::shared_ptr<HeightmapPanelState>& state)
{
  if (state == nullptr || state->terrain == nullptr) {
    return;
  }

  state->terrain->getVisualAspect(true)->setRGBA(state->terrainColor);
}

void regenerateHeightmap(const std::shared_ptr<HeightmapPanelState>& state)
{
  if (state == nullptr || state->terrain == nullptr) {
    return;
  }

  const std::size_t xResolution = clampResolution(state->xResolution);
  const std::size_t yResolution = clampResolution(state->yResolution);
  state->xResolution = static_cast<double>(xResolution);
  state->yResolution = static_cast<double>(yResolution);
  state->xSize = std::max(0.1, state->xSize);
  state->ySize = std::max(0.1, state->ySize);

  state->terrain->setShape(createHeightmapShape(
      xResolution,
      yResolution,
      state->xSize,
      state->ySize,
      state->zMin,
      state->zMax,
      state->generation));
  applyTerrainColor(state);
  applyVisibility(state->terrain, state->terrainVisible);
}

std::shared_ptr<dart::dynamics::SimpleFrame> createInteractiveFrame(
    const std::string& name,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector4d& color)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  frame->setShape(shape);
  frame->getVisualAspect(true)->setRGBA(color);
  return frame;
}

dart::dynamics::SkeletonPtr createStaticVisualSkeleton(
    const std::string& name,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double alpha = 1.0)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>()
                   .second;
  auto* shapeNode
      = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  shapeNode->setRelativeTranslation(position);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(color.x(), color.y(), color.z(), alpha));
  return skeleton;
}

void addReferenceMarkers(const dart::simulation::WorldPtr& world)
{
  world->addSkeleton(createStaticVisualSkeleton(
      kReferenceBoxName,
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.48, 0.48, 0.28)),
      Eigen::Vector3d(0.72, 0.0, 0.20),
      Eigen::Vector3d(0.20, 0.72, 0.28),
      0.48));

  int index = 0;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      world->addSkeleton(createStaticVisualSkeleton(
          kSampleBallPrefix + std::to_string(index++),
          std::make_shared<dart::dynamics::SphereShape>(0.07),
          Eigen::Vector3d(
              -0.25 + static_cast<double>(x) * 0.42,
              static_cast<double>(y) * 0.32,
              0.45),
          Eigen::Vector3d(0.92, 0.48, 0.16)));
    }
  }
}

HeightmapScene createInteractiveHeightmapScene()
{
  HeightmapScene scene;
  scene.world = dart::simulation::World::create("dartsim_heightmap");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  auto terrain = createInteractiveFrame(
      kInteractiveTerrainName,
      std::make_shared<dart::dynamics::HeightmapShaped>(),
      Eigen::Vector4d(0.24, 0.58, 0.88, 1.0));
  scene.world->addSimpleFrame(terrain);
  addReferenceMarkers(scene.world);

  scene.panelState = std::make_shared<HeightmapPanelState>();
  scene.panelState->terrain = terrain;
  scene.panelState->grid.lineCount = 20.0;
  scene.panelState->grid.lineStepSize = 0.1;
  scene.panelState->grid.zOffset = -0.01;
  scene.panelState->grid.minorLineColor
      = Eigen::Vector4d(0.42, 0.42, 0.42, 0.42);
  scene.panelState->grid.majorLineColor
      = Eigen::Vector4d(0.56, 0.56, 0.56, 0.58);
  scene.panelState->grid.axisLineColor
      = Eigen::Vector4d(0.18, 0.24, 0.32, 0.82);
  dart::examples::attachSourceOwnedGridFrames(
      scene.world, scene.panelState->grid, kInteractiveGridName);
  regenerateHeightmap(scene.panelState);
  return scene;
}

dart::gui::OrbitCamera makeHeightmapCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.30);
  camera.yaw = 0.8848934155088675;
  camera.pitch = 0.31896455812752583;
  camera.distance = 4.27318382473771;
  return camera;
}

void addSimulationControls(
    dart::gui::PanelBuilder& builder, dart::gui::PanelContext& context)
{
  if (context.lifecycle == nullptr) {
    return;
  }

  if (builder.button("Play")) {
    context.lifecycle->paused = false;
  }
  builder.sameLine();
  if (builder.button("Pause")) {
    context.lifecycle->paused = true;
  }
  builder.sameLine();
  if (builder.button("Step")) {
    dart::gui::requestSingleStep(*context.lifecycle);
  }
  builder.sameLine();
  if (builder.button("Exit")) {
    dart::gui::requestExit(*context.lifecycle);
  }
}

dart::gui::Panel createHeightmapPanel(
    std::shared_ptr<HeightmapPanelState> state)
{
  dart::gui::Panel panel;
  panel.title = "Heightmap Demo";
  panel.buildWithContext = [state = std::move(state)](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Heightmap rendering example");
    builder.text(
        "Tweak the controls below to regenerate the heightmap. The "
        "grid stays aligned with the terrain.");
    addSimulationControls(builder, context);
    builder.separator();

    if (state == nullptr) {
      return;
    }

    bool changed = false;
    if (builder.checkbox("Show Terrain", state->terrainVisible)) {
      applyVisibility(state->terrain, state->terrainVisible);
    }
    if (builder.colorEdit("Terrain Color", state->terrainColor)) {
      applyTerrainColor(state);
    }
    changed |= builder.slider("X Resolution", state->xResolution, 5.0, 100.0);
    changed |= builder.slider("Y Resolution", state->yResolution, 5.0, 100.0);
    changed |= builder.slider("X Size", state->xSize, 0.1, 4.0);
    changed |= builder.slider("Y Size", state->ySize, 0.1, 4.0);
    changed |= builder.slider("Z Min", state->zMin, -1.0, 1.0);
    changed |= builder.slider("Z Max", state->zMax, -1.0, 1.0);
    if (builder.button("Regenerate")) {
      ++state->generation;
      changed = true;
    }
    if (changed) {
      regenerateHeightmap(state);
    }
    builder.separator();
    dart::examples::addSourceOwnedGridPanelControls(builder, state->grid);
    builder.separator();
    builder.text("Grid controls edit source-owned DART line geometry.");
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

dart::gui::ApplicationOptions makeHeightmapScene()
{
  HeightmapScene scene = createInteractiveHeightmapScene();

  dart::gui::ApplicationOptions options;
  options.world = scene.world;
  options.camera = makeHeightmapCamera();
  options.panels.push_back(createHeightmapPanel(scene.panelState));
  return options;
}

} // namespace dart::examples::demos
