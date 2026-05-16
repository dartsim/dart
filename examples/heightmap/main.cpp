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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#if DART_HAVE_ODE
  #include <dart/collision/ode/ode_collision_detector.hpp>
#endif

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>
#include <cstddef>

namespace {

constexpr const char* kInteractiveTerrainName = "visual_heightmap";
constexpr const char* kInteractiveGridName = "visual_heightmap_grid";
constexpr const char* kReferenceBoxName = "visual_heightmap_reference_box";
constexpr const char* kSampleBallPrefix = "visual_heightmap_sample_ball_";

enum class HeightmapDemo
{
  Interactive,
  Alignment,
};

struct HeightmapConfig
{
  HeightmapDemo demo = HeightmapDemo::Interactive;
};

struct ParseResult
{
  bool help = false;
};

struct HeightmapAlignmentDemoConfig
{
  Eigen::Vector3d heightmapOrigin = Eigen::Vector3d::Zero();
  std::size_t heightmapXResolution = 2u;
  std::size_t heightmapYResolution = 2u;
  double heightmapScale = 2.0;
  double heightmapZMin = 0.0;
  double heightmapZMax = 0.0;
  Eigen::Vector3d boxSize = Eigen::Vector3d(2.0, 2.0, 2.0);
  Eigen::Vector3d boxOffset = Eigen::Vector3d(3.0, 0.0, -1.0);
  std::size_t ballGridCount = 5u;
  double ballRadius = 0.08;
  double ballMass = 0.1;
  double ballDropHeight = 1.0;
};

struct HeightmapPanelState
{
  std::shared_ptr<dart::dynamics::SimpleFrame> terrain;
  std::shared_ptr<dart::dynamics::SimpleFrame> grid;
  double xResolution = 100.0;
  double yResolution = 100.0;
  double xSize = 2.0;
  double ySize = 2.0;
  double zMin = -0.1;
  double zMax = 0.4;
  bool terrainVisible = true;
  bool gridVisible = true;
  int generation = 0;
};

struct HeightmapScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<HeightmapPanelState> panelState;
};

std::optional<std::string_view> getOptionValue(
    std::string_view argument,
    std::string_view option,
    int& index,
    int argc,
    char* argv[])
{
  if (argument == option) {
    if (index + 1 >= argc) {
      throw std::runtime_error("Missing value for " + std::string(option));
    }
    return std::string_view(argv[++index]);
  }

  const std::string prefix = std::string(option) + "=";
  if (argument.starts_with(prefix)) {
    return argument.substr(prefix.size());
  }

  return std::nullopt;
}

std::optional<HeightmapDemo> parseDemo(std::string_view value)
{
  if (value == "interactive") {
    return HeightmapDemo::Interactive;
  }
  if (value == "alignment") {
    return HeightmapDemo::Alignment;
  }
  return std::nullopt;
}

void printHeightmapUsage(const char* executable)
{
  std::cout << "Usage: " << executable << " [--demo interactive|alignment]\n"
            << "       [common dart::gui flags such as --headless, --frames,\n"
            << "        --screenshot, --out, --width, --height, --gui-scale]\n";
}

ParseResult parseHeightmapConfig(
    int argc, char* argv[], HeightmapConfig& config)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument == "--help" || argument == "-h") {
      printHeightmapUsage(argv[0]);
      return {.help = true};
    }
    if (auto value = getOptionValue(argument, "--demo", i, argc, argv)) {
      if (auto demo = parseDemo(*value)) {
        config.demo = *demo;
      } else {
        throw std::runtime_error(
            "Unknown --demo value: " + std::string(*value));
      }
    }
  }

  return {};
}

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
  state->terrain->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.24, 0.58, 0.88, 1.0));
  applyVisibility(state->terrain, state->terrainVisible);
  applyVisibility(state->grid, state->gridVisible);
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createGridShape(
    double halfExtent = 1.0, int lineCount = 20)
{
  auto grid = std::make_shared<dart::dynamics::LineSegmentShape>(1.5f);
  for (int i = 0; i <= lineCount; ++i) {
    const double coordinate
        = -halfExtent + 2.0 * halfExtent * static_cast<double>(i) / lineCount;
    const auto startX
        = grid->addVertex(Eigen::Vector3d(-halfExtent, coordinate, -0.01));
    grid->addVertex(Eigen::Vector3d(halfExtent, coordinate, -0.01), startX);
    const auto startY
        = grid->addVertex(Eigen::Vector3d(coordinate, -halfExtent, -0.01));
    grid->addVertex(Eigen::Vector3d(coordinate, halfExtent, -0.01), startY);
  }
  return grid;
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

dart::dynamics::SkeletonPtr createAlignmentHeightmap(
    const HeightmapAlignmentDemoConfig& config)
{
  auto heightmap = dart::dynamics::Skeleton::create("heightmap");
  auto jointAndBody
      = heightmap->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = config.heightmapOrigin;
  joint->setTransformFromParentBodyNode(transform);

  auto shape = createHeightmapShape(
      config.heightmapXResolution,
      config.heightmapYResolution,
      config.heightmapScale,
      config.heightmapScale,
      config.heightmapZMin,
      config.heightmapZMax,
      0);

  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect,
      dart::dynamics::VisualAspect>(shape);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.2, 0.4, 0.9, 1.0));
  return heightmap;
}

dart::dynamics::SkeletonPtr createAlignmentReferenceBox(
    const HeightmapAlignmentDemoConfig& config)
{
  auto box = dart::dynamics::Skeleton::create("reference_box");
  auto jointAndBody
      = box->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = config.boxOffset;
  joint->setTransformFromParentBodyNode(transform);

  auto shape = std::make_shared<dart::dynamics::BoxShape>(config.boxSize);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect,
      dart::dynamics::VisualAspect>(shape);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.2, 0.7, 0.2, 1.0));
  return box;
}

dart::dynamics::SkeletonPtr createAlignmentBall(
    const std::string& name,
    const Eigen::Vector3d& position,
    double radius,
    double mass,
    const Eigen::Vector3d& color)
{
  auto ball = dart::dynamics::Skeleton::create(name);
  auto jointAndBody
      = ball->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  joint->setTransformFromParentBodyNode(transform);

  auto shape = std::make_shared<dart::dynamics::SphereShape>(radius);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(color.x(), color.y(), color.z(), 1.0));

  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);
  return ball;
}

void addAlignmentBallGrid(
    const dart::simulation::WorldPtr& world,
    const std::string& prefix,
    const Eigen::Vector3d& center,
    double halfExtent,
    std::size_t count,
    double dropHeight,
    double radius,
    double mass,
    const Eigen::Vector3d& color)
{
  if (count == 0u) {
    return;
  }

  const double step = count == 1u ? 0.0 : (2.0 * halfExtent) / (count - 1u);
  std::size_t index = 0u;
  for (std::size_t row = 0; row < count; ++row) {
    for (std::size_t col = 0; col < count; ++col) {
      const Eigen::Vector3d position(
          center.x() - halfExtent + step * static_cast<double>(row),
          center.y() - halfExtent + step * static_cast<double>(col),
          dropHeight);
      world->addSkeleton(createAlignmentBall(
          prefix + std::to_string(index++), position, radius, mass, color));
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
  auto grid = createInteractiveFrame(
      kInteractiveGridName,
      createGridShape(),
      Eigen::Vector4d(0.45, 0.45, 0.45, 0.58));
  scene.world->addSimpleFrame(terrain);
  scene.world->addSimpleFrame(grid);
  addReferenceMarkers(scene.world);

  scene.panelState = std::make_shared<HeightmapPanelState>();
  scene.panelState->terrain = terrain;
  scene.panelState->grid = grid;
  regenerateHeightmap(scene.panelState);
  return scene;
}

void setupAlignmentDemo(const dart::simulation::WorldPtr& world)
{
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);

#if DART_HAVE_ODE
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::OdeCollisionDetector::create());
#else
  std::cerr << "heightmap: ODE is not available; using the default collision "
               "detector for alignment mode.\n";
#endif

  HeightmapAlignmentDemoConfig config;
  world->addSkeleton(createAlignmentHeightmap(config));
  world->addSkeleton(createAlignmentReferenceBox(config));

  const double halfExtent = 1.0 - config.ballRadius * 1.1;
  const Eigen::Vector3d ballColor(0.9, 0.7, 0.3);
  addAlignmentBallGrid(
      world,
      "heightmap_ball_",
      config.heightmapOrigin,
      halfExtent,
      config.ballGridCount,
      config.ballDropHeight,
      config.ballRadius,
      config.ballMass,
      ballColor);
  addAlignmentBallGrid(
      world,
      "box_ball_",
      config.heightmapOrigin + config.boxOffset,
      halfExtent,
      config.ballGridCount,
      config.ballDropHeight,
      config.ballRadius,
      config.ballMass,
      ballColor);
}

HeightmapScene createAlignmentHeightmapScene()
{
  HeightmapScene scene;
  scene.world = dart::simulation::World::create("dartsim_heightmap_alignment");
  setupAlignmentDemo(scene.world);
  return scene;
}

dart::gui::RunOptions makeHeightmapRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 720;
  return options;
}

dart::gui::OrbitCamera makeHeightmapCamera(HeightmapDemo demo)
{
  dart::gui::OrbitCamera camera;
  if (demo == HeightmapDemo::Alignment) {
    camera.target = Eigen::Vector3d(1.5, 0.0, 0.3);
    camera.yaw = 0.8716037370732271;
    camera.pitch = 0.33479550808323927;
    camera.distance = 6.086871117413281;
  } else {
    camera.target = Eigen::Vector3d(0.0, 0.0, 0.30);
    camera.yaw = 0.8848934155088675;
    camera.pitch = 0.31896455812752583;
    camera.distance = 4.27318382473771;
  }
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
    std::shared_ptr<HeightmapPanelState> state, HeightmapDemo demo)
{
  dart::gui::Panel panel;
  panel.title = "Heightmap Demo";
  panel.buildWithContext = [state = std::move(state), demo](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Heightmap rendering example");
    if (demo == HeightmapDemo::Alignment) {
      builder.text(
          "Alignment mode compares a collision heightmap with a "
          "reference box using falling ball grids.");
      addSimulationControls(builder, context);
      builder.separator();
      builder.text("heightmap_ball_ and box_ball_ show contact drift.");
      builder.text("time: " + std::to_string(context.simulationTime));
      builder.text("contacts: " + std::to_string(context.contactCount));
      return;
    }

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
    if (builder.checkbox("Show Grid", state->gridVisible)) {
      applyVisibility(state->grid, state->gridVisible);
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
    builder.text("Grid style controls are a follow-up public debug-grid API.");
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    HeightmapConfig config;
    const auto parseResult = parseHeightmapConfig(argc, argv, config);
    if (parseResult.help) {
      return 0;
    }

    HeightmapScene scene = config.demo == HeightmapDemo::Alignment
                               ? createAlignmentHeightmapScene()
                               : createInteractiveHeightmapScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.runDefaults = makeHeightmapRunDefaults();
    options.camera = makeHeightmapCamera(config.demo);
    options.panels.push_back(
        createHeightmapPanel(scene.panelState, config.demo));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "heightmap: " << e.what() << "\n";
    return 1;
  }
}
