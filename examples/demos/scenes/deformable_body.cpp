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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "memory_diagnostics.hpp"
#include "scenes.hpp"

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/io/deformable_scene_io.hpp>
#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::examples::demos {

namespace {

namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace sx = dart::simulation;
namespace sxio = dart::simulation::io;

//==============================================================================
Eigen::Vector4d rgba(double r, double g, double b, double a = 1.0)
{
  return {r, g, b, a};
}

//==============================================================================
Eigen::Isometry3d makeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  return transform;
}

//==============================================================================
struct DeformableVisual
{
  sx::DeformableBody body;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
  std::vector<std::uint8_t> fixed;
  std::vector<Eigen::Vector3i> surfaceTriangles;
  int surfaceRenderableKey = 0;
  std::shared_ptr<dynamics::LineSegmentShape> edgeShape;
  dynamics::SimpleFramePtr edgeFrame;
  std::vector<dynamics::SimpleFramePtr> nodeFrames;
};

//==============================================================================
struct LaunchOptions
{
  std::filesystem::path scenePath;
  std::filesystem::path diagnosticsJsonPath;
  std::string deformableView = "combined";
  std::string sceneKind = "net"; // built-in scene when no --deformable-scene
};

//==============================================================================
struct ExampleState
{
  sx::World physicsWorld;
  std::vector<DeformableVisual> deformables;
  std::vector<dynamics::SimpleFramePtr> staticFrames;
  int stepsPerFrame = 2;
  bool showSurfaceMesh = true;
  bool showPointMasses = true;
  bool showSpringEdges = true;
  std::size_t surfaceVersion = 1;
  std::filesystem::path diagnosticsJsonPath;
  std::size_t diagnosticsFrameOffset = 0;

  void applyVisualOptions()
  {
    for (const auto& deformable : deformables) {
      if (deformable.edgeFrame) {
        auto* visual = deformable.edgeFrame->getVisualAspect();
        if (showSpringEdges) {
          visual->show();
        } else {
          visual->hide();
        }
      }

      for (const auto& frame : deformable.nodeFrames) {
        auto* visual = frame->getVisualAspect();
        if (showPointMasses) {
          visual->show();
        } else {
          visual->hide();
        }
      }
    }
  }

  void syncRenderFrames()
  {
    for (auto& deformable : deformables) {
      const auto nodeCount = deformable.body.getNodeCount();
      for (std::size_t i = 0; i < nodeCount; ++i) {
        const auto position = deformable.body.getPosition(i);
        deformable.edgeShape->setVertex(i, position);
        deformable.nodeFrames[i]->setTransform(makeTransform(position));
      }
    }
    ++surfaceVersion;
    applyVisualOptions();
  }

  void writeDiagnostics() const
  {
    if (diagnosticsJsonPath.empty()) {
      return;
    }

    auto diagnostics = sxio::collectDeformableSceneDiagnostics(physicsWorld);
    diagnostics.frame = diagnostics.frame >= diagnosticsFrameOffset
                            ? diagnostics.frame - diagnosticsFrameOffset
                            : 0u;
    std::ofstream output(diagnosticsJsonPath);
    sxio::writeDeformableSceneDiagnosticsJson(output, diagnostics);
  }

  void step()
  {
    for (int i = 0; i < stepsPerFrame; ++i) {
      physicsWorld.step();
    }
    syncRenderFrames();
    writeDiagnostics();
  }

  void reset()
  {
    physicsWorld.setTime(0.0);
    diagnosticsFrameOffset = physicsWorld.getFrame();

    for (auto& deformable : deformables) {
      for (std::size_t i = 0; i < deformable.initialPositions.size(); ++i) {
        deformable.body.setPosition(i, deformable.initialPositions[i]);
        deformable.body.setVelocity(i, deformable.initialVelocities[i]);
      }
    }

    syncRenderFrames();
    writeDiagnostics();
  }

  std::vector<gui::RenderableDescriptor> makeSurfaceRenderables() const
  {
    if (!showSurfaceMesh) {
      return {};
    }

    std::vector<gui::RenderableDescriptor> descriptors;
    for (const auto& deformable : deformables) {
      if (deformable.surfaceTriangles.empty()) {
        continue;
      }

      const auto nodeCount = deformable.body.getNodeCount();
      if (nodeCount == 0) {
        continue;
      }

      std::vector<Eigen::Vector3d> positions;
      positions.reserve(nodeCount);
      for (std::size_t i = 0; i < nodeCount; ++i) {
        positions.push_back(deformable.body.getPosition(i));
      }

      gui::DeformableSurfaceRenderOptions surfaceOptions;
      surfaceOptions.version = surfaceVersion;
      const auto descriptor = gui::makeDeformableSurfaceRenderable(
          reinterpret_cast<gui::RenderableId>(&deformable.surfaceRenderableKey),
          positions,
          deformable.surfaceTriangles,
          surfaceOptions);
      if (descriptor) {
        descriptors.push_back(*descriptor);
      }
    }

    return descriptors;
  }

  std::vector<gui::RenderableDescriptor> makeRenderables() const
  {
    std::vector<gui::RenderableDescriptor> descriptors
        = makeSurfaceRenderables();
    for (const auto& frame : staticFrames) {
      if (auto descriptor = gui::describeShapeFrame(*frame)) {
        descriptors.push_back(*descriptor);
      }
    }
    for (const auto& deformable : deformables) {
      if (deformable.edgeFrame) {
        if (auto descriptor = gui::describeShapeFrame(*deformable.edgeFrame)) {
          descriptors.push_back(*descriptor);
        }
      }
      for (const auto& frame : deformable.nodeFrames) {
        if (auto descriptor = gui::describeShapeFrame(*frame)) {
          descriptors.push_back(*descriptor);
        }
      }
    }
    return descriptors;
  }
};

//==============================================================================
void addGround(ExampleState& state)
{
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.04);
  auto ground = state.physicsWorld.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.4, 1.2, 0.04)));
  sx::DeformableObstaclePolicy groundPolicy;
  groundPolicy.groundBarrier = true;
  ground.setDeformableObstaclePolicy(groundPolicy);

  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(),
      "ground_visual",
      makeTransform({0.0, 0.0, -0.04}));
  frame->setShape(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(4.8, 2.4, 0.08)));
  frame->getVisualAspect(true)->setRGBA(rgba(0.37, 0.40, 0.43));
  state.staticFrames.push_back(std::move(frame));
}

//==============================================================================
sx::DeformableBodyOptions makeNetOptions(
    int columns,
    int rows,
    std::vector<sx::DeformableEdge>& edges,
    std::vector<std::uint8_t>& fixed)
{
  sx::DeformableBodyOptions options;
  options.edgeStiffness = 65.0;
  options.damping = 1.1;

  const auto index = [columns](int col, int row) {
    return static_cast<std::size_t>(row * columns + col);
  };

  constexpr double spacing = 0.26;
  const double halfWidth = 0.5 * spacing * static_cast<double>(columns - 1);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < columns; ++col) {
      const double x = spacing * static_cast<double>(col) - halfWidth;
      const double y = 0.035 * std::sin(0.9 * static_cast<double>(col));
      const double z = 0.92 - 0.135 * static_cast<double>(row);
      options.positions.push_back(Eigen::Vector3d(x, y, z));
      options.velocities.push_back(
          Eigen::Vector3d(0.0, 0.16 * std::sin(0.7 * row + 0.3 * col), 0.0));
      options.masses.push_back(0.08);
      fixed.push_back(0u);
    }
  }

  options.fixedNodes = {index(0, 0), index(columns - 1, 0)};
  fixed[index(0, 0)] = 1u;
  fixed[index(columns - 1, 0)] = 1u;

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < columns; ++col) {
      if (col + 1 < columns) {
        edges.push_back({index(col, row), index(col + 1, row), -1.0});
      }
      if (row + 1 < rows) {
        edges.push_back({index(col, row), index(col, row + 1), -1.0});
      }
      if (col + 1 < columns && row + 1 < rows) {
        edges.push_back({index(col, row), index(col + 1, row + 1), -1.0});
        edges.push_back({index(col + 1, row), index(col, row + 1), -1.0});
      }
    }
  }
  options.edges = edges;

  for (const auto& triangle : gui::makeGridSurfaceTriangles(columns, rows)) {
    options.surfaceTriangles.push_back(
        {static_cast<std::size_t>(triangle.x()),
         static_cast<std::size_t>(triangle.y()),
         static_cast<std::size_t>(triangle.z())});
  }
  return options;
}

//==============================================================================
std::vector<Eigen::Vector3i> extractSurfaceTriangles(
    const sx::DeformableBody& body)
{
  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(body.getSurfaceTriangleCount());
  for (std::size_t i = 0; i < body.getSurfaceTriangleCount(); ++i) {
    const auto triangle = body.getSurfaceTriangle(i);
    triangles.emplace_back(
        static_cast<int>(triangle.nodeA),
        static_cast<int>(triangle.nodeB),
        static_cast<int>(triangle.nodeC));
  }
  return triangles;
}

//==============================================================================
void appendDeformableVisual(
    ExampleState& state, const sx::DeformableBody& body, std::string_view name)
{
  DeformableVisual visual;
  visual.body = body;
  const auto nodeCount = body.getNodeCount();
  visual.initialPositions.reserve(nodeCount);
  visual.initialVelocities.reserve(nodeCount);
  visual.fixed.reserve(nodeCount);
  for (std::size_t i = 0; i < nodeCount; ++i) {
    visual.initialPositions.push_back(body.getPosition(i));
    visual.initialVelocities.push_back(body.getVelocity(i));
    visual.fixed.push_back(body.isFixedNode(i) ? 1u : 0u);
  }
  auto surfaceTriangles = extractSurfaceTriangles(body);

  auto edgeShape = std::make_shared<dynamics::LineSegmentShape>(2.8f);
  for (const auto& position : visual.initialPositions) {
    edgeShape->addVertex(position);
  }
  for (std::size_t i = 0; i < body.getEdgeCount(); ++i) {
    const auto edge = body.getEdge(i);
    edgeShape->addConnection(edge.nodeA, edge.nodeB);
  }

  auto edgeFrame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), std::string(name) + "_edges");
  edgeFrame->setShape(edgeShape);
  edgeFrame->getVisualAspect(true)->setRGBA(rgba(0.08, 0.13, 0.17));

  std::vector<dynamics::SimpleFramePtr> nodeFrames;
  nodeFrames.reserve(visual.initialPositions.size());
  for (std::size_t i = 0; i < visual.initialPositions.size(); ++i) {
    auto frame = dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(),
        std::string(name) + "_node_" + std::to_string(i),
        makeTransform(visual.initialPositions[i]));
    frame->setShape(
        std::make_shared<dynamics::SphereShape>(
            visual.fixed[i] != 0u ? 0.045 : 0.032));
    frame->getVisualAspect(true)->setRGBA(
        visual.fixed[i] != 0u ? rgba(0.95, 0.50, 0.16)
                              : rgba(0.12, 0.57, 0.91));
    nodeFrames.push_back(std::move(frame));
  }

  visual.surfaceRenderableKey = static_cast<int>(state.deformables.size() + 1u);
  visual.surfaceTriangles = std::move(surfaceTriangles);
  visual.edgeShape = std::move(edgeShape);
  visual.edgeFrame = std::move(edgeFrame);
  visual.nodeFrames = std::move(nodeFrames);
  state.deformables.push_back(std::move(visual));
}

//==============================================================================
void addDeformableNet(ExampleState& state)
{
  constexpr int columns = 9;
  constexpr int rows = 5;

  std::vector<sx::DeformableEdge> edges;
  std::vector<std::uint8_t> fixed;
  auto options = makeNetOptions(columns, rows, edges, fixed);
  auto body = state.physicsWorld.addDeformableBody("deformable_net", options);
  appendDeformableVisual(state, body, "deformable_net");
}

//==============================================================================
// A raised static box at the scene center, tagged as a deformable ground
// barrier. The ground barrier uses a finite xy footprint, so it forms a step
// in the support height field: a draping mat is held at the box top over the
// footprint and falls past it to the surrounding ground beyond the edges.
void addObstacleBox(ExampleState& state)
{
  const Eigen::Vector3d halfExtents(0.30, 0.30, 0.12);
  const Eigen::Vector3d center(0.0, 0.0, halfExtents.z());

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = center;
  auto obstacle
      = state.physicsWorld.addRigidBody("drape_obstacle", obstacleOptions);
  obstacle.setCollisionShape(sx::CollisionShape::makeBox(halfExtents));
  sx::DeformableObstaclePolicy obstaclePolicy;
  obstaclePolicy.groundBarrier = true;
  obstacle.setDeformableObstaclePolicy(obstaclePolicy);

  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "drape_obstacle_visual", makeTransform(center));
  frame->setShape(std::make_shared<dynamics::BoxShape>(2.0 * halfExtents));
  frame->getVisualAspect(true)->setRGBA(rgba(0.52, 0.45, 0.34));
  state.staticFrames.push_back(std::move(frame));
}

//==============================================================================
// A free deformable mat suspended flat above the obstacle. Under gravity it
// drapes over the box (conservative surface CCD), folds against itself
// (self-contact barrier), and settles onto the ground (ground barrier) -- a
// showcase for the projected-Newton solve at sparse scale (> 256 nodes).
sx::DeformableBodyOptions makeDrapeOptions(
    int columns, int rows, std::vector<sx::DeformableEdge>& edges)
{
  sx::DeformableBodyOptions options;
  options.edgeStiffness = 25.0; // floppy enough to drape over the step edges
  options.damping = 1.5;

  const auto index = [columns](int col, int row) {
    return static_cast<std::size_t>(row * columns + col);
  };

  constexpr double spacing = 0.05;
  constexpr double dropHeight = 0.42; // above the obstacle top (0.24)
  const double halfWidth = 0.5 * spacing * static_cast<double>(columns - 1);
  const double halfDepth = 0.5 * spacing * static_cast<double>(rows - 1);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < columns; ++col) {
      const double x = spacing * static_cast<double>(col) - halfWidth;
      const double y = spacing * static_cast<double>(row) - halfDepth;
      options.positions.push_back(Eigen::Vector3d(x, y, dropHeight));
      options.velocities.push_back(Eigen::Vector3d::Zero());
      options.masses.push_back(0.05);
    }
  }

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < columns; ++col) {
      if (col + 1 < columns) {
        edges.push_back({index(col, row), index(col + 1, row), -1.0});
      }
      if (row + 1 < rows) {
        edges.push_back({index(col, row), index(col, row + 1), -1.0});
      }
      if (col + 1 < columns && row + 1 < rows) {
        edges.push_back({index(col, row), index(col + 1, row + 1), -1.0});
        edges.push_back({index(col + 1, row), index(col, row + 1), -1.0});
      }
    }
  }
  options.edges = edges;

  for (const auto& triangle : gui::makeGridSurfaceTriangles(columns, rows)) {
    options.surfaceTriangles.push_back(
        {static_cast<std::size_t>(triangle.x()),
         static_cast<std::size_t>(triangle.y()),
         static_cast<std::size_t>(triangle.z())});
  }
  return options;
}

//==============================================================================
void addDeformableDrape(ExampleState& state)
{
  constexpr int columns = 26;
  constexpr int rows = 22; // 572 nodes, past the former dense 256-node cap

  addObstacleBox(state);

  std::vector<sx::DeformableEdge> edges;
  auto options = makeDrapeOptions(columns, rows, edges);
  auto body = state.physicsWorld.addDeformableBody("deformable_drape", options);
  appendDeformableVisual(state, body, "deformable_drape");
}

//==============================================================================
// A contact-free hanging cloth (vertical curtain) for the Vertex Block Descent
// solver: the top row is pinned and the sheet billows out of plane under an
// initial sideways gust, then settles under gravity. No ground or obstacle is
// present, so the World step routes this contact-free, tetrahedron-free
// mass-spring body through the VBD inner solver (graph-colored Gauss-Seidel
// block coordinate descent) instead of the default gradient solver.
sx::DeformableBodyOptions makeVbdClothOptions(
    int columns, int rows, std::vector<sx::DeformableEdge>& edges)
{
  sx::DeformableBodyOptions options;
  options.edgeStiffness = 120.0;
  options.damping = 0.6;

  const auto index = [columns](int col, int row) {
    return static_cast<std::size_t>(row * columns + col);
  };

  constexpr double spacing = 0.06;
  constexpr double topHeight = 1.25;
  const double halfWidth = 0.5 * spacing * static_cast<double>(columns - 1);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < columns; ++col) {
      const double x = spacing * static_cast<double>(col) - halfWidth;
      const double z = topHeight - spacing * static_cast<double>(row);
      // A small out-of-plane billow so the rest shape already reads as cloth.
      const double y = 0.02 * std::sin(1.7 * static_cast<double>(col));
      options.positions.push_back(Eigen::Vector3d(x, y, z));
      // An initial sideways gust, strongest at the free bottom edge, to set the
      // sheet swinging out of plane (purely a visual flourish).
      const double gust
          = 0.9 * static_cast<double>(row) / static_cast<double>(rows - 1);
      options.velocities.push_back(
          Eigen::Vector3d(
              0.0, gust * std::sin(0.6 * static_cast<double>(col)), 0.0));
      options.masses.push_back(0.03);
    }
  }

  // Pin the entire top row so the curtain hangs.
  for (int col = 0; col < columns; ++col) {
    options.fixedNodes.push_back(index(col, 0));
  }

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < columns; ++col) {
      if (col + 1 < columns) {
        edges.push_back({index(col, row), index(col + 1, row), -1.0});
      }
      if (row + 1 < rows) {
        edges.push_back({index(col, row), index(col, row + 1), -1.0});
      }
      if (col + 1 < columns && row + 1 < rows) {
        edges.push_back({index(col, row), index(col + 1, row + 1), -1.0});
        edges.push_back({index(col + 1, row), index(col, row + 1), -1.0});
      }
    }
  }
  options.edges = edges;

  for (const auto& triangle : gui::makeGridSurfaceTriangles(columns, rows)) {
    options.surfaceTriangles.push_back(
        {static_cast<std::size_t>(triangle.x()),
         static_cast<std::size_t>(triangle.y()),
         static_cast<std::size_t>(triangle.z())});
  }
  return options;
}

//==============================================================================
// Select the Vertex Block Descent inner solver for every deformable body in the
// world. The public deformable facade is intentionally solver-agnostic, so the
// selection is made here through the internal opt-in component, mirroring the
// World-solver unit test. The default gradient solver runs when this component
// is absent.
void enableVbdSolver(sx::World& world, std::size_t iterations)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(
        entity, sx::comps::DeformableVbdConfig{true, iterations, 0.0});
  }
}

//==============================================================================
void addVbdCloth(ExampleState& state)
{
  constexpr int columns = 17;
  constexpr int rows = 13; // 221 contact-free mass-spring nodes

  std::vector<sx::DeformableEdge> edges;
  auto options = makeVbdClothOptions(columns, rows, edges);
  auto body = state.physicsWorld.addDeformableBody("vbd_cloth", options);
  appendDeformableVisual(state, body, "vbd_cloth");
}

//==============================================================================
void addLoadedScene(ExampleState& state, const std::filesystem::path& scenePath)
{
  sxio::DeformableSceneLoadOptions loadOptions;
  loadOptions.assetRoot = scenePath.parent_path();
  auto sceneInfo
      = sxio::loadDeformableScene(state.physicsWorld, scenePath, loadOptions);
  for (const auto& bodyInfo : sceneInfo.bodies) {
    appendDeformableVisual(state, bodyInfo.body, bodyInfo.name);
  }
}

//==============================================================================
std::shared_ptr<ExampleState> makeExampleState(const LaunchOptions& launch)
{
  auto state = std::make_shared<ExampleState>();
  state->diagnosticsJsonPath = launch.diagnosticsJsonPath;

  constexpr double timeStep = 1.0 / 120.0;
  state->physicsWorld.setTimeStep(timeStep);

  if (launch.scenePath.empty()) {
    if (launch.sceneKind == "vbd") {
      // No ground/obstacle: the body stays contact-free so the World step
      // routes it through the Vertex Block Descent inner solver.
      addVbdCloth(*state);
      enableVbdSolver(state->physicsWorld, /*iterations=*/20);
    } else {
      addGround(*state);
      if (launch.sceneKind == "drape") {
        addDeformableDrape(*state);
      } else {
        addDeformableNet(*state);
      }
    }
  } else {
    addLoadedScene(*state, launch.scenePath);
  }

  state->physicsWorld.enterSimulationMode();
  state->syncRenderFrames();
  state->writeDiagnostics();
  return state;
}

//==============================================================================
bool applyDeformableViewMode(
    const std::shared_ptr<ExampleState>& state, std::string_view value)
{
  if (value == "combined") {
    state->showSurfaceMesh = true;
    state->showPointMasses = true;
    state->showSpringEdges = true;
  } else if (value == "surface") {
    state->showSurfaceMesh = true;
    state->showPointMasses = false;
    state->showSpringEdges = false;
  } else if (value == "points") {
    state->showSurfaceMesh = false;
    state->showPointMasses = true;
    state->showSpringEdges = false;
  } else {
    return false;
  }

  state->applyVisualOptions();
  return true;
}

//==============================================================================
gui::OrbitCamera makeCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.45);
  camera.yaw = -0.74;
  camera.pitch = 0.26;
  camera.distance = 2.8;
  return camera;
}

//==============================================================================
std::vector<gui::KeyboardAction> createKeyboardActions(
    const std::shared_ptr<ExampleState>& state)
{
  gui::KeyboardAction reset;
  reset.label = "reset scene";
  reset.shortcut = gui::KeyboardShortcut::characterKey('r');
  reset.callback = [state](gui::KeyboardActionContext&) {
    state->reset();
  };
  return {std::move(reset)};
}

//==============================================================================
gui::Panel createControlsPanel(const std::shared_ptr<ExampleState>& state)
{
  gui::Panel panel;
  panel.title = "Deformable Body";
  panel.build = [state](gui::PanelBuilder& builder) {
    if (builder.button("Reset Scene")) {
      state->reset();
    }
    builder.separator();
    if (builder.checkbox("Surface Mesh", state->showSurfaceMesh)) {
      state->applyVisualOptions();
    }
    if (builder.checkbox("Point Masses", state->showPointMasses)) {
      state->applyVisualOptions();
    }
    if (builder.checkbox("Spring Edges", state->showSpringEdges)) {
      state->applyVisualOptions();
    }
  };
  return panel;
}

//==============================================================================
gui::OrbitCamera makeVbdCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.8);
  camera.yaw = -0.9;
  camera.pitch = 0.12;
  camera.distance = 2.4;
  return camera;
}

//==============================================================================
gui::Panel createVbdControlsPanel(const std::shared_ptr<ExampleState>& state)
{
  gui::Panel panel;
  panel.title = "Deformable VBD";
  panel.build = [state](gui::PanelBuilder& builder) {
    builder.text("Inner solver: Vertex Block Descent");
    builder.text("graph-colored Gauss-Seidel block descent");
    builder.separator();
    if (builder.button("Reset Scene")) {
      state->reset();
    }
    builder.separator();
    if (builder.checkbox("Surface Mesh", state->showSurfaceMesh)) {
      state->applyVisualOptions();
    }
    if (builder.checkbox("Point Masses", state->showPointMasses)) {
      state->applyVisualOptions();
    }
    if (builder.checkbox("Spring Edges", state->showSpringEdges)) {
      state->applyVisualOptions();
    }
  };
  return panel;
}

} // namespace

dart::gui::ApplicationOptions makeDeformableBodyScene()
{
  // dart-demos scene: built-in net scene, default view, no diagnostics output.
  // The CLI flags the standalone tool exposed (--deformable-scene,
  // --deformable-scene-kind, --deformable-view, --diagnostics-json) live in
  // the standalone tool's evolution history and are not surfaced here.
  LaunchOptions launch;
  const auto state = makeExampleState(launch);
  (void)applyDeformableViewMode(state, launch.deformableView);

  gui::ApplicationOptions options;
  options.camera = makeCamera();
  options.advanceSimulation = false;
  options.preStep = [state]() {
    state->step();
  };
  options.panels.push_back(createControlsPanel(state));
  options.panels.push_back(createMemoryDiagnosticsPanel(
      "Deformable Body", [state]() { return &state->physicsWorld; }));
  options.keyboardActions = createKeyboardActions(state);
  options.renderableProvider = [state]() {
    state->syncRenderFrames();
    return state->makeRenderables();
  };
  return options;
}

dart::gui::ApplicationOptions makeVbdDeformableScene()
{
  // dart-demos scene: a contact-free hanging cloth driven by the Vertex Block
  // Descent inner solver (selected via the internal opt-in component, so the
  // public deformable facade stays solver-agnostic).
  LaunchOptions launch;
  launch.sceneKind = "vbd";
  const auto state = makeExampleState(launch);
  (void)applyDeformableViewMode(state, launch.deformableView);

  gui::ApplicationOptions options;
  options.camera = makeVbdCamera();
  options.advanceSimulation = false;
  options.preStep = [state]() {
    state->step();
  };
  options.panels.push_back(createVbdControlsPanel(state));
  options.panels.push_back(createMemoryDiagnosticsPanel(
      "Deformable VBD", [state]() { return &state->physicsWorld; }));
  options.keyboardActions = createKeyboardActions(state);
  options.renderableProvider = [state]() {
    state->syncRenderFrames();
    return state->makeRenderables();
  };
  return options;
}

} // namespace dart::examples::demos
