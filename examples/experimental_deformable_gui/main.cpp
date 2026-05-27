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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/renderable.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace legacy_sim = dart::simulation;
namespace sx = dart::simulation::experimental;

namespace {

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
struct ExampleState
{
  sx::World physicsWorld;
  legacy_sim::WorldPtr renderWorld
      = legacy_sim::World::create("experimental_deformable_gui");
  DeformableVisual deformable;
  int stepsPerFrame = 2;
  bool showSurfaceMesh = true;
  bool showPointMasses = true;
  bool showSpringEdges = true;
  std::size_t surfaceVersion = 1;

  void applyVisualOptions()
  {
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

  void syncRenderFrames()
  {
    const auto nodeCount = deformable.body.getNodeCount();
    for (std::size_t i = 0; i < nodeCount; ++i) {
      const auto position = deformable.body.getPosition(i);
      deformable.edgeShape->setVertex(i, position);
      deformable.nodeFrames[i]->setTransform(makeTransform(position));
    }
    ++surfaceVersion;
    applyVisualOptions();
  }

  void step()
  {
    for (int i = 0; i < stepsPerFrame; ++i) {
      physicsWorld.step();
    }
    renderWorld->setTime(physicsWorld.getTime());
    syncRenderFrames();
  }

  void reset()
  {
    physicsWorld.setTime(0.0);
    renderWorld->reset();
    renderWorld->setTime(0.0);

    for (std::size_t i = 0; i < deformable.initialPositions.size(); ++i) {
      deformable.body.setPosition(i, deformable.initialPositions[i]);
      deformable.body.setVelocity(i, deformable.initialVelocities[i]);
    }

    syncRenderFrames();
  }

  std::vector<gui::RenderableDescriptor> makeSurfaceRenderables() const
  {
    if (!showSurfaceMesh || deformable.surfaceTriangles.empty()) {
      return {};
    }

    const auto nodeCount = deformable.body.getNodeCount();
    if (nodeCount == 0) {
      return {};
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
    if (!descriptor) {
      return {};
    }
    return {*descriptor};
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
  ground.setDeformableGroundBarrier(true);

  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(),
      "ground_visual",
      makeTransform({0.0, 0.0, -0.04}));
  frame->setShape(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(4.8, 2.4, 0.08)));
  frame->getVisualAspect(true)->setRGBA(rgba(0.37, 0.40, 0.43));
  state.renderWorld->addSimpleFrame(frame);
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
void addDeformableNet(ExampleState& state)
{
  constexpr int columns = 9;
  constexpr int rows = 5;

  std::vector<sx::DeformableEdge> edges;
  std::vector<std::uint8_t> fixed;
  auto options = makeNetOptions(columns, rows, edges, fixed);
  auto body = state.physicsWorld.addDeformableBody("deformable_net", options);
  auto surfaceTriangles = extractSurfaceTriangles(body);

  auto edgeShape = std::make_shared<dynamics::LineSegmentShape>(2.8f);
  for (const auto& position : options.positions) {
    edgeShape->addVertex(position);
  }
  for (const auto& edge : edges) {
    edgeShape->addConnection(edge.nodeA, edge.nodeB);
  }

  auto edgeFrame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "deformable_net_edges");
  edgeFrame->setShape(edgeShape);
  edgeFrame->getVisualAspect(true)->setRGBA(rgba(0.08, 0.13, 0.17));
  state.renderWorld->addSimpleFrame(edgeFrame);

  std::vector<dynamics::SimpleFramePtr> nodeFrames;
  nodeFrames.reserve(options.positions.size());
  for (std::size_t i = 0; i < options.positions.size(); ++i) {
    auto frame = dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(),
        "deformable_node_" + std::to_string(i),
        makeTransform(options.positions[i]));
    frame->setShape(
        std::make_shared<dynamics::SphereShape>(
            fixed[i] != 0u ? 0.045 : 0.032));
    frame->getVisualAspect(true)->setRGBA(
        fixed[i] != 0u ? rgba(0.95, 0.50, 0.16) : rgba(0.12, 0.57, 0.91));
    state.renderWorld->addSimpleFrame(frame);
    nodeFrames.push_back(std::move(frame));
  }

  state.deformable.body = body;
  state.deformable.initialPositions = std::move(options.positions);
  state.deformable.initialVelocities = std::move(options.velocities);
  state.deformable.fixed = std::move(fixed);
  state.deformable.surfaceTriangles = std::move(surfaceTriangles);
  state.deformable.edgeShape = std::move(edgeShape);
  state.deformable.edgeFrame = std::move(edgeFrame);
  state.deformable.nodeFrames = std::move(nodeFrames);
}

//==============================================================================
std::shared_ptr<ExampleState> makeExampleState()
{
  auto state = std::make_shared<ExampleState>();

  constexpr double timeStep = 1.0 / 120.0;
  state->physicsWorld.setTimeStep(timeStep);
  state->renderWorld->setTimeStep(timeStep * state->stepsPerFrame);

  addGround(*state);
  addDeformableNet(*state);

  state->physicsWorld.enterSimulationMode();
  state->syncRenderFrames();
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
    state->showSpringEdges = true;
  } else {
    return false;
  }

  state->applyVisualOptions();
  return true;
}

//==============================================================================
bool parseExampleOptions(
    int argc, char* argv[], const std::shared_ptr<ExampleState>& state)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--deformable-view") {
      if (i + 1 >= argc) {
        std::cerr
            << "--deformable-view requires combined, surface, or points\n";
        return false;
      }
      const std::string_view value(argv[++i]);
      if (!applyDeformableViewMode(state, value)) {
        std::cerr << "Invalid --deformable-view value '" << value
                  << "'. Expected combined, surface, or points.\n";
        return false;
      }
    }
  }
  return true;
}

//==============================================================================
gui::RunOptions makeRunDefaults()
{
  gui::RunOptions options;
  options.windowTitle = "DART Experimental Deformable Body";
  options.width = 1280;
  options.height = 720;
  return options;
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

} // namespace

int main(int argc, char* argv[])
{
  const auto state = makeExampleState();
  if (!parseExampleOptions(argc, argv, state)) {
    return 2;
  }

  gui::ApplicationOptions options;
  options.world = state->renderWorld;
  options.runDefaults = makeRunDefaults();
  options.camera = makeCamera();
  options.simulateWorld = false;
  options.preStep = [state]() {
    state->step();
  };
  options.panels.push_back(createControlsPanel(state));
  options.keyboardActions = createKeyboardActions(state);
  options.renderableProvider = [state]() {
    return state->makeSurfaceRenderables();
  };

  return gui::runApplication(argc, argv, options);
}
