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

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/renderable.hpp>

#include <dart/simulation/world.hpp>

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/object_manager.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/selection_manager.hpp>
#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_engine/simulation_controller.hpp>
#include <dartsim_ui/editor.hpp>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace dartsim::ui {

namespace {

/// Per-application UI state shared by all panel callbacks.
struct EditorApp
{
  SimEngine engine;
  double replayFrame = 0.0;
  bool recording = false;

  void note(const std::string& message)
  {
    engine.logger().info(message);
  }
};

const char* typeLabel(ObjectType type)
{
  switch (type) {
    case ObjectType::RigidBody:
      return "RigidBody";
    case ObjectType::MultiBody:
      return "MultiBody";
    case ObjectType::Link:
      return "Link";
    case ObjectType::Joint:
      return "Joint";
    case ObjectType::FreeFrame:
      return "FreeFrame";
    case ObjectType::FixedFrame:
      return "FixedFrame";
  }
  return "Object";
}

Eigen::Isometry3d makeTranslation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

/// Convert engine render items into dart-gui renderable descriptors so the
/// viewport draws the experimental scene (via ApplicationOptions::
/// renderableProvider). Dynamics pointers stay null; only geometry, material,
/// and transform are needed.
std::vector<dart::gui::RenderableDescriptor> toDescriptors(
    const std::vector<RenderItem>& items)
{
  std::vector<dart::gui::RenderableDescriptor> descriptors;
  descriptors.reserve(items.size());
  for (const RenderItem& item : items) {
    dart::gui::RenderableDescriptor d;
    d.id = static_cast<dart::gui::RenderableId>(item.id);
    d.worldTransform = item.worldTransform;
    d.material.rgba = item.color;
    dart::gui::GeometryDescriptor& g = d.geometry;
    switch (item.shape) {
      case ShapeType::Box:
        g.kind = dart::gui::ShapeKind::Box;
        g.size = item.dimensions;
        break;
      case ShapeType::Sphere:
        g.kind = dart::gui::ShapeKind::Sphere;
        g.radius = item.dimensions.x();
        break;
      case ShapeType::Cylinder:
        g.kind = dart::gui::ShapeKind::Cylinder;
        g.radius = item.dimensions.x();
        g.height = item.dimensions.y();
        break;
      case ShapeType::Capsule:
        g.kind = dart::gui::ShapeKind::Capsule;
        g.radius = item.dimensions.x();
        g.height = item.dimensions.y();
        break;
      case ShapeType::Plane:
        g.kind = dart::gui::ShapeKind::Plane;
        g.normal = item.dimensions;
        g.offset = 0.0;
        break;
    }
    descriptors.push_back(std::move(d));
  }
  return descriptors;
}

void buildTreeNode(
    dart::gui::PanelBuilder& ui, EditorApp& app, ObjectId id, int depth)
{
  const SceneObject* object = app.engine.objects().model().find(id);
  if (object == nullptr) {
    return;
  }
  std::string indent(static_cast<std::size_t>(depth) * 2u, ' ');
  const bool selected = app.engine.selection().isSelected(id);
  std::string label = indent + (selected ? "* " : "  ") + object->name + " ["
                      + typeLabel(object->type) + "]##" + std::to_string(id);
  if (ui.button(label)) {
    app.engine.selection().select(id);
  }
  for (const ObjectId child : app.engine.objects().model().childrenOf(id)) {
    buildTreeNode(ui, app, child, depth + 1);
  }
}

void buildSceneTree(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  ui.text("Scene Tree");
  ui.separator();
  const SceneModel& model = app.engine.objects().model();
  if (model.empty()) {
    ui.text("(empty world)");
    return;
  }
  for (const ObjectId id : model.rootChildren()) {
    buildTreeNode(ui, app, id, 0);
  }
}

void buildInspector(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  ui.text("Inspector");
  ui.separator();
  const ObjectId id = app.engine.selection().primary();
  const SceneObject* object = app.engine.objects().model().find(id);
  if (object == nullptr) {
    ui.text("(no selection)");
    return;
  }
  ui.text(std::string("Name: ") + object->name);
  ui.text(std::string("Type: ") + typeLabel(object->type));

  if (object->type == ObjectType::RigidBody) {
    Eigen::Vector3d position = object->transform.translation();
    double x = position.x();
    double y = position.y();
    double z = position.z();
    bool moved = false;
    moved |= ui.slider("x", x, -10.0, 10.0);
    moved |= ui.slider("y", y, -10.0, 10.0);
    moved |= ui.slider("z", z, -10.0, 10.0);
    if (moved) {
      app.engine.execute(commands::setTransform(id, makeTranslation(x, y, z)));
    }
    double mass = object->mass;
    if (ui.slider("mass", mass, 0.01, 100.0)) {
      app.engine.execute(commands::setMass(id, mass));
    }
  } else if (
      object->type == ObjectType::Link && object->parentLink != kNoObject) {
    double q = object->jointPosition;
    if (ui.slider("joint", q, -3.14159, 3.14159)) {
      app.engine.execute(commands::setJointPosition(id, q));
    }
  }

  ui.separator();
  if (ui.button("Delete##inspector")) {
    app.engine.execute(commands::removeObject(id));
    app.note("Deleted object");
  }
}

void buildConsole(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  ui.text("Console");
  ui.separator();
  const auto& entries = app.engine.logger().entries();
  const std::size_t count = entries.size();
  const std::size_t start = count > 14 ? count - 14 : 0;
  for (std::size_t i = start; i < count; ++i) {
    ui.text(entries[i].message);
  }
}

void buildSimControls(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  SimulationController& sim = app.engine.simulation();

  // Advance the running simulation once per UI frame (~60 Hz wall clock).
  if (sim.isRunning()) {
    sim.advance(1.0 / 60.0);
  }

  ui.text(
      sim.mode() == SimulationController::Mode::Run ? "Mode: Run"
                                                    : "Mode: Edit");
  ui.text("Sim time: " + std::to_string(sim.simTime()));
  ui.text("Frame: " + std::to_string(sim.frameCount()));
  ui.separator();

  if (ui.button("Play")) {
    sim.play();
    app.note("Play");
  }
  ui.sameLine();
  if (ui.button("Pause")) {
    sim.pause();
    app.note("Pause");
  }
  ui.sameLine();
  if (ui.button("Step")) {
    sim.step(1);
    app.note("Step");
  }
  ui.sameLine();
  if (ui.button("Reset")) {
    sim.reset();
    app.note("Reset");
  }

  ui.separator();
  bool record = app.recording;
  if (ui.checkbox("Record", record)) {
    app.recording = record;
    if (record) {
      app.engine.startRecording();
      app.note("Recording started");
    } else {
      app.engine.stopRecording();
      app.engine.loadRecordingIntoPlayer();
      app.note("Recording stopped");
    }
  }

  const std::size_t frames = app.engine.player().frameCount();
  if (frames > 0) {
    ui.text("Replay frames: " + std::to_string(frames));
    double maxIndex = static_cast<double>(frames - 1);
    if (app.replayFrame > maxIndex) {
      app.replayFrame = maxIndex;
    }
    if (ui.slider("timeline", app.replayFrame, 0.0, maxIndex)) {
      app.engine.replaySeek(static_cast<std::size_t>(app.replayFrame + 0.5));
    }
  }
}

void buildMenuBar(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  if (!ui.beginMenuBar()) {
    ui.text("dartsim editor");
    return;
  }
  if (ui.beginMenu("File")) {
    if (ui.menuItem("Save Project")) {
      app.note(
          app.engine.saveProject("scene.dartsim") ? "Saved scene.dartsim"
                                                  : "Save failed");
    }
    if (ui.menuItem("Open Project")) {
      app.note(
          app.engine.loadProject("scene.dartsim") ? "Loaded scene.dartsim"
                                                  : "Open failed");
    }
    ui.endMenu();
  }
  if (ui.beginMenu("Edit")) {
    if (ui.menuItem("Undo")) {
      app.note(app.engine.undo() ? "Undo" : "Nothing to undo");
    }
    if (ui.menuItem("Redo")) {
      app.note(app.engine.redo() ? "Redo" : "Nothing to redo");
    }
    ui.endMenu();
  }
  if (ui.beginMenu("Create")) {
    if (ui.menuItem("Rigid Body (Box)")) {
      app.engine.execute(
          commands::addRigidBody(ShapeType::Box, makeTranslation(0, 0, 1)));
      app.note("Added box");
    }
    if (ui.menuItem("Rigid Body (Sphere)")) {
      app.engine.execute(
          commands::addRigidBody(ShapeType::Sphere, makeTranslation(0, 0, 1)));
      app.note("Added sphere");
    }
    if (ui.menuItem("MultiBody")) {
      app.engine.execute(commands::addMultiBody());
      app.note("Added multibody");
    }
    ui.endMenu();
  }
  if (ui.beginMenu("Simulate")) {
    if (ui.menuItem("Play")) {
      app.engine.simulation().play();
    }
    if (ui.menuItem("Pause")) {
      app.engine.simulation().pause();
    }
    if (ui.menuItem("Step")) {
      app.engine.simulation().step(1);
    }
    if (ui.menuItem("Reset")) {
      app.engine.simulation().reset();
    }
    ui.endMenu();
  }
  ui.endMenuBar();
}

void seedDemoScene(EditorApp& app)
{
  app.engine.execute(
      commands::addRigidBody(ShapeType::Box, makeTranslation(0.0, 0.0, 0.5)));
  app.engine.execute(
      commands::addRigidBody(
          ShapeType::Sphere, makeTranslation(1.0, 0.0, 0.5)));
  app.engine.execute(commands::addMultiBody("arm"));
  const ObjectId arm = app.engine.selection().primary();
  app.engine.execute(commands::addLink(arm));
  const ObjectId base = app.engine.selection().primary();
  app.engine.execute(commands::addLink(arm, base, JointKind::Revolute));
  app.engine.commands().clearHistory();
  app.note("dartsim editor ready");
}

dart::gui::Panel makePanel(
    std::string title,
    bool menuBar,
    std::array<double, 2> position,
    std::array<double, 2> size,
    std::function<void(dart::gui::PanelBuilder&)> build)
{
  dart::gui::Panel panel;
  panel.title = std::move(title);
  panel.menuBar = menuBar;
  panel.initialPosition = position;
  panel.initialSize = size;
  panel.autoResize = false;
  panel.build = std::move(build);
  return panel;
}

} // namespace

int runEditor(int argc, char* argv[])
{
  auto app = std::make_shared<EditorApp>();
  seedDemoScene(*app);

  dart::gui::ApplicationOptions options;
  // Empty legacy world as a render canvas; the experimental engine owns the
  // actual scene and is drawn through the renderable provider below. The legacy
  // world is never stepped (simulateWorld = false).
  options.world = dart::simulation::World::create("dartsim_editor");
  options.simulateWorld = false;
  options.dockingEnabled = true;
  options.renderableProvider = [app]() {
    return toDescriptors(app->engine.renderItems());
  };

  options.panels.push_back(makePanel(
      "Menu",
      true,
      {0.0, 0.0},
      {1280.0, 0.0},
      [app](dart::gui::PanelBuilder& ui) { buildMenuBar(ui, *app); }));
  options.panels.push_back(makePanel(
      "Scene Tree",
      false,
      {0.0, 30.0},
      {280.0, 360.0},
      [app](dart::gui::PanelBuilder& ui) { buildSceneTree(ui, *app); }));
  options.panels.push_back(makePanel(
      "Inspector",
      false,
      {1000.0, 30.0},
      {280.0, 360.0},
      [app](dart::gui::PanelBuilder& ui) { buildInspector(ui, *app); }));
  options.panels.push_back(makePanel(
      "Simulation",
      false,
      {0.0, 400.0},
      {280.0, 220.0},
      [app](dart::gui::PanelBuilder& ui) { buildSimControls(ui, *app); }));
  options.panels.push_back(makePanel(
      "Console",
      false,
      {300.0, 520.0},
      {680.0, 180.0},
      [app](dart::gui::PanelBuilder& ui) { buildConsole(ui, *app); }));

  return dart::gui::runApplication(argc, argv, options);
}

} // namespace dartsim::ui
