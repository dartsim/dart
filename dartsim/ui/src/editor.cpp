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
#include <dartsim_ui/inspector_actions.hpp>
#include <dartsim_ui/outliner_actions.hpp>
#include <dartsim_ui/palette_actions.hpp>
#include <dartsim_ui/project_actions.hpp>
#include <dartsim_ui/project_file_dialog.hpp>
#include <dartsim_ui/relationship_actions.hpp>
#include <dartsim_ui/simulation_actions.hpp>
#include <dartsim_ui/viewport_actions.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
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
  std::chrono::steady_clock::time_point lastRunFrameTime;
  bool haveRunFrameTime = false;
  OutlinerState outliner;
  ViewportTransformGizmo transformGizmo = makeViewportTransformGizmo();
  ProjectFileDialogKind projectPathKind = ProjectFileDialogKind::Open;
  bool projectPathModalOpen = false;
  bool projectPathModalRequested = false;
  std::string projectPath = kDefaultProjectPath;
  std::string projectPathStatus;
  std::filesystem::path projectBrowserDirectory;
  std::string projectBrowserStatus;
  void* projectDialogParentWindow = nullptr;

  void note(const std::string& message)
  {
    engine.logger().info(message);
  }

  double consumeRunFrameSeconds()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!haveRunFrameTime) {
      lastRunFrameTime = now;
      haveRunFrameTime = true;
      return 0.0;
    }

    const double elapsed
        = std::chrono::duration<double>(now - lastRunFrameTime).count();
    lastRunFrameTime = now;
    return elapsed;
  }

  void resetRunFrameClock()
  {
    haveRunFrameTime = false;
  }
};

std::string ensureProjectExtension(std::string path)
{
  if (std::filesystem::path(path).extension().empty()) {
    path += ".dartsim";
  }
  return path;
}

std::string parentPathOrEmpty(const std::string& path)
{
  if (path.empty()) {
    return {};
  }
  return std::filesystem::path(path).parent_path().string();
}

std::string filenameOrDefault(const std::string& path)
{
  if (path.empty()) {
    return kDefaultProjectPath;
  }
  const std::string filename = std::filesystem::path(path).filename().string();
  return filename.empty() ? kDefaultProjectPath : filename;
}

std::filesystem::path currentDirectory()
{
  std::error_code error;
  std::filesystem::path path = std::filesystem::current_path(error);
  return error ? std::filesystem::path(".") : path;
}

std::filesystem::path absolutePath(std::filesystem::path path)
{
  std::error_code error;
  std::filesystem::path absolute = std::filesystem::absolute(path, error);
  return error ? std::move(path) : absolute.lexically_normal();
}

std::filesystem::path projectBrowserDirectoryFor(const std::string& path)
{
  std::filesystem::path candidate(path);
  if (candidate.empty()) {
    candidate = currentDirectory();
  }

  std::error_code error;
  if (!std::filesystem::is_directory(candidate, error)) {
    candidate = candidate.parent_path();
  }
  if (candidate.empty()) {
    candidate = currentDirectory();
  }

  candidate = absolutePath(candidate);
  if (!std::filesystem::is_directory(candidate, error)) {
    return absolutePath(currentDirectory());
  }
  return candidate;
}

struct ProjectBrowserEntry
{
  std::filesystem::path path;
  std::string label;
  bool directory = false;
};

std::vector<ProjectBrowserEntry> projectBrowserEntries(
    const std::filesystem::path& directory, std::string& status)
{
  status.clear();
  std::vector<ProjectBrowserEntry> entries;

  std::error_code error;
  std::filesystem::directory_iterator it(directory, error);
  if (error) {
    status = "Cannot read folder: " + error.message();
    return entries;
  }

  const std::filesystem::directory_iterator end;
  for (; it != end; it.increment(error)) {
    if (error) {
      status = "Cannot read folder: " + error.message();
      break;
    }
    const std::filesystem::directory_entry& entry = *it;
    std::error_code entryError;
    const bool directoryEntry = entry.is_directory(entryError);
    if (entryError) {
      continue;
    }

    const std::filesystem::path path = entry.path();
    if (!directoryEntry && path.extension() != ".dartsim") {
      continue;
    }

    ProjectBrowserEntry browserEntry;
    browserEntry.path = path;
    browserEntry.directory = directoryEntry;
    browserEntry.label = directoryEntry ? "[dir] " : "";
    browserEntry.label += path.filename().string();
    entries.push_back(std::move(browserEntry));
  }

  std::sort(
      entries.begin(),
      entries.end(),
      [](const ProjectBrowserEntry& lhs, const ProjectBrowserEntry& rhs) {
        if (lhs.directory != rhs.directory) {
          return lhs.directory;
        }
        return lhs.label < rhs.label;
      });

  constexpr std::size_t kMaxProjectBrowserEntries = 48;
  if (entries.size() > kMaxProjectBrowserEntries) {
    entries.resize(kMaxProjectBrowserEntries);
    status = "Showing first 48 project entries";
  }

  return entries;
}

std::string projectPathOrDefault(const SimEngine& engine)
{
  return engine.hasProjectPath() ? engine.projectPath() : kDefaultProjectPath;
}

std::string projectPathModalTitle(ProjectFileDialogKind kind)
{
  return kind == ProjectFileDialogKind::Open ? "Open Project"
                                             : "Save Project As";
}

std::string projectDialogFailureMessage(
    ProjectFileDialogKind kind, const ProjectFileDialogResult& result)
{
  std::string message = projectPathModalTitle(kind) + " dialog failed";
  if (!result.error.empty()) {
    message += ": ";
    message += result.error;
  }
  return message;
}

ProjectFileDialogRequest makeProjectFileDialogRequest(
    const EditorApp& app, ProjectFileDialogKind kind)
{
  const std::string path = projectPathOrDefault(app.engine);
  ProjectFileDialogRequest request;
  request.kind = kind;
  request.defaultPath = parentPathOrEmpty(path);
  request.defaultName = filenameOrDefault(path);
  request.parentNativeWindow = app.projectDialogParentWindow;
  return request;
}

void requestProjectPathModal(
    EditorApp& app,
    ProjectFileDialogKind kind,
    std::string status = {},
    std::string path = {})
{
  app.projectPathKind = kind;
  app.projectPath
      = path.empty() ? projectPathOrDefault(app.engine) : std::move(path);
  app.projectPathStatus = std::move(status);
  app.projectBrowserDirectory = projectBrowserDirectoryFor(app.projectPath);
  app.projectBrowserStatus.clear();
  app.projectPathModalOpen = true;
  app.projectPathModalRequested = true;
}

void browseProjectPath(EditorApp& app)
{
  ProjectFileDialogRequest request
      = makeProjectFileDialogRequest(app, app.projectPathKind);
  request.defaultPath = parentPathOrEmpty(app.projectPath);
  request.defaultName = filenameOrDefault(app.projectPath);

  const ProjectFileDialogResult selection = nativeProjectFileDialog(request);
  switch (selection.status) {
    case ProjectFileDialogStatus::Selected:
      if (selection.path.empty()) {
        app.projectPathStatus
            = projectPathModalTitle(app.projectPathKind) + " canceled";
      } else {
        app.projectPath = selection.path;
        app.projectBrowserDirectory
            = projectBrowserDirectoryFor(app.projectPath);
        app.projectBrowserStatus.clear();
        app.projectPathStatus.clear();
      }
      break;
    case ProjectFileDialogStatus::Canceled:
      app.projectPathStatus
          = projectPathModalTitle(app.projectPathKind) + " canceled";
      break;
    case ProjectFileDialogStatus::Failed:
      app.projectPathStatus
          = projectDialogFailureMessage(app.projectPathKind, selection);
      break;
  }
}

void applyProjectPathModal(EditorApp& app)
{
  ProjectActionResult result;
  if (app.projectPathKind == ProjectFileDialogKind::Open) {
    result = openProject(app.engine, app.projectPath);
  } else {
    result = saveProjectAs(app.engine, ensureProjectExtension(app.projectPath));
  }
  app.note(result.message);
  app.projectPathStatus = result.message;
  if (result.ok) {
    app.projectPathModalOpen = false;
  }
}

void openProjectFromNativeDialog(EditorApp& app)
{
  if (app.engine.isProjectDirty()) {
    app.note("Unsaved changes");
    return;
  }

  const ProjectFileDialogResult selection = nativeProjectFileDialog(
      makeProjectFileDialogRequest(app, ProjectFileDialogKind::Open));
  switch (selection.status) {
    case ProjectFileDialogStatus::Selected:
      if (selection.path.empty()) {
        app.note("Open canceled");
        return;
      }
      {
        const ProjectActionResult result
            = openProject(app.engine, selection.path);
        app.note(result.message);
        if (!result.ok) {
          requestProjectPathModal(
              app, ProjectFileDialogKind::Open, result.message, selection.path);
        }
      }
      return;
    case ProjectFileDialogStatus::Canceled:
      app.note("Open canceled");
      return;
    case ProjectFileDialogStatus::Failed: {
      const std::string message
          = projectDialogFailureMessage(ProjectFileDialogKind::Open, selection);
      app.note(message);
      requestProjectPathModal(app, ProjectFileDialogKind::Open, message);
      return;
    }
  }
}

void saveProjectFromNativeDialog(EditorApp& app)
{
  const ProjectFileDialogResult selection = nativeProjectFileDialog(
      makeProjectFileDialogRequest(app, ProjectFileDialogKind::Save));
  switch (selection.status) {
    case ProjectFileDialogStatus::Selected:
      if (selection.path.empty()) {
        app.note("Save canceled");
        return;
      }
      {
        const std::string path = ensureProjectExtension(selection.path);
        const ProjectActionResult result = saveProjectAs(app.engine, path);
        app.note(result.message);
        if (!result.ok) {
          requestProjectPathModal(
              app, ProjectFileDialogKind::Save, result.message, selection.path);
        }
      }
      return;
    case ProjectFileDialogStatus::Canceled:
      app.note("Save canceled");
      return;
    case ProjectFileDialogStatus::Failed: {
      const std::string message
          = projectDialogFailureMessage(ProjectFileDialogKind::Save, selection);
      app.note(message);
      requestProjectPathModal(app, ProjectFileDialogKind::Save, message);
      return;
    }
  }
}

std::string primarySelectionName(const SimEngine& engine)
{
  const SceneObject* object
      = engine.objects().model().find(engine.selection().primary());
  return object == nullptr ? std::string() : object->name;
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

void buildSceneTree(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  const std::vector<OutlinerRow> rows
      = buildOutlinerRows(app.engine, app.outliner);
  if (rows.empty()) {
    ui.text("(empty world)");
    return;
  }
  for (const OutlinerRow& row : rows) {
    const std::string expansionLabel = outlinerExpansionButtonLabel(row);
    if (!expansionLabel.empty()) {
      if (ui.button(expansionLabel)) {
        toggleOutlinerExpanded(app.outliner, app.engine, row.id);
      }
      ui.sameLine();
    }

    bool visible = row.visible;
    if (ui.checkbox("##visible-" + std::to_string(row.id), visible)) {
      if (setOutlinerVisibility(app.engine, row.id, visible)) {
        app.note((visible ? "Shown " : "Hidden ") + row.name);
      }
    }
    ui.sameLine();

    if (row.renaming) {
      std::string draft = app.outliner.renameDraft;
      if (ui.textInput("##rename-" + std::to_string(row.id), draft)) {
        updateOutlinerRenameDraft(app.outliner, draft);
      }
      ui.sameLine();
      if (ui.button("OK##rename-" + std::to_string(row.id))) {
        app.note(commitOutlinerRename(app.engine, app.outliner).message);
      }
      ui.sameLine();
      if (ui.button("Cancel##rename-" + std::to_string(row.id))) {
        app.note(cancelOutlinerRename(app.outliner).message);
      }
    } else {
      if (ui.button(outlinerButtonLabel(row))) {
        selectOutlinerObject(app.engine, row.id);
        syncViewportTransformGizmo(app.transformGizmo, app.engine);
      }
      if (row.selected) {
        for (const OutlinerContextAction& action :
             buildOutlinerContextActions(app.engine, row.id)) {
          if (action.kind == OutlinerContextActionKind::Show
              || action.kind == OutlinerContextActionKind::Hide) {
            continue;
          }
          ui.sameLine();
          const std::string label
              = action.label + "##outliner-context-" + std::to_string(row.id)
                + "-" + std::to_string(static_cast<int>(action.kind));
          if (ui.button(label)) {
            app.note(applyOutlinerContextAction(
                         app.engine, app.outliner, row.id, action.kind)
                         .message);
            syncViewportTransformGizmo(app.transformGizmo, app.engine);
          }
        }
      }
    }
  }
}

void buildInspector(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  const InspectorStatus status = buildInspectorStatus(app.engine);
  if (!status.hasSelection) {
    ui.text("(no selection)");
    return;
  }
  ui.text("Name: " + status.name);
  ui.text("Type: " + status.type);

  if (status.locked) {
    ui.text("Inspector locked during Simulation Mode");
    return;
  }

  for (const InspectorEnumProperty& property : status.enumProperties) {
    std::vector<std::string_view> choices;
    choices.reserve(property.choices.size());
    int selectedIndex = 0;
    for (std::size_t i = 0; i < property.choices.size(); ++i) {
      choices.push_back(property.choices[i].label);
      if (property.choices[i].value == property.value) {
        selectedIndex = static_cast<int>(i);
      }
    }
    if (ui.select(property.label, selectedIndex, choices)) {
      const std::size_t choiceIndex = static_cast<std::size_t>(selectedIndex);
      if (choiceIndex < property.choices.size()) {
        app.note(
            setInspectorEnumProperty(
                app.engine, property.kind, property.choices[choiceIndex].value)
                .message);
      }
    }
  }

  for (const InspectorNumericProperty& property : status.numericProperties) {
    double value = property.value;
    if (ui.slider(property.label, value, property.minimum, property.maximum)) {
      app.note(setInspectorNumericProperty(app.engine, property.kind, value)
                   .message);
    }
  }
  if (status.colorProperty.has_value()) {
    Eigen::Vector4d color = status.colorProperty->rgba;
    if (ui.colorEdit(status.colorProperty->label, color)) {
      app.note(setInspectorShapeColor(app.engine, color).message);
    }
  }

  ui.separator();
  if (status.canDelete && ui.button("Delete##inspector")) {
    app.note(deleteInspectorSelection(app.engine).message);
  }
}

void buildConsole(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  const auto& entries = app.engine.logger().entries();
  const std::size_t count = entries.size();
  const std::size_t start = count > 14 ? count - 14 : 0;
  for (std::size_t i = start; i < count; ++i) {
    ui.text(entries[i].message);
  }
}

void buildSimControls(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  const SimulationStatus status = buildSimulationStatus(app.engine);

  ui.text("Mode: " + status.modeLabel);
  ui.text(status.modeDescription);
  ui.text(status.editStateLabel);
  ui.text("Playback: " + status.playbackLabel);
  ui.text("Reset target: " + status.resetTargetLabel);
  ui.text("Sim time: " + std::to_string(status.simTime));
  ui.text("Frame: " + std::to_string(status.frameCount));
  ui.separator();

  const std::vector<SimulationModeAction> modeActions
      = buildSimulationModeActions(app.engine);
  for (std::size_t i = 0; i < modeActions.size(); ++i) {
    const SimulationModeAction& action = modeActions[i];
    if (i > 0) {
      ui.sameLine();
    }
    std::string label = action.label;
    if (!action.enabled && !action.disabledReason.empty()) {
      label += " (" + action.disabledReason + ")";
    }
    label += "##simulation-mode-" + std::to_string(i);
    if (ui.button(label)) {
      app.note(applySimulationModeAction(app.engine, action.kind).message);
    }
  }

  double realTimeFactor = status.realTimeFactor;
  if (ui.slider("real-time factor", realTimeFactor, 0.0, 5.0)) {
    app.note(setSimulationRealTimeFactor(app.engine, realTimeFactor).message);
  }

  ui.separator();
  // Drive the checkbox from engine state so it stays correct when the recorder
  // is reset elsewhere (e.g. loading a project clears any active recording).
  bool record = status.recording;
  if (ui.checkbox("Record", record)) {
    app.note(setSimulationRecording(app.engine, record).message);
  }

  const std::size_t frames = status.replayFrameCount;
  if (frames > 0) {
    ui.text("Replay frames: " + std::to_string(frames));
    double maxIndex = static_cast<double>(frames - 1);
    if (app.replayFrame > maxIndex) {
      app.replayFrame = maxIndex;
    }
    if (ui.slider("timeline", app.replayFrame, 0.0, maxIndex)) {
      const std::size_t frame = static_cast<std::size_t>(app.replayFrame + 0.5);
      app.note(seekSimulationReplay(app.engine, frame).message);
    }
  }
}

void advanceRunningSimulation(EditorApp& app)
{
  SimulationController& sim = app.engine.simulation();
  if (!sim.isRunning()) {
    app.resetRunFrameClock();
    return;
  }

  sim.advance(app.consumeRunFrameSeconds());
}

dart::gui::KeyboardAction makeViewportMoveAction(
    std::string label,
    dart::gui::KeyboardKey key,
    ViewportMoveInput input,
    const std::shared_ptr<EditorApp>& app)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::namedKey(key);
  action.repeat = true;
  action.callback = [app, input](dart::gui::KeyboardActionContext& context) {
    if (!moveSelectedFromViewport(app->engine, input)) {
      return;
    }
    syncViewportTransformGizmo(app->transformGizmo, app->engine);
    if (context.lifecycle != nullptr) {
      context.lifecycle->paused = true;
    }
    const std::string name = primarySelectionName(app->engine);
    app->note(name.empty() ? "Moved selection" : "Moved " + name);
  };
  return action;
}

std::vector<dart::gui::KeyboardAction> makeViewportMoveActions(
    const std::shared_ptr<EditorApp>& app)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(makeViewportMoveAction(
      "Move selection left",
      dart::gui::KeyboardKey::Left,
      ViewportMoveInput{.left = true},
      app));
  actions.push_back(makeViewportMoveAction(
      "Move selection right",
      dart::gui::KeyboardKey::Right,
      ViewportMoveInput{.right = true},
      app));
  actions.push_back(makeViewportMoveAction(
      "Move selection forward",
      dart::gui::KeyboardKey::Up,
      ViewportMoveInput{.forward = true},
      app));
  actions.push_back(makeViewportMoveAction(
      "Move selection backward",
      dart::gui::KeyboardKey::Down,
      ViewportMoveInput{.backward = true},
      app));
  actions.push_back(makeViewportMoveAction(
      "Move selection up",
      dart::gui::KeyboardKey::PageUp,
      ViewportMoveInput{.up = true},
      app));
  actions.push_back(makeViewportMoveAction(
      "Move selection down",
      dart::gui::KeyboardKey::PageDown,
      ViewportMoveInput{.down = true},
      app));
  return actions;
}

void buildMenuBar(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  if (!ui.beginMenuBar()) {
    ui.text("dartsim editor");
    return;
  }
  if (ui.beginMenu("File")) {
    if (ui.menuItem("New Project")) {
      app.note(newProject(app.engine).message);
    }
    if (ui.menuItem("Open Project...")) {
      openProjectFromNativeDialog(app);
    }
    if (ui.menuItem("Save Project")) {
      if (app.engine.hasProjectPath()) {
        app.note(saveProject(app.engine).message);
      } else {
        saveProjectFromNativeDialog(app);
      }
    }
    if (ui.menuItem("Save Project As...")) {
      saveProjectFromNativeDialog(app);
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
    for (const RelationshipAction& action :
         buildRelationshipActions(app.engine)) {
      std::string label = action.label;
      if (!action.enabled && !action.disabledReason.empty()) {
        label += " (" + action.disabledReason + ")";
      }
      if (ui.menuItem(label)) {
        app.note(applyRelationshipAction(app.engine, action.kind).message);
        syncViewportTransformGizmo(app.transformGizmo, app.engine);
      }
    }
    ui.endMenu();
  }
  if (ui.beginMenu("Create")) {
    for (const PaletteAction& action : buildPaletteActions(app.engine)) {
      std::string label = action.label;
      if (!action.enabled && !action.disabledReason.empty()) {
        label += " (" + action.disabledReason + ")";
      }
      if (ui.menuItem(label)) {
        app.note(applyPaletteAction(app.engine, action.kind).message);
        syncViewportTransformGizmo(app.transformGizmo, app.engine);
      }
    }
    ui.endMenu();
  }
  if (ui.beginMenu("Simulate")) {
    for (const SimulationModeAction& action :
         buildSimulationModeActions(app.engine)) {
      std::string label = action.label;
      if (!action.enabled && !action.disabledReason.empty()) {
        label += " (" + action.disabledReason + ")";
      }
      if (ui.menuItem(label)) {
        app.note(applySimulationModeAction(app.engine, action.kind).message);
      }
    }
    ui.endMenu();
  }
  ui.endMenuBar();
}

void buildProjectPathModal(dart::gui::PanelBuilder& ui, EditorApp& app)
{
  const std::string title = projectPathModalTitle(app.projectPathKind);
  if (app.projectPathModalRequested) {
    ui.openModal(title, app.projectPathModalOpen);
    app.projectPathModalRequested = false;
  }
  if (!app.projectPathModalOpen) {
    return;
  }

  if (!ui.beginModal(title, app.projectPathModalOpen)) {
    return;
  }

  std::string path = app.projectPath;
  if (ui.textInput("Project path", path)) {
    app.projectPath = path;
    app.projectBrowserDirectory = projectBrowserDirectoryFor(app.projectPath);
  }
  if (ui.button("Browse...")) {
    browseProjectPath(app);
  }
  ui.sameLine();
  const std::string primary
      = app.projectPathKind == ProjectFileDialogKind::Open ? "Open" : "Save";
  if (ui.button(primary)) {
    applyProjectPathModal(app);
  }
  ui.sameLine();
  if (ui.button("Cancel")) {
    app.projectPathModalOpen = false;
  }
  if (!app.projectPathStatus.empty()) {
    ui.separator();
    ui.text(app.projectPathStatus);
  }

  ui.separator();
  if (app.projectBrowserDirectory.empty()) {
    app.projectBrowserDirectory = projectBrowserDirectoryFor(app.projectPath);
  }
  ui.text("Folder: " + app.projectBrowserDirectory.string());
  if (ui.button("Up##project-browser")) {
    const std::filesystem::path parent
        = app.projectBrowserDirectory.parent_path();
    if (!parent.empty()) {
      app.projectBrowserDirectory = parent;
      app.projectBrowserStatus.clear();
    }
  }
  ui.sameLine();
  if (ui.button("Refresh##project-browser")) {
    app.projectBrowserStatus.clear();
  }

  std::string browserStatus;
  const std::vector<ProjectBrowserEntry> entries
      = projectBrowserEntries(app.projectBrowserDirectory, browserStatus);
  app.projectBrowserStatus = std::move(browserStatus);
  if (!app.projectBrowserStatus.empty()) {
    ui.text(app.projectBrowserStatus);
  }
  if (entries.empty() && app.projectBrowserStatus.empty()) {
    ui.text("(no project files)");
  }
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const ProjectBrowserEntry& entry = entries[i];
    const std::string label
        = entry.label + "##project-browser-entry-" + std::to_string(i);
    if (!ui.button(label)) {
      continue;
    }
    if (entry.directory) {
      app.projectBrowserDirectory = entry.path;
      app.projectBrowserStatus.clear();
    } else {
      app.projectPath = entry.path.string();
      app.projectPathStatus.clear();
    }
  }

  ui.endModal();
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
  app.engine.markProjectClean();
  app.note("dartsim editor ready");
}

dart::gui::Panel makePanel(
    std::string title,
    bool menuBar,
    std::array<double, 2> position,
    std::array<double, 2> size,
    dart::gui::DockSide dockSide,
    std::function<void(dart::gui::PanelBuilder&)> build)
{
  dart::gui::Panel panel;
  panel.title = std::move(title);
  panel.menuBar = menuBar;
  panel.initialPosition = position;
  panel.initialSize = size;
  panel.autoResize = false;
  panel.dockSide = dockSide;
  panel.build = std::move(build);
  return panel;
}

dart::gui::Panel makePanel(
    std::string title,
    bool menuBar,
    std::array<double, 2> position,
    std::array<double, 2> size,
    dart::gui::DockSide dockSide,
    std::function<void(dart::gui::PanelBuilder&, dart::gui::PanelContext&)>
        build)
{
  dart::gui::Panel panel;
  panel.title = std::move(title);
  panel.menuBar = menuBar;
  panel.initialPosition = position;
  panel.initialSize = size;
  panel.autoResize = false;
  panel.dockSide = dockSide;
  panel.buildWithContext = std::move(build);
  return panel;
}

bool hasSceneOption(int argc, char* argv[])
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] != nullptr && std::string_view(argv[i]) == "--scene") {
      return true;
    }
  }

  return false;
}

} // namespace

int runEditor(int argc, char* argv[])
{
  if (hasSceneOption(argc, argv)) {
    return dart::gui::runApplication(argc, argv);
  }

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
  options.selectedRenderableProvider = [app]() {
    return dart::gui::RenderableSelection{
        selectedViewportRenderable(app->engine),
        selectedViewportLabel(app->engine)};
  };
  options.onRenderableSelected = [app](dart::gui::RenderableId renderableId) {
    if (renderableId == 0) {
      app->engine.select(kNoObject);
      syncViewportTransformGizmo(app->transformGizmo, app->engine);
      return;
    }
    if (!selectViewportRenderable(app->engine, renderableId)) {
      return;
    }
    syncViewportTransformGizmo(app->transformGizmo, app->engine);
    const ObjectId id = objectIdForRenderable(renderableId);
    if (const SceneObject* object = app->engine.objects().model().find(id)) {
      app->note("Selected " + object->name);
    }
  };
  app->transformGizmo.gizmo.isVisible = [app]() {
    return app->transformGizmo.object != kNoObject;
  };
  app->transformGizmo.gizmo.onChanged
      = [app](const Eigen::Isometry3d& transform) {
          if (!applyViewportTransformGizmo(
                  app->engine, app->transformGizmo, transform)) {
            return;
          }
          const std::string name = primarySelectionName(app->engine);
          app->note(
              name.empty() ? "Transformed selection" : "Transformed " + name);
        };
  options.gizmos.push_back(app->transformGizmo.gizmo);
  options.keyboardActions = makeViewportMoveActions(app);
  options.preRender = [app]() {
    syncViewportTransformGizmo(app->transformGizmo, app->engine);
    advanceRunningSimulation(*app);
  };

  if (dart::gui::isDockingAvailable()) {
    app->note("Panel docking: enabled");
  } else {
    app->note(
        "Panel docking: UNAVAILABLE - this build lacks docking support. "
        "Launch with `pixi run dartsim` to rebuild with docking enabled.");
  }
  app->note(
      "Press F2 (or pass --perf-hud) for the live performance HUD "
      "(per-phase CPU/GPU frame timing).");

  options.panels.push_back(makePanel(
      "Menu",
      true,
      {0.0, 0.0},
      {1280.0, 0.0},
      dart::gui::DockSide::Top,
      [app](dart::gui::PanelBuilder& ui, dart::gui::PanelContext& context) {
        app->projectDialogParentWindow = context.nativeWindow;
        buildMenuBar(ui, *app);
        buildProjectPathModal(ui, *app);
      }));
  options.panels.push_back(makePanel(
      "Scene Tree",
      false,
      {0.0, 30.0},
      {280.0, 360.0},
      dart::gui::DockSide::Left,
      [app](dart::gui::PanelBuilder& ui) { buildSceneTree(ui, *app); }));
  options.panels.push_back(makePanel(
      "Inspector",
      false,
      {1000.0, 30.0},
      {280.0, 360.0},
      dart::gui::DockSide::Right,
      [app](dart::gui::PanelBuilder& ui) { buildInspector(ui, *app); }));
  options.panels.push_back(makePanel(
      "Simulation",
      false,
      {0.0, 400.0},
      {280.0, 220.0},
      dart::gui::DockSide::Bottom,
      [app](dart::gui::PanelBuilder& ui) { buildSimControls(ui, *app); }));
  options.panels.push_back(makePanel(
      "Console",
      false,
      {300.0, 520.0},
      {680.0, 180.0},
      dart::gui::DockSide::Bottom,
      [app](dart::gui::PanelBuilder& ui) { buildConsole(ui, *app); }));

  return dart::gui::runApplication(argc, argv, options);
}

} // namespace dartsim::ui
