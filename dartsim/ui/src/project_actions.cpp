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

#include <dartsim_ui/project_actions.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <system_error>
#include <utility>

namespace dartsim::ui {

namespace {

bool shouldBlockDirtyProject(
    const SimEngine& engine, DirtyProjectPolicy dirtyPolicy)
{
  return engine.isProjectDirty() && dirtyPolicy == DirtyProjectPolicy::Block;
}

std::string makeDialogFailureMessage(
    std::string_view action, const ProjectFileDialogResult& result)
{
  std::string message(action);
  message += " dialog failed";
  if (!result.error.empty()) {
    message += ": ";
    message += result.error;
  }
  return message;
}

std::string makeOpenFailureMessage(const std::string& path)
{
  if (path.empty()) {
    return "Open failed: empty path";
  }

  std::error_code error;
  const std::filesystem::path projectPath(path);
  if (!std::filesystem::exists(projectPath, error)) {
    if (error) {
      return "Open failed: " + path + ": " + error.message();
    }
    return "Open failed: " + path + " does not exist";
  }

  if (std::filesystem::is_directory(projectPath, error)) {
    if (error) {
      return "Open failed: " + path + ": " + error.message();
    }
    return "Open failed: " + path + " is a directory";
  }

  std::ifstream file(projectPath, std::ios::binary);
  if (!file) {
    return "Open failed: " + path + " is not readable";
  }

  return "Open failed: " + path + " is not a dartsim project";
}

std::string ensureProjectExtension(std::string path)
{
  if (std::filesystem::path(path).extension().empty()) {
    path += ".dartsim";
  }
  return path;
}

std::string resolveOpenProjectPath(std::string path)
{
  if (path.empty()) {
    return path;
  }

  const std::filesystem::path projectPath(path);
  if (!projectPath.extension().empty()) {
    return path;
  }

  std::error_code error;
  if (std::filesystem::exists(projectPath, error) || error) {
    return path;
  }

  std::string extended = path;
  extended += ".dartsim";
  if (std::filesystem::is_regular_file(std::filesystem::path(extended), error)
      && !error) {
    return extended;
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

std::string displayNameForProjectPath(const std::string& path)
{
  if (path.empty()) {
    return "Untitled";
  }
  const std::string filename = std::filesystem::path(path).filename().string();
  return filename.empty() ? path : filename;
}

std::string currentProjectDisplayName(const SimEngine& engine)
{
  return displayNameForProjectPath(engine.projectPath());
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

ProjectActionResult macroGuardResult(std::string_view action)
{
  return {
      false, "Cannot " + std::string(action) + " during an edit transaction"};
}

ProjectReplacementActionResult projectReplacementPrompt(
    const ProjectReplacementRequest& request)
{
  return {true, {false, "Unsaved changes"}, request};
}

ProjectReplacementRequest makeNewProjectReplacementRequest(
    const SimEngine& engine)
{
  ProjectReplacementRequest request;
  request.kind = ProjectReplacementKind::NewProject;
  request.message = "Create a new project and discard unsaved changes in "
                    + currentProjectDisplayName(engine) + "?";
  request.confirmLabel = "Discard and New";
  return request;
}

ProjectReplacementRequest makeOpenProjectReplacementRequest(
    const SimEngine& engine, const std::string& path)
{
  ProjectReplacementRequest request;
  request.kind = ProjectReplacementKind::OpenProject;
  request.path = path;
  request.message = "Open " + displayNameForProjectPath(path)
                    + " and discard unsaved changes in "
                    + currentProjectDisplayName(engine) + "?";
  request.confirmLabel = "Discard and Open";
  return request;
}

ProjectReplacementRequest makeCloseProjectReplacementRequest(
    const SimEngine& engine)
{
  ProjectReplacementRequest request;
  request.kind = ProjectReplacementKind::CloseProject;
  request.message = "Close " + currentProjectDisplayName(engine)
                    + " and discard unsaved changes?";
  request.confirmLabel = "Discard and Close";
  return request;
}

} // namespace

ProjectStatus buildProjectStatus(
    const SimEngine& engine, const RecentProjectState& recentProjects)
{
  ProjectStatus status;
  status.hasProjectPath = engine.hasProjectPath();
  status.dirty = engine.isProjectDirty();
  status.path = engine.projectPath();
  status.displayName = displayNameForProjectPath(status.path);
  status.dirtyLabel
      = status.dirty ? "Unsaved changes"
                     : (status.hasProjectPath ? "Saved" : "No unsaved changes");
  status.titleLabel = status.displayName + (status.dirty ? " *" : "");
  status.detailLabel = status.displayName + " - " + status.dirtyLabel;
  if (status.hasProjectPath) {
    status.detailLabel += " - " + status.path;
  }
  status.recentProjectCount = recentProjects.paths.size();
  return status;
}

void rememberRecentProject(RecentProjectState& state, std::string path)
{
  if (path.empty() || state.capacity == 0) {
    return;
  }
  auto duplicate = std::find(state.paths.begin(), state.paths.end(), path);
  if (duplicate != state.paths.end()) {
    state.paths.erase(duplicate);
  }
  state.paths.insert(state.paths.begin(), std::move(path));
  if (state.paths.size() > state.capacity) {
    state.paths.resize(state.capacity);
  }
}

std::vector<RecentProjectEntry> buildRecentProjectEntries(
    const SimEngine& engine, const RecentProjectState& state)
{
  std::vector<RecentProjectEntry> entries;
  entries.reserve(state.paths.size());
  for (const std::string& path : state.paths) {
    RecentProjectEntry entry;
    entry.path = path;
    entry.label = displayNameForProjectPath(path);
    entry.current = engine.hasProjectPath() && engine.projectPath() == path;
    entries.push_back(std::move(entry));
  }
  return entries;
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

std::vector<ProjectBrowserEntry> projectBrowserEntries(
    const std::filesystem::path& directory,
    std::string& status,
    std::size_t maxEntries)
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

  if (entries.size() > maxEntries) {
    entries.resize(maxEntries);
    status = "Showing first " + std::to_string(maxEntries) + " project entries";
  }

  return entries;
}

ProjectActionResult newProject(
    SimEngine& engine, DirtyProjectPolicy dirtyPolicy)
{
  if (engine.commands().inMacro()) {
    return macroGuardResult("create a new project");
  }
  if (shouldBlockDirtyProject(engine, dirtyPolicy)) {
    return {false, "Unsaved changes"};
  }
  engine.newProject();
  return {true, "New project"};
}

ProjectActionResult closeProject(
    SimEngine& engine, DirtyProjectPolicy dirtyPolicy)
{
  if (engine.commands().inMacro()) {
    return macroGuardResult("close the project");
  }
  if (shouldBlockDirtyProject(engine, dirtyPolicy)) {
    return {false, "Unsaved changes"};
  }
  engine.newProject();
  return {true, "Closed project"};
}

ProjectReplacementActionResult requestNewProjectReplacement(SimEngine& engine)
{
  if (engine.commands().inMacro()) {
    return {false, macroGuardResult("create a new project"), {}};
  }
  if (engine.isProjectDirty()) {
    return projectReplacementPrompt(makeNewProjectReplacementRequest(engine));
  }
  return {false, newProject(engine, DirtyProjectPolicy::Discard), {}};
}

ProjectReplacementActionResult requestCloseProjectReplacement(SimEngine& engine)
{
  if (engine.commands().inMacro()) {
    return {false, macroGuardResult("close the project"), {}};
  }
  if (engine.isProjectDirty()) {
    return projectReplacementPrompt(makeCloseProjectReplacementRequest(engine));
  }
  return {false, closeProject(engine, DirtyProjectPolicy::Discard), {}};
}

ProjectReplacementActionResult requestOpenProjectReplacement(
    SimEngine& engine, std::string path)
{
  if (engine.commands().inMacro()) {
    return {false, macroGuardResult("load a project"), {}};
  }
  if (engine.isProjectDirty()) {
    return projectReplacementPrompt(
        makeOpenProjectReplacementRequest(engine, path));
  }
  return {
      false,
      openProject(engine, std::move(path), DirtyProjectPolicy::Discard),
      {}};
}

ProjectReplacementActionResult requestOpenProjectReplacementWithDialog(
    SimEngine& engine,
    const ProjectFileDialog& dialog,
    void* parentNativeWindow)
{
  if (engine.commands().inMacro()) {
    return {false, macroGuardResult("load a project"), {}};
  }
  if (!dialog) {
    return {false, {false, "Open dialog unavailable"}, {}};
  }

  ProjectFileDialogRequest request;
  request.kind = ProjectFileDialogKind::Open;
  request.defaultPath = parentPathOrEmpty(engine.projectPath());
  request.defaultName = filenameOrDefault(engine.projectPath());
  request.parentNativeWindow = parentNativeWindow;

  const ProjectFileDialogResult selection = dialog(request);
  switch (selection.status) {
    case ProjectFileDialogStatus::Selected:
      if (selection.path.empty()) {
        return {false, {false, "Open canceled"}, {}};
      }
      return requestOpenProjectReplacement(engine, selection.path);
    case ProjectFileDialogStatus::Canceled:
      return {false, {false, "Open canceled"}, {}};
    case ProjectFileDialogStatus::Failed:
      return {false, {false, makeDialogFailureMessage("Open", selection)}, {}};
  }
  return {false, {false, "Open dialog failed"}, {}};
}

ProjectActionResult confirmProjectReplacement(
    SimEngine& engine, const ProjectReplacementRequest& request)
{
  switch (request.kind) {
    case ProjectReplacementKind::NewProject:
      return newProject(engine, DirtyProjectPolicy::Discard);
    case ProjectReplacementKind::OpenProject:
      return openProject(engine, request.path, DirtyProjectPolicy::Discard);
    case ProjectReplacementKind::CloseProject:
      return closeProject(engine, DirtyProjectPolicy::Discard);
  }
  return {false, "Project replacement failed"};
}

ProjectActionResult cancelProjectReplacement(
    const ProjectReplacementRequest& request)
{
  switch (request.kind) {
    case ProjectReplacementKind::NewProject:
      return {false, "New project canceled"};
    case ProjectReplacementKind::OpenProject:
      return {false, "Open canceled"};
    case ProjectReplacementKind::CloseProject:
      return {false, "Close canceled"};
  }
  return {false, "Project replacement canceled"};
}

ProjectActionResult saveProjectAs(SimEngine& engine, std::string path)
{
  if (!engine.saveProject(std::move(path))) {
    return {false, "Save failed"};
  }
  return {true, "Saved " + engine.projectPath()};
}

ProjectActionResult saveProject(SimEngine& engine, std::string defaultPath)
{
  const bool ok = engine.hasProjectPath() ? engine.saveProject()
                                          : engine.saveProject(defaultPath);
  if (!ok) {
    return {false, "Save failed"};
  }
  return {true, "Saved " + engine.projectPath()};
}

ProjectActionResult saveProjectWithDialog(
    SimEngine& engine,
    const ProjectFileDialog& dialog,
    bool forceDialog,
    void* parentNativeWindow)
{
  if (engine.hasProjectPath() && !forceDialog) {
    return saveProject(engine);
  }

  if (!dialog) {
    return {false, "Save dialog unavailable"};
  }

  ProjectFileDialogRequest request;
  request.kind = ProjectFileDialogKind::Save;
  request.defaultPath = parentPathOrEmpty(engine.projectPath());
  request.defaultName = filenameOrDefault(engine.projectPath());
  request.parentNativeWindow = parentNativeWindow;

  const ProjectFileDialogResult selection = dialog(request);
  switch (selection.status) {
    case ProjectFileDialogStatus::Selected:
      if (selection.path.empty()) {
        return {false, "Save canceled"};
      }
      return saveProjectAs(engine, ensureProjectExtension(selection.path));
    case ProjectFileDialogStatus::Canceled:
      return {false, "Save canceled"};
    case ProjectFileDialogStatus::Failed:
      return {false, makeDialogFailureMessage("Save", selection)};
  }
  return {false, "Save dialog failed"};
}

ProjectActionResult openProject(
    SimEngine& engine, std::string path, DirtyProjectPolicy dirtyPolicy)
{
  if (engine.commands().inMacro()) {
    return macroGuardResult("load a project");
  }
  if (shouldBlockDirtyProject(engine, dirtyPolicy)) {
    return {false, "Unsaved changes"};
  }
  path = resolveOpenProjectPath(std::move(path));
  if (!engine.loadProject(path)) {
    return {false, makeOpenFailureMessage(path)};
  }
  return {true, "Loaded " + path};
}

ProjectActionResult openProjectWithDialog(
    SimEngine& engine,
    const ProjectFileDialog& dialog,
    DirtyProjectPolicy dirtyPolicy)
{
  if (engine.commands().inMacro()) {
    return macroGuardResult("load a project");
  }
  if (shouldBlockDirtyProject(engine, dirtyPolicy)) {
    return {false, "Unsaved changes"};
  }

  if (!dialog) {
    return {false, "Open dialog unavailable"};
  }

  ProjectFileDialogRequest request;
  request.kind = ProjectFileDialogKind::Open;
  request.defaultPath = parentPathOrEmpty(engine.projectPath());
  request.defaultName = filenameOrDefault(engine.projectPath());

  const ProjectFileDialogResult selection = dialog(request);
  switch (selection.status) {
    case ProjectFileDialogStatus::Selected:
      if (selection.path.empty()) {
        return {false, "Open canceled"};
      }
      return openProject(engine, selection.path, DirtyProjectPolicy::Discard);
    case ProjectFileDialogStatus::Canceled:
      return {false, "Open canceled"};
    case ProjectFileDialogStatus::Failed:
      return {false, makeDialogFailureMessage("Open", selection)};
  }
  return {false, "Open dialog failed"};
}

ProjectActionResult openRecentProject(
    SimEngine& engine,
    RecentProjectState& recentProjects,
    std::string path,
    DirtyProjectPolicy dirtyPolicy)
{
  ProjectActionResult result
      = openProject(engine, std::move(path), dirtyPolicy);
  if (result.ok && engine.hasProjectPath()) {
    rememberRecentProject(recentProjects, engine.projectPath());
  }
  return result;
}

} // namespace dartsim::ui
