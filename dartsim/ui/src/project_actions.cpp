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

#include <filesystem>
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
  if (!std::filesystem::exists(path, error)) {
    return "Open failed: " + path + " does not exist";
  }

  return "Open failed: " + path;
}

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

} // namespace

ProjectActionResult newProject(
    SimEngine& engine, DirtyProjectPolicy dirtyPolicy)
{
  if (shouldBlockDirtyProject(engine, dirtyPolicy)) {
    return {false, "Unsaved changes"};
  }
  engine.newProject();
  return {true, "New project"};
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
    SimEngine& engine, const ProjectFileDialog& dialog, bool forceDialog)
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
  if (shouldBlockDirtyProject(engine, dirtyPolicy)) {
    return {false, "Unsaved changes"};
  }
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

} // namespace dartsim::ui
