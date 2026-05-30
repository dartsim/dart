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

#pragma once

#include <dartsim_engine/sim_engine.hpp>

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include <cstddef>

namespace dartsim::ui {

inline constexpr const char* kDefaultProjectPath = "scene.dartsim";
inline constexpr std::size_t kDefaultRecentProjectCapacity = 8;

/// What the menu layer should do after a failed project action.
///
/// `None` (the default) means "just report the message" — this covers success,
/// user cancellation, and dirty-guard/macro-guard rejections. `ModalFallback`
/// means a native file dialog could not deliver a usable path (it was
/// unavailable or failed) or the chosen path failed to load/save, so the menu
/// should offer the in-app project browser / manual path modal instead. The
/// menu branches on this enum rather than matching message text, which kept
/// breaking silently when a message was reworded or a new failure string was
/// added (e.g. "Open dialog unavailable" / "Save dialog unavailable").
enum class ProjectActionFollowUp
{
  None,
  ModalFallback,
};

/// Result of a user-facing project action.
struct ProjectActionResult
{
  bool ok = false;
  std::string message;
  ProjectActionFollowUp followUp = ProjectActionFollowUp::None;
};

struct ProjectStatus
{
  bool hasProjectPath = false;
  bool dirty = false;
  std::string path;
  std::string displayName = "Untitled";
  std::string dirtyLabel = "No unsaved changes";
  std::string titleLabel = "Untitled";
  std::string detailLabel = "Untitled - No unsaved changes";
  std::size_t recentProjectCount = 0;
};

struct RecentProjectState
{
  std::vector<std::string> paths;
  std::size_t capacity = kDefaultRecentProjectCapacity;
};

struct RecentProjectEntry
{
  std::string path;
  std::string label;
  bool current = false;
};

struct ProjectBrowserEntry
{
  std::filesystem::path path;
  std::string label;
  bool directory = false;
};

enum class DirtyProjectPolicy
{
  Block,
  Discard,
};

enum class ProjectReplacementKind
{
  NewProject,
  OpenProject,
  CloseProject,
};

struct ProjectReplacementRequest
{
  ProjectReplacementKind kind = ProjectReplacementKind::NewProject;
  std::string path;
  std::string title = "Unsaved changes";
  std::string message;
  std::string confirmLabel = "Discard Changes";
  std::string cancelLabel = "Cancel";
};

struct ProjectReplacementActionResult
{
  bool promptRequired = false;
  ProjectActionResult result;
  ProjectReplacementRequest request;
};

struct ProjectReplacementPromptState
{
  bool open = false;
  ProjectReplacementRequest request;
};

enum class ProjectFileDialogKind
{
  Open,
  Save,
};

enum class ProjectFileDialogStatus
{
  Selected,
  Canceled,
  Failed,
};

struct ProjectFileDialogRequest
{
  ProjectFileDialogKind kind = ProjectFileDialogKind::Open;
  std::string defaultPath;
  std::string defaultName = kDefaultProjectPath;
  void* parentNativeWindow = nullptr;
};

struct ProjectFileDialogResult
{
  ProjectFileDialogStatus status = ProjectFileDialogStatus::Canceled;
  std::string path;
  std::string error;
};

using ProjectFileDialog
    = std::function<ProjectFileDialogResult(const ProjectFileDialogRequest&)>;

[[nodiscard]] ProjectStatus buildProjectStatus(
    const SimEngine& engine,
    const RecentProjectState& recentProjects = RecentProjectState{});
void rememberRecentProject(RecentProjectState& state, std::string path);
[[nodiscard]] std::vector<RecentProjectEntry> buildRecentProjectEntries(
    const SimEngine& engine, const RecentProjectState& state);
[[nodiscard]] std::filesystem::path projectBrowserDirectoryFor(
    const std::string& path);
[[nodiscard]] std::vector<ProjectBrowserEntry> projectBrowserEntries(
    const std::filesystem::path& directory,
    std::string& status,
    std::size_t maxEntries = 48);

/// Create a fresh project and return the status text for the UI log.
ProjectActionResult newProject(
    SimEngine& engine,
    DirtyProjectPolicy dirtyPolicy = DirtyProjectPolicy::Block);

/// Close the current project to an untitled empty workspace.
ProjectActionResult closeProject(
    SimEngine& engine,
    DirtyProjectPolicy dirtyPolicy = DirtyProjectPolicy::Block);

/// Create a fresh project, or describe the confirmation required to replace a
/// dirty project.
ProjectReplacementActionResult requestNewProjectReplacement(SimEngine& engine);

/// Close the current project, or describe the confirmation required to close a
/// dirty project.
ProjectReplacementActionResult requestCloseProjectReplacement(
    SimEngine& engine);

/// Load a project path, or describe the confirmation required to replace a
/// dirty project.
ProjectReplacementActionResult requestOpenProjectReplacement(
    SimEngine& engine, std::string path = kDefaultProjectPath);

/// Ask for a project path and either load it or describe the confirmation
/// required to replace a dirty project.
ProjectReplacementActionResult requestOpenProjectReplacementWithDialog(
    SimEngine& engine,
    const ProjectFileDialog& dialog,
    void* parentNativeWindow = nullptr);

/// Apply a previously requested dirty-project replacement.
ProjectActionResult confirmProjectReplacement(
    SimEngine& engine, const ProjectReplacementRequest& request);

/// Cancel a pending dirty-project replacement.
ProjectActionResult cancelProjectReplacement(
    const ProjectReplacementRequest& request);

/// Save to the current project path, or `defaultPath` when the project has not
/// been saved yet.
ProjectActionResult saveProject(
    SimEngine& engine, std::string defaultPath = kDefaultProjectPath);

/// Save to an explicit project path.
ProjectActionResult saveProjectAs(SimEngine& engine, std::string path);

/// Save to the current path, or ask for a path when the project has not been
/// saved yet. When `forceDialog` is true, always ask for a destination path.
ProjectActionResult saveProjectWithDialog(
    SimEngine& engine,
    const ProjectFileDialog& dialog,
    bool forceDialog,
    void* parentNativeWindow = nullptr);

/// Load a project path and return the status text for the UI log.
ProjectActionResult openProject(
    SimEngine& engine,
    std::string path = kDefaultProjectPath,
    DirtyProjectPolicy dirtyPolicy = DirtyProjectPolicy::Block);

/// Ask for a project path and load it.
ProjectActionResult openProjectWithDialog(
    SimEngine& engine,
    const ProjectFileDialog& dialog,
    DirtyProjectPolicy dirtyPolicy = DirtyProjectPolicy::Block);

/// Load a recent project path and update the recent-project list on success.
ProjectActionResult openRecentProject(
    SimEngine& engine,
    RecentProjectState& recentProjects,
    std::string path,
    DirtyProjectPolicy dirtyPolicy = DirtyProjectPolicy::Block);

} // namespace dartsim::ui
