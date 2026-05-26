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

#include <functional>
#include <string>

namespace dartsim::ui {

inline constexpr const char* kDefaultProjectPath = "scene.dartsim";

/// Result of a user-facing project action.
struct ProjectActionResult
{
  bool ok = false;
  std::string message;
};

enum class DirtyProjectPolicy
{
  Block,
  Discard,
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

/// Create a fresh project and return the status text for the UI log.
ProjectActionResult newProject(
    SimEngine& engine,
    DirtyProjectPolicy dirtyPolicy = DirtyProjectPolicy::Block);

/// Save to the current project path, or `defaultPath` when the project has not
/// been saved yet.
ProjectActionResult saveProject(
    SimEngine& engine, std::string defaultPath = kDefaultProjectPath);

/// Save to an explicit project path.
ProjectActionResult saveProjectAs(SimEngine& engine, std::string path);

/// Save to the current path, or ask for a path when the project has not been
/// saved yet. When `forceDialog` is true, always ask for a destination path.
ProjectActionResult saveProjectWithDialog(
    SimEngine& engine, const ProjectFileDialog& dialog, bool forceDialog);

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

} // namespace dartsim::ui
