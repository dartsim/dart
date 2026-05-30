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

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_ui/project_actions.hpp>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

} // namespace

TEST(DartsimProjectActions, SaveUsesDefaultPathOnlyBeforeFirstSave)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_default_save.dartsim";
  const std::filesystem::path ignored
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_ignored_default.dartsim";
  std::filesystem::remove(path);
  std::filesystem::remove(ignored);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.isProjectDirty());

  const auto firstSave = ui::saveProject(engine, path.string());
  EXPECT_TRUE(firstSave.ok);
  EXPECT_EQ(firstSave.message, "Saved " + path.string());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_FALSE(engine.isProjectDirty());

  engine.execute(commands::setMass(engine.selection().primary(), 2.0));
  ASSERT_TRUE(engine.isProjectDirty());
  const auto secondSave = ui::saveProject(engine, ignored.string());
  EXPECT_TRUE(secondSave.ok);
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_FALSE(std::filesystem::exists(ignored));
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, OpenAndNewProjectReturnUiStatus)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_open_new.dartsim";

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  const auto open = ui::openProject(engine, path.string());
  EXPECT_TRUE(open.ok);
  EXPECT_EQ(open.message, "Loaded " + path.string());
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_TRUE(engine.hasProjectPath());
  EXPECT_EQ(engine.objects().model().size(), 1u);

  const auto fresh = ui::newProject(engine);
  EXPECT_TRUE(fresh.ok);
  EXPECT_EQ(fresh.message, "New project");
  EXPECT_TRUE(engine.objects().model().empty());
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, ProjectStatusExposesDirtyPathAndRecentCount)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_status_labels.dartsim";
  std::filesystem::remove(path);

  SimEngine engine;
  ui::RecentProjectState recent;
  ui::ProjectStatus status = ui::buildProjectStatus(engine, recent);
  EXPECT_FALSE(status.hasProjectPath);
  EXPECT_FALSE(status.dirty);
  EXPECT_EQ(status.displayName, "Untitled");
  EXPECT_EQ(status.titleLabel, "Untitled");
  EXPECT_EQ(status.dirtyLabel, "No unsaved changes");
  EXPECT_EQ(status.detailLabel, "Untitled - No unsaved changes");
  EXPECT_EQ(status.recentProjectCount, 0u);

  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  status = ui::buildProjectStatus(engine, recent);
  EXPECT_TRUE(status.dirty);
  EXPECT_EQ(status.dirtyLabel, "Unsaved changes");
  EXPECT_EQ(status.titleLabel, "Untitled *");
  EXPECT_EQ(status.detailLabel, "Untitled - Unsaved changes");

  ASSERT_TRUE(engine.saveProject(path.string()));
  ui::rememberRecentProject(recent, engine.projectPath());
  status = ui::buildProjectStatus(engine, recent);
  EXPECT_TRUE(status.hasProjectPath);
  EXPECT_FALSE(status.dirty);
  EXPECT_EQ(status.path, path.string());
  EXPECT_EQ(status.displayName, path.filename().string());
  EXPECT_EQ(status.titleLabel, path.filename().string());
  EXPECT_EQ(
      status.detailLabel,
      path.filename().string() + " - Saved - " + path.string());
  EXPECT_EQ(status.recentProjectCount, 1u);

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, RecentProjectsDeduplicateAndExposeCurrentEntry)
{
  const std::filesystem::path first = std::filesystem::temp_directory_path()
                                      / "dartsim_ui_recent_first.dartsim";
  const std::filesystem::path second = std::filesystem::temp_directory_path()
                                       / "dartsim_ui_recent_second.dartsim";
  const std::filesystem::path third = std::filesystem::temp_directory_path()
                                      / "dartsim_ui_recent_third.dartsim";

  ui::RecentProjectState recent;
  recent.capacity = 2;
  ui::rememberRecentProject(recent, first.string());
  ui::rememberRecentProject(recent, second.string());
  ui::rememberRecentProject(recent, first.string());
  ui::rememberRecentProject(recent, {});
  ui::rememberRecentProject(recent, third.string());

  ASSERT_EQ(recent.paths.size(), 2u);
  EXPECT_EQ(recent.paths[0], third.string());
  EXPECT_EQ(recent.paths[1], first.string());

  SimEngine engine;
  ASSERT_TRUE(engine.saveProject(third.string()));
  const std::vector<ui::RecentProjectEntry> entries
      = ui::buildRecentProjectEntries(engine, recent);
  ASSERT_EQ(entries.size(), 2u);
  EXPECT_EQ(entries[0].path, third.string());
  EXPECT_EQ(entries[0].label, third.filename().string());
  EXPECT_TRUE(entries[0].current);
  EXPECT_EQ(entries[1].path, first.string());
  EXPECT_EQ(entries[1].label, first.filename().string());
  EXPECT_FALSE(entries[1].current);

  std::filesystem::remove(third);
}

TEST(DartsimProjectActions, ProjectBrowserFiltersAndSortsProjectEntries)
{
  const std::filesystem::path root = std::filesystem::temp_directory_path()
                                     / "dartsim_ui_project_browser_entries";
  std::filesystem::remove_all(root);
  ASSERT_TRUE(std::filesystem::create_directories(root / "z_dir"));
  ASSERT_TRUE(std::filesystem::create_directories(root / "a_dir"));
  {
    std::ofstream(root / "beta.dartsim") << "beta\n";
    std::ofstream(root / "alpha.dartsim") << "alpha\n";
    std::ofstream(root / "ignored.txt") << "ignored\n";
  }

  std::string status;
  const std::vector<ui::ProjectBrowserEntry> entries
      = ui::projectBrowserEntries(root, status);
  EXPECT_TRUE(status.empty());
  ASSERT_EQ(entries.size(), 4u);
  EXPECT_TRUE(entries[0].directory);
  EXPECT_EQ(entries[0].label, "[dir] a_dir");
  EXPECT_TRUE(entries[1].directory);
  EXPECT_EQ(entries[1].label, "[dir] z_dir");
  EXPECT_FALSE(entries[2].directory);
  EXPECT_EQ(entries[2].label, "alpha.dartsim");
  EXPECT_FALSE(entries[3].directory);
  EXPECT_EQ(entries[3].label, "beta.dartsim");

  const std::vector<ui::ProjectBrowserEntry> limited
      = ui::projectBrowserEntries(root, status, 2);
  ASSERT_EQ(limited.size(), 2u);
  EXPECT_EQ(status, "Showing first 2 project entries");

  status.clear();
  EXPECT_TRUE(ui::projectBrowserEntries(root / "missing", status).empty());
  EXPECT_NE(status.find("Cannot read folder:"), std::string::npos);

  const std::filesystem::path selectedFile = root / "alpha.dartsim";
  EXPECT_EQ(ui::projectBrowserDirectoryFor(selectedFile.string()), root);

  std::filesystem::remove_all(root);
}

TEST(DartsimProjectActions, ProjectBrowserHandlesEmptyPathsAndRecentCapacity)
{
  ui::RecentProjectState recent;
  recent.capacity = 0;
  ui::rememberRecentProject(recent, "ignored.dartsim");
  EXPECT_TRUE(recent.paths.empty());

  const std::filesystem::path root = std::filesystem::temp_directory_path()
                                     / "dartsim_ui_project_browser_fallbacks";
  std::filesystem::remove_all(root);
  ASSERT_TRUE(std::filesystem::create_directories(root));
  {
    std::ofstream(root / "scene.dartsim") << "{}\n";
  }

  const std::filesystem::path current = ui::projectBrowserDirectoryFor("");
  EXPECT_TRUE(std::filesystem::is_directory(current));
  EXPECT_EQ(
      ui::projectBrowserDirectoryFor((root / "scene.dartsim").string()), root);
  EXPECT_TRUE(
      std::filesystem::is_directory(
          ui::projectBrowserDirectoryFor(
              (root / "missing" / "scene").string())));

  std::string status;
  const std::vector<ui::ProjectBrowserEntry> limited
      = ui::projectBrowserEntries(root, status, 0);
  EXPECT_TRUE(limited.empty());
  EXPECT_EQ(status, "Showing first 0 project entries");

  std::filesystem::remove_all(root);
}

TEST(DartsimProjectActions, OpenWithDialogUsesSelectedProjectPath)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_open.dartsim";

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  bool dialogCalled = false;
  SimEngine engine;
  const auto result = ui::openProjectWithDialog(
      engine, [&](const ui::ProjectFileDialogRequest& request) {
        dialogCalled = true;
        EXPECT_EQ(request.kind, ui::ProjectFileDialogKind::Open);
        EXPECT_EQ(request.defaultName, ui::kDefaultProjectPath);
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, path.string(), {}};
      });

  EXPECT_TRUE(dialogCalled);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Loaded " + path.string());
  EXPECT_TRUE(engine.hasProjectPath());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_EQ(engine.objects().model().size(), 1u);

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, OpenReplacementWithDialogUsesNativeSelection)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_replacement_open.dartsim";
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  int parentToken = 0;
  void* parentWindow = &parentToken;
  bool dialogCalled = false;
  SimEngine engine;
  const auto result = ui::requestOpenProjectReplacementWithDialog(
      engine,
      [&](const ui::ProjectFileDialogRequest& request) {
        dialogCalled = true;
        EXPECT_EQ(request.kind, ui::ProjectFileDialogKind::Open);
        EXPECT_EQ(request.defaultName, ui::kDefaultProjectPath);
        EXPECT_EQ(request.parentNativeWindow, parentWindow);
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, path.string(), {}};
      },
      parentWindow);

  EXPECT_TRUE(dialogCalled);
  EXPECT_FALSE(result.promptRequired);
  EXPECT_TRUE(result.result.ok);
  EXPECT_EQ(result.result.message, "Loaded " + path.string());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, OpenReplacementWithDialogPromptsForDirtyProject)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_dirty_open.dartsim";
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();
  bool dialogCalled = false;
  const auto result = ui::requestOpenProjectReplacementWithDialog(
      engine, [&](const ui::ProjectFileDialogRequest&) {
        dialogCalled = true;
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, path.string(), {}};
      });

  EXPECT_TRUE(dialogCalled);
  EXPECT_TRUE(result.promptRequired);
  EXPECT_FALSE(result.result.ok);
  EXPECT_EQ(result.result.message, "Unsaved changes");
  EXPECT_EQ(result.request.kind, ui::ProjectReplacementKind::OpenProject);
  EXPECT_EQ(result.request.path, path.string());
  EXPECT_EQ(result.request.confirmLabel, "Discard and Open");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, OpenReplacementWithDialogReportsCancelAndFailure)
{
  SimEngine engine;
  const auto canceled = ui::requestOpenProjectReplacementWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Canceled, {}, {}};
      });
  EXPECT_FALSE(canceled.promptRequired);
  EXPECT_FALSE(canceled.result.ok);
  EXPECT_EQ(canceled.result.message, "Open canceled");
  // Cancellation is not an error: the menu reports it but must not fall back to
  // the in-app modal.
  EXPECT_EQ(canceled.result.followUp, ui::ProjectActionFollowUp::None);

  const auto failed = ui::requestOpenProjectReplacementWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Failed, {}, "backend unavailable"};
      });
  EXPECT_FALSE(failed.promptRequired);
  EXPECT_FALSE(failed.result.ok);
  EXPECT_EQ(failed.result.message, "Open dialog failed: backend unavailable");
  // A failed native dialog should fall back to the in-app modal.
  EXPECT_EQ(failed.result.followUp, ui::ProjectActionFollowUp::ModalFallback);

  const auto failedWithoutDetail = ui::requestOpenProjectReplacementWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Failed, {}, {}};
      });
  EXPECT_FALSE(failedWithoutDetail.promptRequired);
  EXPECT_FALSE(failedWithoutDetail.result.ok);
  EXPECT_EQ(failedWithoutDetail.result.message, "Open dialog failed");
  EXPECT_EQ(
      failedWithoutDetail.result.followUp,
      ui::ProjectActionFollowUp::ModalFallback);

  const auto empty = ui::requestOpenProjectReplacementWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, {}, {}};
      });
  EXPECT_FALSE(empty.promptRequired);
  EXPECT_FALSE(empty.result.ok);
  EXPECT_EQ(empty.result.message, "Open canceled");
  EXPECT_EQ(empty.result.followUp, ui::ProjectActionFollowUp::None);

  // An unavailable dialog (no callable) must fall back to the in-app modal —
  // this path silently failed to fall back before the followUp enum landed.
  const auto unavailable = ui::requestOpenProjectReplacementWithDialog(
      engine, ui::ProjectFileDialog{});
  EXPECT_FALSE(unavailable.result.ok);
  EXPECT_EQ(unavailable.result.message, "Open dialog unavailable");
  EXPECT_EQ(
      unavailable.result.followUp, ui::ProjectActionFollowUp::ModalFallback);

  // A selected path that then fails to load (here, a path that does not exist)
  // falls back to the modal too, just like a dialog failure.
  const std::filesystem::path missing
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_dialog_selected_missing_open.dartsim";
  std::filesystem::remove(missing);
  const auto selectedButBadLoad = ui::requestOpenProjectReplacementWithDialog(
      engine, [&](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, missing.string(), {}};
      });
  EXPECT_FALSE(selectedButBadLoad.promptRequired);
  EXPECT_FALSE(selectedButBadLoad.result.ok);
  EXPECT_EQ(
      selectedButBadLoad.result.followUp,
      ui::ProjectActionFollowUp::ModalFallback);
}

TEST(DartsimProjectActions, SaveWithDialogReportsFollowUpForFailures)
{
  SimEngine engine;

  // Cancellation reports but does not fall back.
  const auto canceled = ui::saveProjectWithDialog(
      engine,
      [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Canceled, {}, {}};
      },
      true);
  EXPECT_FALSE(canceled.ok);
  EXPECT_EQ(canceled.message, "Save canceled");
  EXPECT_EQ(canceled.followUp, ui::ProjectActionFollowUp::None);

  // A failed dialog falls back to the in-app modal.
  const auto failed = ui::saveProjectWithDialog(
      engine,
      [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Failed, {}, "backend unavailable"};
      },
      true);
  EXPECT_FALSE(failed.ok);
  EXPECT_EQ(failed.followUp, ui::ProjectActionFollowUp::ModalFallback);

  // An unavailable dialog falls back too (previously a silent no-fallback).
  const auto unavailable
      = ui::saveProjectWithDialog(engine, ui::ProjectFileDialog{}, true);
  EXPECT_FALSE(unavailable.ok);
  EXPECT_EQ(unavailable.message, "Save dialog unavailable");
  EXPECT_EQ(unavailable.followUp, ui::ProjectActionFollowUp::ModalFallback);

  // A selected path that then fails to save (here, a path under a directory
  // that does not exist) falls back to the modal too.
  const std::filesystem::path badDir = std::filesystem::temp_directory_path()
                                       / "dartsim_ui_nonexistent_save_dir_xyz"
                                       / "x.dartsim";
  const auto selectedButBadSave = ui::saveProjectWithDialog(
      engine,
      [&](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, badDir.string(), {}};
      },
      true);
  EXPECT_FALSE(selectedButBadSave.ok);
  EXPECT_EQ(
      selectedButBadSave.followUp, ui::ProjectActionFollowUp::ModalFallback);
}

TEST(DartsimProjectActions, OpenAcceptsExtensionlessProjectPath)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_extensionless_open.dartsim";
  const std::filesystem::path extensionless = path.parent_path() / path.stem();

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  const auto result = ui::openProject(engine, extensionless.string());

  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Loaded " + path.string());
  EXPECT_TRUE(engine.hasProjectPath());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_EQ(engine.objects().model().size(), 1u);

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, OpenRecentProjectTracksSuccessAndDirtyGuard)
{
  const std::filesystem::path first = std::filesystem::temp_directory_path()
                                      / "dartsim_ui_open_recent_first.dartsim";
  const std::filesystem::path second
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_open_recent_second.dartsim";
  std::filesystem::remove(first);
  std::filesystem::remove(second);

  SimEngine firstProject;
  firstProject.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(firstProject.saveProject(first.string()));

  SimEngine secondProject;
  secondProject.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(secondProject.saveProject(second.string()));

  SimEngine engine;
  ui::RecentProjectState recent;
  engine.execute(
      commands::addRigidBody(ShapeType::Capsule, translation(2.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();

  const auto blocked = ui::openRecentProject(engine, recent, first.string());
  EXPECT_FALSE(blocked.ok);
  EXPECT_EQ(blocked.message, "Unsaved changes");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(recent.paths.empty());

  engine.markProjectClean();
  const auto opened = ui::openRecentProject(engine, recent, second.string());
  EXPECT_TRUE(opened.ok);
  EXPECT_EQ(opened.message, "Loaded " + second.string());
  ASSERT_EQ(recent.paths.size(), 1u);
  EXPECT_EQ(recent.paths[0], second.string());
  EXPECT_EQ(engine.projectPath(), second.string());
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(first);
  std::filesystem::remove(second);
}

TEST(DartsimProjectActions, OpenWithDialogHandlesCancelFailureAndDirtyGuard)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();

  bool dialogCalled = false;
  const auto dirty = ui::openProjectWithDialog(
      engine, [&](const ui::ProjectFileDialogRequest&) {
        dialogCalled = true;
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, "ignored.dartsim", {}};
      });
  EXPECT_FALSE(dirty.ok);
  EXPECT_EQ(dirty.message, "Unsaved changes");
  EXPECT_FALSE(dialogCalled);
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  engine.markProjectClean();
  const auto canceled = ui::openProjectWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Canceled, {}, {}};
      });
  EXPECT_FALSE(canceled.ok);
  EXPECT_EQ(canceled.message, "Open canceled");
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto failed = ui::openProjectWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Failed, {}, "backend unavailable"};
      });
  EXPECT_FALSE(failed.ok);
  EXPECT_EQ(failed.message, "Open dialog failed: backend unavailable");
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto failedWithoutDetail = ui::openProjectWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Failed, {}, {}};
      });
  EXPECT_FALSE(failedWithoutDetail.ok);
  EXPECT_EQ(failedWithoutDetail.message, "Open dialog failed");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
}

TEST(DartsimProjectActions, DirtyProjectReplacementIsBlockedByDefault)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dirty_guard.dartsim";

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto blockedOpen = ui::openProject(engine, path.string());
  EXPECT_FALSE(blockedOpen.ok);
  EXPECT_EQ(blockedOpen.message, "Unsaved changes");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(engine.isProjectDirty());

  const auto blockedNew = ui::newProject(engine);
  EXPECT_FALSE(blockedNew.ok);
  EXPECT_EQ(blockedNew.message, "Unsaved changes");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(engine.isProjectDirty());

  const auto discardOpen
      = ui::openProject(engine, path.string(), ui::DirtyProjectPolicy::Discard);
  EXPECT_TRUE(discardOpen.ok);
  EXPECT_EQ(discardOpen.message, "Loaded " + path.string());
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.objects().model().size(), saved.objects().model().size());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, DirtyNewProjectReplacementRequiresConfirmation)
{
  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto requested = ui::requestNewProjectReplacement(engine);
  EXPECT_TRUE(requested.promptRequired);
  EXPECT_FALSE(requested.result.ok);
  EXPECT_EQ(requested.result.message, "Unsaved changes");
  EXPECT_EQ(requested.request.kind, ui::ProjectReplacementKind::NewProject);
  EXPECT_EQ(requested.request.title, "Unsaved changes");
  EXPECT_EQ(requested.request.confirmLabel, "Discard and New");
  EXPECT_EQ(requested.request.cancelLabel, "Cancel");
  EXPECT_NE(
      requested.request.message.find("Create a new project"),
      std::string::npos);
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto canceled = ui::cancelProjectReplacement(requested.request);
  EXPECT_FALSE(canceled.ok);
  EXPECT_EQ(canceled.message, "New project canceled");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(engine.isProjectDirty());

  const auto confirmed
      = ui::confirmProjectReplacement(engine, requested.request);
  EXPECT_TRUE(confirmed.ok);
  EXPECT_EQ(confirmed.message, "New project");
  EXPECT_TRUE(engine.objects().model().empty());
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_FALSE(engine.isProjectDirty());
}

TEST(DartsimProjectActions, DirtyCloseProjectReplacementRequiresConfirmation)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_confirm_close.dartsim";
  std::filesystem::remove(path);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.saveProject(path.string()));
  engine.execute(commands::setMass(engine.selection().primary(), 3.0));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto requested = ui::requestCloseProjectReplacement(engine);
  EXPECT_TRUE(requested.promptRequired);
  EXPECT_FALSE(requested.result.ok);
  EXPECT_EQ(requested.result.message, "Unsaved changes");
  EXPECT_EQ(requested.request.kind, ui::ProjectReplacementKind::CloseProject);
  EXPECT_EQ(requested.request.confirmLabel, "Discard and Close");
  EXPECT_EQ(requested.request.cancelLabel, "Cancel");
  EXPECT_NE(
      requested.request.message.find(path.filename().string()),
      std::string::npos);
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto canceled = ui::cancelProjectReplacement(requested.request);
  EXPECT_FALSE(canceled.ok);
  EXPECT_EQ(canceled.message, "Close canceled");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(engine.isProjectDirty());

  const auto confirmed
      = ui::confirmProjectReplacement(engine, requested.request);
  EXPECT_TRUE(confirmed.ok);
  EXPECT_EQ(confirmed.message, "Closed project");
  EXPECT_TRUE(engine.objects().model().empty());
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, DirtyOpenProjectReplacementRequiresConfirmation)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_confirm_open.dartsim";
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto requested
      = ui::requestOpenProjectReplacement(engine, path.string());
  EXPECT_TRUE(requested.promptRequired);
  EXPECT_FALSE(requested.result.ok);
  EXPECT_EQ(requested.result.message, "Unsaved changes");
  EXPECT_EQ(requested.request.kind, ui::ProjectReplacementKind::OpenProject);
  EXPECT_EQ(requested.request.path, path.string());
  EXPECT_EQ(requested.request.confirmLabel, "Discard and Open");
  EXPECT_NE(
      requested.request.message.find(path.filename().string()),
      std::string::npos);
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto confirmed
      = ui::confirmProjectReplacement(engine, requested.request);
  EXPECT_TRUE(confirmed.ok);
  EXPECT_EQ(confirmed.message, "Loaded " + path.string());
  EXPECT_TRUE(engine.hasProjectPath());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_EQ(engine.objects().model().size(), saved.objects().model().size());
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, DirtyConfirmedOpenFailurePreservesScene)
{
  const std::filesystem::path missing
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_missing_confirmed_open.dartsim";
  std::filesystem::remove(missing);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto requested
      = ui::requestOpenProjectReplacement(engine, missing.string());
  ASSERT_TRUE(requested.promptRequired);

  const auto confirmed
      = ui::confirmProjectReplacement(engine, requested.request);
  EXPECT_FALSE(confirmed.ok);
  EXPECT_EQ(
      confirmed.message,
      "Open failed: " + missing.string() + " does not exist");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_TRUE(engine.isProjectDirty());
}

TEST(DartsimProjectActions, DirtyConfirmedOpenAcceptsExtensionlessPath)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_confirm_extensionless.dartsim";
  const std::filesystem::path extensionless = path.parent_path() / path.stem();
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.isProjectDirty());

  const auto requested
      = ui::requestOpenProjectReplacement(engine, extensionless.string());
  ASSERT_TRUE(requested.promptRequired);
  EXPECT_EQ(requested.request.path, extensionless.string());

  const auto confirmed
      = ui::confirmProjectReplacement(engine, requested.request);
  EXPECT_TRUE(confirmed.ok);
  EXPECT_EQ(confirmed.message, "Loaded " + path.string());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_EQ(engine.objects().model().size(), saved.objects().model().size());
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, CleanProjectReplacementAppliesImmediately)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_clean_replacement.dartsim";
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.markProjectClean();
  auto requested = ui::requestOpenProjectReplacement(engine, path.string());
  EXPECT_FALSE(requested.promptRequired);
  EXPECT_TRUE(requested.result.ok);
  EXPECT_EQ(requested.result.message, "Loaded " + path.string());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_FALSE(engine.isProjectDirty());

  requested = ui::requestNewProjectReplacement(engine);
  EXPECT_FALSE(requested.promptRequired);
  EXPECT_TRUE(requested.result.ok);
  EXPECT_EQ(requested.result.message, "New project");
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_TRUE(engine.objects().model().empty());

  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.saveProject(path.string()));
  requested = ui::requestCloseProjectReplacement(engine);
  EXPECT_FALSE(requested.promptRequired);
  EXPECT_TRUE(requested.result.ok);
  EXPECT_EQ(requested.result.message, "Closed project");
  EXPECT_FALSE(engine.hasProjectPath());
  EXPECT_TRUE(engine.objects().model().empty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, ProjectReplacementConfirmRejectsOpenMacro)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_confirm_macro_guard.dartsim";
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto requestedNew = ui::requestNewProjectReplacement(engine);
  const auto requestedOpen
      = ui::requestOpenProjectReplacement(engine, path.string());
  const auto requestedClose = ui::requestCloseProjectReplacement(engine);
  ASSERT_TRUE(requestedNew.promptRequired);
  ASSERT_TRUE(requestedOpen.promptRequired);
  ASSERT_TRUE(requestedClose.promptRequired);

  engine.commands().beginMacro("Pending Transaction");
  const auto confirmedNew
      = ui::confirmProjectReplacement(engine, requestedNew.request);
  EXPECT_FALSE(confirmedNew.ok);
  EXPECT_EQ(
      confirmedNew.message,
      "Cannot create a new project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto confirmedOpen
      = ui::confirmProjectReplacement(engine, requestedOpen.request);
  EXPECT_FALSE(confirmedOpen.ok);
  EXPECT_EQ(
      confirmedOpen.message,
      "Cannot load a project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), dirtyModel);

  const auto confirmedClose
      = ui::confirmProjectReplacement(engine, requestedClose.request);
  EXPECT_FALSE(confirmedClose.ok);
  EXPECT_EQ(
      confirmedClose.message,
      "Cannot close the project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_TRUE(engine.commands().inMacro());
  engine.commands().endMacro();

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, ProjectReplacementActionsRejectOpenMacro)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_macro_guard.dartsim";
  std::filesystem::remove(path);

  SimEngine saved;
  saved.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  ASSERT_TRUE(saved.saveProject(path.string()));

  SimEngine engine;
  engine.commands().beginMacro("Open Transaction");
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  const SceneModel transactionModel = engine.objects().model();

  const auto blockedNew = ui::newProject(engine);
  EXPECT_FALSE(blockedNew.ok);
  EXPECT_EQ(
      blockedNew.message,
      "Cannot create a new project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), transactionModel);

  const auto blockedOpen = ui::openProject(engine, path.string());
  EXPECT_FALSE(blockedOpen.ok);
  EXPECT_EQ(
      blockedOpen.message, "Cannot load a project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), transactionModel);
  EXPECT_TRUE(engine.commands().inMacro());

  const auto blockedClose = ui::closeProject(engine);
  EXPECT_FALSE(blockedClose.ok);
  EXPECT_EQ(
      blockedClose.message,
      "Cannot close the project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), transactionModel);
  EXPECT_TRUE(engine.commands().inMacro());

  bool dialogCalled = false;
  const auto blockedDialog = ui::openProjectWithDialog(
      engine, [&](const ui::ProjectFileDialogRequest&) {
        dialogCalled = true;
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, path.string(), {}};
      });
  EXPECT_FALSE(blockedDialog.ok);
  EXPECT_EQ(
      blockedDialog.message,
      "Cannot load a project during an edit transaction");
  EXPECT_FALSE(dialogCalled);
  EXPECT_EQ(engine.objects().model(), transactionModel);
  EXPECT_TRUE(engine.commands().inMacro());

  const auto blockedRequestedNew = ui::requestNewProjectReplacement(engine);
  EXPECT_FALSE(blockedRequestedNew.promptRequired);
  EXPECT_FALSE(blockedRequestedNew.result.ok);
  EXPECT_EQ(
      blockedRequestedNew.result.message,
      "Cannot create a new project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), transactionModel);

  const auto blockedRequestedOpen
      = ui::requestOpenProjectReplacement(engine, path.string());
  EXPECT_FALSE(blockedRequestedOpen.promptRequired);
  EXPECT_FALSE(blockedRequestedOpen.result.ok);
  EXPECT_EQ(
      blockedRequestedOpen.result.message,
      "Cannot load a project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), transactionModel);

  const auto blockedRequestedClose = ui::requestCloseProjectReplacement(engine);
  EXPECT_FALSE(blockedRequestedClose.promptRequired);
  EXPECT_FALSE(blockedRequestedClose.result.ok);
  EXPECT_EQ(
      blockedRequestedClose.result.message,
      "Cannot close the project during an edit transaction");
  EXPECT_EQ(engine.objects().model(), transactionModel);

  engine.commands().endMacro();
  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, SaveWithDialogChoosesPathForUnsavedProject)
{
  const std::filesystem::path selected
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_save";
  const std::filesystem::path expected = selected.string() + ".dartsim";
  std::filesystem::remove(selected);
  std::filesystem::remove(expected);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_FALSE(engine.hasProjectPath());

  bool dialogCalled = false;
  int parentToken = 0;
  void* parentWindow = &parentToken;
  const auto result = ui::saveProjectWithDialog(
      engine,
      [&](const ui::ProjectFileDialogRequest& request) {
        dialogCalled = true;
        EXPECT_EQ(request.kind, ui::ProjectFileDialogKind::Save);
        EXPECT_EQ(request.defaultName, ui::kDefaultProjectPath);
        EXPECT_EQ(request.parentNativeWindow, parentWindow);
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, selected.string(), {}};
      },
      /*forceDialog=*/false,
      parentWindow);

  EXPECT_TRUE(dialogCalled);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Saved " + expected.string());
  EXPECT_EQ(engine.projectPath(), expected.string());
  EXPECT_TRUE(std::filesystem::exists(expected));
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(selected);
  std::filesystem::remove(expected);
}

TEST(DartsimProjectActions, SaveWithDialogCanForceSaveAsForExistingProject)
{
  const std::filesystem::path original
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_save_original.dartsim";
  const std::filesystem::path saveAs
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_save_as.dartsim";
  std::filesystem::remove(original);
  std::filesystem::remove(saveAs);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.saveProject(original.string()));

  bool dialogCalled = false;
  const auto result = ui::saveProjectWithDialog(
      engine,
      [&](const ui::ProjectFileDialogRequest& request) {
        dialogCalled = true;
        EXPECT_EQ(request.kind, ui::ProjectFileDialogKind::Save);
        EXPECT_EQ(
            request.defaultPath,
            std::filesystem::temp_directory_path().string());
        EXPECT_EQ(request.defaultName, original.filename().string());
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, saveAs.string(), {}};
      },
      /*forceDialog=*/true);

  EXPECT_TRUE(dialogCalled);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.message, "Saved " + saveAs.string());
  EXPECT_EQ(engine.projectPath(), saveAs.string());
  EXPECT_TRUE(std::filesystem::exists(saveAs));

  std::filesystem::remove(original);
  std::filesystem::remove(saveAs);
}

TEST(DartsimProjectActions, SaveWithDialogHandlesCancelAndExistingPathSave)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_dialog_existing_save.dartsim";
  std::filesystem::remove(path);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.saveProject(path.string()));
  engine.execute(commands::setMass(engine.selection().primary(), 4.0));

  bool dialogCalled = false;
  const auto saved = ui::saveProjectWithDialog(
      engine,
      [&](const ui::ProjectFileDialogRequest&) {
        dialogCalled = true;
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Canceled, {}, {}};
      },
      /*forceDialog=*/false);
  EXPECT_FALSE(dialogCalled);
  EXPECT_TRUE(saved.ok);
  EXPECT_EQ(saved.message, "Saved " + path.string());
  EXPECT_FALSE(engine.isProjectDirty());

  engine.newProject();
  engine.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));
  const auto canceled = ui::saveProjectWithDialog(
      engine,
      [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Canceled, {}, {}};
      },
      /*forceDialog=*/false);
  EXPECT_FALSE(canceled.ok);
  EXPECT_EQ(canceled.message, "Save canceled");
  EXPECT_TRUE(engine.isProjectDirty());

  std::filesystem::remove(path);
}

TEST(DartsimProjectActions, DialogHelpersRejectUnavailableAndEmptySelections)
{
  const std::filesystem::path invalid
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_invalid_open.dartsim";
  std::filesystem::remove(invalid);
  {
    std::ofstream file(invalid);
    file << "not a dartsim project\n";
  }

  SimEngine engine;
  EXPECT_FALSE(ui::openProjectWithDialog(engine, ui::ProjectFileDialog{}).ok);
  EXPECT_FALSE(
      ui::saveProjectWithDialog(engine, ui::ProjectFileDialog{}, true).ok);

  const auto emptyOpen = ui::openProjectWithDialog(
      engine, [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, {}, {}};
      });
  EXPECT_FALSE(emptyOpen.ok);
  EXPECT_EQ(emptyOpen.message, "Open canceled");

  const auto emptySave = ui::saveProjectWithDialog(
      engine,
      [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, {}, {}};
      },
      true);
  EXPECT_FALSE(emptySave.ok);
  EXPECT_EQ(emptySave.message, "Save canceled");

  const auto failedSave = ui::saveProjectWithDialog(
      engine,
      [](const ui::ProjectFileDialogRequest&) {
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Failed, {}, {}};
      },
      true);
  EXPECT_FALSE(failedSave.ok);
  EXPECT_EQ(failedSave.message, "Save dialog failed");

  const auto invalidOpen = ui::openProject(
      engine, invalid.string(), ui::DirtyProjectPolicy::Discard);
  EXPECT_FALSE(invalidOpen.ok);
  EXPECT_EQ(
      invalidOpen.message,
      "Open failed: " + invalid.string() + " is not a dartsim project");

  std::filesystem::remove(invalid);
}

TEST(DartsimProjectActions, OpenReportsEmptyAndExtensionlessMissingPaths)
{
  const std::filesystem::path missingExtensionless
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_missing_extensionless";
  std::filesystem::remove(missingExtensionless);
  std::filesystem::remove(missingExtensionless.string() + ".dartsim");

  SimEngine engine;
  const auto empty = ui::openProject(engine, {});
  EXPECT_FALSE(empty.ok);
  EXPECT_EQ(empty.message, "Open failed: empty path");

  const auto missing = ui::openProject(engine, missingExtensionless.string());
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(
      missing.message,
      "Open failed: " + missingExtensionless.string() + " does not exist");
}

TEST(DartsimProjectActions, OpenReportsFailureWithoutReplacingScene)
{
  const std::filesystem::path missing
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_missing.dartsim";
  std::filesystem::remove(missing);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  engine.markProjectClean();
  const SceneModel before = engine.objects().model();

  const auto result = ui::openProject(engine, missing.string());
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(
      result.message, "Open failed: " + missing.string() + " does not exist");
  EXPECT_EQ(engine.objects().model(), before);
}

TEST(DartsimProjectActions, OpenReportsDirectoryAsInvalidProjectPath)
{
  const std::filesystem::path directory
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_open_directory";
  std::filesystem::remove_all(directory);
  ASSERT_TRUE(std::filesystem::create_directories(directory));

  SimEngine engine;
  const auto result = ui::openProject(engine, directory.string());
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(
      result.message, "Open failed: " + directory.string() + " is a directory");

  std::filesystem::remove_all(directory);
}

TEST(DartsimProjectActions, SaveReportsFailureWithoutChangingProjectState)
{
  const std::filesystem::path saveDir
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_missing_dir";
  const std::filesystem::path projectPath = saveDir / "project.dartsim";
  const std::filesystem::path ignoredDefault
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_ignored_failure_default.dartsim";
  std::filesystem::remove_all(saveDir);
  std::filesystem::remove(ignoredDefault);
  ASSERT_TRUE(std::filesystem::create_directories(saveDir));

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Box, translation(0.0, 0.0, 1.0)));
  ASSERT_TRUE(engine.saveProject(projectPath.string()));
  std::filesystem::remove_all(saveDir);

  engine.execute(commands::setMass(engine.selection().primary(), 3.0));
  const SceneModel dirtyModel = engine.objects().model();
  ASSERT_TRUE(engine.isProjectDirty());

  const auto result = ui::saveProject(engine, ignoredDefault.string());
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message, "Save failed");
  EXPECT_EQ(engine.projectPath(), projectPath.string());
  EXPECT_TRUE(engine.isProjectDirty());
  EXPECT_EQ(engine.objects().model(), dirtyModel);
  EXPECT_FALSE(std::filesystem::exists(ignoredDefault));

  std::filesystem::remove_all(saveDir);
  std::filesystem::remove(ignoredDefault);
}

TEST(DartsimProjectActions, SaveProjectUsesDefaultPathForUnsavedProject)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_ui_project_actions_default_save.dartsim";
  std::filesystem::remove(path);

  SimEngine engine;
  engine.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(1.0, 0.0, 1.0)));

  const auto saved = ui::saveProject(engine, path.string());
  EXPECT_TRUE(saved.ok);
  EXPECT_EQ(saved.message, "Saved " + path.string());
  EXPECT_EQ(engine.projectPath(), path.string());
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_FALSE(engine.isProjectDirty());

  std::filesystem::remove(path);
}
