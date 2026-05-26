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
  const auto result = ui::saveProjectWithDialog(
      engine,
      [&](const ui::ProjectFileDialogRequest& request) {
        dialogCalled = true;
        EXPECT_EQ(request.kind, ui::ProjectFileDialogKind::Save);
        EXPECT_EQ(request.defaultName, ui::kDefaultProjectPath);
        return ui::ProjectFileDialogResult{
            ui::ProjectFileDialogStatus::Selected, selected.string(), {}};
      },
      /*forceDialog=*/false);

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
