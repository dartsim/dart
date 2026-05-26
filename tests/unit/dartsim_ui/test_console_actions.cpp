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

#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_ui/console_actions.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

using namespace dartsim;

namespace {

std::filesystem::path tempProjectPath(std::string name)
{
  const auto suffix
      = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path()
         / ("dartsim_console_actions_" + std::to_string(suffix) + "_"
            + std::move(name));
}

const SceneObject* objectByName(
    const SimEngine& engine, const std::string& name)
{
  for (const ObjectId id : engine.objects().model().allIds()) {
    const SceneObject* object = engine.objects().model().find(id);
    if (object != nullptr && object->name == name) {
      return object;
    }
  }
  return nullptr;
}

} // namespace

TEST(DartsimConsoleActions, TokenizesQuotedArgumentsAndReportsErrors)
{
  std::vector<std::string> tokens;
  EXPECT_TRUE(ui::tokenizeConsoleCommand("rename \"base link\"", tokens).ok);
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens[0], "rename");
  EXPECT_EQ(tokens[1], "base link");

  EXPECT_TRUE(
      ui::tokenizeConsoleCommand("open path\\ with\\ spaces", tokens).ok);
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens[1], "path with spaces");

  EXPECT_TRUE(ui::tokenizeConsoleCommand("rename \"\"", tokens).ok);
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens[1], "");

  EXPECT_TRUE(ui::tokenizeConsoleCommand("select body\\", tokens).ok);
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens[1], "body\\");

  const auto unclosed = ui::tokenizeConsoleCommand("rename \"base", tokens);
  EXPECT_FALSE(unclosed.ok);
  EXPECT_EQ(unclosed.message, "Unclosed quote");
}

TEST(DartsimConsoleActions, HelpStatusAndInvalidCommandsAreStable)
{
  SimEngine engine;
  const auto help = ui::applyConsoleCommand(engine, "help");
  EXPECT_TRUE(help.ok);
  EXPECT_NE(help.message.find("create <kind>"), std::string::npos);
  EXPECT_NE(help.message.find("record <on|off>"), std::string::npos);

  const auto status = ui::applyConsoleCommand(engine, "status");
  EXPECT_TRUE(status.ok);
  EXPECT_NE(status.message.find("Edit Mode"), std::string::npos);
  EXPECT_NE(status.message.find("selected 0"), std::string::npos);
  EXPECT_NE(status.message.find("clean"), std::string::npos);

  EXPECT_EQ(ui::applyConsoleCommand(engine, "").message, "No command");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "unknown").message,
      "Unknown command: unknown");
  EXPECT_EQ(ui::applyConsoleCommand(engine, "help now").message, "Usage: help");
}

TEST(DartsimConsoleActions, CreatesSelectsRenamesHidesShowsAndDeletesObjects)
{
  SimEngine engine;
  const auto created = ui::applyConsoleCommand(engine, "CREATE box");
  EXPECT_TRUE(created.ok);
  EXPECT_EQ(created.message, "Added box");
  const ObjectId body = engine.selection().primary();
  ASSERT_NE(body, kNoObject);

  const auto renamed = ui::applyConsoleCommand(engine, "rename \"base link\"");
  EXPECT_TRUE(renamed.ok);
  EXPECT_EQ(renamed.message, "Renamed Body to base link");
  ASSERT_NE(objectByName(engine, "base link"), nullptr);

  EXPECT_TRUE(ui::applyConsoleCommand(engine, "clear-selection").ok);
  EXPECT_TRUE(ui::applyConsoleCommand(engine, "clear-selection").ok);
  EXPECT_TRUE(engine.selection().empty());

  EXPECT_TRUE(ui::applyConsoleCommand(engine, "select \"base link\"").ok);
  EXPECT_EQ(engine.selection().primary(), body);
  EXPECT_TRUE(ui::applyConsoleCommand(engine, "select \"base link\"").ok);

  EXPECT_TRUE(ui::applyConsoleCommand(engine, "select-add \"base link\"").ok);
  EXPECT_EQ(engine.selection().selected().size(), 1u);
  EXPECT_TRUE(ui::applyConsoleCommand(engine, "deselect \"base link\"").ok);
  EXPECT_TRUE(engine.selection().empty());
  EXPECT_TRUE(ui::applyConsoleCommand(engine, "deselect \"base link\"").ok);
  EXPECT_TRUE(ui::applyConsoleCommand(engine, "select \"base link\"").ok);

  const auto hidden = ui::applyConsoleCommand(engine, "hide selected");
  EXPECT_TRUE(hidden.ok);
  EXPECT_FALSE(engine.objects().model().find(body)->visible);
  const auto hiddenAgain = ui::applyConsoleCommand(engine, "hide selected");
  EXPECT_TRUE(hiddenAgain.ok);
  EXPECT_EQ(hiddenAgain.message, "Object already hidden");

  const auto shown = ui::applyConsoleCommand(
      engine, "show " + std::to_string(static_cast<unsigned long long>(body)));
  EXPECT_TRUE(shown.ok);
  EXPECT_TRUE(engine.objects().model().find(body)->visible);
  const auto shownAgain = ui::applyConsoleCommand(engine, "show selected");
  EXPECT_TRUE(shownAgain.ok);
  EXPECT_EQ(shownAgain.message, "Object already shown");

  const auto deleted = ui::applyConsoleCommand(engine, "delete");
  EXPECT_TRUE(deleted.ok);
  EXPECT_FALSE(engine.objects().model().contains(body));
  EXPECT_TRUE(engine.undo());
  EXPECT_TRUE(engine.objects().model().contains(body));
}

TEST(DartsimConsoleActions, SupportsProjectLifecycleWithDirtyGuards)
{
  SimEngine engine;
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create sphere").ok);

  const auto blockedNew = ui::applyConsoleCommand(engine, "new");
  EXPECT_FALSE(blockedNew.ok);
  EXPECT_EQ(blockedNew.message, "Unsaved changes");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "new now").message,
      "Usage: new [--discard]");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "open").message,
      "Usage: open <path> [--discard]");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "save a b").message,
      "Usage: save [path]");

  const auto projectPath
      = tempProjectPath("dartsim_console_actions_project.dartsim");
  std::filesystem::remove(projectPath);
  const auto saved = ui::applyConsoleCommand(
      engine, "save \"" + projectPath.string() + "\"");
  EXPECT_TRUE(saved.ok);
  EXPECT_TRUE(engine.hasProjectPath());
  EXPECT_FALSE(engine.isProjectDirty());

  const auto normalizedPath
      = tempProjectPath("dartsim_console_actions_save_as");
  std::filesystem::remove(normalizedPath);
  std::filesystem::remove(normalizedPath.string() + ".dartsim");
  const auto savedWithExtension = ui::applyConsoleCommand(
      engine, "save \"" + normalizedPath.string() + "\"");
  EXPECT_TRUE(savedWithExtension.ok);
  EXPECT_EQ(engine.projectPath(), normalizedPath.string() + ".dartsim");

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create capsule").ok);
  const auto blockedOpen = ui::applyConsoleCommand(
      engine, "open \"" + projectPath.string() + "\"");
  EXPECT_FALSE(blockedOpen.ok);
  EXPECT_EQ(blockedOpen.message, "Unsaved changes");

  const auto opened = ui::applyConsoleCommand(
      engine, "open \"" + projectPath.string() + "\" --discard");
  EXPECT_TRUE(opened.ok);
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.objects().model().size(), 1u);

  const auto discarded = ui::applyConsoleCommand(engine, "new --discard");
  EXPECT_TRUE(discarded.ok);
  EXPECT_TRUE(engine.objects().model().empty());

  std::filesystem::remove(projectPath);
  std::filesystem::remove(normalizedPath.string() + ".dartsim");
}

TEST(DartsimConsoleActions, DrivesSimulationRecordingAndReplay)
{
  SimEngine engine;
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create box").ok);

  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "pause").message,
      "Simulation already paused");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "reset").message, "Already in Edit Mode");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "replay 0").message,
      "Replay seek failed");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "mode invalid").message,
      "Unknown mode: invalid");

  const auto recordOn = ui::applyConsoleCommand(engine, "record on");
  EXPECT_TRUE(recordOn.ok);
  EXPECT_TRUE(engine.isRecording());

  const auto stepped = ui::applyConsoleCommand(engine, "step 2");
  EXPECT_TRUE(stepped.ok);
  EXPECT_EQ(engine.simulation().frameCount(), 2u);

  const auto createWhileLocked
      = ui::applyConsoleCommand(engine, "create sphere");
  EXPECT_FALSE(createWhileLocked.ok);
  EXPECT_EQ(createWhileLocked.message, "Scene locked");

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "play").ok);
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "step").message, "Pause before stepping");
  EXPECT_TRUE(ui::applyConsoleCommand(engine, "pause").ok);

  const auto recordOff = ui::applyConsoleCommand(engine, "record off");
  EXPECT_TRUE(recordOff.ok);
  EXPECT_FALSE(engine.player().empty());

  const auto replay = ui::applyConsoleCommand(engine, "replay 0");
  EXPECT_TRUE(replay.ok);

  const auto editMode = ui::applyConsoleCommand(engine, "mode edit");
  EXPECT_TRUE(editMode.ok);
  EXPECT_TRUE(engine.canEditScene());
}

TEST(DartsimConsoleActions, CreatesContextSensitiveModelElements)
{
  SimEngine engine;
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "create root-link").message,
      "Select a MultiBody");

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create multibody").ok);
  const ObjectId multibody = engine.selection().primary();
  ASSERT_NE(multibody, kNoObject);

  const auto root = ui::applyConsoleCommand(engine, "create root-link");
  EXPECT_TRUE(root.ok);
  const ObjectId rootLink = engine.selection().primary();
  ASSERT_NE(rootLink, kNoObject);
  EXPECT_NE(rootLink, multibody);

  const auto child = ui::applyConsoleCommand(engine, "create revolute-link");
  EXPECT_TRUE(child.ok);
  const ObjectId childLink = engine.selection().primary();
  ASSERT_NE(childLink, kNoObject);
  EXPECT_EQ(engine.objects().model().find(childLink)->parentLink, rootLink);

  const auto frame = ui::applyConsoleCommand(engine, "create fixed-frame");
  EXPECT_TRUE(frame.ok);
  const ObjectId fixedFrame = engine.selection().primary();
  ASSERT_NE(fixedFrame, kNoObject);
  EXPECT_EQ(engine.objects().model().find(fixedFrame)->parent, childLink);
}

TEST(DartsimConsoleActions, RejectsMissingAndAmbiguousObjectTargets)
{
  SimEngine engine;
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "select missing").message,
      "Object not found: missing");
  EXPECT_EQ(ui::applyConsoleCommand(engine, "delete").message, "No selection");

  engine.execute(commands::addMultiBody("arm_a"));
  const ObjectId armA = engine.selection().primary();
  engine.execute(commands::addLink(armA, kNoObject, JointKind::Fixed, "link"));
  engine.execute(commands::addMultiBody("arm_b"));
  const ObjectId armB = engine.selection().primary();
  engine.execute(commands::addLink(armB, kNoObject, JointKind::Fixed, "link"));

  const auto ambiguous = ui::applyConsoleCommand(engine, "select link");
  EXPECT_FALSE(ambiguous.ok);
  EXPECT_EQ(ambiguous.message, "Ambiguous object name: link");

  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "step many").message,
      "Invalid step count");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "step 1001").message,
      "Step count must be 1000 or less");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "record maybe").message,
      "Usage: record <on|off>");
}

TEST(DartsimConsoleActions, NumericObjectNamesFallbackAfterMissingIds)
{
  SimEngine engine;
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create box").ok);
  const ObjectId object = engine.selection().primary();
  ASSERT_NE(object, kNoObject);

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "rename 9999").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "clear-selection").ok);

  const auto selected = ui::applyConsoleCommand(engine, "select 9999");
  EXPECT_TRUE(selected.ok);
  EXPECT_EQ(engine.selection().primary(), object);
}
