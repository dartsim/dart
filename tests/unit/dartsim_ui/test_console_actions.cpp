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
#include <dartsim_ui/watch_actions.hpp>
#include <gtest/gtest.h>

#include <algorithm>
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
  EXPECT_NE(
      help.message.find("restart (stay in Simulation Mode)"),
      std::string::npos);
  EXPECT_NE(
      help.message.find("reset (return to Edit Mode)"), std::string::npos);
  EXPECT_NE(help.message.find("record <on|off>"), std::string::npos);
  EXPECT_NE(
      help.message.find("replay <first|previous|next|last|frame>"),
      std::string::npos);
  EXPECT_NE(help.message.find("reparent-link"), std::string::npos);
  EXPECT_EQ(
      help.message.find("watch [target|selection|clear|sample]"),
      std::string::npos);

  ui::WatchState watch;
  const auto watchHelp = ui::applyConsoleCommand(engine, watch, "help");
  EXPECT_TRUE(watchHelp.ok);
  EXPECT_NE(
      watchHelp.message.find("watch [target|selection|clear|sample]"),
      std::string::npos);
  EXPECT_NE(watchHelp.message.find("watch signal"), std::string::npos);

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
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "watch").message,
      "Watch state unavailable");
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
      ui::applyConsoleCommand(engine, "restart").message,
      "Enter Simulation Mode first");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "replay 0").message,
      "Replay seek failed");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "replay next").message,
      "No replay loaded");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "mode invalid").message,
      "Unknown mode: invalid");

  const auto recordOn = ui::applyConsoleCommand(engine, "record on");
  EXPECT_TRUE(recordOn.ok);
  EXPECT_TRUE(engine.isRecording());

  const auto stepped = ui::applyConsoleCommand(engine, "step 2");
  EXPECT_TRUE(stepped.ok);
  EXPECT_EQ(engine.simulation().frameCount(), 2u);

  const auto restarted = ui::applyConsoleCommand(engine, "restart");
  EXPECT_TRUE(restarted.ok);
  EXPECT_EQ(restarted.message, "Simulation restarted");
  EXPECT_EQ(engine.simulation().mode(), SimulationController::Mode::Simulation);
  EXPECT_EQ(engine.simulation().frameCount(), 0u);

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
  EXPECT_EQ(replay.message, "Replay frame 0");

  const auto replayNext = ui::applyConsoleCommand(engine, "replay next");
  EXPECT_TRUE(replayNext.ok);
  EXPECT_EQ(replayNext.message, "Replay frame 1");
  EXPECT_EQ(engine.player().currentIndex(), 1u);

  const auto replayLast = ui::applyConsoleCommand(engine, "replay last");
  EXPECT_TRUE(replayLast.ok);
  EXPECT_EQ(
      engine.player().currentIndex(),
      engine.player().recording().frameCount() - 1);

  const auto replayPrevious
      = ui::applyConsoleCommand(engine, "replay previous");
  EXPECT_TRUE(replayPrevious.ok);
  EXPECT_EQ(
      engine.player().currentIndex(),
      engine.player().recording().frameCount() - 2);

  const auto replayFirst = ui::applyConsoleCommand(engine, "replay first");
  EXPECT_TRUE(replayFirst.ok);
  EXPECT_EQ(engine.player().currentIndex(), 0u);
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "replay prev").message,
      "Already at first replay frame");

  const auto editMode = ui::applyConsoleCommand(engine, "mode edit");
  EXPECT_TRUE(editMode.ok);
  EXPECT_TRUE(engine.canEditScene());
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "replay 0").message,
      "Enter Simulation Mode before replaying");
}

TEST(DartsimConsoleActions, DrivesWatchCommandsThroughSessionState)
{
  SimEngine engine;
  ui::WatchState watch;

  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "create box").ok);
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.markProjectClean();
  const std::size_t undoCount = engine.commands().undoCount();
  const auto revision = engine.commands().currentRevision();

  const auto emptyStatus = ui::applyConsoleCommand(engine, watch, "watch");
  EXPECT_TRUE(emptyStatus.ok);
  EXPECT_EQ(emptyStatus.message, "No watched objects");

  const auto watched
      = ui::applyConsoleCommand(engine, watch, "watch selection");
  EXPECT_TRUE(watched.ok);
  EXPECT_EQ(watched.message, "Watching selection");
  ASSERT_EQ(watch.targets.size(), 1u);
  EXPECT_EQ(watch.targets[0].id, box);
  EXPECT_FALSE(engine.isProjectDirty());
  EXPECT_EQ(engine.commands().undoCount(), undoCount);
  EXPECT_EQ(engine.commands().currentRevision(), revision);

  const auto duplicate
      = ui::applyConsoleCommand(engine, watch, "watch selected");
  EXPECT_FALSE(duplicate.ok);
  EXPECT_EQ(duplicate.message, "Object already watched");

  const auto sampled = ui::applyConsoleCommand(engine, watch, "watch sample");
  EXPECT_TRUE(sampled.ok);
  EXPECT_EQ(sampled.message, "Recorded watch sample");
  EXPECT_FALSE(watch.series.empty());

  const auto disabledHeight
      = ui::applyConsoleCommand(engine, watch, "watch signal z off");
  EXPECT_TRUE(disabledHeight.ok);
  EXPECT_EQ(disabledHeight.message, "Disabled watch signal: z");
  EXPECT_TRUE(
      std::none_of(
          watch.series.begin(),
          watch.series.end(),
          [](const ui::WatchSeries& series) {
            return series.kind == ui::WatchValueKind::TranslationZ;
          }));

  const auto enabledX
      = ui::applyConsoleCommand(engine, watch, "watch signal x on");
  EXPECT_TRUE(enabledX.ok);
  EXPECT_EQ(enabledX.message, "Enabled watch signal: x");
  const auto enabledSensorRange
      = ui::applyConsoleCommand(engine, watch, "watch signal sensor-range on");
  EXPECT_TRUE(enabledSensorRange.ok);
  EXPECT_EQ(enabledSensorRange.message, "Enabled watch signal: sensor range");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, watch, "watch signal unknown on").message,
      "Unknown watch signal: unknown");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, watch, "watch signal x maybe").message,
      "Usage: watch signal <signal> <on|off>");
  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch sample").ok);
  EXPECT_TRUE(
      std::any_of(
          watch.series.begin(),
          watch.series.end(),
          [box](const ui::WatchSeries& series) {
            return series.object == box
                   && series.kind == ui::WatchValueKind::TranslationX;
          }));

  const auto disabledFrame
      = ui::applyConsoleCommand(engine, watch, "watch signal frame off");
  EXPECT_TRUE(disabledFrame.ok);
  EXPECT_EQ(disabledFrame.message, "Disabled watch signal: Frame");
  EXPECT_TRUE(
      std::none_of(
          watch.series.begin(),
          watch.series.end(),
          [](const ui::WatchSeries& series) {
            return series.object == kNoObject
                   && series.kind == ui::WatchValueKind::FrameCount;
          }));
  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch sample").ok);
  EXPECT_TRUE(
      std::none_of(
          watch.series.begin(),
          watch.series.end(),
          [](const ui::WatchSeries& series) {
            return series.object == kNoObject
                   && series.kind == ui::WatchValueKind::FrameCount;
          }));
  const auto enabledFrame
      = ui::applyConsoleCommand(engine, watch, "watch signal frame on");
  EXPECT_TRUE(enabledFrame.ok);
  EXPECT_EQ(enabledFrame.message, "Enabled watch signal: Frame");

  const std::string boxTarget
      = std::to_string(static_cast<unsigned long long>(box));
  const auto deleted = ui::applyConsoleCommand(engine, watch, "delete");
  EXPECT_TRUE(deleted.ok);
  const ui::WatchStatus missingStatus = ui::buildWatchStatus(watch, engine);
  ASSERT_EQ(missingStatus.rows.size(), 1u);
  EXPECT_FALSE(missingStatus.rows[0].exists);

  const auto removed
      = ui::applyConsoleCommand(engine, watch, "unwatch " + boxTarget);
  EXPECT_TRUE(removed.ok);
  EXPECT_TRUE(watch.targets.empty());
  EXPECT_TRUE(
      std::none_of(
          watch.series.begin(),
          watch.series.end(),
          [box](const ui::WatchSeries& series) {
            return series.object == box;
          }));

  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "create sphere").ok);
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, watch, "unwatch selected").message,
      "Object was not watched");

  const auto cleared = ui::applyConsoleCommand(engine, watch, "watch clear");
  EXPECT_TRUE(cleared.ok);
  EXPECT_TRUE(watch.targets.empty());
  EXPECT_TRUE(watch.series.empty());
}

TEST(DartsimConsoleActions, DrivesWatchPresetCommandsThroughWorkspace)
{
  SimEngine engine;
  ui::WatchState watch;

  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "create box").ok);
  const ObjectId box = engine.selection().primary();
  ASSERT_NE(box, kNoObject);
  engine.markProjectClean();

  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch selection").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch signal x on").ok);

  const auto saved
      = ui::applyConsoleCommand(engine, watch, "watch save-preset motion");
  EXPECT_TRUE(saved.ok);
  EXPECT_EQ(saved.message, "Saved watch preset: motion");
  EXPECT_TRUE(engine.isProjectDirty());
  ASSERT_EQ(engine.objects().model().workspace.watchPresets.size(), 1u);

  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch clear").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch signal x off").ok);
  const auto applied
      = ui::applyConsoleCommand(engine, watch, "watch preset motion");
  EXPECT_TRUE(applied.ok);
  EXPECT_EQ(applied.message, "Applied watch preset: motion");
  ASSERT_EQ(watch.targets.size(), 1u);
  EXPECT_EQ(watch.targets[0].id, box);
  EXPECT_TRUE(
      std::find(
          watch.chartSignals.begin(),
          watch.chartSignals.end(),
          ui::WatchValueKind::TranslationX)
      != watch.chartSignals.end());

  const auto missing
      = ui::applyConsoleCommand(engine, watch, "watch preset missing");
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.message, "Unknown watch preset: missing");

  const auto deleted
      = ui::applyConsoleCommand(engine, watch, "watch delete-preset motion");
  EXPECT_TRUE(deleted.ok);
  EXPECT_EQ(deleted.message, "Deleted watch preset: motion");
  EXPECT_TRUE(engine.objects().model().workspace.watchPresets.empty());

  EXPECT_EQ(
      ui::applyConsoleCommand(engine, watch, "watch save-preset").message,
      "Usage: watch [target|selection|clear|sample] or watch signal <signal> "
      "<on|off> or watch <save-preset|preset|delete-preset> <name>");
}

TEST(DartsimConsoleActions, DrivesRelationshipCommandsFromSelection)
{
  SimEngine engine;

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create free-frame").ok);
  const ObjectId child = engine.selection().primary();
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "rename child").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create free-frame").ok);
  const ObjectId parent = engine.selection().primary();
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "rename parent").ok);
  ASSERT_NE(child, kNoObject);
  ASSERT_NE(parent, kNoObject);

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "clear-selection").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select child").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select-add parent").ok);

  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "attach now").message, "Usage: attach");
  const auto attached = ui::applyConsoleCommand(engine, "attach");
  EXPECT_TRUE(attached.ok);
  EXPECT_EQ(attached.message, "Attached child to parent");
  ASSERT_NE(engine.objects().model().find(child), nullptr);
  EXPECT_EQ(engine.objects().model().find(child)->parent, parent);

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select child").ok);
  const auto detached = ui::applyConsoleCommand(engine, "detach");
  EXPECT_TRUE(detached.ok);
  EXPECT_EQ(detached.message, "Detached child");
  ASSERT_NE(engine.objects().model().find(child), nullptr);
  EXPECT_EQ(engine.objects().model().find(child)->parent, kNoObject);
}

TEST(DartsimConsoleActions, DrivesLinkRelationshipCommandsFromSelection)
{
  SimEngine engine;

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create multibody").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create root-link").ok);
  const ObjectId base = engine.selection().primary();
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "rename base").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create revolute-link").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "rename forearm").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create revolute-link").ok);
  const ObjectId tool = engine.selection().primary();
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "rename tool").ok);
  ASSERT_NE(base, kNoObject);
  ASSERT_NE(tool, kNoObject);

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "clear-selection").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select tool").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select-add base").ok);
  const auto reparented = ui::applyConsoleCommand(engine, "reparent-link");
  EXPECT_TRUE(reparented.ok);
  EXPECT_EQ(reparented.message, "Reparented tool to base");
  ASSERT_NE(engine.objects().model().find(tool), nullptr);
  EXPECT_EQ(engine.objects().model().find(tool)->parentLink, base);

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select tool").ok);
  const auto rooted = ui::applyConsoleCommand(engine, "make-root");
  EXPECT_TRUE(rooted.ok);
  EXPECT_EQ(rooted.message, "Made tool a root link");
  ASSERT_NE(engine.objects().model().find(tool), nullptr);
  EXPECT_EQ(engine.objects().model().find(tool)->parentLink, kNoObject);
}

TEST(DartsimConsoleActions, RelationshipCommandsReportInvalidAndLockedState)
{
  SimEngine engine;

  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "attach").message,
      "Select one frame and one parent");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "detach").message,
      "Select an attached frame");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "reparent-link").message,
      "Select one child link and one parent link");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "make-root").message,
      "Select a child link");

  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create free-frame").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create free-frame").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "select-add selected").ok);
  engine.simulation().play();

  EXPECT_EQ(ui::applyConsoleCommand(engine, "attach").message, "Scene locked");
  EXPECT_EQ(ui::applyConsoleCommand(engine, "detach").message, "Scene locked");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "reparent-link").message, "Scene locked");
  EXPECT_EQ(
      ui::applyConsoleCommand(engine, "make-root").message, "Scene locked");
}

TEST(DartsimConsoleActions, ProjectReplacementClearsWatchSessionState)
{
  SimEngine engine;
  ui::WatchState watch;

  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "create box").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch selection").ok);
  ASSERT_TRUE(ui::applyConsoleCommand(engine, watch, "watch sample").ok);
  ASSERT_FALSE(watch.targets.empty());
  ASSERT_FALSE(watch.series.empty());
  const std::uint64_t oldGeneration = engine.projectGeneration();

  const auto replaced = ui::applyConsoleCommand(engine, watch, "new --discard");
  EXPECT_TRUE(replaced.ok);
  EXPECT_NE(engine.projectGeneration(), oldGeneration);
  EXPECT_TRUE(watch.targets.empty());
  EXPECT_TRUE(watch.series.empty());
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

  const auto sensor = ui::applyConsoleCommand(engine, "create contact-sensor");
  EXPECT_TRUE(sensor.ok);
  EXPECT_EQ(sensor.message, "Added contact sensor");
  const ObjectId contactSensor = engine.selection().primary();
  ASSERT_NE(contactSensor, kNoObject);
  const SceneObject* contactObject
      = engine.objects().model().find(contactSensor);
  ASSERT_NE(contactObject, nullptr);
  EXPECT_EQ(contactObject->type, ObjectType::Sensor);
  EXPECT_EQ(contactObject->sensor.kind, SensorKind::Contact);
  EXPECT_EQ(contactObject->parent, fixedFrame);

  ASSERT_TRUE(engine.select(fixedFrame));
  const auto collision
      = ui::applyConsoleCommand(engine, "create collision-box");
  EXPECT_TRUE(collision.ok);
  EXPECT_EQ(collision.message, "Added box collision");
  const ObjectId contactShape = engine.selection().primary();
  ASSERT_NE(contactShape, kNoObject);
  const SceneObject* collisionObject
      = engine.objects().model().find(contactShape);
  ASSERT_NE(collisionObject, nullptr);
  EXPECT_EQ(collisionObject->type, ObjectType::Collision);
  EXPECT_EQ(collisionObject->shape.type, ShapeType::Box);
  EXPECT_EQ(collisionObject->parent, fixedFrame);
}

TEST(DartsimConsoleActions, CreatesCollisionAliasesForEachShape)
{
  struct AliasCase
  {
    const char* token = "";
    ShapeType shape = ShapeType::Box;
  };
  const std::vector<AliasCase> cases{
      {"collision-box", ShapeType::Box},
      {"box-collision", ShapeType::Box},
      {"collision-sphere", ShapeType::Sphere},
      {"sphere-collision", ShapeType::Sphere},
      {"collision-cylinder", ShapeType::Cylinder},
      {"cylinder-collision", ShapeType::Cylinder},
      {"collision-capsule", ShapeType::Capsule},
      {"capsule-collision", ShapeType::Capsule},
      {"collision-plane", ShapeType::Plane},
      {"plane-collision", ShapeType::Plane},
  };

  SimEngine engine;
  ASSERT_TRUE(ui::applyConsoleCommand(engine, "create box").ok);
  const ObjectId parent = engine.selection().primary();
  ASSERT_NE(parent, kNoObject);

  for (const AliasCase& alias : cases) {
    if (engine.selection().primary() != parent) {
      ASSERT_TRUE(engine.select(parent));
    }
    const auto created
        = ui::applyConsoleCommand(engine, std::string("create ") + alias.token);
    ASSERT_TRUE(created.ok) << alias.token;
    const ObjectId collision = engine.selection().primary();
    ASSERT_NE(collision, kNoObject) << alias.token;
    const SceneObject* object = engine.objects().model().find(collision);
    ASSERT_NE(object, nullptr) << alias.token;
    EXPECT_EQ(object->type, ObjectType::Collision) << alias.token;
    EXPECT_EQ(object->shape.type, alias.shape) << alias.token;
    EXPECT_EQ(object->parent, parent) << alias.token;
  }
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
