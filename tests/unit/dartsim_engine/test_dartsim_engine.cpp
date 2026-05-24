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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Geometry>
#include <dartsim_engine/commands.hpp>
#include <dartsim_engine/name_manager.hpp>
#include <dartsim_engine/scene_io.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/sim_engine.hpp>
#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>

using namespace dartsim;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

} // namespace

//==============================================================================
// SceneModel
//==============================================================================

TEST(SceneModel, AddFindRemoveRecursive)
{
  SceneModel model;
  SceneObject parent;
  parent.type = ObjectType::MultiBody;
  parent.name = "robot";
  const ObjectId parentId = model.add(parent);

  SceneObject child;
  child.type = ObjectType::Link;
  child.name = "base";
  child.parent = parentId;
  const ObjectId childId = model.add(child);

  EXPECT_TRUE(model.contains(parentId));
  EXPECT_TRUE(model.contains(childId));
  EXPECT_EQ(model.childrenOf(parentId).size(), 1u);
  EXPECT_EQ(model.rootChildren().size(), 1u);

  model.remove(parentId);
  EXPECT_FALSE(model.contains(parentId));
  EXPECT_FALSE(model.contains(childId)); // descendants removed too
  EXPECT_TRUE(model.empty());
}

TEST(SceneModel, ReparentRejectsCycle)
{
  SceneModel model;
  SceneObject a;
  a.name = "a";
  const ObjectId aId = model.add(a);
  SceneObject b;
  b.name = "b";
  b.parent = aId;
  const ObjectId bId = model.add(b);

  EXPECT_FALSE(model.reparent(aId, bId));      // a under its own descendant
  EXPECT_TRUE(model.reparent(bId, kNoObject)); // b to root
  EXPECT_EQ(model.find(bId)->parent, kNoObject);
  EXPECT_EQ(model.rootChildren().size(), 2u);
}

TEST(SceneModel, NameUniquenessHelper)
{
  SceneModel model;
  SceneObject a;
  a.name = "Body";
  model.add(a);

  EXPECT_FALSE(model.isNameAvailable(kNoObject, "Body"));
  EXPECT_TRUE(model.isNameAvailable(kNoObject, "Other"));
  EXPECT_EQ(NameManager::makeUnique(model, kNoObject, "Body"), "Body 1");
}

//==============================================================================
// ObjectManager: rebuild + render snapshot
//==============================================================================

TEST(ObjectManager, RebuildCreatesBodiesAndResolvesTransforms)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "box";
  body.transform = translation(1.0, 2.0, 3.0);
  const ObjectId id = objects.model().add(body);
  objects.rebuild();

  ASSERT_TRUE(objects.world().hasRigidBody("box"));
  const auto transform = objects.worldTransformOf(id);
  ASSERT_TRUE(transform.has_value());
  EXPECT_TRUE(transform->translation().isApprox(Eigen::Vector3d(1, 2, 3)));

  const auto items = objects.computeRenderItems();
  EXPECT_EQ(items.size(), 1u);
  EXPECT_EQ(items.front().id, id);
}

TEST(ObjectManager, MultiBodyWithLinks)
{
  ObjectManager objects;
  SceneObject mb;
  mb.type = ObjectType::MultiBody;
  mb.name = "arm";
  const ObjectId mbId = objects.model().add(mb);

  SceneObject base;
  base.type = ObjectType::Link;
  base.name = "base";
  base.parent = mbId;
  base.multiBody = mbId;
  const ObjectId baseId = objects.model().add(base);

  SceneObject fore;
  fore.type = ObjectType::Link;
  fore.name = "forearm";
  fore.parent = mbId;
  fore.multiBody = mbId;
  fore.parentLink = baseId;
  fore.jointType = JointKind::Revolute;
  objects.model().add(fore);

  objects.rebuild();

  ASSERT_TRUE(objects.world().hasMultibody("arm"));
  auto arm = objects.world().getMultibody("arm");
  ASSERT_TRUE(arm.has_value());
  EXPECT_EQ(arm->getLinkCount(), 2u);
  EXPECT_EQ(arm->getJointCount(), 1u);
}

//==============================================================================
// SelectionManager
//==============================================================================

TEST(SelectionManager, SelectToggleDeselect)
{
  SelectionManager selection;
  selection.select(7);
  EXPECT_TRUE(selection.isSelected(7));
  EXPECT_EQ(selection.primary(), 7u);

  selection.select(8, /*additive=*/true);
  EXPECT_EQ(selection.selected().size(), 2u);
  EXPECT_EQ(selection.primary(), 8u);

  selection.toggle(8);
  EXPECT_FALSE(selection.isSelected(8));
  EXPECT_EQ(selection.primary(), 7u);

  selection.clear();
  EXPECT_TRUE(selection.empty());
}

//==============================================================================
// Command stack: undo / redo
//==============================================================================

TEST(CommandManager, AddUndoRedoRestoresObjectAndSelection)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(
      commands::addRigidBody(ShapeType::Box, translation(0, 0, 1)));
  EXPECT_EQ(objects.model().size(), 1u);
  const ObjectId id = selection.primary();
  EXPECT_NE(id, kNoObject);
  EXPECT_TRUE(commands.canUndo());

  EXPECT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().size(), 0u);
  EXPECT_TRUE(selection.empty());
  EXPECT_TRUE(commands.canRedo());

  EXPECT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 1u);
  EXPECT_EQ(selection.primary(), id);
  EXPECT_TRUE(objects.world().hasRigidBody(objects.model().find(id)->name));
}

TEST(CommandManager, EditCommandsAndAutoNaming)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addRigidBody());
  commands.execute(commands::addRigidBody());
  // Two auto-named bodies must have distinct, non-empty names.
  const auto ids = objects.model().allIds();
  ASSERT_EQ(ids.size(), 2u);
  EXPECT_NE(
      objects.model().find(ids[0])->name, objects.model().find(ids[1])->name);

  const ObjectId id = ids[1];
  commands.execute(commands::setMass(id, 5.0));
  EXPECT_DOUBLE_EQ(objects.model().find(id)->mass, 5.0);

  commands.execute(commands::rename(id, "renamed"));
  EXPECT_EQ(objects.model().find(id)->name, "renamed");

  commands.execute(commands::removeObject(id));
  EXPECT_FALSE(objects.model().contains(id));
  EXPECT_EQ(objects.model().size(), 1u);
}

//==============================================================================
// SimulationController: edit/run, step, reset
//==============================================================================

TEST(SimulationController, StepAdvancesAndResetRestores)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  body.transform = translation(0, 0, 5);
  objects.model().add(body);
  objects.model().timeStep = 0.01;
  objects.rebuild();

  SimulationController controller(objects);
  EXPECT_EQ(controller.mode(), SimulationController::Mode::Edit);

  controller.step(10);
  EXPECT_EQ(controller.mode(), SimulationController::Mode::Run);
  EXPECT_EQ(controller.frameCount(), 10u);
  EXPECT_GT(controller.simTime(), 0.0);

  controller.reset();
  EXPECT_EQ(controller.mode(), SimulationController::Mode::Edit);
  EXPECT_DOUBLE_EQ(controller.simTime(), 0.0);
  EXPECT_EQ(controller.frameCount(), 0u);
  // Design pose restored.
  auto rb = objects.world().getRigidBody("ball");
  ASSERT_TRUE(rb.has_value());
  EXPECT_TRUE(
      rb->getTransform().translation().isApprox(Eigen::Vector3d(0, 0, 5)));
}

//==============================================================================
// Recorder / Player: record + scrub replay
//==============================================================================

TEST(RecorderPlayer, RecordThenSeekRestoresState)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  body.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  objects.model().add(body);
  objects.model().timeStep = 0.01;
  objects.rebuild();

  SimulationController controller(objects);
  Recorder recorder;
  recorder.start(objects.model().timeStep);
  recorder.capture(objects.world()); // frame 0
  controller.onAfterStep = [&]() {
    recorder.capture(objects.world());
  };
  controller.step(5);

  const double recordedEndTime = recorder.recording().frames().back().time;
  EXPECT_EQ(recorder.recording().frameCount(), 6u);
  EXPECT_DOUBLE_EQ(recorder.recording().frames().front().time, 0.0);

  // Stop recording, advance the live simulation further, then scrub back.
  recorder.stop();
  controller.onAfterStep = nullptr;
  controller.step(5);
  EXPECT_GT(controller.simTime(), recordedEndTime);

  Player player;
  player.setRecording(recorder.recording());
  ASSERT_TRUE(player.seek(objects.world(), 0));
  EXPECT_DOUBLE_EQ(objects.world().getTime(), 0.0);
}

TEST(RecorderPlayer, RecordingRoundTripsThroughStream)
{
  ObjectManager objects;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "ball";
  objects.model().add(body);
  objects.rebuild();

  Recorder recorder;
  recorder.start(0.01);
  recorder.capture(objects.world());
  recorder.capture(objects.world());

  std::ostringstream out(std::ios::binary);
  recorder.recording().save(out);

  Recording loaded;
  std::istringstream in(out.str(), std::ios::binary);
  ASSERT_TRUE(loaded.load(in));
  EXPECT_EQ(loaded.frameCount(), recorder.recording().frameCount());
}

//==============================================================================
// SceneIO: human-readable project round-trip
//==============================================================================

TEST(SceneIO, TextRoundTripIsStable)
{
  SceneModel model;
  model.timeStep = 0.002;
  SceneObject body;
  body.type = ObjectType::RigidBody;
  body.name = "Body With Spaces";
  body.transform = translation(1.5, -2.0, 0.25);
  body.mass = 3.0;
  model.add(body);

  SceneObject mb;
  mb.type = ObjectType::MultiBody;
  mb.name = "arm";
  const ObjectId mbId = model.add(mb);
  SceneObject link;
  link.type = ObjectType::Link;
  link.name = "base";
  link.parent = mbId;
  link.multiBody = mbId;
  model.add(link);

  const std::string text = scene_io::save(model);

  SceneModel loaded;
  ASSERT_TRUE(scene_io::load(text, loaded));
  EXPECT_EQ(loaded.size(), model.size());
  EXPECT_DOUBLE_EQ(loaded.timeStep, 0.002);

  // Re-saving the loaded model yields identical text (stable round-trip).
  EXPECT_EQ(scene_io::save(loaded), text);
}

TEST(SceneIO, RejectsBadHeader)
{
  SceneModel out;
  EXPECT_FALSE(scene_io::load("not a scene file", out));
}

TEST(SceneIO, RejectsUnsupportedVersion)
{
  SceneModel out;
  // A v1 reader must reject a newer format version instead of parsing it with
  // v1 rules (which would silently produce corrupt state).
  EXPECT_FALSE(scene_io::load("dartsim-scene 2\ntimestep 0.001\n", out));
}

TEST(CommandManager, DuplicateExplicitNamesAreDeduplicated)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  // Two bodies with the same caller-supplied name must be de-duplicated;
  // otherwise rebuild() calls World::addRigidBody with a duplicate and throws.
  commands.execute(
      commands::addRigidBody(ShapeType::Box, translation(0, 0, 0), "box"));
  commands.execute(
      commands::addRigidBody(ShapeType::Box, translation(1, 0, 0), "box"));

  EXPECT_EQ(objects.model().size(), 2u);
  EXPECT_TRUE(objects.world().hasRigidBody("box"));
  EXPECT_TRUE(objects.world().hasRigidBody("box 1"));
}

TEST(CommandManager, NoOpCommandPreservesRedoBranch)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addRigidBody());
  const ObjectId id = selection.primary();
  ASSERT_TRUE(commands.undo()); // creates a pending redo branch
  ASSERT_TRUE(commands.canRedo());

  // A command that changes nothing (here the target id no longer exists) must
  // not record a history entry or discard the pending redo branch.
  commands.execute(commands::rename(id, "noop"));
  EXPECT_FALSE(commands.canUndo());
  EXPECT_TRUE(commands.canRedo());

  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 1u);
}

TEST(CommandManager, RemoveDeselectsEntireSubtree)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm));
  const ObjectId link = selection.primary();
  ASSERT_NE(arm, kNoObject);
  ASSERT_NE(link, arm);

  // Select both the parent and its descendant (additive multi-selection).
  selection.select(arm);
  selection.select(link, /*additive=*/true);
  ASSERT_TRUE(selection.isSelected(arm));
  ASSERT_TRUE(selection.isSelected(link));

  commands.execute(commands::removeObject(arm));
  EXPECT_FALSE(objects.model().contains(arm));
  EXPECT_FALSE(objects.model().contains(link)); // descendant removed too
  // Neither the root nor the descendant may linger in the selection.
  EXPECT_FALSE(selection.isSelected(link));
  EXPECT_TRUE(selection.empty());
}

TEST(CommandManager, AddLinkRejectsInvalidParentLink)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  ASSERT_EQ(objects.model().size(), 1u);

  // A parent link referencing a missing id is rejected (nothing added),
  // otherwise the orphaned link is silently dropped from the rebuilt world.
  commands.execute(commands::addLink(arm, /*parentLink=*/9999));
  EXPECT_EQ(objects.model().size(), 1u);

  // A non-Link object is not a valid parent link.
  commands.execute(commands::addRigidBody());
  const ObjectId body = selection.primary();
  commands.execute(commands::addLink(arm, body));
  EXPECT_EQ(objects.model().size(), 2u); // arm + body, no link

  // A real link in the same multibody is accepted as a parent.
  commands.execute(commands::addLink(arm));
  const ObjectId rootLink = selection.primary();
  commands.execute(commands::addLink(arm, rootLink));
  EXPECT_EQ(objects.model().size(), 4u); // arm, body, rootLink, child link
}

TEST(CommandManager, ReparentRejectsLinkButAllowsOthers)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.execute(commands::addMultiBody("arm"));
  const ObjectId arm = selection.primary();
  commands.execute(commands::addLink(arm));
  const ObjectId link = selection.primary();
  commands.execute(commands::addFreeFrame());
  const ObjectId frame = selection.primary();

  // A link must stay under its owning multibody; moving it elsewhere would drop
  // it from the rebuilt world, so the reparent is rejected.
  commands.execute(commands::reparent(link, frame));
  ASSERT_NE(objects.model().find(link), nullptr);
  EXPECT_EQ(objects.model().find(link)->parent, arm);

  // Non-link objects can still be reparented.
  commands.execute(commands::addRigidBody());
  const ObjectId body = selection.primary();
  commands.execute(commands::reparent(body, frame));
  EXPECT_EQ(objects.model().find(body)->parent, frame);
}

//==============================================================================
// SimEngine: end-to-end design -> run -> record -> replay + project file
//==============================================================================

TEST(SimEngine, EndToEndEditorLoop)
{
  SimEngine engine;
  int changes = 0;
  engine.setOnChanged([&]() { ++changes; });

  engine.execute(
      commands::addRigidBody(ShapeType::Sphere, translation(0, 0, 2)));
  engine.execute(commands::addMultiBody("arm"));
  EXPECT_EQ(engine.objects().model().size(), 2u);
  EXPECT_GE(changes, 2);

  // Undo/redo through the facade.
  EXPECT_TRUE(engine.undo());
  EXPECT_EQ(engine.objects().model().size(), 1u);
  EXPECT_TRUE(engine.redo());
  EXPECT_EQ(engine.objects().model().size(), 2u);

  // Render snapshot has the rigid body (multibody has no links yet).
  EXPECT_EQ(engine.renderItems().size(), 1u);

  // Run + record, then scrub replay.
  engine.objects().model().timeStep = 0.01;
  engine.objects().rebuild();
  engine.startRecording();
  engine.simulation().step(5);
  engine.stopRecording();
  EXPECT_GE(engine.recorder().recording().frameCount(), 6u);

  engine.loadRecordingIntoPlayer();
  ASSERT_TRUE(engine.replaySeek(0));
  EXPECT_DOUBLE_EQ(engine.objects().world().getTime(), 0.0);
}

TEST(SimEngine, ProjectFileRoundTrip)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path() / "dartsim_engine_test.dartsim";

  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(1, 1, 1)));
  engine.execute(commands::addMultiBody("arm"));
  const std::size_t expected = engine.objects().model().size();
  ASSERT_TRUE(engine.saveProject(path.string()));

  SimEngine reloaded;
  ASSERT_TRUE(reloaded.loadProject(path.string()));
  EXPECT_EQ(reloaded.objects().model().size(), expected);
  EXPECT_FALSE(reloaded.commands().canUndo()); // history cleared on load

  std::filesystem::remove(path);
}

TEST(SimEngine, LoadProjectClearsRunSnapshot)
{
  const std::filesystem::path path = std::filesystem::temp_directory_path()
                                     / "dartsim_engine_load_reset_test.dartsim";

  SimEngine saved;
  saved.execute(commands::addRigidBody(ShapeType::Box, translation(1, 1, 1)));
  ASSERT_TRUE(saved.saveProject(path.string()));
  const std::size_t savedSize = saved.objects().model().size();

  // Enter Run mode on a different (empty) scene so a design snapshot is
  // captured, then load the saved project over it.
  SimEngine engine;
  engine.simulation().play();
  ASSERT_TRUE(engine.loadProject(path.string()));
  EXPECT_EQ(engine.objects().model().size(), savedSize);
  EXPECT_EQ(engine.simulation().mode(), SimulationController::Mode::Edit);

  // Reset must keep the loaded scene rather than restore the pre-load snapshot.
  engine.simulation().reset();
  EXPECT_EQ(engine.objects().model().size(), savedSize);

  std::filesystem::remove(path);
}

TEST(SimEngine, LoadProjectClearsRecordingAndPlayer)
{
  const std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / "dartsim_engine_load_clears_replay.dartsim";

  // Record several frames on one scene and load them into the player.
  SimEngine engine;
  engine.execute(commands::addRigidBody(ShapeType::Box, translation(0, 0, 5)));
  engine.objects().model().timeStep = 0.01;
  engine.objects().rebuild();
  engine.startRecording();
  engine.simulation().step(3);
  engine.stopRecording();
  engine.loadRecordingIntoPlayer();
  ASSERT_GT(engine.player().frameCount(), 0u);

  // Load a different project over it.
  SimEngine other;
  other.execute(commands::addMultiBody("arm"));
  ASSERT_TRUE(other.saveProject(path.string()));
  ASSERT_TRUE(engine.loadProject(path.string()));

  // The stale recording/replay must not survive the load; otherwise seeking it
  // would restore old-world snapshots into the freshly loaded scene.
  EXPECT_FALSE(engine.isRecording());
  EXPECT_EQ(engine.recorder().recording().frameCount(), 0u);
  EXPECT_EQ(engine.player().frameCount(), 0u);
  EXPECT_FALSE(engine.replaySeek(0));

  std::filesystem::remove(path);
}

//==============================================================================
// Command macros (grouped, single-undo transactions)
//==============================================================================

TEST(CommandManager, MacroGroupsIntoSingleUndo)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Add Two Bodies");
  EXPECT_TRUE(commands.inMacro());
  commands.execute(commands::addRigidBody());
  commands.execute(commands::addRigidBody());
  commands.endMacro();
  EXPECT_FALSE(commands.inMacro());

  EXPECT_EQ(objects.model().size(), 2u);
  EXPECT_EQ(commands.undoLabel(), "Add Two Bodies");

  // One undo reverts the whole macro.
  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().size(), 0u);
  // One redo reapplies the whole macro.
  ASSERT_TRUE(commands.redo());
  EXPECT_EQ(objects.model().size(), 2u);
}

TEST(CommandManager, EmptyMacroAddsNoHistory)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Nothing");
  commands.endMacro();
  EXPECT_FALSE(commands.canUndo());
}

TEST(CommandManager, NestedMacrosFlattenToOneEntry)
{
  ObjectManager objects;
  SelectionManager selection;
  CommandManager commands(objects, selection);

  commands.beginMacro("Outer");
  commands.execute(commands::addRigidBody());
  commands.beginMacro("Inner");
  commands.execute(commands::addRigidBody());
  commands.endMacro();
  EXPECT_TRUE(commands.inMacro()); // outer still open
  commands.execute(commands::addRigidBody());
  commands.endMacro();

  EXPECT_EQ(objects.model().size(), 3u);
  ASSERT_TRUE(commands.undo());
  EXPECT_EQ(objects.model().size(), 0u); // all three reverted together
}

//==============================================================================
// EventBus
//==============================================================================

TEST(EventBus, SubscribeEmitUnsubscribe)
{
  EventBus bus;
  int count = 0;
  EventType last = EventType::SceneChanged;
  const int token = bus.subscribe([&](const Event& event) {
    ++count;
    last = event.type;
  });
  EXPECT_EQ(bus.listenerCount(), 1u);

  bus.emit(EventType::SelectionChanged);
  EXPECT_EQ(count, 1);
  EXPECT_EQ(last, EventType::SelectionChanged);

  bus.unsubscribe(token);
  bus.emit(EventType::SceneChanged);
  EXPECT_EQ(count, 1); // no longer notified
  EXPECT_EQ(bus.listenerCount(), 0u);
}

TEST(EventBus, UnsubscribeDuringEmitIsSafe)
{
  EventBus bus;
  int selfCount = 0;
  int otherCount = 0;
  int selfToken = 0;
  // A listener that unsubscribes itself mid-dispatch must not invalidate the
  // iteration or skip the remaining listeners.
  selfToken = bus.subscribe([&](const Event&) {
    ++selfCount;
    bus.unsubscribe(selfToken);
  });
  bus.subscribe([&](const Event&) { ++otherCount; });

  bus.emit(EventType::SceneChanged);
  EXPECT_EQ(selfCount, 1);
  EXPECT_EQ(otherCount, 1);
  EXPECT_EQ(bus.listenerCount(), 1u); // self removed during dispatch

  bus.emit(EventType::SceneChanged);
  EXPECT_EQ(selfCount, 1); // self no longer notified
  EXPECT_EQ(otherCount, 2);
}

//==============================================================================
// Logger
//==============================================================================

TEST(Logger, RecordsLevelsAndCapsCapacity)
{
  Logger log(3);
  log.info("a");
  log.warning("b");
  log.error("c");
  log.info("d");

  EXPECT_EQ(log.size(), 3u);                     // capped at capacity
  EXPECT_EQ(log.entries().front().message, "b"); // oldest ("a") dropped
  EXPECT_EQ(log.entries().back().message, "d");
  EXPECT_EQ(log.entries().back().level, LogLevel::Info);

  log.clear();
  EXPECT_EQ(log.size(), 0u);
}

//==============================================================================
// SimEngine wiring: events + logging
//==============================================================================

TEST(SimEngine, ExecuteEmitsEventAndLogs)
{
  SimEngine engine;
  int events = 0;
  engine.events().subscribe([&](const Event&) { ++events; });

  engine.execute(commands::addRigidBody());
  EXPECT_GE(events, 1);
  ASSERT_GE(engine.logger().size(), 1u);
  EXPECT_EQ(engine.logger().entries().back().message, "Add Rigid Body");
}

TEST(SimEngine, MacroThroughFacadeGroupsUndo)
{
  SimEngine engine;
  engine.commands().beginMacro("Seed");
  engine.execute(commands::addRigidBody());
  engine.execute(commands::addMultiBody("arm"));
  engine.commands().endMacro();

  EXPECT_EQ(engine.objects().model().size(), 2u);
  ASSERT_TRUE(engine.undo());
  EXPECT_EQ(engine.objects().model().size(), 0u); // grouped undo
}
