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

#ifndef DART_EXAMPLES_DEMOS_DEMOHOST_HPP_
#define DART_EXAMPLES_DEMOS_DEMOHOST_HPP_

#include "ContactVisualizer.hpp"
#include "DemoScene.hpp"
#include "DragForce.hpp"
#include "Inspector.hpp"
#include "LogCapture.hpp"
#include "Profiler.hpp"

#include <dart/gui/osg/PerformanceStatsPanel.hpp>
#include <dart/gui/osg/osg.hpp>

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dart_demos {

//==============================================================================
/// Per-scene WorldNode. Forwards the active scene's preStep/postStep hooks,
/// wrapping each call so a throwing (or otherwise misbehaving) callback is
/// logged once through the host's error sink and then disabled -- it never
/// crashes the app or corrupts the frame loop.
class DemoWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit DemoWorldNode(const dart::simulation::WorldPtr& world);

  /// Installs the active scene's step hooks (any may be empty). `preRefresh`
  /// is wired to customPreRefresh() and therefore runs once per rendered
  /// frame regardless of pause state; preStep/postStep only run while
  /// simulating (see DemoScene.hpp).
  void setHooks(
      std::function<void()> preStep,
      std::function<void()> postStep,
      std::function<void()> preRefresh = nullptr);

  /// Called (hookName, reason) the first time a hook throws; the host uses
  /// this to log the failure and update the status line.
  void setHookErrorSink(
      std::function<void(const std::string&, const std::string&)> sink);

  /// Runs exactly one deterministic simulation step by invoking the same
  /// pre/post hooks that refresh() would. Used by the Step toolbar button and
  /// by headless/cycle-scenes stepping, both of which need determinism
  /// independent of RealTimeWorldNode's wall-clock budget.
  void stepOnce();

  /// Number of steps taken by this node since it was constructed (via either
  /// refresh() or stepOnce()).
  std::size_t getStepCount() const;

  /// Number of steps taken during the most recently completed refresh()
  /// call (0 while paused, since refresh() takes no steps then; may be >1
  /// under real-time catch-up, or 0 on an over-budget refresh with no time
  /// to spare). stepOnce() does not go through refresh() and does not
  /// affect this counter.
  std::size_t getLastRefreshStepCount() const;

  double getLastStepMs() const;
  double getMovingAverageStepMs() const;
  double getMinStepMs() const;
  double getMaxStepMs() const;
  std::size_t getStepTimingSamples() const;

  // Documentation inherited
  void customPreStep() override;
  void customPostStep() override;
  void customPreRefresh() override;
  void customPostRefresh() override;

private:
  void invokeHook(std::function<void()>& hook, const char* name);
  void beginStepTiming();
  void endStepTiming();

  std::function<void()> mPreStep;
  std::function<void()> mPostStep;
  std::function<void()> mPreRefresh;
  std::function<void(const std::string&, const std::string&)> mHookErrorSink;
  std::size_t mStepCount;
  std::size_t mStepCountAtRefreshStart = 0;
  std::size_t mLastRefreshStepCount = 0;
  std::chrono::steady_clock::time_point mStepStart;
  bool mStepTimingActive = false;
  double mLastStepMs = 0.0;
  double mMovingAverageStepMs = 0.0;
  double mMinStepMs = 0.0;
  double mMaxStepMs = 0.0;
  std::size_t mStepTimingSamples = 0;
};

//==============================================================================
/// One line of the diagnostics log console.
struct LogEntry
{
  enum class Level
  {
    Info,
    Warning,
    Error
  };

  Level level;
  std::string message;
};

//==============================================================================
/// Owns the single ImGuiViewer window, the scene registry, and the safe
/// runtime scene-switching machinery described in
/// docs/dev_tasks/dart6_consolidated_demos/BRIEF-phase1.md. Persistent host
/// chrome (window, theme, panel layout) survives scene switches; per-scene
/// state (world node, widgets, drag-and-drop, event handlers) is torn down in
/// full on every switch.
class DemoHost
{
public:
  DemoHost(
      std::vector<DemoScene> scenes,
      double guiScale,
      std::string collisionDetectorName = {},
      std::size_t simulationThreads = 1u);

  /// Tears down the active scene (draining every registered teardown --
  /// drag-and-drop, attachments, event handlers) before the viewer is
  /// destroyed. Without this, a scene left active at process exit (every
  /// exit path -- run(), cycleScenes(), runHeadlessShot() -- destroys this
  /// stack-local host on return) leaks its DnD/attachment/handler
  /// registrations into the viewer's own teardown, which for some resource
  /// types (e.g. InteractiveFrameDnD) crashes instead of merely leaking.
  ~DemoHost();

  /// Runs interactively: opens a window and blocks until it is closed.
  /// Returns a process exit code.
  int run();

  /// Prints the catalog grouped by category (first-appearance order) and
  /// returns 0.
  int listScenes() const;

  /// Advances through every scene in the catalog, `framesPerScene` steps
  /// each, twice in a row (the repeat proves rapid re-switching does not leak
  /// world nodes). No window or GPU context is required. Returns 0 if every
  /// scene's factory succeeded and the world-node bookkeeping stayed
  /// balanced, nonzero otherwise.
  int cycleScenes(int framesPerScene);

  /// Builds `sceneId` (or the initial scene if empty) off-screen via a pbuffer
  /// GraphicsContext, steps it `steps` times, and writes a screenshot to
  /// `shotPath`. Returns 0 on success, nonzero on failure.
  int runHeadlessShot(
      const std::string& shotPath,
      int steps,
      const std::string& sceneId,
      int width,
      int height);

  /// Selects which scene id run()/cycleScenes()/runHeadlessShot() should
  /// start on. An empty id (the default) means "the first scene in the
  /// catalog".
  void setInitialScene(const std::string& id);

  /// Test/debug-only hooks (main.cpp's hidden --debug-* flags) so a headless
  /// capture can exercise UI state that normally requires interactive input:
  /// a body selected in the Inspector, and the Profiler actively recording.
  /// Applied by runHeadlessShot() right after the scene installs.
  void setDebugSelectBodyName(const std::string& name);
  void setDebugRecordProfile(bool on);

  /// Number of world nodes currently registered with the viewer. Exactly one
  /// whenever a scene is active; used by cycleScenes() as a leak audit.
  std::size_t getActiveWorldNodeCount() const;

  //----------------------------------------------------------------------------
  // The following are called by DemoHostContext and the host's own ImGui
  // panel widget / key handler; not part of the app-facing API.
  //----------------------------------------------------------------------------

  dart::gui::osg::ImGuiViewer* getViewer() const;
  DemoWorldNode* getWorldNode() const;
  void addTeardown(std::function<void()> teardown);
  void log(LogEntry::Level level, const std::string& message);

  /// Draws the Demos navigator, Simulation toolbar, and Diagnostics panels,
  /// plus the active scene's own panel. Called once per frame by the host's
  /// ImGuiWidget.
  void renderPanels();

  /// Invokes the key action bound to `key` for the active scene, if any,
  /// guarded so a throwing action is logged and disabled rather than crashing
  /// the app. Called by the host's persistent key event handler. Returns true
  /// if a matching action was found (and thus the key event was consumed).
  bool handleKey(int key);

private:
  struct CategoryGroup
  {
    std::string name;
    std::vector<std::size_t> sceneIndices;
  };

  void buildCategories();
  const DemoScene* findScene(const std::string& id) const;

  /// Queues a scene switch to be executed by processPendingSwitch(). Safe to
  /// call from inside an ImGui panel callback; the switch itself never runs
  /// mid-render.
  void requestSceneSwitch(const std::string& id);

  /// Executes a queued switch, if any: builds the requested scene inside a
  /// try/catch, then tears down the old scene and installs the new one on
  /// success, or leaves the previous scene (or an empty world) running and
  /// surfaces the failure reason on failure. Must only be called between
  /// frames, never from inside ImGui rendering.
  void processPendingSwitch();

  /// Tears down every resource owned by the current scene (in dependency
  /// order: drag-and-drop / attachments / event handlers first, since they
  /// may reference objects owned by the world; then the world node; then the
  /// world itself) and resets host state back to "no active scene".
  void teardownCurrentScene();

  /// Builds and registers `setup` as the current scene.
  void installScene(const DemoScene& scene, DemoSceneSetup setup);

  /// Installs a minimal, guaranteed-safe empty world. Used only when the very
  /// first scene selection fails and there is no previous scene to fall back
  /// to.
  void installEmptyFallback();

  void applyCameraHome(const CameraHome& home, bool viaManipulator);

  void ensureViewerConfigured();
  bool prepareOffscreenContext(int width, int height, const CameraHome& home);

  void renderToolbar();
  void renderNavigator();
  void renderScenePanel();
  void renderDiagnostics();
  void renderInspectorSection();
  void renderViewMenu();
  void renderRuntimeControls();
  void invokeKeyAction(KeyAction& action);
  void applyShadowState();
  void applyRuntimeOptionsToWorld();
  bool setCollisionDetectorByName(const std::string& name);
  void syncCollisionDetectorSelectionFromWorld();

  std::vector<DemoScene> mScenes;
  std::vector<CategoryGroup> mCategories;
  std::unordered_map<std::string, std::size_t> mCategoryIndexByName;

  ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  ::osg::ref_ptr<DemoWorldNode> mWorldNode;
  dart::simulation::WorldPtr mCurrentWorld;
  std::function<void()> mCurrentRenderPanel;
  std::vector<KeyAction> mCurrentKeyActions;
  std::optional<CameraHome> mCurrentCameraHome;
  std::vector<std::function<void()>> mExtraTeardowns;

  std::string mCurrentSceneId;
  std::string mCurrentSceneTitle;
  std::optional<std::string> mPendingSceneId;
  bool mLastSwitchFailed = false;
  std::size_t mActiveWorldNodeCount = 0;

  std::string mStatusLine = "No demo loaded yet.";
  std::deque<LogEntry> mLog;
  std::unique_ptr<LogCapture> mLogCapture;
  bool mLogShowInfo = true;
  bool mLogShowWarning = true;
  bool mLogShowError = true;
  bool mLogAutoscroll = true;
  char mSearchBuf[128] = {};

  Inspector mInspector;
  DragForce mDragForce;
  ContactVisualizer mContactVisualizer;
  Profiler mProfiler;
  dart::gui::osg::PerformanceStatsPanel mPerformanceStatsPanel;

  bool mGravityEnabled = true;
  Eigen::Vector3d mSavedGravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  float mTargetRtf = 1.0f;
  float mTimeStep = 0.001f;
  std::vector<std::string> mAvailableCollisionDetectors;
  std::string mRequestedCollisionDetectorName;
  std::string mCurrentCollisionDetectorName = "none";
  int mCollisionDetectorIndex = -1;
  int mSimulationThreads = 1;

  // View utilities (toolbar "View" menu). Shadows/grid/headlights/GUI scale
  // are host chrome that persists across scene switches; mCurrentSceneWants
  // Shadows tracks the active scene's own DemoSceneSetup::enableShadows so
  // toggling the host's global mShadowsEnabled can be combined with it.
  bool mShadowsEnabled = true;
  bool mCurrentSceneWantsShadows = true;
  ::osg::ref_ptr<dart::gui::osg::GridVisual> mGridVisual;

  double mGuiScale;
  bool mViewerConfigured = false;
  std::string mInitialSceneId;

  std::string mDebugSelectBodyName;
  bool mDebugRecordProfile = false;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_DEMOHOST_HPP_
