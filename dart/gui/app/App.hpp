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

#ifndef DART_GUI_APP_APP_HPP_
#define DART_GUI_APP_APP_HPP_

#include "ContactVisualizer.hpp"
#include "DragForce.hpp"
#include "Inspector.hpp"
#include "LogCapture.hpp"
#include "Profiler.hpp"
#include "Scene.hpp"

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  #include "memory_diagnostics_model.hpp"
#endif

#include <dart/gui/GridVisual.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/PerformanceStatsPanel.hpp>
#include <dart/gui/RealTimeWorldNode.hpp>
#include <dart/gui/Viewer.hpp>

#include <chrono>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dart::gui::app {

class GuiScaleTheme;

//==============================================================================
/// Per-scene WorldNode. Forwards the active scene's preStep/postStep hooks,
/// wrapping each call so a throwing (or otherwise misbehaving) callback is
/// logged once through the host's error sink and then disabled -- it never
/// crashes the app or corrupts the frame loop.
class AppWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  explicit AppWorldNode(const dart::simulation::WorldPtr& world);

  /// Installs the active scene's step hooks (any may be empty). `preRefresh`
  /// is wired to customPreRefresh() and therefore runs once per rendered
  /// frame regardless of pause state; preStep/postStep only run while
  /// simulating (see Scene.hpp).
  void setHooks(
      std::function<void()> preStep,
      std::function<void()> postStep,
      std::function<void()> preRefresh = nullptr);

  /// Called (hookName, reason) the first time a hook throws; the host uses
  /// this to log the failure and update the status line.
  void setHookErrorSink(
      std::function<void(const std::string&, const std::string&)> sink);

  /// Polled after each preStep hook. Once it returns true the node stops
  /// simulating and takes no further step, so a real-time catch-up in
  /// refresh() ends at once instead of running out its step budget.
  void setStopRequested(std::function<bool()> stopRequested);

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
  std::function<bool()> mStopRequested;
  bool mStepStopped = false;
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
/// Owns the single Viewer window, the scene registry, and the safe
/// runtime scene-switching machinery described in
/// docs/onboarding/architecture.md.
/// Persistent host chrome (window, theme, panel layout) survives scene
/// switches; per-scene state (world node, widgets, drag-and-drop, event
/// handlers) is torn down in full on every switch.
class App
{
public:
  App(std::vector<Scene> scenes,
      std::optional<double> guiScale,
      std::string collisionDetectorName = {},
      std::size_t simulationThreads = 1u);

  /// Tears down the active scene (draining every registered teardown --
  /// drag-and-drop, attachments, event handlers) before the viewer is
  /// destroyed. Without this, a scene left active at process exit (every
  /// exit path -- run(), cycleScenes(), runHeadlessShot() -- destroys this
  /// stack-local host on return) leaks its DnD/attachment/handler
  /// registrations into the viewer's own teardown, which for some resource
  /// types (e.g. InteractiveFrameDnD) crashes instead of merely leaking.
  ~App();

  /// Runs interactively: opens a window and blocks until it is closed.
  /// Returns a process exit code, or kStoppedExitCode when stopped.
  int run(std::optional<int> width = {}, std::optional<int> height = {});

  /// Exit code of a run, cycle or capture that setStopRequested() ended.
  static constexpr int kStoppedExitCode = 130;

  /// Polled once per frame, cycle step and capture step; returning true ends
  /// the run after tearing the active scene down.
  void setStopRequested(std::function<bool()> stopRequested);

  /// Classifies an exception that escapes a --cycle-scenes frame, such as one
  /// from a viewer attachment a language binding implemented. Returning true
  /// makes it a stop rather than a failed scene: the run then ends at its
  /// next stop poll, which the classifier must make return true.
  void setStopException(std::function<bool(const std::exception&)> isStop);

  /// Prints the catalog grouped by category (first-appearance order) and
  /// returns 0.
  int listScenes() const;

  /// Advances through every scene in the catalog, `framesPerScene` steps
  /// each, twice in a row (the repeat proves rapid re-switching does not leak
  /// world nodes). Without `render` no window or GPU context is required and
  /// each step also runs the scene's preRefresh; with it, every step renders a
  /// display-free `width` x `height` frame, so scene replacement exercises
  /// renderer resource churn. The last scene is torn down at the end. Returns
  /// 0 if every scene's factory (and render) succeeded or lacked prefetched
  /// assets, no scene callback had to be disabled, and the world-node
  /// bookkeeping stayed balanced; nonzero otherwise.
  int cycleScenes(
      int framesPerScene, bool render = false, int width = 0, int height = 0);

  /// Builds `sceneId` (or the initial scene if empty) in a display-free
  /// capture viewer, steps it `steps` times, and writes a screenshot to
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

  /// Test/debug-only hooks (Cli.cpp's hidden --debug-* flags) so a headless
  /// capture can exercise UI state that normally requires interactive input:
  /// a body selected in the Inspector, and the Profiler actively recording.
  /// Memory diagnostics use the DART_DEMOS_MEMORY_DIAGNOSTICS=1 environment
  /// hook so no additional public command-line option is needed.
  /// Applied by runHeadlessShot() right after the scene installs.
  void setDebugSelectBodyName(const std::string& name);
  void setDebugRecordProfile(bool on);

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  /// Number of wheel-down ticks runHeadlessShot() injects into the viewer
  /// event queue while hovering the scene panel, capturing one frame per tick
  /// next to the --shot path. Exercises the viewer-owned ImGui scroll input
  /// queue so headless captures can prove the Memory tab scrolls without
  /// corrupting the renderer. Verification-only; compiled out with the
  /// diagnostics option.
  void setDebugScrollMemoryPanelTicks(int ticks);
#endif

  /// Number of world nodes currently registered with the viewer. Exactly one
  /// whenever a scene is active; used by cycleScenes() as a leak audit.
  std::size_t getActiveWorldNodeCount() const;

  /// Number of scene callbacks (hooks, key actions, panel callbacks,
  /// onActivate, teardowns) that threw and were disabled since construction.
  std::size_t getCallbackFailureCount() const;

  //----------------------------------------------------------------------------
  // The following are called by SceneContext and the host's own ImGui
  // panel widget / key handler; not part of the app-facing API.
  //----------------------------------------------------------------------------

  dart::gui::Viewer* getViewer() const;
  AppWorldNode* getWorldNode() const;
  std::shared_ptr<dart::gui::Viewer> getViewerPtr() const;
  std::shared_ptr<AppWorldNode> getWorldNodePtr() const;

  /// Throws std::logic_error while the active scene is being torn down.
  void addTeardown(std::function<void()> teardown);
  void log(LogEntry::Level level, const std::string& message);

  /// Logs and counts a disabled scene callback; see invokeSceneCallback().
  void reportCallbackFailure(const std::string& what, const std::string& error);

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
  friend class AppTestAccess;

  enum class ScenePanelTab
  {
    Scene,
    Inspector,
    Tools,
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
    Memory
#endif
  };

  struct CategoryGroup
  {
    std::string name;
    std::vector<std::size_t> sceneIndices;
  };

  void buildCategories();
  const Scene* findScene(const std::string& id) const;

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
  /// Installs a freshly built scene. `applyCameraHomePose` places the camera
  /// at the scene's home framing; Rebuild/Reset of the already-active demo
  /// passes false so only the physics resets, never the user's camera pose.
  void installScene(
      const Scene& scene, SceneSetup setup, bool applyCameraHomePose = true);

  /// Installs a minimal, guaranteed-safe empty world. Used only when the very
  /// first scene selection fails and there is no previous scene to fall back
  /// to.
  void installEmptyFallback();

  void applyCameraHome(const CameraHome& home, bool viaManipulator);

  /// Logs a failure, and also prints it in display-free runs.
  void reportFailure(const std::string& message);

  /// Runs one scene callback; a throw is logged, counted, and disables it.
  void invokeSceneCallback(
      std::function<void()>& callback, const std::string& what);

  /// Runs queued panel change callbacks and panel key-action clicks. Called
  /// between frames and steps in every mode, never from inside ImGui
  /// rendering.
  void processPanelEvents();

  bool stopRequested() const;

  /// Tears the active scene down and returns kStoppedExitCode.
  int stopRun();
  void ensureViewerConfigured();
  void startSimulation();
  void initializeGuiTheme();
  void setGuiScale(double scale);
  void updateAutomaticGuiScale();
  bool prepareOffscreenContext(int width, int height, const CameraHome& home);

  void renderToolbar();
  void renderNavigator();
  void renderScenePanel();
  void renderDiagnostics();
  void renderInspectorSection();
  void renderSceneControlsSection();
  void renderToolsSection();
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  void renderMemorySection();
#endif
  void renderLogSection(float height);
  void renderViewMenu();
  void renderRuntimeControls();
  void invokeKeyAction(KeyAction& action);
  void requestScenePanelTab(ScenePanelTab tab);
  void applyShadowState();
  void fitCameraToWorld();
  void applyRuntimeOptionsToWorld();
  bool setCollisionDetectorByName(const std::string& name);
  void syncCollisionDetectorSelectionFromWorld();

  std::vector<Scene> mScenes;
  std::vector<CategoryGroup> mCategories;
  std::unordered_map<std::string, std::size_t> mCategoryIndexByName;

  std::shared_ptr<dart::gui::Viewer> mViewer;
  std::shared_ptr<AppWorldNode> mWorldNode;
  dart::simulation::WorldPtr mCurrentWorld;
  std::shared_ptr<Control> mCurrentPanel;
  std::vector<KeyAction> mCurrentKeyActions;
  std::optional<std::size_t> mPendingKeyAction;
  std::optional<CameraHome> mCurrentCameraHome;
  std::vector<std::function<void()>> mExtraTeardowns;

  /// Owned for as long as the active scene is installed; SceneContext copies
  /// hold it weakly, so they expire when the scene's teardowns have run.
  std::shared_ptr<const void> mActivation;
  bool mTearingDown = false;
  bool mResumeSimulationOnTeardown = false;
  bool mRestoreSimulationOnTeardown = false;
  bool mEchoErrors = false;
  std::size_t mCallbackFailures = 0;
  std::function<bool()> mStopRequested;
  mutable bool mStopLatched = false;
  std::function<bool(const std::exception&)> mIsStopException;
  const std::chrono::steady_clock::time_point mStartTime
      = std::chrono::steady_clock::now();

  std::string mCurrentSceneId;
  std::string mCurrentSceneTitle;
  std::optional<std::string> mPendingSceneId;
  bool mLastSwitchFailed = false;
  std::string mLastSwitchSkipReason;
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
  dart::gui::PerformanceStatsPanel mPerformanceStatsPanel;
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  dart::gui::app::DiagnosticSession mMemoryDiagnosticsSession;
  std::uint64_t mMemoryDiagnosticsGeneration = 0;
#endif

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
  // Shadows tracks the active scene's own SceneSetup::enableShadows so
  // toggling the host's global mShadowsEnabled can be combined with it.
  bool mShadowsEnabled = true;
  bool mCurrentSceneWantsShadows = true;
  std::shared_ptr<dart::gui::GridVisual> mGridVisual;
  std::shared_ptr<dart::gui::ImGuiWidget> mPanelWidget;
  std::shared_ptr<dart::gui::GUIEventHandler> mKeyHandler;

  double mGuiScale;
  bool mAutomaticGuiScale;
  std::unique_ptr<GuiScaleTheme> mGuiScaleTheme;
  std::chrono::steady_clock::time_point mNextDpiCheck;
  bool mViewerConfigured = false;
  std::string mInitialSceneId;

  std::string mDebugSelectBodyName;
  bool mDebugRecordProfile = false;
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  bool mDebugMemoryDiagnostics = false;
  int mDebugScrollMemoryPanelTicks = 0;
#endif

  ScenePanelTab mRequestedScenePanelTab = ScenePanelTab::Scene;
  bool mHasRequestedScenePanelTab = false;

  /// Rebuilt on the first frame and whenever View > "Reset layout" is selected.
  bool mDockLayoutInitialized = false;
  bool mDockLayoutResetRequested = false;
};

/// Parses the demo app's command line (argv[0] is the program name) and runs
/// the list, cycle, headless capture or interactive mode it selects on an
/// App built from `scenes`. Returns the process exit code. `stopRequested`
/// and `isStopException` work as App::setStopRequested() and
/// App::setStopException() describe. Throws std::logic_error if
/// another call is still running, because the host captures the
/// process-wide output streams.
int runCommandLine(
    std::vector<Scene> scenes,
    const std::vector<std::string>& argv,
    std::function<bool()> stopRequested = {},
    std::function<bool(const std::exception&)> isStopException = {});

} // namespace dart::gui::app

#endif // DART_GUI_APP_APP_HPP_
