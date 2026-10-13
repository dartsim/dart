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

#include "App.hpp"

#include "GuiOptions.hpp"
#include "GuiScale.hpp"
#include "Theme.hpp"

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  #include "memory_diagnostics.hpp"
#endif

#include <dart/collision/CollisionDetector.hpp>

#include <dart/external/imgui/imgui.h>
#ifndef IMGUI_HAS_DOCK
  #error "dart-gui-app requires the Dear ImGui docking release"
#endif
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>

#include <dart/external/imgui/imgui_internal.h>

#include <algorithm>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
// std::strerror is used on the unconditional headless error path, so this stays
// outside the diagnostics guard even though std::strcmp below is guarded.
#include <cstring>

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  #include <cstdlib>
#endif

namespace dart::gui::app {

namespace {

constexpr int kDefaultWindowWidth = 1600;
constexpr int kDefaultWindowHeight = 1000;

//==============================================================================
std::string toLower(const std::string& value)
{
  std::string result = value;
  std::transform(
      result.begin(), result.end(), result.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  return result;
}

//==============================================================================
bool matchesFilter(const Scene& scene, const std::string& filterLower)
{
  if (filterLower.empty())
    return true;

  return toLower(scene.id).find(filterLower) != std::string::npos
         || toLower(scene.title).find(filterLower) != std::string::npos
         || toLower(scene.summary).find(filterLower) != std::string::npos
         || toLower(scene.category).find(filterLower) != std::string::npos;
}

//==============================================================================
/// Renders "f - Shoot sphere" when the key is a plain printable character, or
/// just the label otherwise (e.g. arrow keys).
std::string formatKeyActionLabel(const KeyAction& action)
{
  if (action.key >= 32 && action.key < 127)
    return std::string(1, static_cast<char>(action.key)) + " - " + action.label;
  return action.label;
}

//==============================================================================
/// True if `worldNode`'s RTF stats reflect real real-time-driven refreshes
/// (i.e. RealTimeWorldNode::refresh() has actually run with simulate() on) and
/// are therefore meaningful to display. Deterministic stepping paths --
/// headless capture, --cycle-scenes, the paused/Step-button path -- all drive
/// the world directly via AppWorldNode::stepOnce(), bypassing refresh()'s
/// wall-clock budget entirely, so mLowestRealTimeFactor is left at its
/// never-updated +infinity default (and the smoothed/highest fields would
/// otherwise misleadingly read 0.00 instead of "not measured").
bool hasLiveRtfStats(
    const dart::gui::Viewer* viewer, const AppWorldNode* worldNode)
{
  return worldNode && viewer->isSimulating()
         && std::isfinite(worldNode->getLowestRealTimeFactor());
}

//==============================================================================
float calcButtonWidth(const char* label)
{
  const ImGuiStyle& style = ImGui::GetStyle();
  return ImGui::CalcTextSize(label).x + 2.0f * style.FramePadding.x;
}

//==============================================================================
void sameLineIfEnoughRoom(float nextItemWidth)
{
  const ImGuiStyle& style = ImGui::GetStyle();
  // Content right edge in screen coordinates. The cursor sits at the start of
  // the next line here (the previous item ended the line), so cursor + avail
  // spans the full content width; GetContentRegionMax() is obsolete and
  // removed from the bundled docking build.
  const float windowRight
      = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
  const float nextRight
      = ImGui::GetItemRectMax().x + style.ItemSpacing.x + nextItemWidth;
  if (nextRight <= windowRight)
    ImGui::SameLine();
}

//==============================================================================
float clampedItemWidth(float preferred, float minimum)
{
  const float avail = std::max(1.0f, ImGui::GetContentRegionAvail().x);
  return std::clamp(preferred, std::min(minimum, avail), avail);
}

//==============================================================================
std::vector<std::string> getAvailableCollisionDetectorNames()
{
  std::vector<std::string> names;
  auto* factory = dart::collision::CollisionDetector::getFactory();
  if (!factory)
    return names;

  const auto keys = factory->getKeys();
  names.reserve(keys.size());
  for (const auto& key : keys)
    names.push_back(key);

  std::sort(names.begin(), names.end());
  return names;
}

//==============================================================================
std::string getWorldCollisionDetectorName(
    const dart::simulation::WorldPtr& world)
{
  if (!world)
    return "none";

  const auto detector = world->getCollisionDetector();
  return detector ? detector->getType() : "none";
}

//==============================================================================
struct SceneCounts
{
  std::size_t skeletons = 0;
  std::size_t bodyNodes = 0;
  std::size_t softBodies = 0;
  std::size_t pointMasses = 0;
};

//==============================================================================
SceneCounts collectSceneCounts(const dart::simulation::WorldPtr& world)
{
  SceneCounts counts;
  if (!world)
    return counts;

  counts.skeletons = world->getNumSkeletons();
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto& skeleton = world->getSkeleton(i);
    counts.bodyNodes += skeleton->getNumBodyNodes();
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      const auto* softBodyNode = skeleton->getBodyNode(j)->asSoftBodyNode();
      if (!softBodyNode)
        continue;

      ++counts.softBodies;
      counts.pointMasses += softBodyNode->getNumPointMasses();
    }
  }

  return counts;
}

//==============================================================================
/// Forwards ImGui rendering to the host; kept alive by the viewer's
/// widget list once registered.
class HostPanelWidget : public dart::gui::ImGuiWidget
{
public:
  explicit HostPanelWidget(App* host) : mHost(host) {}

  void render() override
  {
    mHost->renderPanels();
  }

private:
  App* mHost;
};

//==============================================================================
/// Persistent (never torn down) key handler that dispatches to the active
/// scene's key actions. Installed once at startup.
class DemoKeyHandler : public dart::gui::GUIEventHandler
{
public:
  explicit DemoKeyHandler(App* host) : mHost(host) {}

  bool handle(
      const dart::gui::GUIEventAdapter& ea,
      dart::gui::GUIActionAdapter&) override
  {
    if (ea.getEventType() != dart::gui::GUIEventAdapter::KEYDOWN)
      return false;

    // Never let typing in an ImGui field (e.g. the always-visible "Search
    // demos..." box) trigger a scene key action. The viewer already drops key
    // presses while ImGui wants the keyboard; this also covers held keys.
    if (ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureKeyboard)
      return false;

    return mHost->handleKey(ea.getKey());
  }

private:
  App* mHost;
};

} // namespace

//==============================================================================
CameraHome::CameraHome(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up)
  : eye(eye), center(center), up(up)
{
  // Same predicate as Viewer::setCameraHome(), checked where the pose is made.
  if (!eye.allFinite() || !center.allFinite() || !up.allFinite()
      || (eye - center).norm() < 1e-12 || up.norm() < 1e-12
      || (center - eye).cross(up).norm() < 1e-12)
    throw std::invalid_argument(
        "Camera vectors must be finite, distinct and nonparallel");
}

//==============================================================================
SceneContext::SceneContext(App& host, std::weak_ptr<const void> activation)
  : mHost(&host), mActivation(std::move(activation))
{
}

//==============================================================================
App& SceneContext::host() const
{
  if (mActivation.expired())
    throw std::logic_error(
        "This scene context expired when its scene was torn down");
  return *mHost;
}

//==============================================================================
dart::gui::Viewer* SceneContext::viewer() const
{
  return host().getViewer();
}

//==============================================================================
dart::gui::WorldNode* SceneContext::worldNode() const
{
  return host().getWorldNode();
}

//==============================================================================
std::shared_ptr<dart::gui::Viewer> SceneContext::viewerPtr() const
{
  return host().getViewerPtr();
}

//==============================================================================
std::shared_ptr<dart::gui::WorldNode> SceneContext::worldNodePtr() const
{
  return host().getWorldNodePtr();
}

//==============================================================================
void SceneContext::addTeardown(std::function<void()> teardown)
{
  host().addTeardown(std::move(teardown));
}

//==============================================================================
void SceneContext::addOwnedAttachment(
    std::shared_ptr<dart::gui::ViewerAttachment> attachment)
{
  auto* viewer = host().getViewer();
  // Register the removal first: addTeardown() throws during a teardown.
  addTeardown([viewer, attachment] { viewer->removeAttachment(attachment); });
  viewer->addAttachment(attachment);
}

//==============================================================================
void SceneContext::addEventHandler(
    std::shared_ptr<dart::gui::GUIEventHandler> handler)
{
  auto* viewer = host().getViewer();
  addTeardown([viewer, handler] { viewer->removeEventHandler(handler); });
  viewer->addEventHandler(handler);
}

//==============================================================================
void SceneContext::log(const std::string& message) const
{
  host().log(LogEntry::Level::Info, message);
}

//==============================================================================
AppWorldNode::AppWorldNode(const dart::simulation::WorldPtr& world)
  : dart::gui::RealTimeWorldNode(world), mStepCount(0)
{
}

//==============================================================================
void AppWorldNode::setHooks(
    std::function<void()> preStep,
    std::function<void()> postStep,
    std::function<void()> preRefresh)
{
  mPreStep = std::move(preStep);
  mPostStep = std::move(postStep);
  mPreRefresh = std::move(preRefresh);
}

//==============================================================================
void AppWorldNode::setHookErrorSink(
    std::function<void(const std::string&, const std::string&)> sink)
{
  mHookErrorSink = std::move(sink);
}

//==============================================================================
void AppWorldNode::stepOnce()
{
  customPreStep();
  if (mStepStopped)
    return;
  if (getWorld())
    getWorld()->step();
  customPostStep();
}

//==============================================================================
std::size_t AppWorldNode::getStepCount() const
{
  return mStepCount;
}

//==============================================================================
void AppWorldNode::customPreStep()
{
  invokeHook(mPreStep, "preStep");
  // refresh() checks isSimulating() before each step of a catch-up.
  mStepStopped = mStopRequested && mStopRequested();
  if (mStepStopped) {
    simulate(false);
    return;
  }
  beginStepTiming();
}

//==============================================================================
void AppWorldNode::setStopRequested(std::function<bool()> stopRequested)
{
  mStopRequested = std::move(stopRequested);
}

//==============================================================================
void AppWorldNode::customPostStep()
{
  endStepTiming();
  invokeHook(mPostStep, "postStep");
  ++mStepCount;
}

//==============================================================================
void AppWorldNode::customPreRefresh()
{
  mStepCountAtRefreshStart = mStepCount;
  invokeHook(mPreRefresh, "preRefresh");
}

//==============================================================================
void AppWorldNode::customPostRefresh()
{
  mLastRefreshStepCount = mStepCount - mStepCountAtRefreshStart;
}

//==============================================================================
std::size_t AppWorldNode::getLastRefreshStepCount() const
{
  return mLastRefreshStepCount;
}

//==============================================================================
double AppWorldNode::getLastStepMs() const
{
  return mLastStepMs;
}

//==============================================================================
double AppWorldNode::getMovingAverageStepMs() const
{
  return mMovingAverageStepMs;
}

//==============================================================================
double AppWorldNode::getMinStepMs() const
{
  return mMinStepMs;
}

//==============================================================================
double AppWorldNode::getMaxStepMs() const
{
  return mMaxStepMs;
}

//==============================================================================
std::size_t AppWorldNode::getStepTimingSamples() const
{
  return mStepTimingSamples;
}

//==============================================================================
void AppWorldNode::invokeHook(std::function<void()>& hook, const char* name)
{
  if (!hook)
    return;

  try {
    hook();
  } catch (const std::exception& e) {
    if (mHookErrorSink)
      mHookErrorSink(name, e.what());
    hook = nullptr;
  } catch (...) {
    if (mHookErrorSink)
      mHookErrorSink(name, "unknown error");
    hook = nullptr;
  }
}

//==============================================================================
void AppWorldNode::beginStepTiming()
{
  mStepStart = std::chrono::steady_clock::now();
  mStepTimingActive = true;
}

//==============================================================================
void AppWorldNode::endStepTiming()
{
  if (!mStepTimingActive)
    return;

  mStepTimingActive = false;
  mLastStepMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - mStepStart)
                    .count();

  if (mStepTimingSamples == 0u) {
    mMovingAverageStepMs = mLastStepMs;
    mMinStepMs = mLastStepMs;
    mMaxStepMs = mLastStepMs;
  } else {
    constexpr double kAlpha = 0.08;
    mMovingAverageStepMs += kAlpha * (mLastStepMs - mMovingAverageStepMs);
    mMinStepMs = std::min(mMinStepMs, mLastStepMs);
    mMaxStepMs = std::max(mMaxStepMs, mLastStepMs);
  }

  ++mStepTimingSamples;
}

//==============================================================================
App::App(
    std::vector<Scene> scenes,
    std::optional<double> guiScale,
    std::string collisionDetectorName,
    std::size_t simulationThreads)
  : mScenes(std::move(scenes)),
    mPerformanceStatsPanel(240u),
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
    mMemoryDiagnosticsSession([this] {
      return dart::gui::app::collectMemoryDiagnostics(
          mCurrentWorld, mMemoryDiagnosticsGeneration);
    }),
#endif
    mRequestedCollisionDetectorName(toLower(collisionDetectorName)),
    mSimulationThreads(static_cast<int>(std::min<std::size_t>(
        simulationThreads,
        static_cast<std::size_t>(std::numeric_limits<int>::max())))),
    mGuiScale(dart_examples::sanitizeGuiScale(guiScale.value_or(1.0))),
    mAutomaticGuiScale(!guiScale.has_value())
{
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  const char* memoryDiagnosticsEnv
      = std::getenv("DART_DEMOS_MEMORY_DIAGNOSTICS");
  mDebugMemoryDiagnostics
      = memoryDiagnosticsEnv && std::strcmp(memoryDiagnosticsEnv, "1") == 0;
  mMemoryDiagnosticsSession.setEnabled(mDebugMemoryDiagnostics);
#endif

  buildCategories();
  mAvailableCollisionDetectors = getAvailableCollisionDetectorNames();
  mViewer = std::make_shared<dart::gui::Viewer>();

  // Captures dart::common's dtmsg/dtwarn/dterr output (and any other
  // std::cout/std::cerr writes) into the same log the host already uses for
  // its own app-event messages, so the Diagnostics log console shows both
  // through one severity-filtered, autoscrolling view. Restored (stream
  // buffers un-redirected) when mLogCapture is destroyed alongside this host.
  mLogCapture = std::make_unique<LogCapture>(
      [this](CapturedLogLevel level, const std::string& message) {
        LogEntry::Level mapped = LogEntry::Level::Info;
        if (level == CapturedLogLevel::Warning)
          mapped = LogEntry::Level::Warning;
        else if (level == CapturedLogLevel::Error)
          mapped = LogEntry::Level::Error;
        log(mapped, message);
      });
}

//==============================================================================
App::~App()
{
  mGuiScaleTheme.reset();
  // See the header comment: without this, whichever scene is still active at
  // process exit leaks its DnD/attachment/event-handler teardowns into the
  // viewer's own destruction instead of running them first.
  teardownCurrentScene();

  // A caller may keep the viewer (SceneContext::viewerPtr()); it must not
  // call into this host afterwards.
  mDragForce.uninstallMouseHandler();
  if (mPanelWidget)
    mViewer->removeWidget(mPanelWidget);
  if (mKeyHandler)
    mViewer->removeEventHandler(mKeyHandler);
  if (mGridVisual)
    mViewer->removeAttachment(mGridVisual);
}

//==============================================================================
void App::buildCategories()
{
  mCategories.clear();
  mCategoryIndexByName.clear();

  for (std::size_t i = 0; i < mScenes.size(); ++i) {
    const std::string& category = mScenes[i].category;
    const auto found = mCategoryIndexByName.find(category);
    std::size_t categoryIndex;
    if (found == mCategoryIndexByName.end()) {
      categoryIndex = mCategories.size();
      mCategoryIndexByName.emplace(category, categoryIndex);
      mCategories.push_back(CategoryGroup{category, {}});
    } else {
      categoryIndex = found->second;
    }
    mCategories[categoryIndex].sceneIndices.push_back(i);
  }
}

//==============================================================================
const Scene* App::findScene(const std::string& id) const
{
  for (const auto& scene : mScenes) {
    if (scene.id == id)
      return &scene;
  }
  return nullptr;
}

//==============================================================================
void App::setInitialScene(const std::string& id)
{
  mInitialSceneId = id;
}

//==============================================================================
void App::setDebugSelectBodyName(const std::string& name)
{
  mDebugSelectBodyName = name;
}

//==============================================================================
void App::setDebugRecordProfile(bool on)
{
  mDebugRecordProfile = on;
}

//==============================================================================
void App::setDebugPrintWorld(bool on)
{
  mDebugPrintWorld = on;
}

//==============================================================================
void App::setStopRequested(std::function<bool()> stopRequested)
{
  mStopRequested = std::move(stopRequested);
}

//==============================================================================
void App::setStopException(std::function<bool(const std::exception&)> isStop)
{
  mIsStopException = std::move(isStop);
}

//==============================================================================
bool App::stopRequested() const
{
  // Latched until stopRun(): a predicate may report a stop only once, and
  // the world node's poll must not consume it before the run's own poll.
  mStopLatched = mStopLatched || (mStopRequested && mStopRequested());
  return mStopLatched;
}

//==============================================================================
int App::stopRun()
{
  // A stopped run tears its scene down like any other ending.
  teardownCurrentScene();
  mStopLatched = false;
  return kStoppedExitCode;
}

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
//==============================================================================
void App::setDebugScrollMemoryPanelTicks(int ticks)
{
  mDebugScrollMemoryPanelTicks = std::max(0, ticks);
}
#endif

//==============================================================================
void App::requestScenePanelTab(ScenePanelTab tab)
{
  mRequestedScenePanelTab = tab;
  mHasRequestedScenePanelTab = true;
}

//==============================================================================
std::size_t App::getActiveWorldNodeCount() const
{
  return mActiveWorldNodeCount;
}

//==============================================================================
std::size_t App::getCallbackFailureCount() const
{
  return mCallbackFailures;
}

//==============================================================================
dart::gui::Viewer* App::getViewer() const
{
  return mViewer.get();
}

//==============================================================================
AppWorldNode* App::getWorldNode() const
{
  return mWorldNode.get();
}

//==============================================================================
std::shared_ptr<dart::gui::Viewer> App::getViewerPtr() const
{
  return mViewer;
}

//==============================================================================
std::shared_ptr<AppWorldNode> App::getWorldNodePtr() const
{
  return mWorldNode;
}

//==============================================================================
void App::addTeardown(std::function<void()> teardown)
{
  if (mTearingDown)
    throw std::logic_error("Cannot add a teardown while the scene tears down");
  mExtraTeardowns.push_back(std::move(teardown));
}

//==============================================================================
void App::invokeSceneCallback(
    std::function<void()>& callback, const std::string& what)
{
  if (!callback)
    return;

  try {
    callback();
    return;
  } catch (const std::exception& e) {
    reportFailure(
        "Demo '" + mCurrentSceneTitle + "' " + what + " failed: " + e.what()
        + " (disabled)");
  } catch (...) {
    reportFailure(
        "Demo '" + mCurrentSceneTitle + "' " + what
        + " failed: unknown error (disabled)");
  }
  callback = nullptr;
  ++mCallbackFailures;
}

//==============================================================================
void App::log(LogEntry::Level level, const std::string& message)
{
  mLog.push_back(LogEntry{level, message});

  // A ring buffer of ~2000 entries: generous enough to survive a captured
  // dtmsg/dtwarn/dterr burst (e.g. a noisy solver) without losing the app's
  // own recent status messages, while staying bounded.
  constexpr std::size_t kMaxLogEntries = 2000;
  while (mLog.size() > kMaxLogEntries)
    mLog.pop_front();
}

//==============================================================================
void App::reportFailure(const std::string& message)
{
  log(LogEntry::Level::Error, message);
  // Display-free runs show no log console, so failures go to the terminal.
  if (mEchoErrors)
    std::cerr << message << std::endl;
}

//==============================================================================
void App::syncCollisionDetectorSelectionFromWorld()
{
  mCurrentCollisionDetectorName = getWorldCollisionDetectorName(mCurrentWorld);
  mCollisionDetectorIndex = -1;
  for (std::size_t i = 0; i < mAvailableCollisionDetectors.size(); ++i) {
    if (mAvailableCollisionDetectors[i] == mCurrentCollisionDetectorName) {
      mCollisionDetectorIndex = static_cast<int>(i);
      break;
    }
  }
}

//==============================================================================
bool App::setCollisionDetectorByName(const std::string& name)
{
  if (!mCurrentWorld || name.empty())
    return true;

  auto* factory = dart::collision::CollisionDetector::getFactory();
  if (!factory || !factory->canCreate(name)) {
    log(LogEntry::Level::Warning,
        "Collision detector '" + name + "' is not available; keeping '"
            + getWorldCollisionDetectorName(mCurrentWorld) + "'.");
    syncCollisionDetectorSelectionFromWorld();
    return false;
  }

  auto detector = factory->create(name);
  if (!detector) {
    log(LogEntry::Level::Warning,
        "Collision detector '" + name + "' failed to create; keeping '"
            + getWorldCollisionDetectorName(mCurrentWorld) + "'.");
    syncCollisionDetectorSelectionFromWorld();
    return false;
  }

  mCurrentWorld->setCollisionDetector(detector);
  mRequestedCollisionDetectorName = detector->getType();
  syncCollisionDetectorSelectionFromWorld();
  mPerformanceStatsPanel.reset();
  log(LogEntry::Level::Info,
      "Collision detector set to '" + mCurrentCollisionDetectorName + "'.");
  return true;
}

//==============================================================================
void App::applyRuntimeOptionsToWorld()
{
  if (!mCurrentWorld)
    return;

  if (mSimulationThreads < 0)
    mSimulationThreads = 1;
  mCurrentWorld->setNumSimulationThreads(
      static_cast<std::size_t>(mSimulationThreads));
  mSimulationThreads
      = static_cast<int>(mCurrentWorld->getNumSimulationThreads());

  if (!mRequestedCollisionDetectorName.empty())
    setCollisionDetectorByName(mRequestedCollisionDetectorName);
  else
    syncCollisionDetectorSelectionFromWorld();
}

//==============================================================================
void App::ensureViewerConfigured()
{
  if (mViewerConfigured)
    return;
  mViewerConfigured = true;

  mViewer->setBackgroundColor(Eigen::Vector4d(0.58, 0.62, 0.65, 1.0));
  mViewer->setVerticalFieldOfView(30.0);
  if (!mAutomaticGuiScale)
    mViewer->setGuiScale(mGuiScale);
  mPanelWidget = std::make_shared<HostPanelWidget>(this);
  mViewer->addWidget(mPanelWidget);
  mKeyHandler = std::make_shared<DemoKeyHandler>(this);
  mViewer->addEventHandler(mKeyHandler);

  // Host-wide picking: plain left-click selects a body for the Inspector;
  // Ctrl+left-drag starts a spring-force drag (DragForce owns both, since it
  // needs the same persistent mouse handler -- see DragForce.hpp).
  mDragForce.installMouseHandler(
      mViewer.get(), [this](dart::dynamics::BodyNode* body) {
        mInspector.setSelection(body);
      });

  // View-utilities grid attachment: host chrome, persists across scene
  // switches (unlike a scene's own attachments, which are torn down on
  // switch via ctx.addTeardown). Off by default -- most scenes already
  // render their own ground plane/floor.
  mGridVisual = std::make_shared<dart::gui::GridVisual>();
  mGridVisual->display(false);
  mViewer->addAttachment(mGridVisual);
}

void App::initializeGuiTheme()
{
  if (mGuiScaleTheme || !mViewer->getImGuiContext())
    return;
  auto* previous = ImGui::GetCurrentContext();
  ImGui::SetCurrentContext(mViewer->getImGuiContext());
  applyModernDarkColors();
  applyModernDarkMetrics();
  mGuiScaleTheme = std::make_unique<GuiScaleTheme>(*mViewer);
  ImGui::SetCurrentContext(previous);
}

//==============================================================================
void App::applyShadowState()
{
  if (!mWorldNode)
    return;

  mWorldNode->setShadowsEnabled(mShadowsEnabled && mCurrentSceneWantsShadows);
}

//==============================================================================
void App::requestSceneSwitch(const std::string& id)
{
  mPendingSceneId = id;
}

//==============================================================================
void App::processPendingSwitch()
{
  if (!mPendingSceneId.has_value())
    return;

  const std::string requestedId = *mPendingSceneId;
  mPendingSceneId.reset();
  mLastSwitchSkipReason.clear();

  const Scene* scene = findScene(requestedId);
  if (!scene) {
    mLastSwitchFailed = true;
    log(LogEntry::Level::Error, "Unknown demo id '" + requestedId + "'");
    return;
  }

  mStatusLine = "Starting demo '" + scene->title + "'...";
  log(LogEntry::Level::Info, mStatusLine);

  SceneSetup setup;
  bool ok = true;
  std::string failureReason;
  try {
    setup = scene->factory();
    if (!setup.world)
      throw std::runtime_error("factory returned a null world");
  } catch (const AssetsNotPrefetched& e) {
    ok = false;
    failureReason = e.what();
    mLastSwitchSkipReason = failureReason;
  } catch (const std::exception& e) {
    ok = false;
    failureReason = e.what();
  } catch (...) {
    ok = false;
    failureReason = "unknown error";
  }

  if (!ok) {
    mLastSwitchFailed = true;
    if (!mCurrentSceneId.empty()) {
      mStatusLine = "Restored previous demo '" + mCurrentSceneTitle
                    + "' after '" + scene->title + "' failed: " + failureReason;
    } else {
      mStatusLine
          = "Failed to start demo '" + scene->title + "': " + failureReason;
      teardownCurrentScene();
      installEmptyFallback();
    }
    log(LogEntry::Level::Error, mStatusLine);
    return;
  }

  mLastSwitchFailed = false;
  // Rebuild/Reset re-run the factory of the demo that is already active; that
  // must reset only the physics, not the user's camera pose. Only a switch to
  // a different demo applies the new scene's home framing.
  const bool sameSceneReactivation = requestedId == mCurrentSceneId;
  teardownCurrentScene();
  installScene(*scene, std::move(setup), !sameSceneReactivation);
  mCurrentSceneId = scene->id;
  mCurrentSceneTitle = scene->title;
  mStatusLine = "Running demo '" + scene->title + "'";
  log(LogEntry::Level::Info, mStatusLine);
}

//==============================================================================
void App::teardownCurrentScene()
{
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  // A snapshot must never retain comparison/history state from the World that
  // is about to be released. DiagnosticSession::reset() preserves the user's
  // enable preference and its already-reserved bounded ring storage.
  mMemoryDiagnosticsSession.reset();
#endif

  // Host-level facilities first: they hold raw BodyNode/DegreeOfFreedom
  // pointers and SimpleFrame/InteractiveFrame objects that belong to the
  // world about to be destroyed below.
  mInspector.reset();
  mDragForce.reset(mViewer.get());
  mContactVisualizer.reset();

  // No panel callback or queued click may run while the scene is released.
  if (mCurrentPanel)
    mCurrentPanel->clearCallbacks();
  mPendingKeyAction.reset();

  // Tear down drag-and-drop / attachments / event handlers before the world
  // node and world itself, since these extras may hold raw pointers into
  // objects the world owns (e.g. a SimpleFrame's DnD registration). The
  // teardowns run from a snapshot; adding one meanwhile throws.
  auto teardowns = std::exchange(mExtraTeardowns, {});
  mTearingDown = true;
  for (auto it = teardowns.rbegin(); it != teardowns.rend(); ++it)
    invokeSceneCallback(*it, "teardown");
  // Destroying the callables can run destructors that try to add more.
  teardowns.clear();
  mTearingDown = false;
  // The scene's contexts expire only now, so its teardowns could use them.
  mActivation.reset();

  if (mWorldNode) {
    // Something else, such as a language binding, may still hold the node;
    // it must not call into the scene once it is gone.
    mWorldNode->setHooks({}, {}, {});
    mWorldNode->setHookErrorSink({});
    mWorldNode->setStopRequested({});
    mViewer->removeWorldNode(mWorldNode);
    mWorldNode = nullptr;
    if (mActiveWorldNodeCount > 0)
      --mActiveWorldNodeCount;
  }

  // Undo only what installScene() changed: an owner may have disallowed
  // simulation itself.
  if (mRestoreSimulationOnTeardown) {
    mViewer->allowSimulation(true);
    if (mResumeSimulationOnTeardown)
      mViewer->simulate(true);
  }
  mRestoreSimulationOnTeardown = false;
  mResumeSimulationOnTeardown = false;

  mCurrentWorld.reset();
  mCurrentRenderPanel = nullptr;
  mCurrentKeyActions.clear();
  // Destroying the scene's callables may have added panel callbacks, which
  // could keep the panel alive in a cycle.
  if (mCurrentPanel)
    mCurrentPanel->clearCallbacks();
  mCurrentPanel.reset();
  mCurrentCameraHome.reset();
  mCurrentSceneId.clear();
  mCurrentSceneTitle.clear();
}

//==============================================================================
void App::installScene(
    const Scene& scene, SceneSetup setup, bool applyCameraHomePose)
{
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  ++mMemoryDiagnosticsGeneration;
  mMemoryDiagnosticsSession.reset();
#endif
  mCurrentSceneTitle = scene.title;
  mActivation = std::make_shared<int>(0);
  mCurrentWorld = setup.world;
  applyRuntimeOptionsToWorld();
  mPerformanceStatsPanel.reset();
  mDragForce.onSceneInstalled(mCurrentWorld);
  mContactVisualizer.onSceneInstalled(mCurrentWorld);
  mCurrentSceneWantsShadows = setup.enableShadows;

  mWorldNode = std::make_shared<AppWorldNode>(mCurrentWorld);
  mWorldNode->setStopRequested([this] { return stopRequested(); });
  applyShadowState();

  // Compose the host-level facilities (joint-slider edits, drag-force/
  // gizmo spring, contact-visualizer refresh) around the scene's own hooks.
  // Only the scene's part is guarded and disabled when it throws, so the
  // facilities keep running. They never throw themselves; if one did,
  // AppWorldNode::invokeHook would disable the whole composite.
  mWorldNode->setHooks(
      [this, scenePreStep = std::move(setup.preStep)]() mutable {
        mInspector.applyQueuedEdits();
        mDragForce.applyPreStep();
        invokeSceneCallback(scenePreStep, "preStep");
      },
      [this, scenePostStep = std::move(setup.postStep)]() mutable {
        invokeSceneCallback(scenePostStep, "postStep");
        mContactVisualizer.applyPostStep();
      },
      [this, scenePreRefresh = std::move(setup.preRefresh)]() mutable {
        mInspector.applyQueuedEdits();
        // A steady clock, because display-free runs have no ImGui context.
        mInspector.updateHighlight(
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - mStartTime)
                .count());
        mDragForce.applyPreRefresh(!mViewer->isSimulating());
        invokeSceneCallback(scenePreRefresh, "preRefresh");
      });
  mWorldNode->setHookErrorSink(
      [this](const std::string& hookName, const std::string& reason) {
        reportFailure(
            "Demo '" + mCurrentSceneTitle + "' " + hookName
            + " failed: " + reason + " (disabled)");
        ++mCallbackFailures;
      });

  mTargetRtf = 1.0f;
  mWorldNode->setTargetRealTimeFactor(mTargetRtf);

  // Re-read the new world's timestep so the toolbar's Timestep control reflects
  // the active scene (each scene factory sets its own; e.g. the chain scenes
  // use 1/2000 s while most others use 1/1000 s).
  mTimeStep = static_cast<float>(mCurrentWorld->getTimeStep());

  mGravityEnabled = mCurrentWorld->getGravity().norm() > 1e-8;
  mSavedGravity = mGravityEnabled ? mCurrentWorld->getGravity()
                                  : Eigen::Vector3d(0.0, 0.0, -9.81);

  mViewer->addWorldNode(mWorldNode);
  ++mActiveWorldNodeCount;

  if (!setup.allowSimulation && mViewer->isAllowingSimulation()) {
    mResumeSimulationOnTeardown = mViewer->isSimulating();
    mViewer->allowSimulation(false);
    mRestoreSimulationOnTeardown = true;
  }

  mCurrentPanel = std::move(setup.panel);
  mCurrentRenderPanel = setup.renderPanel;
  mCurrentKeyActions = std::move(setup.keyActions);
  mCurrentCameraHome = setup.cameraHome;

  for (const auto& frame : setup.dragFrames) {
    if (!frame)
      continue;
    auto* dnd = mViewer->enableDragAndDrop(frame.get());
    if (dnd) {
      auto* viewer = mViewer.get();
      mExtraTeardowns.push_back(
          [viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    }
  }

  if (setup.onActivate) {
    SceneContext ctx(*this, mActivation);
    std::function<void()> activate = [&] {
      setup.onActivate(ctx);
    };
    invokeSceneCallback(activate, "onActivate");
  }

  // Rebuilding the active demo keeps the user's camera pose; the scene's home
  // framing still refreshes above (mCurrentCameraHome) so "Reset camera" and
  // the offscreen capture home keep using the scene's canonical view.
  if (applyCameraHomePose) {
    const CameraHome defaultHome{
        Eigen::Vector3d(6.0, 8.0, 4.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 0.0, 1.0)};
    applyCameraHome(mCurrentCameraHome.value_or(defaultHome), true);
  }
}

//==============================================================================
void App::installEmptyFallback()
{
  Scene fallback;
  fallback.id.clear();
  fallback.title = "(no demo loaded)";

  SceneSetup setup;
  setup.world = dart::simulation::World::create();
  setup.enableShadows = false;

  installScene(fallback, std::move(setup));
  mCurrentSceneId.clear();
  mCurrentSceneTitle = fallback.title;
}

//==============================================================================
void App::applyCameraHome(const CameraHome& home, bool viaManipulator)
{
  (void)viaManipulator;
  mViewer->setCameraHome(home.eye, home.center, home.up);
}

//==============================================================================
void App::fitCameraToWorld()
{
  if (!mCurrentWorld)
    return;
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero(), maximum = minimum;
  bool found = false;
  for (std::size_t i = 0; i < mCurrentWorld->getNumSkeletons(); ++i) {
    const auto skeleton = mCurrentWorld->getSkeleton(i);
    if (!skeleton->isMobile())
      continue;
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      const Eigen::Vector3d position
          = skeleton->getBodyNode(j)->getTransform().translation();
      // A diverged body must not make the camera pose invalid.
      if (!position.allFinite())
        continue;
      if (!found) {
        minimum = maximum = position;
        found = true;
      } else {
        minimum = minimum.cwiseMin(position);
        maximum = maximum.cwiseMax(position);
      }
    }
  }
  if (!found) {
    mViewer->setCameraHome(
        Eigen::Vector3d(0.0, -20.0, 10.0),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitZ());
    return;
  }
  const Eigen::Vector3d center = 0.5 * (minimum + maximum),
                        extent = maximum - minimum;
  const double distance = std::max(
      10.0, std::max({extent.x(), extent.y(), extent.z() * 1.2}) * 0.9 + 8.0);
  const double lookAtZ = std::clamp(0.6 * center.z(), 0.0, 5.0);
  mViewer->setCameraHome(
      Eigen::Vector3d(
          center.x(),
          center.y() - distance,
          lookAtZ + std::max(6.0, distance * 0.55)),
      Eigen::Vector3d(center.x(), center.y(), lookAtZ),
      Eigen::Vector3d::UnitZ());
}

//==============================================================================
void App::printWorld() const
{
  const SceneCounts counts = collectSceneCounts(mCurrentWorld);
  std::size_t dofs = 0;
  if (mCurrentWorld) {
    for (std::size_t i = 0; i < mCurrentWorld->getNumSkeletons(); ++i)
      dofs += mCurrentWorld->getSkeleton(i)->getNumDofs();
  }
  const std::size_t constraints
      = mCurrentWorld
            ? mCurrentWorld->getConstraintSolver()->getNumConstraints()
            : std::size_t{0};
  std::cout << "[world] skeletons " << counts.skeletons << ", bodies "
            << counts.bodyNodes << ", soft bodies " << counts.softBodies
            << ", point masses " << counts.pointMasses << ", dofs " << dofs
            << ", constraints " << constraints << "\n";
  if (!mCurrentWorld)
    return;
  for (std::size_t i = 0; i < mCurrentWorld->getNumSkeletons(); ++i) {
    const auto skeleton = mCurrentWorld->getSkeleton(i);
    std::size_t softBodies = 0;
    std::size_t pointMasses = 0;
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      if (const auto* soft = skeleton->getBodyNode(j)->asSoftBodyNode()) {
        ++softBodies;
        pointMasses += soft->getNumPointMasses();
      }
    }
    std::cout << "[world] skeleton '" << skeleton->getName() << "': bodies "
              << skeleton->getNumBodyNodes() << ", soft bodies " << softBodies
              << ", point masses " << pointMasses << ", dofs "
              << skeleton->getNumDofs() << "\n";
  }
}

//==============================================================================
int App::listScenes() const
{
  for (const auto& category : mCategories) {
    std::cout << category.name << "\n";
    for (const auto index : category.sceneIndices) {
      const Scene& scene = mScenes[index];
      std::cout << "  " << scene.id << "  -  " << scene.title;
      if (!scene.summary.empty())
        std::cout << ": " << scene.summary;
      std::cout << "\n";
    }
  }
  return 0;
}

//==============================================================================
int App::cycleScenes(int framesPerScene, bool render, int width, int height)
{
  mEchoErrors = true;
  bool anyFailure = false;
  std::size_t skipped = 0;
  if (render) {
    ensureViewerConfigured();
    const CameraHome home{
        Eigen::Vector3d(6.0, 8.0, 4.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 0.0, 1.0)};
    if (!prepareOffscreenContext(width, height, home))
      return 1;
  }

  // Loop the whole catalog twice in-process: this rapid re-switch robustness
  // gate must not leak world nodes or corrupt state across repeated switches.
  for (int repeat = 0; repeat < 2; ++repeat) {
    for (const auto& scene : mScenes) {
      if (stopRequested())
        return stopRun();
      // Includes the previous scene's teardowns, which run in this switch.
      const std::size_t failuresBefore = mCallbackFailures;
      requestSceneSwitch(scene.id);
      processPendingSwitch();
      processPanelEvents();

      if (!mLastSwitchSkipReason.empty()) {
        std::cout << "[cycle-scenes] demo '" << scene.id
                  << "' skipped: " << mLastSwitchSkipReason << "\n";
        ++skipped;
      } else if (mLastSwitchFailed) {
        std::cerr << "[cycle-scenes] demo '" << scene.id
                  << "' failed to start.\n";
        anyFailure = true;
      }

      try {
        for (int frame = 0; frame < framesPerScene && mWorldNode; ++frame) {
          if (stopRequested())
            return stopRun();
          if (render) {
            mWorldNode->stepOnce();
            mViewer->frame();
          } else {
            // Without a viewer frame, run the per-frame hooks it would.
            mWorldNode->customPreRefresh();
            mWorldNode->stepOnce();
            mWorldNode->customPostRefresh();
          }
          processPanelEvents();
        }
      } catch (const std::exception& error) {
        // A stop exception ends the run at the next poll instead.
        if (!(mIsStopException && mIsStopException(error))) {
          std::cerr << "[cycle-scenes] demo '" << scene.id
                    << "' failed to render: " << error.what() << "\n";
          anyFailure = true;
        }
      }

      if (mCallbackFailures != failuresBefore) {
        std::cerr << "[cycle-scenes] demo '" << scene.id << "' disabled "
                  << mCallbackFailures - failuresBefore << " callback(s).\n";
        anyFailure = true;
      }

      if (mActiveWorldNodeCount != 1) {
        std::cerr << "[cycle-scenes] world-node leak detected after '"
                  << scene.id << "' (active=" << mActiveWorldNodeCount << ")\n";
        anyFailure = true;
      }
    }
  }

  // A stop requested by the last frame still ends the run as stopped.
  if (stopRequested())
    return stopRun();

  // Tear the last scene down too, so the result includes its teardowns.
  const std::size_t failuresBefore = mCallbackFailures;
  teardownCurrentScene();
  if (mCallbackFailures != failuresBefore) {
    std::cerr << "[cycle-scenes] the final teardown disabled "
              << mCallbackFailures - failuresBefore << " callback(s).\n";
    anyFailure = true;
  }

  std::cout << "[cycle-scenes] cycled " << mScenes.size() << " demo(s) x2, "
            << framesPerScene << " frame(s) each, " << skipped << " skipped. "
            << (anyFailure ? "FAILED" : "OK") << "\n";
  return anyFailure ? 1 : 0;
}

//==============================================================================
bool App::prepareOffscreenContext(int width, int height, const CameraHome& home)
{
  mGuiScaleTheme.reset();
  mViewer->setCameraHome(home.eye, home.center, home.up);
  mViewer->simulate(false);
  try {
    mViewer->setUpViewForCapture(width, height);
  } catch (const std::exception& error) {
    std::cerr << "[headless] Failed to create an off-screen capture context: "
              << error.what() << "\n";
    return false;
  }
  initializeGuiTheme();
  return true;
}

//==============================================================================
int App::runHeadlessShot(
    const std::string& shotPath,
    int steps,
    const std::string& sceneId,
    int width,
    int height)
{
  mEchoErrors = true;
  ensureViewerConfigured();

  const std::string initial = !sceneId.empty()           ? sceneId
                              : !mInitialSceneId.empty() ? mInitialSceneId
                              : !mScenes.empty()         ? mScenes.front().id
                                                         : std::string();
  if (initial.empty()) {
    std::cerr << "[headless] No demo scenes are registered.\n";
    return 1;
  }

  requestSceneSwitch(initial);
  processPendingSwitch();
  processPanelEvents();
  const bool sceneFailed = mLastSwitchFailed;
  if (sceneFailed) {
    std::cerr << "[headless] demo '" << initial
              << "' failed to start; capturing the fallback state.\n";
  }

  // Hidden debug/test hooks (main.cpp's --debug-select-body/
  // --debug-record-profile): let a headless capture exercise UI state that
  // normally requires interactive input: scene tree + inspector with a
  // selection, and profiler recording in the Tools tab.
  if (!mDebugSelectBodyName.empty() && mCurrentWorld) {
    dart::dynamics::BodyNode* found = nullptr;
    for (std::size_t s = 0; s < mCurrentWorld->getNumSkeletons() && !found;
         ++s) {
      found = mCurrentWorld->getSkeleton(s)->getBodyNode(mDebugSelectBodyName);
    }
    if (found) {
      mInspector.setSelection(found);
      requestScenePanelTab(ScenePanelTab::Inspector);
    } else {
      std::cerr << "[headless] --debug-select-body '" << mDebugSelectBodyName
                << "' not found in demo '" << initial << "'.\n";
    }
  }
  if (mDebugRecordProfile) {
    mProfiler.setRecordingForTest(true);
    requestScenePanelTab(ScenePanelTab::Tools);
  }
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  if (mDebugMemoryDiagnostics) {
    mMemoryDiagnosticsSession.setEnabled(true);
    requestScenePanelTab(ScenePanelTab::Memory);
  }
#endif

  const CameraHome defaultHome{
      Eigen::Vector3d(6.0, 8.0, 4.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0)};
  const CameraHome home = mCurrentCameraHome.value_or(defaultHome);

  if (stopRequested())
    return stopRun();
  if (!prepareOffscreenContext(width, height, home))
    return 1;

  for (int i = 0; i < steps && mWorldNode; ++i) {
    if (stopRequested())
      return stopRun();
    mWorldNode->stepOnce();
    processPanelEvents();
  }
  // Every frame polls for a stop first. A frame's preRefresh may queue panel
  // events too; run them like the interactive loop does.
  const auto frame = [this] {
    if (stopRequested())
      return false;
    mViewer->frame();
    processPanelEvents();
    return true;
  };
  if (mDebugPrintWorld)
    printWorld();

  errno = 0;
  if (std::remove(shotPath.c_str()) != 0 && errno != ENOENT) {
    std::cerr << "[headless] failed to remove stale screenshot '" << shotPath
              << "': " << std::strerror(errno) << "\n";
    return 1;
  }

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  if (mDebugScrollMemoryPanelTicks > 0) {
    auto* previous = ImGui::GetCurrentContext();
    ImGui::SetCurrentContext(mViewer->getImGuiContext());
    const float hoverX = static_cast<float>(width) - 60.0f;
    const float hoverY = static_cast<float>(height) / 2.0f;
    applyCameraHome(home, false);
    const auto stop = [&] {
      ImGui::SetCurrentContext(previous);
      return stopRun();
    };
    ImGui::GetIO().AddMousePosEvent(hoverX, hoverY);
    if (!frame())
      return stop();
    for (int tick = 0; tick < mDebugScrollMemoryPanelTicks; ++tick) {
      ImGui::GetIO().AddMousePosEvent(hoverX, hoverY);
      ImGui::GetIO().AddMouseWheelEvent(0.0f, -1.0f);
      if (!frame())
        return stop();
      char scrollSuffix[24];
      std::snprintf(
          scrollSuffix, sizeof(scrollSuffix), ".scroll%04d.png", tick);
      const std::string tickPath = shotPath + scrollSuffix;
      std::remove(tickPath.c_str());
      mViewer->captureScreen(tickPath);
      if (!frame())
        return stop();
    }
    ImGui::SetCurrentContext(previous);
  }

#endif

  // Re-pin the view and draw, then capture.
  if (stopRequested())
    return stopRun();
  applyCameraHome(home, false);
  if (!frame() || !frame())
    return stopRun();
  mViewer->captureScreen(shotPath);
  // SaveScreen writes the PNG during this frame.
  if (!frame())
    return stopRun();

  std::ifstream captured(shotPath, std::ios::binary);
  if (!captured.good()) {
    std::cerr << "[headless] failed to write screenshot '" << shotPath
              << "'.\n";
    return 1;
  }

  std::cout << "[headless] sim time "
            << (mCurrentWorld ? mCurrentWorld->getTime() : 0.0) << " s; wrote "
            << shotPath << "\n";
  if (stopRequested())
    return stopRun();
  return sceneFailed ? 1 : 0;
}

//==============================================================================
int App::run(std::optional<int> width, std::optional<int> height)
{
  ensureViewerConfigured();

  mGuiScaleTheme.reset();
  try {
    if (!width && !height)
      mViewer->setUpViewInWindow();
    else
      mViewer->setUpViewInWindow(
          0,
          0,
          width.value_or(kDefaultWindowWidth),
          height.value_or(kDefaultWindowHeight));
  } catch (const std::exception& error) {
    std::cerr << "Failed to create the demos window: " << error.what() << "\n";
    return 1;
  }

  mViewer->setWindowTitle("DART demos");
  initializeGuiTheme();
  if (mAutomaticGuiScale)
    mGuiScale = mViewer->getGuiScale();
  mNextDpiCheck
      = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);

  const std::string initial = !mInitialSceneId.empty() ? mInitialSceneId
                              : !mScenes.empty()       ? mScenes.front().id
                                                       : std::string();
  if (!initial.empty()) {
    requestSceneSwitch(initial);
    processPendingSwitch();
  }

  startSimulation();

  while (!mViewer->done()) {
    if (stopRequested())
      return stopRun();
    const auto now = std::chrono::steady_clock::now();
    if (mAutomaticGuiScale && now >= mNextDpiCheck) {
      updateAutomaticGuiScale();
      mNextDpiCheck = now + std::chrono::milliseconds(250);
    }
    // Panel callbacks and scene switches are queued while ImGui draws and
    // only ever executed here, between frames.
    processPanelEvents();
    processPendingSwitch();
    mViewer->frame();
  }

  return 0;
}

//==============================================================================
void App::startSimulation()
{
  // Preserve the interactive start request while the scene disallows Play.
  if (mRestoreSimulationOnTeardown)
    mResumeSimulationOnTeardown = true;
  mViewer->simulate(true);
}

//==============================================================================
void App::setGuiScale(double scale)
{
  const double sanitized = dart_examples::sanitizeGuiScale(scale);
  if (std::abs(mGuiScale - sanitized) <= 1e-6)
    return;
  mGuiScale = sanitized;
  mViewer->setGuiScale(mGuiScale);
  mDockLayoutResetRequested = true;
}

//==============================================================================
void App::updateAutomaticGuiScale()
{
  const double detected = mViewer->getGuiScale();
  if (std::abs(mGuiScale - detected) > 1e-6) {
    mGuiScale = detected;
    mDockLayoutResetRequested = true;
  }
}

//==============================================================================
void App::invokeKeyAction(KeyAction& action)
{
  invokeSceneCallback(action.callback, "action '" + action.label + "'");
}

//==============================================================================
void App::processPanelEvents()
{
  if (mPendingKeyAction) {
    const std::size_t index = *mPendingKeyAction;
    mPendingKeyAction.reset();
    if (index < mCurrentKeyActions.size())
      invokeKeyAction(mCurrentKeyActions[index]);
  }

  if (mCurrentPanel) {
    mCallbackFailures += mCurrentPanel->dispatch(
        [this](Control& control, const std::string& reason) {
          reportFailure(
              "Demo '" + mCurrentSceneTitle + "' control '" + control.getLabel()
              + "' failed: " + reason + " (disabled)");
        });
  }
}

//==============================================================================
bool App::handleKey(int key)
{
  for (auto& action : mCurrentKeyActions) {
    if (action.key == key) {
      invokeKeyAction(action);
      return true;
    }
  }

  // Case-insensitive fallback: try the opposite-case alphabetic key so a scene
  // that binds 'q' still responds to a Shift-held 'Q' (and vice versa). Only
  // ASCII letters are folded; special keys (arrows, Tab, ...) carry large
  // GUIEventAdapter KeySymbol codes and are left untouched.
  if (key >= 0 && key <= 255 && std::isalpha(static_cast<unsigned char>(key))) {
    const int lower = std::tolower(static_cast<unsigned char>(key));
    const int alt = (key == lower)
                        ? std::toupper(static_cast<unsigned char>(key))
                        : lower;
    for (auto& action : mCurrentKeyActions) {
      if (action.key == alt) {
        invokeKeyAction(action);
        return true;
      }
    }
  }
  return false;
}

//==============================================================================
void App::renderPanels()
{
  if (mAutomaticGuiScale)
    updateAutomaticGuiScale();
  const ImGuiIO& io = ImGui::GetIO();
  const float scale = static_cast<float>(guiLayoutScale(mGuiScale));
  const float screenW = std::max(1.0f, io.DisplaySize.x);
  const float screenH = std::max(1.0f, io.DisplaySize.y);

  // The visible title tracks the scene, but the "###" suffix keeps the ImGui
  // window identity stable so the panel keeps its dock slot
  // across scene switches instead of spawning a fresh window per scene.
  const std::string panelTitle
      = (mCurrentSceneTitle.empty() ? std::string("Scene") : mCurrentSceneTitle)
        + "###ScenePanel";

  // Real docking workspace (bundled docking-branch ImGui): a dockspace over
  // the viewport with a transparent central node for the 3D view. The default
  // layout supplies the initial workspace; every panel can be dragged,
  // split, and re-tabbed, and View > "Reset layout" restores this
  // arrangement. The layout is rebuilt deterministically on startup so a
  // stale imgui.ini can never hide the chrome.
  const ImGuiID dockspaceId = ImGui::GetID("DartDemosDockSpace");
  if (!mDockLayoutInitialized || mDockLayoutResetRequested) {
    mDockLayoutInitialized = true;
    mDockLayoutResetRequested = false;

    // Estimate content-sized regions in scaled pixels. The toolbar lays out
    // as one wide row (~1400 px of controls at scale 1) that wraps on narrow
    // windows, so budget one extra row per wrap.
    const float toolbarRowH = 40.0f * scale;
    const float tabBarH = 26.0f * scale;
    const float toolbarRows
        = std::ceil((1400.0f * scale) / std::max(320.0f, screenW));
    // The cap trades toolbar space for viewport space when an extreme scale
    // meets a small window (e.g. scale 4 in the default 1600x1000 wants ~74%
    // of the height); past the cap the docked toolbar window scrolls
    // vertically instead of losing controls, and the user can still drag the
    // splitter or enlarge the window.
    const float topFraction = std::clamp(
        (tabBarH + toolbarRows * toolbarRowH) / screenH, 0.05f, 0.45f);
    const float bottomFraction
        = std::clamp((200.0f * scale) / screenH, 0.10f, 0.40f);
    const float leftFraction
        = std::clamp((260.0f * scale) / screenW, 0.12f, 0.40f);
    const float rightFraction
        = std::clamp((360.0f * scale) / screenW, 0.15f, 0.45f);

    ImGui::DockBuilderRemoveNode(dockspaceId);
    // ImGuiDockNodeFlags_DockSpace is a private flag while
    // PassthruCentralNode is public; OR them as ints to avoid C++20's
    // deprecated-enum-enum-conversion warning.
    ImGui::DockBuilderAddNode(
        dockspaceId,
        static_cast<int>(ImGuiDockNodeFlags_DockSpace)
            | static_cast<int>(ImGuiDockNodeFlags_PassthruCentralNode));
    ImGui::DockBuilderSetNodeSize(dockspaceId, io.DisplaySize);

    ImGuiID center = dockspaceId;
    ImGuiID top = 0;
    ImGuiID bottom = 0;
    ImGuiID left = 0;
    ImGuiID right = 0;
    top = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Up, topFraction, nullptr, &center);
    bottom = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Down, bottomFraction, nullptr, &center);
    left = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Left, leftFraction, nullptr, &center);
    right = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Right, rightFraction, nullptr, &center);

    ImGui::DockBuilderDockWindow("Simulation", top);
    ImGui::DockBuilderDockWindow("Demos", left);
    ImGui::DockBuilderDockWindow(panelTitle.c_str(), right);
    ImGui::DockBuilderDockWindow("Diagnostics", bottom);
    ImGui::DockBuilderFinish(dockspaceId);
  }
  ImGui::DockSpaceOverViewport(
      dockspaceId,
      ImGui::GetMainViewport(),
      ImGuiDockNodeFlags_PassthruCentralNode);

  constexpr ImGuiWindowFlags kDockedFlags = ImGuiWindowFlags_NoCollapse;

  if (ImGui::Begin("Simulation", nullptr, kDockedFlags))
    renderToolbar();
  ImGui::End();

  if (ImGui::Begin("Demos", nullptr, kDockedFlags))
    renderNavigator();
  ImGui::End();

  if (ImGui::Begin(panelTitle.c_str(), nullptr, kDockedFlags))
    renderScenePanel();
  ImGui::End();

  if (ImGui::Begin("Diagnostics", nullptr, kDockedFlags))
    renderDiagnostics();
  ImGui::End();
}

//==============================================================================
void App::renderToolbar()
{
  const bool hasScene = mWorldNode != nullptr;
  const bool simulating = mViewer->isSimulating();
  const float scale = static_cast<float>(guiLayoutScale(mGuiScale));

  ImGui::TextColored(ImVec4(0.42f, 0.70f, 1.0f, 1.0f), "DART");
  ImGui::SameLine();
  ImGui::TextDisabled("demos");
  if (!mCurrentSceneTitle.empty()) {
    sameLineIfEnoughRoom(ImGui::CalcTextSize(mCurrentSceneTitle.c_str()).x);
    ImGui::TextUnformatted(mCurrentSceneTitle.c_str());
  }

  sameLineIfEnoughRoom(calcButtonWidth(simulating ? "Pause" : "Play"));
  ImGui::BeginDisabled(!hasScene);
  ImGui::BeginDisabled(!mViewer->isAllowingSimulation());
  if (ImGui::Button(simulating ? "Pause" : "Play"))
    mViewer->simulate(!simulating);
  ImGui::EndDisabled();
  sameLineIfEnoughRoom(calcButtonWidth("Step"));
  if (ImGui::Button("Step") && mWorldNode)
    mWorldNode->stepOnce();
  sameLineIfEnoughRoom(calcButtonWidth("Rebuild"));
  if (ImGui::Button("Rebuild") && !mCurrentSceneId.empty())
    requestSceneSwitch(mCurrentSceneId);
  sameLineIfEnoughRoom(calcButtonWidth("Reset"));
  // Reset restores the scene's initial state by re-running its factory (a
  // fresh World), the same as Rebuild -- see the Rebuild/Reset note in
  // README or the phase-1 report for why world->reset() alone is not enough.
  if (ImGui::Button("Reset") && !mCurrentSceneId.empty())
    requestSceneSwitch(mCurrentSceneId);
  ImGui::EndDisabled();

  sameLineIfEnoughRoom(90.0f * scale);
  ImGui::Text("Time %.2f s", mCurrentWorld ? mCurrentWorld->getTime() : 0.0);
  sameLineIfEnoughRoom(72.0f * scale);
  ImGui::Text("FPS %.0f", static_cast<double>(ImGui::GetIO().Framerate));
  sameLineIfEnoughRoom(96.0f * scale);
  if (hasLiveRtfStats(mViewer.get(), mWorldNode.get()))
    ImGui::Text("Sim %.2fx", mWorldNode->getSmoothedRealTimeFactor());
  else
    ImGui::TextUnformatted("Sim --");
  sameLineIfEnoughRoom(100.0f * scale);
  ImGui::Text(
      "Steps/frame %zu",
      mWorldNode ? mWorldNode->getLastRefreshStepCount() : std::size_t{0});

  sameLineIfEnoughRoom(190.0f * scale);
  ImGui::TextUnformatted("Target");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(clampedItemWidth(116.0f * scale, 76.0f * scale));
  if (ImGui::SliderFloat(
          "##target_rtf",
          &mTargetRtf,
          0.1f,
          4.0f,
          "%.2fx",
          ImGuiSliderFlags_AlwaysClamp)
      && std::isfinite(mTargetRtf)) {
    mTargetRtf = std::clamp(mTargetRtf, 0.1f, 4.0f);
    if (mWorldNode)
      mWorldNode->setTargetRealTimeFactor(mTargetRtf);
  }
  sameLineIfEnoughRoom(90.0f * scale);
  if (ImGui::Checkbox("Gravity", &mGravityEnabled) && mCurrentWorld) {
    if (mGravityEnabled) {
      mCurrentWorld->setGravity(mSavedGravity);
    } else {
      mSavedGravity = mCurrentWorld->getGravity();
      mCurrentWorld->setGravity(Eigen::Vector3d::Zero());
    }
  }

  // Timestep control. Logarithmic so the 1e-5..1e-2 s range is usable; applied
  // directly to the active world (the same frame-loop thread ImGui renders on,
  // matching the Gravity/Target RTF writes above). NaN input is rejected by
  // keeping the previous value.
  sameLineIfEnoughRoom(170.0f * scale);
  ImGui::TextUnformatted("dt");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(clampedItemWidth(120.0f * scale, 86.0f * scale));
  if (ImGui::SliderFloat(
          "##timestep",
          &mTimeStep,
          1e-5f,
          1e-2f,
          "%.5f s",
          ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic)
      && std::isfinite(mTimeStep)) {
    mTimeStep = std::clamp(mTimeStep, 1e-5f, 1e-2f);
    if (mCurrentWorld)
      mCurrentWorld->setTimeStep(mTimeStep);
  }

  renderRuntimeControls();

  sameLineIfEnoughRoom(calcButtonWidth("View"));
  if (ImGui::Button("View"))
    ImGui::OpenPopup("##view_menu_popup");
  if (ImGui::BeginPopup("##view_menu_popup")) {
    renderViewMenu();
    ImGui::EndPopup();
  }

  sameLineIfEnoughRoom(120.0f * scale);
  mDragForce.renderToolbarStatus(guiLayoutScale(mGuiScale));
}

//==============================================================================
void App::renderRuntimeControls()
{
  const float scale = static_cast<float>(guiLayoutScale(mGuiScale));

  sameLineIfEnoughRoom(250.0f * scale);
  ImGui::SetNextItemWidth(clampedItemWidth(170.0f * scale, 120.0f * scale));
  ImGui::BeginDisabled(!mCurrentWorld || mAvailableCollisionDetectors.empty());
  const char* currentDetector
      = mCollisionDetectorIndex >= 0
                && mCollisionDetectorIndex
                       < static_cast<int>(mAvailableCollisionDetectors.size())
            ? mAvailableCollisionDetectors[mCollisionDetectorIndex].c_str()
            : mCurrentCollisionDetectorName.c_str();
  if (ImGui::BeginCombo("Collision", currentDetector)) {
    for (std::size_t i = 0; i < mAvailableCollisionDetectors.size(); ++i) {
      const bool selected = static_cast<int>(i) == mCollisionDetectorIndex;
      if (ImGui::Selectable(mAvailableCollisionDetectors[i].c_str(), selected))
        setCollisionDetectorByName(mAvailableCollisionDetectors[i]);
      if (selected)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  ImGui::EndDisabled();

  sameLineIfEnoughRoom(128.0f * scale);
  ImGui::SetNextItemWidth(clampedItemWidth(90.0f * scale, 64.0f * scale));
  ImGui::BeginDisabled(!mCurrentWorld);
  int requestedThreads = mSimulationThreads;
  if (ImGui::InputInt("Threads", &requestedThreads, 1, 4)) {
    requestedThreads = std::clamp(requestedThreads, 0, 256);
    if (mCurrentWorld) {
      mCurrentWorld->setNumSimulationThreads(
          static_cast<std::size_t>(requestedThreads));
      mSimulationThreads
          = static_cast<int>(mCurrentWorld->getNumSimulationThreads());
      mPerformanceStatsPanel.reset();
    }
  }
  ImGui::EndDisabled();
}

//==============================================================================
void App::renderViewMenu()
{
  ImGui::BeginDisabled(!mCurrentWorld);
  if (ImGui::Button("Fit scene"))
    fitCameraToWorld();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Reset camera"))
    mViewer->home();

  ImGui::Separator();
  if (ImGui::Checkbox("Shadows", &mShadowsEnabled))
    applyShadowState();

  bool gridDisplayed = mGridVisual && mGridVisual->isDisplayed();
  if (ImGui::Checkbox("Grid", &gridDisplayed) && mGridVisual)
    mGridVisual->display(gridDisplayed);

  if (mGridVisual) {
    static const char* const kPlaneNames[] = {"XY", "YZ", "ZX"};
    int planeIndex = static_cast<int>(mGridVisual->getPlaneType());
    ImGui::SetNextItemWidth(
        120.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
    if (ImGui::Combo(
            "Plane", &planeIndex, kPlaneNames, IM_ARRAYSIZE(kPlaneNames))) {
      mGridVisual->setPlaneType(
          static_cast<dart::gui::GridVisual::PlaneType>(planeIndex));
    }

    float cellSize = static_cast<float>(mGridVisual->getMinorLineStepSize());
    ImGui::SetNextItemWidth(
        120.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
    if (ImGui::SliderFloat(
            "Cell size",
            &cellSize,
            0.05f,
            5.0f,
            "%.2f m",
            ImGuiSliderFlags_AlwaysClamp)
        && std::isfinite(cellSize)) {
      mGridVisual->setMinorLineStepSize(std::clamp(cellSize, 0.05f, 5.0f));
    }

    int numCells = static_cast<int>(mGridVisual->getNumCells());
    ImGui::SetNextItemWidth(
        120.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
    if (ImGui::SliderInt(
            "Num cells", &numCells, 2, 100, "%d", ImGuiSliderFlags_AlwaysClamp)
        && numCells > 0) {
      mGridVisual->setNumCells(static_cast<std::size_t>(numCells));
    }
  }

  ImGui::Separator();
  bool headlights = mViewer->checkHeadlights();
  if (ImGui::Checkbox("Headlights", &headlights))
    mViewer->switchHeadlights(headlights);

  ImGui::Separator();
  if (ImGui::Checkbox("Automatic DPI scaling", &mAutomaticGuiScale)) {
    mViewer->setAutomaticGuiScale(mAutomaticGuiScale);
    if (mAutomaticGuiScale)
      updateAutomaticGuiScale();
  }
  float guiScale = static_cast<float>(mGuiScale);
  const float minGuiScale = static_cast<float>(0.5);
  const float maxGuiScale = static_cast<float>(4.0);
  ImGui::SetNextItemWidth(
      160.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
  if (ImGui::SliderFloat(
          "GUI scale",
          &guiScale,
          minGuiScale,
          maxGuiScale,
          "%.2fx",
          ImGuiSliderFlags_AlwaysClamp)
      && std::isfinite(guiScale)) {
    mAutomaticGuiScale = false;
    setGuiScale(guiScale);
  }

  ImGui::Separator();
  if (ImGui::Button("Reset layout"))
    mDockLayoutResetRequested = true;
  ImGui::SetItemTooltip("Restore the default docked workspace layout.");
}

//==============================================================================
void App::renderNavigator()
{
  ImGui::TextWrapped("%s", mStatusLine.c_str());
  ImGui::Separator();

  const float clearW = calcButtonWidth("Clear");
  const float searchW = std::max(
      ImGui::GetFontSize() * 5.0f,
      ImGui::GetContentRegionAvail().x - clearW
          - ImGui::GetStyle().ItemSpacing.x);
  ImGui::SetNextItemWidth(searchW);
  ImGui::InputTextWithHint(
      "##search", "Search demos...", mSearchBuf, sizeof(mSearchBuf));
  sameLineIfEnoughRoom(clearW);
  if (ImGui::Button("Clear"))
    mSearchBuf[0] = '\0';

  const std::string filterLower = toLower(mSearchBuf);

  std::size_t shown = 0;
  for (const auto& scene : mScenes) {
    if (matchesFilter(scene, filterLower))
      ++shown;
  }
  ImGui::Text("Showing %zu/%zu", shown, mScenes.size());
  ImGui::Separator();

  ImGui::BeginChild("##navigator_scroll", ImVec2(0.0f, 0.0f), false);
  for (const auto& category : mCategories) {
    bool anyVisible = false;
    for (const auto index : category.sceneIndices) {
      if (matchesFilter(mScenes[index], filterLower)) {
        anyVisible = true;
        break;
      }
    }
    if (!anyVisible)
      continue;

    if (ImGui::CollapsingHeader(
            category.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      for (const auto index : category.sceneIndices) {
        const Scene& scene = mScenes[index];
        if (!matchesFilter(scene, filterLower))
          continue;

        std::string label = scene.title;
        if (mPendingSceneId && *mPendingSceneId == scene.id)
          label += " (starting)";

        const bool selected = (scene.id == mCurrentSceneId);
        if (ImGui::Selectable(label.c_str(), selected))
          requestSceneSwitch(scene.id);
        if (ImGui::IsItemHovered() && !scene.summary.empty())
          ImGui::SetTooltip("%s", scene.summary.c_str());
      }
    }
  }
  ImGui::EndChild();
}

//==============================================================================
void App::renderInspectorSection()
{
  if (ImGui::CollapsingHeader("Scene Tree", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginChild(
        "##scene_tree_scroll",
        ImVec2(
            0.0f,
            std::min(
                180.0f * static_cast<float>(guiLayoutScale(mGuiScale)),
                ImGui::GetContentRegionAvail().y * 0.45f)),
        true,
        ImGuiWindowFlags_HorizontalScrollbar);
    mInspector.renderTree(mCurrentWorld);
    ImGui::EndChild();
  }

  if (ImGui::CollapsingHeader("Inspector", ImGuiTreeNodeFlags_DefaultOpen)) {
    const bool paused = !mViewer->isSimulating();
    mInspector.renderDetail(paused, getWorldNode());

    if (auto* selected = mInspector.getSelection()) {
      if (!mDragForce.isGizmoAttached()) {
        if (ImGui::Button("Attach gizmo"))
          mDragForce.attachGizmo(selected, mCurrentWorld, mViewer.get());
      } else if (ImGui::Button("Detach gizmo")) {
        mDragForce.detachGizmo(mViewer.get());
      }
    }
  }
}

//==============================================================================
void App::renderSceneControlsSection()
{
  if (mCurrentPanel)
    mCurrentPanel->render();

  if (mCurrentRenderPanel) {
    try {
      mCurrentRenderPanel();
    } catch (const std::exception& e) {
      log(LogEntry::Level::Error,
          "Scene panel for '" + mCurrentSceneTitle + "' failed: " + e.what()
              + " (panel disabled)");
      mCurrentRenderPanel = nullptr;
      ++mCallbackFailures;
    } catch (...) {
      log(LogEntry::Level::Error,
          "Scene panel for '" + mCurrentSceneTitle
              + "' failed: unknown error (panel disabled)");
      mCurrentRenderPanel = nullptr;
      ++mCallbackFailures;
    }
  } else if (!mCurrentPanel) {
    ImGui::TextDisabled("This demo has no custom controls.");
  }

  if (!mCurrentKeyActions.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Actions");
    for (std::size_t i = 0; i < mCurrentKeyActions.size(); ++i) {
      const KeyAction& action = mCurrentKeyActions[i];
      const std::string label = formatKeyActionLabel(action) + "##key_action_"
                                + std::to_string(action.key);
      ImGui::BeginDisabled(!action.callback);
      // Run between frames, like panel callbacks.
      if (ImGui::Button(label.c_str()))
        mPendingKeyAction = i;
      ImGui::EndDisabled();
    }
  }
}

//==============================================================================
void App::renderToolsSection()
{
  if (ImGui::CollapsingHeader(
          "Contact Visualizer", ImGuiTreeNodeFlags_DefaultOpen)) {
    mContactVisualizer.renderToggle();
  }

  if (ImGui::CollapsingHeader("Drag Force", ImGuiTreeNodeFlags_DefaultOpen)) {
    mDragForce.renderTunables(guiLayoutScale(mGuiScale));
  }

  if (ImGui::CollapsingHeader("Profiler", ImGuiTreeNodeFlags_DefaultOpen)) {
    mProfiler.render();
  }
}

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
//==============================================================================
void App::renderMemorySection()
{
  dart::gui::app::renderMemoryDiagnostics(
      mMemoryDiagnosticsSession, ImGui::GetTime(), mGuiScale);
}
#endif

//==============================================================================
void App::renderScenePanel()
{
#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  bool memoryTabVisible = false;
#endif
  const auto tabSelectionFlags = [this](ScenePanelTab tab) {
    return mHasRequestedScenePanelTab && mRequestedScenePanelTab == tab
               ? ImGuiTabItemFlags_SetSelected
               : static_cast<ImGuiTabItemFlags>(0);
  };

  if (ImGui::BeginTabBar("##scene_panel_tabs")) {
    if (ImGui::BeginTabItem(
            "Scene", nullptr, tabSelectionFlags(ScenePanelTab::Scene))) {
      renderSceneControlsSection();
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem(
            "Inspector",
            nullptr,
            tabSelectionFlags(ScenePanelTab::Inspector))) {
      renderInspectorSection();
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem(
            "Tools", nullptr, tabSelectionFlags(ScenePanelTab::Tools))) {
      renderToolsSection();
      ImGui::EndTabItem();
    }

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
    if (ImGui::BeginTabItem(
            "Memory", nullptr, tabSelectionFlags(ScenePanelTab::Memory))) {
      memoryTabVisible = true;
      renderMemorySection();
      ImGui::EndTabItem();
    }
#endif
    ImGui::EndTabBar();
  }

#if DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  // Sampling belongs to the session, not tab visibility. The visible Memory
  // tab ticks after processing manual actions; hidden tabs tick here so
  // history continues at the configured cadence.
  if (!memoryTabVisible) {
    mMemoryDiagnosticsSession.update(ImGui::GetTime());
  }
#endif

  mHasRequestedScenePanelTab = false;
}

//==============================================================================
void App::renderDiagnostics()
{
  std::size_t numDofs = 0;
  const SceneCounts counts = collectSceneCounts(mCurrentWorld);
  if (mCurrentWorld) {
    for (std::size_t i = 0; i < mCurrentWorld->getNumSkeletons(); ++i) {
      const auto& skel = mCurrentWorld->getSkeleton(i);
      numDofs += skel->getNumDofs();
    }
  }

  const std::size_t numContacts
      = mCurrentWorld ? mCurrentWorld->getLastCollisionResult().getNumContacts()
                      : std::size_t{0};
  const std::size_t numConstraints
      = (mCurrentWorld && mCurrentWorld->getConstraintSolver())
            ? mCurrentWorld->getConstraintSolver()->getNumConstraints()
            : std::size_t{0};

  dart::gui::PerformanceStatsData stats;
  stats.frame = static_cast<int>(mWorldNode ? mWorldNode->getStepCount() : 0u);
  stats.simTime = mCurrentWorld ? mCurrentWorld->getTime() : 0.0;
  stats.sceneName = mCurrentSceneTitle;
  stats.collisionDetectorName = getWorldCollisionDetectorName(mCurrentWorld);
  stats.simulationThreads
      = mCurrentWorld ? mCurrentWorld->getNumSimulationThreads() : 1u;
  stats.renderFps = static_cast<double>(ImGui::GetIO().Framerate);
  stats.targetRealTimeFactor = mTargetRtf;
  stats.timeStep = mCurrentWorld ? mCurrentWorld->getTimeStep() : 0.0;
  stats.lastStepMs = mWorldNode ? mWorldNode->getLastStepMs() : 0.0;
  stats.movingAverageStepMs
      = mWorldNode ? mWorldNode->getMovingAverageStepMs() : 0.0;
  stats.minStepMs = mWorldNode ? mWorldNode->getMinStepMs() : 0.0;
  stats.maxStepMs = mWorldNode ? mWorldNode->getMaxStepMs() : 0.0;
  stats.measuredSteps = mWorldNode ? mWorldNode->getStepTimingSamples() : 0u;
  stats.contacts = numContacts;
  stats.softBodies = counts.softBodies;
  stats.pointMasses = counts.pointMasses;
  stats.skeletons = counts.skeletons;
  stats.bodyNodes = counts.bodyNodes;
  mPerformanceStatsPanel.render(stats);

  ImGui::Text(
      "Steps %zu   Sim %.2f s   Steps/frame %zu   dt %.5f s   Skeletons %zu   "
      "Bodies %zu   Soft %zu   Points %zu   DOFs %zu   Contacts %zu   "
      "Constraints %zu",
      mWorldNode ? mWorldNode->getStepCount() : std::size_t{0},
      mCurrentWorld ? mCurrentWorld->getTime() : 0.0,
      mWorldNode ? mWorldNode->getLastRefreshStepCount() : std::size_t{0},
      mCurrentWorld ? mCurrentWorld->getTimeStep() : 0.0,
      counts.skeletons,
      counts.bodyNodes,
      counts.softBodies,
      counts.pointMasses,
      numDofs,
      numContacts,
      numConstraints);
  if (hasLiveRtfStats(mViewer.get(), mWorldNode.get())) {
    ImGui::Text(
        "RTF min/avg/max %.2f / %.2f / %.2f",
        mWorldNode->getLowestRealTimeFactor(),
        mWorldNode->getSmoothedRealTimeFactor(),
        mWorldNode->getHighestRealTimeFactor());
  }

  ImGui::Separator();
  renderLogSection(0.0f);
}

//==============================================================================
void App::renderLogSection(float height)
{
  ImGui::Checkbox("Info", &mLogShowInfo);
  sameLineIfEnoughRoom(95.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
  ImGui::Checkbox("Warning", &mLogShowWarning);
  sameLineIfEnoughRoom(72.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
  ImGui::Checkbox("Error", &mLogShowError);
  sameLineIfEnoughRoom(100.0f * static_cast<float>(guiLayoutScale(mGuiScale)));
  ImGui::Checkbox("Autoscroll", &mLogAutoscroll);
  sameLineIfEnoughRoom(calcButtonWidth("Copy"));
  if (ImGui::Button("Copy")) {
    std::string joined;
    for (const auto& entry : mLog) {
      if ((entry.level == LogEntry::Level::Info && !mLogShowInfo)
          || (entry.level == LogEntry::Level::Warning && !mLogShowWarning)
          || (entry.level == LogEntry::Level::Error && !mLogShowError))
        continue;
      joined += entry.message;
      joined += '\n';
    }
    ImGui::SetClipboardText(joined.c_str());
  }

  ImGui::BeginChild("##log_scroll", ImVec2(0.0f, height), true);
  for (const auto& entry : mLog) {
    if ((entry.level == LogEntry::Level::Info && !mLogShowInfo)
        || (entry.level == LogEntry::Level::Warning && !mLogShowWarning)
        || (entry.level == LogEntry::Level::Error && !mLogShowError))
      continue;

    ImVec4 color = ImGui::GetStyle().Colors[ImGuiCol_Text];
    if (entry.level == LogEntry::Level::Error)
      color = ImVec4(0.95f, 0.35f, 0.35f, 1.0f);
    else if (entry.level == LogEntry::Level::Warning)
      color = ImVec4(0.95f, 0.75f, 0.25f, 1.0f);

    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextWrapped("%s", entry.message.c_str());
    ImGui::PopStyleColor();
  }
  if (mLogAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
    ImGui::SetScrollHereY(1.0f);
  ImGui::EndChild();
}

} // namespace dart::gui::app
