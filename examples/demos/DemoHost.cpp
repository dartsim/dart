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

#include "DemoHost.hpp"

#include "Theme.hpp"

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Viewport>
#include <osgGA/EventQueue>

#include <algorithm>
#include <iostream>

#include <cctype>
#include <cmath>

namespace dart_demos {

namespace {

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
bool matchesFilter(const DemoScene& scene, const std::string& filterLower)
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
/// the world directly via DemoWorldNode::stepOnce(), bypassing refresh()'s
/// wall-clock budget entirely, so mLowestRealTimeFactor is left at its
/// never-updated +infinity default (and the smoothed/highest fields would
/// otherwise misleadingly read 0.00 instead of "not measured").
bool hasLiveRtfStats(
    const dart::gui::osg::ImGuiViewer* viewer, const DemoWorldNode* worldNode)
{
  return worldNode && viewer->isSimulating()
         && std::isfinite(worldNode->getLowestRealTimeFactor());
}

//==============================================================================
/// Forwards ImGui rendering to the host; kept alive by ImGuiHandler's own
/// widget list once registered.
class HostPanelWidget : public dart::gui::osg::ImGuiWidget
{
public:
  explicit HostPanelWidget(DemoHost* host) : mHost(host) {}

  void render() override
  {
    mHost->renderPanels();
  }

private:
  DemoHost* mHost;
};

//==============================================================================
/// Persistent (never torn down) key handler that dispatches to the active
/// scene's key actions. Installed once at startup.
class DemoKeyHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit DemoKeyHandler(DemoHost* host) : mHost(host) {}

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN)
      return false;

    return mHost->handleKey(ea.getKey());
  }

private:
  DemoHost* mHost;
};

} // namespace

//==============================================================================
DemoHostContext::DemoHostContext(DemoHost& host) : mHost(host) {}

//==============================================================================
dart::gui::osg::ImGuiViewer* DemoHostContext::viewer() const
{
  return mHost.getViewer();
}

//==============================================================================
dart::gui::osg::WorldNode* DemoHostContext::worldNode() const
{
  return mHost.getWorldNode();
}

//==============================================================================
void DemoHostContext::addTeardown(std::function<void()> teardown)
{
  mHost.addTeardown(std::move(teardown));
}

//==============================================================================
void DemoHostContext::addAttachment(
    dart::gui::osg::ViewerAttachment* attachment)
{
  auto* viewer = mHost.getViewer();
  viewer->addAttachment(attachment);
  addTeardown([viewer, attachment] { viewer->removeAttachment(attachment); });
}

//==============================================================================
void DemoHostContext::addEventHandler(::osgGA::GUIEventHandler* handler)
{
  auto* viewer = mHost.getViewer();
  viewer->addEventHandler(handler);
  addTeardown([viewer, handler] { viewer->removeEventHandler(handler); });
}

//==============================================================================
void DemoHostContext::log(const std::string& message) const
{
  mHost.log(LogEntry::Level::Info, message);
}

//==============================================================================
DemoWorldNode::DemoWorldNode(const dart::simulation::WorldPtr& world)
  : dart::gui::osg::RealTimeWorldNode(world), mStepCount(0)
{
}

//==============================================================================
void DemoWorldNode::setHooks(
    std::function<void()> preStep, std::function<void()> postStep)
{
  mPreStep = std::move(preStep);
  mPostStep = std::move(postStep);
}

//==============================================================================
void DemoWorldNode::setHookErrorSink(
    std::function<void(const std::string&, const std::string&)> sink)
{
  mHookErrorSink = std::move(sink);
}

//==============================================================================
void DemoWorldNode::stepOnce()
{
  customPreStep();
  if (getWorld())
    getWorld()->step();
  customPostStep();
}

//==============================================================================
std::size_t DemoWorldNode::getStepCount() const
{
  return mStepCount;
}

//==============================================================================
void DemoWorldNode::customPreStep()
{
  invokeHook(mPreStep, "preStep");
}

//==============================================================================
void DemoWorldNode::customPostStep()
{
  invokeHook(mPostStep, "postStep");
  ++mStepCount;
}

//==============================================================================
void DemoWorldNode::invokeHook(std::function<void()>& hook, const char* name)
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
DemoHost::DemoHost(std::vector<DemoScene> scenes, double guiScale)
  : mScenes(std::move(scenes)),
    mGuiScale(dart::gui::osg::sanitizeGuiScale(guiScale))
{
  buildCategories();
  mViewer = new dart::gui::osg::ImGuiViewer();
}

//==============================================================================
void DemoHost::buildCategories()
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
const DemoScene* DemoHost::findScene(const std::string& id) const
{
  for (const auto& scene : mScenes) {
    if (scene.id == id)
      return &scene;
  }
  return nullptr;
}

//==============================================================================
void DemoHost::setInitialScene(const std::string& id)
{
  mInitialSceneId = id;
}

//==============================================================================
std::size_t DemoHost::getActiveWorldNodeCount() const
{
  return mActiveWorldNodeCount;
}

//==============================================================================
dart::gui::osg::ImGuiViewer* DemoHost::getViewer() const
{
  return mViewer.get();
}

//==============================================================================
DemoWorldNode* DemoHost::getWorldNode() const
{
  return mWorldNode.get();
}

//==============================================================================
void DemoHost::addTeardown(std::function<void()> teardown)
{
  mExtraTeardowns.push_back(std::move(teardown));
}

//==============================================================================
void DemoHost::log(LogEntry::Level level, const std::string& message)
{
  mLog.push_back(LogEntry{level, message});

  constexpr std::size_t kMaxLogEntries = 300;
  while (mLog.size() > kMaxLogEntries)
    mLog.pop_front();
}

//==============================================================================
void DemoHost::ensureViewerConfigured()
{
  if (mViewerConfigured)
    return;
  mViewerConfigured = true;

  mViewer->setThreadingModel(::osgViewer::ViewerBase::SingleThreaded);

  applyModernDarkColors();
  applyModernDarkMetrics();
  mViewer->getImGuiHandler()->setGuiScale(mGuiScale);

  mViewer->getCamera()->setClearColor(::osg::Vec4(0.10f, 0.11f, 0.13f, 1.0f));

  mViewer->getImGuiHandler()->addWidget(
      std::make_shared<HostPanelWidget>(this));
  mViewer->addEventHandler(new DemoKeyHandler(this));
}

//==============================================================================
void DemoHost::requestSceneSwitch(const std::string& id)
{
  mPendingSceneId = id;
}

//==============================================================================
void DemoHost::processPendingSwitch()
{
  if (!mPendingSceneId.has_value())
    return;

  const std::string requestedId = *mPendingSceneId;
  mPendingSceneId.reset();

  const DemoScene* scene = findScene(requestedId);
  if (!scene) {
    mLastSwitchFailed = true;
    log(LogEntry::Level::Error, "Unknown demo id '" + requestedId + "'");
    return;
  }

  mStatusLine = "Starting demo '" + scene->title + "'...";
  log(LogEntry::Level::Info, mStatusLine);

  DemoSceneSetup setup;
  bool ok = true;
  std::string failureReason;
  try {
    setup = scene->factory();
    if (!setup.world)
      throw std::runtime_error("factory returned a null world");
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
      installEmptyFallback();
    }
    log(LogEntry::Level::Error, mStatusLine);
    return;
  }

  mLastSwitchFailed = false;
  teardownCurrentScene();
  installScene(*scene, std::move(setup));
  mCurrentSceneId = scene->id;
  mCurrentSceneTitle = scene->title;
  mStatusLine = "Running demo '" + scene->title + "'";
  log(LogEntry::Level::Info, mStatusLine);
}

//==============================================================================
void DemoHost::teardownCurrentScene()
{
  // Tear down drag-and-drop / attachments / event handlers before the world
  // node and world itself, since these extras may hold raw pointers into
  // objects the world owns (e.g. a SimpleFrame's DnD registration).
  for (auto it = mExtraTeardowns.rbegin(); it != mExtraTeardowns.rend(); ++it) {
    try {
      (*it)();
    } catch (...) {
      // Teardown must never crash the host.
    }
  }
  mExtraTeardowns.clear();

  if (mWorldNode) {
    mViewer->removeWorldNode(mWorldNode.get());
    mWorldNode = nullptr;
    if (mActiveWorldNodeCount > 0)
      --mActiveWorldNodeCount;
  }

  mCurrentWorld.reset();
  mCurrentRenderPanel = nullptr;
  mCurrentKeyActions.clear();
  mCurrentCameraHome.reset();
  mCurrentSceneId.clear();
  mCurrentSceneTitle.clear();
}

//==============================================================================
void DemoHost::installScene(const DemoScene& scene, DemoSceneSetup setup)
{
  mCurrentWorld = setup.world;

  ::osg::ref_ptr<::osgShadow::ShadowTechnique> shadow;
  if (setup.enableShadows) {
    shadow = dart::gui::osg::WorldNode::createDefaultShadowTechnique(
        mViewer.get());
  }

  mWorldNode = new DemoWorldNode(mCurrentWorld);
  mWorldNode->setShadowTechnique(shadow);
  mWorldNode->setHooks(setup.preStep, setup.postStep);
  mWorldNode->setHookErrorSink(
      [this](const std::string& hookName, const std::string& reason) {
        log(LogEntry::Level::Error,
            "Demo '" + mCurrentSceneTitle + "' " + hookName
                + " failed: " + reason + " (disabled)");
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

  mViewer->addWorldNode(mWorldNode.get());
  ++mActiveWorldNodeCount;

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
    DemoHostContext ctx(*this);
    try {
      setup.onActivate(ctx);
    } catch (const std::exception& e) {
      log(LogEntry::Level::Error,
          "Demo '" + scene.title + "' onActivate failed: " + e.what());
    } catch (...) {
      log(LogEntry::Level::Error,
          "Demo '" + scene.title + "' onActivate failed: unknown error");
    }
  }

  const CameraHome defaultHome{
      ::osg::Vec3d(6.0, 8.0, 4.0),
      ::osg::Vec3d(0.0, 0.0, 1.0),
      ::osg::Vec3d(0.0, 0.0, 1.0)};
  applyCameraHome(
      mCurrentCameraHome.value_or(defaultHome),
      mViewer->getCameraManipulator() != nullptr);
}

//==============================================================================
void DemoHost::installEmptyFallback()
{
  DemoScene fallback;
  fallback.id.clear();
  fallback.title = "(no demo loaded)";

  DemoSceneSetup setup;
  setup.world = dart::simulation::World::create();
  setup.enableShadows = false;

  installScene(fallback, std::move(setup));
  mCurrentSceneId.clear();
  mCurrentSceneTitle = fallback.title;
}

//==============================================================================
void DemoHost::applyCameraHome(const CameraHome& home, bool viaManipulator)
{
  if (viaManipulator && mViewer->getCameraManipulator()) {
    mViewer->getCameraManipulator()->setHomePosition(
        home.eye, home.center, home.up);
    mViewer->setCameraManipulator(mViewer->getCameraManipulator());
  } else {
    mViewer->getCamera()->setViewMatrixAsLookAt(home.eye, home.center, home.up);
  }
}

//==============================================================================
int DemoHost::listScenes() const
{
  for (const auto& category : mCategories) {
    std::cout << category.name << "\n";
    for (const auto index : category.sceneIndices) {
      const DemoScene& scene = mScenes[index];
      std::cout << "  " << scene.id << "  -  " << scene.title;
      if (!scene.summary.empty())
        std::cout << ": " << scene.summary;
      std::cout << "\n";
    }
  }
  return 0;
}

//==============================================================================
int DemoHost::cycleScenes(int framesPerScene)
{
  bool anyFailure = false;

  // Loop the whole catalog twice in-process: this is the rapid re-switch
  // robustness gate from BRIEF-phase1.md #5 -- it must not leak world nodes
  // or corrupt state across repeated switches.
  for (int repeat = 0; repeat < 2; ++repeat) {
    for (const auto& scene : mScenes) {
      requestSceneSwitch(scene.id);
      processPendingSwitch();

      if (mLastSwitchFailed) {
        std::cerr << "[cycle-scenes] demo '" << scene.id
                  << "' failed to start.\n";
        anyFailure = true;
      }

      for (int frame = 0; frame < framesPerScene && mWorldNode; ++frame)
        mWorldNode->stepOnce();

      if (mActiveWorldNodeCount != 1) {
        std::cerr << "[cycle-scenes] world-node leak detected after '"
                  << scene.id << "' (active=" << mActiveWorldNodeCount << ")\n";
        anyFailure = true;
      }
    }
  }

  std::cout << "[cycle-scenes] cycled " << mScenes.size() << " demo(s) x2, "
            << framesPerScene << " frame(s) each. "
            << (anyFailure ? "FAILED" : "OK") << "\n";
  return anyFailure ? 1 : 0;
}

//==============================================================================
bool DemoHost::prepareOffscreenContext(
    int width, int height, const CameraHome& home)
{
  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  ::osg::ref_ptr<::osg::GraphicsContext> gc
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!gc) {
    std::cerr << "[headless] Failed to create an off-screen GL context (no "
                 "usable DISPLAY?).\n";
    return false;
  }

  auto* camera = mViewer->getCamera();
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new ::osg::Viewport(0, 0, width, height));
  camera->setProjectionMatrixAsPerspective(
      30.0, static_cast<double>(width) / height, 0.1, 1000.0);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  // Single-threaded so the screen-capture (a camera final-draw callback)
  // completes synchronously inside frame(); see EVIDENCE-gui-capabilities.md.
  mViewer->setThreadingModel(::osgViewer::ViewerBase::SingleThreaded);
  mViewer->setCameraManipulator(nullptr);
  mViewer->simulate(false);
  camera->setViewMatrixAsLookAt(home.eye, home.center, home.up);

  mViewer->realize();
  if (!mViewer->isRealized()) {
    std::cerr << "[headless] Viewer failed to realize off-screen.\n";
    return false;
  }

  if (auto* queue = mViewer->getEventQueue()) {
    queue->windowResize(0, 0, width, height);
    queue->setMouseInputRange(0.0f, 0.0f, width, height);
  }

  return true;
}

//==============================================================================
int DemoHost::runHeadlessShot(
    const std::string& shotPath,
    int steps,
    const std::string& sceneId,
    int width,
    int height)
{
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
  const bool sceneFailed = mLastSwitchFailed;
  if (sceneFailed) {
    std::cerr << "[headless] demo '" << initial
              << "' failed to start; capturing the fallback state.\n";
  }

  const CameraHome defaultHome{
      ::osg::Vec3d(6.0, 8.0, 4.0),
      ::osg::Vec3d(0.0, 0.0, 1.0),
      ::osg::Vec3d(0.0, 0.0, 1.0)};
  const CameraHome home = mCurrentCameraHome.value_or(defaultHome);

  if (!prepareOffscreenContext(width, height, home))
    return 1;

  for (int i = 0; i < steps && mWorldNode; ++i)
    mWorldNode->stepOnce();

  // Re-pin the view (realize() may have reset it) and draw, then capture.
  mViewer->getCamera()->setViewMatrixAsLookAt(home.eye, home.center, home.up);
  mViewer->frame();
  mViewer->frame();
  mViewer->captureScreen(shotPath);
  mViewer->frame(); // SaveScreen writes the PNG during this frame.

  std::cout << "[headless] sim time "
            << (mCurrentWorld ? mCurrentWorld->getTime() : 0.0) << " s; wrote "
            << shotPath << "\n";
  return sceneFailed ? 1 : 0;
}

//==============================================================================
int DemoHost::run()
{
  ensureViewerConfigured();

  mViewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(1280, mGuiScale),
      dart::gui::osg::scaleWindowExtent(800, mGuiScale));

  const std::string initial = !mInitialSceneId.empty() ? mInitialSceneId
                              : !mScenes.empty()       ? mScenes.front().id
                                                       : std::string();
  if (!initial.empty()) {
    requestSceneSwitch(initial);
    processPendingSwitch();
  }

  if (!mViewer->isRealized())
    mViewer->realize();

  mViewer->simulate(true);

  while (!mViewer->done()) {
    // Scene switches are queued (requestSceneSwitch) and only ever executed
    // here, between frames -- never from inside ImGui rendering.
    processPendingSwitch();
    mViewer->frame();
  }

  return 0;
}

//==============================================================================
void DemoHost::invokeKeyAction(KeyAction& action)
{
  if (!action.callback)
    return;

  try {
    action.callback();
  } catch (const std::exception& e) {
    log(LogEntry::Level::Error,
        "Action '" + action.label + "' failed: " + e.what() + " (disabled)");
    action.callback = nullptr;
  } catch (...) {
    log(LogEntry::Level::Error,
        "Action '" + action.label + "' failed: unknown error (disabled)");
    action.callback = nullptr;
  }
}

//==============================================================================
bool DemoHost::handleKey(int key)
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
  // osgGA KeySymbol codes and are left untouched.
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
void DemoHost::renderPanels()
{
  const ImGuiIO& io = ImGui::GetIO();
  const float scale = static_cast<float>(mGuiScale);
  const float screenW = std::max(1.0f, io.DisplaySize.x);
  const float screenH = std::max(1.0f, io.DisplaySize.y);

  const float toolbarH = std::min(96.0f * scale, screenH * 0.3f);
  const float bottomH = std::min(190.0f * scale, screenH * 0.35f);
  const float leftW = std::min(300.0f * scale, screenW * 0.4f);
  const float rightW = std::min(340.0f * scale, screenW * 0.4f);
  const float middleH = std::max(1.0f, screenH - toolbarH - bottomH);

  constexpr ImGuiWindowFlags kChromeFlags
      = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;

  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(screenW, toolbarH), ImGuiCond_Always);
  if (ImGui::Begin("Simulation", nullptr, kChromeFlags))
    renderToolbar();
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(0.0f, toolbarH), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(leftW, middleH), ImGuiCond_Always);
  if (ImGui::Begin("Demos", nullptr, kChromeFlags))
    renderNavigator();
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(screenW - rightW, toolbarH), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(rightW, middleH), ImGuiCond_Always);
  const std::string panelTitle
      = mCurrentSceneTitle.empty() ? "Scene" : mCurrentSceneTitle;
  if (ImGui::Begin(panelTitle.c_str(), nullptr, kChromeFlags))
    renderScenePanel();
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(0.0f, screenH - bottomH), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(screenW, bottomH), ImGuiCond_Always);
  if (ImGui::Begin("Diagnostics", nullptr, kChromeFlags))
    renderDiagnostics();
  ImGui::End();
}

//==============================================================================
void DemoHost::renderToolbar()
{
  const bool hasScene = mWorldNode != nullptr;
  const bool simulating = mViewer->isSimulating();

  ImGui::BeginDisabled(!hasScene);
  if (ImGui::Button(simulating ? "Pause" : "Play"))
    mViewer->simulate(!simulating);
  ImGui::SameLine();
  if (ImGui::Button("Step") && mWorldNode)
    mWorldNode->stepOnce();
  ImGui::SameLine();
  if (ImGui::Button("Rebuild") && !mCurrentSceneId.empty())
    requestSceneSwitch(mCurrentSceneId);
  ImGui::SameLine();
  // Reset restores the scene's initial state by re-running its factory (a
  // fresh World), the same as Rebuild -- see the Rebuild/Reset note in
  // README or the phase-1 report for why world->reset() alone is not enough.
  if (ImGui::Button("Reset") && !mCurrentSceneId.empty())
    requestSceneSwitch(mCurrentSceneId);
  ImGui::EndDisabled();

  ImGui::SameLine();
  ImGui::Text("Time %.2f s", mCurrentWorld ? mCurrentWorld->getTime() : 0.0);
  ImGui::SameLine();
  if (hasLiveRtfStats(mViewer.get(), mWorldNode.get()))
    ImGui::Text("RTF %.2f", mWorldNode->getSmoothedRealTimeFactor());
  else
    ImGui::TextUnformatted("RTF --");

  ImGui::SetNextItemWidth(220.0f * static_cast<float>(mGuiScale));
  if (ImGui::SliderFloat(
          "Target RTF",
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
  ImGui::SameLine();
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
  ImGui::SameLine();
  ImGui::SetNextItemWidth(200.0f * static_cast<float>(mGuiScale));
  if (ImGui::SliderFloat(
          "Timestep",
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
}

//==============================================================================
void DemoHost::renderNavigator()
{
  ImGui::TextWrapped("%s", mStatusLine.c_str());
  ImGui::Separator();

  ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60.0f);
  ImGui::InputTextWithHint(
      "##search", "Search demos...", mSearchBuf, sizeof(mSearchBuf));
  ImGui::SameLine();
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
        const DemoScene& scene = mScenes[index];
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
void DemoHost::renderScenePanel()
{
  if (mCurrentRenderPanel) {
    try {
      mCurrentRenderPanel();
    } catch (const std::exception& e) {
      log(LogEntry::Level::Error,
          "Scene panel for '" + mCurrentSceneTitle + "' failed: " + e.what()
              + " (panel disabled)");
      mCurrentRenderPanel = nullptr;
    } catch (...) {
      log(LogEntry::Level::Error,
          "Scene panel for '" + mCurrentSceneTitle
              + "' failed: unknown error (panel disabled)");
      mCurrentRenderPanel = nullptr;
    }
  } else {
    ImGui::TextDisabled("This demo has no custom controls.");
  }

  if (!mCurrentKeyActions.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Actions");
    for (auto& action : mCurrentKeyActions) {
      const std::string label = formatKeyActionLabel(action) + "##key_action_"
                                + std::to_string(action.key);
      ImGui::BeginDisabled(!action.callback);
      if (ImGui::Button(label.c_str()))
        invokeKeyAction(action);
      ImGui::EndDisabled();
    }
  }
}

//==============================================================================
void DemoHost::renderDiagnostics()
{
  std::size_t numSkeletons = 0;
  std::size_t numBodies = 0;
  std::size_t numDofs = 0;
  if (mCurrentWorld) {
    numSkeletons = mCurrentWorld->getNumSkeletons();
    for (std::size_t i = 0; i < numSkeletons; ++i) {
      const auto& skel = mCurrentWorld->getSkeleton(i);
      numBodies += skel->getNumBodyNodes();
      numDofs += skel->getNumDofs();
    }
  }

  if (hasLiveRtfStats(mViewer.get(), mWorldNode.get())) {
    ImGui::Text(
        "FPS %.0f   RTF min/avg/max %.2f / %.2f / %.2f",
        static_cast<double>(ImGui::GetIO().Framerate),
        mWorldNode->getLowestRealTimeFactor(),
        mWorldNode->getSmoothedRealTimeFactor(),
        mWorldNode->getHighestRealTimeFactor());
  } else {
    ImGui::Text(
        "FPS %.0f   RTF min/avg/max -- / -- / --",
        static_cast<double>(ImGui::GetIO().Framerate));
  }
  ImGui::Text(
      "Steps %zu   Sim time %.2f s   Skeletons %zu   Bodies %zu   DOFs %zu",
      mWorldNode ? mWorldNode->getStepCount() : std::size_t{0},
      mCurrentWorld ? mCurrentWorld->getTime() : 0.0,
      numSkeletons,
      numBodies,
      numDofs);

  ImGui::Separator();
  if (ImGui::CollapsingHeader("Log", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginChild(
        "##log_scroll",
        ImVec2(0.0f, 100.0f * static_cast<float>(mGuiScale)),
        true);
    for (const auto& entry : mLog) {
      ImVec4 color = ImGui::GetStyle().Colors[ImGuiCol_Text];
      if (entry.level == LogEntry::Level::Error)
        color = ImVec4(0.95f, 0.35f, 0.35f, 1.0f);
      else if (entry.level == LogEntry::Level::Warning)
        color = ImVec4(0.95f, 0.75f, 0.25f, 1.0f);

      ImGui::PushStyleColor(ImGuiCol_Text, color);
      ImGui::TextWrapped("%s", entry.message.c_str());
      ImGui::PopStyleColor();
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
      ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
  }
}

} // namespace dart_demos
