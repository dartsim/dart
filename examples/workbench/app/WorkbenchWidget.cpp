#include "WorkbenchWidget.hpp"

#include <imgui.h>
#include <imgui_impl_opengl2.h>
#ifdef DART_IMGUI_DOCKING
#  include <imgui_internal.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <numeric>
#include <sstream>

#include <Eigen/Core>

#include "../core/WorkbenchConstants.hpp"
#include "../core/WorkbenchWorldNode.hpp"
#include "../registry/ExampleRegistry.hpp"

namespace workbench {

WorkbenchWidget::WorkbenchWidget(
    ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer,
    ::osg::ref_ptr<WorkbenchWorldNode> worldNode,
    std::filesystem::path examplesRoot,
    float preferredScale)
  : mViewer(std::move(viewer)),
    mWorldNode(std::move(worldNode)),
    mExamplesRoot(std::move(examplesRoot))
{
  mServices.widget = this;
  mServices.viewer = mViewer.get();
  mServices.worldNode = mWorldNode.get();
  mWorldNode->setServicesPtr(&mServices);

  if (!(preferredScale > 0.0f && std::isfinite(preferredScale)))
    preferredScale = detectUiScale();

  preferredScale = std::clamp(preferredScale, 0.75f, 3.0f);
  mDetectedUiScale = preferredScale;
  mUiScaleControl = preferredScale;
  applyUiScale(preferredScale);

  mExamples = loadExampleRecords(mExamplesRoot);
  rebuildCategoryView();
  ensureSelection();

  log("Workbench initialized.");
}

void WorkbenchWidget::render()
{
  WorkbenchLayout layout = computeLayout();
  updateMetrics();

#ifdef DART_IMGUI_DOCKING
  renderDockingRoot(layout);
#else
  drawTopBar(layout);
#endif
  drawExampleBrowser(layout);
  drawOverview(layout);
  drawProperties(layout);
  drawLog(layout);
}

void WorkbenchWidget::log(const std::string& message)
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
#if defined(_WIN32)
  localtime_s(&tm, &time);
#else
  localtime_r(&time, &tm);
#endif

  char buffer[16];
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm);

  std::ostringstream oss;
  oss << '[' << buffer << "] " << message;

  if (mLogLines.size() >= kLogCapacity)
    mLogLines.pop_front();

  mLogLines.emplace_back(oss.str());
  mScrollLogToBottom = true;
}

void WorkbenchWidget::reloadCurrentExample()
{
  if (mSelectedIndex >= mExamples.size()) {
    log("No example selected to reload.");
    return;
  }

  if (!mExamples[mSelectedIndex].hasFactory()) {
    log("Selected entry is standalone; nothing to reload.");
    return;
  }

  selectExample(mSelectedIndex, true);
}

dart::gui::osg::ImGuiViewer& WorkbenchWidget::viewer()
{
  return *mViewer;
}

WorkbenchWorldNode& WorkbenchWidget::worldNode()
{
  return *mWorldNode;
}

WorkbenchLayout WorkbenchWidget::computeLayout() const
{
  WorkbenchLayout layout;
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  layout.viewportPos = viewport->Pos;
  layout.viewportSize = viewport->Size;
  layout.padding = 6.0f;
  layout.topBarHeight = 32.0f;

  const float minLeft = 260.0f;
  const float minRight = 280.0f;
  const float minBottom = 220.0f;

  const float minViewerHeight = 360.0f;

  const float availableHeight = std::max(
      layout.viewportSize.y - layout.topBarHeight - layout.padding * 2.0f,
      minViewerHeight + minBottom + layout.padding);

  layout.leftWidth = std::max(minLeft, layout.viewportSize.x * 0.22f);
  layout.rightWidth = std::max(minRight, layout.viewportSize.x * 0.26f);
  layout.bottomHeight = std::max(minBottom, availableHeight * 0.28f);
  layout.centerWidth = std::max(
      320.0f,
      layout.viewportSize.x - layout.leftWidth - layout.rightWidth
          - layout.padding * 4.0f);
  layout.centerHeight = std::max(
      240.0f,
      availableHeight - layout.bottomHeight - layout.padding * 3.0f);
  layout.viewerHeight
      = std::max(minViewerHeight, availableHeight - layout.bottomHeight
          - layout.padding * 3.0f);

  layout.origin = ImVec2(
      layout.viewportPos.x + layout.padding,
      layout.viewportPos.y + layout.topBarHeight + layout.padding * 2.0f);

  return layout;
}

void WorkbenchWidget::updateMetrics()
{
  float value = static_cast<float>(mWorldNode->getLastRealTimeFactor());
  if (!std::isfinite(value))
    value = 0.0f;

  mRtfHistory.push_back(value);
  if (mRtfHistory.size() > kRtfCapacity)
    mRtfHistory.erase(mRtfHistory.begin());
}

void WorkbenchWidget::renderMenuBarContent(float windowWidth)
{
  if (!ImGui::BeginMenuBar())
    return;

  if (ImGui::BeginMenu("File")) {
    const bool canReload = (mSelectedIndex < mExamples.size())
        && mExamples[mSelectedIndex].hasFactory();
    if (ImGui::MenuItem("Reload Example", "Ctrl+R", false, canReload))
      reloadCurrentExample();
    if (ImGui::MenuItem("Quit"))
      viewer().setDone(true);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Simulation")) {
    bool running = viewer().isSimulating();
    if (ImGui::MenuItem(running ? "Pause" : "Play"))
      toggleSimulation();
    if (ImGui::MenuItem(
            "Reset Scene",
            nullptr,
            false,
            mSelectedIndex < mExamples.size()
                && mExamples[mSelectedIndex].hasFactory()))
      reloadCurrentExample();
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Help")) {
    if (ImGui::MenuItem("About DART"))
      viewer().showAbout();
    ImGui::EndMenu();
  }

  const float statsWidth = 220.0f;
  ImGui::SameLine(std::max(0.0f, windowWidth - statsWidth));
  ImGui::Text(
      "RTF %.3f | Time %.2fs",
      mWorldNode->getLastRealTimeFactor(),
      mWorldNode->getWorld() ? mWorldNode->getWorld()->getTime() : 0.0);

  ImGui::EndMenuBar();
}

void WorkbenchWidget::drawTopBar(const WorkbenchLayout& layout)
{
  ImGui::SetNextWindowPos(layout.viewportPos);
  ImGui::SetNextWindowSize(ImVec2(layout.viewportSize.x, layout.topBarHeight));
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoSavedSettings
      | ImGuiWindowFlags_NoBringToFrontOnFocus
      | ImGuiWindowFlags_MenuBar;

  if (ImGui::Begin("##WorkbenchTopBar", nullptr, flags))
    renderMenuBarContent(layout.viewportSize.x);
  ImGui::End();
}

#ifdef DART_IMGUI_DOCKING
void WorkbenchWidget::renderDockingRoot(const WorkbenchLayout& layout)
{
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);

  ImGuiWindowFlags windowFlags = ImGuiWindowFlags_MenuBar
      | ImGuiWindowFlags_NoDocking
      | ImGuiWindowFlags_NoTitleBar
      | ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoBringToFrontOnFocus
      | ImGuiWindowFlags_NoNavFocus
      | ImGuiWindowFlags_NoBackground;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

  if (ImGui::Begin("WorkbenchDockRoot", nullptr, windowFlags)) {
    renderMenuBarContent(ImGui::GetWindowWidth());

    const ImGuiID dockspaceId = ImGui::GetID("WorkbenchDockspace");
    ImGui::DockSpace(
        dockspaceId,
        ImVec2(0.0f, 0.0f),
        ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode);

    if (!mDockspaceInitialized) {
      const ImVec2 dockSize = ImGui::GetWindowSize();
      ImGui::DockBuilderRemoveNode(dockspaceId);
      ImGuiID dockMain = ImGui::DockBuilderAddNode(dockspaceId);
      ImGui::DockBuilderSetNodeSize(dockMain, dockSize);

      const float totalWidth = std::max(1.0f, dockSize.x);
      const float totalHeight = std::max(1.0f, dockSize.y);

      ImGuiID dockLeft = ImGui::DockBuilderSplitNode(
          dockMain,
          ImGuiDir_Left,
          std::clamp(layout.leftWidth / totalWidth, 0.1f, 0.4f),
          nullptr,
          &dockMain);
      ImGuiID dockRight = ImGui::DockBuilderSplitNode(
          dockMain,
          ImGuiDir_Right,
          std::clamp(layout.rightWidth / totalWidth, 0.1f, 0.5f),
          nullptr,
          &dockMain);
      ImGuiID dockBottom = ImGui::DockBuilderSplitNode(
          dockMain,
          ImGuiDir_Down,
          std::clamp(
              layout.bottomHeight / std::max(layout.bottomHeight + layout.viewerHeight, 1.0f),
              0.15f,
              0.6f),
          nullptr,
          &dockMain);

      ImGui::DockBuilderDockWindow("Example Browser", dockLeft);
      ImGui::DockBuilderDockWindow("Properties", dockRight);
      ImGui::DockBuilderDockWindow("Log & Metrics", dockBottom);
      ImGui::DockBuilderDockWindow("Overview", dockMain);
      ImGui::DockBuilderFinish(dockspaceId);
      mDockspaceInitialized = true;
    }
  }
  ImGui::End();
  ImGui::PopStyleVar(2);
}
#endif

void WorkbenchWidget::drawExampleBrowser(const WorkbenchLayout& layout)
{
#ifndef DART_IMGUI_DOCKING
  ImGui::SetNextWindowPos(layout.origin);
  ImGui::SetNextWindowSize(ImVec2(layout.leftWidth, layout.centerHeight));
#endif
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoResize;

  if (ImGui::Begin("Example Browser", nullptr, flags)) {
    if (ImGui::Button("Reload Scene"))
      reloadCurrentExample();

    ImGui::SameLine();
    if (ImGui::Button("Toggle Simulation"))
      toggleSimulation();

    ImGui::Separator();

    for (const auto& category : mCategoryOrder) {
      ImGui::Spacing();
      ImGui::TextUnformatted(category.c_str());
      ImGui::Separator();

      const auto& indices = mCategoryIndices[category];
      for (std::size_t index : indices) {
        const auto& record = mExamples[index];
        const bool selected = (index == mSelectedIndex);
        std::string label = record.name + "##" + record.id;
        if (ImGui::Selectable(label.c_str(), selected)) {
          const bool forceReload = (index == mSelectedIndex);
          selectExample(index, forceReload);
        }

        if (!record.description.empty()) {
          const float wrapWidth
              = ImGui::GetCursorPos().x + ImGui::GetContentRegionAvail().x - 24.0f;
          ImGui::PushTextWrapPos(wrapWidth);
          ImGui::TextDisabled("%s", record.description.c_str());
          ImGui::PopTextWrapPos();
        }
      }
    }
  }
  ImGui::End();
}

void WorkbenchWidget::drawOverview(const WorkbenchLayout& layout)
{
#ifndef DART_IMGUI_DOCKING
  ImGui::SetNextWindowPos(ImVec2(
      layout.origin.x + layout.leftWidth + layout.padding,
      layout.origin.y));
  ImGui::SetNextWindowSize(ImVec2(layout.centerWidth, layout.viewerHeight));
#endif
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoResize;

  if (ImGui::Begin("Overview", nullptr, flags)) {
    if (mSelectedIndex < mExamples.size()) {
      const auto& record = mExamples[mSelectedIndex];
      ImGui::TextUnformatted(record.name.c_str());
      ImGui::Separator();
      if (!record.description.empty())
        ImGui::TextWrapped("%s", record.description.c_str());
      else
        ImGui::TextDisabled("No description available.");
      ImGui::Spacing();
    }

    auto world = mWorldNode->getWorld();
    if (world) {
      ImGui::Text("World name: %s", world->getName().c_str());
      ImGui::Text("Simulation time: %.3f s", world->getTime());
      ImGui::Text("Time step: %.5f s", world->getTimeStep());
      ImGui::Text("Skeletons: %d", static_cast<int>(world->getNumSkeletons()));
      const auto g = world->getGravity();
      ImGui::Text(
          "Gravity: (%.2f, %.2f, %.2f)",
          g.x(), g.y(), g.z());
      ImGui::Spacing();
    } else {
      ImGui::TextDisabled("Select a Workbench scene to load it into the viewer.");
      if (mSelectedIndex < mExamples.size()
          && !mExamples[mSelectedIndex].hasFactory()
          && !mExamples[mSelectedIndex].directory.empty()) {
        ImGui::Spacing();
        ImGui::TextWrapped(
            "This entry launches as a standalone executable. Inspect it at: %s",
            mExamples[mSelectedIndex].directory.string().c_str());
      }
    }

    if (mActiveSession && mActiveSession->onOverviewUi) {
      ImGui::Separator();
      mActiveSession->onOverviewUi(mServices);
    }
  }
  ImGui::End();
}

void WorkbenchWidget::drawProperties(const WorkbenchLayout& layout)
{
#ifndef DART_IMGUI_DOCKING
  ImGui::SetNextWindowPos(ImVec2(
      layout.origin.x + layout.leftWidth + layout.padding + layout.centerWidth
          + layout.padding,
      layout.origin.y));
  ImGui::SetNextWindowSize(ImVec2(layout.rightWidth, layout.centerHeight));
#endif
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoResize;

  if (ImGui::Begin("Properties", nullptr, flags)) {
    bool running = viewer().isSimulating();
    if (ImGui::Checkbox("Simulate", &running))
      toggleSimulation();

    auto world = mWorldNode->getWorld();
    if (world) {
      double step = world->getTimeStep();
      float stepFloat = static_cast<float>(step);
      if (ImGui::DragFloat("Time step", &stepFloat, 0.00005f, 0.0001f, 0.02f, "%.5f"))
        world->setTimeStep(static_cast<double>(stepFloat));

      Eigen::Vector3d gravity = world->getGravity();
      float gravityZ = static_cast<float>(gravity.z());
      if (ImGui::SliderFloat("Gravity Z", &gravityZ, -15.0f, 0.0f)) {
        gravity.z() = gravityZ;
        world->setGravity(gravity);
      }

      ImGui::Spacing();
    }

    if (ImGui::SliderFloat("UI scale", &mUiScaleControl, 0.75f, 3.0f, "%.2fx")) {
      applyUiScale(mUiScaleControl);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset scale")) {
      mUiScaleControl = mDetectedUiScale;
      applyUiScale(mUiScaleControl);
    }

    ImGui::Spacing();

    if (mActiveSession && mActiveSession->onPropertiesUi) {
      ImGui::Separator();
      mActiveSession->onPropertiesUi(mServices);
    } else if (
        mSelectedIndex < mExamples.size()
        && mExamples[mSelectedIndex].hasFactory()) {
      ImGui::Separator();
      ImGui::TextDisabled(
          "Select a scene action on the left to begin interacting.");
    } else {
      ImGui::TextDisabled(
          "Standalone examples expose their controls directly.");
    }
  }
  ImGui::End();
}

void WorkbenchWidget::drawLog(const WorkbenchLayout& layout)
{
#ifndef DART_IMGUI_DOCKING
  ImGui::SetNextWindowPos(ImVec2(
      layout.origin.x,
      layout.origin.y + layout.centerHeight + layout.padding));
  ImGui::SetNextWindowSize(ImVec2(
      layout.leftWidth + layout.centerWidth + layout.rightWidth
          + layout.padding * 2.0f,
      layout.bottomHeight));
#endif
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoResize;

  if (ImGui::Begin("Log & Metrics", nullptr, flags)) {
    if (ImGui::Button("Clear"))
      mLogLines.clear();

    ImGui::SameLine();
    ImGui::Text("Entries: %zu", static_cast<std::size_t>(mLogLines.size()));

    if (!mRtfHistory.empty()) {
      ImGui::SameLine();
      const float sum = std::accumulate(
          mRtfHistory.begin(), mRtfHistory.end(), 0.0f);
      const float average = sum / static_cast<float>(mRtfHistory.size());
      ImGui::Text("RTF avg: %.3f", average);
      ImGui::PlotLines(
          "Real-time factor",
          mRtfHistory.data(),
          static_cast<int>(mRtfHistory.size()),
          0,
          nullptr,
          0.0f,
          2.0f,
          ImVec2(-1.0f, 80.0f));
    }

    ImGui::Separator();

    if (ImGui::BeginChild("LogScrollRegion")) {
      for (const auto& line : mLogLines)
        ImGui::TextUnformatted(line.c_str());

      if (mScrollLogToBottom) {
        ImGui::SetScrollHereY(1.0f);
        mScrollLogToBottom = false;
      }
    }
    ImGui::EndChild();
  }
  ImGui::End();
}

void WorkbenchWidget::selectExample(std::size_t index, bool forceReload)
{
  if (index >= mExamples.size())
    return;

  const bool sameSelection = (index == mSelectedIndex);
  mSelectedIndex = index;

  if (!mExamples[index].hasFactory()) {
    unloadExample();
    return;
  }

  const bool shouldReload = forceReload || !sameSelection;
  if (shouldReload)
    loadExample(index);
}

void WorkbenchWidget::loadExample(std::size_t index)
{
  if (index >= mExamples.size())
    return;

  const auto& record = mExamples[index];
  unloadExample();

  ExampleSession session = record.factory(mServices);
  if (!session.world) {
    log("Example \"" + record.name + "\" did not return a world.");
    return;
  }

  mActiveSession = std::make_shared<ExampleSession>(std::move(session));
  mWorldNode->setActiveSession(mActiveSession);
  mWorldNode->simulate(true);
  viewer().simulate(true);

  log("Loaded scene \"" + record.name + "\".");
  if (mActiveSession->onActivate)
    mActiveSession->onActivate(mServices);
}

void WorkbenchWidget::unloadExample()
{
  if (mActiveSession) {
    if (mActiveSession->onDeactivate)
      mActiveSession->onDeactivate(mServices);
    mActiveSession.reset();
  }

  mWorldNode->simulate(false);
  mWorldNode->clearSession();
  viewer().simulate(false);
}

void WorkbenchWidget::toggleSimulation()
{
  const bool running = viewer().isSimulating();
  viewer().simulate(!running);
  mWorldNode->simulate(!running);
  log(std::string("Simulation ") + (!running ? "resumed" : "paused") + ".");
}

void WorkbenchWidget::applyUiScale(float scale)
{
  if (!ImGui::GetCurrentContext())
    return;

  if (!(scale > 0.0f) || !std::isfinite(scale))
    return;

  scale = std::clamp(scale, 0.75f, 3.0f);
  const float epsilon = 0.001f;
  if (std::fabs(scale - mCurrentUiScale) < epsilon)
    return;

  ImGuiStyle& style = ImGui::GetStyle();
  const float ratio = scale / mCurrentUiScale;
  style.ScaleAllSizes(ratio);

  ImGuiIO& io = ImGui::GetIO();
  io.DisplayFramebufferScale = ImVec2(scale, scale);
  io.Fonts->Clear();
  ImFontConfig config;
  config.SizePixels = 16.0f * scale;
  io.Fonts->AddFontDefault(&config);
  io.Fonts->Build();
  ImGui_ImplOpenGL2_DestroyDeviceObjects();
  ImGui_ImplOpenGL2_CreateDeviceObjects();
  io.FontGlobalScale = 1.0f;

  mCurrentUiScale = scale;
#ifdef DART_IMGUI_DOCKING
  mDockspaceInitialized = false;
#endif
}

void WorkbenchWidget::rebuildCategoryView()
{
  mCategoryOrder.clear();
  mCategoryIndices.clear();

  for (std::size_t i = 0; i < mExamples.size(); ++i) {
    const std::string& category = mExamples[i].category;
    if (mCategoryIndices.find(category) == mCategoryIndices.end()) {
      mCategoryOrder.push_back(category);
      mCategoryIndices[category] = {};
    }
    mCategoryIndices[category].push_back(i);
  }
}

void WorkbenchWidget::ensureSelection()
{
  if (mExamples.empty()) {
    mSelectedIndex = std::numeric_limits<std::size_t>::max();
    return;
  }

  if (mSelectedIndex < mExamples.size())
    return;

  const auto it = std::find_if(
      mExamples.begin(),
      mExamples.end(),
      [](const ExampleRecord& record) { return record.hasFactory(); });

  if (it != mExamples.end()) {
    const std::size_t index
        = static_cast<std::size_t>(std::distance(mExamples.begin(), it));
    selectExample(index, true);
  } else {
    selectExample(0, true);
  }
}

} // namespace workbench
