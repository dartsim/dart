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

#include <dart/gui/application.hpp>
#include <dart/gui/detail/application.hpp>
#include <dart/gui/detail/application_runner.hpp>
#include <dart/gui/detail/application_teardown.hpp>
#include <dart/gui/detail/debug_overlay.hpp>
#include <dart/gui/detail/frame_renderer.hpp>
#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/detail/gui_scale.hpp>
#include <dart/gui/detail/imgui_overlay.hpp>
#include <dart/gui/detail/input.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/detail/offscreen_parity.hpp>
#include <dart/gui/detail/perf_hud.hpp>
#include <dart/gui/detail/render_context.hpp>
#include <dart/gui/detail/render_environment.hpp>
#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/detail/scene_frame.hpp>
#include <dart/gui/detail/scene_startup.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/detail/screenshot.hpp>
#include <dart/gui/detail/selection.hpp>
#include <dart/gui/detail/simulation_stepper.hpp>
#include <dart/gui/detail/ui_frame.hpp>
#include <dart/gui/profile.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/body_node.hpp>

#include <dart/common/profile.hpp>

#include <utils/Log.h>

#include <algorithm>
#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace {

using dart::gui::advanceRecordedFramePlayback;
using dart::gui::elapsedMs;
using dart::gui::OrbitCameraController;
using dart::gui::printProfile;
using dart::gui::ProfileAccumulator;
using dart::gui::requestDockLayoutReset;
using dart::gui::requestSceneReplay;
using dart::gui::requestSceneSwitch;
using dart::gui::requestSingleStep;
using dart::gui::RunOptions;
using dart::gui::setRecordedFramePlaybackIndex;
using dart::gui::stepRecordedFramePlayback;
using dart::gui::toggleFrameOutputCapture;
using dart::gui::togglePaused;
using dart::gui::toggleRecordedFramePlayback;
using dart::gui::ViewerLifecycleState;
using dart::gui::detail::ApplicationInputState;
using dart::gui::detail::ApplicationWindow;
using dart::gui::detail::applyRenderSettings;
using dart::gui::detail::AppOptions;
using dart::gui::detail::attachOrbitCameraController;
using dart::gui::detail::attachSceneEnvironment;
using dart::gui::detail::configureMainView;
using dart::gui::detail::createApplicationWindow;
using dart::gui::detail::createConfiguredImGuiOverlay;
using dart::gui::detail::createDartScene;
using dart::gui::detail::createDebugColorGrading;
using dart::gui::detail::createFilamentRenderContext;
using dart::gui::detail::createInitialSceneState;
using dart::gui::detail::createMaterialResources;
using dart::gui::detail::createNeutralIndirectLight;
using dart::gui::detail::createNeutralSkybox;
using dart::gui::detail::createSceneLights;
using dart::gui::detail::DartScene;
using dart::gui::detail::destroyApplicationResources;
using dart::gui::detail::destroyPersistentApplicationResources;
using dart::gui::detail::destroySceneRenderables;
using dart::gui::detail::ExampleScene;
using dart::gui::detail::FilamentRenderContext;
using dart::gui::detail::finalizeScreenshotCapture;
using dart::gui::detail::FrameRenderResult;
using dart::gui::detail::FrameViewport;
using dart::gui::detail::getCurrentImGuiIo;
using dart::gui::detail::getNativeWindow;
using dart::gui::detail::GuiScaleState;
using dart::gui::detail::highFidelityHeadlessRequested;
using dart::gui::detail::ImGuiOverlay;
using dart::gui::detail::initialCameraForScene;
using dart::gui::detail::InitialSceneState;
using dart::gui::detail::isSceneMouseInputCapturedByUi;
using dart::gui::detail::latestGpuFrameMs;
using dart::gui::detail::MaterialResources;
using dart::gui::detail::MaterialSet;
using dart::gui::detail::parseOptions;
using dart::gui::detail::PerfHudState;
using dart::gui::detail::pollApplicationInput;
using dart::gui::detail::Renderable;
using dart::gui::detail::renderApplicationFrame;
using dart::gui::detail::resizeAutomaticApplicationWindow;
using dart::gui::detail::resolveWindowDpiScale;
using dart::gui::detail::runOffscreenParitySelfCheck;
using dart::gui::detail::SceneFrameUpdater;
using dart::gui::detail::SceneLights;
using dart::gui::detail::ScreenshotCapture;
using dart::gui::detail::ScriptedForceDrag;
using dart::gui::detail::SelectionController;
using dart::gui::detail::shouldContinueApplicationLoop;
using dart::gui::detail::SimulationStepper;
using dart::gui::detail::updateConfiguredImGuiOverlayScale;
using dart::gui::detail::updateFrameUi;
using dart::gui::detail::updateFrameViewport;

// Default number of frames each scene renders in `--cycle-scenes` mode before
// the host advances. A few frames exercise scene build, extraction, render, and
// teardown without making the smoke test slow; `--frames N` overrides this per
// scene instead of acting as a global viewer stop.
constexpr int kDefaultCycleFramesPerScene = 4;

// User-requested demo switches are treated as candidate activations. If the
// candidate returns but still exceeds this startup budget, the host restores
// the previous demo instead of leaving the workspace on a slow/broken scene.
constexpr double kDemoSceneStartupTimeoutMs = 5000.0;

bool hasSceneOption(int argc, char* argv[]);

struct ScriptedDemoSwitch
{
  int afterFrames = 0;
  std::string sceneId;
  std::filesystem::path eventLogPath;
  bool requested = false;
  bool observed = false;
};

bool isTruthyEnvironmentVariable(const char* name)
{
  const char* value = std::getenv(name);
  if (value == nullptr) {
    return false;
  }

  const std::string_view text(value);
  return text == "1" || text == "ON" || text == "on" || text == "TRUE"
         || text == "true" || text == "YES" || text == "yes";
}

void discardFilamentLog(void*, const char*) {}

void configureFilamentLogging()
{
  if (isTruthyEnvironmentVariable("DART_VERBOSE")
      || isTruthyEnvironmentVariable("DART_GUI_VERBOSE")) {
    return;
  }

  ::utils::slog.d.setConsumer(discardFilamentLog, nullptr);
  ::utils::slog.i.setConsumer(discardFilamentLog, nullptr);
  ::utils::slog.v.setConsumer(discardFilamentLog, nullptr);
}

// Copy the per-scene fields from a user-provided ApplicationOptions into the
// parsed AppOptions. Used by the demos host when (re)building the active scene.
void applySceneOptions(
    AppOptions& appOptions,
    const dart::gui::ApplicationOptions& src,
    bool renderOutputModeExplicit,
    dart::gui::RenderOutputMode renderOutputMode)
{
  appOptions.renderableProvider = src.renderableProvider;
  appOptions.dockingEnabled = src.dockingEnabled;
  appOptions.preStep = src.preStep;
  appOptions.postStep = src.postStep;
  appOptions.renderSettings = src.renderSettings;
  if (renderOutputModeExplicit) {
    appOptions.renderSettings.outputMode = renderOutputMode;
  }
  appOptions.gizmos = src.gizmos;
  appOptions.debugLabels = src.debugLabels;
  appOptions.debugProvider = src.debugProvider;
  appOptions.ikHandles = src.ikHandles;
  appOptions.bodyNodeDragHandles = src.bodyNodeDragHandles;
  appOptions.keyboardActions = src.keyboardActions;
  appOptions.preRender = src.preRender;
  appOptions.postRender = src.postRender;
  appOptions.timeStep = src.timeStep;
  appOptions.advanceSimulation = src.advanceSimulation;
  appOptions.panels = src.panels;
  appOptions.camera = src.camera;
  appOptions.cameraControlsProvider = src.cameraControlsProvider;
  appOptions.cameraUpdater = src.cameraUpdater;
  appOptions.viewportLayoutProvider = src.viewportLayoutProvider;
  appOptions.onViewportPaneActivated = src.onViewportPaneActivated;
  appOptions.onForceDrag = src.onForceDrag;
}

// Canonicalize a scene id so hyphen- and underscore-separated names compare
// equal. The catalog uses snake_case ids (e.g. "atlas_simbicon") while --help
// and historical usage spell them with hyphens (e.g. "atlas-simbicon").
std::string canonicalSceneId(std::string_view id)
{
  std::string out(id);
  for (char& c : out) {
    if (c == '_') {
      c = '-';
    }
  }
  return out;
}

int demoSceneIndex(
    const std::vector<dart::gui::DemoSceneEntry>& scenes,
    std::string_view id,
    int fallback)
{
  const std::string target = canonicalSceneId(id);
  for (std::size_t i = 0; i < scenes.size(); ++i) {
    if (canonicalSceneId(scenes[i].id) == target) {
      return static_cast<int>(i);
    }
  }
  return fallback;
}

std::string sanitizePathComponent(std::string_view value)
{
  std::string sanitized;
  sanitized.reserve(value.size());
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) || ch == '-' || ch == '_') {
      sanitized.push_back(static_cast<char>(ch));
    } else {
      sanitized.push_back('_');
    }
  }
  return sanitized.empty() ? "scene" : sanitized;
}

std::string defaultDemoFrameOutputDirectory(std::string_view sceneId)
{
  std::error_code error;
  std::filesystem::path root = std::filesystem::temp_directory_path(error);
  if (error) {
    root = ".";
  }
  return (root / "dart_demos_frames" / sanitizePathComponent(sceneId)).string();
}

double resolveDemoSceneStartupTimeoutMs()
{
  const char* value = std::getenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS");
  if (value == nullptr || std::string_view(value).empty()) {
    return kDemoSceneStartupTimeoutMs;
  }

  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || !std::isfinite(parsed) || parsed <= 0.0) {
    return kDemoSceneStartupTimeoutMs;
  }
  return parsed;
}

std::string demoSceneDisplayName(const dart::gui::DemoSceneEntry& scene)
{
  return scene.title.empty() ? scene.id : scene.title;
}

void printDemoCatalog(const std::vector<dart::gui::DemoSceneEntry>& scenes)
{
  std::string lastCategory;
  for (const auto& scene : scenes) {
    if (scene.category != lastCategory) {
      std::cout << "\n[" << scene.category << "]\n";
      lastCategory = scene.category;
    }
    std::cout << "  " << std::left << std::setw(28) << scene.id << " "
              << demoSceneDisplayName(scene);
    if (!scene.summary.empty()) {
      std::cout << " - " << scene.summary;
    }
    std::cout << '\n';
  }
}

std::string jsonEscape(std::string_view value)
{
  std::string out;
  out.reserve(value.size() + 2);
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(ch);
        break;
    }
  }
  return out;
}

void appendDemoEvent(
    const ScriptedDemoSwitch* scriptedSwitch,
    std::string_view event,
    std::string_view activeScene,
    std::string_view status,
    int frame)
{
  if (scriptedSwitch == nullptr || scriptedSwitch->eventLogPath.empty()) {
    return;
  }

  std::ofstream out(scriptedSwitch->eventLogPath, std::ios::app);
  if (!out) {
    return;
  }
  out << "{\"source\":\"viewer\",\"event\":\"" << jsonEscape(event)
      << "\",\"frame\":" << frame << ",\"active_scene\":\""
      << jsonEscape(activeScene) << "\",\"target_scene\":\""
      << jsonEscape(scriptedSwitch->sceneId) << "\",\"status\":\""
      << jsonEscape(status) << "\"}\n";
}

std::optional<ScriptedDemoSwitch> parseScriptedDemoSwitch(
    std::string_view spec, std::ostream& errors)
{
  const std::size_t separator = spec.find(':');
  if (separator == std::string_view::npos || separator == 0
      || separator + 1 >= spec.size()) {
    errors << "--scripted-demo-switch expects <after-frames>:<scene-id>\n";
    return std::nullopt;
  }

  const std::string frameText(spec.substr(0, separator));
  char* end = nullptr;
  const long parsedFrame = std::strtol(frameText.c_str(), &end, 10);
  if (end == frameText.c_str() || *end != '\0' || parsedFrame < 1
      || parsedFrame > std::numeric_limits<int>::max()) {
    errors << "--scripted-demo-switch frame must be a positive integer\n";
    return std::nullopt;
  }

  ScriptedDemoSwitch scriptedSwitch;
  scriptedSwitch.afterFrames = static_cast<int>(parsedFrame);
  scriptedSwitch.sceneId = std::string(spec.substr(separator + 1));
  return scriptedSwitch;
}

std::optional<Eigen::Vector3d> parseVector3(
    std::string_view text, std::ostream& errors)
{
  Eigen::Vector3d values = Eigen::Vector3d::Zero();
  std::size_t begin = 0;
  for (int i = 0; i < 3; ++i) {
    const std::size_t end = i < 2 ? text.find(',', begin) : text.size();
    if (end == std::string_view::npos || end == begin) {
      errors << "expected vector as <x>,<y>,<z>\n";
      return std::nullopt;
    }
    const std::string component(text.substr(begin, end - begin));
    char* parsedEnd = nullptr;
    values[i] = std::strtod(component.c_str(), &parsedEnd);
    if (parsedEnd == component.c_str() || *parsedEnd != '\0'
        || !std::isfinite(values[i])) {
      errors << "expected finite vector component in '" << text << "'\n";
      return std::nullopt;
    }
    begin = end + 1;
  }
  if (begin < text.size() + 1) {
    errors << "expected vector as <x>,<y>,<z>\n";
    return std::nullopt;
  }
  return values;
}

std::optional<Eigen::Vector2d> parseVector2(
    std::string_view text, std::ostream& errors)
{
  Eigen::Vector2d values = Eigen::Vector2d::Zero();
  std::size_t begin = 0;
  for (int i = 0; i < 2; ++i) {
    const std::size_t end = i < 1 ? text.find(',', begin) : text.size();
    if (end == std::string_view::npos || end == begin) {
      errors << "expected vector as <x>,<y>\n";
      return std::nullopt;
    }
    const std::string component(text.substr(begin, end - begin));
    char* parsedEnd = nullptr;
    values[i] = std::strtod(component.c_str(), &parsedEnd);
    if (parsedEnd == component.c_str() || *parsedEnd != '\0'
        || !std::isfinite(values[i])) {
      errors << "expected finite vector component in '" << text << "'\n";
      return std::nullopt;
    }
    begin = end + 1;
  }
  if (begin < text.size() + 1) {
    errors << "expected vector as <x>,<y>\n";
    return std::nullopt;
  }
  return values;
}

std::optional<ScriptedForceDrag> parseScriptedForceDrag(
    std::string_view spec, std::ostream& errors)
{
  const std::size_t first = spec.find(':');
  const std::size_t second = first == std::string_view::npos
                                 ? std::string_view::npos
                                 : spec.find(':', first + 1);
  const std::size_t third = second == std::string_view::npos
                                ? std::string_view::npos
                                : spec.find(':', second + 1);
  if (first == std::string_view::npos || second == std::string_view::npos
      || third == std::string_view::npos || first == 0 || second == first + 1
      || third == second + 1 || third + 1 >= spec.size()) {
    errors << "--scripted-force-drag expects "
              "<after-frames>:<target>:<dx>,<dy>,<dz>:<duration-frames>\n";
    return std::nullopt;
  }

  const auto parsePositiveInt =
      [&errors](
          std::string_view text, std::string_view field) -> std::optional<int> {
    const std::string value(text);
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || parsed < 1
        || parsed > std::numeric_limits<int>::max()) {
      errors << field << " must be a positive integer\n";
      return std::nullopt;
    }
    return static_cast<int>(parsed);
  };

  auto afterFrames = parsePositiveInt(spec.substr(0, first), "after-frames");
  if (!afterFrames.has_value()) {
    return std::nullopt;
  }
  auto offset
      = parseVector3(spec.substr(second + 1, third - second - 1), errors);
  if (!offset.has_value()) {
    return std::nullopt;
  }
  auto durationFrames
      = parsePositiveInt(spec.substr(third + 1), "duration-frames");
  if (!durationFrames.has_value()) {
    return std::nullopt;
  }

  ScriptedForceDrag forceDrag;
  forceDrag.afterFrames = *afterFrames;
  forceDrag.target = std::string(spec.substr(first + 1, second - first - 1));
  forceDrag.targetOffset = *offset;
  forceDrag.durationFrames = *durationFrames;
  return forceDrag;
}

std::optional<ScriptedForceDrag> parseScriptedPointerForceDrag(
    std::string_view spec, std::ostream& errors)
{
  const std::size_t first = spec.find(':');
  const std::size_t second = first == std::string_view::npos
                                 ? std::string_view::npos
                                 : spec.find(':', first + 1);
  const std::size_t third = second == std::string_view::npos
                                ? std::string_view::npos
                                : spec.find(':', second + 1);
  if (first == std::string_view::npos || second == std::string_view::npos
      || third == std::string_view::npos || first == 0 || second == first + 1
      || third == second + 1 || third + 1 >= spec.size()) {
    errors << "--scripted-pointer-force-drag expects "
              "<after-frames>:<x>,<y>:<dx>,<dy>:<duration-frames>\n";
    return std::nullopt;
  }

  const auto parsePositiveInt =
      [&errors](
          std::string_view text, std::string_view field) -> std::optional<int> {
    const std::string value(text);
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || parsed < 1
        || parsed > std::numeric_limits<int>::max()) {
      errors << field << " must be a positive integer\n";
      return std::nullopt;
    }
    return static_cast<int>(parsed);
  };

  auto afterFrames = parsePositiveInt(spec.substr(0, first), "after-frames");
  if (!afterFrames.has_value()) {
    return std::nullopt;
  }
  auto startCursor
      = parseVector2(spec.substr(first + 1, second - first - 1), errors);
  if (!startCursor.has_value()) {
    return std::nullopt;
  }
  auto cursorDelta
      = parseVector2(spec.substr(second + 1, third - second - 1), errors);
  if (!cursorDelta.has_value()) {
    return std::nullopt;
  }
  auto durationFrames
      = parsePositiveInt(spec.substr(third + 1), "duration-frames");
  if (!durationFrames.has_value()) {
    return std::nullopt;
  }

  ScriptedForceDrag forceDrag;
  forceDrag.afterFrames = *afterFrames;
  forceDrag.target
      = "pixel:" + std::string(spec.substr(first + 1, second - first - 1));
  forceDrag.startCursor = *startCursor;
  forceDrag.cursorDelta = *cursorDelta;
  forceDrag.durationFrames = *durationFrames;
  forceDrag.usePointer = true;
  return forceDrag;
}

std::vector<std::filesystem::path> recordedFramePaths(
    const std::string& outputDirectory)
{
  std::vector<std::filesystem::path> paths;
  if (outputDirectory.empty()) {
    return paths;
  }

  std::error_code error;
  if (!std::filesystem::is_directory(outputDirectory, error)) {
    return paths;
  }

  for (const auto& entry :
       std::filesystem::directory_iterator(outputDirectory, error)) {
    if (error || !entry.is_regular_file(error)) {
      continue;
    }
    const auto path = entry.path();
    const std::string filename = path.filename().string();
    if (path.extension() == ".ppm" && filename.starts_with("frame_")) {
      paths.push_back(path);
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

int recordedFrameTimelineSampleCount(int frameCount)
{
  constexpr int kMaximumTimelineSamples = 1024;
  return std::clamp(frameCount, 0, kMaximumTimelineSamples);
}

int frameIndexForTimelineSample(
    int sampleIndex, int sampleCount, int frameCount)
{
  if (sampleCount <= 1 || frameCount <= 1) {
    return 0;
  }

  const double normalized
      = static_cast<double>(sampleIndex) / static_cast<double>(sampleCount - 1);
  return static_cast<int>(
      std::lround(normalized * static_cast<double>(frameCount - 1)));
}

std::vector<double> makeRecordedFrameMarkerTrack(int frameCount)
{
  const int sampleCount = recordedFrameTimelineSampleCount(frameCount);
  if (sampleCount <= 0) {
    return {};
  }

  std::vector<double> values(static_cast<std::size_t>(sampleCount), 0.0);
  const int markerInterval = std::max(1, frameCount / 12);
  for (int sample = 0; sample < sampleCount; ++sample) {
    const int frame = frameIndexForTimelineSample(
        sample, sampleCount, std::max(1, frameCount));
    if (frame == 0 || frame + 1 == frameCount || frame % markerInterval == 0) {
      values[static_cast<std::size_t>(sample)] = 1.0;
    }
  }
  return values;
}

std::vector<double> makeRecordedFrameCursorTrack(int frameCount, int frameIndex)
{
  const int sampleCount = recordedFrameTimelineSampleCount(frameCount);
  if (sampleCount <= 0) {
    return {};
  }

  std::vector<double> values(static_cast<std::size_t>(sampleCount), 0.0);
  const int clampedFrame
      = std::clamp(frameIndex, 0, std::max(0, frameCount - 1));
  const int cursorSample = frameCount <= 1
                               ? 0
                               : static_cast<int>(std::lround(
                                     static_cast<double>(clampedFrame)
                                     * static_cast<double>(sampleCount - 1)
                                     / static_cast<double>(frameCount - 1)));
  values[static_cast<std::size_t>(std::clamp(cursorSample, 0, sampleCount - 1))]
      = 1.0;
  return values;
}

std::string formatFixed(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

bool toolbarButton(
    dart::gui::PanelBuilder& builder,
    std::string_view label,
    std::string_view tooltip)
{
  const bool clicked = builder.button(label);
  builder.itemTooltip(tooltip);
  return clicked;
}

std::string& demoSidebarSearch()
{
  static std::string search;
  return search;
}

dart::gui::Panel makeDemoSimulationPanel(
    const std::vector<dart::gui::DemoSceneEntry>& scenes, int activeIndex)
{
  dart::gui::Panel panel;
  panel.title = "Simulation";
  panel.dockSide = dart::gui::DockSide::Top;
  panel.initialSize = std::array<double, 2>{760.0, 132.0};
  panel.autoResize = false;
  panel.buildWithContext = [&scenes, activeIndex](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    dart::gui::ViewerLifecycleState* lifecycle = context.lifecycle;
    if (lifecycle == nullptr) {
      builder.text("Simulation controls unavailable.");
      return;
    }

    const dart::gui::DemoSceneEntry* active = nullptr;
    if (activeIndex >= 0 && activeIndex < static_cast<int>(scenes.size())) {
      active = &scenes[static_cast<std::size_t>(activeIndex)];
    }

    if (toolbarButton(
            builder,
            lifecycle->paused ? ">##start_simulation" : "||##pause_simulation",
            lifecycle->paused ? "Start simulation." : "Pause simulation.")) {
      togglePaused(*lifecycle);
    }
    builder.sameLine();
    if (toolbarButton(
            builder, ">|##step_simulation", "Advance one simulation step.")) {
      requestSingleStep(*lifecycle);
    }
    builder.sameLine();
    if (toolbarButton(builder, "Rebuild", "Rebuild the active demo scene.")
        && active != nullptr) {
      requestSceneSwitch(*lifecycle, active->id);
    }
    builder.sameLine();
    if (toolbarButton(
            builder,
            "Restart",
            "Rebuild and run the active demo from the beginning.")
        && active != nullptr) {
      requestSceneReplay(*lifecycle, active->id);
    }
    builder.sameLine();
    if (toolbarButton(
            builder,
            "Layout",
            "Restore the default docked workspace layout.")) {
      requestDockLayoutReset(*lifecycle);
    }
    builder.sameLine();
    const bool recordingFrames = lifecycle->frameOutputEnabled
                                 && !lifecycle->frameOutputDirectory.empty();
    const std::string recordDirectory
        = active == nullptr ? defaultDemoFrameOutputDirectory("scene")
                            : defaultDemoFrameOutputDirectory(active->id);
    std::string status = "time " + formatFixed(context.simulationTime, 3)
                         + " s | contacts "
                         + std::to_string(context.contactCount);
    if (active != nullptr) {
      status += " | " + active->title;
    }

    if (toolbarButton(
            builder,
            recordingFrames ? "Stop Capture##record_frames"
                            : "Capture##record_frames",
            recordingFrames ? "Stop recording frame images."
                            : "Record frame images for captured-frame "
                              "playback.")) {
      toggleFrameOutputCapture(*lifecycle, recordDirectory);
    }
    builder.sameLine();
    builder.text(status);
    if (recordingFrames) {
      builder.sameLine();
      builder.text("recording frames");
      builder.itemTooltip(lifecycle->frameOutputDirectory);
    }
    if (!lifecycle->sceneActivationStatus.empty()) {
      builder.text(lifecycle->sceneActivationStatus);
    }

    const auto recordedFrames
        = recordedFramePaths(lifecycle->frameOutputDirectory);
    const int recordedFrameCount
        = recordedFrames.size()
                  > static_cast<std::size_t>(std::numeric_limits<int>::max())
              ? std::numeric_limits<int>::max()
              : static_cast<int>(recordedFrames.size());
    advanceRecordedFramePlayback(*lifecycle, recordedFrameCount);
    if (!lifecycle->frameOutputDirectory.empty() || recordedFrameCount > 0) {
      builder.separator();
      builder.text(
          std::string("captured frames ") + std::to_string(recordedFrameCount));
      builder.itemTooltip(lifecycle->frameOutputDirectory);

      if (recordedFrameCount > 0) {
        builder.sameLine();
        if (toolbarButton(
                builder, "|<##first_recorded_frame", "First recorded frame.")) {
          setRecordedFramePlaybackIndex(*lifecycle, recordedFrameCount, 0);
        }
        builder.sameLine();
        if (toolbarButton(
                builder,
                "<##previous_recorded_frame",
                "Previous recorded frame.")) {
          stepRecordedFramePlayback(*lifecycle, recordedFrameCount, -1);
        }
        builder.sameLine();
        if (toolbarButton(
                builder,
                lifecycle->recordedFramePlaybackPlaying
                    ? "||##pause_recorded_frames"
                    : ">##play_recorded_frames",
                lifecycle->recordedFramePlaybackPlaying
                    ? "Pause recorded-frame playback."
                    : "Play recorded frames.")) {
          toggleRecordedFramePlayback(*lifecycle, recordedFrameCount);
        }
        builder.sameLine();
        if (toolbarButton(
                builder, ">##next_recorded_frame", "Next recorded frame.")) {
          stepRecordedFramePlayback(*lifecycle, recordedFrameCount, 1);
        }
        builder.sameLine();
        if (toolbarButton(
                builder, ">|##last_recorded_frame", "Last recorded frame.")) {
          setRecordedFramePlaybackIndex(
              *lifecycle, recordedFrameCount, recordedFrameCount - 1);
        }
        int selectedIndex = std::clamp(
            lifecycle->recordedFramePlaybackIndex, 0, recordedFrameCount - 1);
        setRecordedFramePlaybackIndex(
            *lifecycle, recordedFrameCount, selectedIndex);
        double selectedFrame = static_cast<double>(selectedIndex);
        const auto markerTrack
            = makeRecordedFrameMarkerTrack(recordedFrameCount);
        const auto cursorTrack = makeRecordedFrameCursorTrack(
            recordedFrameCount, lifecycle->recordedFramePlaybackIndex);
        if (builder.timeline(
                "Captured frame##recorded_frame_timeline",
                selectedFrame,
                0.0,
                static_cast<double>(recordedFrameCount - 1),
                {},
                markerTrack,
                cursorTrack,
                "Captured frames")) {
          selectedIndex = std::clamp(
              static_cast<int>(std::lround(selectedFrame)),
              0,
              recordedFrameCount - 1);
          setRecordedFramePlaybackIndex(
              *lifecycle, recordedFrameCount, selectedIndex);
        }
        const auto& selectedPath
            = recordedFrames[static_cast<std::size_t>(selectedIndex)];
        builder.text(
            std::to_string(selectedIndex + 1) + "/"
            + std::to_string(recordedFrameCount) + " "
            + selectedPath.filename().string());
        builder.itemTooltip(selectedPath.string());
      }
    }
  };
  return panel;
}

// Built-in sidebar panel listing the demo catalog grouped by category (ordered
// by first appearance). Selecting a different scene requests a runtime switch.
dart::gui::Panel makeDemoSidebarPanel(
    const std::vector<dart::gui::DemoSceneEntry>& scenes, int activeIndex)
{
  dart::gui::Panel panel;
  panel.title = "Demos";
  panel.dockSide = dart::gui::DockSide::Left;
  panel.initialPosition = std::array<double, 2>{12.0, 12.0};
  panel.initialSize = std::array<double, 2>{280.0, 560.0};
  panel.autoResize = false;
  panel.buildWithContext = [&scenes, activeIndex](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    dart::gui::ViewerLifecycleState* lifecycle = context.lifecycle;
    const dart::gui::DemoSceneEntry* active = nullptr;
    if (activeIndex >= 0 && activeIndex < static_cast<int>(scenes.size())) {
      active = &scenes[static_cast<std::size_t>(activeIndex)];
      builder.text("Current: " + active->title);
      if (!active->summary.empty()) {
        builder.text(active->summary);
      }
    }
    if (lifecycle != nullptr && !lifecycle->sceneActivationStatus.empty()) {
      builder.separator();
      builder.text(lifecycle->sceneActivationStatus);
    }
    builder.separator();

    std::string& search = demoSidebarSearch();
    builder.text("Search");
    builder.textInput("##demo_search", search);
    if (!search.empty()) {
      builder.sameLine();
      if (builder.button("Clear")) {
        search.clear();
      }
    }
    const std::string normalizedSearch
        = dart::gui::detail::normalizedDemoSearchText(search);
    std::size_t visibleSceneCount = 0;
    for (const auto& scene : scenes) {
      if (dart::gui::detail::demoSceneVisibleInNavigator(
              scene, normalizedSearch)) {
        ++visibleSceneCount;
      }
    }
    builder.text(
        "Showing " + std::to_string(visibleSceneCount) + "/"
        + std::to_string(scenes.size()) + " demos");
    builder.separator();

    // Tree-style catalog: categories are grouped by first appearance even if
    // the scene list later interleaves categories. Scenes within each category
    // are indented list rows (selectable text instead of buttons).
    bool anyVisible = false;
    const std::vector<dart::gui::detail::DemoCategoryGroup> categoryGroups
        = dart::gui::detail::groupDemoScenesByCategory(scenes);
    for (const auto& group : categoryGroups) {
      std::size_t visibleCount = 0;
      std::size_t totalCount = 0;
      bool categoryHasActive = false;
      for (const std::size_t i : group.sceneIndices) {
        ++totalCount;
        if (dart::gui::detail::demoSceneVisibleInNavigator(
                scenes[i], normalizedSearch)) {
          ++visibleCount;
        }
        categoryHasActive
            = categoryHasActive || static_cast<int>(i) == activeIndex;
      }
      if (visibleCount == 0u) {
        continue;
      }

      anyVisible = true;
      std::string header = group.category + " (";
      if (!normalizedSearch.empty()) {
        header += std::to_string(visibleCount) + "/";
      }
      header += std::to_string(totalCount) + ")##demo_category_"
                + sanitizePathComponent(group.category);
      const bool defaultOpen = !normalizedSearch.empty() || categoryHasActive;
      const bool categoryOpen = builder.collapsingHeader(header, defaultOpen);
      if (categoryOpen) {
        builder.indent();
        for (const std::size_t i : group.sceneIndices) {
          const auto& entry = scenes[i];
          if (!dart::gui::detail::demoSceneVisibleInNavigator(
                  entry, normalizedSearch)) {
            continue;
          }
          const bool isActive = static_cast<int>(i) == activeIndex;
          const bool isPending
              = lifecycle != nullptr
                && lifecycle->sceneActivationPendingScene == entry.id;
          std::string label = entry.title;
          if (isPending) {
            label += " (starting)";
          }
          label += "##demo_" + entry.id;
          const bool clicked = builder.selectable(label, isActive || isPending);
          // Explain what each scene demonstrates when the row is hovered.
          builder.itemTooltip(entry.summary);
          if (clicked && !isActive && lifecycle != nullptr
              && lifecycle->sceneActivationPendingScene != entry.id) {
            requestSceneSwitch(*lifecycle, entry.id);
          }
        }
        builder.unindent();
      }
    }
    if (!anyVisible) {
      builder.text("No demos match the current search.");
    }
  };
  return panel;
}

int runGuiBackendApplicationImpl(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions,
    const std::vector<dart::gui::DemoSceneEntry>* demoCatalog,
    int initialDemoIndex,
    bool cycleScenes,
    int cycleFramesPerScene = kDefaultCycleFramesPerScene,
    std::optional<ScriptedDemoSwitch> scriptedDemoSwitch = std::nullopt,
    std::optional<ScriptedForceDrag> scriptedForceDrag = std::nullopt)
{
  AppOptions appOptions
      = parseOptions(argc, argv, applicationOptions.runDefaults);
  if (cycleScenes) {
    appOptions.run.maxFrames = -1;
  }
  const bool renderOutputModeExplicit = appOptions.renderOutputModeExplicit;
  const auto renderOutputMode = appOptions.renderSettings.outputMode;
  appOptions.camera = applicationOptions.camera;
  // Single-scene wiring (one world/options passed in). The demos host instead
  // (re)applies each scene's options inside the scene loop below.
  if (demoCatalog == nullptr) {
    if (!hasSceneOption(argc, argv)) {
      appOptions.renderableProvider = applicationOptions.renderableProvider;
      appOptions.selectedRenderableProvider
          = applicationOptions.selectedRenderableProvider;
      appOptions.onRenderableSelected = applicationOptions.onRenderableSelected;
      appOptions.onForceDrag = applicationOptions.onForceDrag;
      appOptions.dockingEnabled = applicationOptions.dockingEnabled;
      appOptions.preStep = applicationOptions.preStep;
      appOptions.postStep = applicationOptions.postStep;
      appOptions.renderSettings = applicationOptions.renderSettings;
      if (renderOutputModeExplicit) {
        appOptions.renderSettings.outputMode = renderOutputMode;
      }
      appOptions.gizmos = applicationOptions.gizmos;
      appOptions.debugLabels = applicationOptions.debugLabels;
      appOptions.debugProvider = applicationOptions.debugProvider;
      appOptions.ikHandles = applicationOptions.ikHandles;
      appOptions.bodyNodeDragHandles = applicationOptions.bodyNodeDragHandles;
      appOptions.keyboardActions = applicationOptions.keyboardActions;
    }
    appOptions.preRender = applicationOptions.preRender;
    appOptions.postRender = applicationOptions.postRender;
    appOptions.timeStep = applicationOptions.timeStep;
    appOptions.advanceSimulation = applicationOptions.advanceSimulation;
    appOptions.panels = applicationOptions.panels;
    appOptions.cameraControlsProvider
        = applicationOptions.cameraControlsProvider;
    appOptions.cameraUpdater = applicationOptions.cameraUpdater;
    appOptions.viewportLayoutProvider
        = applicationOptions.viewportLayoutProvider;
    appOptions.onViewportPaneActivated
        = applicationOptions.onViewportPaneActivated;
  }
  const RunOptions& runOptions = appOptions.run;

  if (!runOptions.frameOutputDirectory.empty()) {
    std::error_code error;
    const std::filesystem::path outputDirectory(
        runOptions.frameOutputDirectory);
    std::filesystem::create_directories(outputDirectory, error);
    if (error) {
      std::cerr << "Failed to create frame output directory '"
                << runOptions.frameOutputDirectory << "': " << error.message()
                << "\n";
      return 1;
    }
    if (!std::filesystem::is_directory(outputDirectory, error)) {
      std::cerr << "Frame output path is not a directory: "
                << runOptions.frameOutputDirectory << "\n";
      return 1;
    }
  }

  configureFilamentLogging();

  ApplicationWindow appWindow = createApplicationWindow(
      runOptions,
      !appOptions.windowWidthExplicit,
      !appOptions.windowHeightExplicit,
      std::cerr);
  GLFWwindow* window = appWindow.get();
  if (!runOptions.headless && window == nullptr) {
    return 1;
  }

  OrbitCameraController cameraController;
  attachOrbitCameraController(window, cameraController);

  FilamentRenderContext renderContext = createFilamentRenderContext(
      runOptions, runOptions.headless ? nullptr : getNativeWindow(window));
  auto* engine = renderContext.engine;
  auto* scene = renderContext.scene;
  auto* colorGrading = createDebugColorGrading(*engine);
  // The heavy screen-space passes are normally skipped when headless (CI
  // renders on llvmpipe). On a real GPU, DART_GUI_HIGH_FIDELITY restores them
  // so headless sensor/synthetic-data capture matches the windowed image. Treat
  // such runs as non-lightweight even though they are headless.
  const bool useLightweightPasses
      = runOptions.headless && !highFidelityHeadlessRequested();
  for (auto* view : renderContext.views) {
    if (view != nullptr) {
      configureMainView(*view, colorGrading, useLightweightPasses);
    }
  }
  auto* indirectLight = createNeutralIndirectLight(*engine);
  auto* skybox = createNeutralSkybox(*engine);

  MaterialResources materialResources = createMaterialResources(*engine);
  const MaterialSet materials = materialResources.materialSet();

  bool orbitLight = appOptions.orbitLight;
  bool headlightsEnabled = true;
  // Mutable so the F2 key can toggle the perf HUD at runtime (initialized from
  // the --perf-hud option).
  bool showPerfHud = appOptions.showPerfHud;
  SceneLights lights = createSceneLights(
      *engine,
      useLightweightPasses,
      orbitLight,
      appOptions.orbitLightPeriodSeconds);
  attachSceneEnvironment(*scene, indirectLight, skybox, lights);

  GuiScaleState guiScale = dart::gui::detail::makeGuiScaleState(
      runOptions.guiScale, resolveWindowDpiScale(window));
  double automaticWindowEffectiveScale = guiScale.effectiveScale;
  bool automaticWindowSizeResolved
      = runOptions.headless
        || (appOptions.windowWidthExplicit && appOptions.windowHeightExplicit);
  ImGuiOverlay imguiOverlay = createConfiguredImGuiOverlay(
      *engine, static_cast<float>(guiScale.effectiveScale));
  auto& imguiIo = getCurrentImGuiIo();

  ViewerLifecycleState lifecycle;
  lifecycle.frameOutputDirectory = runOptions.frameOutputDirectory;
  lifecycle.frameOutputEnabled = !runOptions.frameOutputDirectory.empty();
  ApplicationInputState inputState;
  SelectionController selectionController;
  ScreenshotCapture screenshotCapture;
  ProfileAccumulator profile;
  PerfHudState perfHud;
  bool frameCaptureSucceeded = true;
  SimulationStepper simulationStepper;
  const auto orbitStartClock = ProfileAccumulator::Clock::now();
  const double demoSceneStartupTimeoutMs = resolveDemoSceneStartupTimeoutMs();

  int activeIndex = initialDemoIndex;
  bool keepRunning = true;
  std::size_t finalContacts = 0;
  std::optional<int> pendingDemoFallbackIndex;

  // Outer scene loop. For the single-scene path this runs exactly once; for the
  // demos host it rebuilds the scene-bound state when a switch is requested,
  // while the window, engine, materials, lights, and ImGui overlay persist.
  while (keepRunning) {
    const auto sceneStartupStart = ProfileAccumulator::Clock::now();
    const dart::gui::DemoSceneEntry* candidateDemoEntry = nullptr;
    auto restorePendingDemoFallback = [&](std::string_view reason) -> bool {
      if (demoCatalog == nullptr || candidateDemoEntry == nullptr
          || !pendingDemoFallbackIndex.has_value()) {
        return false;
      }

      const int fallbackIndex = *pendingDemoFallbackIndex;
      if (fallbackIndex < 0
          || fallbackIndex >= static_cast<int>(demoCatalog->size())
          || fallbackIndex == activeIndex) {
        pendingDemoFallbackIndex.reset();
        return false;
      }

      const auto& fallbackEntry
          = (*demoCatalog)[static_cast<std::size_t>(fallbackIndex)];
      std::cerr << "demo scene '" << candidateDemoEntry->id
                << "' failed to start (" << reason
                << "); restoring previous demo '" << fallbackEntry.id << "'\n";
      lifecycle.sceneActivationStatus
          = "Restored previous demo '" + demoSceneDisplayName(fallbackEntry)
            + "' after '" + demoSceneDisplayName(*candidateDemoEntry)
            + "' failed: " + std::string(reason);
      appendDemoEvent(
          scriptedDemoSwitch ? &*scriptedDemoSwitch : nullptr,
          "restored_previous_demo",
          fallbackEntry.id,
          lifecycle.sceneActivationStatus,
          -1);
      lifecycle.sceneActivationPendingScene.clear();
      activeIndex = fallbackIndex;
      pendingDemoFallbackIndex.reset();
      lifecycle.sceneSwitchRequested = false;
      lifecycle.requestedScene.clear();
      return true;
    };

    if (demoCatalog != nullptr) {
      const auto& demoEntry
          = (*demoCatalog)[static_cast<std::size_t>(activeIndex)];
      candidateDemoEntry = &demoEntry;
      if (cycleScenes) {
        std::cout << "Cycling demo scene " << (activeIndex + 1) << "/"
                  << demoCatalog->size() << ": " << demoEntry.id << "\n"
                  << std::flush;
      }

      dart::gui::ApplicationOptions sceneOptions;
      try {
        sceneOptions = demoEntry.factory();
      } catch (const std::exception& error) {
        if (restorePendingDemoFallback(
                std::string("factory threw: ") + error.what())) {
          continue;
        }
        // Soft-fail: a scene that cannot build (e.g. a missing asset) must not
        // crash the host. Show an empty world so the sidebar stays usable.
        lifecycle.sceneActivationStatus = "Failed to start demo '"
                                          + demoSceneDisplayName(demoEntry)
                                          + "': " + error.what();
        lifecycle.sceneActivationPendingScene.clear();
        std::cerr << "demo scene '" << demoEntry.id
                  << "' failed to load: " << error.what() << "\n";
        sceneOptions = dart::gui::ApplicationOptions{};
      }
      if (elapsedMs(sceneStartupStart) > demoSceneStartupTimeoutMs
          && restorePendingDemoFallback("factory startup exceeded budget")) {
        continue;
      }
      applySceneOptions(
          appOptions, sceneOptions, renderOutputModeExplicit, renderOutputMode);
      // The demos host uses the docked workspace layout so the catalog sidebar
      // and the status HUD never overlap; this is a no-op (floating overlay
      // fallback) on builds without ImGui docking support.
      appOptions.dockingEnabled = true;
      std::vector<dart::gui::Panel> panels;
      panels.reserve(appOptions.panels.size() + 2);
      panels.push_back(makeDemoSimulationPanel(*demoCatalog, activeIndex));
      panels.push_back(makeDemoSidebarPanel(*demoCatalog, activeIndex));
      for (auto& scenePanel : appOptions.panels) {
        panels.push_back(std::move(scenePanel));
      }
      appOptions.panels = std::move(panels);
      lifecycle.sceneSwitchRequested = false;
      lifecycle.requestedScene.clear();
      // Per-scene transient controllers are reset; renderable ids from the
      // previous scene must not leak into the next one.
      selectionController.clear();
      simulationStepper = SimulationStepper{};
    }

    cameraController.camera
        = appOptions.camera.value_or(initialCameraForScene(appOptions.scene));
    if (appOptions.cameraUpdater) {
      appOptions.cameraUpdater(cameraController.camera);
    }
    const auto homeCamera = cameraController.camera;

    const bool validateFixtureRequirements = false;
    DartScene dartScene = createDartScene(appOptions);
    std::optional<InitialSceneState> maybeInitialSceneState
        = createInitialSceneState(
            *engine,
            *scene,
            materials,
            materialResources,
            appOptions.scene,
            dartScene,
            validateFixtureRequirements,
            applicationOptions.allowEmptyScene,
            std::cerr);
    if (!maybeInitialSceneState) {
      if (demoCatalog != nullptr
          && restorePendingDemoFallback("render state creation failed")) {
        continue;
      }
      frameCaptureSucceeded = false;
      keepRunning = false;
      break;
    }
    InitialSceneState sceneState = std::move(*maybeInitialSceneState);
    std::optional<Renderable> selectionDebugOverlay;
    if (demoCatalog != nullptr
        && elapsedMs(sceneStartupStart) > demoSceneStartupTimeoutMs
        && restorePendingDemoFallback("startup exceeded budget")) {
      destroySceneRenderables(
          *engine,
          *scene,
          sceneState.sceneRenderables,
          sceneState.debugOverlays,
          selectionDebugOverlay);
      continue;
    }
    auto& sceneRenderables = sceneState.sceneRenderables;
    auto& debugOverlays = sceneState.debugOverlays;

    SceneFrameUpdater sceneFrameUpdater(
        window,
        *engine,
        *scene,
        materials,
        materialResources,
        runOptions,
        dartScene,
        selectionController,
        sceneState,
        selectionDebugOverlay,
        lifecycle,
        simulationStepper,
        lights,
        orbitStartClock,
        profile,
        scriptedForceDrag ? &*scriptedForceDrag : nullptr);

    bool sceneFrameFailed = false;
    bool restoredPendingDemoDuringFrame = false;
    std::string sceneFrameFailureReason = "first frame failed";
    bool cycleAdvance = false;
    int framesThisScene = 0;

    while (!lifecycle.exitRequested && !lifecycle.sceneSwitchRequested
           && !cycleAdvance
           && shouldContinueApplicationLoop(runOptions.headless, window)) {
      DART_PROFILE_FRAME;
      DART_PROFILE_SCOPED_N("GUI frame");
      const auto frameStart = ProfileAccumulator::Clock::now();
      auto phaseStart = ProfileAccumulator::Clock::now();
      {
        DART_PROFILE_SCOPED_N("GUI input");
        pollApplicationInput(
            window,
            dartScene,
            selectionController,
            lifecycle,
            cameraController,
            homeCamera,
            inputState,
            showPerfHud);
      }
      profile.inputMs += elapsedMs(phaseStart);

      guiScale = dart::gui::detail::makeGuiScaleState(
          runOptions.guiScale, resolveWindowDpiScale(window));
      if (!automaticWindowSizeResolved
          || std::abs(guiScale.effectiveScale - automaticWindowEffectiveScale)
                 > 1e-4) {
        resizeAutomaticApplicationWindow(
            window,
            runOptions,
            guiScale,
            !appOptions.windowWidthExplicit,
            !appOptions.windowHeightExplicit);
        automaticWindowEffectiveScale = guiScale.effectiveScale;
        automaticWindowSizeResolved = true;
      }
      updateConfiguredImGuiOverlayScale(
          *engine, imguiOverlay, static_cast<float>(guiScale.effectiveScale));

      phaseStart = ProfileAccumulator::Clock::now();
      FrameViewport viewport;
      {
        DART_PROFILE_SCOPED_N("GUI viewport camera");
        const dart::gui::OrbitCameraControlOptions cameraControls
            = appOptions.cameraControlsProvider
                  ? appOptions.cameraControlsProvider()
                  : dart::gui::OrbitCameraControlOptions{};
        dart::gui::ViewportLayoutOptions viewportLayout
            = appOptions.viewportLayoutProvider
                  ? appOptions.viewportLayoutProvider(cameraController.camera)
                  : dart::gui::ViewportLayoutOptions{};
        if (!appOptions.viewportLayoutProvider) {
          viewportLayout.panes[0].camera = cameraController.camera;
          viewportLayout.panes[0].active = true;
        }
        viewport = updateFrameViewport(
            window,
            renderContext.views,
            renderContext.cameras,
            cameraController,
            selectionController,
            imguiIo,
            runOptions.width,
            runOptions.height,
            dartScene.timeStep,
            appOptions.showUi,
            cameraControls,
            viewportLayout);
        renderContext.activeViewCount = viewport.paneCount;
      }
      profile.viewportCameraMs += elapsedMs(phaseStart);

      const bool uiCapturesMouse
          = isSceneMouseInputCapturedByUi(appOptions.showUi, imguiIo);
      try {
        DART_PROFILE_SCOPED_N("GUI scene update");
        sceneFrameUpdater.update(
            viewport,
            appOptions.showUi,
            uiCapturesMouse,
            orbitLight,
            appOptions.orbitLightPeriodSeconds);
      } catch (const std::exception& error) {
        sceneFrameFailureReason
            = std::string("frame update threw: ") + error.what();
        std::cerr << "demo scene '"
                  << (candidateDemoEntry == nullptr ? std::string()
                                                    : candidateDemoEntry->id)
                  << "' frame update failed: " << error.what() << "\n";
        sceneFrameFailed = true;
        break;
      } catch (...) {
        sceneFrameFailureReason = "frame update threw an unknown exception";
        std::cerr << "demo scene '"
                  << (candidateDemoEntry == nullptr ? std::string()
                                                    : candidateDemoEntry->id)
                  << "' frame update failed with an unknown exception\n";
        sceneFrameFailed = true;
        break;
      }

      if (appOptions.showUi) {
        DART_PROFILE_SCOPED_N("GUI ui update");
        updateFrameUi(
            window,
            *engine,
            *scene,
            materials.debugColor,
            imguiOverlay,
            imguiIo,
            viewport,
            appOptions.scene,
            dartScene,
            cameraController,
            selectionController,
            orbitLight,
            headlightsEnabled,
            debugOverlays,
            appOptions.panels,
            lifecycle,
            guiScale,
            profile,
            showPerfHud,
            perfHud,
            renderContext.backendName);
      }
      setSceneLightsEnabled(*engine, lights, headlightsEnabled);
      for (std::size_t i = 0; i < renderContext.views.size(); ++i) {
        if (renderContext.views[i] != nullptr) {
          applyRenderSettings(
              *renderContext.views[i], dartScene.renderSettings);
        }
      }

      if (appOptions.preRender) {
        appOptions.preRender();
      }
      FrameRenderResult frameRenderResult;
      {
        DART_PROFILE_SCOPED_N("GUI render frame");
        frameRenderResult = renderApplicationFrame(
            renderContext,
            appOptions.showUi ? imguiOverlay.view : nullptr,
            runOptions,
            viewport.width,
            viewport.height,
            frameStart,
            screenshotCapture,
            lifecycle,
            profile);
      }
      if (showPerfHud) {
        const double gpuMs = latestGpuFrameMs(renderContext);
        if (gpuMs > 0.0) {
          profile.gpuFrameMs = gpuMs;
          if (gpuMs > profile.maxGpuFrameMs) {
            profile.maxGpuFrameMs = gpuMs;
          }
        }
      }
      if (!frameRenderResult.failed && appOptions.postRender) {
        appOptions.postRender();
      }
      if (frameRenderResult.failed) {
        sceneFrameFailed = true;
        break;
      }
      if (demoCatalog != nullptr && framesThisScene == 0
          && pendingDemoFallbackIndex.has_value()
          && elapsedMs(sceneStartupStart) > demoSceneStartupTimeoutMs
          && restorePendingDemoFallback(
              "first frame exceeded startup budget")) {
        restoredPendingDemoDuringFrame = true;
        break;
      }
      if (demoCatalog != nullptr) {
        lifecycle.sceneActivationPendingScene.clear();
        if (lifecycle.sceneActivationStatus.starts_with("Starting ")) {
          lifecycle.sceneActivationStatus.clear();
        }
        if (scriptedDemoSwitch.has_value() && scriptedDemoSwitch->requested
            && !scriptedDemoSwitch->observed
            && (*demoCatalog)[static_cast<std::size_t>(activeIndex)].id
                   == scriptedDemoSwitch->sceneId) {
          scriptedDemoSwitch->observed = true;
          appendDemoEvent(
              &*scriptedDemoSwitch,
              "observed_target_demo",
              scriptedDemoSwitch->sceneId,
              lifecycle.sceneActivationStatus,
              framesThisScene);
        }
      }
      if (demoCatalog != nullptr && !frameRenderResult.continueLoop) {
        pendingDemoFallbackIndex.reset();
      }
      if (frameRenderResult.continueLoop) {
        continue;
      }
      if (frameRenderResult.stopLoop) {
        break;
      }
      ++framesThisScene;
      if (demoCatalog != nullptr && scriptedDemoSwitch.has_value()
          && !scriptedDemoSwitch->requested
          && framesThisScene >= scriptedDemoSwitch->afterFrames) {
        scriptedDemoSwitch->requested = true;
        appendDemoEvent(
            &*scriptedDemoSwitch,
            "requested_demo_switch",
            (*demoCatalog)[static_cast<std::size_t>(activeIndex)].id,
            "scripted switch requested",
            framesThisScene);
        requestSceneSwitch(lifecycle, scriptedDemoSwitch->sceneId);
      }
      if (cycleScenes && framesThisScene >= cycleFramesPerScene) {
        cycleAdvance = true;
      }
    }

    finalContacts = dartScene.contactCount;
    sceneFrameUpdater.releaseScriptedForceDragIfActive(
        "scripted force-drag released during scene teardown");
    selectionController.cancelActiveDrag(dartScene);
    destroySceneRenderables(
        *engine,
        *scene,
        sceneRenderables,
        debugOverlays,
        selectionDebugOverlay);

    if (restoredPendingDemoDuringFrame) {
      continue;
    } else if (sceneFrameFailed) {
      if (demoCatalog != nullptr
          && restorePendingDemoFallback(sceneFrameFailureReason)) {
        continue;
      }
      frameCaptureSucceeded = false;
      keepRunning = false;
    } else if (demoCatalog != nullptr && lifecycle.sceneSwitchRequested) {
      const int requestedIndex
          = demoSceneIndex(*demoCatalog, lifecycle.requestedScene, activeIndex);
      if (requestedIndex != activeIndex) {
        pendingDemoFallbackIndex = activeIndex;
      } else {
        pendingDemoFallbackIndex.reset();
      }
      activeIndex = requestedIndex;
    } else if (demoCatalog != nullptr && cycleScenes && cycleAdvance) {
      pendingDemoFallbackIndex.reset();
      if (activeIndex + 1 < static_cast<int>(demoCatalog->size())) {
        ++activeIndex;
      } else {
        keepRunning = false;
      }
    } else {
      pendingDemoFallbackIndex.reset();
      keepRunning = false;
    }
  }

  if (scriptedDemoSwitch.has_value() && demoCatalog != nullptr) {
    std::string activeScene;
    if (activeIndex >= 0
        && activeIndex < static_cast<int>(demoCatalog->size())) {
      activeScene = (*demoCatalog)[static_cast<std::size_t>(activeIndex)].id;
    }
    appendDemoEvent(
        &*scriptedDemoSwitch,
        scriptedDemoSwitch->observed ? "script_completed"
                                     : "script_finished_without_target",
        activeScene,
        lifecycle.sceneActivationStatus,
        -1);
  }

  const bool screenshotSucceeded = finalizeScreenshotCapture(
      renderContext,
      screenshotCapture,
      runOptions.screenshotPath,
      lifecycle.screenshotRequested,
      profile);

  // Phase-1 gate of the Filament offscreen-viewport spike: when
  // DART_GUI_OFFSCREEN_PARITY is set, render the live scene to an offscreen
  // RenderTarget and confirm it matches the swapchain render (diagnostic only;
  // see docs/design/dartsim_gui_toolkit_decisions.md Decision 3).
  bool offscreenParitySucceeded = true;
  if (runOptions.headless
      && isTruthyEnvironmentVariable("DART_GUI_OFFSCREEN_PARITY")) {
    if (std::string_view(renderContext.backendName) == "noop") {
      std::cout << "[offscreen-parity] SKIP: noop backend produces no pixels\n";
    } else {
      offscreenParitySucceeded = runOffscreenParitySelfCheck(
          renderContext,
          static_cast<std::uint32_t>(
              runOptions.width > 0 ? runOptions.width : 1),
          static_cast<std::uint32_t>(
              runOptions.height > 0 ? runOptions.height : 1),
          std::cout);
    }
  }

  if (runOptions.maxFrames >= 0) {
    std::cout << "Final contacts: " << finalContacts << "\n";
  }
  if (appOptions.profile) {
    printProfile(profile);
    DART_PROFILE_TEXT_DUMP();
  }

  destroyPersistentApplicationResources(
      renderContext,
      imguiOverlay,
      lights,
      indirectLight,
      skybox,
      colorGrading,
      materialResources);

  return screenshotSucceeded && frameCaptureSucceeded
                 && offscreenParitySucceeded
             ? 0
             : 1;
}

bool hasSceneOption(int argc, char* argv[])
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] != nullptr && std::string_view(argv[i]) == "--scene") {
      return true;
    }
  }

  return false;
}

} // namespace

namespace dart::gui {

int runApplication(int argc, char* argv[])
{
  return runApplication(argc, argv, ApplicationOptions{});
}

int runApplication(int argc, char* argv[], const char* defaultScene)
{
  ApplicationOptions options;
  options.defaultScene = defaultScene == nullptr ? "" : defaultScene;
  return runApplication(argc, argv, options);
}

int runApplication(
    int argc, char* argv[], const ApplicationOptions& applicationOptions)
{
  if (applicationOptions.defaultScene.empty() || hasSceneOption(argc, argv)) {
    return detail::runGuiBackendApplication(argc, argv, applicationOptions);
  }

  std::vector<std::string> argumentStorage;
  argumentStorage.reserve(static_cast<std::size_t>(argc) + 2u);
  for (int i = 0; i < argc; ++i) {
    argumentStorage.emplace_back(argv[i] == nullptr ? "" : argv[i]);
  }
  argumentStorage.emplace_back("--scene");
  argumentStorage.emplace_back(applicationOptions.defaultScene);

  std::vector<char*> rewrittenArguments;
  rewrittenArguments.reserve(argumentStorage.size());
  for (std::string& argument : argumentStorage) {
    rewrittenArguments.push_back(argument.data());
  }

  return detail::runGuiBackendApplication(
      static_cast<int>(rewrittenArguments.size()),
      rewrittenArguments.data(),
      applicationOptions);
}

int runDemos(int argc, char* argv[], std::vector<DemoSceneEntry> scenes)
{
  if (scenes.empty()) {
    std::cerr << "runDemos: no scenes registered\n";
    return 1;
  }

  std::string initialId;
  if (const char* env = std::getenv("DART_DEMOS_SCENE")) {
    initialId = env;
  }
  bool cycleScenes = false;
  int cycleFramesPerScene = kDefaultCycleFramesPerScene;
  std::optional<ScriptedDemoSwitch> scriptedDemoSwitch;
  std::optional<ScriptedForceDrag> scriptedForceDrag;
  std::string scriptedEventLogPath;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i] == nullptr ? "" : argv[i];
    if (arg == "--list") {
      printDemoCatalog(scenes);
      return 0;
    } else if (arg == "--cycle-scenes") {
      cycleScenes = true;
    } else if (arg == "--frames" && i + 1 < argc && argv[i + 1] != nullptr) {
      cycleFramesPerScene = std::max(1, std::atoi(argv[i + 1]));
      ++i;
    }
  }

  std::vector<char*> filteredArguments;
  filteredArguments.reserve(static_cast<std::size_t>(argc));
  if (argc > 0) {
    filteredArguments.push_back(argv[0]);
  }
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i] == nullptr ? "" : argv[i];
    if (arg == "--cycle-scenes") {
      cycleScenes = true;
      continue;
    }
    if (cycleScenes && arg == "--frames") {
      if (i + 1 < argc && argv[i + 1] != nullptr) {
        ++i;
      }
      continue;
    }
    if (arg == "--scripted-demo-switch") {
      if (i + 1 >= argc || argv[i + 1] == nullptr) {
        std::cerr << "runDemos: --scripted-demo-switch requires an argument\n";
        return 1;
      }
      auto parsed = parseScriptedDemoSwitch(argv[i + 1], std::cerr);
      if (!parsed.has_value()) {
        return 1;
      }
      if (!scriptedEventLogPath.empty()) {
        parsed->eventLogPath = scriptedEventLogPath;
      }
      scriptedDemoSwitch = std::move(parsed);
      ++i;
      continue;
    }
    if (arg == "--scripted-force-drag") {
      if (i + 1 >= argc || argv[i + 1] == nullptr) {
        std::cerr << "runDemos: --scripted-force-drag requires an argument\n";
        return 1;
      }
      if (scriptedForceDrag.has_value()) {
        std::cerr << "runDemos: only one scripted force-drag mode is allowed\n";
        return 1;
      }
      auto parsed = parseScriptedForceDrag(argv[i + 1], std::cerr);
      if (!parsed.has_value()) {
        return 1;
      }
      if (!scriptedEventLogPath.empty()) {
        parsed->eventLogPath = scriptedEventLogPath;
      }
      scriptedForceDrag = std::move(parsed);
      ++i;
      continue;
    }
    if (arg == "--scripted-pointer-force-drag") {
      if (i + 1 >= argc || argv[i + 1] == nullptr) {
        std::cerr
            << "runDemos: --scripted-pointer-force-drag requires an argument\n";
        return 1;
      }
      if (scriptedForceDrag.has_value()) {
        std::cerr << "runDemos: only one scripted force-drag mode is allowed\n";
        return 1;
      }
      auto parsed = parseScriptedPointerForceDrag(argv[i + 1], std::cerr);
      if (!parsed.has_value()) {
        return 1;
      }
      if (!scriptedEventLogPath.empty()) {
        parsed->eventLogPath = scriptedEventLogPath;
      }
      scriptedForceDrag = std::move(parsed);
      ++i;
      continue;
    }
    if (arg == "--scripted-demo-event-log") {
      if (i + 1 >= argc || argv[i + 1] == nullptr) {
        std::cerr
            << "runDemos: --scripted-demo-event-log requires an argument\n";
        return 1;
      }
      scriptedEventLogPath = argv[i + 1];
      if (scriptedDemoSwitch.has_value()) {
        scriptedDemoSwitch->eventLogPath = scriptedEventLogPath;
      }
      if (scriptedForceDrag.has_value()) {
        scriptedForceDrag->eventLogPath = scriptedEventLogPath;
      }
      ++i;
      continue;
    }
    if (arg == "--scene") {
      if (i + 1 < argc && argv[i + 1] != nullptr) {
        initialId = argv[i + 1];
        ++i;
      }
      continue;
    }
    filteredArguments.push_back(argv[i]);
  }

  int index = 0;
  if (!initialId.empty()) {
    index = ::demoSceneIndex(scenes, initialId, -1);
    if (index < 0) {
      // Don't fall back to the first scene: an unknown id is almost always a
      // typo, and a silent fallback makes headless screenshot/cycle runs render
      // (and exit 0 on) the wrong scene -- the exact case this guard prevents.
      // Fail loudly with a nonzero status instead.
      std::cerr << "runDemos: unknown --scene '" << initialId
                << "'. Available scenes:";
      for (const auto& scene : scenes) {
        std::cerr << ' ' << scene.id;
      }
      std::cerr << '\n';
      return 1;
    }
  }

  if (!scriptedEventLogPath.empty() && !scriptedDemoSwitch.has_value()
      && !scriptedForceDrag.has_value()) {
    std::cerr << "runDemos: --scripted-demo-event-log requires "
                 "--scripted-demo-switch, --scripted-force-drag, or "
                 "--scripted-pointer-force-drag\n";
    return 1;
  }
  if (scriptedDemoSwitch.has_value() && cycleScenes) {
    std::cerr << "runDemos: --scripted-demo-switch cannot be combined with "
                 "--cycle-scenes\n";
    return 1;
  }
  if (scriptedDemoSwitch.has_value() && !scriptedDemoSwitch->sceneId.empty()) {
    const int scriptedIndex
        = ::demoSceneIndex(scenes, scriptedDemoSwitch->sceneId, -1);
    if (scriptedIndex < 0) {
      std::cerr << "runDemos: unknown scripted demo target '"
                << scriptedDemoSwitch->sceneId << "'. Available scenes:";
      for (const auto& scene : scenes) {
        std::cerr << ' ' << scene.id;
      }
      std::cerr << '\n';
      return 1;
    }
    scriptedDemoSwitch->sceneId
        = scenes[static_cast<std::size_t>(scriptedIndex)].id;
    if (!scriptedDemoSwitch->eventLogPath.empty()) {
      std::error_code error;
      const auto parent = scriptedDemoSwitch->eventLogPath.parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent, error);
      }
      std::ofstream eventLog(scriptedDemoSwitch->eventLogPath);
      if (!eventLog) {
        std::cerr << "runDemos: failed to create scripted demo event log '"
                  << scriptedDemoSwitch->eventLogPath.string() << "'\n";
        return 1;
      }
    }
  }
  if (scriptedForceDrag.has_value()
      && !scriptedForceDrag->eventLogPath.empty()) {
    std::error_code error;
    const auto path = std::filesystem::path(scriptedForceDrag->eventLogPath);
    const auto parent = path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, error);
    }
    std::ofstream eventLog(path);
    if (!eventLog) {
      std::cerr << "runDemos: failed to create scripted force-drag event log '"
                << scriptedForceDrag->eventLogPath << "'\n";
      return 1;
    }
  }

  return runGuiBackendApplicationImpl(
      static_cast<int>(filteredArguments.size()),
      filteredArguments.data(),
      ApplicationOptions{},
      &scenes,
      index,
      cycleScenes,
      cycleFramesPerScene,
      std::move(scriptedDemoSwitch),
      std::move(scriptedForceDrag));
}

} // namespace dart::gui

namespace dart::gui::detail {

int runGuiBackendApplication(int argc, char* argv[])
{
  return runGuiBackendApplication(argc, argv, dart::gui::ApplicationOptions{});
}

int runGuiBackendApplication(
    int argc,
    char* argv[],
    const dart::gui::ApplicationOptions& applicationOptions)
{
  return runGuiBackendApplicationImpl(
      argc, argv, applicationOptions, nullptr, 0, false);
}

} // namespace dart::gui::detail

namespace dart::gui::detail {

int runGuiApplication(int argc, char* argv[])
{
  return dart::gui::runApplication(argc, argv);
}

} // namespace dart::gui::detail
