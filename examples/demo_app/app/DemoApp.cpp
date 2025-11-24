/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "app/DemoApp.hpp"

#include <dart/gui/osg/IncludeImGui.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include <ctime>

namespace dart::demo {

namespace {

std::string formatTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);

  std::tm tm = *std::localtime(&time);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

class ControlWidget : public dart::gui::osg::ImGuiWidget
{
public:
  explicit ControlWidget(DemoApp& app) : map(app) {}

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380.0f, 640.0f), ImGuiCond_FirstUseEver);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;

    if (!ImGui::Begin("DART Demo Browser", nullptr, flags)) {
      ImGui::End();
      return;
    }

    renderScenes();
    renderPlayback();
    renderCapture();
    renderSceneDetails();

    ImGui::End();
  }

private:
  void renderScenes()
  {
    if (!ImGui::CollapsingHeader("Scenes", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    if (map.scenes().empty()) {
      ImGui::TextUnformatted("No scenes registered");
      return;
    }

    for (std::size_t i = 0; i < map.scenes().size(); ++i) {
      const auto& scene = map.scenes()[i];
      if (!scene)
        continue;

      const bool selected = (i == map.activeSceneIndex());
      if (ImGui::Selectable(scene->metadata().name.c_str(), selected))
        map.activateScene(i);

      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(scene->metadata().description.c_str());
        if (!scene->metadata().tags.empty()) {
          ImGui::Separator();
          ImGui::TextUnformatted("Tags:");
          for (const auto& tag : scene->metadata().tags)
            ImGui::SameLine(), ImGui::TextUnformatted(tag.c_str());
        }
        ImGui::EndTooltip();
      }
    }
  }

  void renderPlayback()
  {
    if (!ImGui::CollapsingHeader("Playback", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    if (ImGui::Button(map.isPlaying() ? "Pause" : "Play"))
      map.togglePlayback();

    ImGui::SameLine();
    if (ImGui::Button("Step"))
      map.stepOnce();

    ImGui::SameLine();
    if (ImGui::Button("Reset"))
      map.resetScene();

    float rate = static_cast<float>(map.playbackRate());
    if (ImGui::SliderFloat("Playback speed", &rate, 0.25f, 4.0f, "%.2fx"))
      map.setPlaybackRate(static_cast<double>(rate));

    ImGui::Text("Sim time: %.2f s", map.simulationTime());
    ImGui::SameLine();
    ImGui::Text("RTF: %.2f", map.lastRealTimeFactor());
  }

  void renderCapture()
  {
    if (!ImGui::CollapsingHeader("Capture", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    if (ImGui::Button("Snapshot"))
      map.captureFrame();

    ImGui::SameLine();
    if (ImGui::Button(map.isRecording() ? "Stop recording" : "Record frames"))
      map.toggleRecording();

    if (map.isRecording()) {
      ImGui::Text("Recording to: %s", map.recordingDirectory().c_str());
    } else {
      ImGui::TextUnformatted(
          "Records a PNG sequence under captures/<scene>/frames");
    }
  }

  void renderSceneDetails()
  {
    if (!map.hasActiveScene())
      return;

    const auto& scene = map.scenes()[map.activeSceneIndex()];
    if (!scene)
      return;

    if (ImGui::CollapsingHeader(
            "Scene details", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::TextWrapped("%s", scene->metadata().description.c_str());
      if (!scene->metadata().tags.empty()) {
        ImGui::Spacing();
        ImGui::TextUnformatted("Tags:");
        for (const auto& tag : scene->metadata().tags) {
          ImGui::SameLine();
          ImGui::TextUnformatted(tag.c_str());
        }
        ImGui::NewLine();
      }
    }

    if (ImGui::CollapsingHeader(
            "Scene controls", ImGuiTreeNodeFlags_DefaultOpen))
      scene->renderImGui();
  }

  DemoApp& map;
};

} // namespace

DemoApp::DemoApp(SceneList scenes)
  : mScenes(std::move(scenes)),
    mCaptureRoot(dart::common::filesystem::path("captures"))
{
  mViewer.setUpViewInWindow(50, 50, 1280, 720);
}

int DemoApp::run()
{
  if (mScenes.empty()) {
    std::cerr << "No scenes registered. Exiting." << std::endl;
    return 1;
  }

  mControlWidget = std::make_shared<ControlWidget>(*this);
  if (auto* handler = mViewer.getImGuiHandler())
    handler->addWidget(mControlWidget);

  activateScene(0);
  play();

  mViewer.run();
  return 0;
}

const SceneList& DemoApp::scenes() const
{
  return mScenes;
}

std::size_t DemoApp::activeSceneIndex() const
{
  return mActiveScene;
}

bool DemoApp::hasActiveScene() const
{
  return mActiveScene < mScenes.size()
         && static_cast<bool>(mScenes[mActiveScene]);
}

void DemoApp::activateScene(std::size_t index)
{
  if (index >= mScenes.size() || !mScenes[index])
    return;

  const bool wasPlaying = isPlaying();

  stopRecording();

  if (mActiveNode)
    mViewer.removeWorldNode(mActiveNode.get());

  mActiveScene = index;
  attachScene(mScenes[index]);

  if (wasPlaying)
    play();
}

void DemoApp::attachScene(const ScenePtr& scene)
{
  if (!scene)
    return;

  scene->reset();
  mActiveNode = scene->node();
  if (!mActiveNode)
    return;

  mActiveNode->setTargetRealTimeFactor(mPlaybackRate);
  mViewer.addWorldNode(mActiveNode.get());

  if (auto* manipulator = mViewer.getCameraManipulator()) {
    const auto& camera = scene->metadata();
    manipulator->setHomePosition(camera.eye, camera.center, camera.up);
    mViewer.setCameraManipulator(manipulator);
    manipulator->home(0.0);
  }
}

void DemoApp::play()
{
  if (mActiveNode)
    mActiveNode->simulate(true);
}

void DemoApp::pause()
{
  if (mActiveNode)
    mActiveNode->simulate(false);
}

void DemoApp::togglePlayback()
{
  if (isPlaying())
    pause();
  else
    play();
}

bool DemoApp::isPlaying() const
{
  return mActiveNode && mActiveNode->isSimulating();
}

void DemoApp::stepOnce()
{
  if (!hasActiveScene())
    return;

  pause();
  mScenes[mActiveScene]->world()->step();
}

void DemoApp::resetScene()
{
  if (!hasActiveScene())
    return;

  pause();
  mScenes[mActiveScene]->reset();
}

double DemoApp::playbackRate() const
{
  return mPlaybackRate;
}

void DemoApp::setPlaybackRate(double rate)
{
  if (rate < 0.05)
    rate = 0.05;
  else if (rate > 8.0)
    rate = 8.0;

  mPlaybackRate = rate;
  if (mActiveNode)
    mActiveNode->setTargetRealTimeFactor(mPlaybackRate);
}

double DemoApp::simulationTime() const
{
  if (!hasActiveScene())
    return 0.0;

  return mScenes[mActiveScene]->world()->getTime();
}

double DemoApp::lastRealTimeFactor() const
{
  if (!mActiveNode)
    return 0.0;

  return mActiveNode->getLastRealTimeFactor();
}

bool DemoApp::isRecording() const
{
  return mRecording;
}

const std::string& DemoApp::recordingDirectory() const
{
  return mRecordingDir;
}

void DemoApp::captureFrame()
{
  if (!hasActiveScene())
    return;

  const auto& meta = mScenes[mActiveScene]->metadata();
  auto outputDir = mCaptureRoot / meta.key;
  dart::common::filesystem::create_directories(outputDir);

  const auto filename
      = outputDir
        / ("screenshot_" + meta.key + "_" + formatTimestamp() + ".png");
  mViewer.captureScreen(filename.string());
}

void DemoApp::toggleRecording()
{
  if (!hasActiveScene())
    return;

  if (mRecording) {
    stopRecording();
    return;
  }

  const auto& meta = mScenes[mActiveScene]->metadata();
  auto outputDir = mCaptureRoot / meta.key / "frames";
  dart::common::filesystem::create_directories(outputDir);

  mViewer.record(outputDir.string(), meta.key + "_", true, 6);
  mRecording = true;
  mRecordingDir = outputDir.string();
}

void DemoApp::stopRecording()
{
  if (!mRecording)
    return;

  mViewer.pauseRecording();
  mRecording = false;
  mRecordingDir.clear();
}

dart::gui::osg::ImGuiViewer& DemoApp::viewer()
{
  return mViewer;
}

} // namespace dart::demo
