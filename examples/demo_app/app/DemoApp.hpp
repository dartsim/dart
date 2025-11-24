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

#ifndef DART_EXAMPLES_DEMO_APP_DEMOAPP_HPP_
#define DART_EXAMPLES_DEMO_APP_DEMOAPP_HPP_

#include "core/Scene.hpp"

#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/ImGuiWidget.hpp>

#include <dart/common/Filesystem.hpp>

#include <memory>
#include <string>

namespace dart::demo {

class DemoApp
{
public:
  explicit DemoApp(SceneList scenes);

  int run();

  const SceneList& scenes() const;
  std::size_t activeSceneIndex() const;
  bool hasActiveScene() const;

  void activateScene(std::size_t index);

  void play();
  void pause();
  void togglePlayback();
  bool isPlaying() const;

  void stepOnce();
  void resetScene();

  double playbackRate() const;
  void setPlaybackRate(double rate);

  double simulationTime() const;
  double lastRealTimeFactor() const;

  bool isRecording() const;
  const std::string& recordingDirectory() const;
  void captureFrame();
  void toggleRecording();

  dart::gui::osg::ImGuiViewer& viewer();

private:
  void attachScene(const ScenePtr& scene);
  void stopRecording();

  SceneList mScenes;
  std::size_t mActiveScene{0};
  dart::gui::osg::ImGuiViewer mViewer;
  osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> mActiveNode;
  std::shared_ptr<dart::gui::osg::ImGuiWidget> mControlWidget;
  double mPlaybackRate{1.0};
  bool mRecording{false};
  std::string mRecordingDir;
  dart::common::filesystem::path mCaptureRoot;
};

} // namespace dart::demo

#endif // DART_EXAMPLES_DEMO_APP_DEMOAPP_HPP_
