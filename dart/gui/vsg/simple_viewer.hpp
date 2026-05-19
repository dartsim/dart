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

#pragma once

#include <dart/gui/vsg/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vsg/all.h>

#include <memory>
#include <string>

namespace dart::gui::vsg {

class DART_GUI_VSG_API SimpleViewer
{
public:
  struct HeadlessTag
  {
  };

  SimpleViewer(
      int width = 800,
      int height = 600,
      const std::string& title = "DART VSG Viewer");

  SimpleViewer(HeadlessTag, int width = 800, int height = 600);

  static SimpleViewer headless(int width = 800, int height = 600);

  ~SimpleViewer();

  SimpleViewer(const SimpleViewer&) = delete;
  SimpleViewer& operator=(const SimpleViewer&) = delete;
  SimpleViewer(SimpleViewer&&) = default;
  SimpleViewer& operator=(SimpleViewer&&) = default;

  [[nodiscard]] bool isHeadless() const;

  void setScene(::vsg::ref_ptr<::vsg::Node> scene);
  [[nodiscard]] ::vsg::ref_ptr<::vsg::Group> getRoot() const;

  void addNode(::vsg::ref_ptr<::vsg::Node> node);
  void removeNode(::vsg::ref_ptr<::vsg::Node> node);
  void clear();

  void lookAt(
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ());
  void resetCamera();

  void setBackgroundColor(const Eigen::Vector4d& color);

  bool frame();
  void run();
  [[nodiscard]] bool shouldClose() const;

  void addGrid(double size = 10.0, double spacing = 1.0);
  void addAxes(double length = 1.0);

  bool saveScreenshot(const std::string& filename);

  std::vector<uint8_t> captureBuffer();

private:
  void setupWindowedViewer();
  void setupHeadlessViewer();
  void setupCamera();
  void compile();

  int m_width;
  int m_height;
  std::string m_title;
  bool m_headless{false};
  Eigen::Vector4d m_backgroundColor{0.2, 0.2, 0.3, 1.0};

  ::vsg::ref_ptr<::vsg::Viewer> m_viewer;
  ::vsg::ref_ptr<::vsg::Window> m_window;
  ::vsg::ref_ptr<::vsg::Group> m_root;
  ::vsg::ref_ptr<::vsg::Group> m_sceneRoot;
  ::vsg::ref_ptr<::vsg::Camera> m_camera;
  ::vsg::ref_ptr<::vsg::LookAt> m_lookAt;

  ::vsg::ref_ptr<::vsg::Device> m_device;
  ::vsg::ref_ptr<::vsg::Framebuffer> m_framebuffer;
  ::vsg::ref_ptr<::vsg::ImageView> m_colorImageView;
  ::vsg::ref_ptr<::vsg::ImageView> m_depthImageView;
  ::vsg::ref_ptr<::vsg::CommandGraph> m_commandGraph;
  int m_queueFamily{-1};

  bool m_needsCompile{true};
};

} // namespace dart::gui::vsg
