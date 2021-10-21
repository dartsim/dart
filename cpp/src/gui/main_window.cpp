/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/main_window.hpp"

#include <unordered_map>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/gui/widget.hpp"
#include "dart/rendering/camera.hpp"
#include "dart/rendering/scene.hpp"
#include "dart/simulation/world.hpp"

namespace dart::gui {

//==============================================================================
struct MainWindow::Implementation
{
  gui::WidgetPtr main_widget{nullptr};

  // osg::ref_ptr<osgViewer::CompositeViewer> osg_composite_viewer;

  std::unordered_map<std::string, gui::WidgetPtr> widgets;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
MainWindow::MainWindow(const gui::WindowConfig& config)
  : Window(config), m_impl(std::make_unique<Implementation>())
{
  // Set up Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  // Enable docking
  auto& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  // Set up platform/renderer backends for ImGui
  ImGui_ImplGlfw_InitForOpenGL(get_mutable_glfw_window(), true);
  ImGui_ImplOpenGL3_Init(get_glsl_version().c_str());
}

//==============================================================================
MainWindow::~MainWindow()
{
  // Cleanup ImGui resources
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

//==============================================================================
void MainWindow::update()
{
  make_opengl_context_current();

  // Poll and handle events (inputs, window resize, etc)
  poll_events();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Docking setup
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
  ImGui::Begin(
      "##dockspace",
      nullptr,
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse
          | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
          | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_MenuBar
          | ImGuiWindowFlags_NoDocking);
  ImGui::DockSpace(
      ImGui::GetID("##dockspace_id"),
      {0, 0},
      ImGuiDockNodeFlags_PassthruCentralNode);
  ImGui::SetWindowPos({0, 0});
  ImGui::SetWindowSize(ImGui::GetIO().DisplaySize);
  ImGui::End();
  ImGui::PopStyleVar(3);

  // Draw widgets
  for (auto& it_widget : m_impl->widgets) {
    gui::WidgetPtr widget = it_widget.second;
    widget->draw();
  }

  // ImGui rendering
  ImGui::Render();
  int w;
  int h;
  glfwGetFramebufferSize(get_mutable_glfw_window(), &w, &h);
  glViewport(0, 0, w, h);
  glClearColor(0.5, 0.5, 0.5, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  swap_buffers();
}

//==============================================================================
void MainWindow::set_main_widget(gui::WidgetPtr widget)
{
  m_impl->main_widget = std::move(widget);
}

//==============================================================================
void MainWindow::add_widget(WidgetPtr widget)
{
  m_impl->widgets[widget->get_title()] = widget;
}

} // namespace dart::gui
