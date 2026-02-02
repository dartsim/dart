#include "dart/gui/raylib/imgui_raylib_bridge.hpp"

#if __has_include(<imgui.h>)
  #include <imgui.h>
#elif __has_include(<imgui/imgui.h>)
  #include <imgui/imgui.h>
#else
  #error "Could not find imgui.h - check your ImGui installation"
#endif

#if __has_include(<imgui_impl_opengl3.h>)
  #include <imgui_impl_opengl3.h>
#elif __has_include(<backends/imgui_impl_opengl3.h>)
  #include <backends/imgui_impl_opengl3.h>
#endif

#if __has_include(<imgui_impl_glfw.h>)
  #include <imgui_impl_glfw.h>
#elif __has_include(<backends/imgui_impl_glfw.h>)
  #include <backends/imgui_impl_glfw.h>
#endif

#include <GLFW/glfw3.h>

#include <algorithm>

namespace dart {
namespace gui {

void ImGuiRaylibBridge::initialize(GLFWwindow* window)
{
  if (initialized_) {
    return;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui_ImplGlfw_InitForOpenGL(window, false);
  ImGui_ImplOpenGL3_Init("#version 330");

  initialized_ = true;
}

void ImGuiRaylibBridge::shutdown()
{
  if (!initialized_) {
    return;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  initialized_ = false;
}

void ImGuiRaylibBridge::newFrame()
{
  if (!initialized_) {
    return;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void ImGuiRaylibBridge::render()
{
  if (!initialized_) {
    return;
  }

  for (const auto& widget : widgets_) {
    if (widget && widget->isVisible()) {
      widget->render();
    }
  }

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool ImGuiRaylibBridge::wantCaptureMouse() const
{
  return initialized_ && ImGui::GetIO().WantCaptureMouse;
}

bool ImGuiRaylibBridge::wantCaptureKeyboard() const
{
  return initialized_ && ImGui::GetIO().WantCaptureKeyboard;
}

void ImGuiRaylibBridge::addWidget(std::shared_ptr<ImGuiWidget> widget)
{
  if (!widget) {
    return;
  }

  widgets_.push_back(std::move(widget));
}

void ImGuiRaylibBridge::removeWidget(const std::shared_ptr<ImGuiWidget>& widget)
{
  const auto new_end = std::remove(widgets_.begin(), widgets_.end(), widget);
  widgets_.erase(new_end, widgets_.end());
}

} // namespace gui
} // namespace dart
