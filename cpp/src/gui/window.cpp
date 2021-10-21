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

#include "dart/gui/window.hpp"

#include <unordered_map>

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"

#if __APPLE__
  // GL 3.2 + GLSL 150
  #define GLSL_VERSION "#version 150"
  #define OPENGL_MAJOR_VERSION 3
  #define OPENGL_MINOR_VERSION 2
#else
  // GL 3.0 + GLSL 130
  #define GLSL_VERSION "#version 130"
  #define OPENGL_MAJOR_VERSION 3
  #define OPENGL_MINOR_VERSION 2
#endif

namespace dart::gui {

//==============================================================================
struct Window::Implementation
{
  GLFWwindow* glfw_window{nullptr};
  static std::unordered_map<GLFWwindow*, Window*> s_windows_map;

  Implementation()
  {
    // Do nothing
  }

  bool create_glfw_window(const WindowConfig& config, Window* window)
  {
    if (s_windows_map.empty()) {
      glfwSetErrorCallback([](int error, const char* description) {
        DART_ERROR("GLFW error [{}]: {}", error, description);
      });

      if (!glfwInit()) {
        DART_ERROR("Failed to initialize GLFW.");
        return false;
      }
    }

    // Decide GL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on Mac
#else
    // GL 3.0
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    glfw_window = glfwCreateWindow(
        config.width,
        config.height,
        config.title.c_str(),
        /* monitor = */ nullptr,
        /* parent_window = */ nullptr);
    if (!glfw_window) {
      DART_ERROR("Failed to create a GLFW window.");
      return false;
    }

    glfwMakeContextCurrent(glfw_window);
    glfwSwapInterval(1); // Enable vsync

    // Initialize GLEW
    const GLenum glew_error = glewInit();
    if (glew_error != GLEW_OK) {
      DART_ERROR(
          "Failed to initialize OpenGL loader: {}",
          glewGetErrorString(glew_error));
      return false;
    }

    s_windows_map[glfw_window] = window;

    return true;
  }

  static Window* find_window(GLFWwindow* glfw_window)
  {
    auto result = s_windows_map.find(glfw_window);
    if (result != s_windows_map.end()) {
      return result->second;
    } else {
      return nullptr;
    }
  }

  void bind_callbacks()
  {
    DART_ASSERT(glfw_window);

    // Set window position callback
    glfwSetWindowPosCallback(
        glfw_window, [](GLFWwindow* glfw_window, int xpos, int ypos) {
          if (auto window = find_window(glfw_window)) {
            window->on_position_changed(xpos, ypos);
          }
        });

    // Set window size callback
    glfwSetWindowSizeCallback(
        glfw_window, [](GLFWwindow* glfw_window, int xpos, int ypos) {
          if (auto window = find_window(glfw_window)) {
            window->on_size_changed(xpos, ypos);
          }
        });

    glfwSetWindowRefreshCallback(glfw_window, [](GLFWwindow* glfw_window) {
      if (auto window = find_window(glfw_window)) {
        window->on_refresh();
      }
    });
  }
};

//==============================================================================
std::unordered_map<GLFWwindow*, Window*> Window::Implementation::s_windows_map;

//==============================================================================
Window::Window(const WindowConfig& config)
  : m_impl(std::make_unique<Implementation>())
{
  if (!m_impl->create_glfw_window(config, this)) {
    DART_ERROR("Failed to create a GLFW window.");
    return;
  }

  m_impl->bind_callbacks();
}

//==============================================================================
Window::~Window()
{
  if (m_impl->glfw_window) {
    glfwDestroyWindow(m_impl->glfw_window);
    m_impl->s_windows_map.erase(m_impl->glfw_window);
  }

  if (m_impl->s_windows_map.empty()) {
    glfwTerminate();
  }
}

//==============================================================================
void Window::make_opengl_context_current()
{
  glfwMakeContextCurrent(m_impl->glfw_window);
}

//==============================================================================
bool Window::should_close() const
{
  if (!m_impl->glfw_window) {
    return true;
  }

  return glfwWindowShouldClose(m_impl->glfw_window);
}

//==============================================================================
void Window::poll_events()
{
  DART_ASSERT(m_impl->glfw_window != nullptr);
  glfwWaitEvents();
  // glfwPollEvents();
}

//==============================================================================
void Window::swap_buffers()
{
  glfwSwapBuffers(m_impl->glfw_window);
}

//==============================================================================
math::Vector2i Window::get_size() const
{
  math::Vector2i size;
  glfwGetWindowSize(m_impl->glfw_window, &size[0], &size[1]);
  return size;
}

//==============================================================================
void Window::on_position_changed(int /*xpos*/, int /*ypos*/)
{
  // Do nothing
}

//==============================================================================
void Window::on_size_changed(int /*width*/, int /*height*/)
{
  // Do nothing
}

//==============================================================================
void Window::on_refresh()
{
  // Do nothing
}

//==============================================================================
std::string Window::get_glsl_version() const
{
#if defined(IMGUI_IMPL_OPENGL_ES2)
  return "#version 100";
#elif defined(__APPLE__)
  return "#version 150";
#else
  return "#version 130";
#endif
}

//==============================================================================
GLFWwindow* Window::get_mutable_glfw_window()
{
  return m_impl->glfw_window;
}

} // namespace dart::gui
