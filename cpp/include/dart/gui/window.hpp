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

#pragma once

#include <memory>
#include <string>

#include "dart/gui/export.hpp"
#include "dart/gui/glfw_include.hpp"
#include "dart/math/type.hpp"

namespace dart::gui {

struct WindowConfig
{
  std::string title = "notitle";
  int width = 800;
  int height = 600;
  bool offscreen = false;
};

class DART_GUI_API Window
{
public:
  template <typename... Args>
  static std::shared_ptr<Window> Create(Args&&... args)
  {
    return std::make_shared<Window>(std::forward<Args>(args)...);
  }

  explicit Window(const WindowConfig& config = WindowConfig());
  virtual ~Window();

  void make_opengl_context_current();
  bool should_close() const;
  void poll_events();
  void swap_buffers();

  math::Vector2i get_size() const;

  virtual void on_position_changed(int xpos, int ypos);
  virtual void on_size_changed(int width, int height);
  virtual void on_refresh();

  std::string get_glsl_version() const;

  GLFWwindow* get_mutable_glfw_window();

private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

} // namespace dart::gui
