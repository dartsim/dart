/*
 * Copyright (c) 2011-2022, The DART development contributors:
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
#include "dart/gui/osg_include.hpp"
#include "dart/gui/type.hpp"
#include "dart/simulation/world.hpp"

namespace dart::gui {

class DART_GUI_API Scene
{
public:
  template <typename... Args>
  static std::shared_ptr<Scene> Create(Args&&... args)
  {
    return std::make_shared<Scene>(std::forward<Args>(args)...);
  }

  Scene();

  ~Scene();

  bool set_world(std::shared_ptr<simulation::World> world);

  [[nodiscard]] CameraPtr create_camera();

  [[nodiscard]] osg::ref_ptr<const osg::Group> get_osg_root_node() const;

  [[nodiscard]] osg::ref_ptr<osg::Group> get_mutable_osg_root_node();

protected:
private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

} // namespace dart::gui
