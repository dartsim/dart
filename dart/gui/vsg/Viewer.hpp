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

#pragma once

#include <dart/gui/vsg/Export.hpp>
#include <dart/gui/vsg/WorldNode.hpp>

#include <vsg/all.h>

#include <memory>

namespace dart::gui::vsg {

/// Lightweight VulkanSceneGraph viewer wired to a DART World.
class DART_GUI_VSG_API Viewer
{
public:
  explicit Viewer(
      std::shared_ptr<dart::simulation::World> world,
      ::vsg::ref_ptr<::vsg::WindowTraits> traits = {});
  ~Viewer();

  bool step();
  void run();

  void simulate(bool on);
  bool isSimulating() const;
  void setNumStepsPerCycle(std::size_t steps);

  ::vsg::ref_ptr<::vsg::Viewer> getViewer() const;
  ::vsg::ref_ptr<::vsg::Group> getScene() const;

private:
  void initialize(::vsg::ref_ptr<::vsg::WindowTraits> traits);
  ::vsg::ref_ptr<::vsg::Camera> createCamera(
      const ::vsg::ref_ptr<::vsg::Window>& window) const;

  std::shared_ptr<dart::simulation::World> mWorld;
  std::unique_ptr<WorldNode> mWorldNode;
  ::vsg::ref_ptr<::vsg::Viewer> mViewer;
  ::vsg::ref_ptr<::vsg::Group> mScene;
  bool mInitialized;
};

} // namespace dart::gui::vsg
