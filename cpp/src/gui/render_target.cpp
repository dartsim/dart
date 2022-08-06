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

#include "dart/gui/render_target.hpp"

#include "dart/gui/osg_include.hpp"
#include "dart/gui/scene.hpp"

namespace dart::gui {

//==============================================================================
struct RenderTarget::Implementation
{
  /**
   * @brief OSG Viewer object
   */
  osg::ref_ptr<osgViewer::Viewer> osg_viewer{nullptr};

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
RenderTarget::RenderTarget() : m_impl(std::make_unique<Implementation>())
{
  // Do nothing
}

//==============================================================================
RenderTarget::~RenderTarget()
{
  // Do nothing
}

//==============================================================================
void RenderTarget::init()
{
  m_impl->osg_viewer = new osgViewer::Viewer();
  m_impl->osg_viewer->setThreadingModel(
      osgViewer::ViewerBase::ThreadingModel::SingleThreaded);
  m_impl->osg_viewer->setSceneData(
      get_mutable_scene()->get_mutable_osg_root_node());
}

} // namespace dart::gui
