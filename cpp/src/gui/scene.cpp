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

#include "dart/gui/scene.hpp"

#include <unordered_set>

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/gui/camera.hpp"

namespace dart::gui {

//==============================================================================
struct Scene::Implementation
{
  std::shared_ptr<simulation::World> world;

  std::unordered_set<CameraPtr> cameras;

  osg::ref_ptr<osg::Group> osg_root_node;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
Scene::Scene() : m_impl(std::make_unique<Implementation>())
{
  // Create an empty world
  m_impl->world = simulation::World::Create();

  // Create OSG root node
  m_impl->osg_root_node = new osg::Group();
}

//==============================================================================
Scene::~Scene()
{
  // Do nothing
}

//==============================================================================
bool Scene::set_world(std::shared_ptr<simulation::World> world)
{
  if (world == nullptr) {
    return false;
  }

  m_impl->world = std::move(world);
  return true;
}

//==============================================================================
CameraPtr Scene::create_camera()
{
  auto camera = std::shared_ptr<Camera>(new Camera(this));
  m_impl->cameras.insert(camera);
  return camera;
}

//==============================================================================
osg::ref_ptr<const osg::Group> Scene::get_osg_root_node() const
{
  return m_impl->osg_root_node;
}

//==============================================================================
osg::ref_ptr<osg::Group> Scene::get_mutable_osg_root_node()
{
  return m_impl->osg_root_node;
}

} // namespace dart::gui
