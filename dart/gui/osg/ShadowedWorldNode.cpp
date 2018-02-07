/*
 * Copyright (c) 2011-2017, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>

#include "dart/gui/osg/ShadowedWorldNode.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
ShadowedWorldNode::ShadowedWorldNode(std::shared_ptr<dart::simulation::World> _world)
  : WorldNode(_world) {}

//==============================================================================
void ShadowedWorldNode::setupViewer()
{
  constexpr int ReceivesShadowTraversalMask = 0x2;
  constexpr int CastsShadowTraversalMask = 0x1;

  // Setup shadows
  // Create the ShadowedScene
  ::osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;
  shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
  // Use the ShadowMap technique
  ::osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  // increase the resolution of default shadow texture for higher quality
  int mapres = std::pow(2, 13);
  sm->setTextureSize(::osg::Vec2s(mapres,mapres));

  // set the technique
  shadowedScene->setShadowTechnique(sm.get());

  // add the Viewer's root object to the shadowed scene
  shadowedScene->addChild(mViewer->getRootGroup().get());

  // save the shadowed scene
  mShadowedScene = shadowedScene;
  // get camera's home position
  // it is resetted when changing the SceneData, which can lead to undesired behavior
  ::osg::Vec3d eye, center, up;
  mViewer->getCameraManipulator()->getHomePosition(eye, center, up);
  // replace the viewer's scene data with the shadowed scene
  mViewer->setSceneData(mShadowedScene.get());
  // reset the home position
  mViewer->getCameraManipulator()->setHomePosition(eye, center, up);
  mViewer->home();
}

} // namespace osg
} // namespace gui
} // namespace dart