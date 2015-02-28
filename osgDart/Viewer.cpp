/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "osgDart/Viewer.h"
#include "osgDart/TrackballManipulator.h"
#include "osgDart/DefaultEventHandler.h"
#include "osgDart/WorldNode.h"
#include "osgDart/utils.h"

#include "dart/simulation/World.h"

namespace osgDart
{

Viewer::Viewer(const osg::Vec4& clearColor)
  : mRootGroup(new osg::Group),
    mLightGroup(new osg::Group),
    mLight1(new osg::Light),
    mLightSource1(new osg::LightSource),
    mLight2(new osg::Light),
    mLightSource2(new osg::LightSource),
    mUpwards(osg::Vec3(0,0,1)),
    mOver(osg::Vec3(0,1,0)),
    mSimulating(false),
    mHeadlights(true)
{
  setCameraManipulator(new osgDart::TrackballManipulator);
  addInstructionText("Left-click:   Interaction\n");
  addInstructionText("Right-click:  Rotate view\n");
  addInstructionText("Middle-click: Translate view\n");
  addInstructionText("Wheel Scroll: Zoom in/out\n");

  mDefaultEventHandler = new DefaultEventHandler(this);
  // ^ Cannot construct this in the initialization list, because its constructor calls member functions of this object

  setSceneData(mRootGroup);
  addEventHandler(mDefaultEventHandler);
  setupDefaultLights();
  getCamera()->setClearColor(clearColor);
}

//==============================================================================
Viewer::~Viewer()
{
  // Do nothing
}

//==============================================================================
void Viewer::switchDefaultEventHandler(bool _on)
{
  removeEventHandler(mDefaultEventHandler);
  if(_on)
    addEventHandler(mDefaultEventHandler);
}

//==============================================================================
void Viewer::switchHeadlights(bool _on)
{
  mHeadlights = _on;

  if(_on)
  {
    if(getLight())
    {
      getLight()->setAmbient(osg::Vec4(0.1,0.1,0.1,1.0));
      getLight()->setDiffuse(osg::Vec4(0.8,0.8,0.8,1.0));
      getLight()->setSpecular(osg::Vec4(1.0,1.0,1.0,1.0));
    }

    if(mLight1)
    {
      mLight1->setAmbient(osg::Vec4(0.0,0.0,0.0,1.0));
      mLight1->setDiffuse(osg::Vec4(0.0,0.0,0.0,1.0));
      mLight1->setSpecular(osg::Vec4(0.0,0.0,0.0,1.0));
    }

    if(mLight2)
    {
      mLight2->setAmbient(osg::Vec4(0.0,0.0,0.0,1.0));
      mLight2->setDiffuse(osg::Vec4(0.0,0.0,0.0,1.0));
      mLight2->setSpecular(osg::Vec4(0.0,0.0,0.0,1.0));
    }
  }
  else
  {
    if(getLight())
    {
      getLight()->setAmbient(osg::Vec4(0.1,0.1,0.1,1.0));
      getLight()->setDiffuse(osg::Vec4(0.0,0.0,0.0,1.0));
      getLight()->setSpecular(osg::Vec4(0.0,0.0,0.0,1.0));
    }

    if(mLight1)
    {
      mLight1->setAmbient(osg::Vec4(0.0,0.0,0.0,1.0));
      mLight1->setDiffuse(osg::Vec4(0.7,0.7,0.7,1.0));
      mLight1->setSpecular(osg::Vec4(0.9,0.9,0.9,1.0));
    }

    if(mLight2)
    {
      mLight2->setAmbient(osg::Vec4(0.0,0.0,0.0,1.0));
      mLight2->setDiffuse(osg::Vec4(0.3,0.3,0.3,1.0));
      mLight2->setSpecular(osg::Vec4(0.4,0.4,0.4,1.0));
    }
  }
}

//==============================================================================
bool Viewer::checkHeadlights() const
{
  return mHeadlights;
}

//==============================================================================
void Viewer::addWorldNode(WorldNode* _newWorldNode, bool _active)
{
  if(mWorldNodes.find(_newWorldNode) != mWorldNodes.end())
    return;

  mWorldNodes[_newWorldNode] = _active;
  mRootGroup->addChild(_newWorldNode);
  if(_active)
    _newWorldNode->simulate(mSimulating);
}

//==============================================================================
void Viewer::removeWorldNode(WorldNode* _oldWorldNode)
{
  std::map<WorldNode*,bool>::iterator it = mWorldNodes.find(_oldWorldNode);
  if(it == mWorldNodes.end())
    return;

  mRootGroup->removeChild(it->first);
  mWorldNodes.erase(it);
}

//==============================================================================
void Viewer::removeWorldNode(dart::simulation::World* _oldWorld)
{
  WorldNode* node = getWorldNode(_oldWorld);

  if(nullptr == node)
    return;

  mRootGroup->removeChild(node);
  mWorldNodes.erase(node);
}

//==============================================================================
WorldNode* Viewer::getWorldNode(dart::simulation::World* _world) const
{
  std::map<WorldNode*,bool>::const_iterator it = mWorldNodes.begin(),
                                            end = mWorldNodes.end();
  WorldNode* node = nullptr;
  for( ; it != end; ++it)
  {
    WorldNode* checkNode = it->first;
    if(checkNode->getWorld() == _world)
    {
      node = checkNode;
      break;
    }
  }

  return node;
}

//==============================================================================
osg::Group* Viewer::getLightGroup()
{
  return mLightGroup;
}

//==============================================================================
const osg::Group* Viewer::getLightGroup() const
{
  return mLightGroup;
}

//==============================================================================
void Viewer::setupDefaultLights()
{
  setUpwardsDirection(mUpwards);
  switchHeadlights(true);

  osg::ref_ptr<osg::StateSet> lightSS = mRootGroup->getOrCreateStateSet();

  mLight1->setLightNum(1);
  mLightSource1->setLight(mLight1);
  mLightSource1->setLocalStateSetModes(osg::StateAttribute::ON);
  mLightSource1->setStateSetModes(*lightSS, osg::StateAttribute::ON);
  mLightGroup->removeChild(mLightSource1); // Make sure the LightSource is not already present
  mLightGroup->addChild(mLightSource1);

  mLight2->setLightNum(2);
  mLightSource2->setLight(mLight2);
  mLightSource2->setLocalStateSetModes(osg::StateAttribute::ON);
  mLightSource2->setStateSetModes(*lightSS, osg::StateAttribute::ON);
  mLightGroup->removeChild(mLightSource2);
  mLightGroup->addChild(mLightSource2);

  mRootGroup->removeChild(mLightGroup);
  mRootGroup->addChild(mLightGroup);
}

//==============================================================================
void Viewer::setUpwardsDirection(const osg::Vec3& _up)
{
  mUpwards = _up;
  if(mUpwards.length() > 0)
    mUpwards.normalize();
  else
    mUpwards = osg::Vec3(0,0,1);

  mOver = _up^osg::Vec3(1,0,0);
  if(mOver.length() < 1e-12)
    mOver = osg::Vec3(0,0,1)^_up;
  mOver.normalize();

  osg::Vec3 p1 = mUpwards+mOver;
  mLight1->setPosition(osg::Vec4(p1[0], p1[1], p1[2], 0.0));
  osg::Vec3 p2 = mUpwards-mOver;
  mLight2->setPosition(osg::Vec4(p2[0], p2[1], p2[2], 0.0));
}

//==============================================================================
void Viewer::setUpwardsDirection(const Eigen::Vector3d& _up)
{
  setUpwardsDirection(eigToOsgVec3(_up));
}

//==============================================================================
void Viewer::setWorldNodeActive(WorldNode* _node, bool _active)
{
  std::map<WorldNode*,bool>::iterator it = mWorldNodes.find(_node);
  if(it == mWorldNodes.end())
    return;

  it->second = _active;
}

//==============================================================================
void Viewer::setWorldNodeActive(dart::simulation::World* _world, bool _active)
{
  setWorldNodeActive(getWorldNode(_world), _active);
}

//==============================================================================
void Viewer::simulate(bool _on)
{
  mSimulating = _on;
  for( auto& node_pair : mWorldNodes )
  {
    if(node_pair.second)
    {
      node_pair.first->simulate(_on);
    }
  }
}

//==============================================================================
bool Viewer::isSimulating() const
{
  return mSimulating;
}

//==============================================================================
const std::string& Viewer::getInstructions() const
{
  return mInstructions;
}

//==============================================================================
void Viewer::addInstructionText(const std::string& _instruction)
{
  mInstructions.append(_instruction);
}

} // namespace osgDart
