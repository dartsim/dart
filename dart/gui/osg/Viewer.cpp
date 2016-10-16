/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <iomanip>

#include <osg/OperationThread>
#include <osgDB/WriteFile>

#include "dart/gui/osg/Viewer.hpp"
#include "dart/gui/osg/TrackballManipulator.hpp"
#include "dart/gui/osg/DefaultEventHandler.hpp"
#include "dart/gui/osg/DragAndDrop.hpp"
#include "dart/gui/osg/WorldNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/simulation/World.hpp"

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace gui {
namespace osg {

class SaveScreen : public ::osg::Camera::DrawCallback
{
public:

  SaveScreen(Viewer* viewer)
    : mViewer(viewer),
      mImage(new ::osg::Image),
      mCamera(mViewer->getCamera())
  {
    // Do nothing
  }

  virtual void operator () (::osg::RenderInfo& renderInfo) const
  {
    ::osg::Camera::DrawCallback::operator ()(renderInfo);

    if(mViewer->mRecording || mViewer->mScreenCapture)
    {
      int x, y;
      unsigned int width, height;
      ::osg::ref_ptr<::osg::Viewport> vp = mCamera->getViewport();
      x = vp->x();
      y = vp->y();
      width = vp->width();
      height = vp->height();

      mImage->readPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE);
    }

    if(mViewer->mRecording)
    {
      if(!mViewer->mImageDirectory.empty())
      {
        std::stringstream str;
        str << mViewer->mImageDirectory << "/" << mViewer->mImagePrefix
            << std::setfill('0') << std::setw(mViewer->mImageDigits)
            << mViewer->mImageSequenceNum << std::setw(0) << ".png";

        if(::osgDB::writeImageFile(*mImage, str.str()))
        {
          ++mViewer->mImageSequenceNum;
        }
        else
        {
          dtwarn << "[SaveScreen::record] Unable to save image to file named: "
                 << str.str() << "\n";

          // Toggle off recording if the file cannot be saved.
          mViewer->mRecording = false;
        }
      }
    }

    if(mViewer->mScreenCapture)
    {
      if(!mViewer->mScreenCapName.empty())
      {
        if(!::osgDB::writeImageFile(*mImage, mViewer->mScreenCapName))
          dtwarn << "[SaveScreen::capture] Unable to save image to file named: "
                 << mViewer->mScreenCapName << "\n";

        // Toggle off the screen capture after the image is grabbed (or the
        // attempt is made).
        mViewer->mScreenCapture = false;
      }
    }
  }

protected:

  Viewer* mViewer;

  ::osg::ref_ptr<::osg::Image> mImage;

  ::osg::ref_ptr<::osg::Camera> mCamera;
};

//==============================================================================
class ViewerAttachmentCallback : public ::osg::NodeCallback
{
public:

  virtual void operator()(::osg::Node* node, ::osg::NodeVisitor* nv)
  {
    ::osg::ref_ptr<ViewerAttachment> attachment =
        dynamic_cast<ViewerAttachment*>(node);

    if(attachment)
      attachment->refresh();

    traverse(node, nv);
  }

};

//==============================================================================
ViewerAttachment::ViewerAttachment()
  : mViewer(nullptr)
{
  setUpdateCallback(new ViewerAttachmentCallback);
}

//==============================================================================
ViewerAttachment::~ViewerAttachment()
{
  if(mViewer)
    mViewer->removeAttachment(this);
}

//==============================================================================
Viewer* ViewerAttachment::getViewer()
{
  return mViewer;
}

//==============================================================================
const Viewer* ViewerAttachment::getViewer() const
{
  return mViewer;
}

//==============================================================================
void ViewerAttachment::customAttach(Viewer* /*newViewer*/)
{
  // Do nothing
}

//==============================================================================
void ViewerAttachment::attach(Viewer* newViewer)
{
  if(mViewer)
    mViewer->getRootGroup()->removeChild(this);

  newViewer->getRootGroup()->addChild(this);
  customAttach(newViewer);
}

//==============================================================================
Viewer::Viewer(const ::osg::Vec4& clearColor)
  : mImageSequenceNum(0),
    mImageDigits(0),
    mRecording(false),
    mRootGroup(new ::osg::Group),
    mLightGroup(new ::osg::Group),
    mLight1(new ::osg::Light),
    mLightSource1(new ::osg::LightSource),
    mLight2(new ::osg::Light),
    mLightSource2(new ::osg::LightSource),
    mUpwards(::osg::Vec3(0,0,1)),
    mOver(::osg::Vec3(0,1,0)),
    mSimulating(false),
    mAllowSimulation(true),
    mHeadlights(true)
{
  setCameraManipulator(new osg::TrackballManipulator);
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

  getCamera()->setFinalDrawCallback(new SaveScreen(this));
}

//==============================================================================
Viewer::~Viewer()
{
  std::unordered_set<ViewerAttachment*>::iterator
      it = mAttachments.begin(),
      end = mAttachments.end();

  while( it != end )
    removeAttachment(*(it++));
}

//==============================================================================
void Viewer::captureScreen(const std::string& filename)
{
  if(filename.empty())
  {
    dtwarn << "[Viewer::captureScreen] Passed in empty filename for screen "
           << "capture. This is not allowed!\n";
    return;
  }

  dtmsg << "[Viewer::captureScreen] Saving image to file: "
        << filename << std::endl;

  mScreenCapName = filename;
  mScreenCapture = true;
}

//==============================================================================
void Viewer::record(const std::string& directory, const std::string& prefix,
                    bool restart, std::size_t digits)
{
  if(directory.empty())
  {
    dtwarn << "[Viewer::record] Passed in empty directory name for screen "
           << "recording. This is not allowed!\n";
    return;
  }

  mImageDirectory = directory;
  mImagePrefix = prefix;

  if(restart)
    mImageSequenceNum = 0;

  mImageDigits = digits;

  mRecording = true;

  dtmsg << "[Viewer::record] Recording screen image sequence to directory ["
        << mImageDirectory << "] with a prefix of [" << mImagePrefix << "]"
        << " starting from sequence number [" << mImageSequenceNum << "]"
        << std::endl;
}

//==============================================================================
void Viewer::pauseRecording()
{
  if(!mRecording)
    return;

  mRecording = false;
  dtmsg<< "[Viewer::pauseRecording] Screen recording is paused at image "
       << "sequence number [" << mImageSequenceNum << "]" << std::endl;
}

//==============================================================================
bool Viewer::isRecording() const
{
  return mRecording;
}

//==============================================================================
void Viewer::switchDefaultEventHandler(bool _on)
{
  removeEventHandler(mDefaultEventHandler);
  if(_on)
    addEventHandler(mDefaultEventHandler);
}

//==============================================================================
DefaultEventHandler* Viewer::getDefaultEventHandler() const
{
  return mDefaultEventHandler;
}

//==============================================================================
void Viewer::switchHeadlights(bool _on)
{
  mHeadlights = _on;

  if(_on)
  {
    if(getLight())
    {
      getLight()->setAmbient(::osg::Vec4(0.1,0.1,0.1,1.0));
      getLight()->setDiffuse(::osg::Vec4(0.8,0.8,0.8,1.0));
      getLight()->setSpecular(::osg::Vec4(1.0,1.0,1.0,1.0));
    }

    if(mLight1)
    {
      mLight1->setAmbient(::osg::Vec4(0.0,0.0,0.0,1.0));
      mLight1->setDiffuse(::osg::Vec4(0.0,0.0,0.0,1.0));
      mLight1->setSpecular(::osg::Vec4(0.0,0.0,0.0,1.0));
    }

    if(mLight2)
    {
      mLight2->setAmbient(::osg::Vec4(0.0,0.0,0.0,1.0));
      mLight2->setDiffuse(::osg::Vec4(0.0,0.0,0.0,1.0));
      mLight2->setSpecular(::osg::Vec4(0.0,0.0,0.0,1.0));
    }
  }
  else
  {
    if(getLight())
    {
      getLight()->setAmbient(::osg::Vec4(0.1,0.1,0.1,1.0));
      getLight()->setDiffuse(::osg::Vec4(0.0,0.0,0.0,1.0));
      getLight()->setSpecular(::osg::Vec4(0.0,0.0,0.0,1.0));
    }

    if(mLight1)
    {
      mLight1->setAmbient(::osg::Vec4(0.0,0.0,0.0,1.0));
      mLight1->setDiffuse(::osg::Vec4(0.7,0.7,0.7,1.0));
      mLight1->setSpecular(::osg::Vec4(0.9,0.9,0.9,1.0));
    }

    if(mLight2)
    {
      mLight2->setAmbient(::osg::Vec4(0.0,0.0,0.0,1.0));
      mLight2->setDiffuse(::osg::Vec4(0.3,0.3,0.3,1.0));
      mLight2->setSpecular(::osg::Vec4(0.4,0.4,0.4,1.0));
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
  _newWorldNode->mViewer = this;
  _newWorldNode->setupViewer();
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
void Viewer::removeWorldNode(std::shared_ptr<dart::simulation::World> _oldWorld)
{
  WorldNode* node = getWorldNode(_oldWorld);

  if(nullptr == node)
    return;

  mRootGroup->removeChild(node);
  mWorldNodes.erase(node);
}

//==============================================================================
WorldNode* Viewer::getWorldNode(
    std::shared_ptr<dart::simulation::World> _world) const
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
void Viewer::addAttachment(ViewerAttachment* _attachment)
{
  Viewer* oldViewer = _attachment->mViewer;
  if(oldViewer)
    oldViewer->removeAttachment(_attachment);

  _attachment->mViewer = this;
  mAttachments.insert(_attachment);
  _attachment->attach(this);
}

//==============================================================================
void Viewer::removeAttachment(ViewerAttachment* _attachment)
{
  std::unordered_set<ViewerAttachment*>::iterator it =
      mAttachments.find(_attachment);

  if(it == mAttachments.end())
    return;

  _attachment->mViewer = nullptr;
  mAttachments.erase(_attachment);
}

//==============================================================================
const std::unordered_set<ViewerAttachment*>& Viewer::getAttachments() const
{
  return mAttachments;
}

//==============================================================================
::osg::Group* Viewer::getLightGroup()
{
  return mLightGroup;
}

//==============================================================================
const ::osg::Group* Viewer::getLightGroup() const
{
  return mLightGroup;
}

//==============================================================================
void Viewer::setupDefaultLights()
{
  setUpwardsDirection(mUpwards);
  switchHeadlights(true);

  ::osg::ref_ptr<::osg::StateSet> lightSS = mRootGroup->getOrCreateStateSet();

  mLight1->setLightNum(1);
  mLightSource1->setLight(mLight1);
  mLightSource1->setLocalStateSetModes(::osg::StateAttribute::ON);
  mLightSource1->setStateSetModes(*lightSS, ::osg::StateAttribute::ON);
  mLightGroup->removeChild(mLightSource1); // Make sure the LightSource is not already present
  mLightGroup->addChild(mLightSource1);

  mLight2->setLightNum(2);
  mLightSource2->setLight(mLight2);
  mLightSource2->setLocalStateSetModes(::osg::StateAttribute::ON);
  mLightSource2->setStateSetModes(*lightSS, ::osg::StateAttribute::ON);
  mLightGroup->removeChild(mLightSource2);
  mLightGroup->addChild(mLightSource2);

  mRootGroup->removeChild(mLightGroup);
  mRootGroup->addChild(mLightGroup);
}

//==============================================================================
void Viewer::setUpwardsDirection(const ::osg::Vec3& _up)
{
  mUpwards = _up;
  if(mUpwards.length() > 0)
    mUpwards.normalize();
  else
    mUpwards = ::osg::Vec3(0,0,1);

  mOver = _up^::osg::Vec3(1,0,0); // Note: operator^ is the cross product operator in OSG
  if(mOver.length() < 1e-12)
    mOver = ::osg::Vec3(0,0,1)^_up;
  mOver.normalize();

  ::osg::Vec3 p1 = mUpwards+mOver;
  mLight1->setPosition(::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
  ::osg::Vec3 p2 = mUpwards-mOver;
  mLight2->setPosition(::osg::Vec4(p2[0], p2[1], p2[2], 0.0));
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
void Viewer::setWorldNodeActive(std::shared_ptr<dart::simulation::World> _world,
                                bool _active)
{
  setWorldNodeActive(getWorldNode(_world), _active);
}

//==============================================================================
void Viewer::simulate(bool _on)
{
  if(!mAllowSimulation && _on)
    return;

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
void Viewer::allowSimulation(bool _allow)
{
  mAllowSimulation = _allow;

  if(!mAllowSimulation && mSimulating)
    simulate(false);
}

//==============================================================================
bool Viewer::isAllowingSimulation() const
{
  return mAllowSimulation;
}

//==============================================================================
DragAndDrop* Viewer::enableDragAndDrop(dart::dynamics::Entity* _entity)
{
  if(dart::dynamics::BodyNode* bn =
     dynamic_cast<dart::dynamics::BodyNode*>(_entity))
    return enableDragAndDrop(bn);

  if(dart::dynamics::SimpleFrame* sf =
     dynamic_cast<dart::dynamics::SimpleFrame*>(_entity))
    return enableDragAndDrop(sf);

  return nullptr;
}

//==============================================================================
SimpleFrameDnD* Viewer::enableDragAndDrop(dart::dynamics::SimpleFrame* _frame)
{
  if(nullptr == _frame)
    return nullptr;

  std::map<dart::dynamics::SimpleFrame*,SimpleFrameDnD*>::iterator it =
      mSimpleFrameDnDMap.find(_frame);
  if(it != mSimpleFrameDnDMap.end())
    return it->second;

  SimpleFrameDnD* dnd = new SimpleFrameDnD(this, _frame);
  mSimpleFrameDnDMap[_frame] = dnd;
  return dnd;
}

//==============================================================================
// Creating a typedef for a very long and ugly template
namespace sfs_dnd {
typedef std::multimap<dart::dynamics::Shape*,SimpleFrameShapeDnD*>::iterator iterator;
} // namespace sfs_dnd

//==============================================================================
static sfs_dnd::iterator getSimpleFrameShapeDnDFromMultimap(
    dart::dynamics::SimpleFrame* _frame, dart::dynamics::Shape* _shape,
    std::multimap<dart::dynamics::Shape*,SimpleFrameShapeDnD*>& map)
{
  using namespace sfs_dnd;

  std::pair<iterator,iterator> range = map.equal_range(_shape);
  iterator it = range.first, end = range.second;
  while(it != map.end())
  {
    SimpleFrameShapeDnD* dnd = it->second;
    if(dnd->getSimpleFrame() == _frame)
    {
      return it;
    }

    if(it == end)
      break;
    ++it;
  }

  return map.end();
}

//==============================================================================
SimpleFrameShapeDnD* Viewer::enableDragAndDrop(
    dart::dynamics::SimpleFrame* _frame, dart::dynamics::Shape* _shape)
{
  if(nullptr == _frame || nullptr == _shape)
    return nullptr;

  using namespace sfs_dnd;

  iterator existingDnD = getSimpleFrameShapeDnDFromMultimap(
        _frame, _shape, mSimpleFrameShapeDnDMap);
  if(existingDnD != mSimpleFrameShapeDnDMap.end())
    return existingDnD->second;

  SimpleFrameShapeDnD* dnd = new SimpleFrameShapeDnD(this, _frame, _shape);
  mSimpleFrameShapeDnDMap.insert(
        std::pair<dart::dynamics::Shape*,SimpleFrameShapeDnD*>(_shape,dnd));

  return dnd;
}

//==============================================================================
InteractiveFrameDnD* Viewer::enableDragAndDrop(
    dart::gui::osg::InteractiveFrame* _frame)
{
  if(nullptr == _frame)
    return nullptr;

  std::map<InteractiveFrame*,InteractiveFrameDnD*>::iterator it =
      mInteractiveFrameDnDMap.find(_frame);
  if(it != mInteractiveFrameDnDMap.end())
    return it->second;

  InteractiveFrameDnD* dnd = new InteractiveFrameDnD(this, _frame);
  mInteractiveFrameDnDMap[_frame] = dnd;
  return dnd;
}

//==============================================================================
BodyNodeDnD* Viewer::enableDragAndDrop(dart::dynamics::BodyNode* _bn,
                                       bool _useExternalIK, bool _useWholeBody)
{
  if(nullptr == _bn)
    return nullptr;

  std::map<dart::dynamics::BodyNode*,BodyNodeDnD*>::iterator it =
      mBodyNodeDnDMap.find(_bn);
  if(it != mBodyNodeDnDMap.end())
    return it->second;

  BodyNodeDnD* dnd = new BodyNodeDnD(this, _bn, _useExternalIK, _useWholeBody);
  mBodyNodeDnDMap[_bn] = dnd;
  return dnd;
}

//==============================================================================
bool Viewer::disableDragAndDrop(DragAndDrop* _dnd)
{
  if(disableDragAndDrop(dynamic_cast<SimpleFrameShapeDnD*>(_dnd)))
    return true;

  if(disableDragAndDrop(dynamic_cast<SimpleFrameDnD*>(_dnd)))
    return true;

  if(disableDragAndDrop(dynamic_cast<InteractiveFrameDnD*>(_dnd)))
    return true;

  if(disableDragAndDrop(dynamic_cast<BodyNodeDnD*>(_dnd)))
    return true;

  return false;
}

//==============================================================================
bool Viewer::disableDragAndDrop(SimpleFrameDnD* _dnd)
{
  if(nullptr == _dnd)
    return false;

  std::map<dart::dynamics::SimpleFrame*,SimpleFrameDnD*>::iterator it =
      mSimpleFrameDnDMap.find(_dnd->getSimpleFrame());
  if(it == mSimpleFrameDnDMap.end())
    return false;

  delete it->second;
  mSimpleFrameDnDMap.erase(it);

  return true;
}

//==============================================================================
bool Viewer::disableDragAndDrop(SimpleFrameShapeDnD* _dnd)
{
  if(nullptr == _dnd)
    return false;

  using namespace sfs_dnd;

  iterator it = getSimpleFrameShapeDnDFromMultimap(
        _dnd->getSimpleFrame(), _dnd->getShape(), mSimpleFrameShapeDnDMap);

  if(it == mSimpleFrameShapeDnDMap.end())
    return false;

  delete it->second;
  mSimpleFrameShapeDnDMap.erase(it);

  return true;
}

//==============================================================================
bool Viewer::disableDragAndDrop(InteractiveFrameDnD* _dnd)
{
  if(nullptr == _dnd)
    return false;

  std::map<InteractiveFrame*, InteractiveFrameDnD*>::iterator it =
      mInteractiveFrameDnDMap.find(_dnd->getFrame());
  if(it == mInteractiveFrameDnDMap.end())
    return false;

  delete it->second;
  mInteractiveFrameDnDMap.erase(it);

  return true;
}

//==============================================================================
bool Viewer::disableDragAndDrop(BodyNodeDnD* _dnd)
{
  if(nullptr == _dnd)
    return false;

  std::map<dart::dynamics::BodyNode*, BodyNodeDnD*>::iterator it =
      mBodyNodeDnDMap.find(_dnd->getBodyNode());
  if(it == mBodyNodeDnDMap.end())
    return false;

  delete it->second;
  mBodyNodeDnDMap.erase(it);

  return true;
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

//==============================================================================
void Viewer::updateViewer()
{
  updateDragAndDrops();
}

//==============================================================================
void Viewer::updateDragAndDrops()
{
  for(auto& dnd_pair : mSimpleFrameDnDMap)
  {
    SimpleFrameDnD* dnd = dnd_pair.second;
    dnd->update();
  }

  for(auto& dnd_pair : mSimpleFrameShapeDnDMap)
  {
    SimpleFrameShapeDnD* dnd = dnd_pair.second;
    dnd->update();
  }

  for(auto& dnd_pair : mInteractiveFrameDnDMap)
  {
    InteractiveFrameDnD* dnd = dnd_pair.second;
    dnd->update();
  }

  for(auto& dnd_pair : mBodyNodeDnDMap)
  {
    BodyNodeDnD* dnd = dnd_pair.second;
    dnd->update();
  }
}

//==============================================================================
const ::osg::ref_ptr<::osg::Group>& Viewer::getRootGroup() const
{
  return mRootGroup;
}

} // namespace osg
} // namespace gui
} // namespace dart
