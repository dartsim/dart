/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include <osgViewer/Viewer>

#include <dart/dart.h>
#include <dart/gui/osg/osg.h>
#include <dart/utils/utils.h>

using namespace dart::dynamics;

class RecordingWorld : public osgDart::WorldNode
{
public:

  RecordingWorld(const dart::simulation::WorldPtr& world)
    : osgDart::WorldNode(world)
  {
    grabTimeSlice();
    mCurrentIndex = 0;
  }

  void grabTimeSlice()
  {
    TimeSlice slice;
    slice.reserve(mWorld->getNumSkeletons());

    for(size_t i=0; i < mWorld->getNumSkeletons(); ++i)
    {
      const SkeletonPtr& skeleton = mWorld->getSkeleton(i);
      State state;
      state.mConfig = skeleton->getConfiguration();
      state.mAspectStates.reserve(skeleton->getNumBodyNodes());

      for(size_t j=0; j < skeleton->getNumBodyNodes(); ++j)
      {
        BodyNode* bn = skeleton->getBodyNode(j);
        state.mAspectStates.push_back(bn->getCompositeState());
      }

      slice.push_back(state);
    }

    mHistory.push_back(slice);
  }

  void customPostStep() override
  {
    if(mCurrentIndex < mHistory.size()-1)
      mHistory.resize(mCurrentIndex+1);

    grabTimeSlice();
    ++mCurrentIndex;
  }

  void moveTo(size_t index)
  {
    mViewer->simulate(false);

    if(mHistory.empty())
      return;

    if(index >= mHistory.size())
      index = mHistory.size() - 1;

    std::cout << "Moving to time step #" << index << std::endl;

    const TimeSlice& slice = mHistory[index];
    for(size_t i=0; i < slice.size(); ++i)
    {
      const State& state = slice[i];
      const SkeletonPtr& skeleton = mWorld->getSkeleton(i);

      skeleton->setConfiguration(state.mConfig);

      for(size_t j=0; j < skeleton->getNumBodyNodes(); ++j)
      {
        BodyNode* bn = skeleton->getBodyNode(j);
        bn->setCompositeState(state.mAspectStates[j]);
      }
    }

    mCurrentIndex = index;
  }

  void moveForward(int delta)
  {
    moveTo(mCurrentIndex+delta);
  }

  void moveBackward(int delta)
  {
    if(mCurrentIndex > 0)
      moveTo(mCurrentIndex-delta);
  }

  void restart()
  {
    moveTo(0);
  }

  void moveToEnd()
  {
    moveTo(mHistory.size()-1);
  }

  struct State
  {
    Skeleton::Configuration mConfig;
    std::vector<dart::common::Composite::State> mAspectStates;
  };

  using TimeSlice = std::vector<State>;
  using History = std::vector<TimeSlice>;

  History mHistory;

  size_t mCurrentIndex;
};

class RecordingEventHandler : public osgGA::GUIEventHandler
{
public:

  RecordingEventHandler(RecordingWorld* rec)
    : mRecWorld(rec)
  {
    // Do nothing
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&) override
  {
    if(!mRecWorld)
      return false;

    if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if(ea.getKey() == '[')
      {
        mRecWorld->moveBackward(1);
        return true;
      }

      if(ea.getKey() == ']')
      {
        mRecWorld->moveForward(1);
        return true;
      }

      if(ea.getKey() == '{')
      {
        mRecWorld->moveBackward(10);
        return true;
      }

      if(ea.getKey() == '}')
      {
        mRecWorld->moveForward(10);
        return true;
      }

      if(ea.getKey() == 'r')
      {
        mRecWorld->restart();
        return true;
      }

      if(ea.getKey() == '\\')
      {
        mRecWorld->moveToEnd();
        return true;
      }
    }

    return false;
  }

  RecordingWorld* mRecWorld;
};

int main()
{
  using namespace dart::dynamics;

  dart::simulation::WorldPtr world =
      dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/softBodies.skel");

  osg::ref_ptr<RecordingWorld> node = new RecordingWorld(world);

  node->simulate(true);
  node->setNumStepsPerCycle(15);

  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.addEventHandler(new RecordingEventHandler(node));

  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
