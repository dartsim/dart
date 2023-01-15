/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/gui/osg/RealTimeWorldNode.hpp"

#include "dart/common/Console.hpp"
#include "dart/gui/osg/detail/CameraModeCallback.hpp"
#include "dart/simulation/World.hpp"

#include <osgShadow/ShadowedScene>

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
RealTimeWorldNode::RealTimeWorldNode(
    const std::shared_ptr<dart::simulation::World>& world,
    const ::osg::ref_ptr<osgShadow::ShadowTechnique>& shadower,
    const double targetFrequency,
    const double targetRealTimeFactor)
  : WorldNode(world, shadower),
    mFirstRefresh(true),
    mTargetRealTimeLapse(1.0 / targetFrequency),
    mTargetSimTimeLapse(targetRealTimeFactor * mTargetRealTimeLapse),
    mLastRealTimeFactor(0.0),
    mLowestRealTimeFactor(math::inf<double>()),
    mHighestRealTimeFactor(0.0)
{
  // Do nothing
}

//==============================================================================
void RealTimeWorldNode::setTargetFrequency(double targetFrequency)
{
  const double targetRTF = mTargetSimTimeLapse / mTargetRealTimeLapse;
  mTargetRealTimeLapse = 1.0 / targetFrequency;
  mTargetSimTimeLapse = targetRTF * mTargetRealTimeLapse;
}

//==============================================================================
double RealTimeWorldNode::getTargetFrequency() const
{
  return 1.0 / mTargetRealTimeLapse;
}

//==============================================================================
void RealTimeWorldNode::setTargetRealTimeFactor(double targetRTF)
{
  mTargetSimTimeLapse = targetRTF * mTargetRealTimeLapse;
}

//==============================================================================
double RealTimeWorldNode::getTargetRealTimeFactor() const
{
  return mTargetSimTimeLapse / mTargetRealTimeLapse;
}

//==============================================================================
double RealTimeWorldNode::getLastRealTimeFactor() const
{
  return mLastRealTimeFactor;
}

//==============================================================================
double RealTimeWorldNode::getLowestRealTimeFactor() const
{
  return mLowestRealTimeFactor;
}

//==============================================================================
double RealTimeWorldNode::getHighestRealTimeFactor() const
{
  return mHighestRealTimeFactor;
}

//==============================================================================
void RealTimeWorldNode::refresh()
{
  customPreRefresh();
  clearChildUtilizationFlags();

  if (mNumStepsPerCycle != 1)
  {
    dtwarn << "[RealTimeWorldNode] The number of steps per cycle has been set "
           << "to [" << mNumStepsPerCycle << "], but this value is ignored by "
           << "the RealTimeWorldNode::refresh() function. Use the function "
           << "RealTimeWorldNode::setTargetRealTimeFactor(double) to change "
           << "the simulation speed.\n";
    mNumStepsPerCycle = 1;
  }

  if (mWorld && mSimulating)
  {
    if (mFirstRefresh)
    {
      mRefreshTimer.setStartTick();
      mFirstRefresh = false;
    }

    const double startSimTime = mWorld->getTime();
    const double simTimeStep = mWorld->getTimeStep();

    while (mRefreshTimer.time_s() < mTargetRealTimeLapse)
    {
      const double nextSimTimeLapse
          = mWorld->getTime() - startSimTime + simTimeStep;

      if (nextSimTimeLapse <= mTargetSimTimeLapse)
      {
        customPreStep();
        mWorld->step();
        customPostStep();
      }
    }

    mLastRealTimeFactor
        = (mWorld->getTime() - startSimTime) / mTargetRealTimeLapse;
    mLowestRealTimeFactor
        = std::min(mLastRealTimeFactor, mLowestRealTimeFactor);
    mHighestRealTimeFactor
        = std::max(mLastRealTimeFactor, mHighestRealTimeFactor);

    mRefreshTimer.setStartTick();
  }
  else
  {
    mFirstRefresh = true;
  }

  refreshSkeletons();
  refreshSimpleFrames();

  clearUnusedNodes();

  customPostRefresh();
}

} // namespace osg
} // namespace gui
} // namespace dart
