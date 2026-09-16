/*
 * Copyright (c) 2011, The DART development contributors
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/simulation/Recording.hpp"

#include "dart/common/Console.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <iostream>
#include <utility>

namespace dart {
namespace simulation {

//==============================================================================
Recording::Recording(const std::vector<dynamics::SkeletonPtr>& _skeletons)
{
  for (std::size_t i = 0; i < _skeletons.size(); i++)
    mNumGenCoordsForSkeletons.push_back(_skeletons[i]->getNumDofs());
}

//==============================================================================
Recording::Recording(const std::vector<int>& _skelDofs)
{
  for (std::size_t i = 0; i < _skelDofs.size(); i++)
    mNumGenCoordsForSkeletons.push_back(_skelDofs[i]);
}

//==============================================================================
Recording::~Recording() {}

//==============================================================================
int Recording::getNumFrames() const
{
  return mBakedStates.size();
}

//==============================================================================
int Recording::getNumSkeletons() const
{
  return mNumGenCoordsForSkeletons.size();
}

//==============================================================================
int Recording::getNumDofs(int _skelIdx) const
{
  return mNumGenCoordsForSkeletons[_skelIdx];
}

//==============================================================================
int Recording::getNumContacts(int _frameIdx) const
{
  int totalDofs = 0;
  for (std::size_t i = 0; i < mNumGenCoordsForSkeletons.size(); i++)
    totalDofs += mNumGenCoordsForSkeletons[i];
  return (mBakedStates[_frameIdx].size() - totalDofs) / 6;
}

//==============================================================================
Eigen::VectorXd Recording::getConfig(int _frameIdx, int _skelIdx) const
{
  int index = 0;
  for (int i = 0; i < _skelIdx; i++)
    index += mNumGenCoordsForSkeletons[i];
  return mBakedStates[_frameIdx].segment(index, getNumDofs(_skelIdx));
}

//==============================================================================
double Recording::getGenCoord(int _frameIdx, int _skelIdx, int _dofIdx) const
{
  int index = 0;
  for (int i = 0; i < _skelIdx; i++)
    index += mNumGenCoordsForSkeletons[i];
  return mBakedStates[_frameIdx][index + _dofIdx];
}

//==============================================================================
Eigen::Vector3d Recording::getContactPoint(int _frameIdx, int _contactIdx) const
{
  int totalDofs = 0;
  for (std::size_t i = 0; i < mNumGenCoordsForSkeletons.size(); i++)
    totalDofs += mNumGenCoordsForSkeletons[i];
  return mBakedStates[_frameIdx].segment(totalDofs + _contactIdx * 6, 3);
}

//==============================================================================
Eigen::Vector3d Recording::getContactForce(int _frameIdx, int _contactIdx) const
{
  int totalDofs = 0;
  for (std::size_t i = 0; i < mNumGenCoordsForSkeletons.size(); i++)
    totalDofs += mNumGenCoordsForSkeletons[i];
  return mBakedStates[_frameIdx].segment(totalDofs + _contactIdx * 6 + 3, 3);
}

//==============================================================================
void Recording::clear()
{
  mBakedStates.clear();
}

//==============================================================================
void Recording::addState(const Eigen::VectorXd& _state)
{
  mBakedStates.push_back(_state);
}

//==============================================================================
void Recording::updateNumGenCoords(
    const std::vector<dynamics::SkeletonPtr>& _skeletons)
{
  std::vector<int> numGenCoordsForSkeletons;
  numGenCoordsForSkeletons.reserve(_skeletons.size());
  for (const auto& skeleton : _skeletons)
    numGenCoordsForSkeletons.push_back(
        static_cast<int>(skeleton->getNumDofs()));

  if (numGenCoordsForSkeletons == mNumGenCoordsForSkeletons)
    return;

  // Every baked frame is packed under the layout that was current when it was
  // baked, so frames from the previous layout cannot be sliced under the new
  // one. Treat the change as a new recording epoch instead of silently
  // attributing old values to the wrong skeletons.
  if (!mBakedStates.empty()) {
    dtwarn << "[Recording::updateNumGenCoords] Dropping " << mBakedStates.size()
           << " recorded frame(s) because the skeleton DOF layout changed; "
           << "frames baked under the previous layout can no longer be "
           << "interpreted.\n";
    clear();
  }

  mNumGenCoordsForSkeletons = std::move(numGenCoordsForSkeletons);
}

} // namespace simulation
} // namespace dart
