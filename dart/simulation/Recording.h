/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu
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

#ifndef DART_SIMULATION_RECORDING_H_
#define DART_SIMULATION_RECORDING_H_

#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include "dart/dynamics/Skeleton.h"


namespace dart {
namespace simulation {

/// \brief class Recording
class Recording
{
public:
  //--------------------------------------------------------------------------
  // Constructor and Destructor
  //--------------------------------------------------------------------------
  /// \brief Constructor
    Recording(std::vector<dynamics::Skeleton*> & _skeletons) {
        for (int i = 0; i < _skeletons.size(); i++)
            mNumGenCoordsForSkeletons.push_back(_skeletons[i]->getNumGenCoords());
    }
    Recording(std::vector<int> & _skelDofs) {
        for (int i = 0; i < _skelDofs.size(); i++)
            mNumGenCoordsForSkeletons.push_back(_skelDofs[i]);
    }

  /// \brief Destructor
    virtual ~Recording() {};

    int getNumFrames() {
        return mBakedStates.size();
    }
    int getNumSkeletons() {
        return mNumGenCoordsForSkeletons.size();
    }
    int getNumGenCoords(int _skelIdx) {
        return mNumGenCoordsForSkeletons[_skelIdx];
    }
    int getNumContacts(int _frameIdx) {
        int totalDofs = 0;
        for (int i = 0; i < mNumGenCoordsForSkeletons.size(); i++)
            totalDofs += mNumGenCoordsForSkeletons[i];
        return (mBakedStates[_frameIdx].size() - totalDofs) / 6;
    }
    Eigen::VectorXd getConfig(int _frameIdx, int _skelIdx) {
        int index = 0;
        for (int i = 0; i < _skelIdx; i++)
            index += mNumGenCoordsForSkeletons[i];
        return mBakedStates[_frameIdx].segment(index, getNumGenCoords(_skelIdx));
    }
        
    double getGenCoord(int _frameIdx, int _skelIdx, int _dofIdx) {
        int index = 0;
        for (int i = 0; i < _skelIdx; i++)
            index += mNumGenCoordsForSkeletons[i];        
        return mBakedStates[_frameIdx][index + _dofIdx];
    }
    Eigen::Vector3d getContactPoint(int _frameIdx, int _contactIdx) {
        int totalDofs = 0;
        for (int i = 0; i < mNumGenCoordsForSkeletons.size(); i++)
            totalDofs += mNumGenCoordsForSkeletons[i];
        return mBakedStates[_frameIdx].segment(totalDofs + _contactIdx * 6, 3);
    }
    Eigen::Vector3d getContactForce(int _frameIdx, int _contactIdx) {
        int totalDofs = 0;
        for (int i = 0; i < mNumGenCoordsForSkeletons.size(); i++)
            totalDofs += mNumGenCoordsForSkeletons[i];
        return mBakedStates[_frameIdx].segment(totalDofs + _contactIdx * 6 + 3, 3);
    }
    void addState(Eigen::VectorXd & _state) {
        mBakedStates.push_back(_state);
    }

    void addSkeleton(int _numDof) {
        mNumGenCoordsForSkeletons.push_back(_numDof);
    }

 private:
    std::vector<Eigen::VectorXd> mBakedStates;
    std::vector<int> mNumGenCoordsForSkeletons;
};

}  // namespace simulation
}  // namespace dart

#endif  // DART_SIMULATION_WORLD_H_
