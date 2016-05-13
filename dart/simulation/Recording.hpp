/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_SIMULATION_RECORDING_HPP_
#define DART_SIMULATION_RECORDING_HPP_

#include <vector>

#include <Eigen/Dense>

#include "dart/dynamics/Skeleton.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace simulation {

/// \brief class Recording
class Recording
{
public:
  /// \brief Create Recording with a list of skeletons
  explicit Recording(const std::vector<dynamics::SkeletonPtr>& _skeletons);

  /// \brief Create Recording with a list of number of dofs
  explicit Recording(const std::vector<int>& _skelDofs);

  /// \brief Destructor
  virtual ~Recording();

  /// \brief Get number of frames
  int getNumFrames() const;

  /// \brief Get number of skeletons
  int getNumSkeletons() const;

  /// \brief Get number of generalized coordinates of skeleton whoes index is
  /// _skelIdx
  int getNumDofs(int _skelIdx) const;

  /// \brief Get number of contacts at frame number _frameIdx
  int getNumContacts(int _frameIdx) const;

  /// \brief Get skeleton configurations whose index is _skelIdx at frame number
  /// _frameIdx
  Eigen::VectorXd getConfig(int _frameIdx, int _skelIdx) const;

  /// \brief Get _dofIdx-th single configruation of a skeleton whose index is
  /// _skelIdx at frame number _frameIdx
  double getGenCoord(int _frameIdx, int _skelIdx, int _dofIdx) const;

  /// \brief Get contact point whose index is _contactIdx at frame number
  /// _frameIdx
  Eigen::Vector3d getContactPoint(int _frameIdx, int _contactIdx) const;

  /// \brief Get contact force whose index is _contactIdx at frame number
  /// _frameIdx
  Eigen::Vector3d getContactForce(int _frameIdx, int _contactIdx) const;

  /// \brief Clear the saved histories
  void clear();  

  /// \brief Add state
  void addState(const Eigen::VectorXd& _state);

  /// \brief Update list for number of generalized coordinates
  void updateNumGenCoords(const std::vector<dynamics::SkeletonPtr>& _skeletons);

private:
  /// \brief Baked states
  std::vector<Eigen::VectorXd> mBakedStates;

  /// \brief Number of generalized coordinates for skeletons
  std::vector<int> mNumGenCoordsForSkeletons;
};

}  // namespace simulation
}  // namespace dart

#endif  // DART_SIMULATION_RECORDING_HPP_
