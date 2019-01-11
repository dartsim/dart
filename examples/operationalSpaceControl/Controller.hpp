/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>

#include <dart/dart.hpp>

/// \brief Operational space controller for 6-dof manipulator
class Controller
{
public:
  /// \brief Constructor
  Controller(dart::dynamics::SkeletonPtr _robot,
             dart::dynamics::BodyNode* _endEffector);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief
  void update(const Eigen::Vector3d& _targetPosition);

  /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Get end effector of the robot
  dart::dynamics::BodyNode* getEndEffector() const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

private:
  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief End-effector of the robot
  dart::dynamics::BodyNode* mEndEffector;

  /// \brief Control forces
  Eigen::VectorXd mForces;

  /// \brief Proportional gain for the virtual spring forces at the end effector
  Eigen::Matrix3d mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  Eigen::Matrix3d mKv;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
