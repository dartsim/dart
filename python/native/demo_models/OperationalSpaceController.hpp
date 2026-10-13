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

#ifndef DART_PYTHON_NATIVE_DEMO_MODELS_OPERATIONALSPACECONTROLLER_HPP_
#define DART_PYTHON_NATIVE_DEMO_MODELS_OPERATIONALSPACECONTROLLER_HPP_

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

namespace dart_demos {

/// The demo's GUI-free damped-pseudoinverse operational-space control law.
class OperationalSpaceController
{
public:
  OperationalSpaceController(
      dart::dynamics::SkeletonPtr robot, dart::dynamics::SimpleFramePtr target);

  Eigen::VectorXd computeForces() const;
  void update();

  const dart::dynamics::SkeletonPtr& getRobot() const;
  const dart::dynamics::SimpleFramePtr& getTarget() const;
  void setTarget(dart::dynamics::SimpleFramePtr target);
  const Eigen::Vector3d& getOffset() const;
  void setOffset(const Eigen::Vector3d& offset);
  const Eigen::Matrix3d& getKp() const;
  void setKp(const Eigen::Matrix3d& kp);
  const Eigen::MatrixXd& getKd() const;
  void setKd(const Eigen::MatrixXd& kd);

private:
  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::BodyNode* mEndEffector;
  dart::dynamics::SimpleFramePtr mTarget;
  Eigen::Vector3d mOffset = Eigen::Vector3d(0.05, 0, 0);
  Eigen::Matrix3d mKp = 50.0 * Eigen::Matrix3d::Identity();
  Eigen::MatrixXd mKd;
};

} // namespace dart_demos

#endif // DART_PYTHON_NATIVE_DEMO_MODELS_OPERATIONALSPACECONTROLLER_HPP_
