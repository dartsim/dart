/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "WamWorld.hpp"

//==============================================================================
WamWorld::WamWorld(WorldPtr world, SkeletonPtr robot)
  : dart::gui::osg::WorldNode(world),
    mWam(robot),
    iter(0),
    l_foot(robot->getEndEffector("l_foot")),
    r_foot(robot->getEndEffector("r_foot")),
    l_hand(robot->getEndEffector("l_hand")),
    r_hand(robot->getEndEffector("r_hand"))
{
  mMoveComponents.resize(NUM_MOVE, false);
  mAnyMovement = false;
  mAmplifyMovement = false;
}

//==============================================================================
void WamWorld::setMovement(const std::vector<bool>& moveComponents)
{
  mMoveComponents = moveComponents;

  mAnyMovement = false;

  for (bool move : mMoveComponents)
  {
    if (move)
    {
      mAnyMovement = true;
      break;
    }
  }
}

//==============================================================================
void WamWorld::customPreRefresh()
{
  if (mAnyMovement)
  {
    Eigen::Isometry3d old_tf = mWam->getBodyNode(0)->getWorldTransform();
    Eigen::Isometry3d new_tf = Eigen::Isometry3d::Identity();
    Eigen::Vector3d forward = old_tf.linear().col(0);
    forward[2] = 0.0;
    if (forward.norm() > 1e-10)
      forward.normalize();
    else
      forward.setZero();

    Eigen::Vector3d left = old_tf.linear().col(1);
    left[2] = 0.0;
    if (left.norm() > 1e-10)
      left.normalize();
    else
      left.setZero();

    const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ();

    double linearStep = 0.01;
    double elevationStep = 0.2*linearStep;
    double rotationalStep = 2.0*M_PI/180.0;

    if (mAmplifyMovement)
    {
      linearStep *= 2.0;
      elevationStep *= 2.0;
      rotationalStep *= 2.0;
    }

    if (mMoveComponents[MOVE_W])
      new_tf.translate( linearStep*forward);

    if (mMoveComponents[MOVE_S])
      new_tf.translate(-linearStep*forward);

    if (mMoveComponents[MOVE_A])
      new_tf.translate( linearStep*left);

    if (mMoveComponents[MOVE_D])
      new_tf.translate(-linearStep*left);

    if (mMoveComponents[MOVE_F])
      new_tf.translate( elevationStep*up);

    if (mMoveComponents[MOVE_Z])
      new_tf.translate(-elevationStep*up);

    if (mMoveComponents[MOVE_Q])
      new_tf.rotate(Eigen::AngleAxisd( rotationalStep, up));

    if (mMoveComponents[MOVE_E])
      new_tf.rotate(Eigen::AngleAxisd(-rotationalStep, up));

    new_tf.pretranslate(old_tf.translation());
    new_tf.rotate(old_tf.rotation());

    mWam->getJoint(0)->setPositions(FreeJoint::convertToPositions(new_tf));
  }

  mWam->getIK(true)->solve();
}
