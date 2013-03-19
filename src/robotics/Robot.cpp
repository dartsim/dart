/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 2012-08-07
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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
 **
 * @file Robot.cpp
 * @brief Implementation of Robot subclass (from Dynamic Skeleton )
 */

#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include "Robot.h"


namespace robotics {

  
  /**
   * @function Robot
   * @brief Constructor
   */
  Robot::Robot() {
  }
  
  /**
   * @function ~Robot
   * @brief Destructor
   */
  Robot::~Robot() {
    
  }

  /**
   * @function setRootTransform
   * @brief Set x, y, z, roll, pitch and yaw
   */
  void Robot::setRootTransform(const Eigen::Matrix<double, 6, 1>& _pose) {

    kinematics::Joint* joint = getRoot()->getParentJoint();

    assert(joint->getNumTransforms() >= 4);
    assert(joint->getTransform(0)->getType() == kinematics::Transformation::T_TRANSLATE);
    assert(joint->getTransform(1)->getType() == kinematics::Transformation::T_ROTATEZ);
    assert(joint->getTransform(2)->getType() == kinematics::Transformation::T_ROTATEY);
    assert(joint->getTransform(3)->getType() == kinematics::Transformation::T_ROTATEX);

    joint->getTransform(0)->getDof(0)->setValue(_pose(0));
    joint->getTransform(0)->getDof(1)->setValue(_pose(1));
    joint->getTransform(0)->getDof(2)->setValue(_pose(2));
    joint->getTransform(1)->getDof(0)->setValue(_pose(5));
    joint->getTransform(2)->getDof(0)->setValue(_pose(4));
    joint->getTransform(3)->getDof(0)->setValue(_pose(3));

    joint->updateStaticTransform();
    update();
  }

  /**
   * @function getRootTransform
   * @brief Get x, y, z, roll, pitch and yaw
   */
  Eigen::Matrix<double, 6, 1> Robot::getRootTransform() {

    kinematics::Joint *joint = getRoot()->getParentJoint();

    assert(joint->getNumTransforms() >= 4);
    assert(joint->getTransform(0)->getType() == kinematics::Transformation::T_TRANSLATE);
    assert(joint->getTransform(1)->getType() == kinematics::Transformation::T_ROTATEZ);
    assert(joint->getTransform(2)->getType() == kinematics::Transformation::T_ROTATEY);
    assert(joint->getTransform(3)->getType() == kinematics::Transformation::T_ROTATEX);

    Eigen::Matrix<double, 6, 1> pose;
    pose(0) = joint->getTransform(0)->getDof(0)->getValue();
    pose(1) = joint->getTransform(0)->getDof(1)->getValue();
    pose(2) = joint->getTransform(0)->getDof(2)->getValue();
    pose(3) = joint->getTransform(3)->getDof(0)->getValue();
    pose(4) = joint->getTransform(2)->getDof(0)->getValue();
    pose(5) = joint->getTransform(1)->getDof(0)->getValue(); 
    
    return pose;
  }


  /**
   * @function update
   */
  void Robot::update()
  {
    for(int i=0; i < getNumDofs(); i++)
      {  mCurrPose[i] = mDofs.at(i)->getValue();  }

    for(int i=0; i<getNumNodes(); i++) {
      mNodes.at(i)->updateTransform();
    }
    
    for(int i=0; i<getNumNodes(); i++) {
      mNodes.at(i)->updateFirstDerivatives();
    }
  }
  
} // end namespace robotics
