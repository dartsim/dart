/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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
 * @file Object.cpp
 * @brief Implementation of Object subclass (from Skeleton )
 */

#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>

#include "Object.h"

namespace robotics {
  /**
   * @function Object
   * @brief Constructor
   */
  Object::Object() {
    
    // Always set the root node ( 6DOF for rotation and translation )
    kinematics::Joint* joint;
    dynamics::BodyNodeDynamics* node;
    kinematics::Transformation* trans;

    // Set the initial Rootnode that controls the position and orientation of the whole robot
    node = (dynamics::BodyNodeDynamics*) this->createBodyNode("rootBodyNode");
    joint = new kinematics::Joint( NULL, node, "rootJoint" );
    
    // Add DOFs for RPY and XYZ of the whole robot
    trans = new kinematics::TrfmTranslateX( new kinematics::Dof( 0, "rootX" ), "Tx" );
    joint->addTransform( trans, true );
    this->addTransform( trans );
    
    trans = new kinematics::TrfmTranslateY( new kinematics::Dof( 0, "rootY" ), "Ty" );
    joint->addTransform( trans, true );
    this->addTransform( trans );

    trans = new kinematics::TrfmTranslateZ( new kinematics::Dof( 0, "rootZ" ), "Tz" );
    joint->addTransform( trans, true );
    this->addTransform( trans );
   
    trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, "rootYaw" ), "Try" );
    joint->addTransform( trans, true );
    this->addTransform( trans );
 
    trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, "rootPitch" ), "Trp" );
    joint->addTransform( trans, true );
    this->addTransform( trans );

    trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, "rootRoll" ), "Trr" );
    joint->addTransform( trans, true );
    this->addTransform( trans );

    // Set this first node as root node
    this->addNode( node );
    this->initSkel();    
  }

  /**
   * @function ~Object
   * @brief Destructor
   */
  Object::~Object() {
    
  }

} // namespace robotics
