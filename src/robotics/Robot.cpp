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
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include "Robot.h"
#include <stdio.h>
#include <kinematics/ShapeMesh.h>
#include <list>

using namespace std;

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
   * @function setPositionX
   * @brief Set position X of the object in World
   */
  void Robot::setPositionX( double _pos )
  {
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEX ) {
	joint->getTransform(i)->getDof(0)->setValue( _pos ); break; 
      } 
    }  
  }
  
  /**
   * @function getPositionX
   * @brief Get position X of the object in World
   */
  void Robot::getPositionX( double &_pos )
  {
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEX ) {
	_pos = joint->getTransform(i)->getDof(0)->getValue(); break; 
      } 
    }  
  }
  
  /**
   * @function setPositionY
   * @brief Set position Y of the object in World
   */
  void Robot::setPositionY( double _pos )
  {
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEY ) {
	joint->getTransform(i)->getDof(0)->setValue( _pos ); break; 
      } 
    }  
  }
  
  /**
   * @function getPositionY
   * @brief Get position Y of the object in World
   */
    void Robot::getPositionY( double &_pos )
    {
      kinematics::Joint *joint;
      joint = getRoot()->getParentJoint();
      
      for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEY ) {
	  _pos = joint->getTransform(i)->getDof(0)->getValue(); break; 
	} 
      }  
    }
  
  
  /**
   * @function setPositionZ
   * @brief Set position Z of the object in World
   */
  void Robot::setPositionZ( double _pos )
  {
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEZ ) {
	joint->getTransform(i)->getDof(0)->setValue( _pos ); break; 
      } 
    }  
  }
  
  /**
   * @function getPositionZ
   * @brief Get position Z of the object in World
   */
  void Robot::getPositionZ( double &_pos )
  {
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEZ ) {
	_pos = joint->getTransform(i)->getDof(0)->getValue(); break; 
      } 
    }  
  }
  
  
  /**
   * @function setPositionXYZ
   */
  void Robot::setPositionXYZ( double _x, double _y, double _z ) { 
    
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEX ) {
	joint->getTransform(i)->getDof(0)->setValue( _x ); 
      } 
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEY ) {
	joint->getTransform(i)->getDof(0)->setValue( _y ); 
      } 
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEZ ) {
	joint->getTransform(i)->getDof(0)->setValue( _z ); 
      } 
    } 
    
  }
  
  /**
   * @function getPositionXYZ
   */
  void Robot::getPositionXYZ( double &_x, double &_y, double &_z ) {
    
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {
      
      if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEX ) {
	_x = joint->getTransform(i)->getDof(0)->getValue(); 
      } 
      
      else if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEY ) {
	_y = joint->getTransform(i)->getDof(0)->getValue(); 
      } 
      
      else if( joint->getTransform(i)->getType() == kinematics::Transformation::T_TRANSLATEZ ) {
	_z = joint->getTransform(i)->getDof(0)->getValue(); 
      } 
    }  
  }
  
  
  /**
   * @function setRotationRPY
   * @brief Set position Y of the object in World
   */
  void Robot::setRotationRPY( double _roll, double _pitch, double _yaw )
  {
    
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ )
      {
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_ROTATEX )
	  {  joint->getTransform(i)->getDof(0)->setValue( _roll ); } 
	
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_ROTATEY )
	  {  joint->getTransform(i)->getDof(0)->setValue( _pitch ); } 
	
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_ROTATEZ )
	  {  joint->getTransform(i)->getDof(0)->setValue( _yaw ); } 
      } 
  } 
  
  /**
   * @function getRotationRPY
   * @brief Get Roll, Pitch and Yaw of the object in World
   */
  void Robot::getRotationRPY( double &_roll, double &_pitch, double &_yaw )
  {
    kinematics::Joint *joint;
    joint = getRoot()->getParentJoint();
    for( unsigned int i = 0; i < joint->getNumTransforms(); i++ )
      {
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_ROTATEX )
	  {  _roll = joint->getTransform(i)->getDof(0)->getValue(); } 
	
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_ROTATEY )
	  {  _pitch = joint->getTransform(i)->getDof(0)->getValue(); } 
	
	if( joint->getTransform(i)->getType() == kinematics::Transformation::T_ROTATEZ )
	  {  _yaw = joint->getTransform(i)->getDof(0)->getValue(); } 
      } 
    
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
