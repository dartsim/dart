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
   * @fuction addDefaultRootNode
   */
  void Robot::addDefaultRootNode() {
    
    // Always set the root node ( 6DOF for rotation and translation )
    kinematics::Joint* joint;
    dynamics::BodyNodeDynamics* node;
    kinematics::Transformation* trans;

    // Set the initial Rootnode that controls the position and orientation of the whole robot
    node = (dynamics::BodyNodeDynamics*) this->createBodyNode("rootBodyNode");
    node->setShape(new kinematics::Shape());
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
   * @function getNumQuickDOF
   * @brief Returns the number of DOF NOT considering the 6 default DOF (XYZ, RPY)
   */
  int Robot::getNumQuickDofs() {
    return ( getNumDofs() - getRoot()->getNumLocalDofs() );
  }

  /**
   * @function getQuickDofsIndices
   */
  Eigen::VectorXi Robot::getQuickDofsIndices() {
    
    int numDofs = getNumDofs();
    int rootDofs = getRoot()->getNumLocalDofs();
    int n = numDofs - rootDofs;
    
    Eigen::VectorXi indices(n);
    
    for( unsigned int i = 0; i < n; i++ )
      {  indices(i) = rootDofs + i; } 
    
    return indices;
  }
  
  /**
   * @function setQuickDofs
   * @brief Set ALL DOFs but the 6 first ones
   */
  bool Robot::setQuickDofs( Eigen::VectorXd _vals )
  {
    int numDofs = getNumDofs();
    int rootDofs = getRoot()->getNumLocalDofs();
    
    if( _vals.size() != (numDofs - rootDofs) )
      {  printf("--(x) Size of input does not match the number of DOFs...check! (x)--\n"); return false; }
    
    for( unsigned int i = 0; i < numDofs - rootDofs; i++ )
      {
	mDofs.at(rootDofs + i)->setValue( _vals(i) );
      } 
    
    return true;
  }
  
  
  /**
   * @function getQuickDofs
   * @brief Get ALL DOFs but the 6 first ones
   */
  Eigen::VectorXd Robot::getQuickDofs( )
  {
    int numDofs = getNumDofs();
    int rootDofs = getRoot()->getNumLocalDofs();
    Eigen::VectorXd quickDofs( numDofs - rootDofs );
    
    for( unsigned int i = 0; i < numDofs - rootDofs; i++ )
      {
	quickDofs(i) = mDofs.at(rootDofs + i)->getValue();
      } 
    
    return quickDofs;
  }
  
  
  /**
   * @function setDofs
   * @brief Set only specified DOFs
   */
  bool Robot::setDofs( Eigen::VectorXd _vals, std::vector<int> _id )
  {
    int numDofs = getNumDofs();
    int rootDofs = getRoot()->getNumLocalDofs();
    
    if( _vals.size() != _id.size() )
      {  printf("--(x) Size of input does not match the number of DOFs...check! (x)--\n"); return false; }
    
    for( unsigned int i = 0; i < _id.size(); i++ )
      {
	if( _id[i] > numDofs - 1 )
          { printf("--(x) You are trying to set an inexisting DOF (x)--\n");
            return false;
          }       
      }  
    
    for( unsigned int i = 0; i < _id.size(); i++ )
      {
	mDofs.at( _id[i] )->setValue( _vals(i) );
      } 
    
    return true;
  }
  
  /**
   * @function getDofs
   * @brief Get only specified DOFs
   */
  Eigen::VectorXd Robot::getDofs( std::vector<int> _id )
  {
    Eigen::VectorXd getDofs( 0 );
    
    int numDofs = getNumDofs();
    
    for( unsigned int i = 0; i < _id.size(); i++ )
      {
	if( _id[i] > numDofs - 1 )
          { printf("--(x) You are trying to set an inexisting DOF (x)--\n");
            return getDofs;
          }       
      }  
    
    getDofs.resize( _id.size() );
    for( unsigned int i = 0; i < _id.size(); i++ )
      {
	getDofs(i) = mDofs.at( _id[i] )->getValue();
      } 
    
    return getDofs;
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
   * @function getBodyNodeTransform
   * @brief Get World Transform of a bodyNode of Robot
   */
  void Robot::getBodyNodeTransform( std::string _name, Eigen::Transform< double, 3,Eigen::Affine > &_tf )
  {
    const char* name = _name.c_str();
    
    kinematics::BodyNode* bodyNode = getNode( name );
    Eigen::Matrix4d tfm= bodyNode->getWorldTransform();
    
    _tf.setIdentity();
    _tf.matrix() = tfm;       
  }
  
  /**
   * @function getBodyNode Position XYZ
   * @brief Get XYZ in World Coordinates
   */
  void Robot::getBodyNodePositionXYZ( std::string _name, double &_x, double &_y, double &_z )
  {
    Eigen::Transform< double, 3,Eigen::Affine > tf;
    getBodyNodeTransform( _name, tf );
    Eigen::Vector3d xyz = tf.translation();
    
    _x = xyz(0); _y = xyz(1); _z = xyz(2);      
  }
  
  /**
   * @function getBodyNode Rotation Matrix
   * @brief Get Rotation matrix (Global)
   */
  void Robot::getBodyNodeRotationMatrix( std::string _name, Eigen::MatrixXd &_rot )
  {
    Eigen::Transform< double, 3,Eigen::Affine > tf;
    getBodyNodeTransform( _name, tf );
    //_rot = tf.block<3,3>(0,0);
    _rot = tf.rotation();
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

    // Update parents first
    std::list<dynamics::BodyNodeDynamics*> nodeStack;
    dynamics::BodyNodeDynamics* u;
    nodeStack.push_back( (dynamics::BodyNodeDynamics*) this->getRoot() );

    int numIter = 0;
    while( !nodeStack.empty() && numIter < this->getNumNodes() ) {
      // Get front element on stack and update it
      u = nodeStack.front();
      u->updateTransform();
      // Pop it out
      nodeStack.pop_front();

      // Add its kids
      for( int idx = 0; idx < u->getNumChildJoints(); ++idx ) {
	nodeStack.push_back( (dynamics::BodyNodeDynamics*)( u->getChildNode(idx) ) );
      }
      numIter++;
    }

  }
  
} // end namespace robotics
