/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 10/7/2011
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
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include "Object.h"

using namespace std;
using namespace Eigen;

namespace planning {

    /**
     * @function Object
     * @brief Constructor
     */
    Object::Object( ) {
    }

    /**
     * @function ~Object
     * @brief Destructor
     */
    Object::~Object() {
    
    }

    /**
     * @function setPositionX
     * @brief Set position X of the object in World
     */
    void Object::setPositionX( double _pos )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();

        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {

            if( string( joint->getTransform(i)->getName() ) == "RootPos" ) {

                for( unsigned int j = 0; j < joint->getTransform(i)->getNumDofs(); j++ ) {

                    if( string( joint->getTransform(i)->getDof(j)->getName() ) =="RootX" ) { 
                        joint->getTransform(i)->getDof(j)->setValue( _pos ); break; 
                    } 
                }                  
            } 
        } 
    }

    /**
     * @function getPositionX
     * @brief Get position X of the object in World
     */
    void Object::getPositionX( double &_pos )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();

        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {

            if( string( joint->getTransform(i)->getName() ) == "RootPos" ) {

                for( unsigned int j = 0; j < joint->getTransform(i)->getNumDofs(); j++ ) {

                    if( string( joint->getTransform(i)->getDof(j)->getName() ) =="RootX" ) { 
                        _pos = joint->getTransform(i)->getDof(j)->getValue(); break; 
                    } 
                }                  
            } 
        } 
    }

    /**
     * @function setPositionY
     * @brief Set position Y of the object in World
     */
    void Object::setPositionY( double _pos )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();

        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {

            if( string( joint->getTransform(i)->getName() ) == "RootPos" ) {

                for( unsigned int j = 0; j < joint->getTransform(i)->getNumDofs(); j++ ) {

                    if( string( joint->getTransform(i)->getDof(j)->getName() ) =="RootY" ) { 
                        joint->getTransform(i)->getDof(j)->setValue( _pos ); break; 
                    } 
                }                  
            } 
        } 
    } 

    /**
     * @function getPositionY
     * @brief Get position Y of the object in World
     */
    void Object::getPositionY( double &_pos )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();

        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {

            if( string( joint->getTransform(i)->getName() ) == "RootPos" ) {

                for( unsigned int j = 0; j < joint->getTransform(i)->getNumDofs(); j++ ) {

                    if( string( joint->getTransform(i)->getDof(j)->getName() ) =="RootY" ) { 
                        _pos = joint->getTransform(i)->getDof(j)->getValue(); break; 
                    } 
                }                  
            } 
        } 
    } 


    /**
     * @function setPositionZ
     * @brief Set position Z of the object in World
     */
    void Object::setPositionZ( double _pos )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();

        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {

            if( string( joint->getTransform(i)->getName() ) == "RootPos" ) {

                for( unsigned int j = 0; j < joint->getTransform(i)->getNumDofs(); j++ ) {

                    if( string( joint->getTransform(i)->getDof(j)->getName() ) =="RootZ" ) { 
                        joint->getTransform(i)->getDof(j)->setValue( _pos ); break; 
                    } 
                }                  
            } 
        } 
    }

    /**
     * @function getPositionZ
     * @brief Get position Z of the object in World
     */
    void Object::getPositionZ( double &_pos )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();

        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ ) {

            if( string( joint->getTransform(i)->getName() ) == "RootPos" ) {

                for( unsigned int j = 0; j < joint->getTransform(i)->getNumDofs(); j++ ) {

                    if( string( joint->getTransform(i)->getDof(j)->getName() ) =="RootZ" ) { 
                        _pos = joint->getTransform(i)->getDof(j)->getValue(); break; 
                    } 
                }                  
            } 
        } 
    }



    /**
     * @function setRotationRPY
     * @brief Set position Y of the object in World
     */
    void Object::setRotationRPY( double _roll, double _pitch, double _yaw )
    {

        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();
        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ )
        {
            if( string( joint->getTransform(i)->getName() ) == "RootRoll" )
            {  joint->getTransform(i)->getDof(0)->setValue( _roll ); } 

            if( string( joint->getTransform(i)->getName() ) == "RootPitch" )
            {  joint->getTransform(i)->getDof(0)->setValue( _pitch ); } 

            if( string( joint->getTransform(i)->getName() ) == "RootYaw" )
            {  joint->getTransform(i)->getDof(0)->setValue( _yaw ); } 
        } 

    } 

    /**
     * @function getRotationRPY
     * @brief Get Roll, Pitch and Yaw of the object in World
     */
    void Object::getRotationRPY( double &_roll, double &_pitch, double &_yaw )
    {
        kinematics::Joint *joint;
        joint = getRoot()->getParentJoint();
        for( unsigned int i = 0; i < joint->getNumTransforms(); i++ )
        {
            if( string( joint->getTransform(i)->getName() ) == "RootRoll" )
            {  _roll = joint->getTransform(i)->getDof(0)->getValue(); } 

            if( string( joint->getTransform(i)->getName() ) == "RootPitch" )
            {  _pitch = joint->getTransform(i)->getDof(0)->getValue(); } 

            if( string( joint->getTransform(i)->getName() ) == "RootYaw" )
            {  _yaw = joint->getTransform(i)->getDof(0)->getValue(); } 
        } 

    } 

}  // namespace planning
