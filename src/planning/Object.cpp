/**
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
