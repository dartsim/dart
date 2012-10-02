/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 2012-08-03
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
 *
 *   @file Robot.h
 *   @brief Class that describe Robot, a subclass of Skeleton
 */

#ifndef __DART_ROBOTICS_ROBOT_H__
#define __DART_ROBOTICS_ROBOT_H__

#include <string>
#include <vector>
#include "dynamics/SkeletonDynamics.h"

class aiScene;

namespace robotics {
  #define MAX_ROBOT_NAME 128
  
  /**
   * @class Robot
   * @brief Subclass of Dynamic Skeleton
   */
  class Robot : public dynamics::SkeletonDynamics {

  public:
    Robot();
    virtual ~Robot();

    inline std::string getName() { return mName; }
    inline void setName( std::string _name ) { mName = _name; }
    inline std::string getPathName() { return mPathName; }
    inline void setPathName( std::string _pname ) { mPathName = _pname; }
    inline int getGripID() { return mGripID; }
    inline void setGripID( int _i ) { mGripID = _i; }
    inline int getNumModels() { return mModels.size(); }

    int getNumQuickDofs(); 
    Eigen::VectorXi getQuickDofsIndices();
    bool setQuickDofs( Eigen::VectorXd _vals ); 
    Eigen::VectorXd getQuickDofs( );
    bool setDofs( Eigen::VectorXd _vals, Eigen::VectorXi _id );
    Eigen::VectorXd getDofs( Eigen::VectorXi _id );
    
    void setPositionX( double _pos );  
    void getPositionX( double &_pos );
    
    void setPositionY( double _pos );
    void getPositionY( double &_pos );
    
    void setPositionZ( double _pos );
    void getPositionZ( double &_pos );
    
    void setPositionXYZ( double _x, double _y, double _z );
    void getPositionXYZ( double &_x, double &_y, double &_z ); 
    
    void setRotationRPY( double _roll, double _pitch, double _yaw );
    void getRotationRPY( double &_roll, double &_pitch, double &_yaw );
    
    void getBodyNodeTransform( std::string _name, Eigen::Transform< double, 3,Eigen::Affine > &_tf );
    void getBodyNodePositionXYZ( std::string _name, double &_x, double &_y, double &_z );
    void getBodyNodeRotationMatrix( std::string _name, Eigen::MatrixXd &_rot );
    
    void update();
    
    const aiScene* loadModel( std::string _filename );
    void addModel( const aiScene* _model, int _index );
    const aiScene* getModel( int _i );
    int getModelIndex( int _i );
    
  private:
    std::string mName;
    std::string mPathName;
    int mGripID; /// THIS HAS TO BE REMOVED
    std::vector<const aiScene*> mModels;
    std::vector<int> mModelIndices;
  };
}

#endif /** __DART_ROBOTICS_ROBOT_H__ */
