/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author  Sumit Jain
    Date    07/21/2011
*/

#ifndef DYNAMICS_BODYNODE_DYNAMICS_H
#define DYNAMICS_BODYNODE_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>
#include "model3d/BodyNode.h"

namespace dynamics{
    /**
    @brief BodyNodeDynamics class represents a single node of the skeleton for dynamics
    */
    class BodyNodeDynamics : public model3d::BodyNode{
    public:      
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // we need this aligned allocator because we have Matrix4d as members in this class

        BodyNodeDynamics(const char *_name = NULL); ///< Default constructor. The name can be up to 128
        virtual ~BodyNodeDynamics(); ///< Default destructor

        // Inverse Dynamics
        Eigen::MatrixXd mJwJoint;    ///< Jacobian matrix for the parent joint
        Eigen::MatrixXd mJwDotJoint;    ///< Time derivative of the Jacobian matrix for the parent joint
        Eigen::Vector3d mVLocal; ///< linear velocity expressed in the *local frame* of the body 
        Eigen::Vector3d mVDotLocal; ///< linear acceleration expressed in the *local frame* of the body 
        Eigen::Vector3d mOmegaLocal;    ///< angular velocity expressed in the *local frame* of the body 
        Eigen::Vector3d mOmegaDotLocal; ///< angular acceleration expressed in the *local frame* of the body 
        Eigen::Vector3d mForceJointLocal;   ///< the constraint joint force in Cartesian coordinates, expressed in the local frame of the body instead of the joint
        Eigen::Vector3d mTorqueJointLocal;   ///< the torque in Cartesian coordinates for the joint expressed in the local frame of the body instead of the joint
        void computeInvDynVelocities( const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot );   ///< computes the velocities in the first pass of the algorithm
        void computeInvDynForces( const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot );   ///< computes the forces in the second pass of the algorithm

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_BODYNODE_DYNAMICS_H
