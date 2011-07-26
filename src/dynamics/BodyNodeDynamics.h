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
#include "utils/EigenHelper.h"

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
        Eigen::Vector3d mVBody; ///< linear velocity expressed in the *local frame* of the body 
        Eigen::Vector3d mVDotBody; ///< linear acceleration expressed in the *local frame* of the body 
        Eigen::Vector3d mOmegaBody;    ///< angular velocity expressed in the *local frame* of the body 
        Eigen::Vector3d mOmegaDotBody; ///< angular acceleration expressed in the *local frame* of the body 
        Eigen::Vector3d mForceJointBody;   ///< the constraint joint force in Cartesian coordinates, expressed in the local frame of the body instead of the joint
        Eigen::Vector3d mTorqueJointBody;   ///< the torque in Cartesian coordinates for the joint expressed in the local frame of the body instead of the joint
        void computeInvDynVelocities( const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot );   ///< computes the velocities in the first pass of the algorithm
        void computeInvDynForces( const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot );   ///< computes the forces in the second pass of the algorithm

        // Regular Dynamics formulation - M*qdd + C*qdot + g = 0
        Eigen::MatrixXd mM; ///< Mass matrix of dimension numDependentDofs x numDependentDofs; to be added carefully to the skeleton mass matrix
        Eigen::MatrixXd mC; ///< Coriolis matrix of dimension numDependentDofs x numDependentDofs; to be added carefully to the skeleton mass matrix

    protected:

        // Regular Dynamics formulation - second derivatives
        EIGEN_VV_MAT4D mTqq;  ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
        EIGEN_VV_MAT4D mWqq;  ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
        std::vector<Eigen::MatrixXd> mJvq; ///< Linear Jacobian derivative wrt to all the dependent dofs
        std::vector<Eigen::MatrixXd> mJwq; ///< Angular Jacobian derivative wrt to all the dependent dofs
        Eigen::MatrixXd mJvDot; ///< Time derivative of the Linear velocity Jacobian
        Eigen::MatrixXd mJwDot; ///< Time derivative of the Angular velocity Jacobian

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_BODYNODE_DYNAMICS_H
