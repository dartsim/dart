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

        // Following functions called automatically by the skeleton: computeInverseDynamicsLinear and computeDynamics respectively
        void initInverseDynamics();  ///< initialize data structures for linear inverse dynamics computation
        void initDynamics();  ///< initialize data structures for regular dynamics computation

        // Inverse Dynamics
        Eigen::MatrixXd mJwJoint;    ///< Jacobian matrix for the parent joint
        Eigen::MatrixXd mJwDotJoint;    ///< Time derivative of the Jacobian matrix for the parent joint
        Eigen::Vector3d mVelBody; ///< linear velocity expressed in the *local frame* of the body 
        Eigen::Vector3d mVelDotBody; ///< linear acceleration expressed in the *local frame* of the body 
        Eigen::Vector3d mOmegaBody;    ///< angular velocity expressed in the *local frame* of the body 
        Eigen::Vector3d mOmegaDotBody; ///< angular acceleration expressed in the *local frame* of the body 
        Eigen::Vector3d mForceJointBody;   ///< the constraint joint force in Cartesian coordinates, expressed in the local frame of the body instead of the joint
        Eigen::Vector3d mTorqueJointBody;   ///< the torque in Cartesian coordinates for the joint expressed in the local frame of the body instead of the joint
        void computeInvDynVelocities( const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot, bool _computeJacobians=true );   ///< computes the velocities in the first pass of the algorithm; also computes Transform W etc using updateTransform; computes Jacobians Jv and Jw if the flag is true; replaces updateFirstDerivatives of regular dynamics
        void computeInvDynForces( const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot );   ///< computes the forces in the second pass of the algorithm

        // Regular Dynamics formulation - M*qdd + C*qdot + g = 0
        void updateSecondDerivatives();  ///< Update the second derivatives of the transformations 
        Eigen::Vector3d mVel; ///< Linear velocity in the world frame
        Eigen::Vector3d mOmega; ///< Angular velocity in the world frame
        Eigen::MatrixXd mM; ///< Mass matrix of dimension numDependentDofs x numDependentDofs; to be added carefully to the skeleton mass matrix
        Eigen::MatrixXd mC; ///< Coriolis matrix of dimension numDependentDofs x numDependentDofs; to be added carefully to the skeleton Coriolis matrix
        Eigen::VectorXd mCvec; ///< Coriolis vector of dimension numDependentDofs x 1; mCvec = mC*qdot
        Eigen::VectorXd mG; ///< Gravity vector or generalized gravity forces; dimension numDependentDofs x 1
        void evalVelocity(const Eigen::VectorXd &_qDotSkel);   ///< evaluates the velocity of the COM in the world frame
        void evalOmega(const Eigen::VectorXd &_qDotSkel);   ///< evaluates the Omega in the world frame
        void evalMassMatrix();  ///< evaluates the mass matrix mM
        void evalCoriolisMatrix(const Eigen::VectorXd &_qDotSkel);  ///< evaluates the Coriolis matrix mC
        void evalCoriolisVector(const Eigen::VectorXd &_qDotSkel);  ///< evaluates the Coriolis vector mCvec directy: i.e. shortcut for mC*qdot
        void evalGravityVector(const Eigen::Vector3d &_gravity);   ///< evaluates the gravity vector mG in the generalized coordinates

        // add functions to add to the existing *full* matrices typically for the entire skeleton
        void addMass(Eigen::MatrixXd &_M);
        void addCoriolis(Eigen::MatrixXd &_C);
        void addCoriolisVec(Eigen::VectorXd &_Cvec);
        void addGravity(Eigen::VectorXd &_G);

    //protected:
    public:

        // Regular Dynamics formulation - second derivatives
        EIGEN_VV_MAT4D mTqq;  ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
        EIGEN_VV_MAT4D mWqq;  ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
        std::vector<Eigen::MatrixXd> mJvq; ///< Linear Jacobian derivative wrt to all the dependent dofs
        std::vector<Eigen::MatrixXd> mJwq; ///< Angular Jacobian derivative wrt to all the dependent dofs
        Eigen::MatrixXd mJvDot; ///< Time derivative of the Linear velocity Jacobian
        Eigen::MatrixXd mJwDot; ///< Time derivative of the Angular velocity Jacobian

        Eigen::Matrix4d getLocalSecondDeriv(const model3d::Dof *_q1, const model3d::Dof *_q2) const;
        void evalJacDerivLin(int _qi);    ///< Evaluate the first derivatives of the linear Jacobian wrt to the dependent dofs
        void evalJacDerivAng(int _qi);    ///< Evaluate the first derivatives of the angular Jacobian wrt to the dependent dofs
        void evalJacDotLin(const Eigen::VectorXd &_qDotSkel); ///< Evaluate time derivative of the linear Jacobian of this body node (num cols == num dependent dofs)
        void evalJacDotAng(const Eigen::VectorXd &_qDotSkel); ///< Evaluate time derivative of the angular Jacobian of this body node (num cols == num dependent dofs)

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_BODYNODE_DYNAMICS_H
