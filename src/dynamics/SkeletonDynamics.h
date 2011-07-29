/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/21/2011
*/

#ifndef DYNAMICS_SKELETONDYNAMICS_H
#define DYNAMICS_SKELETONDYNAMICS_H

#include <vector>
#include <Eigen/Dense>
#include "model3d/Skeleton.h"

namespace dynamics{

    class SkeletonDynamics : public model3d::Skeleton{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SkeletonDynamics();
        virtual ~SkeletonDynamics();

        virtual model3d::BodyNode* createBodyNode(const char* const _name = NULL);

        // inverse dynamics computation
        Eigen::VectorXd computeInverseDynamicsLinear(const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot=NULL, bool _computeJacobians=true); ///< runs recursive inverse dynamics algorithm and returns the generalized forces; if qdd is NULL, it is treated as zero; also computes Jacobian Jv and Jw in iterative manner if the flag is true i.e. replaces updateFirstDerivatives of non-recursive dynamics

        
        Eigen::MatrixXd mM;    ///< Mass matrix for the skeleton
        Eigen::MatrixXd mC;    ///< Coriolis matrix for the skeleton
        Eigen::VectorXd mCvec;    ///< Coriolis vector for the skeleton == mC*qdot
        Eigen::VectorXd mG;    ///< Gravity vector for the skeleton
        Eigen::VectorXd mCg;   ///< combined coriolis and gravity term == C*qdot + g
        void computeDynamics(const Eigen::Vector3d &_gravity, const Eigen::VectorXd &_qdot, bool _useInvDynamics=true);  ///< compute equations of motion matrices/vectors: M, C/Cvec, g in M*qdd + C*qd + g; if _useInvDynamics==true, uses computeInverseDynamicsLinear to compute C*qd+g term directly; else uses expensive generic computation of non-recursive dynamics

        ///< Clamp joint rotations to the range of [-pi, pi]. 
        ///< It's particularly useful for exponential map because the system will become unstable if the exponential map rotaion is outside this range. For euler angles, the dof values can directly add or subtract 2*pi; for exponential map, once the rotation magnitude is changed, the velocity need to change accordingly to represent the same angular velocity 
        void clampRotation( Eigen::VectorXd& _q, Eigen::VectorXd& _qdot);


    protected:

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_SKELETONDYNAMICS_H
