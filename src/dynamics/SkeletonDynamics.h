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
        Eigen::VectorXd computeInverseDynamicsLinear(const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot=NULL); ///< runs recursive inverse dynamics algorithm and returns the generalized forces; if qdd is NULL, it is treated as zero

        
        Eigen::MatrixXd mM;    ///< Mass matrix for the skeleton
        Eigen::MatrixXd mC;    ///< Coriolis matrix for the skeleton
        Eigen::VectorXd mCvec;    ///< Coriolis vector for the skeleton == mC*qdot
        Eigen::VectorXd mG;    ///< Gravity vector for the skeleton
        Eigen::VectorXd mCg;   ///< combined coriolis and gravity term == C*qdot + g
        void SkeletonDynamics::computeDynamics(const Eigen::Vector3d &_gravity, const Eigen::VectorXd &_qdot, bool _useInvDynamics=true);  // compute equations of motion matrices/vectors: M, C/Cvec, g in M*qdd + C*qd + g; if _useInvDynamics==true, uses computeInverseDynamicsLinear to compute C*qd+g term directly; else uses expensive generic computation

    protected:

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_SKELETONDYNAMICS_H
