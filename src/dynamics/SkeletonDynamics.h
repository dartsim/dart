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

        Eigen::VectorXd inverseDynamics(const Eigen::Vector3d &_gravity, const Eigen::VectorXd *_qdot, const Eigen::VectorXd *_qdotdot=NULL); ///< runs recursive inverse dynamics algorithm and returns the generalized forces; if qdd is NULL, it is treated as zero

    protected:

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_SKELETONDYNAMICS_H
