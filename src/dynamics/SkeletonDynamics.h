/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit jain
  Date		07/21/2011
*/

#ifndef DYNAMICS_SKELETONDYNAMICS_H
#define DYNAMICS_SKELETONDYNAMICS_H

#include <vector>
#include <Eigen/Dense>
#include "renderer/RenderInterface.h"
#include "model3d/Skeleton.h"

namespace dynamics{

    class SkeletonDynamics : public model3d::Skeleton{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SkeletonDynamics();
        virtual ~SkeletonDynamics();

        virtual model3d::BodyNode* createBodyNode(const char* const _name = NULL);

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_SKELETONDYNAMICS_H
