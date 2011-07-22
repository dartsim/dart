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
#include "renderer/RenderInterface.h"
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

    };

} // namespace dynamics

#endif // #ifndef DYNAMICS_BODYNODE_DYNAMICS_H
