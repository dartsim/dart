/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author  Sumit Jain
    Date    07/21/2011
*/

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

namespace dynamics{
    SkeletonDynamics::SkeletonDynamics(): model3d::Skeleton(){
    }

    SkeletonDynamics::~SkeletonDynamics(){
    }

    model3d::BodyNode* SkeletonDynamics::createBodyNode(const char* const _name){
        return new BodyNodeDynamics(_name);
    }



}   // namespace dynamics
