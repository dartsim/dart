/**
   * @file legsLoader.h
   */
#ifndef __GOLEMS_LEGS_LOADER_H__
#define __GOLEMS_LEGS_LOADER_H__

#include "loaderUtils.h"

//using namespace std;
//using namespace Eigen;
//using namespace kinematics;
//using namespace dynamics;

void loadLeftLeg( dynamics::BodyNodeDynamics* _parent_node, dynamics::SkeletonDynamics* _lowerBodySkel, Eigen::VectorXd _offset );
void loadRightLeg( dynamics::BodyNodeDynamics* _parent_node, dynamics::SkeletonDynamics* _lowerBodySkel, Eigen::VectorXd _offset );

#endif  /** __GOLEMS_LEGS_LOADER_H__ */
