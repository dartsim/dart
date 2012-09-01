/**
  * @file loaderUtils.h
  * @author A.H. - C.E.
  */
#ifndef __GOLEMS_LOADER_UTILS_H__
#define __GOLEMS_LOADER_UTILS_H__

#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

// To load Mesh and Skel
#include  <kinematics/Joint.h>
#include <kinematics/ShapeMesh.h>
#include <geometry/Mesh3DTriangle.h>
#include <kinematics/Transformation.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/TrfmRotateEuler.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <robotics/Constants.h>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

// Type of DOF Enum
enum TypeOfDOF {
	GOLEM_X, GOLEM_Y, GOLEM_Z, GOLEM_ROLL, GOLEM_PITCH, GOLEM_YAW
};


void add_XyzRpy(kinematics::Joint* _joint, double _x, double _y, double _z, double _rr, double _rp, double _ry);

void add_DOF(dynamics::SkeletonDynamics* _skel, kinematics::Joint* _joint, double _val, double _min, double _max,
		int _DOF_TYPE);

void add_Shape(dynamics::BodyNodeDynamics* _node, const char *_meshObjPath, double _mass,
		Eigen::Matrix3d _inertiaMatrix);

#endif  /** __GOLEMS_LOADER_UTILS_H__ */
