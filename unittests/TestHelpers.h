/**
 * @file TestHelper.h
 * @author Can Erdogan
 * @date Feb 03, 2013
 * @brief Contains the helper functions for the tests.
 */

#pragma once

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"
#include "kinematics/ShapeBox.h"
#include "kinematics/Transformation.h"
#include "kinematics/TrfmTranslate.h"
#include "kinematics/TrfmRotateEuler.h"
#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Dense>

using namespace kinematics;
using namespace dynamics;
using namespace Eigen;

/// Function headers
enum TypeOfDOF {
	DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
};

/* ********************************************************************************************* */
/// Returns true if the two matrices are equal within the given bound
template <class MATRIX>
bool equals (const Eigen::DenseBase<MATRIX>& A, const Eigen::DenseBase<MATRIX>& B, double tol = 1e-5) {

	// Get the matrix sizes and sanity check the call
  const size_t n1 = A.cols(), m1 = A.rows();
  const size_t n2 = B.cols(), m2 = B.rows();
  if(m1!=m2 || n1!=n2) return false;

	// Check each index
  for(size_t i=0; i<m1; i++) {
    for(size_t j=0; j<n1; j++) {
      if(boost::math::isnan(A(i,j)) ^ boost::math::isnan(B(i,j)))
        return false;
      else if(fabs(A(i,j) - B(i,j)) > tol)
        return false;
    }
	}

	// If no problems, the two matrices are equal
  return true;
}

/* ********************************************************************************************* */
/// Add a transformation to a joint
void add_XyzRpy(Joint* joint, double x, double y, double z, double rr, double rp, double ry) {

	Transformation* trans;
	trans = new TrfmTranslate(new Dof(x), new Dof(y), new Dof(z), "Translate");
	joint->addTransform(trans, false);
	trans = new TrfmRotateEulerZ(new ::Dof(ry));
	joint->addTransform(trans, false);
	trans = new TrfmRotateEulerY(new ::Dof(rp));
	joint->addTransform(trans, false);
	trans = new TrfmRotateEulerX(new ::Dof(rr));
	joint->addTransform(trans, false);
}

/* ********************************************************************************************* */
/// Add a DOF to a given joint
void add_DOF(SkeletonDynamics* skel, Joint* joint, double val, double min, double max, int type) {

	// Create the transformation based on the type
	Transformation* trans;
	if(type == DOF_X) trans = new TrfmTranslateX(new Dof(0, "rootX"), "Tx");
	else if(type == DOF_Y) trans = new TrfmTranslateY(new Dof(0, "rootY"), "Ty");
	else if(type == DOF_Z) trans = new TrfmTranslateZ(new Dof(0, "rootZ"), "Tz");
	else if(type == DOF_YAW) trans = new TrfmRotateEulerZ(new Dof(0, "rootYaw"), "Try");
	else if(type == DOF_PITCH) trans = new TrfmRotateEulerY(new Dof(0, "rootPitch"), "Trp");
	else if(type == DOF_ROLL) trans = new TrfmRotateEulerX(new Dof(0, "rootRoll"), "Trr");
	// Add the transformation to the joint, set the min/max values and set it to the skeleton
	joint->addTransform(trans, true);
	joint->getDof(0)->setMin(min);
	joint->getDof(0)->setMax(max);
	skel->addTransform(trans);
}

/* ********************************************************************************************* */
/// Add an end-effector to the last link of the given robot
void addEndEffector (SkeletonDynamics* robot, BodyNodeDynamics* parent_node, Vector3d dim) {

	// Create the end-effector node with a random dimension
	BodyNodeDynamics* node = (BodyNodeDynamics*) robot->createBodyNode("ee");
	Joint* joint = new Joint(parent_node, node, "eeJoint");
	add_XyzRpy(joint, 0.0, 0.0, dim(2), 0.0, 0.0, 0.0);
	Shape* shape = new ShapeBox(Vector3d(0.2, 0.2, 0.2));
	node->setLocalCOM(Vector3d(0.0, 0.0, 0.0));
	node->setMass(1.0);
	node->addVisualizationShape(shape);
	node->addCollisionShape(shape);
	robot->addNode(node);
}

/* ********************************************************************************************* */
/// Creates a two link manipulator with the given dimensions where the first link rotates around
/// z-axis and second rotates around x in the zero configuration.
SkeletonDynamics* createTwoLinkRobot (Vector3d dim1, TypeOfDOF type1, Vector3d dim2, TypeOfDOF type2, bool 
		unfinished = false) {

	SkeletonDynamics* robot = new SkeletonDynamics();

	// Create the first link, the joint with the ground and its shape
	double mass = 1.0;
	BodyNodeDynamics* node = (BodyNodeDynamics*) robot->createBodyNode("link1");
	Joint* joint = new Joint(NULL, node, "joint1");
	add_XyzRpy(joint, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	add_DOF(robot, joint, 0.0, -M_PI, M_PI, type1);
	Shape* shape = new ShapeBox(dim1);
	node->setLocalCOM(Vector3d(0.0, 0.0, dim1(2)/2.0));
	node->setMass(mass);
	node->addVisualizationShape(shape);
	node->addCollisionShape(shape);
	robot->addNode(node);

	// Create the second link, the joint with link1 and its shape
	BodyNodeDynamics* parent_node = (BodyNodeDynamics*) robot->getNode("link1");
	node = (BodyNodeDynamics*) robot->createBodyNode("link2");
	joint = new Joint(parent_node, node, "joint2");
	add_XyzRpy(joint, 0.0, 0.0, dim1(2), 0.0, 0.0, 0.0);
	add_DOF(robot, joint, 0.0, -M_PI, M_PI, type2);
	shape = new ShapeBox(dim2);
	node->setLocalCOM(Vector3d(0.0, 0.0, dim2(2)/2.0));
	node->addVisualizationShape(shape);
	node->addCollisionShape(shape);
	node->setMass(mass);
	robot->addNode(node);

	// If finished, initialize the skeleton
	if(!unfinished) {
		addEndEffector(robot, node, dim2);
		robot->initSkel();
	}
	return robot;
}

/* ********************************************************************************************* */
/// Creates a two link manipulator with the given dimensions where the first link rotates around
/// z-axis and second rotates around x in the zero configuration.
SkeletonDynamics* createThreeLinkRobot (Vector3d dim1, TypeOfDOF type1, Vector3d dim2, TypeOfDOF type2, 
		Vector3d dim3, TypeOfDOF type3, bool unfinished = false) {

	SkeletonDynamics* robot = new SkeletonDynamics();

	// Create the first link, the joint with the ground and its shape
	double mass = 1.0;
	BodyNodeDynamics* node = (BodyNodeDynamics*) robot->createBodyNode("link1");
	Joint* joint = new Joint(NULL, node, "joint1");
	add_XyzRpy(joint, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	add_DOF(robot, joint, 0.0, -M_PI, M_PI, type1);
	Shape* shape = new ShapeBox(dim1);
	node->setLocalCOM(Vector3d(0.0, 0.0, dim1(2)/2.0));
	node->addVisualizationShape(shape);
	node->addCollisionShape(shape);
	node->setMass(mass);
	robot->addNode(node);

	// Create the second link, the joint with link1 and its shape
	BodyNodeDynamics* parent_node = (BodyNodeDynamics*) robot->getNode("link1");
	node = (BodyNodeDynamics*) robot->createBodyNode("link2");
	joint = new Joint(parent_node, node, "joint2");
	add_XyzRpy(joint, 0.0, 0.0, dim1(2), 0.0, 0.0, 0.0);
	add_DOF(robot, joint, 0.0, -M_PI, M_PI, type2);
	shape = new ShapeBox(dim2);
	node->setLocalCOM(Vector3d(0.0, 0.0, dim2(2)/2.0));
	node->addVisualizationShape(shape);
	node->addCollisionShape(shape);
	node->setMass(mass);
	robot->addNode(node);

	// Create the third link, the joint with link2 and its shape
	parent_node = (BodyNodeDynamics*) robot->getNode("link2");
	node = (BodyNodeDynamics*) robot->createBodyNode("link3");
	joint = new Joint(parent_node, node, "joint3");
	add_XyzRpy(joint, 0.0, 0.0, dim2(2), 0.0, 0.0, 0.0);
	add_DOF(robot, joint, 0.0, -M_PI, M_PI, type3);
	shape = new ShapeBox(dim3);
	node->setLocalCOM(Vector3d(0.0, 0.0, dim3(2)/2.0));
	node->addVisualizationShape(shape);
	node->addCollisionShape(shape);
	node->setMass(mass);
	robot->addNode(node);

	// If finished, initialize the skeleton
	if(!unfinished) {
		addEndEffector(robot, node, dim3);
		robot->initSkel();
	}
	return robot;
}
