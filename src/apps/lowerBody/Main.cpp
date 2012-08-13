/**
 * @file Main.cpp
 * @author A. Huaman and C. Erdogan
 * @date 2012-08-12
 */

#include "MyWindow.h"
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

// To Load Left and Right Leg
#include "legsLoader.h"
#include "loaderUtils.h"


/**
 * @function main
 */
int main(int argc, char* argv[]) {

	srand(time(NULL));

	//-- Create lowerBody skeleton
	dynamics::SkeletonDynamics LowerBodySkel;

	// Pointers to be used during the Skeleton building
	kinematics::Joint* HipJoint;
	dynamics::BodyNodeDynamics* HipNode;

	double x, y, z, roll, pitch, yaw;  // For Local rigid transformations
	double val;  // For DOF
	double vmin;
	double vmax;
	Eigen::Matrix3d inertiaMatrix;
	double mass;

	//-- *****  BodyNode 0: Hip (LHY) *****
	HipNode = (dynamics::BodyNodeDynamics*) LowerBodySkel.createBodyNode("Body_Hip");
	HipJoint = new kinematics::Joint(NULL,  HipNode, "Body_Hip");

	// Add rigids displacement (local)
	x = 0;
	y = 0;
	z = 0;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy( HipJoint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF( &LowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape( HipNode, DART_DATA_PATH"obj/Body_Hip.obj", mass, inertiaMatrix);

	// Add node to Skel
	LowerBodySkel.addNode(HipNode);

  // Load Left Leg
 loadLeftLeg( HipNode, &LowerBodySkel );

  // Load Right Leg
 loadRightLeg( HipNode, &LowerBodySkel );

	// ********** END, NOW INITIALIZE THIS GUY *********
	//-- Initialize mySkeleton
	LowerBodySkel.initSkel();

	// Verify that our skeleton has something inside :)
	printf("Our LowerBodySkel  has %d nodes \n", LowerBodySkel .getNumNodes());
	printf("Our LowerBodySkel  has %d joints \n", LowerBodySkel .getNumJoints());
	printf("Our LowerBodySkel  has %d DOFs \n", LowerBodySkel .getNumDofs());

	MyWindow window(&LowerBodySkel, NULL);

	cout << "space bar: simulation on/off" << endl;
	cout << "'s': simulate one step" << endl;
	cout << "'p': playback/stop" << endl;
	cout << "'[' and ']': play one frame backward and forward" << endl;
	cout << "'v': visualization on/off" << endl;
	cout << "'1' and '2': programmed interaction" << endl;

	glutInit(&argc, argv);
	window.initWindow(640, 480, "Cubes");
	glutMainLoop();

	return 0;
}


