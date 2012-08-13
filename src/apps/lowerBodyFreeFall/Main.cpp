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

// Function declaration
  void loadLowerBody( dynamics::SkeletonDynamics* _lowerBodySkel );

/**
 * @function main
 */
int main(int argc, char* argv[]) {

	srand(time(NULL));

  //-- Add ground Skeleton
  FileInfoSkel<SkeletonDynamics> GroundSkel;
  GroundSkel.loadFile(DART_DATA_PATH"/skel/ground1.skel", kinematics::SKEL);

	//-- Create lowerBody skeleton with X, Y, Z degrees of freedom
	dynamics::SkeletonDynamics LowerBodySkel;
  loadLowerBody( &LowerBodySkel );

  // OpenGL Visualization
	MyWindow window( (dynamics::SkeletonDynamics*) GroundSkel.getSkel(), &LowerBodySkel,  NULL);

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

/**
  * @function loadLowerBody
  */
  void loadLowerBody( dynamics::SkeletonDynamics* _lowerBodySkel ) {

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
	HipNode = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("Body_Hip");
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
	vmin = -1000;
	vmax = 1000;
	add_DOF( _lowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_X);
	add_DOF( _lowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_Y);
	add_DOF( _lowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_Z);
	add_DOF( _lowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_ROLL);
	add_DOF( _lowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_PITCH);
	add_DOF( _lowerBodySkel, HipJoint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0.022766841, 0.000053624, -0.000164219, 0.000053624, 0.009418222, -0.000010039, -0.000164219, -0.000010039, 0.026610520;
	mass = 4.436339231;
	add_Shape( HipNode, DART_DATA_PATH"obj/Body_Hip.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(HipNode);

   // Offset of Left Leg w.r.t. Hip
   Eigen::VectorXd leftLegOffset(6);
   leftLegOffset << 0.0, -0.085, -0.08, 0, 0, 0;

   // Offset of Right Leg w.r.t. Hip
   Eigen::VectorXd rightLegOffset(6);
   rightLegOffset << 0.0, -0.085, 0.08, 0, 0, 0;


  // Load Left Leg
 loadLeftLeg( HipNode, _lowerBodySkel, leftLegOffset );

  // Load Right Leg
 loadRightLeg( HipNode, _lowerBodySkel, rightLegOffset );

	// ********** END, NOW INITIALIZE THIS GUY *********
	//-- Initialize mySkeleton
	_lowerBodySkel->initSkel();

	// Verify that our skeleton has something inside :)
	printf("Our LowerBodySkel  has %d nodes \n", _lowerBodySkel->getNumNodes());
	printf("Our LowerBodySkel  has %d joints \n", _lowerBodySkel->getNumJoints());
	printf("Our LowerBodySkel  has %d DOFs \n", _lowerBodySkel->getNumDofs());

  }

