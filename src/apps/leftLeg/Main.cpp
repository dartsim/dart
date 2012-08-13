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

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

// Function headers
enum TypeOfDOF {
	GOLEM_X, GOLEM_Y, GOLEM_Z, GOLEM_ROLL, GOLEM_PITCH, GOLEM_YAW
};
void add_XyzRpy(kinematics::Joint* _joint, double _x, double _y, double _z, double _rr, double _rp, double _ry);
void add_DOF(dynamics::SkeletonDynamics* _skel, kinematics::Joint* _joint, double _val, double _min, double _max,
		int _DOF_TYPE);
void add_Shape(dynamics::BodyNodeDynamics* _node, const char *_meshObjPath, double _mass,
		Eigen::Matrix3d _inertiaMatrix);

/**
 * @function main
 */
int main(int argc, char* argv[]) {

	srand(time(NULL));

	// **********
	//-- Create Left Leg skeleton
	dynamics::SkeletonDynamics LeftLegSkel;

	// Pointers to be used during the Skeleton building
	kinematics::Joint* joint;
	dynamics::BodyNodeDynamics* node;
	dynamics::BodyNodeDynamics* parent_node;

	double x, y, z, roll, pitch, yaw;  // For Local rigid transformations
	double val;  // For DOF
	double vmin;
	double vmax;
	Eigen::Matrix3d inertiaMatrix;
	double mass;

	//-- *****  BodyNode 1: Left Hip Yaw (LHY) *****
	node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHY");
	joint = new kinematics::Joint(NULL, node, "LHY");

	// Add rigids displacement (local)
	x = 0;
	y = 0;
	z = 0;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(&LeftLegSkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0.000863427, 0.000001238, 0.000042545, 0.000001238, 0.002029641, 0.000000627, 0.000042545, 0.000000627, 0.001473105;
	mass = 0.483118742;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LHY.obj", mass, inertiaMatrix);

	// Add node to Skel
	LeftLegSkel.addNode(node);

	//-- ***** BodyNode 2: Left Hip Roll (LHR)  whose parent is: LHY*****
	parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode("LHY");
	node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHR");
	joint = new kinematics::Joint(parent_node, node, "LHR");

	// Add rigids displacement (local)
	roll = 0;
	pitch = 0;
	yaw = 0;
	x = 0.0;
	y = -0.091;
	z = 0.0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(&LeftLegSkel, joint, val, vmin, vmax, GOLEM_ROLL);

	// Add Shape
	inertiaMatrix << 0.004385578, 0.000224243, -0.000447015, 0.000224243, 0.004633453, -0.000377898, -0.000447015, -0.000377898, 0.004398642;
	mass = 2.644641101;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LHR.obj", mass, inertiaMatrix);

	// Add node to Skel
	LeftLegSkel.addNode(node);

	//-- ***** BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR*****
	parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode("LHR");
	node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHP");
	joint = new kinematics::Joint(parent_node, node, "LHP");

	// Add rigids displacement (local)
	x = 0;
	y = 0;
	z = 0.054;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(&LeftLegSkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0.036844807, 0.000255530, -0.000624748, 0.000255530, 0.034557057, 0.000952212, -0.000624748, 0.000952212, 0.009238020;
	mass = 3.098799394;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LHP.obj", mass, inertiaMatrix);

	// Add node to Skel
	LeftLegSkel.addNode(node);

	//-- ***** BodyNode 4: Left Knee Pitch (LKP) whose parent is: LHP*****
	parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode("LHP");
	node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LKP");
	joint = new kinematics::Joint(parent_node, node, "LKP");

	// Add rigids displacement (local)
	x = -0.00167500000000;
	y = -0.300542000000000;
	z = -0.074999000000000;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(&LeftLegSkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0.022011968, 0.000117575, -0.000867941, 0.000117575, 0.020299002, 0.002037181, -0.000867941, 0.002037181, 0.005045442;
	mass = 1.559604102;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LKP.obj", mass, inertiaMatrix);

	// Add node to Skel
	LeftLegSkel.addNode(node);

	//-- ***** BodyNode 5: Left Ankle Pitch (LAP) whose parent is: LKP*****
	parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode("LKP");
	node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LAP");
	joint = new kinematics::Joint(parent_node, node, "LAP");

	// Add rigids displacement (local)
	x = 0.000045000000000;
	y = -0.299942000000000;
	z = -0.024755000000000;
	roll = 0;
	pitch = 0;
	yaw = 0;
//  -0.000045000000000  -0.024755000000000   0.299942000000000

	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(&LeftLegSkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0.002342287, 0.000114531, 0.000316185, 0.000114531, 0.003019600, 0.000107388, 0.000316185, 0.000107388, 0.002846142;
	mass = 1.674918070;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LAP.obj", mass, inertiaMatrix);

	// Add node to Skel
	LeftLegSkel.addNode(node);

	//-- ***** BodyNode 6: Left Ankle Roll (LAR) whose parent is: LAP*****
	parent_node = (dynamics::BodyNodeDynamics*) LeftLegSkel.getNode("LAP");
	node = (dynamics::BodyNodeDynamics*) LeftLegSkel.createBodyNode("LAR");
	joint = new kinematics::Joint(parent_node, node, "LAR");

	// Add rigids displacement (local)
	x = 0.07124400000000;
	y = -0.000016000000000;
	z = 0.046500000000000;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(&LeftLegSkel, joint, val, vmin, vmax, GOLEM_ROLL);

	// Add Shape
	inertiaMatrix << 0.001299926, -0.000003599, -0.000096266, -0.000003599, 0.002701794, 0.000026851, -0.000096266, 0.000026851, 0.002753586;
	mass = 0.525134235;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LAR.obj", mass, inertiaMatrix);

	// Add node to Skel
	LeftLegSkel.addNode(node);

	// ********** END, NOW INITIALIZE THIS GUY *********
	//-- Initialize mySkeleton
	LeftLegSkel.initSkel();

	// Verify that our skeleton has something inside :)
	printf("Our LeftLeg has %d nodes \n", LeftLegSkel.getNumNodes());
	printf("Our LeftLeg has %d joints \n", LeftLegSkel.getNumJoints());
	printf("Our LeftLeg has %d DOFs \n", LeftLegSkel.getNumDofs());

	MyWindow window(&LeftLegSkel, NULL);

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
 * @function add_XyzRpy
 */
void add_XyzRpy(kinematics::Joint* _joint, double _x, double _y, double _z, double _rr, double _rp, double _ry) {

	kinematics::Transformation* trans;

	trans = new kinematics::TrfmTranslate(new kinematics::Dof(_x), new kinematics::Dof(_y), new kinematics::Dof(_z),
			"Translate");
	_joint->addTransform(trans, false);

	trans = new kinematics::TrfmRotateEulerZ(new ::kinematics::Dof(DEG2RAD(_ry)));
	_joint->addTransform(trans, false);

	trans = new kinematics::TrfmRotateEulerY(new ::kinematics::Dof(DEG2RAD(_rp)));
	_joint->addTransform(trans, false);

	trans = new kinematics::TrfmRotateEulerX(new ::kinematics::Dof(DEG2RAD(_rr)));
	_joint->addTransform(trans, false);
}

/**
 * @function add_DOF
 */
void add_DOF(dynamics::SkeletonDynamics* _skel, kinematics::Joint* _joint, double _val, double _min, double _max,
		int _DOF_TYPE) {

	kinematics::Transformation* trans;

	if(_DOF_TYPE == GOLEM_X) {
		trans = new kinematics::TrfmTranslateX(new kinematics::Dof(0, "rootX"), "Tx");
		_joint->addTransform(trans, true);
		_joint->getDof(0)->setMin(DEG2RAD(_min));
		_joint->getDof(0)->setMax(DEG2RAD(_max));
		_skel->addTransform(trans);
	}
	else if(_DOF_TYPE == GOLEM_Y) {
		trans = new kinematics::TrfmTranslateY(new kinematics::Dof(0, "rootY"), "Ty");
		_joint->addTransform(trans, true);
		_joint->getDof(0)->setMin(DEG2RAD(_min));
		_joint->getDof(0)->setMax(DEG2RAD(_max));
		_skel->addTransform(trans);
	}
	else if(_DOF_TYPE == GOLEM_Z) {
		trans = new kinematics::TrfmTranslateZ(new kinematics::Dof(0, "rootZ"), "Tz");
		_joint->addTransform(trans, true);
		_joint->getDof(0)->setMin(DEG2RAD(_min));
		_joint->getDof(0)->setMax(DEG2RAD(_max));
		_skel->addTransform(trans);
	}
	else if(_DOF_TYPE == GOLEM_YAW) {
		trans = new kinematics::TrfmRotateEulerZ(new kinematics::Dof(0, "rootYaw"), "Try");
		_joint->addTransform(trans, true);
		_joint->getDof(0)->setMin(DEG2RAD(_min));
		_joint->getDof(0)->setMax(DEG2RAD(_max));
		_skel->addTransform(trans);
	}
	else if(_DOF_TYPE == GOLEM_PITCH) {
		trans = new kinematics::TrfmRotateEulerY(new kinematics::Dof(0, "rootPitch"), "Trp");
		_joint->addTransform(trans, true);
		_joint->getDof(0)->setMin(DEG2RAD(_min));
		_joint->getDof(0)->setMax(DEG2RAD(_max));
		_skel->addTransform(trans);
	}
	else if(_DOF_TYPE == GOLEM_ROLL) {
		trans = new kinematics::TrfmRotateEulerX(new kinematics::Dof(0, "rootRoll"), "Trr");
		_joint->addTransform(trans, true);
		_joint->getDof(0)->setMin(DEG2RAD(_min));
		_joint->getDof(0)->setMax(DEG2RAD(_max));
		_skel->addTransform(trans);
	}
	else {
		printf(" WATCH OUT! THIS SHOULD NOT HAPPEN, NO DOF SET \n");
	}

}

/**
 * @function add_Shape
 */
void add_Shape(dynamics::BodyNodeDynamics* _node, const char *_meshObjPath, double _mass,
		Eigen::Matrix3d _inertiaMatrix) {

	kinematics::ShapeMesh *shape = new kinematics::ShapeMesh(Eigen::Vector3d(0, 0, 0), 0);
	_node->setShape(shape);

	// Load a Mesh3DTriangle to save in Shape
	geometry::Mesh3DTriangle* m3d = new geometry::Mesh3DTriangle();
	bool success = m3d->readMesh(_meshObjPath, geometry::Mesh3D::OBJ);
	printf("[AddShape] -- Status of  reading MESH: Reading mesh result was: %d \n", success);

	printf("Num of Vertices is: %d \n", m3d->mNumVertices);

	printf("Num of  Faces is: %d \n", m3d->mNumFaces);

	printf("Num of  vertex normals is: %lu \n", m3d->mVertexNormals.size());
	printf("Num of  Faces vector is: %lu \n", m3d->mFaces.size());
	printf("---------------------------------------------------:D \n");

	// Save Mesh3D in Shape (vizMesh)
	shape->setVizMesh(m3d);
	shape->setCollisionMesh(m3d);
	shape->setMass(_mass);
	shape->setInertia(_inertiaMatrix);

	// Set the color
	size_t r = rand() % 256, g = rand() % 256, b = rand() % 256;
	Eigen::Vector3d color;
	color << r / 255.0, g / 255.0, b / 255.0;
	shape->setColor(color);
}

