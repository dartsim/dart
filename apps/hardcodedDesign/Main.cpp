/**
 * @file Main.cpp
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#include "MyWindow.h"
#include <argp.h>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

/// Function headers
enum TypeOfDOF {
	DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
};

/* ********************************************************************************************* */
// Argp documentation
static char doc[] = "\nThis application shows the creation of a kinematic skeleton from scratch "
	"without the use of a model file. Run the program without arguments and you can use the buttons "
	"{1,2} to move the corresponding joints. The key '-' will make the joints move in the negative "
	"direction.\n\nCan Erdogan\nFeb 02, 2013";
static struct argp argp = {NULL, NULL, NULL, doc};

/* ********************************************************************************************* */
/// Add a transformation to a joint
void add_XyzRpy(Joint* joint, double x, double y, double z, double rr, double rp, double ry) {

	Transformation* trans;
	trans = new TrfmTranslate(new Dof(x), new Dof(y), new Dof(z), "Translate");
	joint->addTransform(trans, false);
	trans = new TrfmRotateEulerZ(new ::Dof(DEG2RAD(ry)));
	joint->addTransform(trans, false);
	trans = new TrfmRotateEulerY(new ::Dof(DEG2RAD(rp)));
	joint->addTransform(trans, false);
	trans = new TrfmRotateEulerX(new ::Dof(DEG2RAD(rr)));
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
	joint->getDof(0)->setMin(DEG2RAD(min));
	joint->getDof(0)->setMax(DEG2RAD(max));
	skel->addTransform(trans);
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Handle argp call
	argp_parse (&argp, argc, argv, 0, 0, NULL);

	// Create Left Leg skeleton
	SkeletonDynamics LeftLegSkel;

	// Pointers to be used during the Skeleton building
	Matrix3d inertiaMatrix;
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	double mass = 1.0;

	// ***** BodyNode 1: Left Hip Yaw (LHY) ***** *
	BodyNodeDynamics* node = (BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHY");
	Joint* joint = new Joint(NULL, node, "LHY");
	add_XyzRpy(joint, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	add_DOF(&LeftLegSkel, joint, 0.0, 0.0, PI, DOF_YAW);
	node->setShape(new ShapeBox(Vector3d(0.3, 0.3, 1.0), mass));
	LeftLegSkel.addNode(node);

	// ***** BodyNode 2: Left Hip Roll (LHR) whose parent is: LHY *****
	BodyNodeDynamics* parent_node = (BodyNodeDynamics*) LeftLegSkel.getNode("LHY");
	node = (BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHR");
	joint = new Joint(parent_node, node, "LHR");
	add_XyzRpy(joint, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
	add_DOF(&LeftLegSkel, joint, 0.0, 0.0, PI, DOF_ROLL);
	Shape* shape = new ShapeBox(Vector3d(0.3, 0.3, 1.0), mass);
	node->setLocalCOM(Vector3d(0.0, 0.0, 0.5));
	node->setShape(shape);
	LeftLegSkel.addNode(node);

	// ***** BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR *****
	parent_node = (BodyNodeDynamics*) LeftLegSkel.getNode("LHR");
	node = (BodyNodeDynamics*) LeftLegSkel.createBodyNode("LHP");
	joint = new Joint(parent_node, node, "LHP");
	add_XyzRpy(joint, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
	add_DOF(&LeftLegSkel, joint, 0.0, 0.0, PI, DOF_ROLL);
	shape = new ShapeBox(Vector3d(0.3, 0.3, 1.0), mass);
	node->setLocalCOM(Vector3d(0.0, 0.0, 0.5));
	node->setShape(shape);
	LeftLegSkel.addNode(node);

	// Initialize the skeleton
	LeftLegSkel.initSkel();

	// Window stuff
	MyWindow window(&LeftLegSkel);
	glutInit(&argc, argv);
	window.initWindow(640, 480, "Skeleton example");
	glutMainLoop();

	return 0;
}
