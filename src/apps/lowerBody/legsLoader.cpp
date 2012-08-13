/**
  * @file legsLoader.cpp
  */
#include "legsLoader.h"

/**
  * @function loadLeftLeg
  */
void loadLeftLeg( dynamics::BodyNodeDynamics* _parent_node, dynamics::SkeletonDynamics* _lowerBodySkel ) {

	// Pointers to be used during the left Leg building
	kinematics::Joint* joint;
	dynamics::BodyNodeDynamics* node;
	dynamics::BodyNodeDynamics* parent_node;

	double x, y, z, roll, pitch, yaw;  // For Local rigid transformations
	double val;  // For DOF
	double vmin;
	double vmax;
	Eigen::Matrix3d inertiaMatrix;
	double mass;

	//-- *****  BodyNode 1: Left Hip Yaw (LHY)  Parent: Input Argument: _parent_node*****
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("LHY");
	joint = new kinematics::Joint( _parent_node, node, "LHY");

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
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LHY.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 2: Left Hip Roll (LHR)  whose parent is: LHY*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("LHY");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("LHR");
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
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_ROLL);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LHR.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("LHR");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("LHP");
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
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LHP.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 4: Left Knee Pitch (LKP) whose parent is: LHP*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("LHP");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("LKP");
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
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LKP.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 5: Left Ankle Pitch (LAP) whose parent is: LKP*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("LKP");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("LAP");
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
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LAP.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 6: Left Ankle Roll (LAR) whose parent is: LAP*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("LAP");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("LAR");
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
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_ROLL);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/leftLeg/Body_LAR.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

}

/**
  * @function loadRightLeg
  */
void loadRightLeg( dynamics::BodyNodeDynamics* _parent_node, dynamics::SkeletonDynamics* _lowerBodySkel ) {

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

	//-- *****  BodyNode 1: Right Hip Yaw (RHY) *****
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("RHY");
	joint = new kinematics::Joint(_parent_node, node, "RHY");

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
	add_DOF(_lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/rightLeg/Body_RHY.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 2: Right Hip Roll (RHR)  whose parent is: LHY*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("RHY");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("RHR");
	joint = new kinematics::Joint(parent_node, node, "RHR");

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
	add_DOF(_lowerBodySkel, joint, val, vmin, vmax, GOLEM_ROLL);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/rightLeg/Body_RHR.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 3: Right Hip Pitch (RHP) whose parent is: LHR*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("RHR");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("RHP");
	joint = new kinematics::Joint(parent_node, node, "RHP");

	// Add rigids displacement (local)
	x = 0;
	y = 0;
	z = -0.054;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(_lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/rightLeg/Body_RHP.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 4: Right Knee Pitch (LKP) whose parent is: LHP*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("RHP");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("RKP");
	joint = new kinematics::Joint(parent_node, node, "RKP");

	// Add rigids displacement (local)
	x = -0.00167500000000;
	y = -0.300542000000000;
	z = 0.074999000000000;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF(_lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/rightLeg/Body_RKP.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 5: Right Ankle Pitch (RAP) whose parent is: RKP*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("RKP");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("RAP");
	joint = new kinematics::Joint(parent_node, node, "RAP");

	// Add rigids displacement (local)
	x = 0.000045000000000;
	y = -0.299942000000000;
	z = 0.024755000000000;
	roll = 0;
	pitch = 0;
	yaw = 0;
//  -0.000045000000000  -0.024755000000000   0.299942000000000

	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_YAW);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/rightLeg/Body_RAP.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);

	//-- ***** BodyNode 6: Right Ankle Roll (RAR) whose parent is: LAP*****
	parent_node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->getNode("RAP");
	node = (dynamics::BodyNodeDynamics*) _lowerBodySkel->createBodyNode("RAR");
	joint = new kinematics::Joint(parent_node, node, "RAR");

	// Add rigids displacement (local)
	x = 0.07124400000000;
	y = -0.000016000000000;
	z = -0.046500000000000;
	roll = 0;
	pitch = 0;
	yaw = 0;
	add_XyzRpy(joint, x, y, z, roll, pitch, yaw);

	// Add DOF
	val = 0;
	vmin = 0;
	vmax = PI;
	add_DOF( _lowerBodySkel, joint, val, vmin, vmax, GOLEM_ROLL);

	// Add Shape
	inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	mass = 1.0;
	add_Shape(node, DART_DATA_PATH"obj/rightLeg/Body_RAR.obj", mass, inertiaMatrix);

	// Add node to Skel
	_lowerBodySkel->addNode(node);


}
