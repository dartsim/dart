/**
 * @file loaderUtils.cpp
 */

#include "loaderUtils.h"

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

	static bool const debug = false;

	// Load a Mesh3DTriangle to save in Shape
	geometry::Mesh3DTriangle* m3d = new geometry::Mesh3DTriangle();
	bool success = m3d->readMesh(_meshObjPath, geometry::Mesh3D::OBJ);

	kinematics::ShapeMesh *shape = new kinematics::ShapeMesh(Eigen::Vector3d(1, 1, 1), 0.0, m3d);
	_node->setShape(shape);

	if(debug) {
		printf("[AddShape] -- Status of  reading MESH: Reading mesh result was: %d \n", success);
		printf("Num of Vertices is: %d \n", m3d->mNumVertices);
		printf("Num of  Faces is: %d \n", m3d->mNumFaces);
		printf("Num of  vertex normals is: %lu \n", m3d->mVertexNormals.size());
		printf("Num of  Faces vector is: %lu \n", m3d->mFaces.size());
		printf("---------------------------------------------------:D \n");
	}

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

