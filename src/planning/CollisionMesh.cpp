#include <fcl/simple_setup.h>
#include "Model3DS.h"
#include "CollisionMesh.h"
#include "CollisionWrapper.h"

CollisionMesh::CollisionMesh() {
	// TODO: Set properties globally using CollisionMeshProperties
	mBVHModel.bv_splitter = new fcl::BVSplitter<fcl::OBB>(fcl::SPLIT_METHOD_MEAN);
}

CollisionMesh::~CollisionMesh() {

}

void CollisionMesh::setCollisionState(int _state) {
	mCollisionState = _state;
}

int CollisionMesh::getCollisionState() {
	return mCollisionState;
}

void CollisionMesh::setMesh(Model3DS _model3DS) {
	std::vector<fcl::Vec3f> vertices;
	std::vector<fcl::Triangle> triangles;

	CollisionWrapper::getModel_3DSToFCL(_model3DS, &triangles, &vertices);

	mBVHModel.beginModel();
	mBVHModel.addSubModel(vertices, triangles);
	mBVHModel.endModel();
}

void CollisionMesh::setTransform(Eigen::Transform< double, 3,Eigen::Affine > _tf) {
	fcl::Vec3f R[3];
	fcl::Vec3f T;

	T = CollisionWrapper::getTrans_EigenToFCL(_tf);
	R[0] = CollisionWrapper::getRotation_EigenToFCL(_tf, 0);
	R[1] = CollisionWrapper::getRotation_EigenToFCL(_tf, 1);
	R[2] = CollisionWrapper::getRotation_EigenToFCL(_tf, 2);

	mBVHModel.setTransform(R, T);
}
