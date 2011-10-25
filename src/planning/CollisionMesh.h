#ifndef COLLISIONMESH_H_
#define COLLISIONMESH_H_

#include <fcl/BVH_model.h>
#include "CollisionWrapper.h"
#include <vector>

class CollisionMesh {

private:
	int mCollisionState;

public:

	enum {COLLISION_FREE=0, COLLISION=1};
	fcl::BVHModel<fcl::OBB> mBVHModel;

	CollisionMesh() {
		// TODO: Set properties globally using CollisionMeshProperties
		mBVHModel.bv_splitter = new fcl::BVSplitter<fcl::OBB>(fcl::SPLIT_METHOD_MEAN);
	}

	~CollisionMesh() {

	}

	void setCollisionState(int _state) {
		mCollisionState = _state;
	}

	int getCollisionState() {
		return mCollisionState;
	}

	void setMesh(Model3DS _model3DS) {
		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;

		CollisionWrapper::getModel_3DSToFCL(_model3DS, &triangles, &vertices);

		mBVHModel.beginModel();
		mBVHModel.addSubModel(vertices, triangles);
		mBVHModel.endModel();
	}

	void setTransform(Eigen::Transform< double, 3,Eigen::Affine > _tf) {
		fcl::Vec3f R[3];
		fcl::Vec3f T;

		T = CollisionWrapper::getTrans_EigenToFCL(_tf);
		R[0] = CollisionWrapper::getRotation_EigenToFCL(_tf, 0);
		R[1] = CollisionWrapper::getRotation_EigenToFCL(_tf, 1);
		R[2] = CollisionWrapper::getRotation_EigenToFCL(_tf, 2);

		mBVHModel.setTransform(R, T);
	}
};

#endif /* COLLISIONMESH_H_ */

