#ifndef COLLISIONMESH_H_
#define COLLISIONMESH_H_

#include <fcl/BVH_model.h>
#include <vector>

class CollisionMesh {

private:
	int mCollisionState;

public:

	enum {COLLISION_FREE=0, COLLISION=1};
	fcl::BVHModel<fcl::OBB> mBVHModel;

	CollisionMesh();
	~CollisionMesh();

	void setCollisionState(int _state);
	int getCollisionState();

	void setMesh(Model3DS _model3DS);
	void setTransform(Eigen::Transform< double, 3,Eigen::Affine > &_tf);
};

#endif /* COLLISIONMESH_H_ */

