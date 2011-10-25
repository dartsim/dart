#ifndef COLLISIONPHYSICS_H_
#define COLLISIONPHYSICS_H_

#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/traversal_node_bvhs.h>
#include <fcl/simple_setup.h>
#include "CollisionWrapper.h"
#include "CollisionMesh.h"

struct CollisionPair {
	fcl::MeshCollisionTraversalNodeOBB node;
	CollisionMesh *mesh1;
	CollisionMesh *mesh2;

	CollisionPair() {}

	~CollisionPair() {}
};

class CollisionPhysics {

	std::vector<CollisionMesh> mCollisionMeshs;
	std::vector<CollisionPair> mCollisionPairs;

public:
	CollisionPhysics() {

	}

	~CollisionPhysics() {

	}

	int addCollisionMesh(CollisionMesh _cMesh) {
		mCollisionMeshs.push_back(_cMesh);
		for (int i = 0; i < mCollisionMeshs.size() - 1; i++) {
			CollisionPair cpair;
			cpair.mesh1 = &(mCollisionMeshs[mCollisionMeshs.size() - 1]);
			cpair.mesh2 = &(mCollisionMeshs[i]);

			if (!fcl::initialize(cpair.node,
					(const fcl::BVHModel<fcl::OBB>&) cpair.mesh1->mBVHModel,
					(const fcl::BVHModel<fcl::OBB>&) cpair.mesh2->mBVHModel))
				return 0;

			cpair.node.enable_statistics = false; //verbose
			cpair.node.num_max_contacts = -1;
			cpair.node.exhaustive = true;
			cpair.node.enable_contact = true;

			mCollisionPairs.push_back(cpair);
		}

		return 1;
	}

	void checkCollisions() {
		for(int i = 0; i < mCollisionPairs.size(); i++) {
			fcl::collide(&(mCollisionPairs[i].node));

			if(mCollisionPairs[i].node.pairs.size() > 0) {
				mCollisionPairs[i].mesh1->setCollisionState(CollisionMesh::COLLISION);
				mCollisionPairs[i].mesh2->setCollisionState(CollisionMesh::COLLISION);
			}
			else {
				mCollisionPairs[i].mesh1->setCollisionState(CollisionMesh::COLLISION_FREE);
				mCollisionPairs[i].mesh2->setCollisionState(CollisionMesh::COLLISION_FREE);
			}
		}
	}
};

#endif /* COLLISIONPHYSICS_H_ */
