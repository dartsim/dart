#ifndef COLLISIONPHYSICS_H_
#define COLLISIONPHYSICS_H_

#include <fcl/traversal_node_bvhs.h>
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
	CollisionPhysics() {}
	~CollisionPhysics() {}

	int addCollisionMesh(CollisionMesh _cMesh);
	void checkCollisions();
};

#endif /* COLLISIONPHYSICS_H_ */
