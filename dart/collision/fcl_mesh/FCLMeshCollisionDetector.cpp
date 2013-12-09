#include <algorithm>
#include <cmath>
#include <fcl/collision.h>

#include "dart/renderer/LoadOpengl.h"
#include "dart/math/Helpers.h"

#include "dart/dynamics/BodyNode.h"

#include "dart/collision/CollisionNode.h"
#include "dart/collision/fcl_mesh/CollisionShapes.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionNode.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"

namespace dart {
namespace collision {

FCLMeshCollisionDetector::~FCLMeshCollisionDetector()
{
}


CollisionNode*FCLMeshCollisionDetector::createCollisionNode(
        dynamics::BodyNode* _bodyNode)
{
    return new FCLMeshCollisionNode(_bodyNode);
}

bool FCLMeshCollisionDetector::detectCollision(bool _checkAllCollisions,
                                               bool _calculateContactPoints)
{
    clearAllContacts();
    bool collision = false;

    FCLMeshCollisionNode* FCLMeshCollisionNode1 = NULL;
    FCLMeshCollisionNode* FCLMeshCollisionNode2 = NULL;

    for (int i = 0; i < mCollisionNodes.size(); i++)
        mCollisionNodes[i]->getBodyNode()->setColliding(false);

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        FCLMeshCollisionNode1 = static_cast<FCLMeshCollisionNode*>(mCollisionNodes[i]);

        for (int j = i + 1; j < mCollisionNodes.size(); j++)
        {
            FCLMeshCollisionNode2 = static_cast<FCLMeshCollisionNode*>(mCollisionNodes[j]);

            if (!isCollidable(FCLMeshCollisionNode1, FCLMeshCollisionNode2))
                continue;
            
            if(FCLMeshCollisionNode1->detectCollision(FCLMeshCollisionNode2,
                    _calculateContactPoints ? &mContacts : NULL,
                    mNumMaxContacts))
            {
                collision = true;
                mCollisionNodes[i]->getBodyNode()->setColliding(true);
                mCollisionNodes[j]->getBodyNode()->setColliding(true);

                if(!_checkAllCollisions)
                    return true;
            }
        }
    }

    return collision;
}

bool FCLMeshCollisionDetector::detectCollision(CollisionNode* _node1,
                                               CollisionNode* _node2,
                                               bool _calculateContactPoints)
{
    FCLMeshCollisionNode* collisionNode1 =
            static_cast<FCLMeshCollisionNode*>(_node1);
    FCLMeshCollisionNode* collisionNode2 =
            static_cast<FCLMeshCollisionNode*>(_node2);
    return collisionNode1->detectCollision(
                collisionNode2,
                _calculateContactPoints ? &mContacts : NULL,
                mNumMaxContacts);
}

void FCLMeshCollisionDetector::draw()
{
    for(int i = 0; i < mCollisionNodes.size(); i++)
        static_cast<FCLMeshCollisionNode*>(
                mCollisionNodes[i])->drawCollisionSkeletonNode();
}

} // namespace collision
} // namespace dart
