#include <algorithm>
#include <cmath>
#include <fcl/collision.h>

#include "renderer/LoadOpengl.h"
#include "math/UtilsMath.h"

#include "kinematics/BodyNode.h"

#include "collision/CollisionNode.h"
#include "collision/fcl_mesh/CollisionShapes.h"
#include "collision/fcl_mesh/FCLMESHCollisionNode.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"

using namespace std;
using namespace Eigen;
using namespace dart_math;

namespace collision
{

FCLMESHCollisionDetector::~FCLMESHCollisionDetector() {
}


CollisionNode*FCLMESHCollisionDetector::createCollisionNode(kinematics::BodyNode* _bodyNode)
{
    return new FCLMESHCollisionNode(_bodyNode);
}

bool FCLMESHCollisionDetector::checkCollision(bool _checkAllCollisions,
                                              bool _calculateContactPoints)
{
    int num_max_contact = 100;
    clearAllContacts();
    bool collision = false;

    FCLMESHCollisionNode* FCLMESHCollisionNode1 = NULL;
    FCLMESHCollisionNode* FCLMESHCollisionNode2 = NULL;

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        mCollisionNodes[i]->getBodyNode()->setColliding(false);
    }

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        FCLMESHCollisionNode1 = static_cast<FCLMESHCollisionNode*>(mCollisionNodes[i]);

        for (int j = i + 1; j < mCollisionNodes.size(); j++)
        {
            FCLMESHCollisionNode2 = static_cast<FCLMESHCollisionNode*>(mCollisionNodes[j]);

            if (!isCollidable(FCLMESHCollisionNode1, FCLMESHCollisionNode2))
            {
                continue;
            }
            
            if(FCLMESHCollisionNode1->checkCollision(FCLMESHCollisionNode2,
                    _calculateContactPoints ? &mContacts : NULL,
                    num_max_contact))
            {
                collision = true;
                mCollisionNodes[i]->getBodyNode()->setColliding(true);
                mCollisionNodes[j]->getBodyNode()->setColliding(true);

                if(!_checkAllCollisions)
                {
                    return true;
                }
            }
        }
    }

    return collision;
}


bool FCLMESHCollisionDetector::checkCollision(CollisionNode* _node1,
                                              CollisionNode* _node2,
                                              bool _calculateContactPoints)
{
    FCLMESHCollisionNode* collisionNode1 = static_cast<FCLMESHCollisionNode*>(_node1);
    FCLMESHCollisionNode* collisionNode2 = static_cast<FCLMESHCollisionNode*>(_node2);
    return collisionNode1->checkCollision(collisionNode2,
                                          _calculateContactPoints ? &mContacts : NULL,
                                          100);
}

void FCLMESHCollisionDetector::draw() {
    for(int i=0;i<mCollisionNodes.size();i++)
        static_cast<FCLMESHCollisionNode*>(mCollisionNodes[i])->drawCollisionSkeletonNode();
}

}
